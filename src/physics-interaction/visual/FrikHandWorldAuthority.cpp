#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string_view>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/visual/DampenedDriverPredictionPolicy.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/NetImmerse/NiNode.h"

namespace rock::frik_hand_world_authority
{
    namespace
    {
        namespace registry_policy = hand_world_claim_registry_policy;
        namespace isolation_policy = tracked_hand_isolation_policy;

        using registry_policy::DriverFrame;
        using registry_policy::DriverSample;
        using registry_policy::handIndex;

        // The weapon offset node hangs a few nodes below the wand; FRIK edits
        // those locals, the game moves the wand. Deeper chains are not player
        // wand chains and fall back to the node's own world.
        constexpr std::size_t kMaxOffsetChainDepth = 6;
        constexpr std::uint32_t kProbeSummaryFrames = 600;

        namespace transport_policy = rendered_bone_transport_policy;

        struct IsolationState
        {
            isolation_policy::RelationState relation{};
            isolation_policy::FrameResult result{};
            RE::NiTransform presentedHandWorld{};
            bool presentedHandValid = false;
            // The wrist FRIK solved this frame (refNode, before the palm blend).
            RE::NiTransform renderedHandNodeWorld{};
            bool renderedHandNodeValid = false;
            transport_policy::HandTransport chainTransport{};
            // This frame's presentation plan, for the trace.
            float presentTranslationGameUnits = 0.0f;
            float presentRotationDegrees = 0.0f;
            bool presentedThisFrame = false;
            std::uint32_t presentTraceLines = 0;
        };

        namespace prediction_policy = dampened_driver_prediction_policy;

        /*
         * FRIK dampens the weapon offset node at its frame start
         * (Skeleton::dampenHand); the seats ROCK publishes are read from
         * that dampened node after FRIK's frame. A claim rebased by the raw
         * chain hands FRIK a target one raw step ahead of the seat, and FRIK
         * solves the whole arm for it. The pass predicts the dampened node
         * with FRIK's own filter instead (factors through its config API,
         * camera step compensated) from the node ROCK read last frame, so
         * the claim FRIK consumes equals the seat up to prediction error.
         * After FRIK's frame the actual node replaces the prediction in the
         * driver frame and every claim is re-anchored to it.
         */
        struct DampenState
        {
            prediction_policy::FrikDampenConfig config{};
            // The dampened node as read after FRIK's last frame; frozen while
            // FRIK does not dampen, as FRIK's own previous frame is.
            std::array<DriverSample, 2> previousDampened{};
            // Diagnostic witnesses follow the exact history write below.
            std::array<std::uint64_t, 2> historySequence{};
            std::array<RE::NiPoint3, 2> historyCamera{};
            std::array<bool, 2> historyCameraValid{};
            std::uint64_t runtimeFrameUsed = 0;
            bool menuUsed = false;
            RE::NiPoint3 cameraNow{};
            RE::NiPoint3 cameraPrev{};
            bool cameraNowValid = false;
            bool cameraPrevValid = false;
            // The factors this pass predicted with (off: the raw chain was published).
            prediction_policy::DampenFactors factorsThisPass{};
            std::array<bool, 2> predictedThisPass{};
            std::uint32_t configRefreshFrames = 0;
            bool configQueried = false;
        };

        struct PredictionError
        {
            float translationGameUnits = 0.0f;
            float rotationDegrees = 0.0f;
            bool valid = false;
        };

        struct ProbeCounters
        {
            std::uint32_t frames = 0;
            std::array<std::uint32_t, 2> predictedFrames{};
            std::array<float, 2> predictionTranslationSum{};
            std::array<float, 2> predictionRotationSum{};
            std::array<float, 2> predictionTranslationMax{};
            std::array<float, 2> predictionRotationMax{};
            std::array<std::uint32_t, 2> predictionTraceLines{};
            std::array<std::uint32_t, 2> reconstructedFrames{};
            std::array<std::uint32_t, 2> contaminatedFrames{};
            std::array<std::uint32_t, 2> unavailableFrames{};
            std::array<std::uint32_t, 2> probeFrames{};
            std::array<float, 2> probeTranslationMax{};
            std::array<float, 2> probeRotationMax{};
            std::array<std::uint32_t, 2> transportActiveFrames{};
            std::array<float, 2> transportTranslationMax{};
            std::array<float, 2> transportRotationMax{};
            // A claim was rendered but the chain could not be carried: every
            // controller-space consumer read ROCK's previous target that frame.
            std::array<std::uint32_t, 2> claimedFramesWithoutTransport{};
            std::array<std::uint32_t, 2> presentedFrames{};
            std::array<float, 2> presentTranslationMax{};
            std::array<float, 2> presentRotationMax{};
            std::array<float, 2> presentElbowMax{};
            // The re-solved wrist lay beyond the straight arm by this much.
            std::array<float, 2> presentStretchMax{};
            std::array<std::uint32_t, 2> presentSkippedNotFollowing{};
            std::array<std::uint32_t, 2> presentSkippedTooLarge{};
            std::array<std::uint32_t, 2> presentWriteFailures{};
            std::uint32_t rebasePublishes = 0;
            std::uint32_t rebaseKeepOrderPublishes = 0;
            std::uint32_t rebaseRejected = 0;
            std::uint32_t driverSamplesMissing = 0;
            std::uint32_t claimsRefusedByGate = 0;
            std::uint32_t claimsRefusedRotation = 0;
        };

        struct Service
        {
            registry_policy::Registry registry{};
            DriverFrame driverFrame{};
            // The pass before: the driver's motion over the frame, for the trace.
            DriverFrame previousDriverFrame{};
            // The winner per hand at the end of the previous ROCK frame, for the trace.
            std::array<registry_policy::ConsumedTarget, 2> lastFrameTargets{};
            std::uint64_t rockFrameSequence = 0;
            std::uint64_t rockFrameIndex = 0;
            std::uint64_t observedFrameIndex = 0;
            // Latched by the first resolve of a frame; later resolves in the
            // same frame see no new kick and must not calibrate on it.
            bool recoilKickThisFrame = false;
            std::array<bool, 2> claimConsumedThisFrame{};
            // The winner per hand when FRIK's frame began: what it solved to.
            std::array<registry_policy::ConsumedTarget, 2> consumedTargets{};
            // Skeleton ready and no scope/config state that suspends the solve.
            bool presentationAllowed = false;
            std::array<IsolationState, 2> isolation{};
            SchedulerState scheduler = SchedulerState::Unverified;
            ProbeCounters probes{};
            DampenState dampen{};
            ScopeDampenTrace scopePreTrace{};
            // This frame's prediction error per hand, for the trace.
            std::array<PredictionError, 2> predictionErrors{};
            // The raw chain samples of this pass, what the prediction started from.
            DriverFrame rawFrame{};
        };

        Service g_service{};

        [[nodiscard]] bool debugEnabled()
        {
            return g_rockConfig.rockDebugHandWorldAuthority;
        }

        [[nodiscard]] const char* handName(const bool isLeft)
        {
            return isLeft ? "left" : "right";
        }

        [[nodiscard]] const char* driverName(const RebaseDriver driver)
        {
            switch (driver) {
            case RebaseDriver::RightHand:
                return "right-hand";
            case RebaseDriver::LeftHand:
                return "left-hand";
            case RebaseDriver::RightHandPosition:
                return "right-hand-position";
            case RebaseDriver::LeftHandPosition:
                return "left-hand-position";
            case RebaseDriver::RightHandAimAxis:
                return "right-hand-aim-axis";
            case RebaseDriver::LeftHandAimAxis:
                return "left-hand-aim-axis";
            default:
                return "static";
            }
        }

        /*
         * The controller-driven frame a claim follows between ROCK frames:
         * the physical hand's weapon offset node recomposed from the wand's
         * fresh world and the chain's locals, so FRIK's own edits to those
         * locals in its previous frame are carried, not its stale worlds.
         */
        [[nodiscard]] DriverSample sampleOffsetChain(const bool isLeft)
        {
            DriverSample sample{};
            const auto* nodes = f4vr::getPlayerNodes();
            if (!nodes) {
                return sample;
            }
            RE::NiNode* const wand = isLeft ? nodes->SecondaryWandNode : nodes->primaryWandNode;
            RE::NiNode* const offset = isLeft ? nodes->SecondaryMeleeWeaponOffsetNode2 : nodes->primaryWeaponOffsetNOde;
            if (!wand || !offset) {
                return sample;
            }

            std::array<const RE::NiAVObject*, kMaxOffsetChainDepth> chain{};
            std::size_t depth = 0;
            const RE::NiAVObject* node = offset;
            while (node && node != wand && depth < kMaxOffsetChainDepth) {
                chain[depth++] = node;
                node = node->parent;
            }

            RE::NiTransform world{};
            if (node == wand) {
                world = wand->world;
                for (std::size_t i = depth; i > 0; --i) {
                    world = transform_math::composeTransforms(world, chain[i - 1]->local);
                }
            } else {
                world = offset->world;
            }
            if (!registry_policy::isFiniteTransform(world)) {
                return sample;
            }
            sample.world = world;
            sample.valid = true;
            return sample;
        }

        [[nodiscard]] DriverFrame sampleDriverFrame(const std::uint64_t sequence)
        {
            DriverFrame frame{};
            frame.sequence = sequence;
            frame.hands[handIndex(false)] = sampleOffsetChain(false);
            frame.hands[handIndex(true)] = sampleOffsetChain(true);
            return frame;
        }

        // The weapon offset node's world as the scene holds it right now.
        [[nodiscard]] bool readOffsetNodeWorld(const bool isLeft, RE::NiTransform& outWorld)
        {
            const auto* nodes = f4vr::getPlayerNodes();
            RE::NiNode* const offset = nodes ? (isLeft ? nodes->SecondaryMeleeWeaponOffsetNode2 : nodes->primaryWeaponOffsetNOde) : nullptr;
            if (!offset || !registry_policy::isFiniteTransform(offset->world)) {
                return false;
            }
            outWorld = offset->world;
            return true;
        }

        [[nodiscard]] bool readFrikDampenFactor(const char* key, float& outFactor)
        {
            const auto* frikApi = frik_visual_authority::api();
            std::array<char, 16> text{};
            (void)frikApi->getConfigValue("Fallout4VRBody", key, text.data(), static_cast<int>(text.size()), "?");
            char* end = nullptr;
            const float factor = std::strtof(text.data(), &end);
            if (end == text.data() || !prediction_policy::isUsableFactor(factor)) {
                return false;
            }
            outFactor = factor;
            return true;
        }

        [[nodiscard]] bool readFrikDampenFlag(const char* key)
        {
            const auto* frikApi = frik_visual_authority::api();
            std::array<char, 16> text{};
            (void)frikApi->getConfigValue("Fallout4VRBody", key, text.data(), static_cast<int>(text.size()), "?");
            return text[0] == 't' || text[0] == 'T' || text[0] == '1';
        }

        /*
         * FRIK's DampenHands configuration through its API, refreshed every
         * probe window so an edit in FRIK's own settings is followed. Any
         * unusable value turns the prediction off (the raw chain is published,
         * as before) rather than predicting with a wrong filter.
         */
        void refreshDampenConfig()
        {
            auto& dampen = g_service.dampen;
            if (dampen.configQueried && ++dampen.configRefreshFrames < kProbeSummaryFrames) {
                return;
            }
            dampen.configRefreshFrames = 0;
            const bool first = !dampen.configQueried;
            dampen.configQueried = true;
            prediction_policy::FrikDampenConfig config{};
            const auto* frikApi = frik_visual_authority::api();
            if (frikApi && frikApi->getConfigValue) {
                const bool enabled = readFrikDampenFlag("DampenHands");
                const bool inScope = readFrikDampenFlag("DampenHandsInVanillaScope");
                config.valid =
                    readFrikDampenFactor("DampenHandsTranslation", config.normal.translation) &&
                    readFrikDampenFactor("DampenHandsRotation", config.normal.rotation) &&
                    readFrikDampenFactor("DampenHandsTranslationInVanillaScope", config.vanillaScope.translation) &&
                    readFrikDampenFactor("DampenHandsRotationInVanillaScope", config.vanillaScope.rotation);
                config.normal.enabled = config.valid && enabled;
                config.vanillaScope.enabled = config.valid && enabled && inScope;
            }
            const bool changed =
                first ||
                config.valid != dampen.config.valid ||
                config.normal.enabled != dampen.config.normal.enabled ||
                config.normal.translation != dampen.config.normal.translation ||
                config.normal.rotation != dampen.config.normal.rotation ||
                config.vanillaScope.enabled != dampen.config.vanillaScope.enabled;
            dampen.config = config;
            if (changed) {
                ROCK_LOG_INFO(Hand,
                    "HandWorldAuthority driver prediction: FRIK dampening {} translation={:.2f} rotation={:.2f} inVanillaScope={} ({:.2f}/{:.2f})",
                    config.normal.enabled ? "on" : "off",
                    config.normal.translation,
                    config.normal.rotation,
                    config.vanillaScope.enabled ? "on" : "off",
                    config.vanillaScope.translation,
                    config.vanillaScope.rotation);
            }
        }

        /*
         * Pre-FRIK: the driver frame the pass rebases by. Each hand's raw
         * chain sample is replaced by the dampened node FRIK will write, when
         * the dampened node of the previous frame is known.
         */
        [[nodiscard]] DriverFrame predictDriverFrame(const DriverFrame& rawFrame)
        {
            auto& dampen = g_service.dampen;
            refreshDampenConfig();
            dampen.cameraPrev = dampen.cameraNow;
            dampen.cameraPrevValid = dampen.cameraNowValid;
            dampen.cameraNow = f4vr::getCameraPosition();
            dampen.cameraNowValid = std::isfinite(dampen.cameraNow.x) && std::isfinite(dampen.cameraNow.y) && std::isfinite(dampen.cameraNow.z);
            dampen.factorsThisPass = prediction_policy::selectFactors(dampen.config, runtime_state::currentFrame().localScopeMenuOpen);
            dampen.runtimeFrameUsed = runtime_state::currentFrame().frameIndex;
            dampen.menuUsed = runtime_state::currentFrame().localScopeMenuOpen;
            dampen.predictedThisPass = {};

            DriverFrame frame = rawFrame;
            if (!dampen.factorsThisPass.enabled || !dampen.cameraNowValid || !dampen.cameraPrevValid) {
                return frame;
            }
            const RE::NiPoint3 cameraDelta{
                dampen.cameraNow.x - dampen.cameraPrev.x,
                dampen.cameraNow.y - dampen.cameraPrev.y,
                dampen.cameraNow.z - dampen.cameraPrev.z,
            };
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const DriverSample& raw = rawFrame.hands[hand];
                const DriverSample& previous = dampen.previousDampened[hand];
                if (!raw.valid || !previous.valid) {
                    continue;
                }
                const RE::NiTransform predicted = prediction_policy::predictDampened(raw.world, previous.world, cameraDelta, dampen.factorsThisPass);
                if (!registry_policy::isFiniteTransform(predicted)) {
                    continue;
                }
                frame.hands[hand].world = predicted;
                dampen.predictedThisPass[hand] = true;
            }
            return frame;
        }

        /*
         * Post-FRIK: the dampened node FRIK actually wrote replaces the
         * prediction in the driver frame, becomes the next prediction's
         * previous value while FRIK dampens, and scores the prediction.
         */
        void observeActualDrivers()
        {
            auto& dampen = g_service.dampen;
            auto& probes = g_service.probes;
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const bool isLeft = hand == handIndex(true);
                g_service.predictionErrors[hand] = {};
                RE::NiTransform actual{};
                if (!readOffsetNodeWorld(isLeft, actual)) {
                    continue;
                }
                DriverSample& sample = g_service.driverFrame.hands[hand];
                if (dampen.predictedThisPass[hand] && sample.valid) {
                    PredictionError& error = g_service.predictionErrors[hand];
                    error.translationGameUnits = registry_policy::translationDeltaGameUnits(actual, sample.world);
                    error.rotationDegrees = registry_policy::rotationDeltaDegrees(actual, sample.world);
                    error.valid = std::isfinite(error.translationGameUnits) && std::isfinite(error.rotationDegrees);
                    if (error.valid && debugEnabled()) {
                        ++probes.predictedFrames[hand];
                        probes.predictionTranslationSum[hand] += error.translationGameUnits;
                        probes.predictionRotationSum[hand] += error.rotationDegrees;
                        probes.predictionTranslationMax[hand] = (std::max)(probes.predictionTranslationMax[hand], error.translationGameUnits);
                        probes.predictionRotationMax[hand] = (std::max)(probes.predictionRotationMax[hand], error.rotationDegrees);
                        constexpr std::uint32_t kDenseTraceLines = 120;
                        const std::uint32_t line = probes.predictionTraceLines[hand]++;
                        const DriverSample& previous = dampen.previousDampened[hand];
                        if (line < kDenseTraceLines || (line - kDenseTraceLines) % 30 == 0) {
                            ROCK_LOG_DEBUG(Hand,
                                "DAMPEN hand={} line={} predErr={:.3f}gu/{:.3f}deg predStep={:.2f}gu actualStep={:.2f}gu rawStep={:.2f}gu f={:.2f}/{:.2f}",
                                isLeft ? "L" : "R",
                                line,
                                error.translationGameUnits,
                                error.rotationDegrees,
                                previous.valid ? registry_policy::translationDeltaGameUnits(sample.world, previous.world) : 0.0f,
                                previous.valid ? registry_policy::translationDeltaGameUnits(actual, previous.world) : 0.0f,
                                previous.valid ? registry_policy::translationDeltaGameUnits(g_service.rawFrame.hands[hand].world, previous.world) : 0.0f,
                                dampen.factorsThisPass.translation,
                                dampen.factorsThisPass.rotation);
                        }
                    }
                }
                sample.world = actual;
                sample.valid = true;
                // FRIK keeps its previous frame while it does not dampen.
                if (dampen.factorsThisPass.enabled || !dampen.previousDampened[hand].valid) {
                    dampen.previousDampened[hand] = DriverSample{ .world = actual, .valid = true };
                    dampen.historySequence[hand] = g_service.driverFrame.sequence;
                    dampen.historyCamera[hand] = dampen.cameraNow;
                    dampen.historyCameraValid[hand] = dampen.cameraNowValid;
                }
            }
        }

        [[nodiscard]] bool frikSetHandWorld(const registry_policy::Claim& claim, const RE::NiTransform& target)
        {
            const auto* frikApi = frik_visual_authority::api();
            return frikApi && frikApi->setHandWorldTransform &&
                   frikApi->setHandWorldTransform(claim.tag.data(), frik_visual_authority::handFromBool(claim.isLeft), target, claim.priority);
        }

        [[nodiscard]] bool frikClearHandWorld(const char* tag, const bool isLeft)
        {
            const auto* frikApi = frik_visual_authority::api();
            return frikApi && frikApi->clearHandWorldTransform &&
                   frikApi->clearHandWorldTransform(tag, frik_visual_authority::handFromBool(isLeft));
        }

        void emitProbeSummary()
        {
            auto& probes = g_service.probes;
            if (!debugEnabled()) {
                probes = {};
                return;
            }
            if (++probes.frames < kProbeSummaryFrames) {
                return;
            }
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const auto& relation = g_service.isolation[hand].relation;
                ROCK_LOG_INFO(Hand,
                    "HandWorldAuthority probe hand={} frames={} reconstructed={} contaminated={} unavailable={} probeFrames={} probeMaxTranslation={:.3f}gu probeMaxRotation={:.3f}deg relation={} relationAccepted={} relationRejected={} transportFrames={} transportMax={:.2f}gu/{:.2f}deg claimedWithoutTransport={} presented={} presentMax={:.2f}gu/{:.2f}deg presentElbowMax={:.2f}gu presentStretchMax={:.2f}gu presentNotFollowing={} presentTooLarge={} presentWriteFailed={}",
                    handName(hand == handIndex(true)),
                    probes.frames,
                    probes.reconstructedFrames[hand],
                    probes.contaminatedFrames[hand],
                    probes.unavailableFrames[hand],
                    probes.probeFrames[hand],
                    probes.probeTranslationMax[hand],
                    probes.probeRotationMax[hand],
                    relation.valid ? "valid" : "missing",
                    relation.acceptedSamples,
                    relation.rejectedSamples,
                    probes.transportActiveFrames[hand],
                    probes.transportTranslationMax[hand],
                    probes.transportRotationMax[hand],
                    probes.claimedFramesWithoutTransport[hand],
                    probes.presentedFrames[hand],
                    probes.presentTranslationMax[hand],
                    probes.presentRotationMax[hand],
                    probes.presentElbowMax[hand],
                    probes.presentStretchMax[hand],
                    probes.presentSkippedNotFollowing[hand],
                    probes.presentSkippedTooLarge[hand],
                    probes.presentWriteFailures[hand]);
            }
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const std::uint32_t frames = probes.predictedFrames[hand];
                ROCK_LOG_INFO(Hand,
                    "HandWorldAuthority driver prediction hand={} frames={} errorMean={:.3f}gu/{:.3f}deg errorMax={:.2f}gu/{:.2f}deg",
                    handName(hand == handIndex(true)),
                    frames,
                    frames ? probes.predictionTranslationSum[hand] / static_cast<float>(frames) : 0.0f,
                    frames ? probes.predictionRotationSum[hand] / static_cast<float>(frames) : 0.0f,
                    probes.predictionTranslationMax[hand],
                    probes.predictionRotationMax[hand]);
            }
            ROCK_LOG_INFO(Hand,
                "HandWorldAuthority rebase frames={} claims={} rebasePublishes={} keepOrderPublishes={} rejected={} driverSamplesMissing={} refusedByGate={} refusedRotation={} scheduler={}",
                probes.frames,
                registry_policy::claimCount(g_service.registry),
                probes.rebasePublishes,
                probes.rebaseKeepOrderPublishes,
                probes.rebaseRejected,
                probes.driverSamplesMissing,
                probes.claimsRefusedByGate,
                probes.claimsRefusedRotation,
                g_service.scheduler == SchedulerState::Verified ? "verified" : g_service.scheduler == SchedulerState::Refused ? "refused" : "unverified");
            probes = {};
        }

        void observeFallbacks(const FrameHandSamples& samples, const bool recoilKickThisFrame)
        {
            if (!samples.fallbackObservationAllowed || recoilKickThisFrame) {
                return;
            }
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const bool isLeft = hand == handIndex(true);
                const RawHandSample& sample = isLeft ? samples.left : samples.right;
                for (auto& claim : g_service.registry.claims) {
                    if (!claim.valid || claim.isLeft != isLeft) {
                        continue;
                    }
                    const registry_policy::Claim* top = registry_policy::winner(g_service.registry, isLeft);
                    if (top != &claim) {
                        continue;
                    }
                    const auto observation = registry_policy::observeFallback(
                        claim,
                        sample.bodyHandNodeWorld,
                        sample.bodyHandNodeValid,
                        false);
                    if (observation == registry_policy::FallbackObservation::Confirmed) {
                        ROCK_LOG_SAMPLE_WARN(Hand,
                            2000,
                            "FRIK solved the tracked {} hand instead of ROCK claim '{}' (priority {}): rendered wrist is {:.1f}gu/{:.1f}deg from the target. The claim is unreachable for this arm; the owner's publishes report failure until FRIK follows the claim again.",
                            handName(isLeft),
                            claim.tag.data(),
                            claim.priority,
                            registry_policy::translationDeltaGameUnits(sample.bodyHandNodeWorld, claim.target),
                            registry_policy::rotationDeltaDegrees(sample.bodyHandNodeWorld, claim.target));
                    }
                }
            }
        }
    }

    void setSchedulerState(const SchedulerState state)
    {
        g_service.scheduler = state;
    }

    SchedulerState schedulerState()
    {
        return g_service.scheduler;
    }

    void runPreFrikPass(const std::uint64_t sequence)
    {
        g_service.previousDriverFrame = g_service.driverFrame;
        g_service.rawFrame = sampleDriverFrame(sequence);
        g_service.driverFrame = predictDriverFrame(g_service.rawFrame);
        if (debugEnabled()) g_service.scopePreTrace = scopeDampenTrace();
        if (registry_policy::claimCount(g_service.registry) == 0) {
            return;
        }

        registry_policy::RebasePassPlan plan{};
        registry_policy::planRebasePass(g_service.registry, g_service.driverFrame, plan);
        for (std::size_t i = 0; i < plan.count; ++i) {
            const auto& entry = plan.entries[i];
            const auto& claim = g_service.registry.claims[entry.claimIndex];
            if (claim.driver != RebaseDriver::Static) {
                const DriverSample* sample = registry_policy::sampleForDriver(g_service.driverFrame, claim.driver);
                if (!sample || !sample->valid) {
                    ++g_service.probes.driverSamplesMissing;
                }
            }
            if (!entry.moved && !entry.keepOrder) {
                continue;
            }
            if (frikSetHandWorld(claim, entry.target)) {
                registry_policy::commitRebasePassEntry(g_service.registry, entry, g_service.driverFrame);
                if (entry.moved) {
                    ++g_service.probes.rebasePublishes;
                } else {
                    ++g_service.probes.rebaseKeepOrderPublishes;
                }
            } else {
                ++g_service.probes.rebaseRejected;
                ROCK_LOG_SAMPLE_WARN(Hand,
                    2000,
                    "HandWorldAuthority rebase republish rejected by FRIK: tag='{}' hand={} priority={} driver={}",
                    claim.tag.data(),
                    handName(claim.isLeft),
                    claim.priority,
                    driverName(claim.driver));
            }
        }
    }

    void beginRockFrame(const std::uint64_t sequence)
    {
        g_service.rockFrameSequence = sequence;
        ++g_service.rockFrameIndex;
        observeActualDrivers();
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            g_service.claimConsumedThisFrame[hand] = registry_policy::hasClaim(g_service.registry, isLeft);
            g_service.consumedTargets[hand] = registry_policy::snapshotConsumedTarget(g_service.registry, isLeft);
            g_service.isolation[hand].result = {};
            g_service.isolation[hand].presentedHandValid = false;
            g_service.isolation[hand].renderedHandNodeValid = false;
            g_service.isolation[hand].chainTransport = {};
            g_service.isolation[hand].presentedThisFrame = false;
            g_service.isolation[hand].presentTranslationGameUnits = 0.0f;
            g_service.isolation[hand].presentRotationDegrees = 0.0f;
        }
        // The consumed targets above are what FRIK solved to (the predicted
        // rebase); the registry now follows the node FRIK actually wrote.
        registry_policy::reanchorClaims(g_service.registry, g_service.driverFrame);
        g_service.presentationAllowed = false;
    }

    bool publish(const char* tag, const bool isLeft, const RE::NiTransform& requestedTarget, const int priority, const RebaseDriver driver)
    {
        if (!tag) {
            return false;
        }
        /*
         * FRIK copies the target rotation into the arm, so a basis that is
         * not a rotation would stretch the rendered hand and come back as
         * this frame's bone input. Small drift is renormalized; anything
         * larger is a broken owner and fails closed.
         */
        if (!registry_policy::isUsableTargetRotation(requestedTarget)) {
            ++g_service.probes.claimsRefusedRotation;
            ROCK_LOG_SAMPLE_WARN(Hand,
                2000,
                "HandWorldAuthority refused claim '{}' hand={}: target rotation is not a rotation (orthonormality error {:.4f}); the owner runs its failure path",
                tag,
                handName(isLeft),
                transform_math::storedRotationOrthonormalityError(requestedTarget.rotate));
            return false;
        }
        const RE::NiTransform worldTarget = transform_math::orthonormalizedTransform(requestedTarget);
        if (g_service.scheduler != SchedulerState::Verified) {
            ++g_service.probes.claimsRefusedByGate;
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "HandWorldAuthority refused claim '{}' hand={}: the pre-FRIK scheduler is {}. Hand world claims stay disabled until it is verified.",
                tag,
                handName(isLeft),
                g_service.scheduler == SchedulerState::Refused ? "refused" : "not verified yet");
            return false;
        }

        const std::string_view tagView(tag);
        const auto* frikApi = frik_visual_authority::api();
        if (!frikApi || !frikApi->setHandWorldTransform ||
            !frikApi->setHandWorldTransform(tag, frik_visual_authority::handFromBool(isLeft), worldTarget, priority)) {
            return false;
        }

        // The driver sample of this frame's pre-FRIK pass is the base the
        // target was computed against. An older sample would mis-rebase.
        DriverSample driverAtPublish{};
        DriverSample otherDriverAtPublish{};
        if (driver != RebaseDriver::Static && g_service.driverFrame.sequence != 0 &&
            g_service.driverFrame.sequence == g_service.rockFrameSequence) {
            if (const DriverSample* sample = registry_policy::sampleForDriver(g_service.driverFrame, driver)) {
                driverAtPublish = *sample;
            }
            if (const DriverSample* other = registry_policy::otherHandSampleForDriver(g_service.driverFrame, driver)) {
                otherDriverAtPublish = *other;
            }
        }

        const auto result = registry_policy::commit(
            g_service.registry, tagView, isLeft, priority, worldTarget, driver, driverAtPublish, otherDriverAtPublish);
        switch (result) {
        case registry_policy::CommitResult::Inserted:
            return true;
        case registry_policy::CommitResult::Updated: {
            /*
             * FRIK holds the new target, but while it demonstrably solves the
             * tracked hand instead (unreachable target) the owner is told the
             * claim is not held, as Experimental did per frame. Publishing
             * keeps FRIK current so a reachable target ends the episode.
             */
            const registry_policy::Claim* claim = registry_policy::find(g_service.registry, tagView, isLeft);
            return claim && !claim->fallbackReported;
        }
        case registry_policy::CommitResult::Full:
            // FRIK holds the claim but ROCK cannot follow it: undo so the hand
            // is never shown one frame stale.
            (void)frikClearHandWorld(tag, isLeft);
            ROCK_LOG_ERROR(Hand,
                "HandWorldAuthority registry is full ({} claims); claim '{}' hand={} was withdrawn",
                registry_policy::kMaxClaims,
                tag,
                handName(isLeft));
            return false;
        default:
            (void)frikClearHandWorld(tag, isLeft);
            return false;
        }
    }

    bool clear(const char* tag, const bool isLeft)
    {
        if (!tag) {
            return false;
        }
        const bool removed = registry_policy::remove(g_service.registry, std::string_view(tag), isLeft);
        const bool cleared = frikClearHandWorld(tag, isLeft);
        return removed || cleared;
    }

    bool hasActiveClaim(const bool isLeft)
    {
        return registry_policy::hasClaim(g_service.registry, isLeft);
    }

    bool wasClaimConsumedThisFrame(const bool isLeft)
    {
        return g_service.claimConsumedThisFrame[handIndex(isLeft)];
    }

    bool tryGetPublishedHandWorld(const bool isLeft, RE::NiTransform& outWorld, const char* excludedTag)
    {
        outWorld = {};
        const registry_policy::Claim* best = nullptr;
        const std::string_view excluded = excludedTag ? std::string_view(excludedTag) : std::string_view{};
        for (const auto& claim : g_service.registry.claims) {
            if (!claim.valid || claim.isLeft != isLeft) {
                continue;
            }
            if (!excluded.empty() && registry_policy::tagView(claim) == excluded) {
                continue;
            }
            if (!best || claim.priority > best->priority ||
                (claim.priority == best->priority && claim.publishOrder > best->publishOrder)) {
                best = &claim;
            }
        }
        if (!best) {
            return false;
        }
        outWorld = best->target;
        return true;
    }

    void resolveRawHands(const FrameHandSamples& samples)
    {
        // Called once from the inner hook (before the scope sync and provider
        // callbacks) and again from the physics update; only the first call of
        // a frame observes fallbacks and advances the probe counters.
        const bool firstResolveThisFrame = g_service.observedFrameIndex != g_service.rockFrameIndex;
        g_service.observedFrameIndex = g_service.rockFrameIndex;
        if (firstResolveThisFrame) {
            g_service.recoilKickThisFrame = samples.recoilKickThisFrame;
            g_service.presentationAllowed = samples.fallbackObservationAllowed;
        }
        const bool recoilKickThisFrame = g_service.recoilKickThisFrame || samples.recoilKickThisFrame;

        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            const RawHandSample& sample = isLeft ? samples.left : samples.right;
            auto& state = g_service.isolation[hand];

            state.presentedHandWorld = sample.flattenedHandWorld;
            state.presentedHandValid = sample.flattenedHandValid;
            if (firstResolveThisFrame) {
                state.renderedHandNodeWorld = sample.bodyHandNodeWorld;
                state.renderedHandNodeValid = sample.bodyHandNodeValid;
            }

            isolation_policy::FrameInput input{};
            input.firstPersonHandValid = frik_visual_authority::tryGetHandWorldTransform(
                frik_visual_authority::handFromBool(isLeft),
                input.firstPersonHandWorld);
            input.bodyHandNodeWorld = sample.bodyHandNodeWorld;
            input.bodyHandNodeValid = sample.bodyHandNodeValid;
            input.flattenedHandWorld = sample.flattenedHandWorld;
            input.flattenedHandValid = sample.flattenedHandValid;
            input.claimConsumed = g_service.claimConsumedThisFrame[hand];
            input.calibrationAllowed = !recoilKickThisFrame;
            state.result = isolation_policy::resolveFrame(state.relation, input);
            state.chainTransport = transport_policy::makeHandTransport(
                state.result.rawHandWorld,
                state.result.valid,
                sample.flattenedHandWorld,
                sample.flattenedHandValid);

            if (!firstResolveThisFrame) {
                continue;
            }
            auto& probes = g_service.probes;
            if (state.chainTransport.active) {
                // Measured at the hand root. The delta's own translation is
                // taken about the world origin and grows with the world
                // coordinate (140000 gu was logged for a 138 deg carry).
                ++probes.transportActiveFrames[hand];
                probes.transportTranslationMax[hand] = (std::max)(probes.transportTranslationMax[hand],
                    isolation_policy::translationGameUnits(state.result.rawHandWorld, sample.flattenedHandWorld));
                probes.transportRotationMax[hand] = (std::max)(probes.transportRotationMax[hand],
                    isolation_policy::rotationDegrees(state.result.rawHandWorld, sample.flattenedHandWorld));
            } else if (input.claimConsumed) {
                ++probes.claimedFramesWithoutTransport[hand];
            }
            switch (state.result.source) {
            case isolation_policy::RawHandSource::Reconstructed:
                ++probes.reconstructedFrames[hand];
                break;
            case isolation_policy::RawHandSource::FlattenedContaminated:
                ++probes.contaminatedFrames[hand];
                ROCK_LOG_SAMPLE_WARN(Hand,
                    5000,
                    "HandWorldAuthority {} hand: a claim was rendered but the controller hand could not be reconstructed (first-person hand {}, body node {}, relation {}); using the rendered bone",
                    handName(isLeft),
                    input.firstPersonHandValid ? "ok" : "missing",
                    input.bodyHandNodeValid ? "ok" : "missing",
                    state.relation.valid ? "ok" : "uncalibrated");
                break;
            case isolation_policy::RawHandSource::Unavailable:
                ++probes.unavailableFrames[hand];
                break;
            default:
                break;
            }
            if (state.result.probeValid) {
                ++probes.probeFrames[hand];
                probes.probeTranslationMax[hand] = (std::max)(probes.probeTranslationMax[hand], state.result.probeTranslationGameUnits);
                probes.probeRotationMax[hand] = (std::max)(probes.probeRotationMax[hand], state.result.probeRotationDegrees);
            }
        }

        if (firstResolveThisFrame) {
            observeFallbacks(samples, recoilKickThisFrame);
            emitProbeSummary();
        }
    }

    bool tryGetRawHandWorld(const bool isLeft, RE::NiTransform& outWorld)
    {
        const auto& state = g_service.isolation[handIndex(isLeft)];
        if (!state.result.valid) {
            outWorld = {};
            return false;
        }
        outWorld = state.result.rawHandWorld;
        return true;
    }

    bool hasCalibratedRawHandFrame(const bool isLeft)
    {
        const auto& state = g_service.isolation[handIndex(isLeft)];
        return isolation_policy::canDriveExternalPose(state.relation, state.result);
    }

    bool tryGetPresentedHandWorld(const bool isLeft, RE::NiTransform& outWorld)
    {
        const auto& state = g_service.isolation[handIndex(isLeft)];
        if (!state.presentedHandValid) {
            outWorld = {};
            return false;
        }
        outWorld = state.presentedHandWorld;
        return true;
    }

    const char* rawHandSourceName(const bool isLeft)
    {
        switch (g_service.isolation[handIndex(isLeft)].result.source) {
        case isolation_policy::RawHandSource::Flattened:
            return "flattened";
        case isolation_policy::RawHandSource::Reconstructed:
            return "reconstructed";
        case isolation_policy::RawHandSource::FlattenedContaminated:
            return "contaminated";
        default:
            return "none";
        }
    }

    bool tryGetHandChainTransport(const bool isLeft, HandChainTransport& outTransport)
    {
        outTransport = g_service.isolation[handIndex(isLeft)].chainTransport;
        return outTransport.active;
    }

    RE::NiTransform transportHandChainWorld(const bool isLeft, const RE::NiTransform& renderedWorld)
    {
        return transport_policy::transportWorld(g_service.isolation[handIndex(isLeft)].chainTransport, renderedWorld);
    }

    bool tryPlanHandPresentation(const bool isLeft, RE::NiTransform& outDelta)
    {
        outDelta = transform_math::makeIdentityTransform<RE::NiTransform>();
        const std::size_t hand = handIndex(isLeft);
        if (!g_service.presentationAllowed || !g_service.claimConsumedThisFrame[hand]) {
            return false;
        }
        auto& state = g_service.isolation[hand];
        const auto plan = registry_policy::planPresentation(
            g_service.consumedTargets[hand],
            registry_policy::winner(g_service.registry, isLeft),
            state.renderedHandNodeWorld,
            state.renderedHandNodeValid);
        auto& probes = g_service.probes;
        switch (plan.decision) {
        case registry_policy::PresentationDecision::Present:
            outDelta = plan.delta;
            state.presentTranslationGameUnits = plan.translationGameUnits;
            state.presentRotationDegrees = plan.rotationDegrees;
            probes.presentTranslationMax[hand] = (std::max)(probes.presentTranslationMax[hand], plan.translationGameUnits);
            probes.presentRotationMax[hand] = (std::max)(probes.presentRotationMax[hand], plan.rotationDegrees);
            return true;
        case registry_policy::PresentationDecision::NotFollowing:
            ++probes.presentSkippedNotFollowing[hand];
            return false;
        case registry_policy::PresentationDecision::TooLarge:
            ++probes.presentSkippedTooLarge[hand];
            return false;
        default:
            return false;
        }
    }

    void recordHandPresentation(
        const bool isLeft,
        const RE::NiTransform& delta,
        const bool applied,
        const float elbowMoveGameUnits,
        const float reachDeficitGameUnits)
    {
        const std::size_t hand = handIndex(isLeft);
        auto& state = g_service.isolation[hand];
        if (!applied) {
            ++g_service.probes.presentWriteFailures[hand];
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "HandWorldAuthority could not re-solve the rendered {} arm onto this frame's claim; the hand shows last frame's target",
                handName(isLeft));
            return;
        }
        ++g_service.probes.presentedFrames[hand];
        g_service.probes.presentElbowMax[hand] = (std::max)(g_service.probes.presentElbowMax[hand], elbowMoveGameUnits);
        g_service.probes.presentStretchMax[hand] = (std::max)(g_service.probes.presentStretchMax[hand], reachDeficitGameUnits);
        state.presentedThisFrame = true;
        const transport_policy::HandTransport transport{ .delta = delta, .active = true };
        if (state.presentedHandValid) {
            state.presentedHandWorld = transport_policy::transportWorld(transport, state.presentedHandWorld);
        }
        if (state.renderedHandNodeValid) {
            state.renderedHandNodeWorld = transport_policy::transportWorld(transport, state.renderedHandNodeWorld);
        }

        if (!debugEnabled()) {
            state.presentTraceLines = 0;
            return;
        }
        /*
         * Per-frame trace of a presentation episode (dense, then every 30th
         * frame). seat = how far this hand's target moved since the previous
         * ROCK frame; rebase = how far the pre-FRIK pass moved it; driver =
         * the driver chain's motion over the frame; delta = seat versus
         * rebase, the residual the arm was re-solved by.
         */
        constexpr std::uint32_t kDenseTraceLines = 240;
        const std::uint32_t line = state.presentTraceLines++;
        if (line >= kDenseTraceLines && (line - kDenseTraceLines) % 30 != 0) {
            return;
        }
        const registry_policy::Claim* top = registry_policy::winner(g_service.registry, isLeft);
        const auto& last = g_service.lastFrameTargets[hand];
        const auto& consumed = g_service.consumedTargets[hand];
        float seatMotion = 0.0f;
        float seatRotation = 0.0f;
        float rebaseMotion = 0.0f;
        float rebaseRotation = 0.0f;
        float driverMotion = 0.0f;
        if (top && last.valid) {
            seatMotion = registry_policy::translationDeltaGameUnits(top->target, last.target);
            seatRotation = registry_policy::rotationDeltaDegrees(top->target, last.target);
            if (consumed.valid) {
                rebaseMotion = registry_policy::translationDeltaGameUnits(consumed.target, last.target);
                rebaseRotation = registry_policy::rotationDeltaDegrees(consumed.target, last.target);
            }
        }
        const PredictionError& prediction = g_service.predictionErrors[hand];
        if (top) {
            const DriverSample* now = registry_policy::sampleForDriver(g_service.driverFrame, top->driver);
            const DriverSample* before = registry_policy::sampleForDriver(g_service.previousDriverFrame, top->driver);
            if (now && before && now->valid && before->valid) {
                driverMotion = isolation_policy::translationGameUnits(now->world, before->world);
            }
        }
        ROCK_LOG_DEBUG(Hand,
            "PRESENT hand={} line={} tag='{}' driver={} delta={:.2f}gu/{:.2f}deg elbow={:.2f}gu stretch={:.2f}gu seat={:.2f}gu/{:.2f}deg rebase={:.2f}gu/{:.2f}deg driverMotion={:.2f}gu predErr={:.3f}gu/{:.3f}deg src={}",
            isLeft ? "L" : "R",
            line,
            top ? top->tag.data() : "-",
            top ? driverName(top->driver) : "-",
            state.presentTranslationGameUnits,
            state.presentRotationDegrees,
            elbowMoveGameUnits,
            reachDeficitGameUnits,
            seatMotion,
            seatRotation,
            rebaseMotion,
            rebaseRotation,
            driverMotion,
            prediction.valid ? prediction.translationGameUnits : 0.0f,
            prediction.valid ? prediction.rotationDegrees : 0.0f,
            rawHandSourceName(isLeft));
    }

    void endRockFrame()
    {
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            g_service.lastFrameTargets[hand] = registry_policy::snapshotConsumedTarget(g_service.registry, isLeft);
            if (!g_service.isolation[hand].presentedThisFrame) {
                g_service.isolation[hand].presentTraceLines = 0;
            }
        }
    }

    ScopeDampenTrace scopeDampenTraceBeforeFrik() noexcept { return g_service.scopePreTrace; }

    ScopeDampenTrace scopeDampenTrace() noexcept
    {
        const auto& d = g_service.dampen;
        ScopeDampenTrace result{};
        result.driverSequence = g_service.driverFrame.sequence;
        result.observedSequence = g_service.rockFrameSequence;
        result.runtimeFrameUsed = d.runtimeFrameUsed;
        result.menuUsed = d.menuUsed;
        result.enabled = d.factorsThisPass.enabled;
        result.translationFactor = d.factorsThisPass.translation;
        result.rotationFactor = d.factorsThisPass.rotation;
        result.cameraNow = d.cameraNow;
        result.cameraPrevious = d.cameraPrev;
        result.cameraNowValid = d.cameraNowValid;
        result.cameraPreviousValid = d.cameraPrevValid;
        result.raw = g_service.rawFrame.hands;
        result.driver = g_service.driverFrame.hands;
        result.history = d.previousDampened;
        result.historySequence = d.historySequence;
        result.historyCamera = d.historyCamera;
        result.historyCameraValid = d.historyCameraValid;
        result.consumed = g_service.consumedTargets;
        for (std::size_t i = 0; i < 2; ++i) {
            result.presented[i] = { g_service.isolation[i].presentedHandWorld, g_service.isolation[i].presentedHandValid };
            result.claimed[i] = registry_policy::snapshotConsumedTarget(g_service.registry, i == handIndex(true));
        }
        return result;
    }

    void clearAllClaims()
    {
        for (const auto& claim : g_service.registry.claims) {
            if (claim.valid) {
                (void)frikClearHandWorld(claim.tag.data(), claim.isLeft);
            }
        }
        registry_policy::clearAll(g_service.registry);
        g_service.claimConsumedThisFrame = {};
        g_service.consumedTargets = {};
        g_service.lastFrameTargets = {};
    }

    void resetForSkeletonRelease()
    {
        g_service.scopePreTrace = {};
        g_service.dampen = {};
        g_service.predictionErrors = {};
        g_service.rawFrame = {};
        registry_policy::clearAll(g_service.registry);
        g_service.claimConsumedThisFrame = {};
        g_service.consumedTargets = {};
        g_service.lastFrameTargets = {};
        g_service.presentationAllowed = false;
        for (auto& state : g_service.isolation) {
            state = {};
        }
    }
}
