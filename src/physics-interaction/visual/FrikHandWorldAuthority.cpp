#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
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
            transport_policy::HandTransport chainTransport{};
        };

        struct ProbeCounters
        {
            std::uint32_t frames = 0;
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
            std::uint64_t rockFrameSequence = 0;
            std::uint64_t rockFrameIndex = 0;
            std::uint64_t observedFrameIndex = 0;
            // Latched by the first resolve of a frame; later resolves in the
            // same frame see no new kick and must not calibrate on it.
            bool recoilKickThisFrame = false;
            std::array<bool, 2> claimConsumedThisFrame{};
            std::array<IsolationState, 2> isolation{};
            SchedulerState scheduler = SchedulerState::Unverified;
            ProbeCounters probes{};
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
                    "HandWorldAuthority probe hand={} frames={} reconstructed={} contaminated={} unavailable={} probeFrames={} probeMaxTranslation={:.3f}gu probeMaxRotation={:.3f}deg relation={} relationAccepted={} relationRejected={} transportFrames={} transportMax={:.2f}gu/{:.2f}deg claimedWithoutTransport={}",
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
                    probes.claimedFramesWithoutTransport[hand]);
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
        g_service.driverFrame = sampleDriverFrame(sequence);
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
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            g_service.claimConsumedThisFrame[hand] = registry_policy::hasClaim(g_service.registry, isLeft);
            g_service.isolation[hand].result = {};
            g_service.isolation[hand].presentedHandValid = false;
            g_service.isolation[hand].chainTransport = {};
        }
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
        if (driver != RebaseDriver::Static && g_service.driverFrame.sequence != 0 &&
            g_service.driverFrame.sequence == g_service.rockFrameSequence) {
            if (const DriverSample* sample = registry_policy::sampleForDriver(g_service.driverFrame, driver)) {
                driverAtPublish = *sample;
            }
        }

        const auto result = registry_policy::commit(g_service.registry, tagView, isLeft, priority, worldTarget, driver, driverAtPublish);
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
        }
        const bool recoilKickThisFrame = g_service.recoilKickThisFrame || samples.recoilKickThisFrame;

        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            const RawHandSample& sample = isLeft ? samples.left : samples.right;
            auto& state = g_service.isolation[hand];

            state.presentedHandWorld = sample.flattenedHandWorld;
            state.presentedHandValid = sample.flattenedHandValid;

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

    void clearAllClaims()
    {
        for (const auto& claim : g_service.registry.claims) {
            if (claim.valid) {
                (void)frikClearHandWorld(claim.tag.data(), claim.isLeft);
            }
        }
        registry_policy::clearAll(g_service.registry);
        g_service.claimConsumedThisFrame = {};
    }

    void resetForSkeletonRelease()
    {
        registry_policy::clearAll(g_service.registry);
        g_service.claimConsumedThisFrame = {};
        for (auto& state : g_service.isolation) {
            state = {};
        }
    }
}
