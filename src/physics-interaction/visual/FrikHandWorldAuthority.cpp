#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

namespace rock::frik_hand_world_authority
{
    namespace
    {
        namespace registry_policy = hand_world_claim_registry_policy;
        namespace isolation_policy = tracked_hand_isolation_policy;
        namespace transport_policy = rendered_bone_transport_policy;

        using registry_policy::DriverFrame;
        using registry_policy::DriverSample;
        using registry_policy::handIndex;
        using frik_visual_authority::HandSolveState;
        using frik_visual_authority::TrackedHandKind;

        constexpr std::uint32_t kProbeSummaryFrames = 600;

        struct IsolationState
        {
            isolation_policy::RelationState relation{};
            isolation_policy::FrameResult result{};
            // The body hand node FRIK's solver wrote this frame.
            RE::NiTransform renderedHandNodeWorld{};
            bool renderedHandNodeValid = false;
            RE::NiTransform sampledArrayWorld{};
            bool sampledArrayValid = false;
        };

        /*
         * What AfterWorldFinal latched for a hand: the rendered flattened
         * bone, its node, the palm blend between them (flattened in node
         * space) and FRIK's verdict on the claim it solved that frame.
         */
        struct RenderedHand
        {
            RE::NiTransform flattenedWorld{};
            bool flattenedValid = false;
            RE::NiTransform nodeWorld{};
            bool nodeValid = false;
            RE::NiTransform palmBlend{};
            bool palmBlendValid = false;
            HandSolveState solveState = HandSolveState::SkeletonNotReady;
            RE::NiTransform solvedWrist{};
            std::uint64_t frameIndex = 0;
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
            std::array<std::uint32_t, 2> unreachableFrames{};
            std::uint32_t publishes = 0;
            std::uint32_t publishesRejected = 0;
            std::uint32_t claimsRefusedRotation = 0;
            std::uint32_t driverSamplesMissing = 0;
        };

        struct Service
        {
            registry_policy::Registry registry{};
            // This frame's controller driver per hand (FRIK's dampened weapon offset node).
            DriverFrame driverFrame{};
            // This frame's first-person hand per hand (FRIK's tracked target).
            std::array<DriverSample, 2> firstPersonInput{};
            std::array<RenderedHand, 2> rendered{};
            // The winner per hand at the end of the previous ROCK frame, for the trace.
            std::array<registry_policy::ConsumedTarget, 2> lastFrameTargets{};
            std::uint64_t rockFrameSequence = 0;
            std::uint64_t rockFrameIndex = 0;
            std::uint64_t observedFrameIndex = 0;
            // Latched by the first resolve of a frame; later resolves in the
            // same frame see no new kick and must not calibrate on it.
            bool recoilKickThisFrame = false;
            std::array<bool, 2> claimConsumedThisFrame{};
            // The winner per hand when FRIK's arm solve ran: what it solved to.
            std::array<registry_policy::ConsumedTarget, 2> consumedTargets{};
            std::array<IsolationState, 2> isolation{};
            ProbeCounters probes{};
            // Frames left in the scope-edge guard (see noteScopeEdge).
            std::uint32_t scopeEdgeFramesRemaining = 0;
            // ROCK's own scope state last frame, for edges FRIK does not broadcast
            // (ROCK's manual/immersive activation).
            bool lastScopeMenuOpen = false;
            bool scopeStateKnown = false;
        };

        constexpr std::uint32_t kScopeEdgeGuardFrames = 2;

        Service g_service{};

        [[nodiscard]] bool debugEnabled()
        {
            return g_rockConfig.rockDebugHandWorldAuthority;
        }

        [[nodiscard]] const char* handName(const bool isLeft)
        {
            return isLeft ? "left" : "right";
        }

        [[nodiscard]] const char* solveStateName(const HandSolveState state)
        {
            switch (state) {
            case HandSolveState::NoClaim:
                return "no-claim";
            case HandSolveState::Consumed:
                return "consumed";
            case HandSolveState::Unreachable:
                return "unreachable";
            default:
                return "skeleton-not-ready";
            }
        }

        [[nodiscard]] DriverSample sampleTrackedHand(const bool isLeft, const TrackedHandKind kind)
        {
            DriverSample sample{};
            RE::NiTransform world{};
            if (!frik_visual_authority::tryGetTrackedHandTransform(frik_visual_authority::handFromBool(isLeft), kind, world) ||
                !registry_policy::isFiniteTransform(world)) {
                return sample;
            }
            sample.world = world;
            sample.valid = true;
            return sample;
        }

        // The tracked inputs FRIK used for this frame's arm solve.
        void sampleTrackedInputs(const std::uint64_t sequence)
        {
            DriverFrame frame{};
            frame.sequence = sequence;
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const bool isLeft = hand == handIndex(true);
                frame.hands[hand] = sampleTrackedHand(isLeft, TrackedHandKind::WeaponOffset);
                g_service.firstPersonInput[hand] = sampleTrackedHand(isLeft, TrackedHandKind::FirstPersonHand);
                if (!frame.hands[hand].valid) {
                    ++g_service.probes.driverSamplesMissing;
                }
            }
            g_service.driverFrame = frame;
        }

        /*
         * FRIK's verdict on the claim it solved in the frame just rendered
         * replaces the old rendered-wrist heuristic: an Unreachable verdict
         * marks the winning claim so its owner's publishes report failure
         * until FRIK follows the claim again.
         */
        void applySolveResults()
        {
            for (std::size_t hand = 0; hand < 2; ++hand) {
                const bool isLeft = hand == handIndex(true);
                const RenderedHand& rendered = g_service.rendered[hand];
                registry_policy::Claim* top = nullptr;
                for (auto& claim : g_service.registry.claims) {
                    if (claim.valid && claim.isLeft == isLeft &&
                        (!top || claim.priority > top->priority ||
                            (claim.priority == top->priority && claim.publishOrder > top->publishOrder))) {
                        top = &claim;
                    }
                }
                if (!top) {
                    continue;
                }
                switch (rendered.solveState) {
                case HandSolveState::Consumed:
                    top->fallbackReported = false;
                    break;
                case HandSolveState::Unreachable:
                    ++g_service.probes.unreachableFrames[hand];
                    if (!top->fallbackReported) {
                        top->fallbackReported = true;
                        ROCK_LOG_SAMPLE_WARN(Hand,
                            2000,
                            "FRIK solved the tracked {} hand instead of ROCK claim '{}' (priority {}): the target is unreachable for this arm; the owner's publishes report failure until FRIK follows the claim again.",
                            handName(isLeft),
                            top->tag.data(),
                            top->priority);
                    }
                    break;
                default:
                    break;
                }
            }
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
                    "HandWorldAuthority probe hand={} frames={} reconstructed={} contaminated={} unavailable={} probeFrames={} probeMaxTranslation={:.3f}gu probeMaxRotation={:.3f}deg relation={} relationAccepted={} relationRejected={} transportFrames={} transportMax={:.2f}gu/{:.2f}deg claimedWithoutTransport={} unreachable={}",
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
                    probes.unreachableFrames[hand]);
            }
            ROCK_LOG_INFO(Hand,
                "HandWorldAuthority frames={} claims={} publishes={} rejected={} refusedRotation={} driverSamplesMissing={}",
                probes.frames,
                registry_policy::claimCount(g_service.registry),
                probes.publishes,
                probes.publishesRejected,
                probes.claimsRefusedRotation,
                probes.driverSamplesMissing);
            probes = {};
        }
    }

    void captureRenderedFrame(const FrameHandSamples& samples)
    {
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            const RawHandSample& sample = isLeft ? samples.left : samples.right;
            RenderedHand& rendered = g_service.rendered[hand];
            rendered.flattenedWorld = sample.flattenedHandWorld;
            rendered.flattenedValid = sample.flattenedHandValid && registry_policy::isFiniteTransform(sample.flattenedHandWorld);
            rendered.nodeWorld = sample.bodyHandNodeWorld;
            rendered.nodeValid = sample.bodyHandNodeValid && registry_policy::isFiniteTransform(sample.bodyHandNodeWorld);
            rendered.palmBlendValid = false;
            if (rendered.flattenedValid && rendered.nodeValid) {
                const RE::NiTransform blend = transform_math::composeTransforms(
                    transform_math::invertTransform(transform_math::orthonormalizedTransform(rendered.nodeWorld)),
                    rendered.flattenedWorld);
                if (registry_policy::isFiniteTransform(blend)) {
                    rendered.palmBlend = blend;
                    rendered.palmBlendValid = true;
                }
            }
            rendered.solveState = frik_visual_authority::getHandSolveResult(
                frik_visual_authority::handFromBool(isLeft), rendered.solvedWrist);
            rendered.frameIndex = g_service.rockFrameIndex;
            if ((debugEnabled() || g_rockConfig.rockDebugShowSkeletonBoneVisualizer ||
                    g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers) && g_service.rockFrameIndex % 120 == 0) {
                const auto& state = g_service.isolation[hand];
                RE::NiTransform claimed{};
                const bool hasClaim = tryGetPublishedHandWorld(isLeft, claimed);
                const auto& raw = state.result.rawHandWorld;
                const RE::NiPoint3 earlyDelta = state.sampledArrayWorld.translate - raw.translate;
                const RE::NiPoint3 finalDelta = rendered.flattenedWorld.translate - raw.translate;
                ROCK_LOG_INFO(Hand,
                    "HAND_FRAME seq={} hand={} raw={} earlyArray={} earlyNode={} finalArray={} finalNode={} claim={} solve={} earlyArrayMinusRaw=({:.3f},{:.3f},{:.3f}) finalArrayMinusRaw=({:.3f},{:.3f},{:.3f}) earlyArrayToRawR={:.3f} finalArrayToRawR={:.3f} finalNodeToClaim={:.3f}gu/{:.3f}deg scales=({:.6f},{:.6f},{:.6f})",
                    g_service.rockFrameSequence, handName(isLeft), state.result.valid, state.sampledArrayValid,
                    state.renderedHandNodeValid, rendered.flattenedValid, rendered.nodeValid, hasClaim, solveStateName(rendered.solveState),
                    earlyDelta.x, earlyDelta.y, earlyDelta.z, finalDelta.x, finalDelta.y, finalDelta.z,
                    state.sampledArrayValid && state.result.valid ? isolation_policy::rotationDegrees(state.sampledArrayWorld, raw) : -1.0f,
                    rendered.flattenedValid && state.result.valid ? isolation_policy::rotationDegrees(rendered.flattenedWorld, raw) : -1.0f,
                    hasClaim && rendered.nodeValid ? isolation_policy::translationGameUnits(rendered.nodeWorld, claimed) : -1.0f,
                    hasClaim && rendered.nodeValid ? isolation_policy::rotationDegrees(rendered.nodeWorld, claimed) : -1.0f,
                    raw.scale, state.sampledArrayWorld.scale, rendered.flattenedWorld.scale);
            }
        }
    }

    void beginRockFrame(const std::uint64_t sequence)
    {
        g_service.rockFrameSequence = sequence;
        ++g_service.rockFrameIndex;
        const bool scopeOpen = runtime_state::isScopeMenuOpenNow();
        if (g_service.scopeStateKnown && scopeOpen != g_service.lastScopeMenuOpen) {
            noteScopeEdge();
        }
        g_service.lastScopeMenuOpen = scopeOpen;
        g_service.scopeStateKnown = true;
        sampleTrackedInputs(sequence);
        applySolveResults();
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            g_service.claimConsumedThisFrame[hand] = registry_policy::hasClaim(g_service.registry, isLeft);
            g_service.consumedTargets[hand] = registry_policy::snapshotConsumedTarget(g_service.registry, isLeft);
            auto& state = g_service.isolation[hand];
            state.result = {};
            state.renderedHandNodeValid = false;
            state.sampledArrayValid = false;
        }
    }

    void noteScopeEdge()
    {
        g_service.scopeEdgeFramesRemaining = kScopeEdgeGuardFrames;
    }

    bool tryGetInputDriverWorld(const bool isLeft, RE::NiTransform& outWorld)
    {
        outWorld = {};
        const auto& frame = g_service.driverFrame;
        const auto& sample = frame.hands[handIndex(isLeft)];
        if (frame.sequence == 0 || frame.sequence != g_service.rockFrameSequence || !sample.valid) {
            return false;
        }
        outWorld = sample.world;
        return true;
    }

    bool publish(const char* tag, const bool isLeft, const RE::NiTransform& requestedTarget, const int priority)
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

        const std::string_view tagView(tag);
        const auto* frikApi = frik_visual_authority::api();
        if (!frikApi || !frikApi->setHandWorldTransform ||
            !frikApi->setHandWorldTransform(tag, frik_visual_authority::handFromBool(isLeft), worldTarget, priority)) {
            ++g_service.probes.publishesRejected;
            return false;
        }
        ++g_service.probes.publishes;

        const auto result = registry_policy::commit(g_service.registry, tagView, isLeft, priority, worldTarget);
        switch (result) {
        case registry_policy::CommitResult::Inserted:
            return true;
        case registry_policy::CommitResult::Updated: {
            /*
             * FRIK holds the new target, but while it reports the target
             * unreachable the owner is told the claim is not held. Publishing
             * keeps FRIK current so a reachable target ends the episode.
             */
            const registry_policy::Claim* claim = registry_policy::find(g_service.registry, tagView, isLeft);
            return claim && !claim->fallbackReported;
        }
        case registry_policy::CommitResult::Full:
            // FRIK holds the claim but ROCK cannot follow it: undo so ROCK's
            // readers never disagree with FRIK about who owns the hand.
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

    bool tryGetPublishedHandWorld(const bool isLeft, RE::NiTransform& outWorld,
        const char* excludedTag, const int maximumPriority)
    {
        outWorld = {};
        const std::string_view excluded = excludedTag ? std::string_view(excludedTag) : std::string_view{};
        const auto* best = registry_policy::winner(g_service.registry, isLeft, excluded, maximumPriority);
        if (!best) {
            return false;
        }
        outWorld = best->target;
        return true;
    }

    void resolveRawHands(const FrameHandSamples& samples)
    {
        // Called once from the frame entry (before the scope sync and provider
        // callbacks) and again from the physics update; only the first call of
        // a frame advances the probe counters.
        const bool firstResolveThisFrame = g_service.observedFrameIndex != g_service.rockFrameIndex;
        g_service.observedFrameIndex = g_service.rockFrameIndex;
        if (firstResolveThisFrame) {
            g_service.recoilKickThisFrame = samples.recoilKickThisFrame;
        }
        const bool recoilKickThisFrame = g_service.recoilKickThisFrame || samples.recoilKickThisFrame;

        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            const RawHandSample& sample = isLeft ? samples.left : samples.right;
            const RenderedHand& rendered = g_service.rendered[hand];
            auto& state = g_service.isolation[hand];

            /*
             * The rendered hand is the last final frame's flattened bone: at
             * AfterArmSolve the live bone array may have been rebuilt before
             * IK and is not that final render. The body hand node is live. The
             * isolation policy measures the palm blend as flattened versus
             * node in the same frame, so the node is carried through the palm
             * blend latched at the last world final to make a matching pair.
             */
            if (firstResolveThisFrame) {
                state.renderedHandNodeWorld = sample.bodyHandNodeWorld;
                state.renderedHandNodeValid = sample.bodyHandNodeValid;
                state.sampledArrayWorld = sample.flattenedHandWorld;
                state.sampledArrayValid = sample.flattenedHandValid;
            }
            // Before the first final capture there is no palm blend yet. Use
            // the current solved node, never the unfinished array as input.
            RE::NiTransform flattenedNow = sample.bodyHandNodeWorld;
            bool flattenedNowValid = sample.bodyHandNodeValid;
            if (sample.bodyHandNodeValid && rendered.palmBlendValid) {
                const RE::NiTransform synthesized = transform_math::composeTransforms(sample.bodyHandNodeWorld, rendered.palmBlend);
                if (registry_policy::isFiniteTransform(synthesized)) {
                    flattenedNow = synthesized;
                    flattenedNowValid = true;
                }
            }

            isolation_policy::FrameInput input{};
            input.firstPersonHandValid = g_service.firstPersonInput[hand].valid;
            input.firstPersonHandWorld = g_service.firstPersonInput[hand].world;
            // Around a scope edge the first-person tree carries the native
            // scope reset: reconstruct through the last sound relation and
            // never calibrate on it.
            input.firstPersonInputCorrected = g_service.scopeEdgeFramesRemaining > 0;
            input.bodyHandNodeWorld = sample.bodyHandNodeWorld;
            input.bodyHandNodeValid = sample.bodyHandNodeValid;
            input.flattenedHandWorld = flattenedNow;
            input.flattenedHandValid = flattenedNowValid;
            input.claimConsumed = g_service.claimConsumedThisFrame[hand];
            input.calibrationAllowed = !recoilKickThisFrame && !input.firstPersonInputCorrected;
            state.result = isolation_policy::resolveFrame(state.relation, input, debugEnabled());
            if (!firstResolveThisFrame) {
                continue;
            }
            // The scope-edge guard (noteScopeEdge) is a backstop for the native
            // tree reset; say when it engaged so a guard that hides a real
            // input fault stays visible.
            if (input.firstPersonInputCorrected && !input.claimConsumed &&
                state.result.source == isolation_policy::RawHandSource::Reconstructed &&
                g_service.scopeEdgeFramesRemaining == kScopeEdgeGuardFrames) {
                ROCK_LOG_DEBUG(Hand,
                    "HandWorldAuthority scope-edge guard reconstructed the {} hand for {} frames",
                    handName(isLeft),
                    kScopeEdgeGuardFrames);
            }
            auto& probes = g_service.probes;
            if (debugEnabled()) {
                const auto sampledTransport = transport_policy::makeHandTransport(
                    state.result.rawHandWorld, state.result.valid, sample.flattenedHandWorld, sample.flattenedHandValid);
                if (sampledTransport.active) {
                    ++probes.transportActiveFrames[hand];
                    probes.transportTranslationMax[hand] = (std::max)(probes.transportTranslationMax[hand],
                        isolation_policy::translationGameUnits(state.result.rawHandWorld, sample.flattenedHandWorld));
                    probes.transportRotationMax[hand] = (std::max)(probes.transportRotationMax[hand],
                        isolation_policy::rotationDegrees(state.result.rawHandWorld, sample.flattenedHandWorld));
                } else if (input.claimConsumed && (!state.result.valid || !sample.flattenedHandValid)) {
                    ++probes.claimedFramesWithoutTransport[hand];
                }
            }
            switch (state.result.source) {
            case isolation_policy::RawHandSource::Reconstructed:
                ++probes.reconstructedFrames[hand];
                break;
            case isolation_policy::RawHandSource::FlattenedContaminated:
                ++probes.contaminatedFrames[hand];
                ROCK_LOG_SAMPLE_WARN(Hand,
                    5000,
                    "HandWorldAuthority {} hand: controller reconstruction unavailable (first-person hand {}, body node {}, relation {}); interaction input disabled",
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
            if (debugEnabled() && rendered.solveState == HandSolveState::Unreachable) {
                ROCK_LOG_DEBUG(Hand,
                    "HandWorldAuthority {} hand solve={} claimConsumed={} source={}",
                    handName(isLeft),
                    solveStateName(rendered.solveState),
                    input.claimConsumed,
                    rawHandSourceName(isLeft));
            }
        }

        if (firstResolveThisFrame) {
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
        const auto& rendered = g_service.rendered[handIndex(isLeft)];
        if (!rendered.flattenedValid) {
            outWorld = {};
            return false;
        }
        outWorld = rendered.flattenedWorld;
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

    void endRockFrame()
    {
        for (std::size_t hand = 0; hand < 2; ++hand) {
            const bool isLeft = hand == handIndex(true);
            g_service.lastFrameTargets[hand] = registry_policy::snapshotConsumedTarget(g_service.registry, isLeft);
        }
        if (g_service.scopeEdgeFramesRemaining > 0) {
            --g_service.scopeEdgeFramesRemaining;
        }
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
        // A guard armed on an edge that coincides with the rebuild still covers
        // the new skeleton's first calibration frames.
        g_service.scopeStateKnown = false;
        g_service.driverFrame = {};
        g_service.firstPersonInput = {};
        g_service.rendered = {};
        registry_policy::clearAll(g_service.registry);
        g_service.claimConsumedThisFrame = {};
        g_service.consumedTargets = {};
        g_service.lastFrameTargets = {};
        for (auto& state : g_service.isolation) {
            state = {};
        }
    }
}
