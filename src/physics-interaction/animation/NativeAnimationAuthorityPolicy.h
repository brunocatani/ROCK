#pragma once

#include <cstdint>
#include <string_view>

namespace rock::native_animation_authority_policy
{
    inline constexpr std::uint32_t kArms = 1u << 0;
    inline constexpr std::uint32_t kHands = 1u << 1;
    inline constexpr std::uint32_t kWeapon = 1u << 2;
    inline constexpr std::uint32_t kWeaponFixedHandsPose = kArms | kHands;
    inline constexpr std::uint32_t kManualCyclePose = kWeaponFixedHandsPose;
    inline constexpr std::uint32_t kReloadPose = kArms | kHands | kWeapon;
    inline constexpr float kManualCycleHandMotionTranslationThresholdGameUnits = 1.5f;
    inline constexpr float kManualCycleHandMotionRotationThresholdDegrees = 10.0f;

    enum class LocalReloadLeaseEndReason : std::uint32_t
    {
        None = 0,
        ReloadEnded,
        WatchdogExpired,
    };

    struct LocalReloadLifecycleSignal
    {
        std::uint64_t startSequence{ 0 };
        std::uint64_t endSequence{ 0 };
        bool reloadActive{ false };
    };

    struct LocalReloadLeaseState
    {
        std::uint32_t watchdogFramesRemaining{ 0 };
        std::uint64_t startSequenceAtArm{ 0 };
        std::uint64_t endSequenceAtArm{ 0 };
        bool observedReloadStart{ false };
    };

    struct LocalReloadLeaseStep
    {
        LocalReloadLeaseState state{};
        LocalReloadLeaseEndReason endReason{ LocalReloadLeaseEndReason::None };

        [[nodiscard]] constexpr bool active() const
        {
            return endReason == LocalReloadLeaseEndReason::None && state.watchdogFramesRemaining > 0;
        }
    };

    struct LocalReloadAuthoritySelection
    {
        bool leaseActive{ false };
        bool partialAuthorityEnabled{ false };
        bool rockTwoHandWeaponAuthorityActive{ false };
    };

    /*
     * Full authority preserves the existing arms/hands/Weapon composition.
     * Partial authority deliberately fails closed unless ROCK owns the weapon
     * through the same stable right-primary two-hand solver used by manual
     * cycling; a one-hand reload therefore remains Bethesda parts-only.
     */
    [[nodiscard]] inline constexpr std::uint32_t resolveLocalReloadAuthorityFlags(
        const LocalReloadAuthoritySelection& selection)
    {
        if (!selection.leaseActive) {
            return 0;
        }
        if (!selection.partialAuthorityEnabled) {
            return kReloadPose;
        }
        return selection.rockTwoHandWeaponAuthorityActive ?
            kWeaponFixedHandsPose :
            0;
    }

    enum class LocalManualCycleLeaseEndReason : std::uint32_t
    {
        None = 0,
        CycleBracketEnded,
        ReloadStarted,
        WatchdogExpired,
    };

    struct LocalManualCycleLifecycleSignal
    {
        std::uint64_t reloadStartSequence{ 0 };
        std::uint64_t reloadEndSequence{ 0 };
        float deltaSeconds{ 0.0f };
    };

    struct LocalManualCycleLeaseState
    {
        float watchdogSecondsRemaining{ 0.0f };
        std::uint64_t reloadStartSequenceAtArm{ 0 };
        std::uint64_t lastReloadEndSequence{ 0 };
        std::uint32_t observedReloadEndEvents{ 0 };
    };

    struct LocalManualCycleLeaseStep
    {
        LocalManualCycleLeaseState state{};
        LocalManualCycleLeaseEndReason endReason{ LocalManualCycleLeaseEndReason::None };

        [[nodiscard]] constexpr bool active() const
        {
            return endReason == LocalManualCycleLeaseEndReason::None &&
                   state.watchdogSecondsRemaining > 0.0f;
        }
    };

    struct ManualCycleTwoHandEligibility
    {
        bool twoHandGripActive{ false };
        bool firingHandIsLeft{ false };
        bool weaponTransformOwned{ false };
    };

    [[nodiscard]] inline constexpr bool canApplyManualCycleHandAnimation(
        const ManualCycleTwoHandEligibility& eligibility)
    {
        return eligibility.twoHandGripActive &&
               !eligibility.firingHandIsLeft &&
               eligibility.weaponTransformOwned;
    }

    struct ManualCycleHandMotionSample
    {
        float translationGameUnits{ 0.0f };
        float rotationDegrees{ 0.0f };
    };

    [[nodiscard]] inline constexpr bool updateManualCycleHandMotionQualification(
        const bool alreadyQualified,
        const ManualCycleHandMotionSample& sample)
    {
        return alreadyQualified ||
               sample.translationGameUnits >=
                   kManualCycleHandMotionTranslationThresholdGameUnits ||
               sample.rotationDegrees >=
                   kManualCycleHandMotionRotationThresholdDegrees;
    }

    struct AuthoredPrimaryFiringGripEligibility
    {
        bool enabled{ false };
        bool runtimeInitialized{ false };
        bool visualAuthorityAvailable{ false };
        bool localSkeletonReady{ false };
        bool menuBlocking{ false };
        bool compatibilityBlocking{ false };
        bool weaponDrawn{ false };
        bool weaponVisible{ false };
        bool weaponKeyValid{ false };
        bool captureValid{ false };
        bool captureNewerThanWeaponBoundary{ false };
        bool nativeReloadAuthorityActive{ false };
        bool conflictingWeaponTransformAuthorityActive{ false };
        bool weaponVisualReturnActive{ false };
        bool primaryHandHoldingObject{ false };
        // Flat Fallout 4 authors its firing pose on RArm_Hand -> Weapon.
        // Game-left-handed mode has a separate Bethesda/hFRIK topology and is
        // not inferred from this right-authored relation. ROCK's physical-left
        // firing role mirrors the validated right canonical downstream.
        bool leftHandedMode{ false };
        bool rockFiringHandIsLeft{ false };
    };

    struct AuthoredSupportGripCandidateInput
    {
        bool featureEnabled{ false };
        bool proximityProbeAcquisition{ false };
        bool authoredSeatTouchAcquisition{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool captureValid{ false };
        bool weaponIdentityMatches{ false };
        bool generationMatches{ false };
        bool completeFingerPose{ false };
    };

    struct AuthoredFiringGripProbeInput
    {
        bool featureEnabled{ false };
        bool proximityProbeAcquisition{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool authoredCanonicalAvailable{ false };
    };

    [[nodiscard]] constexpr bool shouldApplyAuthoredPrimaryFiringGrip(
        const AuthoredPrimaryFiringGripEligibility& input)
    {
        return input.enabled &&
               input.runtimeInitialized &&
               input.visualAuthorityAvailable &&
               input.localSkeletonReady &&
               !input.menuBlocking &&
               !input.compatibilityBlocking &&
               input.weaponDrawn &&
               input.weaponVisible &&
               input.weaponKeyValid &&
               input.captureValid &&
               input.captureNewerThanWeaponBoundary &&
               !input.nativeReloadAuthorityActive &&
               !input.conflictingWeaponTransformAuthorityActive &&
               !input.weaponVisualReturnActive &&
               !input.primaryHandHoldingObject &&
               !input.leftHandedMode &&
               !input.rockFiringHandIsLeft;
    }

    /*
     * Explicit consumer/provider authority remains the highest-priority part
     * grab. Bethesda's authored support grip is selected for a broad proximity
     * acquisition or when the small physical-touch probe is already inside the
     * final authored palm-seat radius. Other physical touches keep ROCK's
     * unrestricted mesh grab. AttachOnly remains consumer-owned glue.
     */
    [[nodiscard]] constexpr bool shouldUseAuthoredSupportGrip(
        const AuthoredSupportGripCandidateInput& input)
    {
        return input.featureEnabled &&
               (input.proximityProbeAcquisition ||
                   input.authoredSeatTouchAcquisition) &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.captureValid &&
               input.weaponIdentityMatches &&
               input.generationMatches &&
               input.completeFingerPose;
    }

    [[nodiscard]] constexpr bool shouldUseAuthoredFiringGripProbe(
        const AuthoredFiringGripProbeInput& input)
    {
        return input.featureEnabled &&
               input.proximityProbeAcquisition &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.authoredCanonicalAvailable;
    }

    [[nodiscard]] constexpr LocalReloadLeaseStep advanceLocalReloadLease(
        LocalReloadLeaseState state,
        LocalReloadLifecycleSignal signal)
    {
        if (state.watchdogFramesRemaining == 0) {
            return { state, LocalReloadLeaseEndReason::WatchdogExpired };
        }

        --state.watchdogFramesRemaining;
        if (signal.reloadActive || signal.startSequence != state.startSequenceAtArm) {
            state.observedReloadStart = true;
        }
        if (state.observedReloadStart && signal.endSequence != state.endSequenceAtArm) {
            return { state, LocalReloadLeaseEndReason::ReloadEnded };
        }
        if (state.watchdogFramesRemaining == 0) {
            return { state, LocalReloadLeaseEndReason::WatchdogExpired };
        }
        return { state, LocalReloadLeaseEndReason::None };
    }

    /*
     * Bethesda's manual-cycle action is bracketed by two ReloadEnd graph
     * events: one at the start of the bolt/lever clip and one at its end. A
     * player WeaponFire event arms this state with the event sequence sampled
     * at that exact boundary. A real reload start wins immediately, while the
     * duration derived from the equipped weapon's live animation data remains
     * a bounded fallback for non-conforming replacement clips.
     */
    [[nodiscard]] constexpr LocalManualCycleLeaseStep advanceLocalManualCycleLease(
        LocalManualCycleLeaseState state,
        LocalManualCycleLifecycleSignal signal)
    {
        if (state.watchdogSecondsRemaining <= 0.0f) {
            state.watchdogSecondsRemaining = 0.0f;
            return { state, LocalManualCycleLeaseEndReason::WatchdogExpired };
        }

        if (signal.reloadStartSequence != state.reloadStartSequenceAtArm) {
            state.watchdogSecondsRemaining = 0.0f;
            return { state, LocalManualCycleLeaseEndReason::ReloadStarted };
        }

        if (signal.reloadEndSequence != state.lastReloadEndSequence) {
            const std::uint64_t eventCount =
                signal.reloadEndSequence > state.lastReloadEndSequence ?
                signal.reloadEndSequence - state.lastReloadEndSequence :
                1;
            state.lastReloadEndSequence = signal.reloadEndSequence;
            const std::uint64_t totalEvents =
                static_cast<std::uint64_t>(state.observedReloadEndEvents) + eventCount;
            constexpr std::uint64_t maxEventCount = 0xFFFFFFFFull;
            state.observedReloadEndEvents = totalEvents > maxEventCount ?
                static_cast<std::uint32_t>(maxEventCount) :
                static_cast<std::uint32_t>(totalEvents);
            if (state.observedReloadEndEvents >= 2) {
                state.watchdogSecondsRemaining = 0.0f;
                return { state, LocalManualCycleLeaseEndReason::CycleBracketEnded };
            }
        }

        const float deltaSeconds =
            signal.deltaSeconds > 0.0f && signal.deltaSeconds <= 0.1f ?
            signal.deltaSeconds :
            (1.0f / 90.0f);
        state.watchdogSecondsRemaining -= deltaSeconds;
        if (state.watchdogSecondsRemaining <= 0.0f) {
            state.watchdogSecondsRemaining = 0.0f;
            return { state, LocalManualCycleLeaseEndReason::WatchdogExpired };
        }
        return { state, LocalManualCycleLeaseEndReason::None };
    }

    /*
     * Resolve one rigid world correction for the complete authored pose:
     *
     *   authoredDelta = inverse(authoredBaseline) * authoredCurrent
     *   desiredAnchor = liveControl * authoredDelta
     *   correction = desiredAnchor * inverse(authoredCurrent)
     *
     * Applying correction to every selected hierarchy root keeps the authored
     * arms/hands/weapon relationship intact while the live controller replaces
     * the flat game's mouse-aim frame.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveControllerAnchoredPoseCorrection(
        const Transform& liveControl,
        const Transform& authoredBaseline,
        const Transform& authoredCurrent,
        Compose&& compose,
        Invert&& invert)
    {
        const Transform authoredDelta = compose(invert(authoredBaseline), authoredCurrent);
        const Transform desiredAnchor = compose(liveControl, authoredDelta);
        return compose(desiredAnchor, invert(authoredCurrent));
    }

    /*
     * Align an authored pose from any native tree space to one already-resolved
     * world target. The visible first-person weapon supplies that target; the
     * full-body tree must not derive a second target from its hidden Weapon
     * node because that node follows the current FRIK arm pose.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolvePoseCorrectionToWorldTarget(
        const Transform& worldTarget,
        const Transform& authoredCurrent,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(worldTarget, invert(authoredCurrent));
    }

    /*
     * Recover the animated hand in Weapon space from two graph-local model
     * transforms captured in the same native hierarchy. This relation is the
     * datum needed by hand-only cycle authority: the visible Weapon remains in
     * ROCK's controller world, while hFRIK solves the hand to W * handInWeapon.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveNativeHandInWeapon(
        const Transform& nativeWeaponModel,
        const Transform& nativeHandModel,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(invert(nativeWeaponModel), nativeHandModel);
    }

    /*
     * Bethesda's native primary-arm pass runs before hFRIK replaces the
     * weapon basis. Capture the resulting hand in that native weapon frame.
     * The forward composition remains useful for measuring the visible
     * mismatch before the inverse solve is applied.
     */
    template <class Transform, class Compose>
    [[nodiscard]] constexpr Transform resolveAuthoredPrimaryHandWorld(
        const Transform& liveWeaponWorld,
        const Transform& authoredHandInWeapon,
        Compose&& compose)
    {
        return compose(liveWeaponWorld, authoredHandInWeapon);
    }

    /*
     * Move the weapon, not the tracked hand. If A is Bethesda's authored
     * hand-in-weapon relation and H is hFRIK's controller-driven hand world,
     * solve W such that W * A == H:
     *
     *   W = H * inverse(A)
     *
     * This is the automatic equivalent of a per-weapon FRIK adjustment, but
     * it is derived from the modeler's native animation rather than a manual
     * translation/rotation guess.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveAuthoredPrimaryWeaponWorld(
        const Transform& trackedPrimaryHandWorld,
        const Transform& authoredHandInWeapon,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(trackedPrimaryHandWorld, invert(authoredHandInWeapon));
    }

    /*
     * The paired non-primary Bethesda pass does not place LArm_Hand in world;
     * it aligns WeaponLeft against the selected secondary offset. Recover the
     * authored support target without consulting that live presentation state:
     *
     *   B = inverse(authoredPrimaryHandModel) * authoredSupportHandModel
     *   supportHandInWeapon = primaryHandInWeapon * B
     *
     * `primaryHandInWeapon` is the already runtime-validated per-weapon result
     * from Bethesda's primary pass. The two model transforms are reconstructed
     * from graph locals in the same flattened hierarchy, so their shared model
     * root cancels and no controller/world transform enters the authored datum.
     */
    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveAuthoredSupportHandInPrimaryHand(
        const Transform& authoredPrimaryHandModel,
        const Transform& authoredSupportHandModel,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(
            invert(authoredPrimaryHandModel),
            authoredSupportHandModel);
    }

    template <class Transform, class Compose>
    [[nodiscard]] constexpr Transform resolveAuthoredSupportHandInWeapon(
        const Transform& primaryHandInWeapon,
        const Transform& supportHandInPrimaryHand,
        Compose&& compose)
    {
        return compose(primaryHandInWeapon, supportHandInPrimaryHand);
    }

    [[nodiscard]] constexpr char asciiLower(char value)
    {
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
    }

    [[nodiscard]] constexpr bool equalsIgnoreCase(std::string_view lhs, std::string_view rhs)
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            if (asciiLower(lhs[i]) != asciiLower(rhs[i])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] constexpr bool startsWithIgnoreCase(std::string_view value, std::string_view prefix)
    {
        return value.size() >= prefix.size() && equalsIgnoreCase(value.substr(0, prefix.size()), prefix);
    }

    /*
     * Only the two arm chains and the two weapon roots are eligible. The hand
     * root participates in both Arms and Hands so either partial request has a
     * stable hierarchy boundary. Finger/thumb descendants belong to Hands;
     * no root, COM, spine, head, or leg transform can enter this authority.
     */
    [[nodiscard]] constexpr std::uint32_t classifyBone(std::string_view name)
    {
        if (equalsIgnoreCase(name, "Weapon") || equalsIgnoreCase(name, "WeaponLeft")) {
            return kWeapon;
        }

        constexpr std::string_view leftArmPrefix = "LArm_";
        constexpr std::string_view rightArmPrefix = "RArm_";
        std::string_view suffix;
        if (startsWithIgnoreCase(name, leftArmPrefix)) {
            suffix = name.substr(leftArmPrefix.size());
        } else if (startsWithIgnoreCase(name, rightArmPrefix)) {
            suffix = name.substr(rightArmPrefix.size());
        } else {
            return 0;
        }

        if (equalsIgnoreCase(suffix, "Hand")) {
            return kArms | kHands;
        }
        if (startsWithIgnoreCase(suffix, "Finger") || startsWithIgnoreCase(suffix, "Thumb")) {
            return kHands;
        }
        return kArms;
    }

    [[nodiscard]] constexpr bool isRequested(std::uint32_t boneFlags, std::uint32_t requestedFlags)
    {
        return (boneFlags & requestedFlags & kReloadPose) != 0;
    }
}
