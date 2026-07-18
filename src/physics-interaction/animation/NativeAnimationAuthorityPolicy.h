#pragma once

#include <cstdint>
#include <string_view>

namespace rock::native_animation_authority_policy
{
    inline constexpr std::uint32_t kArms = 1u << 0;
    inline constexpr std::uint32_t kHands = 1u << 1;
    inline constexpr std::uint32_t kWeapon = 1u << 2;
    inline constexpr std::uint32_t kReloadPose = kArms | kHands | kWeapon;

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
        // Left-handed mirroring has no equivalent modeler-authored relation.
        bool leftHandedMode{ false };
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
               !input.leftHandedMode;
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
