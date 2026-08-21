#pragma once

#include <cstdint>
#include <string_view>

namespace rock::authored_weapon_grip_capture_policy
{
    // These values intentionally match the public ROCK V1 native-animation
    // authority mask. Authored grip application must yield whenever an addon
    // owns any overlapping hand/weapon animation authority.
    inline constexpr std::uint32_t kArms = 1u << 0;
    inline constexpr std::uint32_t kHands = 1u << 1;
    inline constexpr std::uint32_t kWeapon = 1u << 2;
    inline constexpr std::uint16_t
        kCompleteAuthoredSupportFingerLocalTransformMask = 0x7FFFu;

    struct AuthoredPrimaryFiringGripEligibility
    {
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
        bool rockFiringHandIsLeft{ false };
    };

    struct AuthoredSupportGripCandidateInput
    {
        bool interactionAcquisitionValid{ false };
        bool activationZoneValid{ false };
        bool authoredPoseSurfaceEvidenceValid{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool captureValid{ false };
        bool weaponIdentityMatches{ false };
        bool generationMatches{ false };
        bool authoredSeatWeaponSurfaceValid{ false };
        bool completeFingerPose{ false };
    };

    struct AuthoredFiringGripProbeInput
    {
        bool proximityProbeAcquisition{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool authoredCanonicalAvailable{ false };
    };

    struct PresentedFiringHandTargetInput
    {
        bool authoredFingerPosePublished{ false };
        bool presentedHandWorldValid{ false };
        bool currentNativeRecoilSampleNeutral{ false };
    };

    struct StableAuthoredSupportGripReuseInput
    {
        bool snapshotValid{ false };
        bool weaponNodeValid{ false };
        bool weaponNodeMatches{ false };
        std::uint64_t currentWeaponOwnershipKey{ 0 };
        std::uint64_t snapshotWeaponOwnershipKey{ 0 };
        std::uint64_t currentWeaponGenerationKey{ 0 };
        std::uint64_t snapshotWeaponGenerationKey{ 0 };
        std::uint64_t currentPrimaryGripCaptureSequence{ 0 };
        std::uint64_t snapshotPrimaryGripCaptureSequence{ 0 };
        std::uint64_t snapshotSupportGripCaptureSequence{ 0 };
        std::uint16_t snapshotFingerLocalTransformMask{ 0 };
    };

    [[nodiscard]] constexpr bool shouldApplyAuthoredPrimaryFiringGrip(
        const AuthoredPrimaryFiringGripEligibility& input)
    {
        return input.runtimeInitialized &&
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
               !input.rockFiringHandIsLeft;
    }

    /*
     * Full two-hand authority supersedes the authored weapon transform, but
     * it consumes the same identity-bound authored firing-hand canonical.
     * Keep that canonical's finger lease continuous while every independent
     * pose prerequisite remains valid. This prevents a per-frame clear/set
     * cycle without allowing the authored transform writer back in.
     */
    [[nodiscard]] constexpr bool
        shouldRetainAuthoredFiringFingerPoseWhileWeaponTransformYields(
            const AuthoredPrimaryFiringGripEligibility& input)
    {
        if (!input.conflictingWeaponTransformAuthorityActive) {
            return false;
        }

        auto fingerEligibility = input;
        fingerEligibility.conflictingWeaponTransformAuthorityActive = false;
        return shouldApplyAuthoredPrimaryFiringGrip(fingerEligibility);
    }

    /*
     * A ROCK object grab owns the occupied physical hand's fingers. The
     * equipped weapon may keep its independent transform/carry authority,
     * but its persistent authored firing pose must not compete with the
     * ROCK_Grab pose for that same hand.
     */
    [[nodiscard]] constexpr bool shouldPublishAuthoredFiringFingerPose(
        const bool targetHandHoldingObject) noexcept
    {
        return !targetHandHoldingObject;
    }

    [[nodiscard]] constexpr bool shouldUseAuthoredSupportGrip(
        const AuthoredSupportGripCandidateInput& input)
    {
        return input.interactionAcquisitionValid &&
               input.activationZoneValid &&
               input.authoredPoseSurfaceEvidenceValid &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.captureValid &&
               input.weaponIdentityMatches &&
               input.generationMatches &&
               input.authoredSeatWeaponSurfaceValid &&
               input.completeFingerPose;
    }

    [[nodiscard]] constexpr bool shouldUseAuthoredFiringGripProbe(
        const AuthoredFiringGripProbeInput& input)
    {
        return input.proximityProbeAcquisition &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.authoredCanonicalAvailable;
    }

    /*
     * The rendered firing wrist is the exact deferred hFRIK target only when
     * this skeleton frame has no native recoil. A firing frame deliberately
     * moves that wrist away from its tracking target. Feeding the moved wrist
     * back into the persistent weapon alignment latches the shot transform and
     * invalidates the weapon-relative support grip until a reload or equip
     * boundary rebuilds it.
     */
    [[nodiscard]] constexpr bool shouldUsePresentedFiringHandTarget(
        const PresentedFiringHandTargetInput& input) noexcept
    {
        return input.authoredFingerPosePublished &&
               input.presentedHandWorldValid &&
               input.currentNativeRecoilSampleNeutral;
    }

    /*
     * Bethesda may omit the paired support-arm pass during transient firing
     * animation frames. Reuse is safe only for the last fully validated pose
     * while every equipped-weapon and canonical identity witness still
     * matches; reload and weapon boundaries clear the snapshot separately.
     */
    [[nodiscard]] constexpr bool shouldReuseStableAuthoredSupportGrip(
        const StableAuthoredSupportGripReuseInput& input) noexcept
    {
        return input.snapshotValid &&
               input.weaponNodeValid &&
               input.weaponNodeMatches &&
               input.currentWeaponOwnershipKey != 0 &&
               input.currentWeaponOwnershipKey ==
                   input.snapshotWeaponOwnershipKey &&
               input.currentWeaponGenerationKey != 0 &&
               input.currentWeaponGenerationKey ==
                   input.snapshotWeaponGenerationKey &&
               input.currentPrimaryGripCaptureSequence != 0 &&
               input.currentPrimaryGripCaptureSequence ==
                   input.snapshotPrimaryGripCaptureSequence &&
               input.snapshotSupportGripCaptureSequence != 0 &&
               input.snapshotFingerLocalTransformMask ==
                   kCompleteAuthoredSupportFingerLocalTransformMask;
    }

    template <class Transform, class Compose>
    [[nodiscard]] constexpr Transform resolveAuthoredPrimaryHandWorld(
        const Transform& liveWeaponWorld,
        const Transform& authoredHandInWeapon,
        Compose&& compose)
    {
        return compose(liveWeaponWorld, authoredHandInWeapon);
    }

    template <class Transform, class Compose, class Invert>
    [[nodiscard]] constexpr Transform resolveAuthoredPrimaryWeaponWorld(
        const Transform& trackedPrimaryHandWorld,
        const Transform& authoredHandInWeapon,
        Compose&& compose,
        Invert&& invert)
    {
        return compose(trackedPrimaryHandWorld, invert(authoredHandInWeapon));
    }

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

    [[nodiscard]] constexpr char asciiLower(const char value)
    {
        return value >= 'A' && value <= 'Z' ?
            static_cast<char>(value - 'A' + 'a') :
            value;
    }

    [[nodiscard]] constexpr bool equalsIgnoreCase(
        const std::string_view lhs,
        const std::string_view rhs)
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t index = 0; index < lhs.size(); ++index) {
            if (asciiLower(lhs[index]) != asciiLower(rhs[index])) {
                return false;
            }
        }
        return true;
    }
}
