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
        bool proximityProbeAcquisition{ false };
        bool providerAuthorityActive{ false };
        bool attachOnly{ false };
        bool authoredCanonicalAvailable{ false };
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
        return (input.proximityProbeAcquisition ||
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
        return input.proximityProbeAcquisition &&
               !input.providerAuthorityActive &&
               !input.attachOnly &&
               input.authoredCanonicalAvailable;
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
