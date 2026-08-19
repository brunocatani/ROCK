#pragma once

#include <cstddef>
#include <cstdint>

namespace rock::authored_weapon_grip_authority_policy
{
    inline constexpr std::uint16_t kCompleteFiringFingerMask = 0x7FFF;

    enum class PublicationAuthority : std::uint8_t
    {
        Unknown,
        LiveEquippedGraph,
        PersistedNativeIdle,
        FreshNativeIdle,
    };

    [[nodiscard]] constexpr bool completeFiringFingerPose(const std::uint16_t enabledMask) noexcept { return enabledMask == kCompleteFiringFingerMask; }

    [[nodiscard]] constexpr bool publicationHasRequiredFingerPose(const bool nativeIdlePreharvest, const bool completeFingerPose) noexcept
    {
        return !nativeIdlePreharvest || completeFingerPose;
    }

    /*
     * Live equipped capture is retained only as a compatibility fallback.
     * Once the matching off-screen idle asset has been sampled, later live
     * frames cannot replace that stable authored relation or erase its exact
     * finger pose.
     */
    [[nodiscard]] constexpr bool shouldAcceptPublication(
        const bool sameIdentity,
        const PublicationAuthority existing,
        const PublicationAuthority incoming) noexcept
    {
        return !sameIdentity || static_cast<std::uint8_t>(incoming) >= static_cast<std::uint8_t>(existing);
    }

    enum class LookupSelection : std::uint8_t
    {
        None,
        ExactVariant,
        SoleNativeIdleVariant,
        SoleFormVariant,
    };

    /*
     * The equipped Weapon root can temporarily report a generic variant key
     * while its P-Grip subtree is still being assembled. A live capture made
     * in that frame must not shadow the one unambiguous native-idle entry that
     * already supplied the loose weapon's exact pose. Multiple native-idle
     * variants remain ambiguous and therefore fail closed to the exact live
     * variant (or no result when no exact variant exists).
     */
    [[nodiscard]] constexpr LookupSelection selectLookup(
        const bool exactVariantFound,
        const bool exactVariantIsNativeIdle,
        const bool exactVariantIsGeneric,
        const std::size_t nativeIdleVariantCount,
        const std::size_t formVariantCount) noexcept
    {
        if (exactVariantFound && exactVariantIsNativeIdle) {
            return LookupSelection::ExactVariant;
        }
        if (nativeIdleVariantCount == 1 && (!exactVariantFound || exactVariantIsGeneric)) {
            return LookupSelection::SoleNativeIdleVariant;
        }
        if (exactVariantFound) {
            return LookupSelection::ExactVariant;
        }
        if (formVariantCount == 1) {
            return LookupSelection::SoleFormVariant;
        }
        return LookupSelection::None;
    }
}
