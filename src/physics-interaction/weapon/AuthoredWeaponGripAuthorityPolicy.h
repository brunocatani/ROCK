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
        FreshNativeIdle = 3,
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

    /*
     * Value witness for "same authored hand-in-weapon relation". Capture
     * sequences change whenever the same authored idle is re-published from a
     * different source (live graph or fresh off-screen harvest), so
     * identity-by-sequence alone discards still-valid paired data. Two
     * relations are the same authored pose when every rotation entry and the
     * grip translation agree within the idle-sway tolerance below; a real
     * authoring change (different idle, different variant geometry) exceeds
     * it and fails closed.
     */
    inline constexpr float kHandRelationRotationEntryEpsilon = 0.05f;
    inline constexpr float kHandRelationTranslationEpsilonGameUnits = 1.0f;

    template <class Transform>
    [[nodiscard]] constexpr bool handRelationValueMatches(
        const Transform& lhs,
        const Transform& rhs) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                const float delta =
                    lhs.rotate.entry[row][column] -
                    rhs.rotate.entry[row][column];
                if (!(delta <= kHandRelationRotationEntryEpsilon &&
                        -delta <= kHandRelationRotationEntryEpsilon)) {
                    return false;
                }
            }
        }
        const float dx = lhs.translate.x - rhs.translate.x;
        const float dy = lhs.translate.y - rhs.translate.y;
        const float dz = lhs.translate.z - rhs.translate.z;
        const float squaredDistance = dx * dx + dy * dy + dz * dz;
        return squaredDistance <=
               kHandRelationTranslationEpsilonGameUnits *
                   kHandRelationTranslationEpsilonGameUnits;
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
