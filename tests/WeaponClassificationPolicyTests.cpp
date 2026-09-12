#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/weapon/MinigunFiringGripPolicy.h"

using namespace rock;
using namespace rock::weapon_classification_policy;

namespace
{
    [[nodiscard]] consteval std::uint64_t flags(
        const WeaponKeywordFlag first,
        const WeaponKeywordFlag second = WeaponKeywordFlag::None)
    {
        return static_cast<std::uint64_t>(first) |
               static_cast<std::uint64_t>(second);
    }
}

int main()
{
    static_assert(classify({
        .nativeMeleeType = true,
    }).sizeClass == WeaponSizeClass::Melee);
    static_assert(classify({
        .nativeMeleeType = true,
    }).source == WeaponClassificationSource::WeaponData);

    static_assert(classify({
        .keywordFlags = flags(WeaponKeywordFlag::Minigun),
    }).sizeClass == WeaponSizeClass::Heavy);
    static_assert(classify({
        .keywordFlags = flags(WeaponKeywordFlag::GammaGun),
    }).sizeClass == WeaponSizeClass::Pistol);
    static_assert(classify({
        .keywordFlags = flags(WeaponKeywordFlag::RailwayRifle),
    }).sizeClass == WeaponSizeClass::Rifle);
    static_assert(classify({
        .keywordFlags = flags(WeaponKeywordFlag::Ripper),
    }).sizeClass == WeaponSizeClass::Melee);

    constexpr auto convertedPistol = classify({
        .keywordFlags = flags(
            WeaponKeywordFlag::Pistol,
            WeaponKeywordFlag::Rifle),
        .effectiveEquipSlotFormID = kBothHandsEquipSlotFormID,
    });
    static_assert(convertedPistol.sizeClass == WeaponSizeClass::Rifle);
    static_assert(convertedPistol.source == WeaponClassificationSource::EquipSlot);

    constexpr auto optionallySupportedPistol = classify({
        .keywordFlags = flags(
            WeaponKeywordFlag::Pistol,
            WeaponKeywordFlag::Rifle),
        .effectiveEquipSlotFormID =
            kBothHandsLeftOptionalEquipSlotFormID,
    });
    static_assert(
        optionallySupportedPistol.sizeClass == WeaponSizeClass::Pistol);
    static_assert(
        optionallySupportedPistol.source ==
        WeaponClassificationSource::EquipSlot);
    static_assert(optionallySupportedPistol.resolved);

    constexpr auto unresolvedConflictingKeywords = classify({
        .keywordFlags = flags(
            WeaponKeywordFlag::Pistol,
            WeaponKeywordFlag::Rifle),
        .effectiveEquipSlotFormID = 0xDEADBEEFu,
    });
    static_assert(!unresolvedConflictingKeywords.resolved);
    static_assert(unresolvedConflictingKeywords.source == WeaponClassificationSource::None);

    constexpr auto unresolvedConflictingKeywordsWithoutSlot = classify({
        .keywordFlags = flags(
            WeaponKeywordFlag::Pistol,
            WeaponKeywordFlag::Rifle),
    });
    static_assert(!unresolvedConflictingKeywordsWithoutSlot.resolved);
    static_assert(unresolvedConflictingKeywordsWithoutSlot.source == WeaponClassificationSource::None);

    static_assert(!classify({
        .effectiveEquipSlotFormID = kRightHandEquipSlotFormID,
    }).resolved);
    static_assert(!classify({
        .effectiveEquipSlotFormID = kBothHandsEquipSlotFormID,
    }).resolved);
    static_assert(!classify({
        .effectiveEquipSlotFormID =
            kBothHandsLeftOptionalEquipSlotFormID,
    }).resolved);
    static_assert(!classify({}).resolved);
    static_assert(classify({}).source == WeaponClassificationSource::None);

    using namespace rock::minigun_firing_grip_policy;
    static_assert(usesCompiledFiringSeat(
        flags(WeaponKeywordFlag::Minigun)));
    static_assert(usesCompiledFiringSeat(
        flags(
            WeaponKeywordFlag::Minigun,
            WeaponKeywordFlag::HeavyGun)));
    static_assert(!usesCompiledFiringSeat(
        flags(WeaponKeywordFlag::HeavyGun)));
    static_assert(!usesCompiledFiringSeat(
        flags(WeaponKeywordFlag::GatlingLaser)));

    constexpr auto regularMinigunSeat = weaponInFiringHand(false);
    constexpr auto powerArmorMinigunSeat = weaponInFiringHand(true);
    static_assert(regularMinigunSeat.rotation ==
                  powerArmorMinigunSeat.rotation);
    static_assert(regularMinigunSeat.translation ==
                  powerArmorMinigunSeat.translation);
    static_assert(regularMinigunSeat.scale ==
                  powerArmorMinigunSeat.scale);
    static_assert(regularMinigunSeat.translation[0] ==
                  6.887054443359375f);
    static_assert(regularMinigunSeat.translation[1] ==
                  5.837783336639404f);
    static_assert(regularMinigunSeat.translation[2] ==
                  8.01791000366211f);

    return 0;
}
