#include "physics-interaction/weapon/WeaponClassificationPolicy.h"

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

    static_assert(!classify({
        .effectiveEquipSlotFormID = kRightHandEquipSlotFormID,
    }).resolved);
    static_assert(!classify({
        .effectiveEquipSlotFormID = kBothHandsEquipSlotFormID,
    }).resolved);
    static_assert(!classify({}).resolved);
    static_assert(classify({}).source == WeaponClassificationSource::None);

    return 0;
}
