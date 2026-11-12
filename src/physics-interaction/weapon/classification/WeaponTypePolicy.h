// Exact WEAPON_TYPE comparisons: melee, equipped melee and whether a type
// occupies a hand slot.
#pragma once

namespace rock::weapon_type_policy
{
    template <class WeaponType>
    [[nodiscard]] constexpr bool isMelee(const WeaponType type) noexcept
    {
        // WEAPON_TYPE is sequential, not a bitmask. Exact comparisons keep
        // kGun (9) from aliasing melee values through EnumSet::any().
        return type == WeaponType::kOneHandSword || type == WeaponType::kOneHandDagger ||
            type == WeaponType::kOneHandAxe || type == WeaponType::kOneHandMace || type == WeaponType::kTwoHandSword ||
            type == WeaponType::kTwoHandAxe;
    }

    template <class WeaponType>
    [[nodiscard]] constexpr bool isEquippedMelee(const WeaponType type) noexcept
    {
        // Knuckles/power fists own an actual hand-to-hand inventory weapon.
        // An equipped grenade or mine does not own the drawn bare-fist pose.
        return type == WeaponType::kHandToHand || isMelee(type);
    }

    template <class WeaponType>
    [[nodiscard]] constexpr bool occupiesHandSlot(const WeaponType type) noexcept
    {
        // A weapon native equipment index 0 or 1 presents in a hand: a gun
        // or an equipped melee weapon. Grenades and mines are throwable
        // selection state, never a hand slot.
        return type == WeaponType::kGun || isEquippedMelee(type);
    }
}
