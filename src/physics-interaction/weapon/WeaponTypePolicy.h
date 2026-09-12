#pragma once

namespace rock::weapon_type_policy
{
    template <class WeaponType>
    [[nodiscard]] constexpr bool isMelee(const WeaponType type) noexcept
    {
        // WEAPON_TYPE is sequential, not a bitmask. Exact comparisons keep
        // kGun (9) from aliasing melee values through EnumSet::any().
        return type == WeaponType::kOneHandSword ||
               type == WeaponType::kOneHandDagger ||
               type == WeaponType::kOneHandAxe ||
               type == WeaponType::kOneHandMace ||
               type == WeaponType::kTwoHandSword ||
               type == WeaponType::kTwoHandAxe;
    }

    template <class WeaponType>
    [[nodiscard]] constexpr bool isEquippedMelee(const WeaponType type) noexcept
    {
        // Knuckles/power fists own an actual hand-to-hand inventory weapon.
        // An equipped grenade or mine does not own the drawn bare-fist pose.
        return type == WeaponType::kHandToHand || isMelee(type);
    }
}
