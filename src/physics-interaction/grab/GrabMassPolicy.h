#pragma once

#include <algorithm>
#include <cmath>

namespace rock::grab_mass_policy
{
    // Some loose WEAP bodies expose composite inverse-mass data that is heavier
    // than large world clutter. Clamp only while the loose weapon is held; the
    // original packed mass is restored with the rest of the Havok motion state.
    inline constexpr float kLooseWeaponGrabMassCeiling = 50.0f;

    [[nodiscard]] inline float massFromInverseMass(float inverseMass) noexcept
    {
        if (!std::isfinite(inverseMass) || inverseMass <= 0.0f) {
            return 0.0f;
        }
        return 1.0f / inverseMass;
    }

    [[nodiscard]] inline float normalizedLooseWeaponGrabMass(float mass, bool looseWeaponGrab) noexcept
    {
        if (!looseWeaponGrab || !std::isfinite(mass) || mass <= 0.0f) {
            return mass;
        }
        return (std::min)(mass, kLooseWeaponGrabMassCeiling);
    }

    [[nodiscard]] inline bool shouldNormalizeLooseWeaponGrabMass(float mass, bool looseWeaponGrab) noexcept
    {
        return looseWeaponGrab &&
               std::isfinite(mass) &&
               mass > kLooseWeaponGrabMassCeiling;
    }
}
