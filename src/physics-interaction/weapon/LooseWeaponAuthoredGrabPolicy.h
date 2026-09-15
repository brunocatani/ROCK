#pragma once

#include <cmath>
#include <cstdint>

namespace rock::loose_weapon_authored_grab_policy
{
    enum class Role : std::uint8_t { None, Firing, Support };

    // Outside both authored seats the existing mesh grab remains authoritative.
    // Compare actual distances, not radii, when the two zones overlap.
    [[nodiscard]] inline Role select(bool firingEligible, float firingDistance,
        bool supportEligible, float supportDistance)
    {
        firingEligible = firingEligible && std::isfinite(firingDistance) && firingDistance >= 0.0f;
        supportEligible = supportEligible && std::isfinite(supportDistance) && supportDistance >= 0.0f;
        if (supportEligible && (!firingEligible || supportDistance < firingDistance)) {
            return Role::Support;
        }
        return firingEligible ? Role::Firing : Role::None;
    }
}
