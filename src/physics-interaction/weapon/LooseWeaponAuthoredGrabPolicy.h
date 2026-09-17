#pragma once

#include <cmath>
#include <cstdint>

namespace rock::loose_weapon_authored_grab_policy
{
    enum class Role : std::uint8_t { None, Firing, Support };
    enum class Arrangement : std::uint8_t { Pending, Separated, Close, OneHanded };

    [[nodiscard]] inline Arrangement arrangement(bool firingReady, bool supportReady,
        bool supportAbsent, float seatSeparation, float closeRadius)
    {
        if (!firingReady) return Arrangement::Pending;
        if (supportReady && std::isfinite(seatSeparation) && seatSeparation >= 0.0f &&
            std::isfinite(closeRadius) && closeRadius >= 0.0f) {
            return seatSeparation <= closeRadius ? Arrangement::Close : Arrangement::Separated;
        }
        return supportAbsent ? Arrangement::OneHanded : Arrangement::Pending;
    }

    [[nodiscard]] constexpr bool sharedFiringZone(Arrangement value) noexcept
    {
        return value == Arrangement::Close || value == Arrangement::OneHanded;
    }

    [[nodiscard]] constexpr Role acquisitionRole(bool touching, bool transfer,
        Role transferRole, Arrangement layout, bool peerHolding, Role zoneRole) noexcept
    {
        if (transfer) return transferRole;
        if (!touching) return Role::Firing;
        if (sharedFiringZone(layout) && zoneRole != Role::None) {
            return peerHolding ? Role::Support : Role::Firing;
        }
        return zoneRole;
    }

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
