#pragma once

#include <cmath>
#include <cstdint>

namespace rock::loose_weapon_authored_grab_policy
{
    enum class Role : std::uint8_t
    {
        None,
        Firing,
        Support
    };
    enum class AcquisitionSource : std::uint8_t
    {
        Close,
        PullCatch,
        ForcedArrival,
        Transfer
    };

    [[nodiscard]] constexpr AcquisitionSource acquisitionSource(bool pullCatch, bool forcedArrival,
        bool transfer) noexcept
    {
        if (transfer)
            return AcquisitionSource::Transfer;
        if (pullCatch)
            return AcquisitionSource::PullCatch;
        return forcedArrival ? AcquisitionSource::ForcedArrival : AcquisitionSource::Close;
    }

    // The station a free weapon takes without palm contact: a close grab the
    // one nearest the palm, a far pull or forced arrival the firing grip.
    [[nodiscard]] constexpr Role freeWeaponRole(AcquisitionSource source, Role nearestRole) noexcept
    {
        return source == AcquisitionSource::Close ? nearestRole : Role::Firing;
    }

    [[nodiscard]] constexpr bool useAuthoredArrival(AcquisitionSource source, bool poseAvailable, bool throwable,
        bool pinchSelection) noexcept
    {
        return source != AcquisitionSource::Close && poseAvailable && !throwable && !pinchSelection;
    }

    enum class Arrangement : std::uint8_t
    {
        Pending,
        Separated,
        Close,
        OneHanded
    };

    [[nodiscard]] inline Arrangement arrangement(bool firingReady, bool supportReady, bool supportAbsent,
        float seatSeparation, float closeRadius)
    {
        if (!firingReady)
            return Arrangement::Pending;
        if (supportReady && std::isfinite(seatSeparation) && seatSeparation >= 0.0f && std::isfinite(closeRadius) &&
            closeRadius >= 0.0f) {
            return seatSeparation <= closeRadius ? Arrangement::Close : Arrangement::Separated;
        }
        return supportAbsent ? Arrangement::OneHanded : Arrangement::Pending;
    }

    [[nodiscard]] constexpr bool sharedFiringZone(Arrangement value) noexcept
    {
        return value == Arrangement::Close || value == Arrangement::OneHanded;
    }

    // freeRole: freeWeaponRole for this acquisition.
    [[nodiscard]] constexpr Role acquisitionRole(bool touching, bool transfer, Role transferRole, Arrangement layout,
        bool peerHolding, Role zoneRole, Role freeRole) noexcept
    {
        if (transfer)
            return transferRole;
        // A free weapon taken without palm contact seats at freeRole; a touch
        // grab keeps the zone it lands in, or the mesh grab outside every
        // zone. A second hand on an already held weapon keeps its selected
        // station even before palm contact.
        if (!touching && !peerHolding)
            return freeRole;
        if (sharedFiringZone(layout) && zoneRole != Role::None) {
            return peerHolding ? Role::Support : Role::Firing;
        }
        return zoneRole;
    }

    [[nodiscard]] constexpr bool dynamicSupportAcquisition(Role role, bool peerHolding, Arrangement layout,
        bool transfer) noexcept
    {
        return role == Role::Support && peerHolding && layout == Arrangement::OneHanded && !transfer;
    }

    // Surface evidence may still change touch admission. Preselect only when
    // both possible touch outcomes choose the same authored station, or when
    // an arrival/transfer already determines it independently of touch.
    [[nodiscard]] constexpr Role surfaceIndependentAcquisitionRole(bool programmaticArrival, bool transfer,
        Role transferRole, Arrangement layout, bool peerHolding, Role zoneRole, Role freeRole) noexcept
    {
        const auto role = acquisitionRole(false, transfer, transferRole, layout, peerHolding, zoneRole, freeRole);
        if ((!programmaticArrival &&
                role != acquisitionRole(true, transfer, transferRole, layout, peerHolding, zoneRole, freeRole)) ||
            dynamicSupportAcquisition(role, peerHolding, layout, transfer))
            return Role::None;
        return role;
    }

    // Outside both authored seats the existing mesh grab remains authoritative.
    // Compare actual distances, not radii, when the two zones overlap.
    [[nodiscard]] inline Role select(bool firingEligible, float firingDistance, bool supportEligible,
        float supportDistance)
    {
        firingEligible = firingEligible && std::isfinite(firingDistance) && firingDistance >= 0.0f;
        supportEligible = supportEligible && std::isfinite(supportDistance) && supportDistance >= 0.0f;
        if (supportEligible && (!firingEligible || supportDistance < firingDistance)) {
            return Role::Support;
        }
        return firingEligible ? Role::Firing : Role::None;
    }
}
