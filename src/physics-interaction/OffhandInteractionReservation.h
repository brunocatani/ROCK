#pragma once

#include <cstdint>

#include "api/ROCKProviderApi.h"

namespace frik::rock
{
    enum class OffhandInteractionReservation : std::uint8_t
    {
        Normal,
        ReloadReserved,
        ReloadPoseOverride,
    };

    namespace offhand_interaction_reservation
    {
        inline OffhandInteractionReservation fromProvider(::rock::provider::RockProviderOffhandReservation reservation)
        {
            switch (reservation) {
            case ::rock::provider::RockProviderOffhandReservation::ReloadReserved:
                return OffhandInteractionReservation::ReloadReserved;
            case ::rock::provider::RockProviderOffhandReservation::ReloadPoseOverride:
                return OffhandInteractionReservation::ReloadPoseOverride;
            case ::rock::provider::RockProviderOffhandReservation::Normal:
            default:
                return OffhandInteractionReservation::Normal;
            }
        }

        inline bool allowsSupportGrip(OffhandInteractionReservation reservation)
        {
            return reservation == OffhandInteractionReservation::Normal;
        }
    }
}
