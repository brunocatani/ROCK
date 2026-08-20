#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    void clearEquippedWeaponHandlingAuthorityForOwnerLocked(
        std::uint64_t ownerToken);
    void pruneExpiredEquippedWeaponHandlingAuthorityLocked(
        std::uint64_t frameIndex);
    void publishOffhandReservationLocked(const OffhandReservationSlot& slot);
    void clearOffhandReservationLocked(
        RockProviderSuppressionInvalidationReasonV1 reason);
    void pruneExpiredOffhandReservationLocked(std::uint64_t frameIndex);
    void pruneExpiredHandInputSuppressionsLocked(std::uint64_t frameIndex);
    void clearHandInputSuppressionsForOwnerLocked(
        std::uint64_t ownerToken,
        RockProviderHand hand,
        RockProviderSuppressionInvalidationReasonV1 reason,
        bool includeInactive = false);
}
