#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    [[nodiscard]] bool clearHandVisualAuthoritySlotLocked(
        HandVisualAuthoritySlot& slot,
        bool releaseSlot);
    [[nodiscard]] bool clearHandVisualAuthorityForOwner(
        std::uint64_t ownerToken,
        RockProviderHand hand,
        bool releaseSlots);
    void pruneHandVisualAuthorityLocked(std::uint64_t frameIndex);
    void clearAnimationPhaseCallbacksForOwnerLocked(std::uint64_t ownerToken);
    void clearNativeAnimationRuntimePublicationLocked();
    void clearNativeAnimationRuntimePublicationForOwner(
        std::uint64_t ownerToken);
    void pruneExpiredNativeAnimationRuntimePublicationLocked(
        std::uint64_t frameIndex);
    void publishNativeAnimationAuthorityAggregateLocked();
    void pruneExpiredNativeAnimationAuthorityLocked(std::uint64_t frameIndex);
    void clearNativeAnimationAuthorityForOwnerLocked(
        std::uint64_t ownerToken,
        bool publishAggregate = true);
    void clearOwnerStateAfterCallbackFault(std::uint64_t ownerToken);
}
