#pragma once

#include "api/ROCKProviderApi.h"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace rock::provider_lease_policy
{
    [[nodiscard]] inline constexpr std::uint32_t clampLeaseFrames(
        const std::uint32_t requestedFrames,
        const std::uint32_t maximumFrames)
    {
        return requestedFrames < maximumFrames ? requestedFrames : maximumFrames;
    }

    [[nodiscard]] inline constexpr std::uint64_t exclusiveExpiryFrame(
        const std::uint64_t publicationFrame,
        const std::uint32_t leaseFrames)
    {
        constexpr auto maximumFrame =
            (std::numeric_limits<std::uint64_t>::max)();
        return maximumFrame - publicationFrame < leaseFrames ?
            maximumFrame :
            publicationFrame + leaseFrames;
    }

    [[nodiscard]] inline constexpr bool isActive(
        const std::uint64_t currentFrame,
        const std::uint64_t expiresAfterFrame)
    {
        return expiresAfterFrame != 0 && currentFrame < expiresAfterFrame;
    }

    [[nodiscard]] inline constexpr std::uint32_t remainingFrames(
        const std::uint64_t currentFrame,
        const std::uint64_t expiresAfterFrame)
    {
        if (!isActive(currentFrame, expiresAfterFrame)) {
            return 0;
        }
        const auto remaining = expiresAfterFrame - currentFrame;
        constexpr auto maximumResult =
            (std::numeric_limits<std::uint32_t>::max)();
        return remaining > maximumResult ?
            maximumResult :
            static_cast<std::uint32_t>(remaining);
    }

    template <class IsActive, class GenerationChanged, class ExpiresAfter,
              class Revoke>
    [[nodiscard]] bool pruneExpiredSlots(
        const std::size_t slotCount,
        const std::uint64_t frameIndex,
        IsActive&& isActive,
        GenerationChanged&& generationChanged,
        ExpiresAfter&& expiresAfter,
        Revoke&& revoke)
    {
        bool changed = false;
        for (std::size_t index = 0; index < slotCount; ++index) {
            if (!isActive(index)) {
                continue;
            }

            const bool staleGeneration = generationChanged(index);
            if (!staleGeneration && provider_lease_policy::isActive(
                    frameIndex,
                    expiresAfter(index))) {
                continue;
            }

            const auto reason = staleGeneration ?
                rock::provider::RockProviderSuppressionInvalidationReasonV1::
                    GenerationChanged :
                rock::provider::RockProviderSuppressionInvalidationReasonV1::
                    Expired;
            revoke(index, reason);
            changed = true;
        }
        return changed;
    }
}
