#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace rock::provider_collider_visualization
{
    struct Snapshot
    {
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
    };

    struct Invalidation
    {
        std::uint64_t ownerToken{ 0 };
        provider::RockProviderSuppressionInvalidationReasonV1 reason{
            provider::RockProviderSuppressionInvalidationReasonV1::None
        };
    };

    [[nodiscard]] provider::RockProviderResultV1 set(
        std::uint64_t ownerToken,
        const provider::RockProviderColliderVisualizationRequestV1& request,
        std::uint64_t frameIndex);
    void prune(
        std::uint64_t frameIndex,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        std::uint64_t weaponGenerationKey,
        bool weaponBodyCurrent,
        Invalidation& outInvalidation);
    void clear(std::uint64_t ownerToken);
    void clearAll(
        Invalidation& outInvalidation,
        provider::RockProviderSuppressionInvalidationReasonV1 reason);
    void clearAll();
    [[nodiscard]] bool hasOverride();
    [[nodiscard]] bool copySnapshot(Snapshot& outSnapshot);
}
