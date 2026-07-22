#pragma once

#include "api/ROCKProviderApi.h"

#include <array>
#include <cstdint>

namespace rock::provider_debug_overlay
{
    // Normal frame polling uses an atomic empty fast path. Publication,
    // unregister, and provider-loss cleanup may arrive through different API
    // lifecycles, so populated snapshots are copied under the private store
    // lock before the renderer sees them.
    struct Snapshot
    {
        std::array<provider::RockProviderDebugOverlayLineV1,
            provider::ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1>
            lines{};
        std::array<provider::RockProviderDebugOverlayTextV1,
            provider::ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1>
            textEntries{};
        std::uint32_t lineCount{ 0 };
        std::uint32_t textCount{ 0 };
    };

    struct InvalidatedPublisher
    {
        std::uint64_t ownerToken{ 0 };
        provider::RockProviderSuppressionInvalidationReasonV1 reason{
            provider::RockProviderSuppressionInvalidationReasonV1::None
        };
    };

    struct PruneResult
    {
        std::array<InvalidatedPublisher,
            provider::ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1>
            publishers{};
        std::uint32_t count{ 0 };
    };

    [[nodiscard]] provider::RockProviderResultV1 publish(
        std::uint64_t ownerToken,
        const provider::RockProviderDebugOverlayPublicationV1& publication,
        std::uint64_t frameIndex);
    void prune(
        std::uint64_t frameIndex,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        PruneResult& outResult);
    void clear(std::uint64_t ownerToken);
    void clearAll(
        PruneResult& outResult,
        provider::RockProviderSuppressionInvalidationReasonV1 reason);
    void clearAll();
    [[nodiscard]] bool hasContent();
    void copySnapshot(Snapshot& outSnapshot);
}
