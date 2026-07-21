#pragma once

#include <cstdint>

namespace rock::debug_overlay_stats
{
    struct RuntimeStats
    {
        std::uint32_t bodyEntries{ 0 };
        std::uint32_t bodiesDrawn{ 0 };
        std::uint32_t bodyMeshBinds{ 0 };
        std::uint32_t bodyDrawCalls{ 0 };
        std::uint32_t bodyInstanceMaps{ 0 };
        std::uint32_t bodyInstanceRejects{ 0 };
        std::uint32_t shapeCacheHits{ 0 };
        std::uint32_t shapeCacheMisses{ 0 };
        std::uint32_t shapeCaptures{ 0 };
        std::uint32_t shapeCaptureDeferrals{ 0 };
        std::uint32_t shapePendingProxies{ 0 };
        std::uint32_t shapeUploadsProcessed{ 0 };
        std::uint32_t shapeUploadsCompleted{ 0 };
        std::uint32_t shapeUploadFailures{ 0 };
        std::uint32_t shapeProxyFallbacks{ 0 };
        std::uint32_t unsupportedShapeProxies{ 0 };
        std::uint32_t unsupportedShapeSkips{ 0 };
        std::uint32_t bodyExtractFailures{ 0 };
        std::uint32_t lineVertices{ 0 };
        std::uint32_t lineLogicalLines{ 0 };
        std::uint32_t lineDrawCalls{ 0 };
        std::uint32_t lineMapFailures{ 0 };
        std::uint32_t lineBudgetRejects{ 0 };
        std::uint32_t rtvCacheHits{ 0 };
        std::uint32_t rtvCacheMisses{ 0 };
        std::uint32_t textVertices{ 0 };
        std::uint32_t textDrawCalls{ 0 };
        std::uint32_t textVertexTruncations{ 0 };
        std::uint32_t textRejectedVertices{ 0 };
        std::uint32_t textMapFailures{ 0 };
    };
}
