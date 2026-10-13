#pragma once

#include <cstdint>

namespace rock::debug_overlay_stats
{
    struct RuntimeStats
    {
        std::uint32_t bodyEntries{ 0 };
        std::uint32_t bodiesDrawn{ 0 };
        std::uint32_t bodyInstanceRejects{ 0 };
        std::uint32_t bodyInvalidRejects{ 0 };
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
        std::uint32_t lineBudgetRejects{ 0 };
        std::uint32_t lineInvalidRejects{ 0 };
        std::uint32_t textTruncations{ 0 };
        std::uint32_t textInvalidRejects{ 0 };
        std::uint32_t screenTextSkips{ 0 };
    };
}
