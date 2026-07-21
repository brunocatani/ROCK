#pragma once

#include <cstddef>
#include <cstdint>

#include "physics-interaction/debug/DebugOverlayPolicy.h"

namespace rock::debug_overlay_runtime
{
    inline constexpr std::uint32_t kDefaultMaxShapeCapturesPerFrame = 32;
    inline constexpr std::uint32_t kMaxShapeCapturesPerFrame = 32;
    inline constexpr std::uint32_t kDefaultMaxShapeQueuedJobs = 64;
    inline constexpr std::uint32_t kMaxShapeQueuedJobs = 256;
    inline constexpr std::uint32_t kDefaultMaxShapeCompletedJobs = 64;
    inline constexpr std::uint32_t kMaxShapeCompletedJobs = 256;
    inline constexpr std::uint32_t kDefaultMaxShapeUploadsPerFrame = 2;
    inline constexpr std::uint32_t kMaxShapeUploadsPerFrame = 16;
    inline constexpr std::size_t kDefaultMaxShapeCacheBytes = 64u * 1024u * 1024u;
    inline constexpr std::size_t kMinShapeCacheBytes = 1u * 1024u * 1024u;
    inline constexpr std::size_t kMaxShapeCacheBytes = 512u * 1024u * 1024u;
    inline constexpr std::uint32_t kDefaultMaxBodyInstances = 171;
    inline constexpr std::uint32_t kMaxBodyInstances = 171;
    inline constexpr std::uint32_t kDefaultMaxTextVertices = 131072;
    inline constexpr std::uint32_t kMaxTextVertices = 131072;

    struct RequestedLimits
    {
        int maxShapeCapturesPerFrame{ static_cast<int>(kDefaultMaxShapeCapturesPerFrame) };
        int maxConvexSupportVertices{ 8 };
        int maxCompoundChildren{ static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundChildren) };
        int maxCompoundDepth{ static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundDepth) };
        int maxShapeQueuedJobs{ static_cast<int>(kDefaultMaxShapeQueuedJobs) };
        int maxShapeCompletedJobs{ static_cast<int>(kDefaultMaxShapeCompletedJobs) };
        int maxShapeUploadsPerFrame{ static_cast<int>(kDefaultMaxShapeUploadsPerFrame) };
        int maxShapeCacheEntries{ static_cast<int>(debug_overlay_policy::kDefaultShapeCacheBudget) };
        int maxShapeCacheBytes{ static_cast<int>(kDefaultMaxShapeCacheBytes) };
        int maxBodyInstances{ static_cast<int>(kDefaultMaxBodyInstances) };
        int maxLineVertices{ static_cast<int>(debug_overlay_policy::kDefaultLineVertexBudget) };
        int maxTextVertices{ static_cast<int>(kDefaultMaxTextVertices) };
    };

    struct Limits
    {
        std::uint32_t maxShapeCapturesPerFrame{ kDefaultMaxShapeCapturesPerFrame };
        std::uint32_t maxConvexSupportVertices{ 8 };
        std::uint32_t maxCompoundChildren{ debug_overlay_policy::kDefaultMaxCompoundChildren };
        std::uint32_t maxCompoundDepth{ debug_overlay_policy::kDefaultMaxCompoundDepth };
        std::uint32_t maxShapeQueuedJobs{ kDefaultMaxShapeQueuedJobs };
        std::uint32_t maxShapeCompletedJobs{ kDefaultMaxShapeCompletedJobs };
        std::uint32_t maxShapeUploadsPerFrame{ kDefaultMaxShapeUploadsPerFrame };
        std::uint32_t maxShapeCacheEntries{ debug_overlay_policy::kDefaultShapeCacheBudget };
        std::size_t maxShapeCacheBytes{ kDefaultMaxShapeCacheBytes };
        std::uint32_t maxBodyInstances{ kDefaultMaxBodyInstances };
        std::uint32_t maxLineVertices{ debug_overlay_policy::kDefaultLineVertexBudget };
        std::uint32_t maxTextVertices{ kDefaultMaxTextVertices };
    };

    constexpr std::uint32_t clampNonNegative(int requested, std::uint32_t maximum) noexcept
    {
        if (requested <= 0) {
            return 0;
        }
        if (requested > static_cast<int>(maximum)) {
            return maximum;
        }
        return static_cast<std::uint32_t>(requested);
    }

    constexpr std::size_t clampShapeCacheBytes(int requested) noexcept
    {
        if (requested < static_cast<int>(kMinShapeCacheBytes)) {
            return kMinShapeCacheBytes;
        }
        if (requested > static_cast<int>(kMaxShapeCacheBytes)) {
            return kMaxShapeCacheBytes;
        }
        return static_cast<std::size_t>(requested);
    }

    constexpr Limits sanitize(const RequestedLimits& requested) noexcept
    {
        Limits limits{};
        limits.maxShapeCapturesPerFrame = clampNonNegative(requested.maxShapeCapturesPerFrame, kMaxShapeCapturesPerFrame);
        limits.maxConvexSupportVertices = debug_overlay_policy::clampMaxConvexSupportVertices(requested.maxConvexSupportVertices);
        limits.maxCompoundChildren = debug_overlay_policy::clampMaxCompoundChildren(requested.maxCompoundChildren);
        limits.maxCompoundDepth = debug_overlay_policy::clampMaxCompoundDepth(requested.maxCompoundDepth);
        limits.maxShapeQueuedJobs = clampNonNegative(requested.maxShapeQueuedJobs, kMaxShapeQueuedJobs);
        limits.maxShapeCompletedJobs = clampNonNegative(requested.maxShapeCompletedJobs, kMaxShapeCompletedJobs);
        if (limits.maxShapeCompletedJobs > limits.maxShapeQueuedJobs) {
            limits.maxShapeCompletedJobs = limits.maxShapeQueuedJobs;
        }
        limits.maxShapeUploadsPerFrame = clampNonNegative(requested.maxShapeUploadsPerFrame, kMaxShapeUploadsPerFrame);
        limits.maxShapeCacheEntries = debug_overlay_policy::clampShapeCacheBudget(requested.maxShapeCacheEntries);
        limits.maxShapeCacheBytes = clampShapeCacheBytes(requested.maxShapeCacheBytes);
        limits.maxBodyInstances = clampNonNegative(requested.maxBodyInstances, kMaxBodyInstances);
        limits.maxLineVertices = debug_overlay_policy::clampLineVertexBudget(requested.maxLineVertices);
        limits.maxTextVertices = clampNonNegative(requested.maxTextVertices, kMaxTextVertices);
        return limits;
    }
}
