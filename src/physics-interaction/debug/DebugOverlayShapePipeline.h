#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/debug/DebugOverlayShapeGeometry.h"

namespace rock::debug_overlay_shape
{
    enum class CacheState : std::uint8_t
    {
        Missing,
        Pending,
        Ready,
        Unsupported
    };

    enum class ReserveStatus : std::uint8_t
    {
        Reserved,
        Ready,
        Pending,
        Unsupported,
        NotRunning,
        QueueFull,
        CacheFull
    };

    struct PipelineLimits
    {
        std::uint32_t maxQueuedJobs{ debug_overlay_runtime::kDefaultMaxShapeQueuedJobs };
        std::uint32_t maxCompletedJobs{ debug_overlay_runtime::kDefaultMaxShapeCompletedJobs };
        std::uint32_t maxCacheEntries{ debug_overlay_policy::kDefaultShapeCacheBudget };
        std::size_t maxBytes{ debug_overlay_runtime::kDefaultMaxShapeCacheBytes };
    };

    // Hands finished meshes to the display host (RPS UI world drawing) and
    // takes them back. Both run on the host's frame-callback thread or, for
    // release, wherever the last cache reference drops.
    struct MeshSink
    {
        bool (*registerMesh)(std::uint64_t meshId, const MeshData& mesh) noexcept{ nullptr };
        void (*releaseMesh)(std::uint64_t meshId) noexcept{ nullptr };
    };

    // A mesh registered with the display host. Releasing the last reference
    // releases the host copy, so eviction and invalidation free it.
    struct RegisteredShape
    {
        RegisteredShape() = default;
        RegisteredShape(const RegisteredShape&) = delete;
        RegisteredShape& operator=(const RegisteredShape&) = delete;
        ~RegisteredShape()
        {
            if (release && meshId != 0) release(meshId);
        }

        std::uint64_t meshId{ 0 };
        void (*release)(std::uint64_t meshId) noexcept{ nullptr };
        std::uint32_t indexCount{ 0 };
        debug_overlay_policy::ShapeDecodeMode decodeMode{ debug_overlay_policy::ShapeDecodeMode::Detailed };
        int shapeType{ -1 };
        std::size_t approximateBytes{ 0 };
    };

    // Ids below this value are reserved for meshes the overlay registers itself.
    inline constexpr std::uint64_t kFirstPipelineMeshId = 0x100;

    struct ShapeReservation
    {
        ShapeKey key{};
        std::uint64_t generation{ 0 };
        std::uint32_t maxCompletedJobs{ 0 };
        bool valid{ false };
    };

    struct ReserveResult
    {
        ReserveStatus status{ ReserveStatus::NotRunning };
        ShapeReservation reservation{};
    };

    struct LookupResult
    {
        CacheState state{ CacheState::Missing };
        std::shared_ptr<const RegisteredShape> shape;
    };

    struct UploadResult
    {
        std::uint32_t processed{ 0 };
        std::uint32_t uploaded{ 0 };
        std::uint32_t stale{ 0 };
        std::uint32_t unsupported{ 0 };
        std::uint32_t failed{ 0 };
    };

    struct PipelineStats
    {
        std::size_t entries{ 0 };
        std::size_t ready{ 0 };
        std::size_t pending{ 0 };
        std::size_t unsupported{ 0 };
        std::size_t reservedJobs{ 0 };
        std::size_t queuedJobs{ 0 };
        std::size_t activeJobs{ 0 };
        std::size_t completedJobs{ 0 };
        std::size_t approximateBytes{ 0 };
        std::uint64_t generation{ 0 };
        std::uint64_t evictions{ 0 };
        std::uint64_t staleResultDrops{ 0 };
        std::uint64_t completedQueueDrops{ 0 };
        bool running{ false };
    };

    class ShapePipeline
    {
    public:
        ShapePipeline();
        ShapePipeline(const ShapePipeline&) = delete;
        ShapePipeline& operator=(const ShapePipeline&) = delete;
        ~ShapePipeline();

        [[nodiscard]] bool initialize();
        void shutdown() noexcept;
        void invalidate() noexcept;
        void trim(const PipelineLimits& limits) noexcept;

        [[nodiscard]] ReserveResult reserve(const ShapeKey& key, const PipelineLimits& limits);
        [[nodiscard]] bool submit(ShapeReservation reservation, ShapeRecipe recipe);
        void markUnsupported(ShapeReservation reservation) noexcept;

        [[nodiscard]] LookupResult lookup(const ShapeKey& key);
        [[nodiscard]] UploadResult processCompletedUploads(
            const MeshSink& sink, std::uint32_t maxUploads, const PipelineLimits& limits);
        [[nodiscard]] PipelineStats stats() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
}
