#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include <d3d11.h>
#include <wrl/client.h>

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
        std::size_t maxGpuBytes{ debug_overlay_runtime::kDefaultMaxShapeCacheBytes };
    };

    struct GpuShape
    {
        Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer;
        std::uint32_t indexCount{ 0 };
        debug_overlay_policy::ShapeDecodeMode decodeMode{ debug_overlay_policy::ShapeDecodeMode::Detailed };
        int shapeType{ -1 };
        std::size_t approximateBytes{ 0 };
    };

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
        std::shared_ptr<const GpuShape> shape;
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
        std::size_t approximateGpuBytes{ 0 };
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
            ID3D11Device* device, std::uint32_t maxUploads, const PipelineLimits& limits);
        [[nodiscard]] PipelineStats stats() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
}
