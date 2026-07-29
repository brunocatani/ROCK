#include <chrono>
#include <iostream>
#include <thread>

#include "physics-interaction/debug/DebugOverlayShapePipeline.h"

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << message << '\n';
            return false;
        }
        return true;
    }

    rock::debug_overlay_shape::ShapeRecipe sphereRecipe()
    {
        rock::debug_overlay_shape::ShapeRecipe recipe{};
        recipe.kind = rock::debug_overlay_shape::ShapeRecipe::Kind::Sphere;
        recipe.shapeType = 2;
        recipe.settings.havokToGameScale = 10.0f;
        recipe.settings.maxConvexSupportVertices = 32;
        recipe.convexRadius = 0.5f;
        recipe.valid = true;
        return recipe;
    }

    bool waitForCompleted(rock::debug_overlay_shape::ShapePipeline& pipeline, std::size_t count)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            if (pipeline.stats().completedJobs >= count) {
                return true;
            }
            std::this_thread::yield();
        }
        return false;
    }

    bool waitForCompletedDrop(rock::debug_overlay_shape::ShapePipeline& pipeline)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto stats = pipeline.stats();
            if (stats.completedQueueDrops > 0 && stats.activeJobs == 0 && stats.queuedJobs == 0) {
                return true;
            }
            std::this_thread::yield();
        }
        return false;
    }
}

int main()
{
    using namespace rock::debug_overlay_shape;
    bool passed = true;
    ShapePipeline pipeline;
    passed &= expect(pipeline.initialize(), "Worker initialization failed.");
    passed &= expect(pipeline.stats().running, "Pipeline did not report its running state.");

    PipelineLimits limits{};
    limits.maxQueuedJobs = 2;
    limits.maxCompletedJobs = 2;
    limits.maxCacheEntries = 2;
    limits.maxGpuBytes = 1024 * 1024;

    const ShapeKey firstKey{ 0x1000, 0xA1 };
    const auto first = pipeline.reserve(firstKey, limits);
    passed &= expect(first.status == ReserveStatus::Reserved && first.reservation.valid, "First recipe was not reserved.");
    passed &= expect(pipeline.reserve(firstKey, limits).status == ReserveStatus::Pending, "Duplicate reservation was not coalesced.");
    passed &= expect(pipeline.submit(first.reservation, sphereRecipe()), "Reserved recipe was not queued.");
    passed &= expect(waitForCompleted(pipeline, 1), "Worker did not publish a completed CPU mesh.");
    passed &= expect(pipeline.lookup(firstKey).state == CacheState::Pending, "CPU completion became Ready before render-thread upload.");

    const auto generationBeforeClear = pipeline.stats().generation;
    pipeline.invalidate();
    const auto afterClear = pipeline.stats();
    passed &= expect(afterClear.generation == generationBeforeClear + 1, "Invalidation did not advance the generation token.");
    passed &= expect(afterClear.entries == 0 && afterClear.queuedJobs == 0 && afterClear.completedJobs == 0,
        "Invalidation retained stale cache or queue state.");
    passed &= expect(pipeline.lookup(firstKey).state == CacheState::Missing, "Invalidated key remained visible.");

    const ShapeKey secondKey{ 0x2000, 0xB2 };
    const ShapeKey thirdKey{ 0x3000, 0xC3 };
    const ShapeKey fourthKey{ 0x4000, 0xD4 };
    const auto second = pipeline.reserve(secondKey, limits);
    const auto third = pipeline.reserve(thirdKey, limits);
    passed &= expect(second.status == ReserveStatus::Reserved && third.status == ReserveStatus::Reserved,
        "Bounded queue did not admit its configured capacity.");
    passed &= expect(pipeline.reserve(fourthKey, limits).status == ReserveStatus::QueueFull,
        "Queue admitted work beyond its configured backlog.");
    pipeline.markUnsupported(second.reservation);
    pipeline.markUnsupported(third.reservation);
    passed &= expect(pipeline.lookup(secondKey).state == CacheState::Unsupported,
        "Failed capture did not enter the Unsupported cache state.");

    pipeline.invalidate();
    PipelineLimits completedLimit = limits;
    completedLimit.maxCompletedJobs = 1;
    const auto completedFirst = pipeline.reserve(firstKey, completedLimit);
    const auto completedSecond = pipeline.reserve(secondKey, completedLimit);
    passed &= expect(completedFirst.status == ReserveStatus::Reserved && completedSecond.status == ReserveStatus::Reserved,
        "Completed-queue test reservations failed.");
    passed &= expect(pipeline.submit(completedFirst.reservation, sphereRecipe()), "First completed-queue job was not submitted.");
    passed &= expect(pipeline.submit(completedSecond.reservation, sphereRecipe()), "Second completed-queue job was not submitted.");
    passed &= expect(waitForCompletedDrop(pipeline), "Completed queue did not enforce its independent bound.");
    const auto completedStats = pipeline.stats();
    passed &= expect(completedStats.completedJobs == 1 && completedStats.completedQueueDrops == 1,
        "Completed queue retained work beyond its configured capacity.");

    pipeline.invalidate();
    const auto lruFirst = pipeline.reserve(firstKey, limits);
    const auto lruSecond = pipeline.reserve(secondKey, limits);
    pipeline.markUnsupported(lruFirst.reservation);
    pipeline.markUnsupported(lruSecond.reservation);
    (void)pipeline.lookup(firstKey);
    const auto lruThird = pipeline.reserve(thirdKey, limits);
    passed &= expect(lruThird.status == ReserveStatus::Reserved, "LRU cache did not make room for a new reservation.");
    passed &= expect(pipeline.lookup(firstKey).state == CacheState::Unsupported,
        "LRU cache evicted the most recently used entry.");
    passed &= expect(pipeline.lookup(secondKey).state == CacheState::Missing,
        "LRU cache did not evict the least recently used terminal entry.");
    pipeline.markUnsupported(lruThird.reservation);

    pipeline.shutdown();
    const auto stopped = pipeline.stats();
    passed &= expect(!stopped.running && stopped.entries == 0 && stopped.queuedJobs == 0 && stopped.completedJobs == 0,
        "Shutdown did not join and clear the worker pipeline.");
    passed &= expect(pipeline.initialize(), "Pipeline could not restart after deterministic shutdown.");
    pipeline.shutdown();

    return passed ? 0 : 1;
}
