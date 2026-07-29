#include <iostream>

#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"

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
}

int main()
{
    using namespace rock;
    using namespace debug_overlay_runtime;

    bool passed = true;
    const auto defaults = sanitize(RequestedLimits{});
    passed &= expect(defaults.maxShapeCapturesPerFrame == 32, "Shape capture default drifted from the effective historical cap.");
    passed &= expect(defaults.maxConvexSupportVertices == 8, "Convex support default is not aligned with the shipped INIs.");
    passed &= expect(defaults.maxCompoundChildren == 256 && defaults.maxCompoundDepth == 4,
        "Compound fidelity defaults changed.");
    passed &= expect(defaults.maxShapeQueuedJobs == 64 && defaults.maxShapeCompletedJobs == 64,
        "Shape queue defaults changed.");
    passed &= expect(defaults.maxShapeUploadsPerFrame == 2 && defaults.maxShapeCacheEntries == 512 &&
                         defaults.maxShapeCacheBytes == 64u * 1024u * 1024u,
        "Shape upload/cache defaults changed.");
    passed &= expect(defaults.maxBodyInstances == 171 && defaults.maxLineVertices == 8192 && defaults.maxTextVertices == 131072,
        "Renderer stream defaults changed.");

    RequestedLimits excessive{};
    excessive.maxShapeCapturesPerFrame = 1000;
    excessive.maxConvexSupportVertices = 1000;
    excessive.maxCompoundChildren = 10000;
    excessive.maxCompoundDepth = 100;
    excessive.maxShapeQueuedJobs = 5;
    excessive.maxShapeCompletedJobs = 200;
    excessive.maxShapeUploadsPerFrame = 100;
    excessive.maxShapeCacheEntries = 10000;
    excessive.maxShapeCacheBytes = 1'000'000'000;
    excessive.maxBodyInstances = 10000;
    excessive.maxLineVertices = 1'000'000;
    excessive.maxTextVertices = 1'000'000;
    const auto capped = sanitize(excessive);
    passed &= expect(capped.maxShapeCapturesPerFrame == kMaxShapeCapturesPerFrame &&
                         capped.maxConvexSupportVertices == debug_overlay_policy::kMaxDetailedConvexSupportVertices,
        "Capture/convex hard caps were bypassed.");
    passed &= expect(capped.maxCompoundChildren == debug_overlay_policy::kMaxCompoundChildren &&
                         capped.maxCompoundDepth == debug_overlay_policy::kMaxCompoundDepth,
        "Compound hard caps were bypassed.");
    passed &= expect(capped.maxShapeQueuedJobs == 5 && capped.maxShapeCompletedJobs == 5,
        "Completed work was not constrained by total queued backlog.");
    passed &= expect(capped.maxShapeUploadsPerFrame == kMaxShapeUploadsPerFrame &&
                         capped.maxShapeCacheEntries == debug_overlay_policy::kMaxShapeCacheBudget &&
                         capped.maxShapeCacheBytes == kMaxShapeCacheBytes,
        "Upload/cache hard caps were bypassed.");
    passed &= expect(capped.maxBodyInstances == kMaxBodyInstances &&
                         capped.maxLineVertices == debug_overlay_policy::kMaxLineVertexBudget &&
                         capped.maxTextVertices == kMaxTextVertices,
        "GPU stream hard caps were bypassed.");

    RequestedLimits disabled{};
    disabled.maxShapeCapturesPerFrame = -1;
    disabled.maxShapeQueuedJobs = 0;
    disabled.maxShapeCompletedJobs = -1;
    disabled.maxShapeUploadsPerFrame = 0;
    disabled.maxBodyInstances = 0;
    disabled.maxLineVertices = -1;
    disabled.maxTextVertices = 0;
    disabled.maxShapeCacheBytes = 0;
    const auto boundedLow = sanitize(disabled);
    passed &= expect(boundedLow.maxShapeCapturesPerFrame == 0 && boundedLow.maxShapeQueuedJobs == 0 &&
                         boundedLow.maxShapeCompletedJobs == 0 && boundedLow.maxShapeUploadsPerFrame == 0,
        "Disable-capable work budgets did not clamp to zero.");
    passed &= expect(boundedLow.maxBodyInstances == 0 && boundedLow.maxLineVertices == 0 && boundedLow.maxTextVertices == 0,
        "Disable-capable renderer budgets did not clamp to zero.");
    passed &= expect(boundedLow.maxShapeCacheBytes == kMinShapeCacheBytes,
        "Shape cache byte budget did not retain its safe minimum allocation policy.");

    const auto baseKey = debug_overlay_policy::makeShapeDecodeSettingsKey(8, true, 256, 4);
    passed &= expect(baseKey != debug_overlay_policy::makeShapeDecodeSettingsKey(8, true, 255, 4) &&
                         baseKey != debug_overlay_policy::makeShapeDecodeSettingsKey(8, true, 256, 5),
        "Compound limits do not participate in shape-cache invalidation identity.");

    return passed ? 0 : 1;
}
