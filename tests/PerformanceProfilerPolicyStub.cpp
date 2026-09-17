#include "physics-interaction/performance/PerformanceProfiler.h"

namespace rock::performance_profiler
{
    // Keep the optional wait timer inert in pure policy tests. A separate
    // object lets the profiler integration test link the real implementation.
    ScopedTimer::ScopedTimer(Scope) noexcept {}
    ScopedTimer::~ScopedTimer() = default;
    void ScopedTimer::stop() noexcept {}
}
