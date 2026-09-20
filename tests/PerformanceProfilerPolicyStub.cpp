#include "physics-interaction/performance/PerformanceProfiler.h"

// Geometry policy tests keep optional diagnostics inert. The profiler's own
// integration executable links its real implementation instead of this object.
namespace rock::performance_profiler
{
    bool enabled() noexcept { return false; }
    ScopedTimer::ScopedTimer(Scope scope) noexcept : _scope(scope) {}
    ScopedTimer::~ScopedTimer() = default;
    void ScopedTimer::stop() noexcept {}
    void observeValue(ValueMetric, std::uint64_t) noexcept {}
}
