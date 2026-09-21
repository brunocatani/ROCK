#pragma once

#include "physics-interaction/performance/PerformanceProfiler.h"

namespace rock::performance_profiler
{
    // Owned by the existing world-listener callback state, under its whole-update
    // lease. No native pointers or TLS attribution survive a callback. Whole
    // update time includes listeners and waits; the collide/solve intervals begin
    // after ROCK's preceding callback and end before its following callback.
    class PhysicsStepProfile
    {
    public:
        void reset() noexcept
        {
            _whole = {};
            _collide = {};
            _solve = {};
            _completedSubsteps = 0;
        }

        void beginUpdate() noexcept
        {
            reset();
            _whole = beginInterval();
        }

        void beginCollide() noexcept
        {
            _solve = {};
            _collide = _whole.startTicks ? beginInterval() : IntervalSample{};
            if (_collide.generation != _whole.generation) _collide = {};
        }

        void endCollide() noexcept { endInterval(Scope::NativePhysicsCollideInterval, _collide); }
        void beginSolve() noexcept
        {
            _solve = _whole.startTicks ? beginInterval() : IntervalSample{};
            if (_solve.generation != _whole.generation) _solve = {};
        }

        void endSolve() noexcept
        {
            if (endInterval(Scope::NativePhysicsSolveInterval, _solve)) ++_completedSubsteps;
        }

        void endUpdate() noexcept
        {
            if (endInterval(Scope::NativePhysicsUpdate, _whole)) {
                observeValue(ValueMetric::PhysicsCompletedSubsteps, _completedSubsteps);
            }
            reset();
        }

    private:
        IntervalSample _whole{}, _collide{}, _solve{};
        std::uint32_t _completedSubsteps{ 0 };
    };
}
