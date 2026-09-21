#pragma once

#include "physics-interaction/timing/GameFrameTimingPolicy.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>

namespace rock::release_velocity
{
    // Preserve the former five samples at 90 Hz, with the same peak policy,
    // while making the retention interval independent of game/physics cadence.
    inline constexpr double kRetentionSeconds = 5.0 / 90.0;

    inline double sampleTimeSeconds() noexcept
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    [[nodiscard]] inline bool usablePhysicsSample(std::uint64_t source, double capturedAt,
        double notBefore, double now) noexcept
    {
        return source != 0 && std::isfinite(capturedAt) && std::isfinite(now) &&
            capturedAt >= notBefore && now >= capturedAt && now - capturedAt < kRetentionSeconds;
    }

    struct SampleAdmission { bool duplicate{}; bool rebase{}; float deltaSeconds{}; };
    [[nodiscard]] inline SampleAdmission admitControllerSample(std::uint64_t previousFrame,
        const game_frame_timing_policy::GameFrameTiming& timing) noexcept
    {
        if (!timing.sequence || timing.sequence == previousFrame) return { true, false, 0.0f };
        const bool rebase = !timing.valid || timing.discontinuity || timing.menuPaused ||
            (previousFrame && timing.sequence != previousFrame + 1) ||
            !std::isfinite(timing.deltaSeconds) || timing.deltaSeconds <= 0.0f;
        return { false, rebase, rebase ? 0.0f : timing.deltaSeconds };
    }

    template <class Point>
    class History
    {
    public:
        void clear() noexcept { *this = {}; }
        [[nodiscard]] bool needsSource(std::uint64_t source) const noexcept { return source && source > _lastSource; }

        void append(std::uint64_t source, double time, Point linear, Point angular = {}) noexcept
        {
            if (!source || source <= _lastSource || !std::isfinite(time) ||
                !finite(linear) || !finite(angular)) return;
            _lastSource = source;
            _samples[_next] = { time, linear, angular, true };
            _next = (_next + 1) % _samples.size();
        }

        struct Peaks { Point linear{}; Point angular{}; std::size_t count{}; };
        [[nodiscard]] Peaks peaks(double now) const noexcept
        {
            Peaks result{};
            float linearLength = -1.0f, angularLength = -1.0f;
            std::array<const Sample*, 32> ordered{};
            std::size_t linearPeak = 0, angularPeak = 0;
            if (!std::isfinite(now)) return result;
            // Visit oldest first, retaining the established first-wins tie rule.
            for (std::size_t n = 0; n < _samples.size(); ++n) {
                const auto& sample = _samples[(_next + n) % _samples.size()];
                const double age = now - sample.time;
                if (!sample.valid || age < 0.0 || age + 1.0e-9 >= kRetentionSeconds) continue;
                ordered[result.count] = &sample;
                const float l = lengthSquared(sample.linear), a = lengthSquared(sample.angular);
                if (l > linearLength) { linearPeak = result.count; linearLength = l; }
                if (a > angularLength) { angularPeak = result.count; angularLength = a; }
                ++result.count;
            }
            const auto smoothPeak = [&](std::size_t peak, bool angular) -> Point {
                if (!result.count) return {};
                const auto value = [&](std::size_t index) { return angular ? ordered[index]->angular : ordered[index]->linear; };
                const auto center = value(peak);
                // The old peak used its two neighbours at 90 Hz. Sample that
                // same time span at other cadences, without extrapolating ends.
                constexpr double radius = 1.0 / 90.0;
                const double before = ordered[peak]->time - radius, after = ordered[peak]->time + radius;
                if (before < ordered[0]->time - 1.0e-9 || after > ordered[result.count - 1]->time + 1.0e-9) return center;
                const auto at = [&](double time) {
                    for (std::size_t i = 1; i < result.count; ++i) {
                        if (ordered[i]->time + 1.0e-9 < time) continue;
                        const auto a = value(i - 1), b = value(i);
                        const double interval = ordered[i]->time - ordered[i - 1]->time;
                        const float t = interval > 0.0 ? static_cast<float>(std::clamp((time - ordered[i - 1]->time) / interval, 0.0, 1.0)) : 1.0f;
                        return Point{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
                    }
                    return value(result.count - 1);
                };
                const auto a = at(before), b = at(after);
                return Point{ (a.x + center.x + b.x) / 3.0f, (a.y + center.y + b.y) / 3.0f, (a.z + center.z + b.z) / 3.0f };
            };
            result.linear = smoothPeak(linearPeak, false);
            result.angular = smoothPeak(angularPeak, true);
            return result;
        }

    private:
        static bool finite(Point p) noexcept { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
        static float lengthSquared(Point p) noexcept { return p.x * p.x + p.y * p.y + p.z * p.z; }
        struct Sample { double time{}; Point linear{}; Point angular{}; bool valid{}; };
        // Bounded storage with ample room above supported display cadences.
        std::array<Sample, 32> _samples{};
        std::size_t _next{};
        std::uint64_t _lastSource{};
    };
}
