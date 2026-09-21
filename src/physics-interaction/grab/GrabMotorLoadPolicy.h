#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::grab_motor_load
{
    inline constexpr std::size_t kAxisCount = 6;
    using Axes = std::array<float, kAxisCount>;

    struct Load
    {
        Axes impulse{};
        Axes recoveryState{};
        Axes averageEffort{};
        Axes netUtilization{};
        std::uint8_t enabledAxes = 0;
        bool valid = false;
    };

    // FO4VR exports one 8-byte result then advances TWO result slots per
    // motor. Order: angular 0/1/2, linear 0/1/2. The second float is native
    // recovery state, not another impulse. See the 2026-09-20 native audit.
    inline Load decode(const std::array<float, 24>& runtime, const Axes& limits, float seconds)
    {
        Load result{};
        if (!std::isfinite(seconds) || seconds <= 0.000001f || seconds > 0.25f) return result;
        for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
            const float impulse = runtime[axis * 4];
            const float recovery = runtime[axis * 4 + 1];
            const float limit = limits[axis];
            if (!std::isfinite(impulse) || !std::isfinite(recovery) ||
                !std::isfinite(limit) || limit < 0.0f) return {};
            result.impulse[axis] = impulse;
            result.recoveryState[axis] = recovery;
            result.averageEffort[axis] = impulse / seconds;
            if (!std::isfinite(result.averageEffort[axis])) return {};
            if (limit > 0.0f) {
                result.enabledAxes |= static_cast<std::uint8_t>(1u << axis);
                result.netUtilization[axis] = std::abs(result.averageEffort[axis]) / limit;
                if (!std::isfinite(result.netUtilization[axis])) return {};
            } else if (impulse != 0.0f) {
                // A zero-limit row cannot deliver new impulse. Reject stale or
                // mismatched command/result data rather than dividing by zero.
                return {};
            }
        }
        result.valid = true;
        return result;
    }

    inline bool matchingSolve(std::uint64_t before, std::uint64_t after)
    {
        return after != 0 && after - 1 == before;
    }
}
