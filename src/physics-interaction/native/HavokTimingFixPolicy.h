#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::havok_timing_fix_policy
{
    constexpr float kDefaultMinPhysicsFrameRate = 70.0f;
    constexpr int kDefaultMaxSubsteps = 3;
    constexpr int kEngineMaxSubsteps = 6;
    constexpr float kMinPhysicsFrameRateFloor = 30.0f;
    constexpr float kMinPhysicsFrameRateCeiling = 240.0f;
    constexpr float kMaxAcceptedFrameDeltaSeconds = 0.25f;

    struct TimingFixInput
    {
        float rawDeltaSeconds = 0.0f;
        float accumulatedDeltaSeconds = 0.0f;
        float minPhysicsFrameRate = kDefaultMinPhysicsFrameRate;
        int maxSubsteps = kDefaultMaxSubsteps;
    };

    struct TimingFixDecision
    {
        bool valid = false;
        float substepDeltaSeconds = 0.0f;
        std::uint32_t substepCount = 0;
        float maxPhysicsFrameSeconds = 0.0f;
        int clampedMaxSubsteps = kDefaultMaxSubsteps;
        const char* reason = "uninitialized";
    };

    inline bool isUsableDeltaSeconds(float value)
    {
        return std::isfinite(value) && value > 0.0f && value <= kMaxAcceptedFrameDeltaSeconds;
    }

    inline float sanitizeMinPhysicsFrameRate(float value)
    {
        if (!std::isfinite(value)) {
            return kDefaultMinPhysicsFrameRate;
        }
        return std::clamp(value, kMinPhysicsFrameRateFloor, kMinPhysicsFrameRateCeiling);
    }

    inline int sanitizeMaxSubsteps(int value)
    {
        return std::clamp(value, 1, kEngineMaxSubsteps);
    }

    inline TimingFixDecision evaluateTimingFix(const TimingFixInput& input)
    {
        const float minPhysicsFrameRate = sanitizeMinPhysicsFrameRate(input.minPhysicsFrameRate);
        const int maxSubsteps = sanitizeMaxSubsteps(input.maxSubsteps);
        const float maxPhysicsFrameSeconds = 1.0f / minPhysicsFrameRate;

        if (!isUsableDeltaSeconds(input.rawDeltaSeconds)) {
            return TimingFixDecision{
                .valid = false,
                .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
                .clampedMaxSubsteps = maxSubsteps,
                .reason = "invalidRawDelta",
            };
        }

        if (!isUsableDeltaSeconds(input.accumulatedDeltaSeconds)) {
            return TimingFixDecision{
                .valid = false,
                .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
                .clampedMaxSubsteps = maxSubsteps,
                .reason = "invalidAccumulatedDelta",
            };
        }

        float substepDeltaSeconds = input.rawDeltaSeconds;
        for (int substeps = 2; substeps <= maxSubsteps && substepDeltaSeconds > maxPhysicsFrameSeconds; ++substeps) {
            substepDeltaSeconds = input.rawDeltaSeconds / static_cast<float>(substeps);
        }

        if (!isUsableDeltaSeconds(substepDeltaSeconds)) {
            return TimingFixDecision{
                .valid = false,
                .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
                .clampedMaxSubsteps = maxSubsteps,
                .reason = "invalidSubstepDelta",
            };
        }

        constexpr float kSubstepCountEpsilon = 1.0e-4f;
        const float rawSubstepCount = std::floor((input.accumulatedDeltaSeconds / substepDeltaSeconds) + kSubstepCountEpsilon);
        const auto substepCount = static_cast<std::uint32_t>(
            std::clamp(rawSubstepCount, 1.0f, static_cast<float>(maxSubsteps)));

        return TimingFixDecision{
            .valid = true,
            .substepDeltaSeconds = substepDeltaSeconds,
            .substepCount = substepCount,
            .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
            .clampedMaxSubsteps = maxSubsteps,
            .reason = "ok",
        };
    }
}
