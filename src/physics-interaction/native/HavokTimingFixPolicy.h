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
    constexpr float kMinAcceptedFrameDeltaSeconds = 0.000001f;
    constexpr float kMaxAcceptedFrameDeltaSeconds = 0.25f;

    /*
     * The source interval is measured at ROCK's game-frame hook. It is the
     * same interval used to produce player-attached target segments. The
     * native global multiplier is simulation time scaling; it is unrelated to
     * Calendar::timeScale, which only controls the passage of game-world days.
     */
    struct TimingFixInput
    {
        float sourceDeltaSeconds = 0.0f;
        float globalTimeMultiplier = 1.0f;
        float nativeRemainderDeltaSeconds = 0.0f;
        float nativePreviousRemainderDeltaSeconds = 0.0f;
        float nativeAccumulatedDeltaSeconds = 0.0f;
        float nativeSubstepDeltaSeconds = 0.0f;
        std::uint32_t nativeSubstepCount = 0;
        float minPhysicsFrameRate = kDefaultMinPhysicsFrameRate;
        int maxSubsteps = kDefaultMaxSubsteps;
        bool sourceValid = false;
        bool sourceDiscontinuity = true;
        bool sourcePaused = false;
    };

    struct TimingFixDecision
    {
        bool valid = false;
        float sourceDeltaSeconds = 0.0f;
        float globalTimeMultiplier = 1.0f;
        float coherentDeltaSeconds = 0.0f;
        float substepDeltaSeconds = 0.0f;
        float simulatedDeltaSeconds = 0.0f;
        float nativePreviousRemainderDeltaSeconds = 0.0f;
        float nativeNextRemainderDeltaSeconds = 0.0f;
        std::uint32_t substepCount = 0;
        float maxPhysicsFrameSeconds = 0.0f;
        int clampedMaxSubsteps = kDefaultMaxSubsteps;
        const char* reason = "uninitialized";
    };

    inline bool isUsableDeltaSeconds(float value)
    {
        return std::isfinite(value) && value > kMinAcceptedFrameDeltaSeconds && value <= kMaxAcceptedFrameDeltaSeconds;
    }

    inline bool isUsableRemainderDeltaSeconds(float value)
    {
        return std::isfinite(value) && value >= 0.0f && value <= kMaxAcceptedFrameDeltaSeconds;
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
        const auto invalid = [&](const char* reason) {
            return TimingFixDecision{
                .valid = false,
                .sourceDeltaSeconds = input.sourceDeltaSeconds,
                .globalTimeMultiplier = input.globalTimeMultiplier,
                .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
                .clampedMaxSubsteps = maxSubsteps,
                .reason = reason,
            };
        };

        if (!input.sourceValid) {
            return invalid("invalidSourceFrame");
        }
        if (input.sourcePaused) {
            return invalid("sourcePaused");
        }
        if (input.sourceDiscontinuity) {
            return invalid("sourceDiscontinuity");
        }
        if (!isUsableDeltaSeconds(input.sourceDeltaSeconds)) {
            return invalid("invalidSourceDelta");
        }
        if (!std::isfinite(input.globalTimeMultiplier) || input.globalTimeMultiplier <= 0.0f) {
            return invalid("invalidGlobalTimeMultiplier");
        }
        if (!isUsableRemainderDeltaSeconds(input.nativeRemainderDeltaSeconds) ||
            !isUsableRemainderDeltaSeconds(input.nativePreviousRemainderDeltaSeconds) ||
            !isUsableRemainderDeltaSeconds(input.nativeAccumulatedDeltaSeconds) ||
            !isUsableDeltaSeconds(input.nativeSubstepDeltaSeconds) ||
            input.nativeSubstepCount > static_cast<std::uint32_t>(kEngineMaxSubsteps)) {
            return invalid("invalidNativeAccumulatorState");
        }

        /*
         * bhkWorld::Update begins by adding accumulated-minus-consumed time
         * to the live remainder. ROCK replaces the simulated interval, but
         * FO4VR's character controller and presentation paths still consume
         * both native remainder values. Pre-seed the live field with the
         * value native Update would have produced. Because the coherent
         * schedule consumes its accumulated duration exactly, Update then
         * leaves this native phase value unchanged.
         */
        const float nativeConsumedDeltaSeconds =
            input.nativeSubstepDeltaSeconds * static_cast<float>(input.nativeSubstepCount);
        float nativeNextRemainderDeltaSeconds =
            input.nativeRemainderDeltaSeconds +
            (input.nativeAccumulatedDeltaSeconds - nativeConsumedDeltaSeconds);
        constexpr float kRemainderRoundoffToleranceSeconds = 0.000001f;
        if (nativeNextRemainderDeltaSeconds < 0.0f &&
            nativeNextRemainderDeltaSeconds >= -kRemainderRoundoffToleranceSeconds) {
            nativeNextRemainderDeltaSeconds = 0.0f;
        }
        if (!isUsableRemainderDeltaSeconds(nativeNextRemainderDeltaSeconds)) {
            return invalid("invalidNativeNextRemainder");
        }

        const float coherentDeltaSeconds = input.sourceDeltaSeconds * input.globalTimeMultiplier;
        if (!isUsableDeltaSeconds(coherentDeltaSeconds)) {
            return invalid("invalidCoherentDelta");
        }

        /*
         * Consume exactly one scaled source interval in the upcoming world
         * update. Substeps divide that interval; they never select a different
         * total duration. The small epsilon prevents an exact threshold from
         * splitting because of one floating-point ulp.
         */
        constexpr float kSubstepCountEpsilon = 1.0e-4f;
        const float requiredSubsteps = std::ceil((coherentDeltaSeconds / maxPhysicsFrameSeconds) - kSubstepCountEpsilon);
        const auto substepCount = static_cast<std::uint32_t>(
            std::clamp(requiredSubsteps, 1.0f, static_cast<float>(maxSubsteps)));
        const float substepDeltaSeconds = coherentDeltaSeconds / static_cast<float>(substepCount);
        if (!isUsableDeltaSeconds(substepDeltaSeconds)) {
            return invalid("invalidSubstepDelta");
        }

        return TimingFixDecision{
            .valid = true,
            .sourceDeltaSeconds = input.sourceDeltaSeconds,
            .globalTimeMultiplier = input.globalTimeMultiplier,
            .coherentDeltaSeconds = coherentDeltaSeconds,
            .substepDeltaSeconds = substepDeltaSeconds,
            .simulatedDeltaSeconds = substepDeltaSeconds * static_cast<float>(substepCount),
            .nativePreviousRemainderDeltaSeconds = input.nativePreviousRemainderDeltaSeconds,
            .nativeNextRemainderDeltaSeconds = nativeNextRemainderDeltaSeconds,
            .substepCount = substepCount,
            .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
            .clampedMaxSubsteps = maxSubsteps,
            .reason = "ok",
        };
    }
}
