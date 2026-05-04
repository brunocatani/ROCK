#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace frik::rock::havok_physics_timing
{
    /*
     * ROCK-generated keyframed bodies are consumed by FO4VR's hknp solver, so
     * their drive delta must come from bhkWorld::SetDeltaTime/bhkWorld::Update
     * rather than FRIK's render-frame timer. Keeping the binary-backed timing
     * sample separate from input/visual frame time prevents hand, weapon, and
     * held-object constraints from scaling velocity against the wrong clock.
     */
    inline constexpr float kFallbackPhysicsDeltaSeconds = 1.0f / 90.0f;

    struct PhysicsTimingSample
    {
        float rawDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float substepDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float remainderDeltaSeconds = 0.0f;
        float accumulatedDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float simulatedDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        std::uint32_t substepCount = 1;
        bool valid = false;
        bool usedFallback = true;
    };

    inline bool isUsableDelta(float value)
    {
        return std::isfinite(value) && value > 0.000001f && value <= 0.25f;
    }

    inline PhysicsTimingSample makeTimingSample(
        float rawDeltaSeconds,
        float substepDeltaSeconds,
        float remainderDeltaSeconds,
        float accumulatedDeltaSeconds,
        std::uint32_t substepCount)
    {
        PhysicsTimingSample sample{};
        sample.rawDeltaSeconds = isUsableDelta(rawDeltaSeconds) ? rawDeltaSeconds : kFallbackPhysicsDeltaSeconds;
        sample.substepDeltaSeconds = isUsableDelta(substepDeltaSeconds) ? substepDeltaSeconds : sample.rawDeltaSeconds;
        sample.remainderDeltaSeconds = std::isfinite(remainderDeltaSeconds) ? remainderDeltaSeconds : 0.0f;
        sample.accumulatedDeltaSeconds = isUsableDelta(accumulatedDeltaSeconds) ? accumulatedDeltaSeconds : sample.rawDeltaSeconds;
        sample.substepCount = (std::min)(substepCount, 6u);
        if (sample.substepCount == 0) {
            sample.substepCount = 1;
        }

        const float simulated = sample.substepDeltaSeconds * static_cast<float>(sample.substepCount);
        sample.simulatedDeltaSeconds = isUsableDelta(simulated) ? simulated : sample.rawDeltaSeconds;
        sample.valid = isUsableDelta(sample.simulatedDeltaSeconds);
        sample.usedFallback = !isUsableDelta(rawDeltaSeconds) || !isUsableDelta(substepDeltaSeconds) || substepCount == 0;
        return sample;
    }

    PhysicsTimingSample sampleCurrentTiming();

    inline float driveDeltaSeconds(const PhysicsTimingSample& timing)
    {
        /*
         * ROCK calls Bethesda's keyframe drive once from the whole-world step
         * listener, matching HIGGS' single target-per-frame model. Substep data
         * stays in telemetry so we can prove what Havok consumed, but the drive
         * boundary receives the whole bhkWorld frame delta rather than an
         * inferred multiplied substep duration.
         */
        if (timing.valid && isUsableDelta(timing.rawDeltaSeconds)) {
            return timing.rawDeltaSeconds;
        }
        return kFallbackPhysicsDeltaSeconds;
    }
}
