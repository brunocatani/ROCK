#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::havok_physics_timing
{
    /*
     * ROCK-generated keyframed bodies are consumed by FO4VR's hknp solver, so
     * their drive delta must come from bhkWorld::SetDeltaTime/bhkWorld::Update
     * rather than FRIK's render-frame timer. Keeping the binary-backed timing
     * sample separate from input/visual frame time prevents hand, weapon, and
     * held-object constraints from scaling velocity against the wrong clock.
     */
    inline constexpr float kFallbackPhysicsDeltaSeconds = 1.0f / 90.0f;

    enum class PhysicsStepPhase : std::uint8_t
    {
        WholePreStep,
        SubstepPreCollide,
        BetweenCollideAndSolve,
        SubstepPostSolve,
    };

    struct PhysicsTimingSample
    {
        float rawDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float substepDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float remainderDeltaSeconds = 0.0f;
        float accumulatedDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float simulatedDeltaSeconds = kFallbackPhysicsDeltaSeconds;
        float substepProgress = 0.0f;
        std::uint32_t substepCount = 1;
        std::uint32_t substepIndex = 0;
        PhysicsStepPhase phase = PhysicsStepPhase::WholePreStep;
        /*
         * Identity and cumulative-clock fields stamped by the step coordinator.
         * stepSequence identifies the owning bhkWorld update; solveSequence
         * counts completed substep solves (a post-solve sample includes its own
         * completed solve). elapsedSimulatedSeconds accumulates measured
         * substep time only — fabricated fallback deltas never advance it.
         * All three are monotonic across world resets so stored physics
         * timestamps keep ordering through world loss and recreation.
         */
        std::uint64_t stepSequence = 0;
        std::uint64_t solveSequence = 0;
        double elapsedSimulatedSeconds = 0.0;
        bool valid = false;
        /*
         * usedFallback is the raw-validity contract: true whenever any native
         * timing input was unusable and a sanitized substitute is present in
         * this sample. Behavior that integrates or retains state must treat a
         * fallback sample as unmeasured and fail closed.
         */
        bool usedFallback = true;
    };

    // A completed substep advances the cumulative simulated clock only when
    // its delta was actually measured from native timing.
    [[nodiscard]] inline bool shouldAccumulateSimulatedTime(const PhysicsTimingSample& sample)
    {
        return sample.valid && !sample.usedFallback;
    }

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

    inline PhysicsTimingSample makeSubstepTimingSample(
        const PhysicsTimingSample& wholeStep,
        float substepProgress,
        float substepDeltaSeconds,
        std::uint32_t substepIndex)
    {
        PhysicsTimingSample sample = wholeStep;
        sample.phase = PhysicsStepPhase::SubstepPreCollide;
        sample.substepProgress = std::isfinite(substepProgress) ? std::clamp(substepProgress, 0.0f, 1.0f) : 0.0f;
        sample.substepDeltaSeconds = isUsableDelta(substepDeltaSeconds) ? substepDeltaSeconds : wholeStep.substepDeltaSeconds;
        sample.substepIndex = substepIndex;
        sample.usedFallback = sample.usedFallback || !isUsableDelta(substepDeltaSeconds);
        sample.valid = isUsableDelta(sample.substepDeltaSeconds);
        return sample;
    }

    inline PhysicsTimingSample makeSubstepPhaseTimingSample(const PhysicsTimingSample& substep, PhysicsStepPhase phase)
    {
        PhysicsTimingSample sample = substep;
        sample.phase = phase;
        sample.valid = isUsableDelta(sample.substepDeltaSeconds);
        return sample;
    }

    PhysicsTimingSample sampleCurrentTiming();

    inline float driveDeltaSeconds(const PhysicsTimingSample& timing)
    {
        /*
         * FO4VR exposes both a whole-world pre-step callback and a per-substep
         * pre-collide callback. Native mouse-spring grabs keep the whole-frame
         * boundary because that path is smooth. Generated keyframed colliders
         * use SubstepPreCollide samples so Bethesda's keyframe velocity is
         * scaled to the same hknp substep that will immediately consume it.
         */
        if ((timing.phase == PhysicsStepPhase::SubstepPreCollide ||
                timing.phase == PhysicsStepPhase::BetweenCollideAndSolve ||
                timing.phase == PhysicsStepPhase::SubstepPostSolve) &&
            isUsableDelta(timing.substepDeltaSeconds)) {
            return timing.substepDeltaSeconds;
        }
        if (timing.valid && isUsableDelta(timing.rawDeltaSeconds)) {
            return timing.rawDeltaSeconds;
        }
        return kFallbackPhysicsDeltaSeconds;
    }
}
