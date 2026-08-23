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
    constexpr float kMaxNativeAnchorDeltaSeconds = 0.1f;

    struct TimingFixRuntimeState
    {
        bool characterPresentationPhaseInitialized = false;
        float characterPresentationPhaseSeconds = 0.0f;
    };

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
        float nativeRawDeltaSeconds = 0.0f;
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
        float presentationPhaseSeconds = 0.0f;
        float characterPresentationPhaseSeconds = 0.0f;
        std::uint32_t substepCount = 0;
        float maxPhysicsFrameSeconds = 0.0f;
        int clampedMaxSubsteps = kDefaultMaxSubsteps;
        bool characterPresentationPhaseInitializedThisFrame = false;
        const char* reason = "uninitialized";
    };

    inline void resetTimingFixRuntimeState(TimingFixRuntimeState& state)
    {
        state = {};
    }

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

    inline TimingFixDecision evaluateTimingFix(
        const TimingFixInput& input,
        TimingFixRuntimeState& runtimeState)
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
        const auto resetAndInvalidate = [&](const char* reason) {
            resetTimingFixRuntimeState(runtimeState);
            return invalid(reason);
        };

        if (!input.sourceValid) {
            return resetAndInvalidate("invalidSourceFrame");
        }
        if (input.sourcePaused) {
            return resetAndInvalidate("sourcePaused");
        }
        if (input.sourceDiscontinuity) {
            return resetAndInvalidate("sourceDiscontinuity");
        }
        if (!isUsableDeltaSeconds(input.sourceDeltaSeconds)) {
            return resetAndInvalidate("invalidSourceDelta");
        }
        if (!std::isfinite(input.globalTimeMultiplier) || input.globalTimeMultiplier <= 0.0f) {
            return resetAndInvalidate("invalidGlobalTimeMultiplier");
        }
        if (!isUsableDeltaSeconds(input.nativeRawDeltaSeconds) ||
            input.nativeRawDeltaSeconds > kMaxNativeAnchorDeltaSeconds ||
            !isUsableRemainderDeltaSeconds(input.nativeRemainderDeltaSeconds) ||
            !isUsableRemainderDeltaSeconds(input.nativePreviousRemainderDeltaSeconds) ||
            !isUsableRemainderDeltaSeconds(input.nativeAccumulatedDeltaSeconds) ||
            !isUsableDeltaSeconds(input.nativeSubstepDeltaSeconds) ||
            input.nativeSubstepCount > static_cast<std::uint32_t>(kEngineMaxSubsteps)) {
            return resetAndInvalidate("invalidNativeAccumulatorState");
        }

        /*
         * bhkWorld::Update begins by adding accumulated-minus-consumed time
         * to the live remainder. Calculate the value the untouched native
         * schedule would have produced so it can validate and initialize a
         * separate player character-controller phase. Do not publish this
         * cycling value into the global collision-object presentation path.
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
            return resetAndInvalidate("invalidNativeNextRemainder");
        }

        const float coherentDeltaSeconds = input.sourceDeltaSeconds * input.globalTimeMultiplier;
        if (!isUsableDeltaSeconds(coherentDeltaSeconds)) {
            return resetAndInvalidate("invalidCoherentDelta");
        }

        /*
         * FO4VR's collision-object and character-controller presentation paths
         * both read the live remainder. They cannot share one value after ROCK
         * completes a source-clock solve every outer frame: a nonzero global
         * phase advances solved rigid bodies again, while removing the native
         * cycle from the player controller breaks locomotion presentation.
         *
         * Publish zero globally. Maintain the native sawtooth separately for a
         * narrow player-controller hook. The shadow phase starts from the last
         * untouched native result, then advances only from native raw duration
         * and the native fixed-step interval. ROCK's coherent substep selection
         * never becomes an input to this character-only clock.
         */
        bool characterPresentationPhaseInitializedThisFrame = false;
        float characterPresentationPhaseSeconds = 0.0f;
        if (runtimeState.characterPresentationPhaseInitialized) {
            characterPresentationPhaseSeconds = std::fmod(
                runtimeState.characterPresentationPhaseSeconds + input.nativeRawDeltaSeconds,
                input.nativeSubstepDeltaSeconds);
        } else {
            characterPresentationPhaseSeconds = std::fmod(
                nativeNextRemainderDeltaSeconds,
                input.nativeSubstepDeltaSeconds);
            runtimeState.characterPresentationPhaseInitialized = true;
            characterPresentationPhaseInitializedThisFrame = true;
        }

        if (!isUsableRemainderDeltaSeconds(characterPresentationPhaseSeconds)) {
            return resetAndInvalidate("invalidCharacterPresentationPhase");
        }
        runtimeState.characterPresentationPhaseSeconds = characterPresentationPhaseSeconds;

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
            return resetAndInvalidate("invalidSubstepDelta");
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
            .presentationPhaseSeconds = 0.0f,
            .characterPresentationPhaseSeconds = characterPresentationPhaseSeconds,
            .substepCount = substepCount,
            .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
            .clampedMaxSubsteps = maxSubsteps,
            .characterPresentationPhaseInitializedThisFrame = characterPresentationPhaseInitializedThisFrame,
            .reason = "ok",
        };
    }
}
