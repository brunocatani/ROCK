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
        bool presentationPhaseInitialized = false;
        float presentationPhaseSeconds = 0.0f;
        float globalTimeMultiplier = 1.0f;
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
        std::uint32_t substepCount = 0;
        float maxPhysicsFrameSeconds = 0.0f;
        int clampedMaxSubsteps = kDefaultMaxSubsteps;
        bool presentationPhaseInitializedThisFrame = false;
        bool presentationPhaseRescaled = false;
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
         * schedule would have produced so it can validate and initialize the
         * coherent presentation phase. Do not publish this cycling value into
         * an every-frame coherent schedule.
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
         * Native FO4VR uses the live remainder as an extrapolation interval:
         * presentedPosition = completedPosition + remainder * velocity. A
         * native fixed-step world needs a cycling remainder between solves.
         * ROCK's coherent schedule completes one source interval every outer
         * frame, so carrying that unrelated native cycle forward moves every
         * presented Havok body by a second, discontinuous clock.
         *
         * Anchor one normal native phase and keep it stable while ordinary
         * coherent frames run. This is the behavior the previous timing fix
         * reached naturally after it forced at least one raw-duration step:
         * accumulated - consumed became the prior remainder. A zero phase is
         * not fabricated; if native timing is exactly on a step boundary,
         * preserve the native schedule until it supplies a usable phase.
         */
        bool presentationPhaseInitializedThisFrame = false;
        bool presentationPhaseRescaled = false;
        if (runtimeState.presentationPhaseInitialized) {
            constexpr float kMultiplierChangeTolerance = 0.0001f;
            if (std::fabs(runtimeState.globalTimeMultiplier - input.globalTimeMultiplier) >
                kMultiplierChangeTolerance) {
                runtimeState.presentationPhaseSeconds *=
                    input.globalTimeMultiplier / runtimeState.globalTimeMultiplier;
                runtimeState.globalTimeMultiplier = input.globalTimeMultiplier;
                presentationPhaseRescaled = true;
            }
        } else {
            float phaseCandidate = input.nativePreviousRemainderDeltaSeconds;
            if (phaseCandidate <= kMinAcceptedFrameDeltaSeconds) {
                phaseCandidate = nativeNextRemainderDeltaSeconds;
            }
            if (phaseCandidate <= kMinAcceptedFrameDeltaSeconds) {
                return invalid("nativePresentationPhaseNotReady");
            }

            phaseCandidate = std::fmod(phaseCandidate, coherentDeltaSeconds);
            if (!isUsableDeltaSeconds(phaseCandidate)) {
                return invalid("nativePresentationPhaseNotReady");
            }
            runtimeState.presentationPhaseInitialized = true;
            runtimeState.presentationPhaseSeconds = phaseCandidate;
            runtimeState.globalTimeMultiplier = input.globalTimeMultiplier;
            presentationPhaseInitializedThisFrame = true;
        }

        if (runtimeState.presentationPhaseSeconds >= coherentDeltaSeconds) {
            const float foldedPhase =
                std::fmod(runtimeState.presentationPhaseSeconds, coherentDeltaSeconds);
            if (!isUsableDeltaSeconds(foldedPhase)) {
                return resetAndInvalidate("invalidPresentationPhase");
            }
            runtimeState.presentationPhaseSeconds = foldedPhase;
        }
        const float presentationPhaseSeconds = runtimeState.presentationPhaseSeconds;

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
            .presentationPhaseSeconds = presentationPhaseSeconds,
            .substepCount = substepCount,
            .maxPhysicsFrameSeconds = maxPhysicsFrameSeconds,
            .clampedMaxSubsteps = maxSubsteps,
            .presentationPhaseInitializedThisFrame = presentationPhaseInitializedThisFrame,
            .presentationPhaseRescaled = presentationPhaseRescaled,
            .reason = "ok",
        };
    }
}
