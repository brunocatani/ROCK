#include "physics-interaction/native/HavokTimingFixPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <tuple>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectEqual(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    rock::havok_timing_fix_policy::TimingFixDecision evaluate(
        float sourceDelta,
        float multiplier = 1.0f,
        float minHz = 70.0f,
        int maxSubsteps = 3)
    {
        rock::havok_timing_fix_policy::TimingFixRuntimeState runtimeState{};
        return rock::havok_timing_fix_policy::evaluateTimingFix(
            rock::havok_timing_fix_policy::TimingFixInput{
                .sourceDeltaSeconds = sourceDelta,
                .globalTimeMultiplier = multiplier,
                .nativeRawDeltaSeconds = 1.0f / 90.0f,
                .nativeRemainderDeltaSeconds = 0.0f,
                .nativePreviousRemainderDeltaSeconds = 0.004f,
                .nativeAccumulatedDeltaSeconds = 1.0f / 90.0f,
                .nativeSubstepDeltaSeconds = 1.0f / 60.0f,
                .nativeSubstepCount = 0,
                .minPhysicsFrameRate = minHz,
                .maxSubsteps = maxSubsteps,
                .sourceValid = true,
                .sourceDiscontinuity = false,
                .sourcePaused = false,
            },
            runtimeState);
    }
}

int main()
{
    using namespace rock::havok_timing_fix_policy;
    bool ok = true;

    for (const auto [label, sourceDelta, expectedCount] : {
             std::tuple{ "120hz", 1.0f / 120.0f, 1u },
             std::tuple{ "90hz", 1.0f / 90.0f, 1u },
             std::tuple{ "72hz", 1.0f / 72.0f, 1u },
             std::tuple{ "60hz", 1.0f / 60.0f, 2u },
             std::tuple{ "45hz", 1.0f / 45.0f, 2u },
             std::tuple{ "30hz", 1.0f / 30.0f, 3u },
         }) {
        const auto decision = evaluate(sourceDelta);
        ok &= expectTrue(label, decision.valid);
        ok &= expectEqual(label, decision.substepCount, expectedCount);
        ok &= expectNear(label, decision.simulatedDeltaSeconds, sourceDelta, 0.000001f);
        ok &= expectNear(label, decision.substepDeltaSeconds, sourceDelta / static_cast<float>(expectedCount), 0.000001f);
        ok &= expectNear(label, decision.nativePreviousRemainderDeltaSeconds, 0.004f, 0.000001f);
        ok &= expectNear(label, decision.nativeNextRemainderDeltaSeconds, 1.0f / 90.0f, 0.000001f);
        ok &= expectNear(label, decision.presentationPhaseSeconds, 1.0f / 120.0f, 0.000001f);
    }

    // Ordinary pacing variation must always preserve the complete source
    // interval, even when the substep count changes at the configured bound.
    for (const float sourceDelta : { 0.006f, 0.007f, 0.008f, 0.0108f, 0.0115f, 0.0139f, 0.0142f, 0.015f }) {
        const auto decision = evaluate(sourceDelta);
        ok &= expectTrue("irregular source frame", decision.valid);
        ok &= expectNear("irregular simulated duration", decision.simulatedDeltaSeconds, sourceDelta, 0.000001f);
    }

    // The global simulation multiplier scales physics time. Calendar
    // TimeScale is deliberately absent from this policy.
    {
        constexpr float sourceDelta = 1.0f / 90.0f;
        const auto halfSpeed = evaluate(sourceDelta, 0.5f);
        ok &= expectTrue("half-speed multiplier", halfSpeed.valid);
        ok &= expectNear("half-speed coherent duration", halfSpeed.coherentDeltaSeconds, sourceDelta * 0.5f, 0.000001f);
        ok &= expectEqual("half-speed substeps", halfSpeed.substepCount, 1);

        const auto doubleSpeed = evaluate(sourceDelta, 2.0f);
        ok &= expectTrue("double-speed multiplier", doubleSpeed.valid);
        ok &= expectNear("double-speed coherent duration", doubleSpeed.coherentDeltaSeconds, sourceDelta * 2.0f, 0.000001f);
        ok &= expectEqual("double-speed substeps", doubleSpeed.substepCount, 2);
    }

    {
        const auto clamped = evaluate(1.0f / 20.0f, 1.0f, 120.0f, 99);
        ok &= expectTrue("engine max-substep clamp", clamped.valid);
        ok &= expectEqual("engine max-substep count", clamped.substepCount, 6);
        ok &= expectNear("engine max-substep duration", clamped.simulatedDeltaSeconds, 1.0f / 20.0f, 0.000001f);
    }

    {
        auto input = TimingFixInput{
            .sourceDeltaSeconds = 1.0f / 90.0f,
            .globalTimeMultiplier = 1.0f,
            .nativeRawDeltaSeconds = 1.0f / 90.0f,
            .nativeRemainderDeltaSeconds = 0.0f,
            .nativePreviousRemainderDeltaSeconds = 0.004f,
            .nativeAccumulatedDeltaSeconds = 2.0f / 90.0f,
            .nativeSubstepDeltaSeconds = 1.0f / 60.0f,
            .nativeSubstepCount = 1,
            .sourceValid = true,
            .sourceDiscontinuity = false,
            .sourcePaused = false,
        };
        TimingFixRuntimeState runtimeState{};
        const auto decision = evaluateTimingFix(input, runtimeState);
        ok &= expectTrue("native remainder phase", decision.valid);
        ok &= expectNear(
            "native previous remainder",
            decision.nativePreviousRemainderDeltaSeconds,
            0.004f,
            0.000001f);
        ok &= expectNear(
            "native next remainder",
            decision.nativeNextRemainderDeltaSeconds,
            (2.0f / 90.0f) - (1.0f / 60.0f),
            0.000001f);
        ok &= expectNear("stable presentation phase", decision.presentationPhaseSeconds, 1.0f / 120.0f, 0.000001f);
        ok &= expectTrue("presentation phase initialized", decision.presentationPhaseInitializedThisFrame);

        input.nativePreviousRemainderDeltaSeconds = 0.010f;
        input.nativeAccumulatedDeltaSeconds = 0.010f;
        input.nativeSubstepCount = 0;
        const auto continued = evaluateTimingFix(input, runtimeState);
        ok &= expectTrue("continued presentation phase", continued.valid);
        ok &= expectNear("continued presentation phase", continued.presentationPhaseSeconds, 1.0f / 120.0f, 0.000001f);
        ok &= expectFalse("continued phase not reinitialized", continued.presentationPhaseInitializedThisFrame);

        input.nativeAccumulatedDeltaSeconds = 0.0f;
        input.nativeSubstepCount = 1;
        ok &= expectFalse("negative native remainder", evaluateTimingFix(input, runtimeState).valid);
        ok &= expectFalse("invalid native state resets phase", runtimeState.presentationPhaseInitialized);
    }

    {
        auto input = TimingFixInput{
            .sourceDeltaSeconds = 1.0f / 60.0f,
            .globalTimeMultiplier = 1.0f,
            .nativeRawDeltaSeconds = 1.0f / 60.0f,
            .nativeRemainderDeltaSeconds = 0.0f,
            .nativePreviousRemainderDeltaSeconds = 0.0f,
            .nativeAccumulatedDeltaSeconds = 1.0f / 60.0f,
            .nativeSubstepDeltaSeconds = 1.0f / 60.0f,
            .nativeSubstepCount = 1,
            .sourceValid = true,
            .sourceDiscontinuity = false,
            .sourcePaused = false,
        };
        TimingFixRuntimeState runtimeState{};
        const auto decision = evaluateTimingFix(input, runtimeState);
        ok &= expectTrue("native phase on step boundary", decision.valid);
        ok &= expectNear("mean native presentation phase", decision.presentationPhaseSeconds, 1.0f / 120.0f, 0.000001f);
        ok &= expectTrue("mean native phase initialized", runtimeState.presentationPhaseInitialized);
    }

    // The first normal source frame after loading can still carry the native
    // adaptive timer's much larger recovery substep. Do not freeze that load
    // phase into the presentation clock; initialize after native timing has
    // returned to its ordinary fixed step.
    {
        auto input = TimingFixInput{
            .sourceDeltaSeconds = 0.012251f,
            .globalTimeMultiplier = 1.0f,
            .nativeRawDeltaSeconds = 0.009f,
            .nativeRemainderDeltaSeconds = 0.0f,
            .nativePreviousRemainderDeltaSeconds = 0.024833f,
            .nativeAccumulatedDeltaSeconds = 0.033833f,
            .nativeSubstepDeltaSeconds = 0.033167f,
            .nativeSubstepCount = 1,
            .sourceValid = true,
            .sourceDiscontinuity = false,
            .sourcePaused = false,
        };
        TimingFixRuntimeState runtimeState{};
        ok &= expectFalse("adaptive recovery phase rejected", evaluateTimingFix(input, runtimeState).valid);
        ok &= expectFalse("adaptive recovery phase not initialized", runtimeState.presentationPhaseInitialized);

        input.nativeRawDeltaSeconds = 0.011f;
        input.nativePreviousRemainderDeltaSeconds = 0.000332f;
        input.nativeAccumulatedDeltaSeconds = 0.011332f;
        input.nativeSubstepDeltaSeconds = 1.0f / 60.0f;
        input.nativeSubstepCount = 0;
        const auto recovered = evaluateTimingFix(input, runtimeState);
        ok &= expectTrue("recovered native phase", recovered.valid);
        ok &= expectNear("recovered mean phase", recovered.presentationPhaseSeconds, 1.0f / 120.0f, 0.000001f);
    }

    {
        auto input = TimingFixInput{
            .sourceDeltaSeconds = 1.0f / 90.0f,
            .globalTimeMultiplier = 1.0f,
            .nativeRawDeltaSeconds = 1.0f / 90.0f,
            .nativeRemainderDeltaSeconds = 0.0f,
            .nativePreviousRemainderDeltaSeconds = 0.004f,
            .nativeAccumulatedDeltaSeconds = 1.0f / 90.0f,
            .nativeSubstepDeltaSeconds = 1.0f / 60.0f,
            .nativeSubstepCount = 0,
            .sourceValid = true,
            .sourceDiscontinuity = false,
            .sourcePaused = false,
        };
        TimingFixRuntimeState runtimeState{};
        input.sourceValid = false;
        ok &= expectFalse("invalid source frame", evaluateTimingFix(input, runtimeState).valid);
        input.sourceValid = true;
        input.sourceDiscontinuity = true;
        ok &= expectFalse("source discontinuity", evaluateTimingFix(input, runtimeState).valid);
        input.sourceDiscontinuity = false;
        input.sourcePaused = true;
        ok &= expectFalse("source pause", evaluateTimingFix(input, runtimeState).valid);
    }

    for (const float invalid : {
             0.0f,
             kMinAcceptedFrameDeltaSeconds,
             -0.011f,
             std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::infinity(),
         }) {
        ok &= expectFalse("invalid source delta", evaluate(invalid).valid);
        ok &= expectFalse("invalid global multiplier", evaluate(1.0f / 90.0f, invalid).valid);
    }

    return ok ? 0 : 1;
}
