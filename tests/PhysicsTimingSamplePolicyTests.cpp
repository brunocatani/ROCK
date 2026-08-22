#include "physics-interaction/native/HavokPhysicsTiming.h"

#include <cmath>
#include <cstdio>
#include <limits>

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

    bool expectNear(const char* label, double actual, double expected, double epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }

        std::printf("%s expected %.9f got %.9f\n", label, expected, actual);
        return false;
    }

    bool expectEqualU64(const char* label, std::uint64_t actual, std::uint64_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }
}

int main()
{
    using namespace rock::havok_physics_timing;

    bool ok = true;

    // Measured native timing keeps its values and reports no fallback use.
    {
        const auto sample = makeTimingSample(1.0f / 90.0f, 1.0f / 90.0f, 0.0f, 1.0f / 90.0f, 1);
        ok &= expectTrue("measured sample valid", sample.valid);
        ok &= expectFalse("measured sample no fallback", sample.usedFallback);
        ok &= expectTrue("measured sample accumulates", shouldAccumulateSimulatedTime(sample));
    }

    // Missing or unusable native inputs are flagged as fallback so consumers
    // can refuse to integrate fabricated time.
    {
        const auto missingRaw = makeTimingSample(0.0f, 1.0f / 90.0f, 0.0f, 1.0f / 90.0f, 1);
        ok &= expectTrue("missing raw flags fallback", missingRaw.usedFallback);
        ok &= expectFalse("missing raw never accumulates", shouldAccumulateSimulatedTime(missingRaw));

        const auto nanRaw = makeTimingSample(std::numeric_limits<float>::quiet_NaN(), 1.0f / 90.0f, 0.0f, 1.0f / 90.0f, 1);
        ok &= expectTrue("nan raw flags fallback", nanRaw.usedFallback);

        const auto zeroCount = makeTimingSample(1.0f / 90.0f, 1.0f / 90.0f, 0.0f, 1.0f / 90.0f, 0);
        ok &= expectTrue("zero substep count flags fallback", zeroCount.usedFallback);

        const auto badSubstep = makeTimingSample(1.0f / 90.0f, std::numeric_limits<float>::infinity(), 0.0f, 1.0f / 90.0f, 2);
        ok &= expectTrue("unusable substep flags fallback", badSubstep.usedFallback);
    }

    // Identity fields default to the unstamped state and survive the
    // whole-step -> substep -> phase derivation chain unmodified.
    {
        auto wholeStep = makeTimingSample(1.0f / 45.0f, 1.0f / 90.0f, 0.0f, 1.0f / 45.0f, 2);
        ok &= expectEqualU64("unstamped step sequence", wholeStep.stepSequence, 0);
        wholeStep.stepSequence = 41;
        wholeStep.solveSequence = 82;
        wholeStep.elapsedSimulatedSeconds = 12.5;

        const auto substep = makeSubstepTimingSample(wholeStep, 0.5f, 1.0f / 90.0f, 1);
        ok &= expectEqualU64("substep inherits step sequence", substep.stepSequence, 41);
        ok &= expectEqualU64("substep inherits solve sequence", substep.solveSequence, 82);
        ok &= expectNear("substep inherits simulated clock", substep.elapsedSimulatedSeconds, 12.5, 0.0);
        ok &= expectEqualU64("substep index stamped", substep.substepIndex, 1);
        ok &= expectTrue("substep phase pre-collide", substep.phase == PhysicsStepPhase::SubstepPreCollide);

        const auto postSolve = makeSubstepPhaseTimingSample(substep, PhysicsStepPhase::SubstepPostSolve);
        ok &= expectEqualU64("phase sample keeps step sequence", postSolve.stepSequence, 41);
        ok &= expectTrue("phase sample post-solve", postSolve.phase == PhysicsStepPhase::SubstepPostSolve);
        ok &= expectNear("phase sample keeps substep delta", postSolve.substepDeltaSeconds, 1.0 / 90.0, 1.0e-9);
    }

    // A fallback substep delta poisons derived samples: the fallback flag is
    // sticky through the derivation chain.
    {
        const auto wholeStep = makeTimingSample(1.0f / 90.0f, 1.0f / 90.0f, 0.0f, 1.0f / 90.0f, 1);
        const auto substep = makeSubstepTimingSample(wholeStep, 0.0f, std::numeric_limits<float>::quiet_NaN(), 0);
        ok &= expectTrue("fallback substep delta flagged", substep.usedFallback);
        ok &= expectFalse("fallback substep never accumulates", shouldAccumulateSimulatedTime(substep));

        const auto postSolve = makeSubstepPhaseTimingSample(substep, PhysicsStepPhase::SubstepPostSolve);
        ok &= expectTrue("fallback flag sticky through phases", postSolve.usedFallback);
    }

    // Missing native globals (all zeros) must classify as an invalid fallback
    // sample, never a usable-looking schedule, and the drive-delta accessor
    // must refuse it.
    {
        const auto missingGlobals = makeTimingSample(0.0f, 0.0f, 0.0f, 0.0f, 0);
        ok &= expectFalse("missing globals invalid", missingGlobals.valid);
        ok &= expectTrue("missing globals flag fallback", missingGlobals.usedFallback);
        ok &= expectNear("missing globals zero raw", missingGlobals.rawDeltaSeconds, 0.0, 0.0);
        ok &= expectNear("missing globals zero substep", missingGlobals.substepDeltaSeconds, 0.0, 0.0);

        float driveDelta = -1.0f;
        ok &= expectFalse("missing globals refuse drive delta", tryGetDriveDeltaSeconds(missingGlobals, driveDelta));
        ok &= expectNear("refused drive delta zero", driveDelta, 0.0, 0.0);
    }

    // A measured substep sample yields its substep delta through the
    // drive-delta accessor; a fallback-tainted one is refused even when its
    // fields look plausible.
    {
        const auto wholeStep = makeTimingSample(1.0f / 45.0f, 1.0f / 90.0f, 0.0f, 1.0f / 45.0f, 2);
        const auto substep = makeSubstepTimingSample(wholeStep, 0.5f, 1.0f / 90.0f, 1);
        float driveDelta = 0.0f;
        ok &= expectTrue("measured substep provides drive delta", tryGetDriveDeltaSeconds(substep, driveDelta));
        ok &= expectNear("measured substep drive delta", driveDelta, 1.0 / 90.0, 1.0e-9);

        auto tainted = substep;
        tainted.usedFallback = true;
        float taintedDelta = 0.0f;
        ok &= expectFalse("fallback-tainted sample refused", tryGetDriveDeltaSeconds(tainted, taintedDelta));
    }

    // Substep deltas measured by the engine (alternating 10/11/12 ms) pass
    // through the substep derivation unmodified.
    {
        const auto wholeStep = makeTimingSample(0.033f, 0.011f, 0.0f, 0.033f, 3);
        const float substepDeltas[] = { 0.010f, 0.011f, 0.012f };
        for (std::uint32_t index = 0; index < 3; ++index) {
            const auto substep = makeSubstepTimingSample(wholeStep, static_cast<float>(index) / 3.0f, substepDeltas[index], index);
            ok &= expectTrue("alternating substep valid", substep.valid);
            ok &= expectFalse("alternating substep measured", substep.usedFallback);
            ok &= expectNear("alternating substep delta passthrough", substep.substepDeltaSeconds, substepDeltas[index], 1.0e-9);
        }
    }

    return ok ? 0 : 1;
}
