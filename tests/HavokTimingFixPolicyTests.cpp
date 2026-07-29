#include "physics-interaction/native/HavokTimingFixPolicy.h"

#include <cmath>
#include <cstdio>

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
}

int main()
{
    using namespace rock::havok_timing_fix_policy;

    bool ok = true;

    {
        constexpr float rawDelta = 1.0f / 90.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("90hz frame is valid", decision.valid);
        ok &= expectNear("90hz stays single substep", decision.substepDeltaSeconds, rawDelta, 0.000001f);
        ok &= expectEqual("90hz count", decision.substepCount, 1);
    }

    {
        constexpr float rawDelta = 1.0f / 60.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("60hz frame is valid", decision.valid);
        ok &= expectNear("60hz splits into two", decision.substepDeltaSeconds, rawDelta * 0.5f, 0.000001f);
        ok &= expectEqual("60hz count", decision.substepCount, 2);
    }

    {
        constexpr float rawDelta = 1.0f / 30.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("30hz frame is valid", decision.valid);
        ok &= expectNear("30hz uses max three substeps", decision.substepDeltaSeconds, rawDelta / 3.0f, 0.000001f);
        ok &= expectEqual("30hz count", decision.substepCount, 3);
    }

    {
        constexpr float rawDelta = 1.0f / 20.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 120.0f,
            .maxSubsteps = 99,
        });

        ok &= expectTrue("max substeps clamp is valid", decision.valid);
        ok &= expectNear("max substeps clamp uses six", decision.substepDeltaSeconds, rawDelta / 6.0f, 0.000001f);
        ok &= expectEqual("max substeps clamp count", decision.substepCount, 6);
    }

    {
        constexpr float rawDelta = 1.0f / 90.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta * 2.0f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("accumulated 90hz two-step frame is valid", decision.valid);
        ok &= expectNear("accumulated 90hz keeps frame delta", decision.substepDeltaSeconds, rawDelta, 0.000001f);
        ok &= expectEqual("accumulated 90hz count", decision.substepCount, 2);
    }

    {
        constexpr float rawDelta = 1.0f / 60.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta * 1.5f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("accumulated 60hz clamp frame is valid", decision.valid);
        ok &= expectNear("accumulated 60hz keeps split delta", decision.substepDeltaSeconds, rawDelta * 0.5f, 0.000001f);
        ok &= expectEqual("accumulated 60hz count clamps to max", decision.substepCount, 3);
    }

    {
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = 0.0f,
            .accumulatedDeltaSeconds = 1.0f / 90.0f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectFalse("zero raw delta is invalid", decision.valid);
    }

    {
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = 1.0f / 90.0f,
            .accumulatedDeltaSeconds = 0.0f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectFalse("zero accumulated delta is invalid", decision.valid);
    }

    {
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = kMinAcceptedFrameDeltaSeconds,
            .accumulatedDeltaSeconds = 1.0f / 90.0f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectFalse("minimum threshold raw delta is invalid", decision.valid);
    }

    return ok ? 0 : 1;
}
