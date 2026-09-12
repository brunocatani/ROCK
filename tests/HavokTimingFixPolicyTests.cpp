#include "physics-interaction/native/HavokTimingFixPolicy.h"

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

    /*
     * Supported-rate matrix rows for the timing-normalization project. The
     * 45 FPS row also covers 45 FPS game cadence under 90 Hz headset
     * reprojection: reprojection produces no extra game or physics callbacks,
     * so the raw delta ROCK receives is identical.
     */
    {
        constexpr float rawDelta = 1.0f / 45.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("45hz frame is valid", decision.valid);
        ok &= expectNear("45hz splits into two 90hz substeps", decision.substepDeltaSeconds, rawDelta * 0.5f, 0.000001f);
        ok &= expectEqual("45hz count", decision.substepCount, 2);
    }

    {
        constexpr float rawDelta = 1.0f / 72.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("72hz frame is valid", decision.valid);
        ok &= expectNear("72hz stays single substep", decision.substepDeltaSeconds, rawDelta, 0.000001f);
        ok &= expectEqual("72hz count", decision.substepCount, 1);
    }

    {
        constexpr float rawDelta = 1.0f / 120.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("120hz frame is valid", decision.valid);
        ok &= expectNear("120hz stays single substep", decision.substepDeltaSeconds, rawDelta, 0.000001f);
        ok &= expectEqual("120hz count", decision.substepCount, 1);
    }

    // Irregular pacing: jitter around the 72 Hz and 90 Hz cadences must not
    // flip the substep decision.
    {
        for (const float rawDelta : { 0.0134f, 0.0142f, 0.0139f }) {
            const auto decision = evaluateTimingFix(TimingFixInput{
                .rawDeltaSeconds = rawDelta,
                .accumulatedDeltaSeconds = rawDelta,
                .minPhysicsFrameRate = 70.0f,
                .maxSubsteps = 3,
            });
            ok &= expectTrue("72hz jitter frame is valid", decision.valid);
            ok &= expectEqual("72hz jitter stays single substep", decision.substepCount, 1);
        }
        for (const float rawDelta : { 0.0108f, 0.0115f, 0.0111f }) {
            const auto decision = evaluateTimingFix(TimingFixInput{
                .rawDeltaSeconds = rawDelta,
                .accumulatedDeltaSeconds = rawDelta,
                .minPhysicsFrameRate = 70.0f,
                .maxSubsteps = 3,
            });
            ok &= expectTrue("90hz jitter frame is valid", decision.valid);
            ok &= expectEqual("90hz jitter stays single substep", decision.substepCount, 1);
        }
    }

    // Accumulated delta producing three substeps at the 45 FPS split delta.
    {
        constexpr float rawDelta = 1.0f / 45.0f;
        const auto decision = evaluateTimingFix(TimingFixInput{
            .rawDeltaSeconds = rawDelta,
            .accumulatedDeltaSeconds = rawDelta * 1.5f,
            .minPhysicsFrameRate = 70.0f,
            .maxSubsteps = 3,
        });

        ok &= expectTrue("accumulated 45hz frame is valid", decision.valid);
        ok &= expectNear("accumulated 45hz keeps split delta", decision.substepDeltaSeconds, rawDelta * 0.5f, 0.000001f);
        ok &= expectEqual("accumulated 45hz count", decision.substepCount, 3);
    }

    // Non-finite and negative inputs must stay invalid instead of becoming a
    // usable-looking schedule.
    {
        const float invalidValues[] = {
            -0.011f,
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(),
        };
        for (const float rawDelta : invalidValues) {
            const auto rawDecision = evaluateTimingFix(TimingFixInput{
                .rawDeltaSeconds = rawDelta,
                .accumulatedDeltaSeconds = 1.0f / 90.0f,
                .minPhysicsFrameRate = 70.0f,
                .maxSubsteps = 3,
            });
            ok &= expectFalse("non-finite or negative raw delta is invalid", rawDecision.valid);

            const auto accumulatedDecision = evaluateTimingFix(TimingFixInput{
                .rawDeltaSeconds = 1.0f / 90.0f,
                .accumulatedDeltaSeconds = rawDelta,
                .minPhysicsFrameRate = 70.0f,
                .maxSubsteps = 3,
            });
            ok &= expectFalse("non-finite or negative accumulated delta is invalid", accumulatedDecision.valid);
        }
    }

    return ok ? 0 : 1;
}
