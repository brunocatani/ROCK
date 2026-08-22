#include "physics-interaction/timing/GameFrameTimingPolicy.h"

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
    using namespace rock::game_frame_timing_policy;

    bool ok = true;

    // First frame: no measurable delta, fail-closed identity.
    {
        GameFrameClockState state{};
        const auto timing = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.011f });
        ok &= expectFalse("first frame invalid", timing.valid);
        ok &= expectTrue("first frame discontinuity", timing.discontinuity);
        ok &= expectNear("first frame delta zero", timing.deltaSeconds, 0.0, 0.0);
        ok &= expectEqualU64("first frame sequence", timing.sequence, 1);
        ok &= expectNear("first frame elapsed zero", timing.elapsedGameSeconds, 0.0, 0.0);
    }

    // Supported cadence matrix: the sanitized delta passes through unmodified
    // and the cumulative clock accumulates the same real duration at every
    // rate. 45 FPS with 90 Hz headset reprojection is the 45 FPS row: the
    // game callback cadence is what ROCK receives; reprojection creates no
    // extra samples.
    {
        constexpr float kCadences[] = {
            1.0f / 45.0f,
            1.0f / 60.0f,
            1.0f / 72.0f,
            1.0f / 90.0f,
            1.0f / 120.0f,
        };
        for (const float delta : kCadences) {
            GameFrameClockState state{};
            (void)advanceGameFrameClock(state, GameFrameSampleInput{});
            const int frames = static_cast<int>(std::lround(1.0f / delta));
            GameFrameTiming last{};
            for (int i = 0; i < frames; ++i) {
                last = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = delta });
                ok &= expectTrue("cadence frame valid", last.valid);
                ok &= expectFalse("cadence frame continuous", last.discontinuity);
                ok &= expectNear("cadence delta passthrough", last.deltaSeconds, delta, 1.0e-9);
            }
            // One nominal second of frames accumulates one second of game time.
            ok &= expectNear("cadence elapsed one second", last.elapsedGameSeconds, static_cast<double>(frames) * delta, 1.0e-4);
        }
    }

    // Equal-elapsed-time invariance: 90 frames at 90 FPS and 45 frames at
    // 45 FPS accumulate the same cumulative game time within tolerance.
    {
        GameFrameClockState state90{};
        GameFrameClockState state45{};
        (void)advanceGameFrameClock(state90, GameFrameSampleInput{});
        (void)advanceGameFrameClock(state45, GameFrameSampleInput{});
        GameFrameTiming last90{};
        GameFrameTiming last45{};
        for (int i = 0; i < 90; ++i) {
            last90 = advanceGameFrameClock(state90, GameFrameSampleInput{ .rawDeltaSeconds = 1.0f / 90.0f });
        }
        for (int i = 0; i < 45; ++i) {
            last45 = advanceGameFrameClock(state45, GameFrameSampleInput{ .rawDeltaSeconds = 1.0f / 45.0f });
        }
        ok &= expectNear("rate-invariant elapsed", last90.elapsedGameSeconds, last45.elapsedGameSeconds, 1.0e-4);
    }

    // Invalid raw samples: zero, negative, NaN, infinity.
    {
        GameFrameClockState state{};
        (void)advanceGameFrameClock(state, GameFrameSampleInput{});
        (void)advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.02f });
        const double elapsedBefore = state.elapsedGameSeconds;

        const float invalidValues[] = {
            0.0f,
            -0.01f,
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(),
        };
        for (const float raw : invalidValues) {
            const auto timing = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = raw });
            ok &= expectFalse("invalid sample invalid", timing.valid);
            ok &= expectTrue("invalid sample discontinuity", timing.discontinuity);
            ok &= expectNear("invalid sample zero delta", timing.deltaSeconds, 0.0, 0.0);
            ok &= expectNear("invalid sample frozen elapsed", timing.elapsedGameSeconds, elapsedBefore, 0.0);
        }
        ok &= expectEqualU64("invalid sample count", state.invalidSampleCount, 5);
    }

    // Hitches: 100 ms is the last ordinary delta; 250 ms is a valid
    // discontinuity clamped to the ordinary bound.
    {
        GameFrameClockState state{};
        (void)advanceGameFrameClock(state, GameFrameSampleInput{});

        const auto boundary = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.1f });
        ok &= expectTrue("100ms frame valid", boundary.valid);
        ok &= expectFalse("100ms frame continuous", boundary.discontinuity);
        ok &= expectNear("100ms delta passthrough", boundary.deltaSeconds, 0.1f, 1.0e-9);

        const auto hitch = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.25f });
        ok &= expectTrue("250ms hitch valid", hitch.valid);
        ok &= expectTrue("250ms hitch discontinuity", hitch.discontinuity);
        ok &= expectNear("250ms hitch clamped", hitch.deltaSeconds, kMaxOrdinaryDeltaSeconds, 1.0e-9);
        ok &= expectNear("250ms hitch raw preserved", hitch.rawDeltaSeconds, 0.25, 1.0e-9);
        ok &= expectNear("hitch elapsed bounded", hitch.elapsedGameSeconds, 0.1 + kMaxOrdinaryDeltaSeconds, 1.0e-6);
    }

    // Menu pause: samples stay valid, cumulative game time freezes.
    {
        GameFrameClockState state{};
        (void)advanceGameFrameClock(state, GameFrameSampleInput{});
        (void)advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.02f });
        const double elapsedBefore = state.elapsedGameSeconds;
        for (int i = 0; i < 10; ++i) {
            const auto timing = advanceGameFrameClock(state, GameFrameSampleInput{
                .rawDeltaSeconds = 0.011f,
                .menuPaused = true,
            });
            ok &= expectTrue("paused frame valid", timing.valid);
            ok &= expectTrue("paused frame flagged", timing.menuPaused);
            ok &= expectNear("paused frame delta reported", timing.deltaSeconds, 0.011, 1.0e-6);
            ok &= expectNear("paused elapsed frozen", timing.elapsedGameSeconds, elapsedBefore, 0.0);
        }
        const auto resumed = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.011f });
        ok &= expectTrue("resumed frame valid", resumed.valid);
        ok &= expectNear("resumed elapsed advances", resumed.elapsedGameSeconds, elapsedBefore + 0.011, 1.0e-6);
    }

    // Reset: first-frame semantics return; sequence and cumulative time stay
    // monotonic so stored timestamps never see the clock rewind.
    {
        GameFrameClockState state{};
        (void)advanceGameFrameClock(state, GameFrameSampleInput{});
        for (int i = 0; i < 5; ++i) {
            (void)advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.02f });
        }
        const double elapsedBefore = state.elapsedGameSeconds;
        const std::uint64_t sequenceBefore = state.sequence;

        resetGameFrameClock(state);
        const auto firstAfterReset = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.02f });
        ok &= expectFalse("post-reset first frame invalid", firstAfterReset.valid);
        ok &= expectTrue("post-reset first frame discontinuity", firstAfterReset.discontinuity);
        ok &= expectEqualU64("post-reset sequence monotonic", firstAfterReset.sequence, sequenceBefore + 1);
        ok &= expectNear("post-reset elapsed monotonic", firstAfterReset.elapsedGameSeconds, elapsedBefore, 0.0);

        const auto secondAfterReset = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = 0.02f });
        ok &= expectTrue("post-reset second frame valid", secondAfterReset.valid);
        ok &= expectNear("post-reset accumulation resumes", secondAfterReset.elapsedGameSeconds, elapsedBefore + 0.02, 1.0e-6);
    }

    // Irregular pacing keeps the cumulative clock equal to the sum of the
    // measured deltas (72/90 Hz jitter schedule).
    {
        GameFrameClockState state{};
        (void)advanceGameFrameClock(state, GameFrameSampleInput{});
        const float jitterDeltas[] = { 0.0134f, 0.0142f, 0.0139f, 0.0108f, 0.0115f, 0.0111f };
        double expected = 0.0;
        GameFrameTiming last{};
        for (const float delta : jitterDeltas) {
            last = advanceGameFrameClock(state, GameFrameSampleInput{ .rawDeltaSeconds = delta });
            expected += delta;
            ok &= expectTrue("jitter frame valid", last.valid);
        }
        ok &= expectNear("jitter elapsed exact", last.elapsedGameSeconds, expected, 1.0e-6);
    }

    return ok ? 0 : 1;
}
