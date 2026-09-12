#include "physics-interaction/timing/RockGameTiming.h"

#include <chrono>

namespace rock::game_timing
{
    namespace
    {
        bool s_hasLastSampleTime = false;
        std::chrono::steady_clock::time_point s_lastSampleTime{};
        game_frame_timing_policy::GameFrameClockState s_clockState{};
        GameFrameTiming s_currentTiming{};
    }

    const GameFrameTiming& beginGameFrame(const bool menuPaused)
    {
        const auto now = std::chrono::steady_clock::now();
        game_frame_timing_policy::GameFrameSampleInput input{
            .menuPaused = menuPaused,
        };
        if (s_hasLastSampleTime) {
            const std::chrono::duration<float> elapsed = now - s_lastSampleTime;
            input.rawDeltaSeconds = elapsed.count();
        } else {
            // No wall reference yet; the policy reports first-frame semantics
            // because the clock state has no reference either.
            s_hasLastSampleTime = true;
        }
        s_lastSampleTime = now;

        s_currentTiming = game_frame_timing_policy::advanceGameFrameClock(s_clockState, input);
        return s_currentTiming;
    }

    const GameFrameTiming& currentFrameTiming()
    {
        return s_currentTiming;
    }

    void resetForNewSession()
    {
        s_hasLastSampleTime = false;
        s_lastSampleTime = {};
        game_frame_timing_policy::resetGameFrameClock(s_clockState);
    }

    GameTimingTelemetry telemetry()
    {
        return GameTimingTelemetry{
            .sequence = s_currentTiming.sequence,
            .rawDeltaSeconds = s_currentTiming.rawDeltaSeconds,
            .deltaSeconds = s_currentTiming.deltaSeconds,
            .elapsedGameSeconds = s_currentTiming.elapsedGameSeconds,
            .discontinuityCount = s_clockState.discontinuityCount,
            .invalidSampleCount = s_clockState.invalidSampleCount,
            .valid = s_currentTiming.valid,
            .menuPaused = s_currentTiming.menuPaused,
        };
    }
}
