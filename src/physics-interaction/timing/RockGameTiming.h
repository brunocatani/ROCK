#pragma once

#include "physics-interaction/timing/GameFrameTimingPolicy.h"

namespace rock::game_timing
{
    using game_frame_timing_policy::GameFrameTiming;

    /*
     * Runtime owner of the game-frame clock domain. Samples the monotonic
     * clock exactly once per game frame and publishes the immutable snapshot
     * every game-frame consumer shares.
     *
     * Thread model: all functions run on the game thread only, matching the
     * existing runtime_state ownership. The snapshot is plain state with no
     * synchronization; do not read it from physics or worker threads.
     */

    // Call once per game frame from the owning game-loop callback before any
    // consumer reads currentFrameTiming().
    const GameFrameTiming& beginGameFrame(bool menuPaused);

    [[nodiscard]] const GameFrameTiming& currentFrameTiming();

    // Drops the measurement reference so the next frame reports first-frame
    // semantics. Sequence and cumulative time stay monotonic.
    void resetForNewSession();

    struct GameTimingTelemetry
    {
        std::uint64_t sequence = 0;
        float rawDeltaSeconds = 0.0f;
        float deltaSeconds = 0.0f;
        double elapsedGameSeconds = 0.0;
        std::uint64_t discontinuityCount = 0;
        std::uint64_t invalidSampleCount = 0;
        bool valid = false;
        bool menuPaused = false;
    };

    [[nodiscard]] GameTimingTelemetry telemetry();
}
