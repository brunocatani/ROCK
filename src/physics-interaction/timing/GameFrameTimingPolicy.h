#pragma once

#include <cmath>
#include <cstdint>

namespace rock::game_frame_timing_policy
{
    /*
     * Game-frame clock domain policy for the timing-normalization project.
     *
     * One immutable GameFrameTiming snapshot is created per game-loop callback
     * and shared by every game-frame consumer (animation phases, ROCK update,
     * provider publication). The policy never fabricates a nominal-rate delta:
     * an unmeasurable frame stays valid=false with deltaSeconds=0 and each
     * consumer applies its own fail-closed response.
     *
     * Sample classification:
     * - ordinary frame: raw delta in (0, kMaxOrdinaryDeltaSeconds]
     *   -> valid=true, discontinuity=false, delta=raw.
     * - hitch: raw delta above kMaxOrdinaryDeltaSeconds. Real time passed, so
     *   the sample stays valid with the delta clamped to the ordinary bound,
     *   but discontinuity=true tells estimators and smoothers to rebase
     *   instead of interpolating across the gap.
     * - invalid: non-finite or non-positive raw delta, or the first frame
     *   after initialization/reset -> valid=false, delta=0, discontinuity=true.
     *
     * elapsedGameSeconds is the cumulative gameplay clock used to timestamp
     * freshness and dwell state. It advances only for valid samples while the
     * game is not menu-paused, so stored timestamps stay fresh through menus
     * exactly like the physics clock (which stops stepping during menus). It
     * is monotonic across session resets: rewinding it would make every
     * stored timestamp compute a negative age after a game load.
     */
    inline constexpr float kMaxOrdinaryDeltaSeconds = 0.1f;

    struct GameFrameTiming
    {
        std::uint64_t sequence = 0;
        float rawDeltaSeconds = 0.0f;
        float deltaSeconds = 0.0f;
        double elapsedGameSeconds = 0.0;
        bool valid = false;
        bool discontinuity = false;
        bool menuPaused = false;
    };

    struct GameFrameClockState
    {
        std::uint64_t sequence = 0;
        double elapsedGameSeconds = 0.0;
        bool hasReference = false;
        std::uint64_t discontinuityCount = 0;
        std::uint64_t invalidSampleCount = 0;
    };

    struct GameFrameSampleInput
    {
        // Measured monotonic delta since the previous frame. Ignored while the
        // clock has no reference timestamp (first frame, or first frame after
        // a reset).
        float rawDeltaSeconds = 0.0f;
        bool menuPaused = false;
    };

    [[nodiscard]] inline GameFrameTiming advanceGameFrameClock(GameFrameClockState& state, const GameFrameSampleInput& input)
    {
        GameFrameTiming timing{};
        timing.sequence = ++state.sequence;
        timing.menuPaused = input.menuPaused;

        if (!state.hasReference) {
            state.hasReference = true;
            timing.valid = false;
            timing.discontinuity = true;
            ++state.discontinuityCount;
            ++state.invalidSampleCount;
            timing.elapsedGameSeconds = state.elapsedGameSeconds;
            return timing;
        }

        timing.rawDeltaSeconds = input.rawDeltaSeconds;
        if (!std::isfinite(input.rawDeltaSeconds) || input.rawDeltaSeconds <= 0.0f) {
            timing.deltaSeconds = 0.0f;
            timing.valid = false;
            timing.discontinuity = true;
            ++state.discontinuityCount;
            ++state.invalidSampleCount;
        } else if (input.rawDeltaSeconds > kMaxOrdinaryDeltaSeconds) {
            timing.deltaSeconds = kMaxOrdinaryDeltaSeconds;
            timing.valid = true;
            timing.discontinuity = true;
            ++state.discontinuityCount;
        } else {
            timing.deltaSeconds = input.rawDeltaSeconds;
            timing.valid = true;
            timing.discontinuity = false;
        }

        if (timing.valid && !timing.menuPaused) {
            state.elapsedGameSeconds += timing.deltaSeconds;
        }
        timing.elapsedGameSeconds = state.elapsedGameSeconds;
        return timing;
    }

    inline void resetGameFrameClock(GameFrameClockState& state)
    {
        // Sequence and cumulative time deliberately survive the reset (see the
        // header comment); only the measurement reference is dropped so the
        // next frame reports first-frame semantics.
        state.hasReference = false;
    }
}
