#pragma once

#include <cstdint>

namespace rock::bare_fist_gesture
{
    inline constexpr std::uint64_t kButtons = (1ull << 2) | (1ull << 33);
    inline constexpr std::uint64_t kMaximumSampleAgeMilliseconds = 100;
    inline constexpr float kDefaultHoldSeconds = 0.30f;
    inline constexpr float kMinimumHoldSeconds = 0.1f;
    inline constexpr float kMaximumHoldSeconds = 10.0f;
    inline constexpr float kDrawTimeoutSeconds = 2.0f;

    // Low bits hold capture state; the upper bits identify a physical cycle.
    // A release/repress between game frames must never continue an old session.
    enum class Capture : std::uint64_t { Idle = 0, Holding = 1, Rearming = 2, Draining = 3 };
    [[nodiscard]] constexpr Capture capture(std::uint64_t cycle) noexcept
    {
        return static_cast<Capture>(cycle & 3u);
    }
    [[nodiscard]] constexpr std::uint64_t observe(
        std::uint64_t cycle, bool eligible, bool held, bool released, bool interrupted) noexcept
    {
        switch (capture(cycle)) {
        case Capture::Idle:
            return eligible && held && !interrupted ? ((cycle & ~3ull) + 4u) | 1u : cycle;
        case Capture::Holding:
            if (!eligible || !held || interrupted) return (cycle & ~3ull) | 3u;
            return cycle;
        case Capture::Rearming:
        case Capture::Draining:
            return released ? cycle & ~3ull : cycle;
        }
        return cycle;
    }

    enum class Phase : std::uint8_t { Idle, Qualifying, Drawing, Active };
    struct State
    {
        Phase phase{ Phase::Idle };
        std::uint64_t cycle{ 0 };
        float seconds{ 0.0f };
        float requiredHoldSeconds{ kDefaultHoldSeconds };
    };
    struct Input
    {
        std::uint64_t cycle{ 0 };
        bool eligible{ false };
        bool drawnUnarmed{ false };
        float deltaSeconds{ 0.0f };
        float holdSeconds{ kDefaultHoldSeconds };
    };
    enum class Action { None, Draw, Cancel };

    [[nodiscard]] inline Action update(State& state, const Input& input) noexcept
    {
        if (!input.eligible || capture(input.cycle) != Capture::Holding ||
            (state.phase != Phase::Idle && state.cycle != input.cycle)) {
            const bool started = state.phase != Phase::Idle;
            state = {};
            return started ? Action::Cancel : Action::None;
        }
        if (state.phase == Phase::Idle) {
            // A hot reload affects the next physical hold, never shortens
            // qualification under buttons the player is already holding.
            state = { Phase::Qualifying, input.cycle, 0.0f, input.holdSeconds };
            return Action::None;
        }
        if (state.phase == Phase::Qualifying) {
            state.seconds += input.deltaSeconds;
            if (state.seconds >= state.requiredHoldSeconds) {
                state.phase = Phase::Drawing;
                state.seconds = 0.0f;
                return Action::Draw;
            }
        } else if (state.phase == Phase::Drawing) {
            if (input.drawnUnarmed) {
                state.phase = Phase::Active;
                state.seconds = 0.0f;
            } else {
                state.seconds += input.deltaSeconds;
                if (state.seconds >= kDrawTimeoutSeconds) {
                    state = {};
                    return Action::Cancel;
                }
            }
        } else if (!input.drawnUnarmed) {
            state = {};
            return Action::Cancel;
        }
        return Action::None;
    }
}
