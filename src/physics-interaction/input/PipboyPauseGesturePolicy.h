#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::pipboy_pause_gesture_policy
{
    inline constexpr float kDefaultHoldSeconds = 0.35f;
    inline constexpr float kMinimumHoldSeconds = 0.15f;
    inline constexpr float kMaximumHoldSeconds = 2.0f;

    enum class State : std::uint8_t
    {
        Idle,
        Pending,
        PauseCommitted,
        BlockedUntilRelease,
    };

    struct RuntimeState
    {
        State state{ State::Idle };
    };

    struct Input
    {
        bool enabled{ true };
        bool eligible{ true };
        bool pressed{ false };
        bool held{ false };
        bool released{ false };
        bool pipboyDispatchAllowed{ true };
        float heldSeconds{ 0.0f };
        float holdSeconds{ kDefaultHoldSeconds };
    };

    struct Decision
    {
        State state{ State::Idle };
        bool consume{ false };
        bool dispatchPipboy{ false };
        bool dispatchPause{ false };
        const char* reason{ "idle" };
    };

    [[nodiscard]] inline float sanitizedHoldSeconds(const float value)
    {
        return std::isfinite(value) ? std::clamp(value, kMinimumHoldSeconds, kMaximumHoldSeconds) : kDefaultHoldSeconds;
    }

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{ .state = state.state };

        if (!input.enabled) {
            reset(state);
            decision.state = state.state;
            decision.reason = "disabled";
            return decision;
        }

        if (!input.eligible) {
            if (state.state == State::Idle) {
                decision.reason = "ineligible";
                return decision;
            }

            decision.consume = true;
            if (input.released || !input.held) {
                reset(state);
                decision.reason = "ineligible-release";
            } else {
                state.state = State::BlockedUntilRelease;
                decision.reason = "ineligible-held";
            }
            decision.state = state.state;
            return decision;
        }

        const float threshold = sanitizedHoldSeconds(input.holdSeconds);
        switch (state.state) {
        case State::Idle:
            if (input.pressed) {
                state.state = State::Pending;
                decision.consume = true;
                decision.reason = "press";
            } else if (input.held) {
                state.state = State::BlockedUntilRelease;
                decision.consume = true;
                decision.reason = "untracked-hold";
            } else if (input.released) {
                decision.consume = true;
                decision.reason = "untracked-release";
            }
            break;

        case State::Pending:
            decision.consume = true;
            if (input.released) {
                if (input.heldSeconds >= threshold) {
                    decision.dispatchPause = true;
                    decision.reason = "release-after-threshold";
                } else if (input.pipboyDispatchAllowed) {
                    decision.dispatchPipboy = true;
                    decision.reason = "short-release";
                } else {
                    decision.reason = "short-release-blocked";
                }
                reset(state);
            } else if (!input.held) {
                state.state = State::BlockedUntilRelease;
                decision.reason = "lost-level";
            } else if (input.heldSeconds >= threshold) {
                state.state = State::PauseCommitted;
                decision.dispatchPause = true;
                decision.reason = "hold-threshold";
            } else {
                decision.reason = "pending";
            }
            break;

        case State::PauseCommitted:
            decision.consume = true;
            if (input.released || !input.held) {
                reset(state);
                decision.reason = "pause-release";
            } else {
                decision.reason = "pause-committed";
            }
            break;

        case State::BlockedUntilRelease:
            decision.consume = true;
            if (input.released || !input.held) {
                reset(state);
                decision.reason = "blocked-release";
            } else {
                decision.reason = "blocked";
            }
            break;
        }

        decision.state = state.state;
        return decision;
    }
}
