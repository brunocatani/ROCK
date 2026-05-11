#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::grab_input_intent_policy
{
    /*
     * ROCK keeps grab input alive for a short leeway window so trigger/grip
     * edges are not lost while the hand state moves from far selection, pull, or
     * vanilla-input suppression into a grabbable state. The leeway applies only
     * to ROCK's internal grab edge. Game-facing input filtering remains owned by
     * InputRemapPolicy, and held-object release still follows the raw release
     * edge so a player can always cancel immediately.
     */
    enum class State : std::uint8_t
    {
        Idle,
        Leeway,
        Blocked,
        Force,
    };

    struct Config
    {
        bool enabled = true;
        float leewaySeconds = 0.12f;
        float forceSeconds = 0.08f;
    };

    struct RawButtonState
    {
        bool held = false;
        bool pressed = false;
        bool released = false;
    };

    struct RuntimeState
    {
        State state = State::Idle;
        float leewayRemainingSeconds = 0.0f;
        float forceRemainingSeconds = 0.0f;
        bool pendingPress = false;
    };

    struct Decision
    {
        bool held = false;
        bool pressed = false;
        bool released = false;
        bool syntheticPressed = false;
        bool pendingPress = false;
        State state = State::Idle;
        const char* reason = "idle";
    };

    [[nodiscard]] constexpr const char* stateName(State state)
    {
        switch (state) {
        case State::Idle:
            return "Idle";
        case State::Leeway:
            return "Leeway";
        case State::Blocked:
            return "Blocked";
        case State::Force:
            return "Force";
        }
        return "Unknown";
    }

    [[nodiscard]] inline float finiteNonNegative(float value, float fallback)
    {
        return std::isfinite(value) && value >= 0.0f ? value : fallback;
    }

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    [[nodiscard]] inline Decision update(RuntimeState& state, const RawButtonState& raw, bool pressConsumerReady, bool resetIntent, float deltaSeconds, const Config& config)
    {
        Decision decision{};
        decision.held = raw.held;
        decision.released = raw.released;

        if (!config.enabled || resetIntent) {
            reset(state);
            decision.pressed = raw.pressed;
            decision.state = State::Idle;
            decision.reason = config.enabled ? "reset" : "disabled";
            return decision;
        }

        const float dt = finiteNonNegative(deltaSeconds, 0.0f);
        const float leewaySeconds = finiteNonNegative(config.leewaySeconds, 0.12f);
        const float forceSeconds = finiteNonNegative(config.forceSeconds, 0.08f);

        if (raw.released || !raw.held) {
            reset(state);
            decision.state = State::Idle;
            decision.reason = raw.released ? "released" : "notHeld";
            return decision;
        }

        if (raw.pressed) {
            state.pendingPress = true;
            state.state = State::Leeway;
            state.leewayRemainingSeconds = leewaySeconds;
            state.forceRemainingSeconds = forceSeconds;
        }

        if (!state.pendingPress) {
            state.state = State::Blocked;
            decision.state = state.state;
            decision.reason = "heldAfterDelivery";
            return decision;
        }

        if (pressConsumerReady) {
            state.pendingPress = false;
            state.state = State::Blocked;
            decision.pressed = true;
            decision.syntheticPressed = !raw.pressed;
            decision.state = state.state;
            decision.reason = raw.pressed ? "rawPressReady" : "latchedPressReady";
            return decision;
        }

        if (state.state == State::Leeway) {
            if (state.leewayRemainingSeconds > 0.0f) {
                state.leewayRemainingSeconds = (std::max)(0.0f, state.leewayRemainingSeconds - dt);
                decision.pendingPress = true;
                decision.state = state.state;
                decision.reason = "waitingForConsumer";
                return decision;
            }

            state.state = State::Force;
            state.forceRemainingSeconds = forceSeconds;
        }

        if (state.state == State::Force && state.forceRemainingSeconds > 0.0f) {
            state.forceRemainingSeconds = (std::max)(0.0f, state.forceRemainingSeconds - dt);
            decision.pressed = true;
            decision.syntheticPressed = true;
            decision.pendingPress = true;
            decision.state = state.state;
            decision.reason = "forceWindow";
            return decision;
        }

        state.pendingPress = false;
        state.state = State::Blocked;
        decision.state = state.state;
        decision.reason = "expired";
        return decision;
    }
}
