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

namespace rock::peer_held_join_retry_policy
{
    /*
     * Joining the object already held by the peer hand is not a normal free grab
     * edge: selection can be absent for the first frame because the peer hand has
     * already converted the object to ROCK-held dynamic state. Keep a short,
     * held-button retry window so the original press can become a close grab
     * after fresh contact evidence arrives, while rate-limiting expensive mesh
     * and contact validation attempts.
     */
    struct Config
    {
        bool enabled = true;
        float leewaySeconds = 0.12f;
        float forceSeconds = 0.08f;
        float minWindowSeconds = 0.08f;
        float maxWindowSeconds = 0.35f;
        float retryIntervalSeconds = 0.05f;
    };

    struct RuntimeState
    {
        bool active = false;
        float elapsedSeconds = 0.0f;
        float retryElapsedSeconds = 0.0f;
        float windowSeconds = 0.0f;
        std::uint32_t peerFormId = 0;
        std::uint32_t attempts = 0;
        const char* lastRefusalReason = "none";
    };

    struct Input
    {
        bool rawHeld = false;
        bool rawPressed = false;
        bool rawReleased = false;
        bool normalGrabSuppressed = false;
        bool handHolding = false;
        bool peerHoldingLooseObject = false;
        bool peerStillHoldingSameObject = false;
        bool unrelatedSelection = false;
        bool grabSucceeded = false;
        std::uint32_t peerFormId = 0;
        float deltaSeconds = 0.0f;
        Config config{};
    };

    struct Decision
    {
        bool active = false;
        bool started = false;
        bool attempt = false;
        bool cancelled = false;
        bool timeout = false;
        bool releaseCancel = false;
        bool success = false;
        const char* reason = "idle";
    };

    [[nodiscard]] inline float finiteNonNegative(float value, float fallback)
    {
        return std::isfinite(value) && value >= 0.0f ? value : fallback;
    }

    [[nodiscard]] inline float retryWindowSeconds(const Config& config)
    {
        const float leeway = finiteNonNegative(config.leewaySeconds, 0.12f);
        const float force = finiteNonNegative(config.forceSeconds, 0.08f);
        const float minWindow = finiteNonNegative(config.minWindowSeconds, 0.08f);
        const float maxWindow = (std::max)(minWindow, finiteNonNegative(config.maxWindowSeconds, 0.35f));
        return std::clamp(leeway + force, minWindow, maxWindow);
    }

    [[nodiscard]] inline float retryIntervalSeconds(const Config& config)
    {
        return std::clamp(finiteNonNegative(config.retryIntervalSeconds, 0.05f), 0.016f, 0.125f);
    }

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    [[nodiscard]] inline bool canStartRetry(const Input& input)
    {
        return input.config.enabled &&
               input.rawPressed &&
               input.rawHeld &&
               !input.normalGrabSuppressed &&
               !input.handHolding &&
               input.peerHoldingLooseObject &&
               input.peerFormId != 0 &&
               !input.unrelatedSelection;
    }

    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{};

        if (!input.config.enabled) {
            if (state.active) {
                reset(state);
                decision.cancelled = true;
                decision.reason = "disabled";
                return decision;
            }
            decision.reason = "disabled";
            return decision;
        }

        if (state.active && input.grabSucceeded) {
            reset(state);
            decision.cancelled = true;
            decision.success = true;
            decision.reason = "grab-succeeded";
            return decision;
        }

        if (state.active && (input.rawReleased || !input.rawHeld)) {
            reset(state);
            decision.cancelled = true;
            decision.releaseCancel = true;
            decision.reason = input.rawReleased ? "released" : "not-held";
            return decision;
        }

        if (state.active && input.normalGrabSuppressed) {
            reset(state);
            decision.cancelled = true;
            decision.reason = "normal-grab-suppressed";
            return decision;
        }

        if (state.active && input.handHolding) {
            reset(state);
            decision.cancelled = true;
            decision.success = true;
            decision.reason = "hand-holding";
            return decision;
        }

        if (state.active && (!input.peerStillHoldingSameObject || input.peerFormId == 0)) {
            reset(state);
            decision.cancelled = true;
            decision.reason = "peer-release";
            return decision;
        }

        if (state.active && input.unrelatedSelection) {
            reset(state);
            decision.cancelled = true;
            decision.reason = "unrelated-selection";
            return decision;
        }

        if (!state.active && canStartRetry(input)) {
            state.active = true;
            state.elapsedSeconds = 0.0f;
            state.retryElapsedSeconds = retryIntervalSeconds(input.config);
            state.windowSeconds = retryWindowSeconds(input.config);
            state.peerFormId = input.peerFormId;
            state.attempts = 0;
            state.lastRefusalReason = "none";
            decision.started = true;
            decision.reason = "started";
        }

        if (!state.active) {
            decision.reason = "idle";
            return decision;
        }

        const float dt = finiteNonNegative(input.deltaSeconds, 0.0f);
        state.elapsedSeconds += dt;
        state.retryElapsedSeconds += dt;

        if (state.elapsedSeconds > state.windowSeconds) {
            reset(state);
            decision.cancelled = true;
            decision.timeout = true;
            decision.reason = "timeout";
            return decision;
        }

        const float retryInterval = retryIntervalSeconds(input.config);
        if (state.retryElapsedSeconds >= retryInterval) {
            state.retryElapsedSeconds = 0.0f;
            ++state.attempts;
            decision.attempt = true;
        }

        decision.active = state.active;
        if (!decision.started) {
            decision.reason = decision.attempt ? "retry-attempt" : "waiting";
        }
        return decision;
    }
}
