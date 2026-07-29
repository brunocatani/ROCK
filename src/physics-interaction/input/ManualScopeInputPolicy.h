#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::manual_scope_input_policy
{
    enum class State : std::uint8_t
    {
        Idle,
        Pending,
        ScopeHeld,
        BlockedUntilRelease,
    };

    struct ButtonState
    {
        bool available{ false };
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    struct RuntimeState
    {
        State state{ State::Idle };
        float heldSeconds{ 0.0f };
        bool gestureHandIsLeft{ false };
    };

    struct Input
    {
        bool manualModeEnabled{ false };
        bool gameplayInputAllowed{ false };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool firingHandIsLeft{ false };
        ButtonState leftButton{};
        ButtonState rightButton{};
        float deltaSeconds{ 0.0f };
        float holdSeconds{ 0.30f };
    };

    struct Decision
    {
        bool scopeRequested{ false };
        bool dispatchReload{ false };
        State state{ State::Idle };
        const char* reason{ "idle" };
    };

    [[nodiscard]] constexpr const ButtonState& buttonForHand(const Input& input, const bool isLeft)
    {
        return isLeft ? input.leftButton : input.rightButton;
    }

    [[nodiscard]] constexpr bool anyButtonHeld(const Input& input)
    {
        return input.leftButton.held || input.rightButton.held;
    }

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    inline void blockUntilRelease(RuntimeState& state)
    {
        state = {};
        state.state = State::BlockedUntilRelease;
    }

    [[nodiscard]] inline float finiteNonNegative(const float value)
    {
        return std::isfinite(value) && value >= 0.0f ? value : 0.0f;
    }

    [[nodiscard]] inline float sanitizedHoldSeconds(const float value)
    {
        return std::clamp(std::isfinite(value) ? value : 0.30f, 0.05f, 2.0f);
    }

    /*
     * Manual native-scope input owns one complete A/X gesture. A release
     * before the threshold dispatches reload exactly once; crossing the
     * threshold irrevocably converts that gesture into scope ownership until
     * release. Hand changes and invalid gameplay state discard the gesture
     * and require every accept button to return up before rearming, preventing
     * stale edges from transferring across weapon/hand/menu transitions.
     */
    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{};

        if (!input.manualModeEnabled) {
            reset(state);
            decision.reason = "automatic-mode";
            return decision;
        }

        const bool gameplayEligible = input.gameplayInputAllowed && !input.menuInputActive && input.weaponDrawn;
        if (!gameplayEligible) {
            if (anyButtonHeld(input)) {
                blockUntilRelease(state);
                decision.state = state.state;
                decision.reason = "gameplay-blocked-held";
            } else {
                reset(state);
                decision.reason = "gameplay-blocked";
            }
            return decision;
        }

        if (state.state == State::BlockedUntilRelease) {
            if (!anyButtonHeld(input)) {
                reset(state);
                decision.reason = "rearmed";
            } else {
                decision.state = state.state;
                decision.reason = "waiting-for-release";
            }
            return decision;
        }

        if (state.state == State::Idle) {
            const auto& firingButton = buttonForHand(input, input.firingHandIsLeft);
            if (!firingButton.available) {
                decision.reason = "input-unavailable";
                return decision;
            }
            if (!firingButton.pressed) {
                decision.reason = "idle";
                return decision;
            }

            if (firingButton.released || !firingButton.held) {
                decision.dispatchReload = true;
                decision.reason = "tap-release";
                return decision;
            }

            state.state = State::Pending;
            state.heldSeconds = 0.0f;
            state.gestureHandIsLeft = input.firingHandIsLeft;
            decision.state = state.state;
            decision.reason = "press-pending";
            return decision;
        }

        if (state.gestureHandIsLeft != input.firingHandIsLeft) {
            blockUntilRelease(state);
            decision.state = state.state;
            decision.reason = "firing-hand-changed";
            return decision;
        }

        const auto& gestureButton = buttonForHand(input, state.gestureHandIsLeft);
        if (!gestureButton.available) {
            blockUntilRelease(state);
            decision.state = state.state;
            decision.reason = "input-lost";
            return decision;
        }

        if (gestureButton.released || !gestureButton.held) {
            const bool pendingReload = state.state == State::Pending;
            reset(state);
            decision.dispatchReload = pendingReload;
            decision.reason = pendingReload ? "pending-release-reload" : "scope-release";
            return decision;
        }

        if (state.state == State::Pending) {
            state.heldSeconds += finiteNonNegative(input.deltaSeconds);
            if (state.heldSeconds >= sanitizedHoldSeconds(input.holdSeconds)) {
                state.state = State::ScopeHeld;
                decision.scopeRequested = true;
                decision.state = state.state;
                decision.reason = "hold-threshold";
                return decision;
            }

            decision.state = state.state;
            decision.reason = "pending";
            return decision;
        }

        decision.scopeRequested = true;
        decision.state = state.state;
        decision.reason = "scope-held";
        return decision;
    }
}
