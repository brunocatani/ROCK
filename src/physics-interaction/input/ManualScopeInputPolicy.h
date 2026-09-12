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
        NativeActivation,
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
        bool primaryPressSeenByNative{ false };
        bool primaryPressSeenByRaw{ false };
        bool primaryPressUsesNative{ false };
        bool immersiveScopesEnabled{ true };
    };

    struct Input
    {
        bool gameplayInputAllowed{ false };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool firingHandIsLeft{ false };
        bool nativeActivationTarget{ false };
        bool immersiveScopesEnabled{ true };
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
        const bool enabled = state.immersiveScopesEnabled;
        state = {};
        state.immersiveScopesEnabled = enabled;
    }

    inline void blockUntilRelease(RuntimeState& state)
    {
        reset(state);
        state.state = State::BlockedUntilRelease;
    }

    // Retain the press decision through release: a complete tap can reach the
    // raw consumer before the native event hook sees its press (or vice versa).
    inline void finishGesture(RuntimeState& state)
    {
        state.state = State::Idle;
        state.heldSeconds = 0.0f;
        state.gestureHandIsLeft = false;
    }

    // Each consumer sees one press edge per gesture. Its next edge starts a
    // new decision; the other consumer joins the existing one without changing
    // ownership or restarting an already-completed tap.
    inline void beginPrimaryActivateGesture(RuntimeState& state, bool nativeActivationTarget, bool nativeEvent = true)
    {
        if (nativeEvent ? state.primaryPressSeenByNative : state.primaryPressSeenByRaw) {
            state.primaryPressSeenByNative = false;
            state.primaryPressSeenByRaw = false;
        }
        const bool firstConsumer = !state.primaryPressSeenByNative && !state.primaryPressSeenByRaw;
        if (firstConsumer) {
            state.primaryPressUsesNative = state.state == State::Idle && nativeActivationTarget;
            if (nativeEvent && state.state == State::Idle) {
                state.state = state.primaryPressUsesNative ? State::NativeActivation : State::Pending;
                state.heldSeconds = 0.0f;
                state.gestureHandIsLeft = false;
            }
        }
        (nativeEvent ? state.primaryPressSeenByNative : state.primaryPressSeenByRaw) = true;
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
        if (state.immersiveScopesEnabled != input.immersiveScopesEnabled) {
            blockUntilRelease(state);
            state.immersiveScopesEnabled = input.immersiveScopesEnabled;
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

        if (!input.immersiveScopesEnabled) {
            // The primary wand returns to its ordinary native event route.
            // ROCK still supplies reload for a left-hand firing grip, once
            // on press. Neither hold nor release can request a scope.
            decision.dispatchReload = input.firingHandIsLeft &&
                input.leftButton.available && input.leftButton.pressed;
            decision.reason = "vanilla-scopes";
            return decision;
        }

        if (!input.firingHandIsLeft && input.rightButton.available && input.rightButton.pressed) {
            beginPrimaryActivateGesture(state, input.nativeActivationTarget, false);
        }

        if (state.state == State::NativeActivation) {
            const auto& button = input.rightButton;
            if (!button.available) {
                blockUntilRelease(state);
            } else if (button.released || !button.held) {
                finishGesture(state);
            }
            decision.state = state.state;
            decision.reason = "native-activation";
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

            // The primary/right wand owns native Activate. Its use target
            // takes priority over reload/scope for this complete press.
            if (!input.firingHandIsLeft && state.primaryPressUsesNative) {
                if (firingButton.held && !firingButton.released) {
                    state.state = State::NativeActivation;
                    state.gestureHandIsLeft = false;
                }
                decision.state = state.state;
                decision.reason = "native-activation-press";
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
            finishGesture(state);
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
