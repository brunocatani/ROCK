#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::pipboy_pause_gesture_policy
{
    inline constexpr float kDefaultHoldSeconds = 0.35f;
    inline constexpr float kMinimumHoldSeconds = 0.15f;
    inline constexpr float kMaximumHoldSeconds = 2.0f;

    enum class PipboyRoute : std::uint8_t
    {
        Unavailable,
        Native,
        FrikWrist,
    };

    [[nodiscard]] constexpr PipboyRoute selectPipboyRoute(bool settingsAvailable, bool projected, bool attachedToHmd,
        bool inPowerArmor, bool wristBindingReady)
    {
        if (!settingsAvailable) {
            return PipboyRoute::Unavailable;
        }
        if (projected || attachedToHmd || inPowerArmor) {
            return PipboyRoute::Native;
        }
        return wristBindingReady ? PipboyRoute::FrikWrist : PipboyRoute::Unavailable;
    }

    /*
     * Inside a workshop's buildable area the game enters Workshop mode when the
     * same off-hand menu button is held for fEnterWorkshopDelay (1.5 s). There
     * a hold past the Pause threshold only arms Pause: the release opens it,
     * and the game's own Workshop entry, reached first, ends the gesture with
     * nothing dispatched. A tap still opens the Pip-Boy.
     */
    enum class State : std::uint8_t
    {
        Idle,
        Pending,
        PauseCommitted,
        PauseOnRelease,
        BlockedUntilRelease,
    };

    struct RuntimeState
    {
        State state{ State::Idle };
        // The game's Workshop gate passed at this gesture's press.
        bool workshop{ false };
    };

    struct Input
    {
        bool enabled{ true };
        bool eligible{ true };
        bool pressed{ false };
        bool held{ false };
        bool released{ false };
        bool pipboyDispatchAllowed{ true };
        // Read at the press: the game would enter Workshop mode on this hold.
        bool workshopAvailable{ false };
        // The game attempted its Workshop entry on this held event.
        bool workshopEntered{ false };
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

    inline void reset(RuntimeState& state) { state = {}; }

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
        // Menus can consume the release after a Pause hold. A new native
        // press is proof of a new gesture, even if that release never reached
        // us. Held-only input still cannot rearm a gesture across a menu.
        if (input.pressed) {
            decision.reason = state.state == State::Idle ? "press" : "fresh-press-after-missed-release";
            state.state = State::Pending;
            state.workshop = input.workshopAvailable;
            decision.state = state.state;
            decision.consume = true;
            return decision;
        }

        if (input.workshopEntered && (state.state == State::Pending || state.state == State::PauseOnRelease)) {
            state.state = State::BlockedUntilRelease;
            decision.state = state.state;
            decision.consume = true;
            decision.reason = "workshop-entered";
            return decision;
        }

        switch (state.state) {
        case State::Idle:
            if (input.held) {
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
            } else if (input.heldSeconds >= threshold && state.workshop) {
                state.state = State::PauseOnRelease;
                decision.reason = "hold-threshold-workshop-area";
            } else if (input.heldSeconds >= threshold) {
                state.state = State::PauseCommitted;
                decision.dispatchPause = true;
                decision.reason = "hold-threshold";
            } else {
                decision.reason = "pending";
            }
            break;

        case State::PauseOnRelease:
            decision.consume = true;
            if (input.released) {
                decision.dispatchPause = true;
                decision.reason = "release-before-workshop";
                reset(state);
            } else if (!input.held) {
                state.state = State::BlockedUntilRelease;
                decision.reason = "lost-level";
            } else {
                decision.reason = "pause-on-release";
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
