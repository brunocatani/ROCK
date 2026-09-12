#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::vats_grenade_gesture_policy
{
    inline constexpr float kDefaultHoldSeconds = 0.25f;
    inline constexpr float kMinimumHoldSeconds = 0.05f;
    inline constexpr float kMaximumHoldSeconds = 10.0f;

    enum class State : std::uint8_t
    {
        Idle,
        Pending,
        HoldCommitted,
        BlockedUntilRelease,
    };

    struct RuntimeState
    {
        State state{ State::Idle };
        bool immersiveGrenades{ true };
    };

    struct Input
    {
        bool eligible{ true };
        bool pressed{ false };
        bool held{ false };
        bool released{ false };
        float heldSeconds{ 0.0f };
        float holdSeconds{ kDefaultHoldSeconds };
        bool immersiveGrenades{ true };
    };

    struct Decision
    {
        State state{ State::Idle };
        bool requestGrenade{ false };
        bool requestNativeThrow{ false };
        bool releaseNativeThrow{ false };
        bool cancelNativeThrow{ false };
        const char* reason{ "idle" };
    };

    [[nodiscard]] inline float sanitizedHoldSeconds(const float value)
    {
        return std::isfinite(value) ?
                   std::clamp(value, kMinimumHoldSeconds, kMaximumHoldSeconds) :
                   kDefaultHoldSeconds;
    }

    [[nodiscard]] inline float sanitizedHeldSeconds(const float value)
    {
        return std::isfinite(value) && value >= 0.0f ? value : 0.0f;
    }

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    /*
     * Primary-wand B is one exclusive gesture. A qualified hold draws an
     * immersive grenade or primes the native throw for physical release.
     * PALM/menu ownership and a mode change cancel the gesture; a button first
     * observed while ineligible or already held must release before rearming.
     */
    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{ .state = state.state };
        const float heldSeconds = sanitizedHeldSeconds(input.heldSeconds);
        const float holdSeconds = sanitizedHoldSeconds(input.holdSeconds);

        // A menu transition can hide the prior release. A genuinely new edge
        // always starts a new physical gesture instead of inheriting stale
        // committed or blocked state.
        if (input.pressed && state.state != State::Idle) {
            decision.cancelNativeThrow = state.state == State::HoldCommitted && !state.immersiveGrenades;
            reset(state);
        }

        const bool modeChanged = state.state != State::Idle && state.immersiveGrenades != input.immersiveGrenades;
        if (!input.eligible || modeChanged) {
            decision.cancelNativeThrow = decision.cancelNativeThrow ||
                (state.state == State::HoldCommitted && !state.immersiveGrenades);
            if (input.held) {
                state.state = State::BlockedUntilRelease;
                decision.reason = "ineligible-held";
            } else {
                reset(state);
                decision.reason = "ineligible";
            }
            decision.state = state.state;
            return decision;
        }

        switch (state.state) {
        case State::Idle:
            if (input.pressed && input.held) {
                state.immersiveGrenades = input.immersiveGrenades;
                if (heldSeconds >= holdSeconds) {
                    state.state = State::HoldCommitted;
                    decision.requestGrenade = state.immersiveGrenades;
                    decision.requestNativeThrow = !state.immersiveGrenades;
                    decision.reason = "press-at-threshold";
                } else {
                    state.state = State::Pending;
                    decision.reason = "press-pending";
                }
            } else if (input.held) {
                state.state = State::BlockedUntilRelease;
                decision.reason = "untracked-hold";
            } else if (input.released) {
                decision.reason = "untracked-release";
            }
            break;

        case State::Pending:
            if (input.released) {
                decision.requestGrenade = heldSeconds >= holdSeconds && state.immersiveGrenades;
                decision.requestNativeThrow = heldSeconds >= holdSeconds && !state.immersiveGrenades;
                decision.releaseNativeThrow = decision.requestNativeThrow;
                decision.reason = heldSeconds >= holdSeconds ?
                    "release-after-threshold" :
                    "short-release";
                reset(state);
            } else if (!input.held) {
                state.state = State::BlockedUntilRelease;
                decision.reason = "lost-level";
            } else if (heldSeconds >= holdSeconds) {
                state.state = State::HoldCommitted;
                decision.requestGrenade = state.immersiveGrenades;
                decision.requestNativeThrow = !state.immersiveGrenades;
                decision.reason = "hold-threshold";
            } else {
                decision.reason = "pending";
            }
            break;

        case State::HoldCommitted:
            if (input.released || !input.held) {
                decision.releaseNativeThrow = input.released && !state.immersiveGrenades;
                decision.cancelNativeThrow = !input.released && !state.immersiveGrenades;
                reset(state);
                decision.reason = "hold-release";
            } else {
                decision.reason = "hold-committed";
            }
            break;

        case State::BlockedUntilRelease:
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
