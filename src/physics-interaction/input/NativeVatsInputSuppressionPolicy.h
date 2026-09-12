#pragma once

#include <algorithm>
#include <cmath>

namespace rock::native_vats_input_suppression_policy
{
    inline constexpr float kDefaultHoldSeconds = 0.25f;
    inline constexpr float kMinimumHoldSeconds = 0.05f;
    inline constexpr float kMaximumHoldSeconds = 10.0f;
    inline float sanitizedHoldSeconds(float value) {
        return std::isfinite(value) ? std::clamp(value,kMinimumHoldSeconds,kMaximumHoldSeconds) : kDefaultHoldSeconds;
    }
    inline float sanitizedHeldSeconds(float value) {
        return std::isfinite(value) && value >= 0.0f ? value : 0.0f;
    }
    struct RuntimeState
    {
        bool suppressVatsOnRelease{ false };
        bool suppressVansWhileDown{ false };
        bool reserveHoldGesture{ false };
        bool holdThresholdReached{ false };
    };

    struct Input
    {
        bool buttonDown{ false };
        bool justPressed{ false };
        bool released{ false };
        float heldSeconds{ 0.0f };
        float holdSeconds{
            kDefaultHoldSeconds
        };
        bool suppressVats{ false };
        bool suppressVans{ false };
        bool reserveHoldGesture{ false };
        bool suppressAll{ false };
    };

    struct Decision
    {
        bool forwardNative{ true };
        bool vatsSuppressed{ false };
        bool vansSuppressed{ false };
        bool holdReleaseSuppressed{ false };
        const char* reason{ "native" };
    };

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    /*
     * Bethesda's native helper has two phases: button-down samples can start
     * V.A.N.S. after its hold threshold, while every non-negative release
     * sample opens ordinary VATS regardless of how long the button was held.
     * Provider phase suppression remains independent. ROCK's reserved B-button
     * hold additionally consumes release only after the same threshold, making
     * tap-VATS and the held UI gesture mutually exclusive.
     */
    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{};
        const float heldSeconds =
            sanitizedHeldSeconds(
                input.heldSeconds);
        const float holdSeconds =
            sanitizedHoldSeconds(
                input.holdSeconds);

        if (input.buttonDown) {
            if (input.justPressed) {
                // A menu can prevent the prior physical release from reaching
                // this native helper. Never transfer that stale latch into a
                // newly observed gesture.
                reset(state);
            }
            state.suppressVatsOnRelease =
                state.suppressVatsOnRelease ||
                input.suppressVats ||
                input.suppressAll;
            state.suppressVansWhileDown =
                state.suppressVansWhileDown ||
                input.suppressVans ||
                input.suppressAll;
            state.reserveHoldGesture =
                state.reserveHoldGesture ||
                input.reserveHoldGesture;
            state.holdThresholdReached =
                state.holdThresholdReached ||
                (state.reserveHoldGesture && heldSeconds >= holdSeconds);

            if (state.suppressVansWhileDown) {
                decision.forwardNative = false;
                decision.vansSuppressed = true;
                decision.reason = input.suppressAll ?
                    "all-suppression-held" :
                    "vans-suppression-held";
            } else if (state.suppressVatsOnRelease) {
                decision.reason = "native-vans-vats-release-armed";
            }
            return decision;
        }

        if (input.released) {
            const bool suppressHeldGestureRelease =
                (state.reserveHoldGesture || input.reserveHoldGesture) &&
                (state.holdThresholdReached || heldSeconds >= holdSeconds);
            const bool suppressVats =
                state.suppressVatsOnRelease ||
                input.suppressVats ||
                input.suppressAll ||
                suppressHeldGestureRelease;
            reset(state);
            if (suppressVats) {
                decision.forwardNative = false;
                decision.vatsSuppressed = true;
                decision.holdReleaseSuppressed =
                    suppressHeldGestureRelease;
                decision.reason = input.suppressAll ?
                    "all-suppression-release" :
                    suppressHeldGestureRelease ?
                        "reserved-hold-release" :
                        "vats-suppression-release";
            } else {
                decision.reason = "native-vats-release";
            }
            return decision;
        }

        if (input.suppressAll) {
            state.suppressVatsOnRelease = true;
            state.suppressVansWhileDown = true;
            decision.forwardNative = false;
            decision.vatsSuppressed = true;
            decision.vansSuppressed = true;
            decision.reason = "all-suppression-unclassified";
        } else if (state.suppressVansWhileDown) {
            decision.forwardNative = false;
            decision.vansSuppressed = true;
            decision.reason = "vans-suppression-unclassified";
        }
        return decision;
    }
}
