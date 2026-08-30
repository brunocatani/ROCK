#pragma once

namespace rock::native_vats_input_suppression_policy
{
    struct RuntimeState
    {
        bool suppressVatsOnRelease{ false };
        bool suppressVansWhileDown{ false };
    };

    struct Input
    {
        bool buttonDown{ false };
        bool justPressed{ false };
        bool released{ false };
        bool suppressVats{ false };
        bool suppressVans{ false };
        bool suppressAll{ false };
    };

    struct Decision
    {
        bool forwardNative{ true };
        bool vatsSuppressed{ false };
        bool vansSuppressed{ false };
        const char* reason{ "native" };
    };

    inline void reset(RuntimeState& state)
    {
        state = {};
    }

    /*
     * Bethesda's native helper has two orthogonal phases: button-down samples
     * can start V.A.N.S. after its hold threshold, while the release sample
     * opens ordinary VATS. Suppression acquired during either phase is latched
     * through that physical gesture so a short provider lease cannot expire
     * between press and release and leak an action into the game.
     */
    [[nodiscard]] inline Decision update(RuntimeState& state, const Input& input)
    {
        Decision decision{};

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
            const bool suppressVats =
                state.suppressVatsOnRelease ||
                input.suppressVats ||
                input.suppressAll;
            reset(state);
            if (suppressVats) {
                decision.forwardNative = false;
                decision.vatsSuppressed = true;
                decision.reason = input.suppressAll ?
                    "all-suppression-release" :
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
