#include "physics-interaction/input/GrabInputIntentPolicy.h"

#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectState(const char* label, rock::grab_input_intent_policy::State actual, rock::grab_input_intent_policy::State expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %s got %s\n",
            label,
            rock::grab_input_intent_policy::stateName(expected),
            rock::grab_input_intent_policy::stateName(actual));
        return false;
    }
}

int main()
{
    using namespace rock::grab_input_intent_policy;

    bool ok = true;
    Config config{};
    config.enabled = true;
    config.leewaySeconds = 0.12f;
    config.forceSeconds = 0.08f;

    RuntimeState state{};
    auto decision = update(state, RawButtonState{ .held = true, .pressed = true }, false, false, 1.0f / 90.0f, config);
    ok &= expectFalse("raw press is latched while consumer is not ready", decision.pressed);
    ok &= expectTrue("press remains pending during leeway", decision.pendingPress);
    ok &= expectState("leeway state after early press", decision.state, State::Leeway);

    decision = update(state, RawButtonState{ .held = true }, true, false, 1.0f / 90.0f, config);
    ok &= expectTrue("latched press delivers when consumer becomes ready", decision.pressed);
    ok &= expectTrue("delivered press is marked synthetic", decision.syntheticPressed);
    ok &= expectState("state blocks repeats after delivery", decision.state, State::Blocked);

    decision = update(state, RawButtonState{ .held = true }, true, false, 1.0f / 90.0f, config);
    ok &= expectFalse("held button does not repeat after latched delivery", decision.pressed);
    ok &= expectState("held after delivery stays blocked", decision.state, State::Blocked);

    decision = update(state, RawButtonState{ .released = true }, true, false, 1.0f / 90.0f, config);
    ok &= expectTrue("release passes through immediately", decision.released);
    ok &= expectState("release resets intent", decision.state, State::Idle);

    RuntimeState forceState{};
    decision = update(forceState, RawButtonState{ .held = true, .pressed = true }, false, false, 0.13f, config);
    ok &= expectFalse("first late frame does not force before leeway is consumed", decision.pressed);
    decision = update(forceState, RawButtonState{ .held = true }, false, false, 1.0f / 90.0f, config);
    ok &= expectTrue("expired leeway enters force window", decision.pressed);
    ok &= expectTrue("force window press is synthetic", decision.syntheticPressed);
    ok &= expectState("force state is observable", decision.state, State::Force);

    RuntimeState disabledState{};
    Config disabled = config;
    disabled.enabled = false;
    decision = update(disabledState, RawButtonState{ .held = true, .pressed = true }, false, false, 1.0f / 90.0f, disabled);
    ok &= expectTrue("disabled policy preserves raw press", decision.pressed);
    ok &= expectState("disabled policy leaves runtime idle", decision.state, State::Idle);

    RuntimeState resetState{};
    decision = update(resetState, RawButtonState{ .held = true, .pressed = true }, false, false, 1.0f / 90.0f, config);
    ok &= expectTrue("reset setup has pending press", decision.pendingPress);
    decision = update(resetState, RawButtonState{ .held = true }, true, true, 1.0f / 90.0f, config);
    ok &= expectFalse("explicit reset drops pending synthetic press", decision.pressed);
    ok &= expectState("explicit reset leaves idle", decision.state, State::Idle);

    return ok ? 0 : 1;
}
