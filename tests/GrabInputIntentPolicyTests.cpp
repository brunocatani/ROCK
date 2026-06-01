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

    RuntimeState peerIntentState{};
    decision = update(peerIntentState, RawButtonState{ .held = true, .pressed = true }, false, false, 1.0f / 90.0f, config);
    ok &= expectFalse("peer-held miss keeps original press pending", decision.pressed);
    decision = update(peerIntentState, RawButtonState{ .held = true }, true, false, 1.0f / 90.0f, config);
    ok &= expectTrue("peer-held acquired close selection receives latched press", decision.pressed);
    ok &= expectTrue("peer-held acquired press is synthetic after initial miss", decision.syntheticPressed);

    {
        using namespace rock::peer_held_join_retry_policy;
        rock::peer_held_join_retry_policy::Config retryConfig{};
        retryConfig.enabled = true;
        retryConfig.leewaySeconds = 0.12f;
        retryConfig.forceSeconds = 0.08f;
        retryConfig.retryIntervalSeconds = 0.05f;

        rock::peer_held_join_retry_policy::RuntimeState retryState{};
        auto retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .rawPressed = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry starts from held press", retryDecision.started);
        ok &= expectTrue("peer-held retry attempts immediately on start", retryDecision.attempt);
        ok &= expectTrue("peer-held retry remains active after start", retryState.active);

        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 0.02f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry stays active while held", retryState.active);
        ok &= expectFalse("peer-held retry is rate limited between attempts", retryDecision.attempt);

        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = false,
                .rawReleased = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry cancels on release", retryDecision.releaseCancel);
        ok &= expectFalse("peer-held retry inactive after release", retryState.active);

        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .rawPressed = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry restarted for timeout case", retryState.active);
        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 0.30f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry times out inside bounded window", retryDecision.timeout);
        ok &= expectFalse("peer-held retry inactive after timeout", retryState.active);

        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .rawPressed = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry restarted for success case", retryState.active);
        retryDecision = rock::peer_held_join_retry_policy::update(
            retryState,
            Input{
                .rawHeld = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .grabSucceeded = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry reports success cancellation", retryDecision.success);
        ok &= expectFalse("peer-held retry inactive after success", retryState.active);
    }

    return ok ? 0 : 1;
}
