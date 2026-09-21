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
    {
        RuntimeState retry{};
        const Target target{ 0x1234, 42 };
        decision = update(retry, { .held = true, .pressed = true }, true, false, 0.01f, config, target);
        ok &= expectTrue("close grab starts immediately", decision.pressed);
        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, true, false, 0.01f, config, target);
        ok &= expectFalse("contact retry bounds repeated mesh work", decision.pressed);
        decision = update(retry, { .held = true }, true, false, 0.05f, config, target);
        ok &= expectTrue("same held squeeze retries recoverable contact", decision.pressed && decision.syntheticPressed);
        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, true, false, 2.0f, config, target);
        ok &= expectTrue("same-target intent does not expire", decision.pressed);
        decision = update(retry, { .held = true }, true, false, 0.1f, config, target);
        ok &= expectFalse("terminal refusal does not retry without explicit retention", decision.pressed);

        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, true, false, 0.1f, config, Target{ 0x5678, 42 });
        ok &= expectFalse("retry never transfers to another reference", decision.pressed);
        decision = update(retry, { .held = true }, true, false, 0.1f, config, target);
        ok &= expectFalse("returning to lost target needs new input", decision.pressed);
        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, true, false, 0.1f, config, Target{ 0x1234, 43 });
        ok &= expectFalse("replaced body cancels retry", decision.pressed);
        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, false, false, 0.1f, config);
        ok &= expectFalse("selection loss cancels retry", retry.retryTarget.valid());
        retainContactRetry(retry, target);
        decision = update(retry, { .released = true }, true, false, 0.1f, config, target);
        ok &= expectTrue("retry release passes through immediately", decision.released);
        ok &= expectFalse("release clears retry", retry.retryTarget.valid());
        retainContactRetry(retry, target);
        decision = update(retry, { .held = true }, true, true, 0.1f, config, target);
        ok &= expectFalse("successful hold or lifecycle reset clears retry", retry.retryTarget.valid());
    }
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

        rock::peer_held_join_retry_policy::RuntimeState unrelatedSelectionState{};
        retryDecision = rock::peer_held_join_retry_policy::update(
            unrelatedSelectionState,
            Input{
                .rawHeld = true,
                .rawPressed = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .unrelatedSelection = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectFalse("peer-held retry does not start with unrelated selection", retryDecision.started);
        ok &= expectFalse("peer-held retry remains inactive with unrelated selection", unrelatedSelectionState.active);

        retryDecision = rock::peer_held_join_retry_policy::update(
            unrelatedSelectionState,
            Input{
                .rawHeld = true,
                .rawPressed = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry starts before unrelated selection appears", unrelatedSelectionState.active);
        retryDecision = rock::peer_held_join_retry_policy::update(
            unrelatedSelectionState,
            Input{
                .rawHeld = true,
                .peerHoldingLooseObject = true,
                .peerStillHoldingSameObject = true,
                .unrelatedSelection = true,
                .peerFormId = 0x1234,
                .deltaSeconds = 1.0f / 90.0f,
                .config = retryConfig,
            });
        ok &= expectTrue("peer-held retry cancels when unrelated selection appears", retryDecision.cancelled);
        ok &= expectFalse("peer-held retry inactive after unrelated selection", unrelatedSelectionState.active);

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
