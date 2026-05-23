#include <cstdint>
#include <cstdio>

#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"

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

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected equality\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::contact_signal_subscription_policy;

    bool ok = true;

    const ContactSignalSubscriptionSnapshot inactive{};
    auto plan = planSubscription(inactive, 0, 0x2000);
    ok &= expectEqual("null world is ignored", plan.action, ContactSignalSubscriptionAction::IgnoreNullSignal);
    ok &= expectFalse("null world does not subscribe", plan.subscribeRequestedSignal);

    plan = planSubscription(inactive, 0x1000, 0x2000);
    ok &= expectEqual("inactive runtime subscribes fresh", plan.action, ContactSignalSubscriptionAction::SubscribeFresh);
    ok &= expectTrue("fresh plan subscribes requested signal", plan.subscribeRequestedSignal);
    ok &= expectFalse("fresh plan does not reuse a native slot", plan.reuseExistingNativeSlot);
    ok &= expectFalse("fresh plan does not replace existing state", plan.replaceExistingRuntimeStateWithoutUnsubscribe);

    const ContactSignalSubscriptionSnapshot active{
        .world = 0x1000,
        .signal = 0x2000,
        .active = true,
    };

    plan = planSubscription(active, 0x1000, 0x2000);
    ok &= expectEqual("same world and signal reuses native slot", plan.action, ContactSignalSubscriptionAction::AlreadySubscribed);
    ok &= expectTrue("same world plan marks native slot reuse", plan.reuseExistingNativeSlot);
    ok &= expectFalse("same world plan does not resubscribe", plan.subscribeRequestedSignal);
    ok &= expectFalse("same world plan does not replace bridge state", plan.replaceExistingRuntimeStateWithoutUnsubscribe);

    plan = planSubscription(active, 0x1000, 0x3000);
    ok &= expectEqual("same world changed signal is replaced without native unsubscribe", plan.action, ContactSignalSubscriptionAction::ReplaceSameWorldSignal);
    ok &= expectTrue("same world changed signal subscribes requested signal", plan.subscribeRequestedSignal);
    ok &= expectTrue("same world changed signal replaces bridge state only", plan.replaceExistingRuntimeStateWithoutUnsubscribe);
    ok &= expectFalse("same world changed signal does not reuse old slot", plan.reuseExistingNativeSlot);

    plan = planSubscription(active, 0x4000, 0x5000);
    ok &= expectEqual("different world is replaced without native unsubscribe", plan.action, ContactSignalSubscriptionAction::ReplaceDifferentWorldSignal);
    ok &= expectTrue("different world subscribes requested signal", plan.subscribeRequestedSignal);
    ok &= expectTrue("different world replaces bridge state only", plan.replaceExistingRuntimeStateWithoutUnsubscribe);
    ok &= expectFalse("different world does not reuse old slot", plan.reuseExistingNativeSlot);

    auto acceptance = evaluateCallbackAcceptance(active, 0x1000);
    ok &= expectTrue("matching callback world is accepted", acceptance.accept);
    ok &= expectEqual("matching callback world is effective world", acceptance.effectiveWorld, static_cast<std::uintptr_t>(0x1000));
    ok &= expectTrue("matching callback marks world matched", acceptance.callbackWorldMatched);
    ok &= expectFalse("matching callback does not use fallback", acceptance.usedSubscribedWorldFallback);

    acceptance = evaluateCallbackAcceptance(active, 0);
    ok &= expectFalse("missing callback world is rejected", acceptance.accept);
    ok &= expectTrue("missing callback world is reported", acceptance.callbackWorldMissing);
    ok &= expectFalse("missing callback world does not use fallback", acceptance.usedSubscribedWorldFallback);

    acceptance = evaluateCallbackAcceptance(active, 0x4000);
    ok &= expectFalse("mismatched callback world is rejected", acceptance.accept);
    ok &= expectTrue("mismatched callback world is reported", acceptance.callbackWorldMismatched);

    acceptance = evaluateCallbackAcceptance(inactive, 0x1000);
    ok &= expectFalse("inactive subscription rejects callbacks", acceptance.accept);

    ok &= expectTrue(
        "live matching world retains native slot on shutdown",
        shouldRetainNativeSlotAfterDeactivation(active, 0x1000));
    ok &= expectFalse(
        "missing live world clears bridge state on shutdown",
        shouldRetainNativeSlotAfterDeactivation(active, 0));
    ok &= expectFalse(
        "different live world clears bridge state on shutdown",
        shouldRetainNativeSlotAfterDeactivation(active, 0x4000));

    return ok ? 0 : 1;
}
