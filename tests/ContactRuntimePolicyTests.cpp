#include <cstdio>

#include "physics-interaction/collision/ContactActivityTracker.h"
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
    bool expectEq(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s mismatch\n", label);
        return false;
    }
}

int main()
{
    using namespace frik::rock::contact_activity_tracker;
    using namespace frik::rock::contact_signal_subscription_policy;

    bool ok = true;

    ContactActivityTracker tracker;

    auto first = tracker.registerHandContact(false, 100, 200);
    ok &= expectTrue("first body-pair registration is tracked", first.tracked);
    ok &= expectTrue("first body-pair registration is newly active", first.newlyActive);
    ok &= expectTrue("first body-pair registration inserts a slot", first.inserted);
    ok &= expectTrue("registered pair is active immediately", tracker.isHandContactActive(false, 100, 200));

    auto repeat = tracker.registerHandContact(false, 100, 200);
    ok &= expectTrue("repeat body-pair registration is tracked", repeat.tracked);
    ok &= expectFalse("repeat body-pair registration inside active window is not newly active", repeat.newlyActive);
    ok &= expectFalse("repeat body-pair registration reuses existing slot", repeat.inserted);

    for (std::uint32_t i = 0; i < kActiveContactFrames; ++i) {
        tracker.advanceFrame();
    }
    ok &= expectTrue("contact remains active through active frame window", tracker.isHandContactActive(false, 100, 200));

    tracker.advanceFrame();
    ok &= expectFalse("contact becomes inactive after active frame window", tracker.isHandContactActive(false, 100, 200));

    auto revived = tracker.registerHandContact(false, 100, 200);
    ok &= expectTrue("stale tracked pair is newly active when contact returns", revived.newlyActive);
    ok &= expectFalse("stale tracked pair reuses its existing slot", revived.inserted);

    for (std::uint32_t i = 0; i <= kCleanupContactFrames; ++i) {
        tracker.advanceFrame();
    }
    ok &= expectFalse("cleanup window evicts stale contact pair", tracker.isHandContactTracked(false, 100, 200));

    auto invalid = tracker.registerHandContact(true, kInvalidBodyId, 300);
    ok &= expectFalse("invalid source body is rejected", invalid.tracked);

    const auto emptySubscription = ContactSignalSubscriptionSnapshot{};
    const auto activeSubscription = ContactSignalSubscriptionSnapshot{
        .world = 0x1000,
        .signal = 0x2000,
        .active = true,
    };

    ok &= expectEq("null signal subscription is ignored",
        planSubscription(emptySubscription, 0x1000, 0).action,
        ContactSignalSubscriptionAction::IgnoreNullSignal);
    ok &= expectEq("fresh signal subscription is planned",
        planSubscription(emptySubscription, 0x1000, 0x2000).action,
        ContactSignalSubscriptionAction::SubscribeFresh);
    ok &= expectEq("same signal subscription is stable",
        planSubscription(activeSubscription, 0x1000, 0x2000).action,
        ContactSignalSubscriptionAction::AlreadySubscribed);
    ok &= expectEq("same-world signal replacement requests unsubscribe",
        planSubscription(activeSubscription, 0x1000, 0x3000).action,
        ContactSignalSubscriptionAction::ReplaceSameWorldSignal);
    ok &= expectEq("different-world signal replacement avoids stale dereference",
        planSubscription(activeSubscription, 0x4000, 0x5000).action,
        ContactSignalSubscriptionAction::ReplaceDifferentWorldSignal);
    const auto matchingCallback = evaluateCallbackAcceptance(activeSubscription, 0x1000);
    ok &= expectTrue("matching world callback is accepted", matchingCallback.accept);
    ok &= expectTrue("matching world callback keeps callback world", matchingCallback.callbackWorldMatched);
    ok &= expectEq("matching world callback effective world", matchingCallback.effectiveWorld, std::uintptr_t{ 0x1000 });

    const auto missingWorldCallback = evaluateCallbackAcceptance(activeSubscription, 0);
    ok &= expectTrue("missing callback world uses subscribed world fallback", missingWorldCallback.accept);
    ok &= expectTrue("missing callback world records fallback", missingWorldCallback.usedSubscribedWorldFallback);
    ok &= expectTrue("missing callback world records missing slot", missingWorldCallback.callbackWorldMissing);
    ok &= expectEq("missing callback effective world", missingWorldCallback.effectiveWorld, std::uintptr_t{ 0x1000 });

    const auto mismatchedWorldCallback = evaluateCallbackAcceptance(activeSubscription, 0x4000);
    ok &= expectFalse("mismatched callback world is rejected", mismatchedWorldCallback.accept);
    ok &= expectFalse("mismatched callback world does not use subscribed world fallback", mismatchedWorldCallback.usedSubscribedWorldFallback);
    ok &= expectTrue("mismatched callback world records mismatch", mismatchedWorldCallback.callbackWorldMismatched);
    ok &= expectEq("mismatched callback effective world is empty", mismatchedWorldCallback.effectiveWorld, std::uintptr_t{ 0 });

    ok &= expectFalse("inactive subscription callback is rejected", shouldAcceptCallback(emptySubscription, 0x1000));
    ok &= expectTrue("matching world unsubscribe is safe", canUnsubscribeFromWorld(activeSubscription, 0x1000));
    ok &= expectFalse("different world unsubscribe is not safe", canUnsubscribeFromWorld(activeSubscription, 0x4000));

    return ok ? 0 : 1;
}
