#pragma once

#include <cstdint>

namespace rock::contact_signal_subscription_policy
{
    /*
     * FO4VR exposes contact notifications through an hkSignal slot on the hknp
     * world. The subscription decision is kept as a pure policy so runtime code
     * can avoid duplicate slots, reject callbacks from stale worlds, and avoid
     * mutating the native hkSignal slot list while Havok may be dispatching
     * contact events on worker threads.
     */

    enum class ContactSignalSubscriptionAction : std::uint32_t
    {
        IgnoreNullSignal = 0,
        AlreadySubscribed,
        SubscribeFresh,
        ReplaceSameWorldSignal,
        ReplaceDifferentWorldSignal,
    };

    struct ContactSignalSubscriptionSnapshot
    {
        std::uintptr_t world = 0;
        std::uintptr_t signal = 0;
        bool active = false;
    };

    struct ContactSignalSubscriptionPlan
    {
        ContactSignalSubscriptionAction action = ContactSignalSubscriptionAction::IgnoreNullSignal;
        bool subscribeRequestedSignal = false;
        bool reuseExistingNativeSlot = false;
        bool replaceExistingRuntimeStateWithoutUnsubscribe = false;
    };

    struct ContactCallbackAcceptance
    {
        bool accept = false;
        std::uintptr_t effectiveWorld = 0;
        bool usedSubscribedWorldFallback = false;
        bool callbackWorldMatched = false;
        bool callbackWorldMissing = false;
        bool callbackWorldMismatched = false;
    };

    inline constexpr bool isActiveSubscription(const ContactSignalSubscriptionSnapshot& snapshot)
    {
        return snapshot.active && snapshot.world != 0 && snapshot.signal != 0;
    }

    inline constexpr ContactSignalSubscriptionPlan planSubscription(
        const ContactSignalSubscriptionSnapshot& current,
        std::uintptr_t requestedWorld,
        std::uintptr_t requestedSignal,
        bool requestedNativeSlotRetained = false)
    {
        if (requestedWorld == 0 || requestedSignal == 0) {
            return {};
        }

        if (requestedNativeSlotRetained) {
            return {
                .action = ContactSignalSubscriptionAction::AlreadySubscribed,
                .reuseExistingNativeSlot = true,
            };
        }

        if (!isActiveSubscription(current)) {
            return {
                .action = ContactSignalSubscriptionAction::SubscribeFresh,
                .subscribeRequestedSignal = true,
            };
        }

        if (current.world == requestedWorld && current.signal == requestedSignal) {
            return {
                .action = ContactSignalSubscriptionAction::AlreadySubscribed,
                .reuseExistingNativeSlot = true,
            };
        }

        if (current.world == requestedWorld) {
            return {
                .action = ContactSignalSubscriptionAction::ReplaceSameWorldSignal,
                .subscribeRequestedSignal = true,
                .replaceExistingRuntimeStateWithoutUnsubscribe = true,
            };
        }

        return {
            .action = ContactSignalSubscriptionAction::ReplaceDifferentWorldSignal,
            .subscribeRequestedSignal = true,
            .replaceExistingRuntimeStateWithoutUnsubscribe = true,
        };
    }

    inline constexpr ContactCallbackAcceptance evaluateCallbackAcceptance(const ContactSignalSubscriptionSnapshot& current, std::uintptr_t callbackWorld)
    {
        if (!isActiveSubscription(current)) {
            return {};
        }

        if (callbackWorld == current.world) {
            return {
                .accept = true,
                .effectiveWorld = current.world,
                .callbackWorldMatched = true,
            };
        }

        if (callbackWorld == 0) {
            return {
                .accept = false,
                .callbackWorldMissing = true,
            };
        }

        return {
            .accept = false,
            .callbackWorldMismatched = true,
        };
    }

    inline constexpr bool shouldAcceptCallback(const ContactSignalSubscriptionSnapshot& current, std::uintptr_t callbackWorld)
    {
        return evaluateCallbackAcceptance(current, callbackWorld).accept;
    }

    inline constexpr bool shouldRetainNativeSlotAfterDeactivation(const ContactSignalSubscriptionSnapshot& current)
    {
        return isActiveSubscription(current);
    }
}
