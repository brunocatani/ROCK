#pragma once

#include <cstdint>

namespace rock::contact_signal_subscription_policy
{
    /*
     * FO4VR exposes contact notifications through an hkSignal slot rather than
     * a Skyrim-style listener object. The subscription decision is kept as a
     * pure policy so runtime code can avoid duplicate slots, reject callbacks
     * from stale worlds, and only call the Ghidra-verified removal function
     * when the stored signal belongs to the same live world.
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
        bool unsubscribeExistingSignal = false;
        bool clearExistingWithoutUnsubscribe = false;
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
        std::uintptr_t requestedSignal)
    {
        if (requestedWorld == 0 || requestedSignal == 0) {
            return {};
        }

        if (!isActiveSubscription(current)) {
            return {
                .action = ContactSignalSubscriptionAction::SubscribeFresh,
                .subscribeRequestedSignal = true,
            };
        }

        if (current.world == requestedWorld && current.signal == requestedSignal) {
            return { .action = ContactSignalSubscriptionAction::AlreadySubscribed };
        }

        if (current.world == requestedWorld) {
            return {
                .action = ContactSignalSubscriptionAction::ReplaceSameWorldSignal,
                .subscribeRequestedSignal = true,
                .unsubscribeExistingSignal = true,
            };
        }

        return {
            .action = ContactSignalSubscriptionAction::ReplaceDifferentWorldSignal,
            .subscribeRequestedSignal = true,
            .unsubscribeExistingSignal = false,
            .clearExistingWithoutUnsubscribe = true,
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
                .accept = true,
                .effectiveWorld = current.world,
                .usedSubscribedWorldFallback = true,
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

    inline constexpr bool canUnsubscribeFromWorld(const ContactSignalSubscriptionSnapshot& current, std::uintptr_t currentWorld)
    {
        return isActiveSubscription(current) && currentWorld != 0 && current.world == currentWorld;
    }
}
