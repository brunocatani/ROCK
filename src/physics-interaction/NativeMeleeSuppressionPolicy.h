#pragma once

#include <cstdint>

namespace frik::rock::native_melee_suppression
{
    /*
     * ROCK suppresses player native melee at the animation-event boundary because
     * PLANCK's durable pattern is not "turn combat off"; it is "skip the
     * animation-driven player melee timing, then let a physical swing system
     * invoke native hit/damage side effects later." HIGGS does not broadly do
     * this because HIGGS keeps Skyrim VR melee data as a collision source, while
     * ROCK is preparing for a PLANCK-style physical-hit pipeline. This policy is
     * pure and tested so the hook layer remains a narrow FO4VR adapter around
     * Ghidra-verified handler methods.
     */
    enum class NativeMeleeEvent
    {
        WeaponSwing,
        HitFrame
    };

    enum class NativeMeleeSuppressionAction
    {
        CallNative,
        ReturnUnhandled,
        ReturnHandled
    };

    struct NativeMeleePolicyInput
    {
        bool rockEnabled = false;
        bool suppressionEnabled = false;
        bool fullSuppression = true;
        bool suppressWeaponSwing = true;
        bool suppressHitFrame = true;
        bool actorIsPlayer = false;
        bool physicalSwingActive = false;
    };

    struct NativeMeleePolicyDecision
    {
        NativeMeleeSuppressionAction action = NativeMeleeSuppressionAction::CallNative;
        const char* reason = "native";
    };

    inline bool isPhysicalSwingLeaseActive(std::uint64_t currentTick, std::uint64_t expiresAtTick)
    {
        return expiresAtTick != 0 && currentTick <= expiresAtTick;
    }

    inline NativeMeleePolicyDecision evaluateNativeMeleeSuppression(NativeMeleeEvent event, const NativeMeleePolicyInput& input)
    {
        if (!input.rockEnabled) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "rock-disabled" };
        }

        if (!input.suppressionEnabled) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "non-player" };
        }

        switch (event) {
        case NativeMeleeEvent::WeaponSwing:
            if (!input.suppressWeaponSwing) {
                return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "weapon-swing-pass-through" };
            }
            return { .action = NativeMeleeSuppressionAction::ReturnUnhandled,
                .reason = input.fullSuppression ? "player-weapon-swing-full-suppressed" : "player-weapon-swing-suppressed" };

        case NativeMeleeEvent::HitFrame:
            if (!input.suppressHitFrame) {
                return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "hitframe-pass-through" };
            }
            if (input.physicalSwingActive && !input.fullSuppression) {
                return { .action = NativeMeleeSuppressionAction::ReturnHandled, .reason = "physical-swing-handled-hitframe" };
            }
            return { .action = NativeMeleeSuppressionAction::ReturnUnhandled,
                .reason = input.fullSuppression ? "player-hitframe-full-suppressed" : "player-hitframe-suppressed" };
        }

        return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "unknown-event" };
    }
}
