#pragma once

#include <bit>
#include <cstdint>

namespace rock::native_melee_suppression
{
    /*
     * This master switch is a permanent recovery contract, not a temporary
     * compatibility path. Every current and future ROCK melee-hit suppression
     * boundary must call native behavior when it is disabled, and every owned
     * runtime override must be restored or safely relinquished.
     */
    inline constexpr char kVelocityCheckSetting[] = "bMeleeVelocityCheck:VRInput";
    inline constexpr char kLinearVelocityThresholdSetting[] = "fMeleeLinearVelocityThreshold:VRInput";
    inline constexpr char kAngularVelocityThresholdSetting[] = "fMeleeAngularVelocityThreshold:VRInput";
    inline constexpr float kSuppressedVelocityThreshold = 1.0e9f;

    [[nodiscard]] constexpr bool sameRuntimeFloatBits(float left, float right) noexcept
    {
        return std::bit_cast<std::uint32_t>(left) == std::bit_cast<std::uint32_t>(right);
    }

    /*
     * ROCK suppresses player native melee at the verified FO4VR swing and
     * impact boundaries: the animation-event handlers, the PlayerCharacter
     * weapon swing callback, and the VRMeleeImpact collision callback. Returning
     * "handled" matters for animation events because the dispatcher should not
     * keep treating a suppressed event as unresolved, while native hit/damage
     * side effects remain skipped. Void callbacks use the same policy only to
     * decide whether native side effects should run.
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

    enum class NativeMeleeInputEvent
    {
        Unknown,
        RightStick,
        PrimaryAttack,
        SecondaryAttack
    };

    enum class NativeMeleeInputGateAction
    {
        CallNative,
        ReturnFalse
    };

    enum class NativeMeleeImpactAction
    {
        CallNative,
        Suppress
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

    struct NativeMeleeImpactPolicyInput
    {
        bool rockEnabled = false;
        bool suppressionEnabled = false;
        bool fullSuppression = true;
        bool actorIsPlayer = false;
    };

    struct NativeMeleeImpactPolicyDecision
    {
        NativeMeleeImpactAction action = NativeMeleeImpactAction::CallNative;
        const char* reason = "native";
    };

    struct NativeMeleeInputGatePolicyInput
    {
        bool rockEnabled = false;
        bool suppressionEnabled = false;
        bool fullSuppression = true;
        NativeMeleeInputEvent inputEvent = NativeMeleeInputEvent::Unknown;
    };

    struct NativeMeleeInputGatePolicyDecision
    {
        NativeMeleeInputGateAction action = NativeMeleeInputGateAction::CallNative;
        const char* reason = "native";
    };

    struct NativeMeleeRuntimeSettingPolicyInput
    {
        bool hooksInstalled = false;
        bool rockEnabled = false;
        bool suppressionEnabled = false;
        bool fullSuppression = true;
    };

    [[nodiscard]] constexpr bool shouldSuppressNativeMeleeRuntimeSettings(const NativeMeleeRuntimeSettingPolicyInput& input) noexcept
    {
        return input.hooksInstalled && input.rockEnabled && input.suppressionEnabled && input.fullSuppression;
    }

    [[nodiscard]] constexpr bool shouldRestoreNativeMeleeRuntimeSettings(
        bool previouslyApplied, const NativeMeleeRuntimeSettingPolicyInput& input) noexcept
    {
        return previouslyApplied && !shouldSuppressNativeMeleeRuntimeSettings(input);
    }

    inline bool isPhysicalSwingLeaseActive(std::uint64_t currentFrame, std::uint64_t expiresAtFrame)
    {
        return expiresAtFrame != 0 && currentFrame <= expiresAtFrame;
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
            return { .action = NativeMeleeSuppressionAction::ReturnHandled,
                .reason = input.fullSuppression ? "player-weapon-swing-full-suppressed" : "player-weapon-swing-suppressed" };

        case NativeMeleeEvent::HitFrame:
            if (!input.suppressHitFrame) {
                return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "hitframe-pass-through" };
            }
            if (input.physicalSwingActive && !input.fullSuppression) {
                return { .action = NativeMeleeSuppressionAction::ReturnHandled, .reason = "physical-swing-handled-hitframe" };
            }
            return { .action = NativeMeleeSuppressionAction::ReturnHandled,
                .reason = input.fullSuppression ? "player-hitframe-full-suppressed" : "player-hitframe-suppressed" };
        }

        return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "unknown-event" };
    }

    inline NativeMeleeInputGatePolicyDecision evaluateNativeMeleeInputGate(const NativeMeleeInputGatePolicyInput& input)
    {
        if (!input.rockEnabled) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "rock-disabled" };
        }

        if (!input.suppressionEnabled) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.fullSuppression) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "partial-suppression" };
        }

        if (input.inputEvent != NativeMeleeInputEvent::RightStick) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "non-right-stick" };
        }

        return { .action = NativeMeleeInputGateAction::ReturnFalse, .reason = "right-stick-native-melee-gate-suppressed" };
    }

    inline NativeMeleeImpactPolicyDecision evaluateNativeMeleeImpactSuppression(const NativeMeleeImpactPolicyInput& input)
    {
        if (!input.rockEnabled) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "rock-disabled" };
        }

        if (!input.suppressionEnabled) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "non-player" };
        }

        if (!input.fullSuppression) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "partial-suppression" };
        }

        return { .action = NativeMeleeImpactAction::Suppress, .reason = "player-vr-melee-impact-full-suppressed" };
    }
}
