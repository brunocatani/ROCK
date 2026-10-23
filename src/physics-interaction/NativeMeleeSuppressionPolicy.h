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
    inline constexpr char kGunBashLinearVelocityThresholdSetting[] = "fGunBashLinearVelocityThreshold:VRInput";
    inline constexpr char kGunBashAngularVelocityThresholdSetting[] = "fGunBashAngularVelocityThreshold:VRInput";
    inline constexpr float kSuppressedVelocityThreshold = 1.0e9f;

    /*
     * FO4VR's velocity gate (140EFE880) starts a melee swing (melee weapon) or
     * gun bash (other weapons) when the wand's averaged linear speed OR its
     * angular speed exceeds the threshold. The wand update (140C62D10) clamps
     * each angular axis to 0..1 rad/s before taking the magnitude, so the
     * angular speed saturates at sqrt(3) from a 57 deg/s wrist turn and the
     * vanilla threshold of 1.0 fires on ordinary hand movement. ROCK owns the
     * gate while its hooks are installed: linear speed decides, angular never
     * does (2.0 is above sqrt(3)); suppression puts every threshold out of
     * reach. Linear speeds are in game units per second (70 = 1 m/s).
     */
    inline constexpr float kAngularGateDisabled = 2.0f;

    struct NativeMeleeVelocityGate
    {
        float meleeLinear = kSuppressedVelocityThreshold;
        float meleeAngular = kSuppressedVelocityThreshold;
        float gunBashLinear = kSuppressedVelocityThreshold;
        float gunBashAngular = kSuppressedVelocityThreshold;

        friend constexpr bool operator==(const NativeMeleeVelocityGate&, const NativeMeleeVelocityGate&) = default;
    };

    [[nodiscard]] constexpr NativeMeleeVelocityGate velocityGate(
        bool suppressed, float meleeSwingSpeed, float gunBashSpeed) noexcept
    {
        if (suppressed) return {};
        return { meleeSwingSpeed, kAngularGateDisabled, gunBashSpeed, kAngularGateDisabled };
    }

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
        bool suppressionActive = false;
        bool actorIsPlayer = false;
    };

    struct NativeMeleePolicyDecision
    {
        NativeMeleeSuppressionAction action = NativeMeleeSuppressionAction::CallNative;
        const char* reason = "native";
    };

    struct NativeMeleeImpactPolicyInput
    {
        bool suppressionActive = false;
        bool actorIsPlayer = false;
    };

    struct NativeMeleeImpactPolicyDecision
    {
        NativeMeleeImpactAction action = NativeMeleeImpactAction::CallNative;
        const char* reason = "native";
    };

    struct NativeMeleeInputGatePolicyInput
    {
        bool suppressionActive = false;
        NativeMeleeInputEvent inputEvent = NativeMeleeInputEvent::Unknown;
    };

    struct NativeMeleeInputGatePolicyDecision
    {
        NativeMeleeInputGateAction action = NativeMeleeInputGateAction::CallNative;
        const char* reason = "native";
    };


    inline NativeMeleePolicyDecision evaluateNativeMeleeSuppression(NativeMeleeEvent event, const NativeMeleePolicyInput& input)
    {
        if (!input.suppressionActive) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "non-player" };
        }

        switch (event) {
        case NativeMeleeEvent::WeaponSwing:
            return { .action = NativeMeleeSuppressionAction::ReturnHandled, .reason = "player-weapon-swing-suppressed" };

        case NativeMeleeEvent::HitFrame:
            return { .action = NativeMeleeSuppressionAction::ReturnHandled, .reason = "player-hitframe-suppressed" };
        }

        return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "unknown-event" };
    }

    inline NativeMeleeInputGatePolicyDecision evaluateNativeMeleeInputGate(const NativeMeleeInputGatePolicyInput& input)
    {
        if (!input.suppressionActive) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "suppression-disabled" };
        }

        if (input.inputEvent != NativeMeleeInputEvent::RightStick) {
            return { .action = NativeMeleeInputGateAction::CallNative, .reason = "non-right-stick" };
        }

        return { .action = NativeMeleeInputGateAction::ReturnFalse, .reason = "right-stick-native-melee-gate-suppressed" };
    }

    inline NativeMeleeImpactPolicyDecision evaluateNativeMeleeImpactSuppression(const NativeMeleeImpactPolicyInput& input)
    {
        if (!input.suppressionActive) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "non-player" };
        }

        return { .action = NativeMeleeImpactAction::Suppress, .reason = "player-vr-melee-impact-suppressed" };
    }
}
