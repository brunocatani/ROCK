// Native player melee gate: the VRInput velocity settings ROCK owns and the
// player-only cases where native melee callbacks are skipped. Vanilla melee is
// always on; SCISSORS builds its physical melee on the native hit path.
#pragma once

#include <bit>
#include <cstdint>

namespace rock::native_melee_suppression
{
    inline constexpr char kVelocityCheckSetting[] = "bMeleeVelocityCheck:VRInput";
    inline constexpr char kLinearVelocityThresholdSetting[] = "fMeleeLinearVelocityThreshold:VRInput";
    inline constexpr char kAngularVelocityThresholdSetting[] = "fMeleeAngularVelocityThreshold:VRInput";
    inline constexpr char kGunBashLinearVelocityThresholdSetting[] = "fGunBashLinearVelocityThreshold:VRInput";
    inline constexpr char kGunBashAngularVelocityThresholdSetting[] = "fGunBashAngularVelocityThreshold:VRInput";

    /*
     * FO4VR's velocity gate (140EFE880) starts a melee swing (melee weapon) or
     * gun bash (other weapons) when the wand's averaged linear speed OR its
     * angular speed exceeds the threshold. The wand update (140C62D10) clamps
     * each angular axis to 0..1 rad/s before taking the magnitude, so the
     * angular speed saturates at sqrt(3) from a 57 deg/s wrist turn and the
     * vanilla threshold of 1.0 fires on ordinary hand movement. ROCK owns the
     * gate while its hooks are installed: linear speed decides, angular never
     * does (2.0 is above sqrt(3)). Linear speeds are in game units per second
     * (70 = 1 m/s).
     */
    inline constexpr float kAngularGateDisabled = 2.0f;

    struct NativeMeleeVelocityGate
    {
        float meleeLinear = 0.0f;
        float meleeAngular = 0.0f;
        float gunBashLinear = 0.0f;
        float gunBashAngular = 0.0f;

        friend constexpr bool operator==(const NativeMeleeVelocityGate&, const NativeMeleeVelocityGate&) = default;
    };

    [[nodiscard]] constexpr NativeMeleeVelocityGate velocityGate(float meleeSwingSpeed, float gunBashSpeed) noexcept
    {
        return { meleeSwingSpeed, kAngularGateDisabled, gunBashSpeed, kAngularGateDisabled };
    }

    [[nodiscard]] constexpr bool sameRuntimeFloatBits(float left, float right) noexcept
    {
        return std::bit_cast<std::uint32_t>(left) == std::bit_cast<std::uint32_t>(right);
    }

    /*
     * ROCK skips player native melee at the verified FO4VR swing and impact
     * boundaries (the animation-event handlers, the PlayerCharacter weapon
     * swing callback and the VRMeleeImpact collision callback) only while the
     * player holds no real weapon: SCISSORS owns free-hand strikes.
     * Returning "handled" matters for animation events because the dispatcher
     * should not keep treating a skipped event as unresolved.
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

    enum class NativeMeleeImpactAction
    {
        CallNative,
        Suppress
    };

    /*
     * What the player's native melee contact touched. VRMeleeImpact
     * (140EFF000) sends every partner whose reference is not an Actor (ACHR)
     * to the object hit (140E56FA0), which pushes the body with a point
     * impulse of up to 5000 per axis. Movable objects take their push from the
     * weapon's real collision body instead, so only actor and static-world
     * contacts reach native. Unknown partners (unreadable body) stay native.
     */
    enum class NativeMeleePartner
    {
        Unknown,
        Actor,
        StaticWorld,
        MovableObject
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
        NativeMeleePartner partner = NativeMeleePartner::Unknown;
    };

    struct NativeMeleeImpactPolicyDecision
    {
        NativeMeleeImpactAction action = NativeMeleeImpactAction::CallNative;
        const char* reason = "native";
    };

    inline NativeMeleePolicyDecision evaluateNativeMeleeSuppression(NativeMeleeEvent event,
        const NativeMeleePolicyInput& input)
    {
        if (!input.suppressionActive) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "suppression-disabled" };
        }

        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "non-player" };
        }

        switch (event) {
        case NativeMeleeEvent::WeaponSwing:
            return { .action = NativeMeleeSuppressionAction::ReturnHandled,
                .reason = "player-weapon-swing-suppressed" };

        case NativeMeleeEvent::HitFrame:
            return { .action = NativeMeleeSuppressionAction::ReturnHandled, .reason = "player-hitframe-suppressed" };
        }

        return { .action = NativeMeleeSuppressionAction::CallNative, .reason = "unknown-event" };
    }

    inline NativeMeleeImpactPolicyDecision evaluateNativeMeleeImpactSuppression(
        const NativeMeleeImpactPolicyInput& input)
    {
        if (!input.actorIsPlayer) {
            return { .action = NativeMeleeImpactAction::CallNative, .reason = "non-player" };
        }

        if (input.suppressionActive) {
            return { .action = NativeMeleeImpactAction::Suppress, .reason = "player-vr-melee-impact-suppressed" };
        }

        if (input.partner == NativeMeleePartner::MovableObject) {
            return { .action = NativeMeleeImpactAction::Suppress, .reason = "player-movable-object-contact" };
        }

        return { .action = NativeMeleeImpactAction::CallNative, .reason = "native" };
    }
}
