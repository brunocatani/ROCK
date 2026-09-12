#pragma once

#include "api/ROCKProviderApi.h"
#include "physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h"

namespace rock
{
    struct RockEquippedWeaponHandlingBaseline
    {
        bool ambidextrousHandoffEnabled{ false };
        bool authoredOnlySupportGrabsEnabled{ true };
        bool toggleGrabEnabled{ false };
        bool equippedWeaponShoulderStashEnabled{ false };
        // Whether the last hand carrying the weapon may drop it by letting go.
        // With false that grip is retained; releases are honored only while
        // the other hand still carries the weapon.
        bool lastGripReleaseDropEnabled{ true };
        immersive_weapon_policy::Config immersiveWeapon{};
        float firingGripReattachCylinderRadiusGameUnits{ 2.0f };
        float firingGripProximitySupportRadiusGameUnits{ 6.0f };
        float firingGripPromotionRadiusGameUnits{ 5.0f };
        float leftFiringAimYawDegrees{ 0.0f };
        float leftFiringAimPitchDegrees{ 0.0f };
        float leftFiringAimOffsetXGameUnits{ 0.0f };
        float leftFiringAimOffsetYGameUnits{ 0.0f };
        float leftFiringAimOffsetZGameUnits{ 0.0f };
    };

    struct EquippedWeaponHandlingSettings
    {
        bool externalAuthorityActive{ false };
        bool firingGripOwnershipEnabled{ false };
        bool primaryDetachEnabled{ false };
        immersive_weapon_policy::DetachAuthority detachAuthority{
            immersive_weapon_policy::DetachAuthority::None
        };
        bool preserveWeaponPoseOnDetach{ false };
        bool ambidextrousHandoffEnabled{ false };
        bool authoredOnlySupportGrabsEnabled{ true };
        bool toggleGrabEnabled{ false };
        bool lastGripReleaseDropEnabled{ true };
        bool gripZoneEquipEnabled{ false };
        bool gripZoneHoverHapticsEnabled{ false };
        bool equippedWeaponShoulderStashEnabled{ false };
        immersive_weapon_policy::Config immersiveWeapon{};

        float gripZoneEquipRadiusGameUnits{ 3.0f };
        float gripZoneEquipSettleSeconds{ 0.15f };
        float firingGripReattachRadiusGameUnits{ 12.0f };
        float firingGripReattachCylinderRadiusGameUnits{ 2.0f };
        float gripZoneHoverHapticIntensity{ 0.75f };
        float firingGripProximitySupportRadiusGameUnits{ 6.0f };
        float weaponGripHapticDurationSeconds{ 0.10f };
        float firingGripAttachHapticIntensity{ 0.85f };
        float firingGripDetachHapticIntensity{ 0.30f };
        float supportGripHapticIntensity{ 0.50f };
        float firingGripPromotionRadiusGameUnits{ 5.0f };
        float leftFiringAimYawDegrees{ 0.0f };
        float leftFiringAimPitchDegrees{ 0.0f };
        float leftFiringAimOffsetXGameUnits{ 0.0f };
        float leftFiringAimOffsetYGameUnits{ 0.0f };
        float leftFiringAimOffsetZGameUnits{ 0.0f };
        float equipVisualBridgeTimeoutSeconds{ 1.0f };
        float equipVisualBridgeBlendSeconds{ 0.15f };
    };

    [[nodiscard]] inline EquippedWeaponHandlingSettings
    makeEquippedWeaponHandlingSettings(
        const RockEquippedWeaponHandlingBaseline& rockBaseline,
        const provider::RockProviderEquippedWeaponHandlingRequestV1* request)
    {
        EquippedWeaponHandlingSettings settings{};
        settings.firingGripOwnershipEnabled =
            rockBaseline.ambidextrousHandoffEnabled;
        // This base snapshot retains provider detach separately. Integrated
        // immersive detach is resolved as the same current-firing-role
        // contract for either physical hand.
        settings.primaryDetachEnabled = false;
        settings.ambidextrousHandoffEnabled =
            rockBaseline.ambidextrousHandoffEnabled;
        // This is a ROCK-local acquisition preference. Provider equipped-
        // weapon handling authority cannot disable it; an exact matched
        // weapon-part target is the narrower, explicit override at capture.
        settings.authoredOnlySupportGrabsEnabled =
            rockBaseline.authoredOnlySupportGrabsEnabled;
        // Toggle grab is a ROCK input preference. A handling-provider lease
        // can add weapon capabilities, but it cannot replace this input mode.
        settings.toggleGrabEnabled = rockBaseline.toggleGrabEnabled;
        // Like toggle grab, the last-grip drop is a ROCK release preference
        // that applies under either detach authority; a handling-provider
        // lease cannot re-enable dropping for the player's last carrying hand.
        settings.lastGripReleaseDropEnabled =
            rockBaseline.lastGripReleaseDropEnabled;
        settings.equippedWeaponShoulderStashEnabled =
            rockBaseline.equippedWeaponShoulderStashEnabled;
        settings.immersiveWeapon = rockBaseline.immersiveWeapon;
        // The reattach cylinder radius is ROCK tuning under both detach
        // authorities; a handling owner may replace only the reach.
        settings.firingGripReattachCylinderRadiusGameUnits =
            rockBaseline.firingGripReattachCylinderRadiusGameUnits;
        // Near-firing-grip VisualOnlySupport is a ROCK weapon-support safety
        // contract, not ambidextrous ownership. ROCK supplies the baseline
        // radius; an active handling owner may replace only that tuning value.
        settings.firingGripProximitySupportRadiusGameUnits =
            rockBaseline.firingGripProximitySupportRadiusGameUnits;
        settings.firingGripPromotionRadiusGameUnits =
            rockBaseline.firingGripPromotionRadiusGameUnits;
        settings.leftFiringAimYawDegrees =
            rockBaseline.leftFiringAimYawDegrees;
        settings.leftFiringAimPitchDegrees =
            rockBaseline.leftFiringAimPitchDegrees;
        settings.leftFiringAimOffsetXGameUnits =
            rockBaseline.leftFiringAimOffsetXGameUnits;
        settings.leftFiringAimOffsetYGameUnits =
            rockBaseline.leftFiringAimOffsetYGameUnits;
        settings.leftFiringAimOffsetZGameUnits =
            rockBaseline.leftFiringAimOffsetZGameUnits;
        if (!request) {
            return settings;
        }

        const auto enabled = [request](
                                 const provider::RockProviderEquippedWeaponHandlingFlagV1 flag) {
            return provider::hasEquippedWeaponHandlingFlagV1(
                request->flags,
                flag);
        };
        settings.externalAuthorityActive = true;
        // Shoulder sheath/retrieval is a ROCK-owned capability. An addon may
        // add handling capabilities, but its lease cannot suppress ROCK's
        // configured native shoulder path.
        settings.firingGripOwnershipEnabled =
            settings.firingGripOwnershipEnabled || enabled(
                provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
        settings.primaryDetachEnabled =
            settings.primaryDetachEnabled || enabled(
                provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach);
        settings.ambidextrousHandoffEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff);
        settings.gripZoneEquipEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip);
        settings.gripZoneHoverHapticsEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::GripZoneHoverHaptics);
        // RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip
        // remains an accepted ABI flag but no longer maps to any behavior:
        // the Pip-Boy hand-equip mode was removed. Physical handoff,
        // trigger/grip-zone equip, and shoulder retrieval are the supported
        // left-hand entry points.
        settings.gripZoneEquipRadiusGameUnits = request->gripZoneEquipRadiusGameUnits;
        settings.gripZoneEquipSettleSeconds = request->gripZoneEquipSettleSeconds;
        settings.firingGripReattachRadiusGameUnits = request->firingGripReattachRadiusGameUnits;
        settings.gripZoneHoverHapticIntensity = request->gripZoneHoverHapticIntensity;
        if (enabled(provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport)) {
            settings.firingGripProximitySupportRadiusGameUnits =
                request->firingGripProximitySupportRadiusGameUnits;
        }
        settings.weaponGripHapticDurationSeconds = request->weaponGripHapticDurationSeconds;
        settings.firingGripAttachHapticIntensity = request->firingGripAttachHapticIntensity;
        settings.firingGripDetachHapticIntensity = request->firingGripDetachHapticIntensity;
        settings.supportGripHapticIntensity = request->supportGripHapticIntensity;
        settings.firingGripPromotionRadiusGameUnits = request->firingGripPromotionRadiusGameUnits;
        settings.leftFiringAimYawDegrees = request->leftFiringAimYawDegrees;
        settings.leftFiringAimPitchDegrees = request->leftFiringAimPitchDegrees;
        settings.leftFiringAimOffsetXGameUnits = request->leftFiringAimOffsetGameUnits[0];
        settings.leftFiringAimOffsetYGameUnits = request->leftFiringAimOffsetGameUnits[1];
        settings.leftFiringAimOffsetZGameUnits = request->leftFiringAimOffsetGameUnits[2];
        settings.equipVisualBridgeTimeoutSeconds = request->equipVisualBridgeTimeoutSeconds;
        settings.equipVisualBridgeBlendSeconds = request->equipVisualBridgeBlendSeconds;
        return settings;
    }

    [[nodiscard]] inline constexpr immersive_weapon_policy::Decision
    resolveEquippedWeaponDetachDecision(
        const EquippedWeaponHandlingSettings& settings) noexcept
    {
        return immersive_weapon_policy::resolve(
            immersive_weapon_policy::ResolveInput{
                .integrated = settings.immersiveWeapon,
                .firingGripOwnershipEnabled =
                    settings.firingGripOwnershipEnabled,
                .externalPrimaryDetachEnabled =
                    settings.externalAuthorityActive &&
                    settings.primaryDetachEnabled,
                .externalReattachRadiusGameUnits =
                    settings.firingGripReattachRadiusGameUnits,
                .externalGripHapticDurationSeconds =
                    settings.weaponGripHapticDurationSeconds,
                .externalGripAttachHapticIntensity =
                    settings.firingGripAttachHapticIntensity,
                .externalGripDetachHapticIntensity =
                    settings.firingGripDetachHapticIntensity,
            });
    }

    [[nodiscard]] inline constexpr bool
    requiresEquippedWeaponHandlingModeReconcile(
        const EquippedWeaponHandlingSettings& previous,
        const EquippedWeaponHandlingSettings& current) noexcept
    {
        if (previous.toggleGrabEnabled != current.toggleGrabEnabled) {
            return true;
        }

        // Request-source changes are intentionally ignored. A compatible
        // addon-to-ROCK fallback keeps the same ROCK executor and can preserve
        // its live handoff. Reconcile only when a state-owning capability is
        // removed and the current manual state may no longer be legal.
        const bool integratedImmersiveDetachRemoved =
            previous.immersiveWeapon.
                firingGripDetachEnabled &&
            !current.immersiveWeapon.
                firingGripDetachEnabled &&
            !current.primaryDetachEnabled;

        return (previous.firingGripOwnershipEnabled &&
                   !current.firingGripOwnershipEnabled) ||
               (previous.primaryDetachEnabled &&
                   !current.primaryDetachEnabled) ||
               integratedImmersiveDetachRemoved ||
               (previous.ambidextrousHandoffEnabled &&
                   !current.ambidextrousHandoffEnabled);
    }
}
