#pragma once

#include "api/ROCKProviderApi.h"

namespace rock
{
    struct EquippedWeaponHandlingSettings
    {
        bool externalAuthorityActive{ false };
        bool firingGripOwnershipEnabled{ false };
        bool primaryDetachEnabled{ false };
        bool ambidextrousHandoffEnabled{ false };
        bool gripZoneEquipEnabled{ false };
        bool gripZoneHoverHapticsEnabled{ false };
        bool equippedWeaponShoulderStashEnabled{ false };
        bool pipboyTriggerHandEquipEnabled{ false };

        float gripZoneEquipRadiusGameUnits{ 3.0f };
        float gripZoneEquipSettleSeconds{ 0.15f };
        float firingGripReattachRadiusGameUnits{ 3.0f };
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
        float equipVisualBridgeTimeoutSeconds{ 2.0f };
        float equipVisualBridgeBlendSeconds{ 0.15f };
    };

    [[nodiscard]] inline EquippedWeaponHandlingSettings
    makeEquippedWeaponHandlingSettings(
        const float rockFiringGripProximitySupportRadiusGameUnits,
        const provider::RockProviderEquippedWeaponHandlingRequestV1* request)
    {
        EquippedWeaponHandlingSettings settings{};
        // Near-firing-grip VisualOnlySupport is a ROCK weapon-support safety
        // contract, not ambidextrous ownership. ROCK supplies the baseline
        // radius; an active handling owner may replace only that tuning value.
        settings.firingGripProximitySupportRadiusGameUnits =
            rockFiringGripProximitySupportRadiusGameUnits;
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
        settings.firingGripOwnershipEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
        settings.primaryDetachEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach);
        settings.ambidextrousHandoffEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff);
        settings.gripZoneEquipEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip);
        settings.gripZoneHoverHapticsEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::GripZoneHoverHaptics);
        settings.equippedWeaponShoulderStashEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
        settings.pipboyTriggerHandEquipEnabled = enabled(
            provider::RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip);
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
}
