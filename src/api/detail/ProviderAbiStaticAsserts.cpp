#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/ROCKProviderApi.h"
#include "physics-interaction/body/BodyZone.h"
#include "physics-interaction/weapon/WeaponTypes.h"
#include "physics-interaction/weapon/parts/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/parts/WeaponPartRuntime.h"

namespace rock::provider::detail
{
    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneKind::LeftShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::LeftShoulder));
    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneKind::RightShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::RightShoulder));
    static_assert(static_cast<std::uint32_t>(RockProviderBodyZoneSide::Left) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneSide::Left));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::FiringGrip) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::FiringGrip));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::SupportFullAuthority) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportFullAuthority));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::SupportVisualOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportVisualOnly));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::PartCarry) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::PartCarry));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartGripKindV1::AttachOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::AttachOnly));
    static_assert(static_cast<std::uint32_t>(
                      RockProviderNativeAnimationAuthorityFlagV1::Arms) ==
                  (1u << 0));
    static_assert(static_cast<std::uint32_t>(
                      RockProviderNativeAnimationAuthorityFlagV1::Hands) ==
                  (1u << 1));
    static_assert(static_cast<std::uint32_t>(
                      RockProviderNativeAnimationAuthorityFlagV1::Weapon) ==
                  (1u << 2));
    static_assert(static_cast<std::uint32_t>(
                      RockProviderNativeAnimationAuthorityFlagV1::ReloadPose) ==
                  ((1u << 0) | (1u << 1) | (1u << 2)));
    static_assert(static_cast<std::uint32_t>(
                      RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureFault) ==
                  (1u << 6));

    // A public enum reorder must fail this build before it breaks a consumer.
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Receiver) == static_cast<std::uint32_t>(WeaponPartKind::Receiver));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Barrel) == static_cast<std::uint32_t>(WeaponPartKind::Barrel));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Handguard) == static_cast<std::uint32_t>(WeaponPartKind::Handguard));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Foregrip) == static_cast<std::uint32_t>(WeaponPartKind::Foregrip));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Pump) == static_cast<std::uint32_t>(WeaponPartKind::Pump));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Stock) == static_cast<std::uint32_t>(WeaponPartKind::Stock));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Grip) == static_cast<std::uint32_t>(WeaponPartKind::Grip));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Magazine) == static_cast<std::uint32_t>(WeaponPartKind::Magazine));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Magwell) == static_cast<std::uint32_t>(WeaponPartKind::Magwell));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Bolt) == static_cast<std::uint32_t>(WeaponPartKind::Bolt));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Slide) == static_cast<std::uint32_t>(WeaponPartKind::Slide));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::ChargingHandle) == static_cast<std::uint32_t>(WeaponPartKind::ChargingHandle));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::BreakAction) == static_cast<std::uint32_t>(WeaponPartKind::BreakAction));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Cylinder) == static_cast<std::uint32_t>(WeaponPartKind::Cylinder));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Chamber) == static_cast<std::uint32_t>(WeaponPartKind::Chamber));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Shell) == static_cast<std::uint32_t>(WeaponPartKind::Shell));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Round) == static_cast<std::uint32_t>(WeaponPartKind::Round));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::LaserCell) == static_cast<std::uint32_t>(WeaponPartKind::LaserCell));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Lever) == static_cast<std::uint32_t>(WeaponPartKind::Lever));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Sight) == static_cast<std::uint32_t>(WeaponPartKind::Sight));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Accessory) == static_cast<std::uint32_t>(WeaponPartKind::Accessory));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::CosmeticAmmo) == static_cast<std::uint32_t>(WeaponPartKind::CosmeticAmmo));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Other) == static_cast<std::uint32_t>(WeaponPartKind::Other));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::LaserSight) == static_cast<std::uint32_t>(WeaponPartKind::LaserSight));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Flashlight) == static_cast<std::uint32_t>(WeaponPartKind::Flashlight));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::LaserFlashlightCombo) == static_cast<std::uint32_t>(WeaponPartKind::LaserFlashlightCombo));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Scope) == static_cast<std::uint32_t>(WeaponPartKind::Scope));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::MuzzleDevice) == static_cast<std::uint32_t>(WeaponPartKind::MuzzleDevice));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartKindV1::Bipod) == static_cast<std::uint32_t>(WeaponPartKind::Bipod));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartClassificationSourceV1::NameToken) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::NameToken));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartClassificationSourceV1::SlotAnchor) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::SlotAnchor));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartClassificationSourceV1::RigAnchor) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::RigAnchor));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponPartClassificationSourceV1::AttachmentEvidence) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::AttachmentEvidence));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::None) == static_cast<std::uint32_t>(WeaponActionRole::None));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Bolt) == static_cast<std::uint32_t>(WeaponActionRole::Bolt));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Slide) == static_cast<std::uint32_t>(WeaponActionRole::Slide));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::ChargingHandle) == static_cast<std::uint32_t>(WeaponActionRole::ChargingHandle));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Pump) == static_cast<std::uint32_t>(WeaponActionRole::Pump));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::BreakAction) == static_cast<std::uint32_t>(WeaponActionRole::BreakAction));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Cylinder) == static_cast<std::uint32_t>(WeaponActionRole::Cylinder));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Lever) == static_cast<std::uint32_t>(WeaponActionRole::Lever));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponActionRoleV1::Latch) == static_cast<std::uint32_t>(WeaponActionRole::Latch));
}
