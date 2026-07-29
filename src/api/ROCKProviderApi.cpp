#define ROCK_API_EXPORTS
#include "ROCKProviderApiInternal.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ProviderLeasePolicy.h"
#include "api/TouchGrabRegistry.h"

#include <array>
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string_view>

#include "physics-interaction/object/ExternalBodyRegistry.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"

#ifdef DrawText
#undef DrawText
#endif

namespace
{
    using namespace rock::provider;
    using namespace rock;

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
    // Public V1 part-kind / action-role values are a wire contract for
    // external consumers (PAPER_Toolkit); pin every enumerator to the internal
    // classification enums so a reorder breaks this build, not a consumer.
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

    struct CallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        RockProviderFrameCallback callback{ nullptr };
        void* userData{ nullptr };
    };

    std::atomic<PhysicsInteraction*> s_physicsInteraction{ nullptr };
    std::atomic<std::uint64_t> s_nextFrameIndex{ 1 };
    std::atomic<std::uint64_t> s_nextCallbackToken{ 1 };
    std::mutex s_callbackMutex;
    std::array<CallbackSlot, 16> s_callbacks{};

    std::mutex s_snapshotMutex;
    RockProviderFrameSnapshot s_lastSnapshot{};
    bool s_hasSnapshot{ false };
    std::atomic<bool> s_generationStateAvailable{ false };
    std::atomic<std::uint32_t> s_currentWorldGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentSkeletonGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentProviderGeneration{ 0 };
    // Published together with the frame snapshot; indexed [right, left].
    std::array<RockProviderWeaponPartGripStateV1, 2> s_lastPartGripStates{};
    std::array<RockProviderHandInteractionStateV1, 2> s_lastHandInteractionStates{};
    RockProviderEquippedWeaponStateV1 s_lastEquippedWeaponState{};

    std::mutex s_externalBodyMutex;
    ExternalBodyRegistry s_externalBodies{};
    std::mutex s_touchGrabMutex;
    TouchGrabRegistry s_touchGrabTargets{};

    struct OffhandReservationSlot
    {
        std::uint64_t ownerToken{ 0 };
        RockProviderOffhandReservation reservation{
            RockProviderOffhandReservation::Normal
        };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    std::mutex s_offhandReservationMutex;
    OffhandReservationSlot s_offhandReservationSlot{};
    std::atomic<std::uint64_t> s_offhandReservationOwner{ 0 };
    std::atomic<std::uint64_t> s_offhandReservationExpiry{ 0 };
    std::atomic<std::uint32_t> s_offhandReservation{
        static_cast<std::uint32_t>(RockProviderOffhandReservation::Normal)
    };

    // ROCK's runtime firing hand (left-hand fire); published each frame by
    // PhysicsInteraction so primary/offhand resolution tracks who fires.
    std::atomic<bool> s_equippedWeaponFiringHandIsLeft{ false };

    constexpr std::uint64_t kRockIssuedOwnerTokenNamespace = 0xA000'0000'0000'0000ull;
    constexpr std::uint64_t kRockIssuedOwnerTokenSequenceMask = 0x0FFF'FFFF'FFFF'FFFFull;
    constexpr std::uint32_t kImplementedConsumerCapabilitiesV1 =
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::FrameSnapshots) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodies) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::OffhandReservation) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::InteractionCommands) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponPartInteraction) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::NativeAnimationAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::AnimationPhases) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::EquippedWeaponGripState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandVisualAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::DebugOverlayPublication) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ProviderEvents) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::HandInteractionState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodyScopes) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponPartObservability) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WeaponComposition) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PoseReadback) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::SemanticHandContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PlayerColliderDescriptors) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ScopeSightState) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::InputObservability) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::TouchGrabTargets) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WorldRaycasts);
    constexpr std::uint32_t kProviderFeatureBitsV1 =
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::FrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::LifecycleFields) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEvidence) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::BodyContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ConsumerRegistrationV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::OwnerFilteredExternalContactsV1) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::InteractionCommandQueue) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceGrabCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ForceReleaseCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::ThrownDropCommand) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartInteraction) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartGripState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartRecordIdentity) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponPartTargetNonExclusive) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::RawWandButtonState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::PipboyInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::WeaponEmitters) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::NativeAnimationAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::AnimationPhases) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponGripState) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::HandVisualAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::NativeAnimationRuntimeProvider) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponHandlingAuthority) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::DebugOverlayPublication) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::PresentedHandFrames) |
        static_cast<std::uint32_t>(RockProviderFeatureBitV1::EquippedWeaponHandRequest);
    constexpr std::uint32_t kProviderFeatureBits2V1 =
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SafeDescriptor) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExtendedLimits) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PublicStructureSizes) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OwnerFrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::HandInteractionState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ProviderEvents) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::EquippedWeaponState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalBodyScopes) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalContactCursor) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartResolution) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartPoses) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponPartDriveResults) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ScopeSightState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponComposition) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::AuthoredGripSnapshot) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PresentedHandPose) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SemanticHandContacts) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::PlayerColliderDescriptors) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::HandCollisionAvailability) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::CommandCancellation) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::InputSuppressionState) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OffhandReservationLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::SnapshotEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::NativeAnimationRuntimeLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::StatefulPublicationLeases) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::CommandLifecycle) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::InputSampleMetadata) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WeaponClassificationEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExternalContactEnrichment) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::TouchGrabTargets) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::NativeVatsVansInputSuppression) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::WorldRaycasts);
    constexpr std::uint32_t kImplementedForceGrabFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame);
    constexpr std::uint32_t kImplementedForceReleaseFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok);
    constexpr std::uint32_t kImplementedThrownDropFlagsV1 =
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget) |
        static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok);
    constexpr std::uint32_t kImplementedHandInputSuppressionFlagsV1 =
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVats) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVans);
    constexpr std::uint32_t kWeaponPartTargetMatcherFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole);
    constexpr std::uint32_t kImplementedWeaponPartTargetFlagsV1 =
        kWeaponPartTargetMatcherFlagsV1 |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::NonExclusive);
    constexpr std::uint32_t kImplementedWeaponPartDriveMatcherFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName);
    constexpr std::uint32_t kProviderInvalidBodyId = 0x7FFF'FFFFu;

    struct ConsumerSlot
    {
        std::uint64_t token{ 0 };
        std::uint32_t grantedCapabilities{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t worldRaycastFrameIndex{ 0 };
        std::uint32_t worldRaycastCount{ 0 };
        char modName[64]{};
    };

    std::mutex s_consumerMutex;
    std::array<ConsumerSlot, ROCK_PROVIDER_MAX_CONSUMERS_V1> s_consumers{};
    std::atomic<std::uint64_t> s_nextConsumerTokenSequence{ 1 };

    struct InteractionCommandSlot
    {
        bool active{ false };
        QueuedInteractionCommandV1 command{};
    };

    struct InteractionCommandResultSlot
    {
        bool active{ false };
        RockProviderInteractionCommandResultV1 result{};
    };

    std::mutex s_interactionCommandMutex;
    std::array<InteractionCommandSlot, ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1> s_interactionCommands{};
    std::array<InteractionCommandResultSlot, ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1> s_interactionResults{};
    static_assert(
        ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1 >= ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 + 3,
        "Result history must retain the full queue, both deferred force-grab slots, and one dequeued command.");
    std::size_t s_nextInteractionResultSlot{ 0 };
    std::atomic<std::uint64_t> s_nextInteractionCommandId{ 1 };
    interaction_command_policy::ForceGrabReservations s_forceGrabReservations{};

    std::mutex s_providerEventMutex;
    std::array<RockProviderEventV1, ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1>
        s_providerEvents{};
    std::uint32_t s_providerEventCount{ 0 };
    std::uint32_t s_providerEventHead{ 0 };
    std::uint64_t s_nextProviderEventSequence{ 1 };
    std::uint64_t s_overwrittenProviderEventCount{ 0 };

    struct HandInputSuppressionSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        RockProviderSuppressionInvalidationReasonV1 lastInvalidationReason{
            RockProviderSuppressionInvalidationReasonV1::None
        };
        std::uint64_t lastInvalidatedFrame{ 0 };
    };

    std::mutex s_handInputSuppressionMutex;
    std::array<HandInputSuppressionSlot, ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1> s_handInputSuppressions{};

    struct NativeAnimationAuthoritySlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t expiresAtFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    std::mutex s_nativeAnimationAuthorityMutex;
    std::array<NativeAnimationAuthoritySlot, ROCK_PROVIDER_MAX_CONSUMERS_V1> s_nativeAnimationAuthoritySlots{};
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityFlags{ 0 };
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityOwnerCount{ 0 };

    struct AnimationPhaseCallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        RockProviderAnimationPhaseCallbackV1 callback{ nullptr };
        void* userData{ nullptr };
    };

    std::mutex s_animationPhaseCallbackMutex;
    std::array<AnimationPhaseCallbackSlot, ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1>
        s_animationPhaseCallbacks{};
    std::atomic<std::uint64_t> s_nextAnimationPhaseCallbackToken{ 1 };
    std::atomic<std::uint64_t> s_nextAnimationPhaseFrameIndex{ 1 };
    std::atomic<std::uint64_t> s_activeAnimationPhaseFrameIndex{ 0 };
    std::atomic<std::uint32_t> s_animationOwnerThreadId{ 0 };
    std::atomic<bool> s_animationThreadMismatchLogged{ false };

    struct HandVisualAuthoritySlot
    {
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t publishedFlags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        char tag[64]{};
    };

    std::mutex s_handVisualAuthorityMutex;
    std::array<HandVisualAuthoritySlot, ROCK_PROVIDER_MAX_CONSUMERS_V1 * 2>
        s_handVisualAuthoritySlots{};

    std::mutex s_nativeAnimationRuntimePublicationMutex;
    std::uint64_t s_nativeAnimationRuntimeProviderOwner{ 0 };
    RockProviderNativeAnimationRuntimePublicationV1
        s_nativeAnimationRuntimePublication{};
    bool s_hasNativeAnimationRuntimePublication{ false };
    std::uint64_t s_nativeAnimationRuntimeExpiresAfterFrame{ 0 };

    struct EquippedWeaponHandlingAuthoritySlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        RockProviderEquippedWeaponHandlingRequestV1 request{};
    };

    std::mutex s_equippedWeaponHandlingAuthorityMutex;
    EquippedWeaponHandlingAuthoritySlot s_equippedWeaponHandlingAuthority{};

    struct WeaponPartTargetSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderWeaponPartTargetV1 target{};
    };

    struct WeaponPartDriveSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        RockProviderWeaponPartDriveTargetV1 target{};
    };

    std::mutex s_weaponPartMutex;
    std::array<WeaponPartTargetSlot, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> s_weaponPartTargets{};
    std::array<WeaponPartDriveSlot, ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> s_weaponPartDrives{};

    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability);
    [[nodiscard]] bool generationGuardsStale(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration);
    void publishAuthorityLostEvent(
        std::uint64_t ownerToken,
        RockProviderAuthorityKindV1 authorityKind,
        std::uint32_t reason);

    std::uint32_t ROCK_PROVIDER_CALL apiGetVersion() { return ROCK_PROVIDER_API_VERSION; }

    const char* ROCK_PROVIDER_CALL apiGetModVersion()
    {
        static constexpr const char* version = "0.5.0";
        return version;
    }

    bool ROCK_PROVIDER_CALL apiIsProviderReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized();
    }

    std::uint64_t ROCK_PROVIDER_CALL apiRegisterFrameCallback(RockProviderFrameCallback callback, void* userData)
    {
        if (!callback) {
            return 0;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (!slot.callback) {
                slot.token = s_nextCallbackToken.fetch_add(1, std::memory_order_acq_rel);
                slot.ownerToken = 0;
                slot.callback = callback;
                slot.userData = userData;
                return slot.token;
            }
        }

        return 0;
    }

    bool ROCK_PROVIDER_CALL apiUnregisterFrameCallback(std::uint64_t callbackToken)
    {
        if (callbackToken == 0) {
            return false;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (slot.token == callbackToken) {
                slot = {};
                return true;
            }
        }

        return false;
    }

    void clearCallbackSlot(std::uint64_t callbackToken)
    {
        if (callbackToken == 0) {
            return;
        }

        std::scoped_lock lock(s_callbackMutex);
        for (auto& slot : s_callbacks) {
            if (slot.token == callbackToken) {
                slot = {};
                return;
            }
        }
    }

    struct FrameCallbackInvocationResult
    {
        bool healthy{ true };
        std::uint32_t exceptionCode{ 0 };
        std::uintptr_t exceptionAddress{ 0 };
    };

#if defined(_MSC_VER)
    int captureFrameCallbackException(
        EXCEPTION_POINTERS* exception,
        FrameCallbackInvocationResult* result) noexcept
    {
        if (result) {
            result->healthy = false;
            if (exception && exception->ExceptionRecord) {
                result->exceptionCode =
                    exception->ExceptionRecord->ExceptionCode;
                result->exceptionAddress =
                    reinterpret_cast<std::uintptr_t>(
                        exception->ExceptionRecord->ExceptionAddress);
            }
        }
        return EXCEPTION_EXECUTE_HANDLER;
    }
#endif

    FrameCallbackInvocationResult invokeFrameCallbackSafely(
        RockProviderFrameCallback callback,
        const RockProviderFrameSnapshot* snapshot,
        void* userData)
    {
        FrameCallbackInvocationResult result{};
        if (!callback) {
            return result;
        }

#if defined(_MSC_VER)
        __try {
            callback(snapshot, userData);
        } __except (captureFrameCallbackException(
            GetExceptionInformation(),
            &result)) {
        }
#else
        callback(snapshot, userData);
#endif
        return result;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        RockProviderFrameCallback callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (ownerToken == 0 || !callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (!slot.callback) {
                auto token = s_nextCallbackToken.fetch_add(
                    1,
                    std::memory_order_acq_rel);
                if (token == 0) {
                    token = s_nextCallbackToken.fetch_add(
                        1,
                        std::memory_order_acq_rel);
                }
                slot = CallbackSlot{
                    .token = token,
                    .ownerToken = ownerToken,
                    .callback = callback,
                    .userData = userData,
                };
                *outCallbackToken = token;
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (slot.token != callbackToken) {
                continue;
            }
            if (slot.ownerToken != ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            slot = {};
            return RockProviderResultV1::Ok;
        }
        return RockProviderResultV1::TargetUnavailable;
    }

    bool invokeAnimationPhaseCallbackSafely(
        RockProviderAnimationPhaseCallbackV1 callback,
        const RockProviderAnimationPhaseContextV1* context,
        void* userData)
    {
        if (!callback) {
            return true;
        }

#if defined(_MSC_VER)
        __try {
            callback(context, userData);
            return true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return false;
        }
#else
        callback(context, userData);
        return true;
#endif
    }

    [[nodiscard]] bool claimOrValidateAnimationOwnerThread()
    {
        const auto currentThread =
            static_cast<std::uint32_t>(GetCurrentThreadId());
        std::uint32_t expected = 0;
        if (s_animationOwnerThreadId.compare_exchange_strong(
                expected,
                currentThread,
                std::memory_order_acq_rel)) {
            return true;
        }
        return expected == currentThread;
    }

    [[nodiscard]] bool onAnimationOwnerThread()
    {
        const auto ownerThread =
            s_animationOwnerThreadId.load(std::memory_order_acquire);
        return ownerThread != 0 && ownerThread ==
            static_cast<std::uint32_t>(GetCurrentThreadId());
    }

    bool ROCK_PROVIDER_CALL apiGetFrameSnapshot(RockProviderFrameSnapshot* outSnapshot)
    {
        if (!outSnapshot || outSnapshot->size < ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE) {
            return false;
        }

        const auto requestedSize = outSnapshot->size;
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return false;
        }

        const auto copySize = (std::min<std::size_t>)(requestedSize, sizeof(RockProviderFrameSnapshot));
        std::memcpy(outSnapshot, &s_lastSnapshot, copySize);
        outSnapshot->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    RockProviderHand ROCK_PROVIDER_CALL apiGetPrimaryHandV1()
    {
        // ROCK's firing role is the only primary/offhand authority. Native
        // Fallout/FRIK handedness never changes ROCK controller identity.
        const bool primaryIsLeft =
            s_equippedWeaponFiringHandIsLeft.load(std::memory_order_acquire);
        return primaryIsLeft ? RockProviderHand::Left : RockProviderHand::Right;
    }

    RockProviderHand ROCK_PROVIDER_CALL apiGetOffhandHandV1()
    {
        return apiGetPrimaryHandV1() == RockProviderHand::Left ? RockProviderHand::Right : RockProviderHand::Left;
    }

    bool ROCK_PROVIDER_CALL apiGetHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* outFrame)
    {
        /*
         * Hand frames expose ROCK's hand authority as a value snapshot instead
         * of a NiNode lookup. Consumers need the same primary/offhand mapping,
         * body id, and root-flattened transform that ROCK drives each frame,
         * while ROCK deliberately does not promise a live scene node for that
         * authority surface.
         */
        if (!outFrame || outFrame->size < 112) {
            return false;
        }
        const auto requestedSize = outFrame->size;

        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return false;
            }
            snapshot = s_lastSnapshot;
        }

        if (snapshot.providerReady == 0) {
            return false;
        }

        const bool isLeft = hand == RockProviderHand::Left;
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::RootFlattenedAuthority);
        if (isLeft) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Left);
        }
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Offhand);
        }

        frame.transform = isLeft ? snapshot.leftHandTransform : snapshot.rightHandTransform;
        frame.bodyId = isLeft ? snapshot.leftHandBodyId : snapshot.rightHandBodyId;
        frame.state = isLeft ? snapshot.leftHandState : snapshot.rightHandState;
        frame.frameIndex = snapshot.frameIndex;
        frame.worldGeneration = snapshot.worldGeneration;
        frame.skeletonGeneration = snapshot.skeletonGeneration;
        frame.providerGeneration = snapshot.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiGetWeaponPartGripStateV1(RockProviderHand hand, RockProviderWeaponPartGripStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderWeaponPartGripStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return false;
        }
        *outState = s_lastPartGripStates[hand == RockProviderHand::Left ? 1u : 0u];
        return true;
    }

    std::size_t boundedStringLength(const char* value, std::size_t capacity)
    {
        for (std::size_t i = 0; i < capacity; ++i) {
            if (value[i] == '\0') {
                return i;
            }
        }
        return capacity;
    }

    bool modNameEquals(const ConsumerSlot& slot, const char* modName, std::size_t modNameLength)
    {
        return slot.token != 0 &&
               boundedStringLength(slot.modName, sizeof(slot.modName)) == modNameLength &&
               std::memcmp(slot.modName, modName, modNameLength) == 0;
    }

    ConsumerSlot* findConsumerSlotLocked(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return nullptr;
        }

        for (auto& slot : s_consumers) {
            if (slot.token == ownerToken) {
                return &slot;
            }
        }
        return nullptr;
    }

    bool consumerHasCapabilityLocked(std::uint64_t ownerToken, RockProviderConsumerCapabilityV1 capability)
    {
        const auto* slot = findConsumerSlotLocked(ownerToken);
        return slot && hasConsumerCapabilityV1(slot->grantedCapabilities, capability);
    }

    [[nodiscard]] bool finiteProviderTransform(const RockProviderTransform& transform)
    {
        for (const float value : transform.rotate) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return std::isfinite(transform.translate[0]) &&
               std::isfinite(transform.translate[1]) &&
               std::isfinite(transform.translate[2]) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > 0.000001f;
    }

    [[nodiscard]] bool finiteProviderPoint(
        const RockProviderPoint3& point)
    {
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.z);
    }

    [[nodiscard]] RE::NiTransform toNiTransform(const RockProviderTransform& source)
    {
        RE::NiTransform target{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotate.entry[row][column] = source.rotate[row * 3 + column];
            }
        }
        target.translate = RE::NiPoint3(
            source.translate[0],
            source.translate[1],
            source.translate[2]);
        target.scale = source.scale;
        return target;
    }

    [[nodiscard]] RockProviderTransform toProviderTransform(
        const RE::NiTransform& source)
    {
        RockProviderTransform target{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotate[row * 3 + column] =
                    source.rotate.entry[row][column];
            }
        }
        target.translate[0] = source.translate.x;
        target.translate[1] = source.translate.y;
        target.translate[2] = source.translate.z;
        target.scale = source.scale;
        return target;
    }

    [[nodiscard]] constexpr frik_visual_authority::Hand toVisualHand(
        const RockProviderHand hand)
    {
        return hand == RockProviderHand::Left ?
            frik_visual_authority::Hand::Left :
            frik_visual_authority::Hand::Right;
    }

    bool ROCK_PROVIDER_CALL apiGetPresentedHandFrameV1(
        const RockProviderHand hand,
        RockProviderHandFrameV1* outFrame)
    {
        if (!outFrame ||
            outFrame->size < 112 ||
            (hand != RockProviderHand::Right &&
                hand != RockProviderHand::Left) ||
            !onAnimationOwnerThread() ||
            !apiIsProviderReady() ||
            !frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return false;
        }

        const auto presentedWorld =
            frik_visual_authority::getHandWorldTransform(
                toVisualHand(hand));
        const auto providerTransform =
            toProviderTransform(presentedWorld);
        if (!finiteProviderTransform(providerTransform)) {
            return false;
        }

        const auto requestedSize = outFrame->size;
        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (s_hasSnapshot) {
                snapshot = s_lastSnapshot;
            }
        }
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::PresentedVisual);
        if (hand == RockProviderHand::Left) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Left);
        }
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Offhand);
        }
        frame.transform = providerTransform;
        frame.frameIndex = snapshot.frameIndex;
        frame.worldGeneration = snapshot.worldGeneration;
        frame.skeletonGeneration = snapshot.skeletonGeneration;
        frame.providerGeneration = snapshot.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    [[nodiscard]] HandVisualAuthoritySlot* findHandVisualAuthoritySlotLocked(
        const std::uint64_t ownerToken,
        const RockProviderHand hand)
    {
        HandVisualAuthoritySlot* available = nullptr;
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken == ownerToken && slot.hand == hand) {
                return &slot;
            }
            if (slot.ownerToken == 0 && !available) {
                available = &slot;
            }
        }
        return available;
    }

    [[nodiscard]] bool clearHandVisualAuthoritySlotLocked(
        HandVisualAuthoritySlot& slot,
        const bool releaseSlot)
    {
        if (slot.ownerToken == 0) {
            return true;
        }

        const auto hand = toVisualHand(slot.hand);
        const auto worldFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        const auto fingerFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::FingerLocalTransforms);
        bool cleared = true;
        if ((slot.publishedFlags & fingerFlag) != 0) {
            cleared = frik_visual_authority::clearHandPose(slot.tag, hand) && cleared;
        }
        if ((slot.publishedFlags & worldFlag) != 0) {
            cleared = frik_visual_authority::clearExternalHandWorldTransform(slot.tag, hand) && cleared;
        }

        if (cleared || !frik_visual_authority::isSkeletonReadyHint() || releaseSlot) {
            slot = {};
            return true;
        }
        return false;
    }

    [[nodiscard]] bool clearHandVisualAuthorityForOwner(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        const bool releaseSlots)
    {
        bool cleared = true;
        std::scoped_lock lock(s_handVisualAuthorityMutex);
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken != ownerToken ||
                (hand != RockProviderHand::None && slot.hand != hand)) {
                continue;
            }
            cleared = clearHandVisualAuthoritySlotLocked(slot, releaseSlots) &&
                cleared;
        }
        return cleared;
    }

    void pruneHandVisualAuthorityLocked(const std::uint64_t frameIndex)
    {
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken == 0) {
                continue;
            }
            const bool generationChanged = generationGuardsStale(
                slot.worldGeneration,
                slot.skeletonGeneration,
                slot.providerGeneration);
            if (!generationChanged && provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAfterFrame)) {
                continue;
            }

            const auto ownerToken = slot.ownerToken;
            (void)clearHandVisualAuthoritySlotLocked(slot, true);
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::HandVisual,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                        RockProviderSuppressionInvalidationReasonV1::Expired));
        }
    }

    void clearAnimationPhaseCallbacksForOwnerLocked(const std::uint64_t ownerToken)
    {
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void clearNativeAnimationRuntimePublicationForOwner(const std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
        if (s_nativeAnimationRuntimeProviderOwner == ownerToken) {
            s_nativeAnimationRuntimeProviderOwner = 0;
            s_nativeAnimationRuntimePublication = {};
            s_nativeAnimationRuntimeExpiresAfterFrame = 0;
            s_hasNativeAnimationRuntimePublication = false;
        }
    }

    void clearEquippedWeaponHandlingAuthorityForOwnerLocked(
        const std::uint64_t ownerToken)
    {
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken == ownerToken) {
            s_equippedWeaponHandlingAuthority = {};
        }
    }

    void pruneExpiredEquippedWeaponHandlingAuthorityLocked(
        const std::uint64_t frameIndex)
    {
        if (!s_equippedWeaponHandlingAuthority.active) {
            return;
        }
        const bool generationChanged = generationGuardsStale(
            s_equippedWeaponHandlingAuthority.request.worldGeneration,
            s_equippedWeaponHandlingAuthority.request.skeletonGeneration,
            s_equippedWeaponHandlingAuthority.request.providerGeneration);
        if (generationChanged ||
            !provider_lease_policy::isActive(
                frameIndex,
                s_equippedWeaponHandlingAuthority.expiresAfterFrame)) {
            publishAuthorityLostEvent(
                s_equippedWeaponHandlingAuthority.ownerToken,
                RockProviderAuthorityKindV1::EquippedWeaponHandling,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                        RockProviderSuppressionInvalidationReasonV1::Expired));
            s_equippedWeaponHandlingAuthority = {};
        }
    }

    std::uint64_t currentProviderFrameIndex()
    {
        const auto nextFrameIndex = s_nextFrameIndex.load(std::memory_order_acquire);
        return nextFrameIndex > 0 ? nextFrameIndex - 1 : 0;
    }

    void publishProviderEvent(RockProviderEventV1 event)
    {
        event.size = sizeof(RockProviderEventV1);
        event.version = ROCK_PROVIDER_API_VERSION;
        if (event.frameIndex == 0) {
            event.frameIndex = currentProviderFrameIndex();
        }
        if (s_generationStateAvailable.load(std::memory_order_acquire)) {
            if (event.worldGeneration == 0) {
                event.worldGeneration = s_currentWorldGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.skeletonGeneration == 0) {
                event.skeletonGeneration = s_currentSkeletonGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.providerGeneration == 0) {
                event.providerGeneration = s_currentProviderGeneration.load(
                    std::memory_order_acquire);
            }
        }

        std::scoped_lock lock(s_providerEventMutex);
        event.sequence = s_nextProviderEventSequence++;
        if (s_providerEventCount < s_providerEvents.size()) {
            const auto index =
                (s_providerEventHead + s_providerEventCount) %
                s_providerEvents.size();
            s_providerEvents[index] = event;
            ++s_providerEventCount;
        } else {
            s_providerEvents[s_providerEventHead] = event;
            s_providerEventHead =
                (s_providerEventHead + 1) % s_providerEvents.size();
            ++s_overwrittenProviderEventCount;
        }
    }

    [[nodiscard]] const RockProviderEventV1& providerEventAtLocked(
        const std::uint32_t logicalIndex)
    {
        return s_providerEvents[
            (s_providerEventHead + logicalIndex) % s_providerEvents.size()];
    }

    void publishAuthorityLostEvent(
        const std::uint64_t ownerToken,
        const RockProviderAuthorityKindV1 authorityKind,
        const std::uint32_t reason)
    {
        RockProviderEventV1 event{};
        event.kind = RockProviderEventKindV1::AuthorityLost;
        event.ownerToken = ownerToken;
        event.result = reason;
        event.data[0] = static_cast<std::uint32_t>(authorityKind);
        publishProviderEvent(event);
    }

    void publishOffhandReservationLocked(const OffhandReservationSlot& slot)
    {
        s_offhandReservationOwner.store(
            slot.ownerToken,
            std::memory_order_release);
        s_offhandReservation.store(
            static_cast<std::uint32_t>(slot.reservation),
            std::memory_order_release);
        s_offhandReservationExpiry.store(
            slot.expiresAfterFrame,
            std::memory_order_release);
    }

    void clearOffhandReservationLocked(
        const RockProviderSuppressionInvalidationReasonV1 reason)
    {
        const auto ownerToken = s_offhandReservationSlot.ownerToken;
        s_offhandReservationSlot = {};
        publishOffhandReservationLocked(s_offhandReservationSlot);
        if (ownerToken != 0) {
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::OffhandReservation,
                static_cast<std::uint32_t>(reason));
        }
    }

    void pruneExpiredOffhandReservationLocked(const std::uint64_t frameIndex)
    {
        if (s_offhandReservationSlot.ownerToken == 0) {
            return;
        }
        const bool generationChanged = generationGuardsStale(
            s_offhandReservationSlot.worldGeneration,
            s_offhandReservationSlot.skeletonGeneration,
            s_offhandReservationSlot.providerGeneration);
        if (generationChanged ||
            !provider_lease_policy::isActive(
                frameIndex,
                s_offhandReservationSlot.expiresAfterFrame)) {
            clearOffhandReservationLocked(
                generationChanged ?
                    RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                    RockProviderSuppressionInvalidationReasonV1::Expired);
        }
    }

    void pruneExpiredNativeAnimationRuntimePublicationLocked(
        const std::uint64_t frameIndex)
    {
        if (!s_hasNativeAnimationRuntimePublication) {
            return;
        }
        const bool generationChanged = generationGuardsStale(
            s_nativeAnimationRuntimePublication.worldGeneration,
            s_nativeAnimationRuntimePublication.skeletonGeneration,
            s_nativeAnimationRuntimePublication.providerGeneration);
        if (generationChanged ||
            !provider_lease_policy::isActive(
                frameIndex,
                s_nativeAnimationRuntimeExpiresAfterFrame)) {
            const auto ownerToken = s_nativeAnimationRuntimeProviderOwner;
            s_nativeAnimationRuntimeProviderOwner = 0;
            s_nativeAnimationRuntimePublication = {};
            s_nativeAnimationRuntimeExpiresAfterFrame = 0;
            s_hasNativeAnimationRuntimePublication = false;
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::NativeAnimationRuntime,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                        RockProviderSuppressionInvalidationReasonV1::Expired));
        }
    }

    void pruneExpiredHandInputSuppressionsLocked(std::uint64_t frameIndex)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                continue;
            }
            const bool generationChanged = generationGuardsStale(
                slot.worldGeneration,
                slot.skeletonGeneration,
                slot.providerGeneration);
            if (generationChanged || !provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAfterFrame)) {
                const auto reason = generationChanged ?
                    RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                    RockProviderSuppressionInvalidationReasonV1::Expired;
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::HandInputSuppression,
                    static_cast<std::uint32_t>(reason));
                slot.active = false;
                slot.flags = 0;
                slot.lastInvalidationReason = reason;
                slot.lastInvalidatedFrame = frameIndex;
            }
        }
    }

    void clearHandInputSuppressionsForOwnerLocked(
        std::uint64_t ownerToken,
        RockProviderHand hand,
        RockProviderSuppressionInvalidationReasonV1 reason =
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active || slot.ownerToken != ownerToken) {
                continue;
            }
            if (hand == RockProviderHand::None || slot.hand == hand) {
                slot.active = false;
                slot.flags = 0;
                slot.lastInvalidationReason = reason;
                slot.lastInvalidatedFrame = currentProviderFrameIndex();
            }
        }
    }

    void publishNativeAnimationAuthorityAggregateLocked()
    {
        std::uint32_t flags = 0;
        std::uint32_t ownerCount = 0;
        for (const auto& slot : s_nativeAnimationAuthoritySlots) {
            if (!slot.active) {
                continue;
            }
            flags |= slot.flags;
            ++ownerCount;
        }
        s_nativeAnimationAuthorityFlags.store(flags, std::memory_order_release);
        s_nativeAnimationAuthorityOwnerCount.store(ownerCount, std::memory_order_release);
    }

    void pruneExpiredNativeAnimationAuthorityLocked(std::uint64_t frameIndex)
    {
        bool changed = false;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (!slot.active) {
                continue;
            }
            const bool generationChanged = generationGuardsStale(
                slot.worldGeneration,
                slot.skeletonGeneration,
                slot.providerGeneration);
            if (generationChanged || !provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAtFrame)) {
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::NativeAnimation,
                    static_cast<std::uint32_t>(
                        generationChanged ?
                            RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                            RockProviderSuppressionInvalidationReasonV1::Expired));
                slot = {};
                changed = true;
            }
        }
        if (changed) {
            publishNativeAnimationAuthorityAggregateLocked();
        }
    }

    void clearNativeAnimationAuthorityForOwnerLocked(std::uint64_t ownerToken)
    {
        bool changed = false;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
                changed = true;
            }
        }
        if (changed) {
            publishNativeAnimationAuthorityAggregateLocked();
        }
    }

    void clearWeaponPartTargetsForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartTargets) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void clearWeaponPartDrivesForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartDrives) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void pruneExpiredWeaponPartDrivesLocked(std::uint64_t frameIndex)
    {
        for (auto& slot : s_weaponPartDrives) {
            if (slot.active && !provider_lease_policy::isActive(
                    frameIndex,
                    slot.expiresAfterFrame)) {
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::WeaponPartDrive,
                    static_cast<std::uint32_t>(
                        RockProviderSuppressionInvalidationReasonV1::Expired));
                slot = {};
            }
        }
    }

    bool hasValidWeaponPartMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        // NonExclusive is a semantics flag, not a matcher: at least one match
        // flag must still be present for the target to select anything.
        if ((flags & ~kImplementedWeaponPartTargetFlagsV1) != 0 || (flags & kWeaponPartTargetMatcherFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return true;
    }

    bool hasConcreteWeaponPartDriveMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        if ((flags & ~kImplementedWeaponPartDriveMatcherFlagsV1) != 0 || (flags & kImplementedWeaponPartDriveMatcherFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot != 0) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId != kProviderInvalidBodyId) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
                   boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) != 0);
    }

    bool isValidWeaponPartKindValue(std::uint32_t value)
    {
        return value < static_cast<std::uint32_t>(WeaponPartKind::Count);
    }

    bool isValidWeaponReloadRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponReloadRole::Receiver);
    }

    bool isValidWeaponSupportRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSupportGripRole::ReceiverSupport);
    }

    bool isValidWeaponSocketRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSocketRole::LoadingGate);
    }

    bool isValidWeaponActionRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponActionRole::Latch);
    }

    bool hasValidWeaponPartTargetSemantics(const RockProviderWeaponPartTargetV1& target)
    {
        const auto flags = target.flags;
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind)) != 0 &&
            !isValidWeaponPartKindValue(target.partKind)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole)) != 0 &&
            !isValidWeaponReloadRoleValue(target.reloadRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole)) != 0 &&
            !isValidWeaponSupportRoleValue(target.supportRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole)) != 0 &&
            !isValidWeaponSocketRoleValue(target.socketRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole)) != 0 &&
            !isValidWeaponActionRoleValue(target.actionRole)) {
            return false;
        }
        return true;
    }

    std::size_t availableWeaponPartTargetSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartTargets) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }

    std::size_t availableWeaponPartDriveSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }

    bool isValidWeaponPartGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        return mode == RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority ||
               mode == RockProviderWeaponPartGrabModeV1::AttachOnly;
    }

    bool isValidWeaponPartDriveSpace(RockProviderWeaponPartDriveSpaceV1 space)
    {
        return space == RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal ||
               space == RockProviderWeaponPartDriveSpaceV1::SourceParentLocal;
    }

    bool isFiniteProviderTransform(const RockProviderTransform& transform)
    {
        for (float value : transform.rotate) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return std::isfinite(transform.translate[0]) &&
               std::isfinite(transform.translate[1]) &&
               std::isfinite(transform.translate[2]) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > 0.0001f;
    }

    weapon_part_runtime::GrabMode toRuntimeGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        switch (mode) {
        case RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority:
            return weapon_part_runtime::GrabMode::FullTwoHandAuthority;
        case RockProviderWeaponPartGrabModeV1::AttachOnly:
            return weapon_part_runtime::GrabMode::AttachOnly;
        case RockProviderWeaponPartGrabModeV1::None:
        default:
            return weapon_part_runtime::GrabMode::None;
        }
    }

    RockProviderWeaponPartGrabModeV1 fromRuntimeGrabMode(weapon_part_runtime::GrabMode mode)
    {
        switch (mode) {
        case weapon_part_runtime::GrabMode::FullTwoHandAuthority:
            return RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority;
        case weapon_part_runtime::GrabMode::AttachOnly:
            return RockProviderWeaponPartGrabModeV1::AttachOnly;
        case weapon_part_runtime::GrabMode::None:
        default:
            return RockProviderWeaponPartGrabModeV1::None;
        }
    }

    weapon_part_runtime::Target toRuntimeTarget(const WeaponPartTargetSlot& slot)
    {
        weapon_part_runtime::Target target{};
        if (!slot.active) {
            return target;
        }

        target.active = true;
        target.ownerToken = slot.ownerToken;
        target.weaponGenerationKey = slot.target.weaponGenerationKey;
        target.flags = slot.target.flags;
        target.grabMode = toRuntimeGrabMode(slot.target.grabMode);
        target.bodyId = slot.target.bodyId;
        target.sourceRoot = slot.target.sourceRoot;
        std::memcpy(target.sourceName.data(), slot.target.sourceName, target.sourceName.size());
        target.sourceName[target.sourceName.size() - 1] = '\0';
        target.partKind = static_cast<WeaponPartKind>(slot.target.partKind);
        target.reloadRole = static_cast<WeaponReloadRole>(slot.target.reloadRole);
        target.supportRole = static_cast<WeaponSupportGripRole>(slot.target.supportRole);
        target.socketRole = static_cast<WeaponSocketRole>(slot.target.socketRole);
        target.actionRole = static_cast<WeaponActionRole>(slot.target.actionRole);
        target.groupId = slot.target.groupId;
        target.priority = slot.target.priority;
        return target;
    }

    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (!findConsumerSlotLocked(ownerToken)) {
            return RockProviderResultV1::OwnerNotRegistered;
        }
        if (!consumerHasCapabilityLocked(ownerToken, capability)) {
            return RockProviderResultV1::PermissionDenied;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 validateInteractionCommandOwnerLocked(std::uint64_t ownerToken)
    {
        return validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::InteractionCommands);
    }

    RockProviderResultV1 validateGenerationGuards(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration)
    {
        if (worldGeneration == 0 && skeletonGeneration == 0 && providerGeneration == 0) {
            return RockProviderResultV1::Ok;
        }

        if (!s_generationStateAvailable.load(std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        if (worldGeneration != 0 &&
            worldGeneration != s_currentWorldGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::WorldNotReady;
        }
        if (skeletonGeneration != 0 &&
            skeletonGeneration != s_currentSkeletonGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        if (providerGeneration != 0 &&
            providerGeneration != s_currentProviderGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
    }

    std::uint32_t currentProviderGenerationForRegistration()
    {
        return s_generationStateAvailable.load(std::memory_order_acquire) ?
            s_currentProviderGeneration.load(std::memory_order_acquire) :
            0;
    }

    std::uint64_t nextConsumerToken()
    {
        const auto sequence = s_nextConsumerTokenSequence.fetch_add(1, std::memory_order_acq_rel);
        return kRockIssuedOwnerTokenNamespace | (sequence & kRockIssuedOwnerTokenSequenceMask);
    }

    std::uint64_t nextInteractionCommandId()
    {
        auto id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        if (id == 0) {
            id = s_nextInteractionCommandId.fetch_add(1, std::memory_order_acq_rel);
        }
        return id;
    }

    RockProviderHand commandHand(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.hand;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.hand;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.hand;
        default:
            return RockProviderHand::None;
        }
    }

    std::uint32_t commandTargetFormId(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.targetFormId;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.targetFormId;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.targetFormId;
        default:
            return 0;
        }
    }

    std::uint32_t commandTargetBodyId(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.targetBodyId;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.targetBodyId;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.targetBodyId;
        default:
            return kProviderInvalidBodyId;
        }
    }

    std::uint32_t commandWorldGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.worldGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.worldGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.worldGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandSkeletonGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.skeletonGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.skeletonGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.skeletonGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandProviderGeneration(const QueuedInteractionCommandV1& command)
    {
        switch (command.kind) {
        case RockProviderInteractionCommandKindV1::ForceGrab:
            return command.forceGrab.providerGeneration;
        case RockProviderInteractionCommandKindV1::ForceRelease:
            return command.forceRelease.providerGeneration;
        case RockProviderInteractionCommandKindV1::ThrownDrop:
            return command.thrownDrop.providerGeneration;
        default:
            return 0;
        }
    }

    RockProviderInteractionCommandResultV1 makeCommandResult(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        RockProviderInteractionCommandResultV1 result{};
        result.size = sizeof(RockProviderInteractionCommandResultV1);
        result.version = ROCK_PROVIDER_API_VERSION;
        result.ownerToken = command.ownerToken;
        result.commandId = command.commandId;
        result.kind = command.kind;
        result.state = state;
        result.failure = failure;
        result.hand = commandHand(command);
        result.targetFormId = commandTargetFormId(command);
        result.targetBodyId = commandTargetBodyId(command);
        result.worldGeneration = commandWorldGeneration(command);
        result.skeletonGeneration = commandSkeletonGeneration(command);
        result.providerGeneration = commandProviderGeneration(command);
        result.stage = interaction_command_policy::isTerminal(state) ?
            RockProviderCommandStageV1::Terminal :
            RockProviderCommandStageV1::Queued;
        result.failureStage = failure;
        result.acceptedFrame = currentProviderFrameIndex();
        if (result.stage == RockProviderCommandStageV1::Terminal) {
            result.frameIndex = result.acceptedFrame;
        }
        return result;
    }

    void storeInteractionResultLocked(const RockProviderInteractionCommandResultV1& result)
    {
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == result.ownerToken && slot.result.commandId == result.commandId) {
                const bool wasTerminal =
                    interaction_command_policy::isTerminal(slot.result.state);
                const auto merged = interaction_command_policy::mergeResultHistory(
                    slot.result,
                    result,
                    currentProviderFrameIndex());
                slot.result = merged;
                if (!wasTerminal &&
                    interaction_command_policy::isTerminal(merged.state)) {
                    RockProviderEventV1 event{};
                    event.kind =
                        RockProviderEventKindV1::InteractionCommandTerminal;
                    event.ownerToken = merged.ownerToken;
                    event.hand = merged.hand;
                    event.subjectSequence = merged.commandId;
                    event.result = static_cast<std::uint32_t>(merged.state);
                    event.data[0] = static_cast<std::uint32_t>(merged.kind);
                    event.data[1] = static_cast<std::uint32_t>(merged.failure);
                    event.formId = merged.targetFormId;
                    publishProviderEvent(event);
                }
                return;
            }
        }

        /*
         * Queued results are live command state, not disposable polling
         * history. The bounded queue cannot produce more live commands than
         * this result table can hold, so rotate only through empty/terminal
         * slots and never make an in-flight command disappear from polling.
         */
        for (std::size_t offset = 0; offset < s_interactionResults.size(); ++offset) {
            const std::size_t index = (s_nextInteractionResultSlot + offset) % s_interactionResults.size();
            auto& slot = s_interactionResults[index];
            if (slot.active && !interaction_command_policy::isTerminal(slot.result.state)) {
                continue;
            }
            auto storedResult = result;
            if (interaction_command_policy::isTerminal(storedResult.state)) {
                storedResult.stage = RockProviderCommandStageV1::Terminal;
                if (storedResult.frameIndex == 0) {
                    storedResult.frameIndex = currentProviderFrameIndex();
                }
                if (storedResult.state ==
                        RockProviderInteractionCommandStateV1::Succeeded &&
                    storedResult.appliedFrame == 0) {
                    storedResult.appliedFrame = storedResult.frameIndex;
                }
                if (storedResult.failureStage ==
                    RockProviderInteractionFailureV1::None) {
                    storedResult.failureStage = storedResult.failure;
                }
            }
            slot = InteractionCommandResultSlot{
                .active = true,
                .result = storedResult,
            };
            if (interaction_command_policy::isTerminal(storedResult.state)) {
                RockProviderEventV1 event{};
                event.kind = RockProviderEventKindV1::InteractionCommandTerminal;
                event.ownerToken = storedResult.ownerToken;
                event.hand = storedResult.hand;
                event.subjectSequence = storedResult.commandId;
                event.result = static_cast<std::uint32_t>(storedResult.state);
                event.data[0] = static_cast<std::uint32_t>(storedResult.kind);
                event.data[1] = static_cast<std::uint32_t>(storedResult.failure);
                event.formId = storedResult.targetFormId;
                publishProviderEvent(event);
            }
            s_nextInteractionResultSlot = (index + 1) % s_interactionResults.size();
            return;
        }
    }

    void completeInteractionCommandLocked(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure)
    {
        const auto result = makeCommandResult(command, state, failure);
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
    }

    void clearInteractionCommandsForOwnerLocked(std::uint64_t ownerToken, RockProviderInteractionFailureV1 failure)
    {
        if (ownerToken == 0) {
            return;
        }

        for (auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken) {
                completeInteractionCommandLocked(slot.command, RockProviderInteractionCommandStateV1::Cancelled, failure);
                slot = {};
            }
        }

        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken) {
                slot = {};
            }
        }
        s_forceGrabReservations.clearOwner(ownerToken);
    }

    void clearOwnerStateAfterCallbackFault(const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        {
            std::scoped_lock lock(
                s_interactionCommandMutex,
                s_handInputSuppressionMutex,
                s_weaponPartMutex,
                s_nativeAnimationAuthorityMutex,
                s_equippedWeaponHandlingAuthorityMutex);
            clearInteractionCommandsForOwnerLocked(
                ownerToken,
                RockProviderInteractionFailureV1::InvalidRequest);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                RockProviderHand::None,
                RockProviderSuppressionInvalidationReasonV1::CallbackFault);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }
        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(
                    RockProviderSuppressionInvalidationReasonV1::CallbackFault);
            }
        }
        {
            std::scoped_lock lock(s_callbackMutex);
            for (auto& callback : s_callbacks) {
                if (callback.ownerToken == ownerToken) {
                    callback = {};
                }
            }
        }
        {
            std::scoped_lock lock(s_animationPhaseCallbackMutex);
            clearAnimationPhaseCallbacksForOwnerLocked(ownerToken);
        }
        (void)clearHandVisualAuthorityForOwner(
            ownerToken,
            RockProviderHand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
        provider_debug_overlay::clear(ownerToken);
        publishAuthorityLostEvent(
            ownerToken,
            RockProviderAuthorityKindV1::Unknown,
            static_cast<std::uint32_t>(
                RockProviderSuppressionInvalidationReasonV1::CallbackFault));
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterConsumerV1(
        const RockProviderConsumerRegistrationV1* registration,
        RockProviderConsumerHandleV1* outHandle)
    {
        if (!registration || !outHandle) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (registration->size != sizeof(RockProviderConsumerRegistrationV1) || outHandle->size != sizeof(RockProviderConsumerHandleV1)) {
            return RockProviderResultV1::InvalidSize;
        }

        if (registration->version == 0 || registration->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }

        const auto modNameLength = boundedStringLength(registration->modName, sizeof(registration->modName));
        if (modNameLength == 0 || modNameLength >= sizeof(registration->modName)) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto grantedCapabilities = registration->requestedCapabilities & kImplementedConsumerCapabilitiesV1;
        const auto providerGeneration = currentProviderGenerationForRegistration();

        std::scoped_lock lock(s_consumerMutex);
        for (const auto& slot : s_consumers) {
            if (modNameEquals(slot, registration->modName, modNameLength)) {
                return RockProviderResultV1::OwnerConflict;
            }
        }

        for (auto& slot : s_consumers) {
            if (slot.token != 0) {
                continue;
            }

            slot = {};
            slot.token = nextConsumerToken();
            slot.grantedCapabilities = grantedCapabilities;
            slot.providerGeneration = providerGeneration;
            std::memcpy(slot.modName, registration->modName, modNameLength);

            *outHandle = {};
            outHandle->size = sizeof(RockProviderConsumerHandleV1);
            outHandle->version = ROCK_PROVIDER_API_VERSION;
            outHandle->ownerToken = slot.token;
            outHandle->grantedCapabilities = slot.grantedCapabilities;
            outHandle->providerGeneration = slot.providerGeneration;
            return RockProviderResultV1::Ok;
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterConsumerV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        {
            std::scoped_lock lock(
                s_consumerMutex,
                s_interactionCommandMutex,
                s_handInputSuppressionMutex,
                s_weaponPartMutex,
                s_nativeAnimationAuthorityMutex,
                s_equippedWeaponHandlingAuthorityMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            *slot = {};
            clearInteractionCommandsForOwnerLocked(ownerToken, RockProviderInteractionFailureV1::OwnerNotRegistered);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                RockProviderHand::None,
                RockProviderSuppressionInvalidationReasonV1::OwnerUnregistered);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }

        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(
                    RockProviderSuppressionInvalidationReasonV1::OwnerUnregistered);
            }
        }

        {
            std::scoped_lock lock(s_callbackMutex);
            for (auto& callback : s_callbacks) {
                if (callback.ownerToken == ownerToken) {
                    callback = {};
                }
            }
        }

        {
            std::scoped_lock lock(s_animationPhaseCallbackMutex);
            clearAnimationPhaseCallbacksForOwnerLocked(ownerToken);
        }
        (void)clearHandVisualAuthorityForOwner(
            ownerToken,
            RockProviderHand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
        provider_debug_overlay::clear(ownerToken);

        return RockProviderResultV1::Ok;
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetGrantedCapabilitiesV1(std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_consumerMutex);
        auto* slot = findConsumerSlotLocked(ownerToken);
        return slot ? slot->grantedCapabilities : 0;
    }

    bool ROCK_PROVIDER_CALL apiGetProviderLimitsV1(RockProviderLimitsV1* outLimits)
    {
        if (!outLimits || outLimits->size <
                offsetof(RockProviderLimitsV1, featureBits) +
                    sizeof(outLimits->featureBits)) {
            return false;
        }

        RockProviderLimitsV1 limits{};
        limits.size = sizeof(RockProviderLimitsV1);
        limits.version = ROCK_PROVIDER_API_VERSION;
        limits.featureBits = kProviderFeatureBitsV1;
        limits.maxFrameCallbacks = ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1;
        limits.maxConsumers = ROCK_PROVIDER_MAX_CONSUMERS_V1;
        limits.maxExternalBodies = ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        limits.maxExternalContacts = ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        limits.maxBodyContacts = ROCK_PROVIDER_MAX_BODY_CONTACTS_V1;
        limits.maxWeaponBodies = ROCK_PROVIDER_MAX_WEAPON_BODIES;
        limits.maxInteractionCommands = ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1;
        limits.maxCompletedInteractionCommands = ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1;
        limits.providerApiByteSize = static_cast<std::uint32_t>(sizeof(RockProviderApi));
        limits.maxWeaponEmitters = ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1;
        limits.maxAnimationPhaseCallbacks =
            ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1;
        limits.maxHandVisualAuthorityPublications =
            static_cast<std::uint32_t>(s_handVisualAuthoritySlots.size());
        limits.maxNativeAnimationRuntimeProviders = 1;
        limits.maxEquippedWeaponHandlingAuthorities = 1;
        limits.maxEquippedWeaponHandlingLeaseFrames =
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublishers =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1;
        limits.maxDebugOverlayLinesPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1;
        limits.maxDebugOverlayTextPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1;
        limits.maxDebugOverlayLines =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1;
        limits.maxDebugOverlayText =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1;
        const auto copySize = (std::min<std::size_t>)(
            outLimits->size,
            sizeof(limits));
        std::memcpy(outLimits, &limits, copySize);
        outLimits->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiGetProviderLimitsExtV1(
        RockProviderLimitsExtV1* outLimits)
    {
        if (!outLimits || outLimits->size <
                offsetof(RockProviderLimitsExtV1, featureBits2) +
                    sizeof(outLimits->featureBits2)) {
            return false;
        }
        RockProviderLimitsExtV1 limits{};
        limits.featureBits = kProviderFeatureBitsV1;
        limits.featureBits2 = kProviderFeatureBits2V1;
        limits.providerApiByteSize = sizeof(RockProviderApi);
        limits.maxConsumers = ROCK_PROVIDER_MAX_CONSUMERS_V1;
        limits.maxFrameCallbacks = ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1;
        limits.maxExternalBodies = ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1;
        limits.maxExternalScopes = ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1;
        limits.maxExternalContacts = ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1;
        limits.maxBodyContacts = ROCK_PROVIDER_MAX_BODY_CONTACTS_V1;
        limits.maxWeaponBodies = ROCK_PROVIDER_MAX_WEAPON_BODIES;
        limits.maxWeaponEmitters = ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1;
        limits.maxInteractionCommands = ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1;
        limits.maxCompletedInteractionCommands =
            ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1;
        limits.maxHandInputSuppressions =
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1;
        limits.maxHandInputSuppressionLeaseFrames =
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1;
        limits.maxWeaponPartTargets = ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1;
        limits.maxWeaponPartDrives = ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1;
        limits.maxWeaponPartDriveLeaseFrames =
            ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1;
        limits.maxWeaponPartPoses = ROCK_PROVIDER_MAX_WEAPON_PART_POSES_V1;
        limits.maxWeaponPartDriveResults =
            ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1;
        limits.maxNativeAnimationAuthorityLeaseFrames =
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1;
        limits.maxAnimationPhaseCallbacks =
            ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1;
        limits.maxHandVisualAuthorityPublications =
            static_cast<std::uint32_t>(s_handVisualAuthoritySlots.size());
        limits.maxNativeAnimationRuntimeProviders = 1;
        limits.maxEquippedWeaponHandlingAuthorities = 1;
        limits.maxEquippedWeaponHandlingLeaseFrames =
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublishers =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1;
        limits.maxDebugOverlayLinesPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1;
        limits.maxDebugOverlayTextPerPublisher =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1;
        limits.maxDebugOverlayLines = ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1;
        limits.maxDebugOverlayText = ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1;
        limits.maxProviderEvents = ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1;
        limits.maxWeaponCompositionEntries =
            ROCK_PROVIDER_MAX_WEAPON_COMPOSITION_ENTRIES_V1;
        limits.maxSemanticHandContacts =
            ROCK_PROVIDER_MAX_SEMANTIC_HAND_CONTACTS_V1;
        limits.maxPlayerColliderDescriptors =
            ROCK_PROVIDER_MAX_PLAYER_COLLIDER_DESCRIPTORS_V1;
        limits.maxOffhandReservationLeaseFrames =
            ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1;
        limits.maxHandVisualAuthorityLeaseFrames =
            ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1;
        limits.maxNativeAnimationRuntimeLeaseFrames =
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1;
        limits.maxDebugOverlayPublicationLeaseFrames =
            ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1;
        limits.maxNativeAnimationAuthorityOwners =
            static_cast<std::uint32_t>(s_nativeAnimationAuthoritySlots.size());
        limits.maxWeaponEvidenceDetails =
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1;
        limits.maxWeaponEvidencePointsPerDetail =
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_POINTS_PER_DETAIL_V1;
        limits.maxTouchGrabTargets =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1;
        limits.maxTouchGrabScopes =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1;
        limits.maxTouchGrabTargetLeaseFrames =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1;
        limits.maxWorldRaycastsPerOwnerPerFrame =
            ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1;

        const auto copySize = (std::min<std::size_t>)(
            outLimits->size,
            sizeof(limits));
        std::memcpy(outLimits, &limits, copySize);
        outLimits->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetPublicStructureSizeV1(
        const RockProviderStructureIdV1 structureId)
    {
        switch (structureId) {
        case RockProviderStructureIdV1::ApiDescriptor:
            return sizeof(RockProviderApiDescriptorV1);
        case RockProviderStructureIdV1::ConsumerRegistration:
            return sizeof(RockProviderConsumerRegistrationV1);
        case RockProviderStructureIdV1::ConsumerHandle:
            return sizeof(RockProviderConsumerHandleV1);
        case RockProviderStructureIdV1::Limits:
            return sizeof(RockProviderLimitsV1);
        case RockProviderStructureIdV1::LimitsExt:
            return sizeof(RockProviderLimitsExtV1);
        case RockProviderStructureIdV1::FrameSnapshot:
            return sizeof(RockProviderFrameSnapshot);
        case RockProviderStructureIdV1::HandFrame:
            return sizeof(RockProviderHandFrameV1);
        case RockProviderStructureIdV1::HandInteractionState:
            return sizeof(RockProviderHandInteractionStateV1);
        case RockProviderStructureIdV1::ProviderEvent:
            return sizeof(RockProviderEventV1);
        case RockProviderStructureIdV1::ProviderEventStreamState:
            return sizeof(RockProviderEventStreamStateV1);
        case RockProviderStructureIdV1::EquippedWeaponState:
            return sizeof(RockProviderEquippedWeaponStateV1);
        case RockProviderStructureIdV1::ExternalBodyRegistration:
            return sizeof(RockProviderExternalBodyRegistration);
        case RockProviderStructureIdV1::ExternalContact:
            return sizeof(RockProviderExternalContactV1);
        case RockProviderStructureIdV1::ExternalContactRecord:
            return sizeof(RockProviderExternalContactRecordV1);
        case RockProviderStructureIdV1::ExternalContactStreamState:
            return sizeof(RockProviderExternalContactStreamStateV1);
        case RockProviderStructureIdV1::WeaponPartTargetQuery:
            return sizeof(RockProviderWeaponPartResolutionQueryV1);
        case RockProviderStructureIdV1::WeaponPartTargetResolution:
            return sizeof(RockProviderWeaponPartResolutionResultV1);
        case RockProviderStructureIdV1::WeaponPartPose:
            return sizeof(RockProviderWeaponPartPoseV1);
        case RockProviderStructureIdV1::WeaponPartDriveResult:
            return sizeof(RockProviderWeaponPartDriveApplicationResultV1);
        case RockProviderStructureIdV1::ScopeSightState:
            return sizeof(RockProviderScopeSightStateV1);
        case RockProviderStructureIdV1::WeaponCompositionState:
            return sizeof(RockProviderWeaponCompositionStateV1);
        case RockProviderStructureIdV1::WeaponCompositionEntry:
            return sizeof(RockProviderWeaponCompositionEntryV1);
        case RockProviderStructureIdV1::AuthoredGripPose:
            return sizeof(RockProviderAuthoredGripPoseV1);
        case RockProviderStructureIdV1::PresentedHandPose:
            return sizeof(RockProviderPresentedHandPoseV1);
        case RockProviderStructureIdV1::SemanticHandContact:
            return sizeof(RockProviderSemanticHandContactV1);
        case RockProviderStructureIdV1::PlayerColliderDescriptor:
            return sizeof(RockProviderPlayerColliderDescriptorV1);
        case RockProviderStructureIdV1::HandCollisionAvailability:
            return sizeof(RockProviderHandCollisionAvailabilityV1);
        case RockProviderStructureIdV1::InputSuppressionState:
            return sizeof(RockProviderHandInputSuppressionStateV1);
        case RockProviderStructureIdV1::OffhandReservationRequest:
            return sizeof(RockProviderOffhandReservationRequestV1);
        case RockProviderStructureIdV1::OffhandReservationState:
            return sizeof(RockProviderOffhandReservationStateV1);
        case RockProviderStructureIdV1::ForceGrabRequest:
            return sizeof(RockProviderForceGrabRequestV1);
        case RockProviderStructureIdV1::ForceReleaseRequest:
            return sizeof(RockProviderForceReleaseRequestV1);
        case RockProviderStructureIdV1::ThrownDropRequest:
            return sizeof(RockProviderThrownDropRequestV1);
        case RockProviderStructureIdV1::InteractionCommandResult:
            return sizeof(RockProviderInteractionCommandResultV1);
        case RockProviderStructureIdV1::HandInputSuppressionRequest:
            return sizeof(RockProviderHandInputSuppressionRequestV1);
        case RockProviderStructureIdV1::RawWandButtonState:
            return sizeof(RockProviderRawWandButtonStateV1);
        case RockProviderStructureIdV1::WeaponPartTarget:
            return sizeof(RockProviderWeaponPartTargetV1);
        case RockProviderStructureIdV1::Transform:
            return sizeof(RockProviderTransform);
        case RockProviderStructureIdV1::WeaponPartDriveTarget:
            return sizeof(RockProviderWeaponPartDriveTargetV1);
        case RockProviderStructureIdV1::WeaponPartGripState:
            return sizeof(RockProviderWeaponPartGripStateV1);
        case RockProviderStructureIdV1::WeaponContactQuery:
            return sizeof(RockProviderWeaponContactQuery);
        case RockProviderStructureIdV1::WeaponContactResult:
            return sizeof(RockProviderWeaponContactResult);
        case RockProviderStructureIdV1::WeaponClassification:
            return sizeof(RockProviderWeaponClassificationV1);
        case RockProviderStructureIdV1::Point3:
            return sizeof(RockProviderPoint3);
        case RockProviderStructureIdV1::Bounds3:
            return sizeof(RockProviderBounds3);
        case RockProviderStructureIdV1::WeaponEmitter:
            return sizeof(RockProviderWeaponEmitterV1);
        case RockProviderStructureIdV1::NativeAnimationAuthorityRequest:
            return sizeof(RockProviderNativeAnimationAuthorityRequestV1);
        case RockProviderStructureIdV1::NativeAnimationAuthorityState:
            return sizeof(RockProviderNativeAnimationAuthorityStateV1);
        case RockProviderStructureIdV1::AnimationPhaseContext:
            return sizeof(RockProviderAnimationPhaseContextV1);
        case RockProviderStructureIdV1::EquippedWeaponGripState:
            return sizeof(RockProviderEquippedWeaponGripStateV1);
        case RockProviderStructureIdV1::EquippedWeaponHandlingRequest:
            return sizeof(RockProviderEquippedWeaponHandlingRequestV1);
        case RockProviderStructureIdV1::EquippedWeaponHandlingState:
            return sizeof(RockProviderEquippedWeaponHandlingStateV1);
        case RockProviderStructureIdV1::HandVisualAuthorityRequest:
            return sizeof(RockProviderHandVisualAuthorityRequestV1);
        case RockProviderStructureIdV1::NativeAnimationRuntimePublication:
            return sizeof(RockProviderNativeAnimationRuntimePublicationV1);
        case RockProviderStructureIdV1::DebugOverlayLine:
            return sizeof(RockProviderDebugOverlayLineV1);
        case RockProviderStructureIdV1::DebugOverlayText:
            return sizeof(RockProviderDebugOverlayTextV1);
        case RockProviderStructureIdV1::DebugOverlayPublication:
            return sizeof(RockProviderDebugOverlayPublicationV1);
        case RockProviderStructureIdV1::WeaponEvidenceDetail:
            return sizeof(RockProviderWeaponEvidenceDetailV1);
        case RockProviderStructureIdV1::BodyContact:
            return sizeof(RockProviderBodyContactV1);
        case RockProviderStructureIdV1::ApiFunctionTable:
            return sizeof(RockProviderApi);
        case RockProviderStructureIdV1::TouchGrabTarget:
            return sizeof(RockProviderTouchGrabTargetV1);
        case RockProviderStructureIdV1::TouchGrabState:
            return sizeof(RockProviderTouchGrabStateV1);
        case RockProviderStructureIdV1::EquippedWeaponHandRequest:
            return sizeof(RockProviderEquippedWeaponHandRequestV1);
        case RockProviderStructureIdV1::WorldRaycastRequest:
            return sizeof(RockProviderWorldRaycastRequestV1);
        case RockProviderStructureIdV1::WorldRaycastResult:
            return sizeof(RockProviderWorldRaycastResultV1);
        default:
            return 0;
        }
    }

    RockProviderResultV1 validateReadCapability(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability)
    {
        std::scoped_lock lock(s_consumerMutex);
        return validateRegisteredOwnerCapabilityLocked(ownerToken, capability);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInteractionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInteractionStateV1* outState)
    {
        if (!outState || ownerToken == 0 ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderHandInteractionStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::HandInteractionState);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return RockProviderResultV1::NotReady;
        }
        *outState = s_lastHandInteractionStates[
            hand == RockProviderHand::Left ? 1u : 0u];
        return RockProviderResultV1::Ok;
    }

    bool generationGuardsStale(
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration)
    {
        return validateGenerationGuards(
                   worldGeneration,
                   skeletonGeneration,
                   providerGeneration) != RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyProviderEventsSinceV1(
        const std::uint64_t ownerToken,
        const std::uint64_t afterSequence,
        RockProviderEventV1* outEvents,
        const std::uint32_t maxEvents,
        RockProviderEventStreamStateV1* outStreamState)
    {
        if (ownerToken == 0 || !outStreamState ||
            outStreamState->size < sizeof(RockProviderEventStreamStateV1) ||
            (maxEvents != 0 && !outEvents)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::ProviderEvents);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        std::scoped_lock lock(s_providerEventMutex);
        *outStreamState = {};
        if (s_providerEventCount != 0) {
            outStreamState->oldestRetainedSequence =
                providerEventAtLocked(0).sequence;
        }
        outStreamState->latestEmittedSequence =
            s_nextProviderEventSequence > 1 ?
                s_nextProviderEventSequence - 1 :
                0;
        outStreamState->overwrittenCount = s_overwrittenProviderEventCount;
        if (afterSequence != 0 &&
            outStreamState->oldestRetainedSequence > 1 &&
            afterSequence <
                outStreamState->oldestRetainedSequence - 1) {
            outStreamState->flags |= static_cast<std::uint32_t>(
                RockProviderEventStreamFlagV1::GapBeforeFirstCopied);
        }
        if (s_overwrittenProviderEventCount != 0) {
            outStreamState->flags |= static_cast<std::uint32_t>(
                RockProviderEventStreamFlagV1::RingOverwroteRecords);
        }

        std::uint32_t copied = 0;
        for (std::uint32_t i = 0;
             i < s_providerEventCount && copied < maxEvents;
             ++i) {
            const auto& event = providerEventAtLocked(i);
            if (event.sequence <= afterSequence ||
                (event.ownerToken != 0 && event.ownerToken != ownerToken)) {
                continue;
            }
            outEvents[copied++] = event;
        }
        outStreamState->copiedCount = copied;
        if (copied != 0) {
            outStreamState->firstCopiedSequence = outEvents[0].sequence;
            outStreamState->lastCopiedSequence =
                outEvents[copied - 1].sequence;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponStateV1* outState)
    {
        if (ownerToken == 0 || !outState) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderEquippedWeaponStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return RockProviderResultV1::NotReady;
        }
        *outState = s_lastEquippedWeaponState;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWeaponPartTargetResolutionV1(
        const std::uint64_t ownerToken,
        const RockProviderWeaponPartResolutionQueryV1* query,
        RockProviderWeaponPartResolutionResultV1* outResolution)
    {
        if (ownerToken == 0 || !query || !outResolution) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (query->size < sizeof(RockProviderWeaponPartResolutionQueryV1) ||
            outResolution->size < sizeof(RockProviderWeaponPartResolutionResultV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        RockProviderWeaponPartTargetQueryV1 internal{};
        internal.weaponGenerationKey = query->weaponGenerationKey;
        internal.bodyId = query->bodyId;
        internal.partKind = query->partKind;
        internal.reloadRole = query->reloadRole;
        internal.supportRole = query->supportRole;
        internal.socketRole = query->socketRole;
        internal.actionRole = query->actionRole;
        internal.sourceRoot = query->sourceRoot;
        std::memcpy(internal.sourceName, query->sourceName,
            sizeof(internal.sourceName));
        internal.sourceName[sizeof(internal.sourceName) - 1] = '\0';
        RockProviderWeaponPartTargetResolutionV1 resolution{};
        (void)resolveWeaponPartTargetV1(internal, resolution);

        *outResolution = {};
        outResolution->whitelistActive = resolution.whitelistActive;
        outResolution->matched = resolution.matched;
        outResolution->grabMode = resolution.grabMode;
        outResolution->groupId = resolution.groupId;
        outResolution->priority = resolution.priority;
        outResolution->winningOwnerToken = resolution.ownerToken;
        outResolution->weaponGenerationKey = query->weaponGenerationKey;
        outResolution->frameIndex = currentProviderFrameIndex();
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartPoseSnapshotV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartPoseV1* outParts,
        const std::uint32_t maxParts,
        std::uint32_t* outPartCount)
    {
        if (ownerToken == 0 || !outPartCount ||
            (maxParts != 0 && !outParts)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outPartCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outPartCount = pi->copyProviderWeaponPartPosesV1(
            outParts,
            maxParts);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartDriveApplicationResultsV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartDriveApplicationResultV1* outResults,
        const std::uint32_t maxResults,
        std::uint32_t* outResultCount)
    {
        if (ownerToken == 0 || !outResultCount ||
            (maxResults != 0 && !outResults)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outResultCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outResultCount = pi->copyProviderWeaponPartDriveResultsV1(
            ownerToken,
            outResults,
            maxResults);
        return RockProviderResultV1::Ok;
    }

    template <class Output, class Query>
    RockProviderResultV1 queryPhysicsInteractionValueV1(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability,
        Output* output,
        Query&& query,
        const bool requireAnimationThread = false)
    {
        if (ownerToken == 0 || !output) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (output->size < sizeof(Output)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(ownerToken, capability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (requireAnimationThread && !onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return query(*pi, *output) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetScopeSightStateV1(
        const std::uint64_t ownerToken,
        RockProviderScopeSightStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::ScopeSightState,
            outState,
            [](PhysicsInteraction& pi, RockProviderScopeSightStateV1& state) {
                return pi.queryProviderScopeSightStateV1(state);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetWeaponCompositionStateV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition,
            outState,
            [](PhysicsInteraction& pi, RockProviderWeaponCompositionStateV1& state) {
                return pi.queryProviderWeaponCompositionStateV1(state);
            });
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponCompositionEntriesV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionEntryV1* outEntries,
        const std::uint32_t maxEntries,
        std::uint32_t* outEntryCount)
    {
        if (ownerToken == 0 || !outEntryCount ||
            (maxEntries != 0 && !outEntries)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outEntryCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outEntryCount = pi->copyProviderWeaponCompositionEntriesV1(
            outEntries,
            maxEntries);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetSelectedAuthoredGripPoseV1(
        const std::uint64_t ownerToken,
        RockProviderAuthoredGripPoseV1* outPose)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [](PhysicsInteraction& pi, RockProviderAuthoredGripPoseV1& pose) {
                return pi.queryProviderSelectedAuthoredGripPoseV1(pose);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPresentedHandPoseV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderPresentedHandPoseV1* outPose)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        const auto result = queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [hand](PhysicsInteraction& pi, RockProviderPresentedHandPoseV1& pose) {
                return pi.queryProviderPresentedHandPoseV1(hand, pose);
            },
            true);
        if (result == RockProviderResultV1::Ok) {
            outPose->frameIndex = currentProviderFrameIndex();
            outPose->presentationSequence =
                s_activeAnimationPhaseFrameIndex.load(std::memory_order_acquire);
        }
        return result;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopySemanticHandContactsV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        const std::uint32_t maxFramesSinceContact,
        RockProviderSemanticHandContactV1* outContacts,
        const std::uint32_t maxContacts,
        std::uint32_t* outContactCount)
    {
        if (ownerToken == 0 || !outContactCount ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left) ||
            (maxContacts != 0 && !outContacts)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outContactCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::SemanticHandContacts);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outContactCount = pi->copyProviderSemanticHandContactsV1(
            hand,
            maxFramesSinceContact,
            outContacts,
            maxContacts);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyPlayerColliderDescriptorsV1(
        const std::uint64_t ownerToken,
        RockProviderPlayerColliderDescriptorV1* outDescriptors,
        const std::uint32_t maxDescriptors,
        std::uint32_t* outDescriptorCount)
    {
        if (ownerToken == 0 || !outDescriptorCount ||
            (maxDescriptors != 0 && !outDescriptors)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outDescriptorCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outDescriptorCount = pi->copyProviderPlayerColliderDescriptorsV1(
            outDescriptors,
            maxDescriptors);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandCollisionAvailabilityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandCollisionAvailabilityV1* outState)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors,
            outState,
            [hand](PhysicsInteraction& pi, RockProviderHandCollisionAvailabilityV1& state) {
                return pi.queryProviderHandCollisionAvailabilityV1(hand, state);
            },
            true);
    }

    bool hasInteractionTargetIdentity(std::uint32_t targetFormId, std::uint32_t targetBodyId)
    {
        return targetFormId != 0 || targetBodyId != kProviderInvalidBodyId;
    }

    bool isFiniteVector3(const float values[3])
    {
        return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
    }

    RockProviderResultV1 validateInteractionCommandProviderReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 enqueueInteractionCommand(QueuedInteractionCommandV1 command, std::uint64_t* outCommandId)
    {
        if (!outCommandId) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        const auto providerReadyResult = validateInteractionCommandProviderReady();
        if (providerReadyResult != RockProviderResultV1::Ok) {
            return providerReadyResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(command.ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
            s_forceGrabReservations.isReserved(command.forceGrab.hand)) {
            return RockProviderResultV1::HandBusy;
        }

        for (auto& slot : s_interactionCommands) {
            if (!slot.active) {
                command.commandId = nextInteractionCommandId();
                if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
                    !s_forceGrabReservations.reserve(command.forceGrab.hand, command.ownerToken, command.commandId)) {
                    return RockProviderResultV1::HandBusy;
                }
                slot = InteractionCommandSlot{
                    .active = true,
                    .command = command,
                };
                *outCommandId = command.commandId;
                storeInteractionResultLocked(makeCommandResult(command, RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::RequestQueued;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceGrabV1(
        std::uint64_t ownerToken,
        const RockProviderForceGrabRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceGrabRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->targetFormId == 0 || (request->flags & ~kImplementedForceGrabFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!std::isfinite(request->maxDistanceGame)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request->preferredGrabPointGame)) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceGrab;
        command.forceGrab = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceReleaseV1(
        std::uint64_t ownerToken,
        const RockProviderForceReleaseRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceReleaseRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedForceReleaseFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceRelease;
        command.forceRelease = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestThrownDropV1(
        std::uint64_t ownerToken,
        const RockProviderThrownDropRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderThrownDropRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedThrownDropFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ThrownDrop;
        command.thrownDrop = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetInteractionCommandResultV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderInteractionCommandResultV1* outResult)
    {
        if (!outResult || ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto requestedSize = outResult->size;
        if (requestedSize <
            ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto copyResult = [outResult, requestedSize](
                                    RockProviderInteractionCommandResultV1 result) {
            const auto copySize = (std::min<std::size_t>)(
                requestedSize,
                sizeof(result));
            result.size = static_cast<std::uint32_t>(copySize);
            std::memcpy(outResult, &result, copySize);
        };

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                copyResult(slot.result);
                return RockProviderResultV1::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                copyResult(makeCommandResult(
                    slot.command,
                    RockProviderInteractionCommandStateV1::Queued,
                    RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCancelInteractionCommandV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_interactionCommands) {
            if (!slot.active || slot.command.ownerToken != ownerToken ||
                slot.command.commandId != commandId) {
                continue;
            }
            completeInteractionCommandLocked(
                slot.command,
                RockProviderInteractionCommandStateV1::Cancelled,
                RockProviderInteractionFailureV1::None);
            slot = {};
            return RockProviderResultV1::Ok;
        }
        for (const auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId) {
                continue;
            }
            return interaction_command_policy::isTerminal(slot.result.state) ?
                RockProviderResultV1::RequestNotFound :
                RockProviderResultV1::AlreadyCommitted;
        }
        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandInputSuppressionV1(
        std::uint64_t ownerToken,
        const RockProviderHandInputSuppressionRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderHandInputSuppressionRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags == 0 || (request->flags & ~kImplementedHandInputSuppressionFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1);
        const auto frameIndex = currentProviderFrameIndex();
        const auto expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(frameIndex, leaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.ownerToken == ownerToken && slot.hand == request->hand) {
                slot.flags = request->flags;
                slot.expiresAfterFrame = expiresAfterFrame;
                slot.worldGeneration = request->worldGeneration;
                slot.skeletonGeneration = request->skeletonGeneration;
                slot.providerGeneration = request->providerGeneration;
                slot.lastInvalidationReason =
                    RockProviderSuppressionInvalidationReasonV1::None;
                slot.lastInvalidatedFrame = 0;
                return RockProviderResultV1::Ok;
            }
        }

        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                slot = HandInputSuppressionSlot{
                    .active = true,
                    .ownerToken = ownerToken,
                    .hand = request->hand,
                    .flags = request->flags,
                    .expiresAfterFrame = expiresAfterFrame,
                    .worldGeneration = request->worldGeneration,
                    .skeletonGeneration = request->skeletonGeneration,
                    .providerGeneration = request->providerGeneration,
                    .lastInvalidationReason =
                        RockProviderSuppressionInvalidationReasonV1::None,
                    .lastInvalidatedFrame = 0,
                };
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandInputSuppressionV1(
        std::uint64_t ownerToken,
        RockProviderHand hand)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (hand != RockProviderHand::None && hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        clearHandInputSuppressionsForOwnerLocked(ownerToken, hand);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInputSuppressionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInputSuppressionStateV1* outState)
    {
        if (ownerToken == 0 || !outState ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderHandInputSuppressionStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::InputObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredHandInputSuppressionsLocked(frameIndex);

        *outState = {};
        outState->frameIndex = frameIndex;
        outState->hand = hand;
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.hand != hand) {
                continue;
            }
            if (slot.active) {
                outState->effectiveFlags |= slot.flags;
            }
            if (slot.ownerToken != ownerToken) {
                continue;
            }
            if (slot.active) {
                outState->callerFlags = slot.flags;
                outState->callerLeaseActive = 1;
                outState->callerExpiresAfterFrame = slot.expiresAfterFrame;
                outState->callerRemainingFrames =
                    provider_lease_policy::remainingFrames(
                        frameIndex,
                        slot.expiresAfterFrame);
            } else if (outState->lastInvalidationReason ==
                       RockProviderSuppressionInvalidationReasonV1::None) {
                outState->lastInvalidationReason =
                    slot.lastInvalidationReason;
            }
        }
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration = s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration = s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetNativeAnimationAuthorityV1(
        std::uint64_t ownerToken,
        const RockProviderNativeAnimationAuthorityRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderNativeAnimationAuthorityRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr auto implementedFlags = static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityFlagV1::ReloadPose);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        const auto frameIndex = currentProviderFrameIndex();
        const auto boundedLeaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1);
        const auto expiresAtFrame = provider_lease_policy::exclusiveExpiryFrame(
            frameIndex,
            boundedLeaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredNativeAnimationAuthorityLocked(frameIndex);
        NativeAnimationAuthoritySlot* available = nullptr;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (slot.active && slot.ownerToken == ownerToken) {
                available = &slot;
                break;
            }
            if (!slot.active && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        *available = NativeAnimationAuthoritySlot{
            .active = true,
            .ownerToken = ownerToken,
            .flags = request->flags,
            .expiresAtFrame = expiresAtFrame,
            .worldGeneration = request->worldGeneration,
            .skeletonGeneration = request->skeletonGeneration,
            .providerGeneration = request->providerGeneration,
        };
        publishNativeAnimationAuthorityAggregateLocked();
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationAuthorityV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetNativeAnimationAuthorityStateV1(
        RockProviderNativeAnimationAuthorityStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderNativeAnimationAuthorityStateV1)) {
            return false;
        }

        RockProviderNativeAnimationRuntimePublicationV1 publication{};
        bool hasRuntimeProvider = false;
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                currentProviderFrameIndex());
            hasRuntimeProvider = s_hasNativeAnimationRuntimePublication;
            if (hasRuntimeProvider) {
                publication = s_nativeAnimationRuntimePublication;
            }
        }
        *outState = {};
        outState->size = sizeof(RockProviderNativeAnimationAuthorityStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->activeFlags = s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
        outState->statusFlags = hasRuntimeProvider ?
            publication.statusFlags |
                static_cast<std::uint32_t>(
                    RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeProviderAvailable) :
            0;
        outState->activeOwnerCount = s_nativeAnimationAuthorityOwnerCount.load(std::memory_order_acquire);
        outState->capturedTransformCount = hasRuntimeProvider ?
            publication.capturedTransformCount : 0;
        outState->captureSequence = hasRuntimeProvider ?
            publication.captureSequence : 0;
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        RockProviderAnimationPhaseCallbackV1 callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (!callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        AnimationPhaseCallbackSlot* available = nullptr;
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            if (!slot.callback && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        auto token = s_nextAnimationPhaseCallbackToken.fetch_add(
            1,
            std::memory_order_acq_rel);
        if (token == 0) {
            token = s_nextAnimationPhaseCallbackToken.fetch_add(
                1,
                std::memory_order_acq_rel);
        }
        *available = AnimationPhaseCallbackSlot{
            .token = token,
            .ownerToken = ownerToken,
            .callback = callback,
            .userData = userData,
        };
        *outCallbackToken = token;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.token == callbackToken && slot.ownerToken == ownerToken) {
                slot = {};
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::TargetUnavailable;
    }

    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponGripStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponGripStateV1* outState)
    {
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponGripStateV1)) {
            return false;
        }
        if (!onAnimationOwnerThread()) {
            return false;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            if (!consumerHasCapabilityLocked(
                    ownerToken,
                    RockProviderConsumerCapabilityV1::EquippedWeaponGripState)) {
                return false;
            }
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized() &&
               pi->queryProviderEquippedWeaponGripStateV1(*outState);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHandVisualAuthorityRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        if (request->size != sizeof(RockProviderHandVisualAuthorityRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right &&
            request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }

        constexpr std::uint32_t worldFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        constexpr std::uint32_t fingerFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::FingerLocalTransforms);
        constexpr std::uint32_t implementedFlags = worldFlag | fingerFlag;
        if (request->flags == 0 || (request->flags & ~implementedFlags) != 0 ||
            request->priority < -10000 || request->priority > 10000 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & worldFlag) != 0 &&
            !finiteProviderTransform(request->worldTransform)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & fingerFlag) != 0) {
            if (request->fingerLocalTransformMask == 0 ||
                (request->fingerLocalTransformMask &
                    ~ROCK_PROVIDER_ALL_FINGER_LOCAL_TRANSFORMS_V1) != 0) {
                return RockProviderResultV1::InvalidArgument;
            }
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((request->fingerLocalTransformMask & bit) != 0 &&
                    !finiteProviderTransform(request->fingerLocalTransforms[index])) {
                    return RockProviderResultV1::InvalidArgument;
                }
            }
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return RockProviderResultV1::NotReady;
        }

        std::scoped_lock lock(s_handVisualAuthorityMutex);
        auto* slot = findHandVisualAuthoritySlotLocked(ownerToken, request->hand);
        if (!slot) {
            return RockProviderResultV1::CapacityFull;
        }
        if (slot->ownerToken != 0 && slot->publishedFlags != request->flags &&
            !clearHandVisualAuthoritySlotLocked(*slot, false)) {
            return RockProviderResultV1::TargetUnavailable;
        }
        if (slot->ownerToken == 0) {
            slot->ownerToken = ownerToken;
            slot->hand = request->hand;
            const int length = std::snprintf(
                slot->tag,
                sizeof(slot->tag),
                "ROCK_API_%016llX",
                static_cast<unsigned long long>(ownerToken));
            if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(slot->tag)) {
                *slot = {};
                return RockProviderResultV1::InvalidArgument;
            }
        }

        const auto hand = toVisualHand(request->hand);
        bool published = true;
        if ((request->flags & fingerFlag) != 0) {
            frik_visual_authority::FingerLocalTransformOverride fingerLocals{};
            fingerLocals.enabledMask = request->fingerLocalTransformMask;
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((fingerLocals.enabledMask & bit) != 0) {
                    fingerLocals.localTransforms[index] =
                        toNiTransform(request->fingerLocalTransforms[index]);
                }
            }
            published = frik_visual_authority::setHandPoseCustomWithPriority(
                            slot->tag,
                            hand,
                            frik_visual_authority::HandPoseData{},
                            request->priority) &&
                        frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(
                            slot->tag,
                            hand,
                            &fingerLocals,
                            request->priority);
        }
        if (published && (request->flags & worldFlag) != 0) {
            published = frik_visual_authority::applyExternalHandWorldTransform(
                slot->tag,
                hand,
                toNiTransform(request->worldTransform),
                request->priority);
        }
        if (!published) {
            slot->publishedFlags = request->flags;
            (void)clearHandVisualAuthoritySlotLocked(*slot, true);
            return RockProviderResultV1::TargetUnavailable;
        }

        slot->publishedFlags = request->flags;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1);
        slot->expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
            currentProviderFrameIndex(),
            leaseFrames);
        slot->worldGeneration = request->worldGeneration;
        slot->skeletonGeneration = request->skeletonGeneration;
        slot->providerGeneration = request->providerGeneration;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand)
    {
        if (ownerToken == 0 ||
            (hand != RockProviderHand::None &&
                hand != RockProviderHand::Right &&
                hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }

        return clearHandVisualAuthorityForOwner(ownerToken, hand, false) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken,
        const RockProviderNativeAnimationRuntimePublicationV1* publication)
    {
        if (!publication || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (publication->size !=
            sizeof(RockProviderNativeAnimationRuntimePublicationV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (publication->version == 0 ||
            publication->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr std::uint32_t implementedStatusFlags =
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstalled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeEnabled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureValid) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::LocalReloadTestLeaseActive) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstallFailed) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::ThreadMismatch) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureFault);
        if ((publication->statusFlags & ~implementedStatusFlags) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (publication->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication->worldGeneration,
            publication->skeletonGeneration,
            publication->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        s_nativeAnimationRuntimeProviderOwner = ownerToken;
        s_nativeAnimationRuntimePublication = *publication;
        s_nativeAnimationRuntimePublication.size =
            sizeof(RockProviderNativeAnimationRuntimePublicationV1);
        s_nativeAnimationRuntimePublication.version = ROCK_PROVIDER_API_VERSION;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            publication->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1);
        s_nativeAnimationRuntimePublication.leaseFrames = leaseFrames;
        s_nativeAnimationRuntimeExpiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                leaseFrames);
        s_hasNativeAnimationRuntimePublication = true;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        s_nativeAnimationRuntimeProviderOwner = 0;
        s_nativeAnimationRuntimePublication = {};
        s_nativeAnimationRuntimeExpiresAfterFrame = 0;
        s_hasNativeAnimationRuntimePublication = false;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiSetTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderTouchGrabTargetV1* targets,
        const std::uint32_t targetCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            targetCount > ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 ||
            (targetCount != 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        for (std::uint32_t index = 0; index < targetCount; ++index) {
            if (targets[index].size !=
                sizeof(RockProviderTouchGrabTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (targets[index].version == 0 ||
                targets[index].version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            const auto generationResult = validateGenerationGuards(
                targets[index].worldGeneration,
                targets[index].skeletonGeneration,
                targets[index].providerGeneration);
            if (generationResult != RockProviderResultV1::Ok) {
                return generationResult;
            }
        }

        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_touchGrabTargets.setScope(
            ownerToken,
            scopeToken,
            targets,
            targetCount,
            currentProviderFrameIndex())) {
        case TouchGrabRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case TouchGrabRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case TouchGrabRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case TouchGrabRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiClearTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiCopyTouchGrabStatesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        RockProviderTouchGrabStateV1* outStates,
        const std::uint32_t maxStates,
        std::uint32_t* outStateCount)
    {
        if (ownerToken == 0 || scopeToken == 0 || !outStateCount ||
            (maxStates != 0 && !outStates)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        *outStateCount = s_touchGrabTargets.copyStates(
            ownerToken,
            scopeToken,
            outStates,
            maxStates,
            currentProviderFrameIndex());
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiRequestTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        if (ownerToken == 0 || scopeToken == 0 || targetId == 0 ||
            targetGeneration == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.requestYield(
                   ownerToken,
                   scopeToken,
                   targetId,
                   targetGeneration,
                   currentProviderFrameIndex()) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    [[nodiscard]] bool equippedWeaponHandlingRequestValuesValid(
        const RockProviderEquippedWeaponHandlingRequestV1& request)
    {
        const auto inRange = [](const float value, const float minimum, const float maximum) {
            return std::isfinite(value) && value >= minimum && value <= maximum;
        };
        return inRange(request.gripZoneEquipRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.gripZoneEquipSettleSeconds, 0.0f, 5.0f) &&
               inRange(request.firingGripReattachRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.gripZoneHoverHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripProximitySupportRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.weaponGripHapticDurationSeconds, 0.01f, 0.50f) &&
               inRange(request.firingGripAttachHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripDetachHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.supportGripHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripPromotionRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.leftFiringAimYawDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimPitchDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[0], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[1], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[2], -15.0f, 15.0f) &&
               inRange(request.equipVisualBridgeTimeoutSeconds, 0.25f, 5.0f) &&
               inRange(request.equipVisualBridgeBlendSeconds, 0.0f, 1.0f);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderEquippedWeaponHandlingRequestV1* request)
    {
        if (ownerToken == 0 || !request) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderEquippedWeaponHandlingRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr std::uint32_t implementedFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneHoverHaptics) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquipVisualBridge);
        const auto ownershipFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
        const auto ownershipDependentFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip);
        const auto primaryDetachFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach);
        const auto shoulderStashFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0 ||
            ((request->flags & ownershipDependentFlags) != 0 &&
                (request->flags & ownershipFlag) == 0) ||
            ((request->flags & shoulderStashFlag) != 0 &&
                (request->flags & primaryDetachFlag) == 0) ||
            !equippedWeaponHandlingRequestValuesValid(*request)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1);
        s_equippedWeaponHandlingAuthority.active = true;
        s_equippedWeaponHandlingAuthority.ownerToken = ownerToken;
        s_equippedWeaponHandlingAuthority.expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames);
        s_equippedWeaponHandlingAuthority.request = *request;
        s_equippedWeaponHandlingAuthority.request.size =
            sizeof(RockProviderEquippedWeaponHandlingRequestV1);
        s_equippedWeaponHandlingAuthority.request.version =
            ROCK_PROVIDER_API_VERSION;
        s_equippedWeaponHandlingAuthority.request.leaseFrames = leaseFrames;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponHandlingStateV1(
        RockProviderEquippedWeaponHandlingStateV1* outState)
    {
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponHandlingStateV1)) {
            return false;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() ||
            !pi->queryProviderEquippedWeaponHandlingStateV1(*outState)) {
            return false;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active) {
            outState->authorityFlags =
                s_equippedWeaponHandlingAuthority.request.flags;
            outState->ownerToken =
                s_equippedWeaponHandlingAuthority.ownerToken;
            outState->expiresAfterFrame =
                s_equippedWeaponHandlingAuthority.expiresAfterFrame;
            outState->runtimeFlags |= static_cast<std::uint32_t>(
                RockProviderEquippedWeaponHandlingRuntimeFlagV1::AuthorityActive);
        }
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiRequestEquippedWeaponHandV1(
        const std::uint64_t ownerToken,
        const RockProviderEquippedWeaponHandRequestV1* request)
    {
        if (ownerToken == 0 || !request) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        if (request->size !=
            sizeof(RockProviderEquippedWeaponHandRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 ||
            request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right &&
            request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        {
            const auto frameIndex = currentProviderFrameIndex();
            std::scoped_lock lock(
                s_consumerMutex,
                s_equippedWeaponHandlingAuthorityMutex);
            const auto ownerResult =
                validateRegisteredOwnerCapabilityLocked(
                    ownerToken,
                    RockProviderConsumerCapabilityV1::
                        EquippedWeaponHandlingAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
            pruneExpiredEquippedWeaponHandlingAuthorityLocked(
                frameIndex);
            if (!s_equippedWeaponHandlingAuthority.active) {
                return RockProviderResultV1::PermissionDenied;
            }
            if (s_equippedWeaponHandlingAuthority.ownerToken !=
                ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }

            std::uint32_t requiredFlags =
                static_cast<std::uint32_t>(
                    RockProviderEquippedWeaponHandlingFlagV1::
                        FiringGripOwnership);
            if (request->hand == RockProviderHand::Left) {
                requiredFlags |= static_cast<std::uint32_t>(
                    RockProviderEquippedWeaponHandlingFlagV1::
                        AmbidextrousHandoff);
            }
            if ((s_equippedWeaponHandlingAuthority.request.flags &
                    requiredFlags) != requiredFlags) {
                return RockProviderResultV1::PermissionDenied;
            }
        }

        auto* pi =
            s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized() ?
            pi->requestProviderEquippedWeaponHandV1(
                ownerToken,
                *request) :
            RockProviderResultV1::NotReady;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWorldRaycastV1(
        const std::uint64_t ownerToken,
        const RockProviderWorldRaycastRequestV1* request,
        RockProviderWorldRaycastResultV1* outResult)
    {
        if (ownerToken == 0 || !request || !outResult) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderWorldRaycastRequestV1) ||
            outResult->size != sizeof(RockProviderWorldRaycastResultV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version != ROCK_PROVIDER_API_VERSION ||
            outResult->version != ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!finiteProviderPoint(request->startGame) ||
            !finiteProviderPoint(request->directionGame) ||
            !std::isfinite(request->maxDistanceGame) ||
            request->maxDistanceGame <= 0.0f ||
            request->maxDistanceGame >
                ROCK_PROVIDER_MAX_WORLD_RAYCAST_DISTANCE_GAME_V1) {
            return RockProviderResultV1::InvalidArgument;
        }
        const float directionLengthSquared =
            request->directionGame.x * request->directionGame.x +
            request->directionGame.y * request->directionGame.y +
            request->directionGame.z * request->directionGame.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }

        const auto frameIndex = currentProviderFrameIndex();
        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            if (!hasConsumerCapabilityV1(
                    slot->grantedCapabilities,
                    RockProviderConsumerCapabilityV1::WorldRaycasts)) {
                return RockProviderResultV1::PermissionDenied;
            }
            if (slot->worldRaycastFrameIndex != frameIndex) {
                slot->worldRaycastFrameIndex = frameIndex;
                slot->worldRaycastCount = 0;
            }
            if (slot->worldRaycastCount >=
                ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1) {
                return RockProviderResultV1::CapacityFull;
            }
            ++slot->worldRaycastCount;
        }

        *outResult = {};
        outResult->frameIndex = frameIndex;
        if (!pi->queryProviderWorldRaycastV1(*request, *outResult)) {
            return RockProviderResultV1::WorldNotReady;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishDebugOverlayV1(
        const std::uint64_t ownerToken,
        const RockProviderDebugOverlayPublicationV1* publication)
    {
        if (ownerToken == 0 || !publication) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        if (publication->size !=
            sizeof(RockProviderDebugOverlayPublicationV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (publication->version == 0 ||
            publication->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (publication->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication->worldGeneration,
            publication->skeletonGeneration,
            publication->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        return provider_debug_overlay::publish(
            ownerToken,
            *publication,
            currentProviderFrameIndex());
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearDebugOverlayV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        provider_debug_overlay::clear(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1) {
            return RockProviderResultV1::CapacityFull;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            if (!isValidWeaponPartGrabMode(target.grabMode) ||
                !hasValidWeaponPartMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName) ||
                !hasValidWeaponPartTargetSemantics(target)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }

        if (targetCount > availableWeaponPartTargetSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }

        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            bool stored = false;
            for (auto& slot : s_weaponPartTargets) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartTargetsForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartTargetsV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartDriveTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartDriveTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1) {
            return RockProviderResultV1::CapacityFull;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartDriveTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            if (!isValidWeaponPartDriveSpace(target.driveSpace) ||
                target.leaseFrames == 0 ||
                !isFiniteProviderTransform(target.targetTransform) ||
                !hasConcreteWeaponPartDriveMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }

        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        if (targetCount > availableWeaponPartDriveSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
                targets[i].leaseFrames,
                ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1);
            const auto expiresAfterFrame =
                provider_lease_policy::exclusiveExpiryFrame(
                    frameIndex,
                    leaseFrames);
            bool stored = false;
            for (auto& slot : s_weaponPartDrives) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.expiresAfterFrame = expiresAfterFrame;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartDrivesForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiQueryWeaponContactAtPoint(
        const RockProviderWeaponContactQuery* query,
        RockProviderWeaponContactResult* outResult)
    {
        if (!query || !outResult ||
            query->size != sizeof(RockProviderWeaponContactQuery) ||
            outResult->size != sizeof(RockProviderWeaponContactResult)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderWeaponContactAtPoint(*query, *outResult);
    }

    bool ROCK_PROVIDER_CALL apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult)
    {
        if (!outResult || outResult->size != sizeof(RockProviderWeaponClassificationV1)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderEquippedWeaponClassificationV1(*outResult);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailCountV1()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailCountV1();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailsV1(
        RockProviderWeaponEvidenceDetailV1* outDetails,
        std::uint32_t maxDetails)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailsV1(outDetails, maxDetails);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailPointCountV1(bodyId);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailPointsV1(
        std::uint32_t bodyId,
        RockProviderPoint3* outPoints,
        std::uint32_t maxPoints)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailPointsV1(bodyId, outPoints, maxPoints);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetBodyContactSnapshotV1(
        RockProviderBodyContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderBodyContacts(outContacts, maxContacts);
    }

    bool ROCK_PROVIDER_CALL apiRegisterExternalBodiesV1(
        std::uint64_t ownerToken,
        const RockProviderExternalBodyRegistration* bodies,
        std::uint32_t bodyCount)
    {
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalBodies)) {
            return false;
        }
        return s_externalBodies.registerBodies(ownerToken, bodies, bodyCount);
    }

    void ROCK_PROVIDER_CALL apiClearExternalBodies(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalBodies)) {
            return;
        }
        s_externalBodies.clearOwner(ownerToken);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderExternalBodyRegistration* bodies,
        const std::uint32_t bodyCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            (bodyCount != 0 && !bodies)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_externalBodies.registerBodiesForScopeDetailed(
            ownerToken,
            scopeToken,
            bodies,
            bodyCount)) {
        case ExternalBodyRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case ExternalBodyRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case ExternalBodyRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case ExternalBodyRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_externalBodies.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetExternalContactSnapshotForOwnerV1(
        std::uint64_t ownerToken,
        RockProviderExternalContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        if (ownerToken == 0 || !outContacts || maxContacts == 0) {
            return 0;
        }

        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalContacts)) {
            return 0;
        }
        return s_externalBodies.copyContactsForOwnerV1(ownerToken, outContacts, maxContacts);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyExternalContactsSinceV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t afterSequence,
        RockProviderExternalContactRecordV1* outContacts,
        const std::uint32_t maxContacts,
        RockProviderExternalContactStreamStateV1* outStreamState)
    {
        if (ownerToken == 0 || !outStreamState ||
            outStreamState->size < sizeof(RockProviderExternalContactStreamStateV1) ||
            (maxContacts != 0 && !outContacts)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalContacts)) {
            return RockProviderResultV1::PermissionDenied;
        }
        (void)s_externalBodies.copyContactsSinceV1(
            ownerToken,
            scopeToken,
            afterSequence,
            outContacts,
            maxContacts,
            *outStreamState);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiSetOffhandInteractionReservation(std::uint64_t ownerToken, RockProviderOffhandReservation reservation)
    {
        if (ownerToken == 0 ||
            (reservation != RockProviderOffhandReservation::Normal &&
                reservation != RockProviderOffhandReservation::ReloadReserved &&
                reservation != RockProviderOffhandReservation::ReloadPoseOverride)) {
            return false;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::OffhandReservation)) {
            return false;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (reservation == RockProviderOffhandReservation::Normal) {
            if (s_offhandReservationSlot.ownerToken != 0 &&
                s_offhandReservationSlot.ownerToken != ownerToken) {
                return false;
            }
            clearOffhandReservationLocked(
                RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
            return true;
        }
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return false;
        }
        s_offhandReservationSlot = OffhandReservationSlot{
            .ownerToken = ownerToken,
            .reservation = reservation,
            .expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1),
        };
        publishOffhandReservationLocked(s_offhandReservationSlot);
        return true;
    }

    [[nodiscard]] bool validOffhandReservationRequest(
        const RockProviderOffhandReservationRequestV1& request)
    {
        return request.size == sizeof(RockProviderOffhandReservationRequestV1) &&
               request.version != 0 &&
               request.version <= ROCK_PROVIDER_API_VERSION &&
               request.reservation != RockProviderOffhandReservation::Normal &&
               (request.reservation == RockProviderOffhandReservation::ReloadReserved ||
                   request.reservation == RockProviderOffhandReservation::ReloadPoseOverride) &&
               request.leaseFrames != 0;
    }

    RockProviderResultV1 setOffhandReservationLeaseV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request,
        const bool requireExisting)
    {
        if (ownerToken == 0 || !request) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderOffhandReservationRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!validOffhandReservationRequest(*request)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        if (requireExisting &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::TargetUnavailable;
        }
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1);
        s_offhandReservationSlot = OffhandReservationSlot{
            .ownerToken = ownerToken,
            .reservation = request->reservation,
            .expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames),
            .worldGeneration = request->worldGeneration,
            .skeletonGeneration = request->skeletonGeneration,
            .providerGeneration = request->providerGeneration,
        };
        publishOffhandReservationLocked(s_offhandReservationSlot);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiAcquireOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, false);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRenewOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiReleaseOffhandReservationV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        clearOffhandReservationLocked(
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetOffhandReservationStateV1(
        const std::uint64_t ownerToken,
        RockProviderOffhandReservationStateV1* outState)
    {
        if (ownerToken == 0 || !outState) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderOffhandReservationStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        *outState = {};
        outState->reservation = s_offhandReservationSlot.reservation;
        outState->active = s_offhandReservationSlot.ownerToken != 0 ? 1u : 0u;
        outState->ownerToken = s_offhandReservationSlot.ownerToken;
        outState->expiresAfterFrame =
            s_offhandReservationSlot.expiresAfterFrame;
        outState->remainingFrames = provider_lease_policy::remainingFrames(
            frameIndex,
            outState->expiresAfterFrame);
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration =
                    s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration =
                    s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetRawWandButtonStateV1(RockProviderHand hand, std::uint32_t buttonId, RockProviderRawWandButtonStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderRawWandButtonStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Left && hand != RockProviderHand::Right) {
            return false;
        }
        if (!rock::input_remap_policy::isValidButtonId(static_cast<int>(buttonId))) {
            return false;
        }

        // Level state only by design: ROCK consumes its press/release edge queues internally each frame, so exposing them would race consumers.
        const auto raw = rock::input_remap_runtime::peekRawButtonState(hand == RockProviderHand::Left, static_cast<int>(buttonId));
        *outState = {};
        outState->size = sizeof(RockProviderRawWandButtonStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->available = raw.available ? 1u : 0u;
        outState->held = raw.held ? 1u : 0u;
        outState->sampleSequence = raw.sampleSequence;
        outState->sampleAgeMilliseconds = raw.sampleAgeMilliseconds;
        outState->availabilityReason =
            static_cast<RockProviderInputAvailabilityReasonV1>(
                raw.availabilityReason);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiIsNativePipboyInputSuppressedV1()
    {
        return rock::input_remap_runtime::isNativePipboyInputSuppressionActive();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEmitterCountV1()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->getProviderWeaponEmitterCountV1();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEmittersV1(
        RockProviderWeaponEmitterV1* outEmitters,
        std::uint32_t maxEmitters)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->copyProviderWeaponEmittersV1(outEmitters, maxEmitters);
    }

    [[nodiscard]] constexpr std::uint64_t advanceSequence(
        const std::uint64_t sequence) noexcept
    {
        return sequence == UINT64_MAX ? UINT64_MAX : sequence + 1;
    }

    [[nodiscard]] bool sameHeldBodies(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        if (left.heldBodyCount != right.heldBodyCount) {
            return false;
        }
        for (std::uint32_t index = 0; index < left.heldBodyCount; ++index) {
            if (left.heldBodyIds[index] != right.heldBodyIds[index]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool sameHandTarget(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        return left.targetKind == right.targetKind &&
               left.targetFormId == right.targetFormId &&
               left.primaryBodyId == right.primaryBodyId &&
               sameHeldBodies(left, right);
    }

    [[nodiscard]] bool handGripActive(
        const RockProviderHandInteractionStateV1& state) noexcept
    {
        constexpr std::uint32_t gripFlags =
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FiringGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartCarry);
        return state.phase == RockProviderHandInteractionPhaseV1::Holding ||
               (state.flags & gripFlags) != 0;
    }

    [[nodiscard]] bool sameHandGrip(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        constexpr std::uint32_t gripFlags =
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FiringGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartCarry) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::LooseObject) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::LooseWeapon);
        return handGripActive(left) == handGripActive(right) &&
               (left.flags & gripFlags) == (right.flags & gripFlags) &&
               sameHandTarget(left, right);
    }

    [[nodiscard]] bool sameHandInteractionPayload(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        return left.hand == right.hand &&
               left.phase == right.phase &&
               left.flags == right.flags &&
               sameHandTarget(left, right) &&
               left.effectiveInputSuppressionFlags ==
                   right.effectiveInputSuppressionFlags &&
               left.collisionAvailabilityFlags ==
                   right.collisionAvailabilityFlags &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration &&
               left.collisionGeneration == right.collisionGeneration;
    }

    void assignHandInteractionSequences(
        RockProviderHandInteractionStateV1& current,
        const RockProviderHandInteractionStateV1& previous,
        const bool hasPrevious)
    {
        if (!hasPrevious) {
            current.stateSequence = 1;
            current.targetSequence =
                sameHandTarget(current, RockProviderHandInteractionStateV1{}) ?
                    0 :
                    1;
            current.gripSequence = handGripActive(current) ? 1 : 0;
            current.releaseSequence = 0;
            return;
        }

        current.stateSequence = sameHandInteractionPayload(current, previous) ?
            previous.stateSequence :
            advanceSequence(previous.stateSequence);
        current.targetSequence = sameHandTarget(current, previous) ?
            previous.targetSequence :
            advanceSequence(previous.targetSequence);
        const bool wasGripActive = handGripActive(previous);
        const bool gripActive = handGripActive(current);
        current.gripSequence = gripActive &&
                (!wasGripActive || !sameHandGrip(current, previous)) ?
            advanceSequence(previous.gripSequence) :
            previous.gripSequence;
        current.releaseSequence = wasGripActive && !gripActive ?
            advanceSequence(previous.releaseSequence) :
            previous.releaseSequence;
    }

    [[nodiscard]] bool sameLifecyclePayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept
    {
        return left.frikSkeletonReady == right.frikSkeletonReady &&
               left.menuBlocking == right.menuBlocking &&
               left.configBlocking == right.configBlocking &&
               left.providerReady == right.providerReady &&
               left.physicsScaleRevision == right.physicsScaleRevision &&
               left.lifecycleFlags == right.lifecycleFlags &&
               left.lastLifecycleReason == right.lastLifecycleReason &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration;
    }

    [[nodiscard]] bool sameWeaponPayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept
    {
        if (left.weaponFormId != right.weaponFormId ||
            left.weaponGenerationKey != right.weaponGenerationKey ||
            left.weaponBodyCount != right.weaponBodyCount) {
            return false;
        }
        for (std::uint32_t index = 0; index < left.weaponBodyCount; ++index) {
            if (left.weaponBodyIds[index] != right.weaponBodyIds[index]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool sameEquippedWeaponPayload(
        const RockProviderEquippedWeaponStateV1& left,
        const RockProviderEquippedWeaponStateV1& right) noexcept
    {
        return left.flags == right.flags &&
               left.weaponFormId == right.weaponFormId &&
               left.weaponGenerationKey == right.weaponGenerationKey &&
               left.transitionSequence == right.transitionSequence &&
               left.terminalSequence == right.terminalSequence &&
               left.transitionSource == right.transitionSource &&
               left.terminalResult == right.terminalResult &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration;
    }

    constexpr RockProviderApi ROCK_PROVIDER_API_FUNCTION_TABLE{
        .getVersion = &apiGetVersion,
        .getModVersion = &apiGetModVersion,
        .isProviderReady = &apiIsProviderReady,
        .registerFrameCallback = &apiRegisterFrameCallback,
        .unregisterFrameCallback = &apiUnregisterFrameCallback,
        .getFrameSnapshot = &apiGetFrameSnapshot,
        .queryWeaponContactAtPoint = &apiQueryWeaponContactAtPoint,
        .clearExternalBodies = &apiClearExternalBodies,
        .setOffhandInteractionReservation = &apiSetOffhandInteractionReservation,
        .registerExternalBodiesV1 = &apiRegisterExternalBodiesV1,
        .getWeaponEvidenceDetailCountV1 = &apiGetWeaponEvidenceDetailCountV1,
        .copyWeaponEvidenceDetailsV1 = &apiCopyWeaponEvidenceDetailsV1,
        .getWeaponEvidenceDetailPointCountV1 = &apiGetWeaponEvidenceDetailPointCountV1,
        .copyWeaponEvidenceDetailPointsV1 = &apiCopyWeaponEvidenceDetailPointsV1,
        .getBodyContactSnapshotV1 = &apiGetBodyContactSnapshotV1,
        .getPrimaryHandV1 = &apiGetPrimaryHandV1,
        .getOffhandHandV1 = &apiGetOffhandHandV1,
        .getHandFrameV1 = &apiGetHandFrameV1,
        .registerConsumerV1 = &apiRegisterConsumerV1,
        .unregisterConsumerV1 = &apiUnregisterConsumerV1,
        .getGrantedCapabilitiesV1 = &apiGetGrantedCapabilitiesV1,
        .getProviderLimitsV1 = &apiGetProviderLimitsV1,
        .getExternalContactSnapshotForOwnerV1 = &apiGetExternalContactSnapshotForOwnerV1,
        .requestForceGrabV1 = &apiRequestForceGrabV1,
        .getInteractionCommandResultV1 = &apiGetInteractionCommandResultV1,
        .requestForceReleaseV1 = &apiRequestForceReleaseV1,
        .requestThrownDropV1 = &apiRequestThrownDropV1,
        .setHandInputSuppressionV1 = &apiSetHandInputSuppressionV1,
        .clearHandInputSuppressionV1 = &apiClearHandInputSuppressionV1,
        .setWeaponPartTargetsV1 = &apiSetWeaponPartTargetsV1,
        .clearWeaponPartTargetsV1 = &apiClearWeaponPartTargetsV1,
        .setWeaponPartDriveTargetsV1 = &apiSetWeaponPartDriveTargetsV1,
        .clearWeaponPartDriveTargetsV1 = &apiClearWeaponPartDriveTargetsV1,
        .queryEquippedWeaponClassificationV1 = &apiQueryEquippedWeaponClassificationV1,
        .getWeaponPartGripStateV1 = &apiGetWeaponPartGripStateV1,
        .getRawWandButtonStateV1 = &apiGetRawWandButtonStateV1,
        .isNativePipboyInputSuppressedV1 = &apiIsNativePipboyInputSuppressedV1,
        .getWeaponEmitterCountV1 = &apiGetWeaponEmitterCountV1,
        .copyWeaponEmittersV1 = &apiCopyWeaponEmittersV1,
        .setNativeAnimationAuthorityV1 = &apiSetNativeAnimationAuthorityV1,
        .clearNativeAnimationAuthorityV1 = &apiClearNativeAnimationAuthorityV1,
        .getNativeAnimationAuthorityStateV1 = &apiGetNativeAnimationAuthorityStateV1,
        .registerAnimationPhaseCallbackV1 = &apiRegisterAnimationPhaseCallbackV1,
        .unregisterAnimationPhaseCallbackV1 = &apiUnregisterAnimationPhaseCallbackV1,
        .getEquippedWeaponGripStateV1 = &apiGetEquippedWeaponGripStateV1,
        .setHandVisualAuthorityV1 = &apiSetHandVisualAuthorityV1,
        .clearHandVisualAuthorityV1 = &apiClearHandVisualAuthorityV1,
        .publishNativeAnimationRuntimeV1 = &apiPublishNativeAnimationRuntimeV1,
        .setEquippedWeaponHandlingAuthorityV1 = &apiSetEquippedWeaponHandlingAuthorityV1,
        .clearEquippedWeaponHandlingAuthorityV1 = &apiClearEquippedWeaponHandlingAuthorityV1,
        .getEquippedWeaponHandlingStateV1 = &apiGetEquippedWeaponHandlingStateV1,
        .publishDebugOverlayV1 = &apiPublishDebugOverlayV1,
        .clearDebugOverlayV1 = &apiClearDebugOverlayV1,
        .getPresentedHandFrameV1 = &apiGetPresentedHandFrameV1,
        .getProviderLimitsExtV1 = &apiGetProviderLimitsExtV1,
        .getPublicStructureSizeV1 = &apiGetPublicStructureSizeV1,
        .registerFrameCallbackForOwnerV1 =
            &apiRegisterFrameCallbackForOwnerV1,
        .unregisterFrameCallbackForOwnerV1 =
            &apiUnregisterFrameCallbackForOwnerV1,
        .getHandInteractionStateV1 = &apiGetHandInteractionStateV1,
        .copyProviderEventsSinceV1 = &apiCopyProviderEventsSinceV1,
        .getEquippedWeaponStateV1 = &apiGetEquippedWeaponStateV1,
        .registerExternalBodiesForScopeV1 =
            &apiRegisterExternalBodiesForScopeV1,
        .clearExternalBodiesForScopeV1 =
            &apiClearExternalBodiesForScopeV1,
        .copyExternalContactsSinceV1 = &apiCopyExternalContactsSinceV1,
        .queryWeaponPartTargetResolutionV1 =
            &apiQueryWeaponPartTargetResolutionV1,
        .copyWeaponPartPoseSnapshotV1 =
            &apiCopyWeaponPartPoseSnapshotV1,
        .copyWeaponPartDriveApplicationResultsV1 =
            &apiCopyWeaponPartDriveApplicationResultsV1,
        .getScopeSightStateV1 = &apiGetScopeSightStateV1,
        .getWeaponCompositionStateV1 = &apiGetWeaponCompositionStateV1,
        .copyWeaponCompositionEntriesV1 =
            &apiCopyWeaponCompositionEntriesV1,
        .getSelectedAuthoredGripPoseV1 = &apiGetSelectedAuthoredGripPoseV1,
        .getPresentedHandPoseV1 = &apiGetPresentedHandPoseV1,
        .copySemanticHandContactsV1 = &apiCopySemanticHandContactsV1,
        .copyPlayerColliderDescriptorsV1 =
            &apiCopyPlayerColliderDescriptorsV1,
        .getHandCollisionAvailabilityV1 =
            &apiGetHandCollisionAvailabilityV1,
        .cancelInteractionCommandV1 = &apiCancelInteractionCommandV1,
        .getHandInputSuppressionStateV1 =
            &apiGetHandInputSuppressionStateV1,
        .acquireOffhandReservationV1 = &apiAcquireOffhandReservationV1,
        .renewOffhandReservationV1 = &apiRenewOffhandReservationV1,
        .releaseOffhandReservationV1 = &apiReleaseOffhandReservationV1,
        .getOffhandReservationStateV1 = &apiGetOffhandReservationStateV1,
        .clearNativeAnimationRuntimeV1 =
            &apiClearNativeAnimationRuntimeV1,
        .setTouchGrabTargetsForScopeV1 =
            &apiSetTouchGrabTargetsForScopeV1,
        .clearTouchGrabTargetsForScopeV1 =
            &apiClearTouchGrabTargetsForScopeV1,
        .copyTouchGrabStatesForScopeV1 =
            &apiCopyTouchGrabStatesForScopeV1,
        .requestTouchGrabYieldV1 =
            &apiRequestTouchGrabYieldV1,
        .requestEquippedWeaponHandV1 =
            &apiRequestEquippedWeaponHandV1,
        .queryWorldRaycastV1 =
            &apiQueryWorldRaycastV1,
    };

    constexpr RockProviderApiDescriptorV1 ROCK_PROVIDER_API_DESCRIPTOR{
        .size = sizeof(RockProviderApiDescriptorV1),
        .apiVersion = ROCK_PROVIDER_API_VERSION,
        .tableByteSize = sizeof(RockProviderApi),
        .featureBits = kProviderFeatureBitsV1,
        .featureBits2 = kProviderFeatureBits2V1,
        .table = &ROCK_PROVIDER_API_FUNCTION_TABLE,
    };
}

namespace rock::provider
{
    ROCK_PROVIDER_API const RockProviderApi* ROCK_PROVIDER_CALL ROCKAPI_GetProviderApi()
    {
        return &ROCK_PROVIDER_API_FUNCTION_TABLE;
    }

    ROCK_PROVIDER_API const RockProviderApiDescriptorV1* ROCK_PROVIDER_CALL
    ROCKAPI_GetDescriptorV1()
    {
        return &ROCK_PROVIDER_API_DESCRIPTOR;
    }

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.store(pi, std::memory_order_release);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        RockProviderFrameSnapshot snapshot{};
        snapshot.frameIndex = s_nextFrameIndex.fetch_add(1, std::memory_order_acq_rel);
        pi.fillProviderFrameSnapshot(snapshot);
        snapshot.externalBodyCount = currentExternalBodyCount();

        // These atomics are the allocation-free generation authority used by
        // lease validation during this frame. Publishing them before pruning
        // makes a generation transition revoke stale state in the same frame.
        s_currentWorldGeneration.store(
            snapshot.worldGeneration,
            std::memory_order_release);
        s_currentSkeletonGeneration.store(
            snapshot.skeletonGeneration,
            std::memory_order_release);
        s_currentProviderGeneration.store(
            snapshot.providerGeneration,
            std::memory_order_release);
        s_generationStateAvailable.store(true, std::memory_order_release);

        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            pruneExpiredHandInputSuppressionsLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
            pruneExpiredNativeAnimationAuthorityLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
            pruneExpiredEquippedWeaponHandlingAuthorityLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            pruneExpiredWeaponPartDrivesLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            pruneExpiredOffhandReservationLocked(snapshot.frameIndex);
        }
        snapshot.offhandReservation = currentOffhandReservation();

        provider_debug_overlay::PruneResult overlayPrune{};
        provider_debug_overlay::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            overlayPrune);
        for (std::uint32_t index = 0; index < overlayPrune.count; ++index) {
            publishAuthorityLostEvent(
                overlayPrune.publishers[index].ownerToken,
                RockProviderAuthorityKindV1::DebugOverlay,
                static_cast<std::uint32_t>(
                    overlayPrune.publishers[index].reason));
        }

        std::array<RockProviderWeaponPartGripStateV1, 2> partGripStates{};
        pi.fillProviderWeaponPartGripStates(partGripStates);

        std::array<RockProviderHandInteractionStateV1, 2>
            handInteractionStates{};
        pi.fillProviderHandInteractionStates(handInteractionStates);
        for (auto& state : handInteractionStates) {
            state.frameIndex = snapshot.frameIndex;
        }

        RockProviderEquippedWeaponStateV1 equippedWeaponState{};
        (void)pi.queryProviderEquippedWeaponStateV1(equippedWeaponState);
        equippedWeaponState.frameIndex = snapshot.frameIndex;

        bool hadPrevious = false;
        bool lifecycleChanged = false;
        bool equippedTerminalChanged = false;
        std::array<bool, 2> handChanged{};

        {
            std::scoped_lock lock(s_snapshotMutex);
            hadPrevious = s_hasSnapshot;
            const auto previousSnapshot = s_lastSnapshot;
            const auto previousHands = s_lastHandInteractionStates;
            const auto previousEquipped = s_lastEquippedWeaponState;

            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                auto& currentHand = handInteractionStates[index];
                const auto& previousHand = previousHands[index];
                if (hadPrevious &&
                    handGripActive(previousHand) &&
                    !handGripActive(currentHand) &&
                    currentHand.phase ==
                        RockProviderHandInteractionPhaseV1::Idle) {
                    currentHand.phase =
                        RockProviderHandInteractionPhaseV1::Releasing;
                    currentHand.targetKind = previousHand.targetKind;
                    currentHand.targetFormId = previousHand.targetFormId;
                    currentHand.primaryBodyId = previousHand.primaryBodyId;
                    currentHand.heldBodyCount = previousHand.heldBodyCount;
                    std::copy(
                        std::begin(previousHand.heldBodyIds),
                        std::end(previousHand.heldBodyIds),
                        std::begin(currentHand.heldBodyIds));
                }
                assignHandInteractionSequences(
                    currentHand,
                    previousHand,
                    hadPrevious);
                handChanged[index] = !hadPrevious ||
                    !sameHandInteractionPayload(
                        currentHand,
                        previousHand);
            }

            lifecycleChanged = !hadPrevious ||
                !sameLifecyclePayload(snapshot, previousSnapshot);
            const bool weaponChanged = !hadPrevious ||
                !sameWeaponPayload(snapshot, previousSnapshot) ||
                !sameEquippedWeaponPayload(
                    equippedWeaponState,
                    previousEquipped);
            const bool transitionChanged = !hadPrevious ||
                equippedWeaponState.transitionSequence !=
                    previousEquipped.transitionSequence ||
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            const bool collisionChanged = !hadPrevious ||
                snapshot.collisionGeneration !=
                    previousSnapshot.collisionGeneration;
            const bool handRolesChanged = !hadPrevious ||
                snapshot.primaryHand != previousSnapshot.primaryHand ||
                snapshot.offhandHand != previousSnapshot.offhandHand ||
                snapshot.offhandReservation !=
                    previousSnapshot.offhandReservation;

            if (lifecycleChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Lifecycle);
            }
            if (handChanged[0]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::RightHand);
            }
            if (handChanged[1]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::LeftHand);
            }
            if (weaponChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Weapon);
            }
            if (transitionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::EquippedTransition);
            }
            if (collisionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Collision);
            }
            if (handRolesChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::HandRoles);
            }
            snapshot.stateSequence = !hadPrevious ?
                1 :
                (snapshot.stateChangeMask != 0 ?
                        advanceSequence(previousSnapshot.stateSequence) :
                        previousSnapshot.stateSequence);
            snapshot.enrichmentFlags |= static_cast<std::uint32_t>(
                RockProviderFrameEnrichmentFlagV1::StateSequenceValid);
            snapshot.equippedWeaponTransitionSequence =
                equippedWeaponState.transitionSequence;

            equippedTerminalChanged = hadPrevious &&
                equippedWeaponState.terminalSequence != 0 &&
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
            s_lastPartGripStates = partGripStates;
            s_lastHandInteractionStates = handInteractionStates;
            s_lastEquippedWeaponState = equippedWeaponState;
        }

        if (hadPrevious && lifecycleChanged) {
            RockProviderEventV1 event{};
            event.kind = RockProviderEventKindV1::LifecycleChanged;
            event.result = static_cast<std::uint32_t>(
                snapshot.lastLifecycleReason);
            event.subjectSequence = snapshot.stateSequence;
            event.data[0] = snapshot.lifecycleFlags;
            event.data[1] = snapshot.providerReady;
            event.data[2] = snapshot.stateChangeMask;
            publishProviderEvent(event);
        }
        if (hadPrevious) {
            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                if (!handChanged[index]) {
                    continue;
                }
                const auto& state = handInteractionStates[index];
                RockProviderEventV1 event{};
                event.kind = RockProviderEventKindV1::GrabStateChanged;
                event.hand = state.hand;
                event.formId = state.targetFormId;
                event.result = static_cast<std::uint32_t>(state.phase);
                event.subjectSequence = state.stateSequence;
                event.data[0] = static_cast<std::uint32_t>(
                    state.targetKind);
                event.data[1] = state.primaryBodyId;
                event.data[2] = state.flags;
                publishProviderEvent(event);
            }
        }
        if (equippedTerminalChanged) {
            RockProviderEventV1 event{};
            event.kind =
                RockProviderEventKindV1::EquippedWeaponTransitionTerminal;
            event.weaponGenerationKey =
                equippedWeaponState.weaponGenerationKey;
            event.formId = equippedWeaponState.weaponFormId;
            event.result = static_cast<std::uint32_t>(
                equippedWeaponState.terminalResult);
            event.subjectSequence =
                equippedWeaponState.terminalSequence;
            event.data[0] = static_cast<std::uint32_t>(
                equippedWeaponState.transitionSource);
            event.data[1] = equippedWeaponState.flags;
            publishProviderEvent(event);
        }

        for (std::size_t index = 0; index < s_callbacks.size(); ++index) {
            CallbackSlot slot{};
            {
                std::scoped_lock lock(s_callbackMutex);
                slot = s_callbacks[index];
            }
            if (slot.callback) {
                FrameCallbackInvocationResult callbackResult{};
                try {
                    callbackResult = invokeFrameCallbackSafely(
                        slot.callback,
                        &snapshot,
                        slot.userData);
                } catch (...) {
                    callbackResult.healthy = false;
                }

                if (!callbackResult.healthy) {
                    HMODULE faultModule = nullptr;
                    constexpr auto moduleFlags =
                        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
                    const auto faultAddress =
                        callbackResult.exceptionAddress;
                    const bool hasFaultModule =
                        faultAddress != 0 &&
                        GetModuleHandleExA(
                            moduleFlags,
                            reinterpret_cast<LPCSTR>(faultAddress),
                            &faultModule) != FALSE;
                    if (hasFaultModule) {
                        std::array<char, MAX_PATH> modulePath{};
                        const auto pathLength = GetModuleFileNameA(
                            faultModule,
                            modulePath.data(),
                            static_cast<DWORD>(modulePath.size()));
                        const char* moduleName = modulePath.data();
                        if (pathLength != 0) {
                            if (const auto* slash =
                                    std::strrchr(moduleName, '\\')) {
                                moduleName = slash + 1;
                            }
                        } else {
                            moduleName = "<unknown-module>";
                        }
                        const auto moduleBase =
                            reinterpret_cast<std::uintptr_t>(
                                faultModule);
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction={}+0x{:X} "
                            "(0x{:016X}); unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            moduleName,
                            faultAddress - moduleBase,
                            faultAddress);
                    } else {
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction=0x{:016X}; "
                            "unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            faultAddress);
                    }
                    if (slot.ownerToken != 0) {
                        clearOwnerStateAfterCallbackFault(slot.ownerToken);
                    } else {
                        clearCallbackSlot(slot.token);
                    }
                }
            }
        }
    }

    void dispatchAnimationPhaseCallbacksV1(
        const RockProviderAnimationPhaseV1 phase,
        const float deltaSeconds)
    {
        if (!claimOrValidateAnimationOwnerThread()) {
            if (!s_animationThreadMismatchLogged.exchange(
                    true,
                    std::memory_order_acq_rel)) {
                logger::error(
                    "ROCK provider animation phases observed multiple threads; callback dispatch is disabled for the mismatching thread.");
            }
            return;
        }

        std::uint64_t phaseFrameIndex =
            s_activeAnimationPhaseFrameIndex.load(std::memory_order_acquire);
        const bool startingAnimationFrame = phaseFrameIndex == 0;
        if (phaseFrameIndex == 0) {
            phaseFrameIndex = s_nextAnimationPhaseFrameIndex.fetch_add(
                1,
                std::memory_order_acq_rel);
            if (phaseFrameIndex == 0) {
                phaseFrameIndex = s_nextAnimationPhaseFrameIndex.fetch_add(
                    1,
                    std::memory_order_acq_rel);
            }
            s_activeAnimationPhaseFrameIndex.store(
                phaseFrameIndex,
                std::memory_order_release);
        }

        if (startingAnimationFrame) {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            pruneHandVisualAuthorityLocked(currentProviderFrameIndex());
        }

        RockProviderAnimationPhaseContextV1 context{};
        context.phase = phase;
        context.frameIndex = phaseFrameIndex;
        context.deltaSeconds =
            std::isfinite(deltaSeconds) && deltaSeconds > 0.0f &&
                    deltaSeconds <= 0.1f ?
                deltaSeconds :
                (1.0f / 90.0f);
        context.activeNativeAnimationAuthorityFlags =
            s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);

        const auto& runtime = runtime_state::currentFrame();
        if (g_rockConfig.rockEnabled) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::RockEnabled);
        }
        if (apiIsProviderReady()) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::ProviderReady);
        }
        if (runtime.localSkeletonReady) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::SkeletonReady);
        }
        if (runtime.localMenuBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::MenuBlocking);
        }
        if (runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::ConfigBlocking);
        }
        if (runtime.visualAuthorityAvailable && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking &&
            !runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                RockProviderAnimationPhaseContextFlagV1::VisualWritesAllowed);
        }
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (s_hasSnapshot) {
                context.worldGeneration = s_lastSnapshot.worldGeneration;
                context.skeletonGeneration = s_lastSnapshot.skeletonGeneration;
                context.providerGeneration = s_lastSnapshot.providerGeneration;
            }
        }

        for (std::size_t index = 0;
             index < s_animationPhaseCallbacks.size();
             ++index) {
            AnimationPhaseCallbackSlot slot{};
            {
                std::scoped_lock lock(s_animationPhaseCallbackMutex);
                slot = s_animationPhaseCallbacks[index];
            }
            if (!slot.callback) {
                continue;
            }

            bool callbackHealthy = true;
            try {
                callbackHealthy = invokeAnimationPhaseCallbackSafely(
                    slot.callback,
                    &context,
                    slot.userData);
            } catch (...) {
                callbackHealthy = false;
            }
            if (callbackHealthy) {
                continue;
            }

            logger::error(
                "ROCK provider animation phase callback token {} owner {:016X} faulted; releasing its animation publications.",
                slot.token,
                slot.ownerToken);
            clearOwnerStateAfterCallbackFault(slot.ownerToken);
        }

        if (phase == RockProviderAnimationPhaseV1::Complete) {
            s_activeAnimationPhaseFrameIndex.store(0, std::memory_order_release);
        }
    }

    void clearExternalBodiesForProviderLoss()
    {
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            suppressionOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            targetOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            driveOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            nativeAnimationOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            handVisualOwners{};
        std::uint32_t suppressionOwnerCount = 0;
        std::uint32_t targetOwnerCount = 0;
        std::uint32_t driveOwnerCount = 0;
        std::uint32_t nativeAnimationOwnerCount = 0;
        std::uint32_t handVisualOwnerCount = 0;
        const auto addUniqueOwner = [](
                                        auto& owners,
                                        std::uint32_t& count,
                                        const std::uint64_t ownerToken) {
            if (ownerToken == 0) {
                return;
            }
            for (std::uint32_t index = 0; index < count; ++index) {
                if (owners[index] == ownerToken) {
                    return;
                }
            }
            if (count < owners.size()) {
                owners[count++] = ownerToken;
            }
        };
        std::uint64_t nativeRuntimeOwner = 0;
        std::uint64_t equippedHandlingOwner = 0;

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearAll();
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearAll();
        }
        clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1::ProviderNotReady);
        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            for (auto& slot : s_handInputSuppressions) {
                if (slot.ownerToken == 0) {
                    continue;
                }
                if (slot.active) {
                    addUniqueOwner(
                        suppressionOwners,
                        suppressionOwnerCount,
                        slot.ownerToken);
                }
                slot.active = false;
                slot.flags = 0;
                slot.lastInvalidationReason =
                    RockProviderSuppressionInvalidationReasonV1::ProviderLost;
                slot.lastInvalidatedFrame = currentProviderFrameIndex();
            }
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            for (const auto& slot : s_weaponPartTargets) {
                if (slot.active) {
                    addUniqueOwner(
                        targetOwners,
                        targetOwnerCount,
                        slot.ownerToken);
                }
            }
            for (const auto& slot : s_weaponPartDrives) {
                if (slot.active) {
                    addUniqueOwner(
                        driveOwners,
                        driveOwnerCount,
                        slot.ownerToken);
                }
            }
            s_weaponPartTargets = {};
            s_weaponPartDrives = {};
        }
        {
            std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
            for (const auto& slot : s_nativeAnimationAuthoritySlots) {
                if (slot.active) {
                    addUniqueOwner(
                        nativeAnimationOwners,
                        nativeAnimationOwnerCount,
                        slot.ownerToken);
                }
            }
            s_nativeAnimationAuthoritySlots = {};
            publishNativeAnimationAuthorityAggregateLocked();
        }
        {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            for (auto& slot : s_handVisualAuthoritySlots) {
                addUniqueOwner(
                    handVisualOwners,
                    handVisualOwnerCount,
                    slot.ownerToken);
                (void)clearHandVisualAuthoritySlotLocked(slot, true);
            }
        }
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            nativeRuntimeOwner = s_nativeAnimationRuntimeProviderOwner;
            s_nativeAnimationRuntimeProviderOwner = 0;
            s_nativeAnimationRuntimePublication = {};
            s_nativeAnimationRuntimeExpiresAfterFrame = 0;
            s_hasNativeAnimationRuntimePublication = false;
        }
        {
            std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
            equippedHandlingOwner =
                s_equippedWeaponHandlingAuthority.ownerToken;
            s_equippedWeaponHandlingAuthority = {};
        }
        provider_debug_overlay::PruneResult overlayLost{};
        provider_debug_overlay::clearAll(
            overlayLost,
            RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            clearOffhandReservationLocked(
                RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        }

        constexpr auto providerLost = static_cast<std::uint32_t>(
            RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        for (std::uint32_t index = 0;
             index < suppressionOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                suppressionOwners[index],
                RockProviderAuthorityKindV1::HandInputSuppression,
                providerLost);
        }
        for (std::uint32_t index = 0; index < targetOwnerCount; ++index) {
            publishAuthorityLostEvent(
                targetOwners[index],
                RockProviderAuthorityKindV1::WeaponPartTargets,
                providerLost);
        }
        for (std::uint32_t index = 0; index < driveOwnerCount; ++index) {
            publishAuthorityLostEvent(
                driveOwners[index],
                RockProviderAuthorityKindV1::WeaponPartDrive,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < nativeAnimationOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                nativeAnimationOwners[index],
                RockProviderAuthorityKindV1::NativeAnimation,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < handVisualOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                handVisualOwners[index],
                RockProviderAuthorityKindV1::HandVisual,
                providerLost);
        }
        if (nativeRuntimeOwner != 0) {
            publishAuthorityLostEvent(
                nativeRuntimeOwner,
                RockProviderAuthorityKindV1::NativeAnimationRuntime,
                providerLost);
        }
        if (equippedHandlingOwner != 0) {
            publishAuthorityLostEvent(
                equippedHandlingOwner,
                RockProviderAuthorityKindV1::EquippedWeaponHandling,
                providerLost);
        }
        for (std::uint32_t index = 0; index < overlayLost.count; ++index) {
            publishAuthorityLostEvent(
                overlayLost.publishers[index].ownerToken,
                RockProviderAuthorityKindV1::DebugOverlay,
                providerLost);
        }
        s_generationStateAvailable.store(false, std::memory_order_release);
        s_currentWorldGeneration.store(0, std::memory_order_release);
        s_currentSkeletonGeneration.store(0, std::memory_order_release);
        s_currentProviderGeneration.store(0, std::memory_order_release);
    }

    bool dequeueInteractionCommandV1(QueuedInteractionCommandV1& outCommand)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        InteractionCommandSlot* oldestSlot = nullptr;
        for (auto& slot : s_interactionCommands) {
            if (slot.active && (!oldestSlot || slot.command.commandId < oldestSlot->command.commandId)) {
                oldestSlot = &slot;
            }
        }

        if (oldestSlot) {
            outCommand = oldestSlot->command;
            *oldestSlot = {};
            for (auto& resultSlot : s_interactionResults) {
                if (resultSlot.active &&
                    resultSlot.result.ownerToken == outCommand.ownerToken &&
                    resultSlot.result.commandId == outCommand.commandId) {
                    resultSlot.result.stage =
                        RockProviderCommandStageV1::Committed;
                    resultSlot.result.committedFrame = currentProviderFrameIndex();
                    break;
                }
            }
            return true;
        }
        return false;
    }

    bool isInteractionCommandActiveV1(std::uint64_t ownerToken, std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return false;
        }

        std::scoped_lock lock(s_interactionCommandMutex);
        // The reservation is the durable ownership record. Result slots are a
        // bounded polling history and may legitimately wrap while a deferred
        // physics commit is still alive.
        return s_forceGrabReservations.matches(ownerToken, commandId);
    }

    bool completeInteractionCommandV1(const RockProviderInteractionCommandResultV1& result)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        if (result.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
            !s_forceGrabReservations.matches(result.ownerToken, result.commandId)) {
            return false;
        }
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
        return true;
    }

    void clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1 failure)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        /*
         * Results remain Queued after dequeue while PhysicsInteraction owns the
         * deferred commit. Cancel those as well as slots still in the bounded
         * queue so consumers never observe an immortal command.
         */
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.state == RockProviderInteractionCommandStateV1::Queued) {
                auto terminal = slot.result;
                terminal.state =
                    RockProviderInteractionCommandStateV1::Cancelled;
                terminal.failure = failure;
                terminal.failureStage = failure;
                terminal.stage = RockProviderCommandStageV1::Terminal;
                terminal.frameIndex = currentProviderFrameIndex();
                storeInteractionResultLocked(terminal);
            }
        }
        s_interactionCommands = {};
        s_forceGrabReservations.clear();
    }

    bool isExternalBodyId(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.containsBody(bodyId);
    }

    bool isExternalBodyDynamicPushSuppressed(std::uint32_t bodyId)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.suppressesRockDynamicPush(bodyId);
    }

    bool recordExternalHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t externalBodyId, std::uint64_t frameIndex)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        if (!s_externalBodies.containsBody(externalBodyId)) {
            return false;
        }
        s_externalBodies.recordHandContact(isLeft, handBodyId, externalBodyId, frameIndex);
        return true;
    }

    bool recordExternalContact(
        const RockProviderExternalContactV1& contact,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration)
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.recordContactV1(
            contact,
            worldGeneration,
            skeletonGeneration,
            providerGeneration);
    }

    RockProviderOffhandReservation currentOffhandReservation()
    {
        const auto expiry =
            s_offhandReservationExpiry.load(std::memory_order_acquire);
        if (expiry != 0 && !provider_lease_policy::isActive(
                currentProviderFrameIndex(),
                expiry)) {
            return RockProviderOffhandReservation::Normal;
        }
        return static_cast<RockProviderOffhandReservation>(s_offhandReservation.load(std::memory_order_acquire));
    }

    void setEquippedWeaponFiringHandIsLeft(const bool isLeft)
    {
        s_equippedWeaponFiringHandIsLeft.store(isLeft, std::memory_order_release);
    }

    bool getEquippedWeaponHandlingAuthorityV1(
        RockProviderEquippedWeaponHandlingRequestV1& outRequest)
    {
        outRequest = {};
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (!s_equippedWeaponHandlingAuthority.active) {
            return false;
        }
        outRequest = s_equippedWeaponHandlingAuthority.request;
        return true;
    }

    bool ownsEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken,
        const std::uint32_t requiredFlags)
    {
        if (ownerToken == 0) {
            return false;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(
            s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(
            frameIndex);
        return s_equippedWeaponHandlingAuthority.active &&
               s_equippedWeaponHandlingAuthority.ownerToken ==
                   ownerToken &&
               (s_equippedWeaponHandlingAuthority.request.flags &
                   requiredFlags) == requiredFlags;
    }

    void markInteractionCommandStageV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId,
        const RockProviderCommandStageV1 stage)
    {
        if (ownerToken == 0 || commandId == 0 ||
            stage == RockProviderCommandStageV1::Unknown ||
            stage == RockProviderCommandStageV1::Terminal) {
            return;
        }
        std::scoped_lock lock(s_interactionCommandMutex);
        for (auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId ||
                interaction_command_policy::isTerminal(slot.result.state)) {
                continue;
            }
            slot.result.stage = stage;
            const auto frameIndex = currentProviderFrameIndex();
            if (stage == RockProviderCommandStageV1::Committed &&
                slot.result.committedFrame == 0) {
                slot.result.committedFrame = frameIndex;
            }
            if (stage == RockProviderCommandStageV1::Applied &&
                slot.result.appliedFrame == 0) {
                slot.result.appliedFrame = frameIndex;
            }
            return;
        }
    }

    std::uint32_t currentHandInputSuppressionFlagsV1(RockProviderHand hand)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t flags = 0;
        std::scoped_lock lock(s_handInputSuppressionMutex);
        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.hand == hand) {
                flags |= slot.flags;
            }
        }
        return flags;
    }

    std::uint32_t currentNativeAnimationAuthorityFlagsV1()
    {
        return s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
    }

    void refreshNativeAnimationAuthorityLeasesV1()
    {
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
        pruneExpiredNativeAnimationAuthorityLocked(frameIndex);
    }

    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution)
    {
        outResolution = {};
        std::array<weapon_part_runtime::Target, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> runtimeTargets{};
        {
            std::scoped_lock lock(s_weaponPartMutex);
            for (std::size_t i = 0; i < s_weaponPartTargets.size(); ++i) {
                runtimeTargets[i] = toRuntimeTarget(s_weaponPartTargets[i]);
            }
        }

        const weapon_part_runtime::Contact contact{
            .weaponGenerationKey = query.weaponGenerationKey,
            .bodyId = query.bodyId,
            .sourceRoot = query.sourceRoot,
            .sourceName = std::string_view(query.sourceName, boundedStringLength(query.sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME)),
            .partKind = static_cast<WeaponPartKind>(query.partKind),
            .reloadRole = static_cast<WeaponReloadRole>(query.reloadRole),
            .supportRole = static_cast<WeaponSupportGripRole>(query.supportRole),
            .socketRole = static_cast<WeaponSocketRole>(query.socketRole),
            .actionRole = static_cast<WeaponActionRole>(query.actionRole),
        };
        const auto resolution = weapon_part_runtime::resolveTarget(runtimeTargets, contact);
        outResolution.whitelistActive = resolution.whitelistActive ? 1u : 0u;
        outResolution.matched = resolution.matched ? 1u : 0u;
        outResolution.grabMode = fromRuntimeGrabMode(resolution.grabMode);
        outResolution.groupId = resolution.groupId;
        outResolution.ownerToken = resolution.ownerToken;
        outResolution.priority = resolution.priority;
        // Non-exclusive targets can match without raising whitelistActive, so
        // a resolution is meaningful whenever either signal is set.
        return resolution.whitelistActive || resolution.matched;
    }

    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets,
        std::uint64_t* outOwnerTokens)
    {
        if (!outTargets || maxTargets == 0) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t copied = 0;
        std::scoped_lock lock(s_weaponPartMutex);
        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active) {
                continue;
            }
            if (copied >= maxTargets) {
                break;
            }
            outTargets[copied] = slot.target;
            outTargets[copied].sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
            if (outOwnerTokens) {
                outOwnerTokens[copied] = slot.ownerToken;
            }
            ++copied;
        }
        return copied;
    }

    std::uint32_t currentExternalBodyCount()
    {
        std::scoped_lock lock(s_externalBodyMutex);
        return s_externalBodies.bodyCount();
    }

    bool resolveTouchGrabTargetV1(
        const std::uint32_t bodyId,
        const std::uint32_t collisionLayer,
        const TouchGrabMotionClassV1 motionClass,
        const RockProviderHand hand,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        outMatch = s_touchGrabTargets.resolve(
            bodyId,
            collisionLayer,
            motionClass,
            hand,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            currentProviderFrameIndex());
        return outMatch.matched;
    }

    bool currentTouchGrabTargetV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        return s_touchGrabTargets.currentTarget(
            ownerToken,
            scopeToken,
            targetId,
            targetGeneration,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            currentProviderFrameIndex(),
            outMatch);
    }

    bool publishTouchGrabStateV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderTouchGrabStateV1& state)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        return s_touchGrabTargets.publishState(
            ownerToken,
            scopeToken,
            state,
            currentProviderFrameIndex());
    }

    void acknowledgeTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        s_touchGrabTargets.acknowledgeYield(
            ownerToken,
            scopeToken,
            targetId,
            targetGeneration);
    }
}
