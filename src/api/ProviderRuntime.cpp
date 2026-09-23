#include "ProviderFrameThreadOwner.h"
#include "providers/WeaponPartsMarshalling.h"
#include "api/InterfaceNegotiation.h"
#include "ProviderRuntimeServices.h"
#include "OwnerBindingPolicy.h"
#include "EventStreams.h"
#include "ROCKProviderApiInternal.h"
#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ProviderLeasePolicy.h"
#include "api/ProviderInstanceAccess.h"
#include "api/ProviderFrameClock.h"
#include "api/ProviderStatePolicy.h"
#include "api/TouchGrabRegistry.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

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
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/native/WeaponActionTrace.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "Version.h"

#ifdef DrawText
#undef DrawText
#endif

namespace rock::provider::runtime
{
    void drainDeferredRevocations();
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
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponClassificationSourceV1::Keyword) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::Keyword));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponClassificationSourceV1::WeaponData) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::WeaponData));
    static_assert(static_cast<std::uint32_t>(RockProviderWeaponClassificationSourceV1::EquipSlot) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::EquipSlot));
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

    ProviderInstanceAccess s_physicsInteraction;
    ProviderFrameClock s_frameClock;
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
    std::array<api::weaponparts::WeaponPartGripStateV1, 2> s_lastPartGripStates{};
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
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::WorldRaycasts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PlayerController) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::TargetDetails) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::PowerArmor) |
        static_cast<std::uint32_t>(
            RockProviderConsumerCapabilityV1::ColliderVisualizationOverride);
    constexpr std::uint32_t kImplementedForceGrabFlagsV1 =
        static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame) |
        static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory);
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
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVans) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressGrenadeQuickDraw) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::ReserveTriggerGripChord) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::ReserveButtonChord);
    std::uint32_t effectiveHandInputSuppressionFlags(RockProviderHand hand, std::uint32_t flags, std::uint64_t leftChord, std::uint64_t rightChord)
    {
        const auto reservation = static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::ReserveTriggerGripChord);
        const bool reserved = (flags & reservation) != 0;
        flags &= ~reservation;
        if (reserved && rock::input_remap_runtime::isTriggerGripChordHeld(hand == RockProviderHand::Left)) {
            flags |= static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
        }
        const auto generic = static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::ReserveButtonChord);
        const bool chordReserved = (flags & generic) != 0;
        flags &= ~generic;
        if (chordReserved && (leftChord || rightChord) &&
            rock::input_remap_runtime::areRawButtonsHeld(true, leftChord) &&
            rock::input_remap_runtime::areRawButtonsHeld(false, rightChord)) {
            flags |= static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput) |
                static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVats) |
                static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVans);
        }
        return flags;
    }
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
        std::array<InterfaceBinding, 13> interfaces{};
        bool revoked{ false };
        bool pendingRevoke{ false };
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


    struct HandInputSuppressionSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t leftChord{ 0 }, rightChord{ 0 };
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
    std::atomic<std::uint64_t> s_activeAnimationPhaseFrameIndex{ 0 };
    thread_local bool s_presentedReadbackPhase = false;
    ProviderFrameThreadOwner s_frameThreadOwner;
    std::atomic<std::uint32_t> s_threadMismatchReportedPhases{ 0 };

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
        rock::api::core::SnapshotV1 converted{};
        converted.frameIndex = snapshot->frameIndex;
        converted.frikSkeletonReady = snapshot->frikSkeletonReady;
        converted.menuBlocking = snapshot->menuBlocking;
        converted.configBlocking = snapshot->configBlocking;
        converted.providerReady = snapshot->providerReady;
        converted.lifecycleFlags = snapshot->lifecycleFlags;
        converted.lastLifecycleReason = static_cast<rock::api::core::LifecycleReason>(snapshot->lastLifecycleReason);
        converted.worldGeneration = snapshot->worldGeneration;
        converted.skeletonGeneration = snapshot->skeletonGeneration;
        converted.providerGeneration = snapshot->providerGeneration;
        converted.stableFrameCount = snapshot->stableFrameCount;
        converted.deltaSeconds = snapshot->deltaSeconds;
        converted.stateSequence = snapshot->stateSequence;
        FrameCallbackInvocationResult result{};
        if (!callback) {
            return result;
        }

#if defined(_MSC_VER)
        __try {
            callback(&converted, userData);
        } __except (captureFrameCallbackException(
            GetExceptionInformation(),
            &result)) {
        }
#else
        callback(&converted, userData);
#endif
        return result;
    }



    bool invokeAnimationPhaseCallbackSafely(
        RockProviderAnimationPhaseCallbackV1 callback,
        const RockProviderAnimationPhaseContextV1* context,
        void* userData)
    {
        rock::api::core::AnimationPhaseContextV1 converted{};
        converted.phase = static_cast<rock::api::core::AnimationPhaseV1>(context->phase);
        converted.flags = context->flags;
        converted.frameIndex = context->frameIndex;
        converted.deltaSeconds = context->deltaSeconds;
        converted.worldGeneration = context->worldGeneration;
        converted.skeletonGeneration = context->skeletonGeneration;
        converted.providerGeneration = context->providerGeneration;
        std::copy_n(context->reserved, std::size(converted.reserved), converted.reserved);
        if (!callback) {
            return true;
        }

#if defined(_MSC_VER)
        __try {
            callback(&converted, userData);
            return true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return false;
        }
#else
        callback(&converted, userData);
        return true;
#endif
    }

    void reportFrameThreadMismatch(const std::uint32_t phase) noexcept
    {
        const auto bit = 1u << (phase < 32 ? phase : 31);
        if (!(s_threadMismatchReportedPhases.fetch_or(bit, std::memory_order_relaxed) & bit)) {
            try {
                logger::error("ROCK provider thread rejected: phase={} (0=FrameBegin) owner={} caller={} frame={}; owner is established only at FrameBegin",
                    phase, s_frameThreadOwner.owner(), GetCurrentThreadId(), s_frameClock.current());
            } catch (...) {}
        }
    }

    [[nodiscard]] bool onAnimationOwnerThread()
    {
        return s_frameThreadOwner.allows(static_cast<std::uint32_t>(GetCurrentThreadId()));
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
            cleared = frik_visual_authority::clearHandWorld(slot.tag, hand) && cleared;
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

    std::uint64_t currentGameFrameIndex()
    {
        return s_frameClock.current();
    }

    std::uint64_t currentProviderFrameIndex()
    {
        return s_frameClock.leaseBoundary();
    }

    void publishProviderEvent(RockProviderEventV1 event)
    {
        event.size = sizeof(RockProviderEventV1);
        event.version = ROCK_PROVIDER_API_VERSION;
        if (event.frameIndex == 0) {
            event.frameIndex = currentGameFrameIndex();
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

        events::publish(event);
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
        result.acceptedFrame = currentGameFrameIndex();
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
                    currentGameFrameIndex());
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
                    storedResult.frameIndex = currentGameFrameIndex();
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
        {
            std::scoped_lock lock(s_consumerMutex);
            if (auto* slot = findConsumerSlotLocked(ownerToken)) slot->revoked = true;
        }
        if (ownerToken == 0) {
            return;
        }
        events::clearGrabCallback(ownerToken);
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
        provider_collider_visualization::clear(ownerToken);
        // Completed PA commands have already left the queue and scope registry.
        // Retire their hand attachments too, preserving the consumer's event
        // access and any manually attached peer hand.
        {
            const auto access = s_physicsInteraction.borrow();
            if (auto* pi = access.get()) {
                pi->releaseProviderPowerArmorGrabs(ownerToken);
            }
        }
        publishAuthorityLostEvent(
            ownerToken,
            RockProviderAuthorityKindV1::Unknown,
            static_cast<std::uint32_t>(
                RockProviderSuppressionInvalidationReasonV1::CallbackFault));
    }







    RockProviderResultV1 validateReadCapability(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability)
    {
        std::scoped_lock lock(s_consumerMutex);
        return validateRegisteredOwnerCapabilityLocked(ownerToken, capability);
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






    template <class Output, class Query>
    RockProviderResultV1 queryPhysicsInteractionValueV1(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability,
        Output* output,
        Query&& query,
        const bool requireAnimationThread = false)
    {
        provider_state_policy::clearQueryOutput(output);
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
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return query(*pi, *output) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
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
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
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





    RockProviderResultV1 validateTargetQuery(std::uint64_t ownerToken,
        const RockProviderReferenceQueryV1* query, RockProviderConsumerCapabilityV1 capability)
    {
        if (!query || !ownerToken || !query->referenceFormId || query->furnitureMarkerIndex < 0) return RockProviderResultV1::InvalidArgument;
        if (query->size != sizeof(*query)) return RockProviderResultV1::InvalidSize;
        if (query->version != ROCK_PROVIDER_API_VERSION) return RockProviderResultV1::UnsupportedVersion;
        const auto permission = validateReadCapability(ownerToken, capability);
        if (permission != RockProviderResultV1::Ok) return permission;
        if (!onAnimationOwnerThread()) return RockProviderResultV1::WrongThread;
        if (!apiIsProviderReady()) return RockProviderResultV1::NotReady;
        return validateGenerationGuards(query->worldGeneration, query->skeletonGeneration, query->providerGeneration);
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






    [[nodiscard]] RockProviderPoint3 toProviderPoint(
        const RE::NiPoint3& point) noexcept
    {
        return RockProviderPoint3{ point.x, point.y, point.z };
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
               left.reservedTargetIdentity ==
                   right.reservedTargetIdentity &&
               left.targetFormId == right.targetFormId &&
               left.primaryBodyId == right.primaryBodyId &&
               sameHeldBodies(left, right);
    }

    [[nodiscard]] bool handGripActive(
        const RockProviderHandInteractionStateV1& state) noexcept
    {
        return provider_state_policy::handHolding(state);
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
                RockProviderHandInteractionFlagV1::LooseWeapon) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::TouchGrab) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FixedSurfaceLatch) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::GlobalSurfaceLatch);
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
        if (!(current.flags & static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::Valid))) {
            // Loss of observation is not a measured release.
            current.gripSequence = previous.gripSequence;
            current.releaseSequence = previous.releaseSequence;
            return;
        }
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
               left.transitionWeaponFormId == right.transitionWeaponFormId &&
               left.terminalWeaponFormId == right.terminalWeaponFormId &&
               left.terminalSource == right.terminalSource &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration;
    }


#include "services/CoreService.inl"
#include "services/HandsService.inl"
#include "services/CollisionService.inl"
#include "services/GrabService.inl"
#include "services/TouchService.inl"
#include "services/WeaponService.inl"
#include "services/WeaponInventoryService.inl"
#include "services/WeaponPartsService.inl"
#include "services/AnimationService.inl"
#include "services/InputService.inl"
#include "services/ReferencesService.inl"
#include "services/PlayerControllerService.inl"
#include "services/DiagnosticsService.inl"

}

namespace rock::provider
{
    using namespace runtime;
    bool isPowerArmorGrabOwnerRegisteredV1(const std::uint64_t ownerToken)
    {
        return authorize(ownerToken,rock::api::InterfaceId::Grab,2,false)==rock::api::Status::Ok;
    }


    void beginGameFrame(const std::uint64_t frameIndex) noexcept
    {
        if (!s_frameThreadOwner.beginFrame(static_cast<std::uint32_t>(GetCurrentThreadId()))) {
            reportFrameThreadMismatch(0);
            return;
        }
        s_frameClock.beginFrame(frameIndex);
    }

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.publish(pi);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        drainDeferredRevocations();
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::ProviderFrameDispatch);
        RockProviderFrameSnapshot snapshot{};
        snapshot.frameIndex = s_frameClock.publishFrame();
        pi.fillProviderFrameSnapshot(snapshot);
        snapshot.externalBodyCount = currentExternalBodyCount();
        pi.refreshProviderWeaponSources();

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

        provider_collider_visualization::Invalidation
            colliderVisualizationInvalidation{};
        provider_collider_visualization::Snapshot
            colliderVisualizationSnapshot{};
        const bool colliderVisualizationActive =
            provider_collider_visualization::copySnapshot(
                colliderVisualizationSnapshot);
        const bool colliderVisualizationBodyCurrent =
            !colliderVisualizationActive ||
            pi.isProviderWeaponBodyCurrentV1(
                colliderVisualizationSnapshot.weaponGenerationKey,
                colliderVisualizationSnapshot.bodyId);
        provider_collider_visualization::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            snapshot.weaponGenerationKey,
            colliderVisualizationBodyCurrent,
            colliderVisualizationInvalidation);
        if (colliderVisualizationInvalidation.ownerToken != 0) {
            publishAuthorityLostEvent(
                colliderVisualizationInvalidation.ownerToken,
                RockProviderAuthorityKindV1::ColliderVisualization,
                static_cast<std::uint32_t>(
                    colliderVisualizationInvalidation.reason));
        }

        std::array<RockProviderWeaponPartGripStateV1, 2> partGripStates{};
        pi.fillProviderWeaponPartGripStates(partGripStates);
        // Resolve opaque source keys while the live catalog is owned by this
        // frame. Task readers copy these values without touching scene nodes.
        std::array<api::weaponparts::WeaponPartGripStateV1, 2> publicPartGripStates{};
        for (std::size_t index = 0; index < partGripStates.size(); ++index)
            api::boundary::convert(publicPartGripStates[index], partGripStates[index]);

        std::array<RockProviderHandInteractionStateV1, 2>
            handInteractionStates{};
        pi.fillProviderHandInteractionStates(handInteractionStates);
        for (auto& state : handInteractionStates) {
            state.frameIndex = snapshot.frameIndex;
        }

        std::uint8_t traceHands = 0;
        for (const auto& state : handInteractionStates) {
            using Flag = RockProviderHandInteractionFlagV1;
            const auto has = [&](Flag flag) { return (state.flags & static_cast<std::uint32_t>(flag)) != 0; };
            const bool holding = state.phase == RockProviderHandInteractionPhaseV1::Holding;
            const std::uint8_t bits = has(Flag::Valid) ? static_cast<std::uint8_t>(1 |
                ((holding && has(Flag::TouchGrab) && has(Flag::FixedSurfaceLatch)) ? 2 : 0) |
                ((holding && (has(Flag::LooseObject) || has(Flag::LooseWeapon))) ? 4 : 0) |
                (has(Flag::FiringGrip) ? 8 : 0)) : 0;
            traceHands |= static_cast<std::uint8_t>(bits << (state.hand == RockProviderHand::Left ? 4 : 0));
        }
        weapon_action_trace::publishHands(snapshot.frameIndex, traceHands);

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
                    (currentHand.flags & static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::Valid)) &&
                    handGripActive(previousHand) &&
                    !handGripActive(currentHand) &&
                    currentHand.phase ==
                        RockProviderHandInteractionPhaseV1::Idle) {
                    currentHand.phase =
                        RockProviderHandInteractionPhaseV1::Releasing;
                    currentHand.targetKind = previousHand.targetKind;
                    currentHand.reservedTargetIdentity =
                        previousHand.reservedTargetIdentity;
                    currentHand.targetFormId = previousHand.targetFormId;
                    currentHand.primaryBodyId = previousHand.primaryBodyId;
                    constexpr std::uint32_t touchGrabClassificationFlags =
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::TouchGrab) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                FixedSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                GlobalSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                SurfaceAnchorValid) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshSurfaceAnchor) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshFingerPose) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshCollisionFallback);
                    currentHand.flags |=
                        previousHand.flags &
                        touchGrabClassificationFlags;
                    currentHand.surfaceAnchorGame =
                        previousHand.surfaceAnchorGame;
                    currentHand.surfaceGripMode =
                        previousHand.surfaceGripMode;
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
            snapshot.rightHandState |= provider_state_policy::handStateFlags(handInteractionStates[0],
                PhysicsInteraction::s_rightHandDisabled.load(std::memory_order_acquire));
            snapshot.leftHandState |= provider_state_policy::handStateFlags(handInteractionStates[1],
                PhysicsInteraction::s_leftHandDisabled.load(std::memory_order_acquire));
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
            s_lastPartGripStates = publicPartGripStates;
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
                equippedWeaponState.terminalWeaponFormId == equippedWeaponState.weaponFormId ?
                    equippedWeaponState.weaponGenerationKey : 0;
            event.formId = equippedWeaponState.terminalWeaponFormId;
            event.result = static_cast<std::uint32_t>(
                equippedWeaponState.terminalResult);
            event.subjectSequence =
                equippedWeaponState.terminalSequence;
            event.data[0] = static_cast<std::uint32_t>(
                equippedWeaponState.terminalSource);
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
                    performance_profiler::ScopedTimer consumerTimer(performance_profiler::Scope::ProviderFrameConsumer);
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
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::ProviderAnimationDispatch);
        if (!onAnimationOwnerThread()) {
            // Graph hooks may fire before the first FRIK frame. They must not
            // select the thread that later authorizes every gameplay endpoint.
            if (s_frameThreadOwner.owner() != 0)
                reportFrameThreadMismatch(static_cast<std::uint32_t>(phase));
            return;
        }

        const bool previousReadbackPhase = std::exchange(s_presentedReadbackPhase, phase == RockProviderAnimationPhaseV1::Presented);
        const auto restoreReadbackPhase = F4SE::stl::scope_exit([previousReadbackPhase] {
            s_presentedReadbackPhase = previousReadbackPhase;
        });
        drainDeferredRevocations();
        const auto phaseFrameIndex = s_frameClock.beginPhase(
            static_cast<api::core::AnimationPhaseV1>(phase), timing.sequence);
        s_activeAnimationPhaseFrameIndex.store(phaseFrameIndex, std::memory_order_release);
        const bool startingAnimationFrame = phase == RockProviderAnimationPhaseV1::BeforeRock;

        if (startingAnimationFrame) {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            pruneHandVisualAuthorityLocked(currentProviderFrameIndex());
        }

        RockProviderAnimationPhaseContextV1 context{};
        context.phase = phase;
        context.frameIndex = phaseFrameIndex;
        /*
         * Truthful timing publication: the central game clock already
         * sanitized the delta, so an unmeasurable frame publishes zero
         * elapsed time instead of a fabricated nominal-rate value.
         */
        context.deltaSeconds = timing.valid ? timing.deltaSeconds : 0.0f;
        context.activeNativeAnimationAuthorityFlags =
            s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);

        const auto& runtime = runtime_state::currentFrame();
        context.flags |= static_cast<std::uint32_t>(
            RockProviderAnimationPhaseContextFlagV1::RockEnabled);
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
        if (!s_presentedReadbackPhase && runtime.visualAuthorityAvailable && runtime.localSkeletonReady &&
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
                performance_profiler::ScopedTimer consumerTimer(performance_profiler::Scope::ProviderAnimationConsumer);
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

        if (phase == RockProviderAnimationPhaseV1::Complete || phase == RockProviderAnimationPhaseV1::Presented) {
            s_activeAnimationPhaseFrameIndex.store(0, std::memory_order_release);
        }
    }

    void clearExternalBodiesForProviderLoss()
    {
        {
            std::scoped_lock lock(s_snapshotMutex);
            // Preserve lifecycle/terminal history, invalidate all live reads.
            s_lastSnapshot.providerReady = 0;
            s_lastSnapshot.lifecycleFlags &= ~(
                static_cast<std::uint32_t>(RockProviderLifecycleFlag::ProviderReady) |
                static_cast<std::uint32_t>(RockProviderLifecycleFlag::GeneratedBodiesValid) |
                static_cast<std::uint32_t>(RockProviderLifecycleFlag::PhysicsWriteAllowed) |
                static_cast<std::uint32_t>(RockProviderLifecycleFlag::VisualWriteAllowed));
            s_lastSnapshot.enrichmentFlags &= ~(
                static_cast<std::uint32_t>(RockProviderFrameEnrichmentFlagV1::RightHandTransformValid) |
                static_cast<std::uint32_t>(RockProviderFrameEnrichmentFlagV1::LeftHandTransformValid));
            s_lastSnapshot.rightHandState = 0;
            s_lastSnapshot.leftHandState = 0;
            for (auto& state : s_lastHandInteractionStates) state.flags = 0;
            s_lastPartGripStates = {};
            s_lastEquippedWeaponState.flags = 0;
        }
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
        provider_collider_visualization::Invalidation
            colliderVisualizationLost{};
        provider_collider_visualization::clearAll(
            colliderVisualizationLost,
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
        if (colliderVisualizationLost.ownerToken != 0) {
            publishAuthorityLostEvent(
                colliderVisualizationLost.ownerToken,
                RockProviderAuthorityKindV1::ColliderVisualization,
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
                    resultSlot.result.committedFrame = currentGameFrameIndex();
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
                terminal.frameIndex = currentGameFrameIndex();
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
            const auto frameIndex = currentGameFrameIndex();
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
        std::uint32_t flags = input_remap_runtime::uiInputSuppressionFlags(hand == RockProviderHand::Left);
        std::scoped_lock lock(s_handInputSuppressionMutex);
        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.hand == hand) {
                flags |= effectiveHandInputSuppressionFlags(hand, slot.flags, slot.leftChord, slot.rightChord);
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

    bool hasWeaponPartTargetsForGeneration(const std::uint64_t weaponGenerationKey)
    {
        std::scoped_lock lock(s_weaponPartMutex);
        for (const auto& slot : s_weaponPartTargets) {
            if (!slot.active) {
                continue;
            }
            const auto target = toRuntimeTarget(slot);
            if (weapon_part_runtime::targetAppliesToGeneration(target, weaponGenerationKey) &&
                weapon_part_runtime::targetHasUsableMatcher(target)) {
                return true;
            }
        }
        return false;
    }

    std::size_t copyWeaponPartTargets(std::span<weapon_part_runtime::Target> outTargets)
    {
        std::size_t count = 0;
        std::scoped_lock lock(s_weaponPartMutex);
        for (const auto& slot : s_weaponPartTargets) {
            if (count == outTargets.size()) {
                break;
            }
            if (slot.active) {
                outTargets[count++] = toRuntimeTarget(slot);
            }
        }
        return count;
    }

    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution)
    {
        outResolution = {};
        std::array<weapon_part_runtime::Target, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> runtimeTargets{};
        const auto targetCount = copyWeaponPartTargets(runtimeTargets);

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
        const auto resolution = weapon_part_runtime::resolveTarget(
            std::span(runtimeTargets).first(targetCount), contact);
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

    bool publishTouchGrabStateV1(std::uint64_t ownerToken,std::uint64_t scopeToken,const RockProviderTouchGrabStateV1& state) {
        bool changed=false; bool accepted=false;
        const auto frame=currentGameFrameIndex();
        {
            std::scoped_lock lock(s_touchGrabMutex);
            accepted=s_touchGrabTargets.publishState(ownerToken,scopeToken,state,frame,&changed);
        }
        if (accepted && changed) events::publishTouch(ownerToken,scopeToken,state,frame);
        return accepted;
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

namespace rock::provider::runtime {
    rock::api::Status authorize(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t permission, bool requireThread, OwnerAccess access) {
        using rock::api::Status;
        if (events::inSynchronousCallback()) return Status::Busy;
        const auto id=static_cast<std::uint32_t>(family);
        if (!owner || id<1 || id>13) return Status::InvalidArgument;
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto* slot=findConsumerSlotLocked(owner);
            if (!slot) return Status::OwnerNotRegistered;
            const auto status = authorizeBinding(slot->interfaces[id-1], slot->revoked, permission, access);
            if (status != Status::Ok) return status;
        }
        if (requireThread && !onAnimationOwnerThread() && !(id==1 && (permission==0 || permission==4) && s_frameThreadOwner.owner()==0)) return Status::WrongThread;
        return Status::Ok;
    }
    rock::api::Status bind(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t major, std::uint32_t permissions) {
        using rock::api::Status;
        if (events::inSynchronousCallback()) return Status::Busy;
        const auto id=static_cast<std::uint32_t>(family);
        if (id<1 || id>13) return Status::UnknownInterface;
        const auto* registration = api::discovery::findRegistration(api::discovery::registeredInterfaces(), family, major);
        if (!registration) return Status::UnsupportedMajor;
        const auto supported = registration->permissions;
        if (!permissions || (permissions & ~supported)) return Status::InvalidArgument;
        std::scoped_lock lock(s_consumerMutex);
        auto* slot=findConsumerSlotLocked(owner);
        if (!slot) return Status::OwnerNotRegistered;
        if (slot->revoked) return Status::OwnerRevoked;
        auto binding=slot->interfaces[id-1];
        const auto status=bindInterface(binding,major,permissions,supported);
        if (status!=Status::Ok) return status;
        if (permissions & 1) {
            const auto eventStatus=events::bind(owner,family);
            if (eventStatus!=Status::Ok) return eventStatus;
        }
        slot->interfaces[id-1]=binding;
        return Status::Ok;
    }
    rock::api::SampleV1 sample() {
        std::scoped_lock lock(s_snapshotMutex);
        const auto& s=s_lastSnapshot;
        return {s.frameIndex,s.stateSequence,s.worldGeneration,s.skeletonGeneration,s.providerGeneration,s.collisionGeneration};
    }
    void revoke(std::uint64_t owner) { clearOwnerStateAfterCallbackFault(owner); }
}

namespace rock::provider::runtime {
    void refreshSources() {
        const auto access=s_physicsInteraction.borrow();
        if (auto* pi=access.get()) pi->refreshProviderWeaponSources();
    }
    std::uintptr_t resolveSourceKey(std::uint64_t generation,std::uint64_t key) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get(); return pi?pi->resolveProviderWeaponSource(generation,key):0;
    }
    std::uint64_t sourceKey(std::uint64_t generation,std::uintptr_t node) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get(); return pi?pi->providerWeaponSourceKey(generation,node):0;
    }
    std::uint64_t sourceKeyForBody(std::uint64_t generation,std::uint32_t body) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get(); return pi?pi->providerWeaponSourceKeyForBody(generation,body):0;
    }
    std::uintptr_t resolveSourceName(std::uint64_t generation,const char* name) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get(); return pi?pi->resolveProviderWeaponSourceName(generation,name):0;
    }
    rock::api::Status copySources(std::uint64_t generation,std::uint32_t offset,rock::provider::WeaponSourceRecord* output,std::uint32_t capacity,std::uint32_t& copied,std::uint32_t& total) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get(); return pi?pi->copyProviderWeaponSources(generation,offset,output,capacity,copied,total):rock::api::Status::NotReady;
    }
}

namespace rock::provider::runtime {
    void deferRevoke(std::uint64_t owner) {
        std::scoped_lock lock(s_consumerMutex);
        if (auto* slot=findConsumerSlotLocked(owner)) { slot->revoked=true; slot->pendingRevoke=true; }
    }
    void drainDeferredRevocations() {
        std::array<std::uint64_t,64> pending{};
        std::size_t count=0;
        {
            std::scoped_lock lock(s_consumerMutex);
            for (auto& slot:s_consumers) if (slot.pendingRevoke) { pending[count++]=slot.token; slot.pendingRevoke=false; }
        }
        for (std::size_t i=0;i<count;++i) { logger::error("ROCK grab event callback faulted for owner {:016X}; revoking its publications",pending[i]); clearOwnerStateAfterCallbackFault(pending[i]); }
    }
}

namespace rock::provider::runtime {
    rock::api::Status querySourcePose(std::uint64_t generation,std::uint64_t key,WeaponSourcePose& output) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get();
        if (pi) pi->refreshProviderWeaponSources();
        return pi?pi->queryProviderWeaponSourcePose(generation,key,output):rock::api::Status::NotReady;
    }
}

namespace rock::provider::runtime {
    void reportBoundaryFailure(std::uint64_t owner,rock::api::InterfaceId family) noexcept {
        static std::atomic<std::uint32_t> reported{};
        const auto id=static_cast<std::uint32_t>(family);
        const auto bit=1u<<(id<32?id:0);
        if (reported.fetch_or(bit,std::memory_order_relaxed)&bit) return;
        try { logger::error("ROCK interface {} failed at the DLL boundary for owner {:016X}",id,owner); } catch (...) {}
    }
}

namespace rock::provider::runtime {
    rock::api::Status querySourcePath(std::uint64_t generation,std::uint64_t key,std::uint64_t& parentKey,std::uint32_t& childIndex) {
        const auto access=s_physicsInteraction.borrow(); auto* pi=access.get();
        if (!pi) return rock::api::Status::NotReady;
        pi->refreshProviderWeaponSources();
        return pi->queryProviderWeaponSourcePath(generation,key,parentKey,childIndex);
    }
}
