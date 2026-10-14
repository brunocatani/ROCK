#include "ProviderFrameThreadOwner.h"
#include "LooseWeaponStateService.h"
#include "NativeInteractionService.h"
#include "api/InterfaceNegotiation.h"
#include "ProviderRuntimeServices.h"
#include "physics-interaction/input/PhysicalReloadInput.h"
#include "physics-interaction/input/SupportTriggerInput.h"
#include "OwnerBindingPolicy.h"
#include "EventStreams.h"
#include "ProviderRuntime.h"
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

#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "Version.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"

#ifdef DrawText
#undef DrawText
#endif

namespace rock::provider::runtime
{
    void drainDeferredRevocations();
    using namespace rock::provider;
    using namespace rock;

    static_assert(static_cast<std::uint32_t>(api::collision::BodyZoneKind::LeftShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::LeftShoulder));
    static_assert(static_cast<std::uint32_t>(api::collision::BodyZoneKind::RightShoulder) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneKind::RightShoulder));
    static_assert(static_cast<std::uint32_t>(api::collision::BodyZoneSide::Left) ==
                  static_cast<std::uint32_t>(body_zone::BodyZoneSide::Left));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::GripKind::FiringGrip) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::FiringGrip));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::GripKind::SupportFullAuthority) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportFullAuthority));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::GripKind::SupportVisualOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::SupportVisualOnly));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::GripKind::PartCarry) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::PartCarry));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::GripKind::AttachOnly) ==
                  static_cast<std::uint32_t>(weapon_part_grip_report_policy::HandGripKind::AttachOnly));
    static_assert(static_cast<std::uint32_t>(
                      ::rock::api::animation::NativeAnimationAuthorityFlag::Arms) ==
                  (1u << 0));
    static_assert(static_cast<std::uint32_t>(
                      ::rock::api::animation::NativeAnimationAuthorityFlag::Hands) ==
                  (1u << 1));
    static_assert(static_cast<std::uint32_t>(
                      ::rock::api::animation::NativeAnimationAuthorityFlag::Weapon) ==
                  (1u << 2));
    static_assert(static_cast<std::uint32_t>(
                      ::rock::api::animation::NativeAnimationAuthorityFlag::ReloadPose) ==
                  ((1u << 0) | (1u << 1) | (1u << 2)));
    static_assert(static_cast<std::uint32_t>(
                      ::rock::api::animation::NativeAnimationAuthorityStatusFlag::CaptureFault) ==
                  (1u << 6));
    static_assert(static_cast<std::uint32_t>(api::weapon::ClassificationSource::Keyword) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::Keyword));
    static_assert(static_cast<std::uint32_t>(api::weapon::ClassificationSource::WeaponData) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::WeaponData));
    static_assert(static_cast<std::uint32_t>(api::weapon::ClassificationSource::EquipSlot) ==
                  static_cast<std::uint32_t>(WeaponClassificationSource::EquipSlot));
    // Public V1 part-kind / action-role values are a wire contract for
    // external consumers (PAPER_Toolkit); pin every enumerator to the internal
    // classification enums so a reorder breaks this build, not a consumer.
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Receiver) == static_cast<std::uint32_t>(WeaponPartKind::Receiver));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Barrel) == static_cast<std::uint32_t>(WeaponPartKind::Barrel));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Handguard) == static_cast<std::uint32_t>(WeaponPartKind::Handguard));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Foregrip) == static_cast<std::uint32_t>(WeaponPartKind::Foregrip));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Pump) == static_cast<std::uint32_t>(WeaponPartKind::Pump));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Stock) == static_cast<std::uint32_t>(WeaponPartKind::Stock));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Grip) == static_cast<std::uint32_t>(WeaponPartKind::Grip));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Magazine) == static_cast<std::uint32_t>(WeaponPartKind::Magazine));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Magwell) == static_cast<std::uint32_t>(WeaponPartKind::Magwell));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Bolt) == static_cast<std::uint32_t>(WeaponPartKind::Bolt));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Slide) == static_cast<std::uint32_t>(WeaponPartKind::Slide));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::ChargingHandle) == static_cast<std::uint32_t>(WeaponPartKind::ChargingHandle));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::BreakAction) == static_cast<std::uint32_t>(WeaponPartKind::BreakAction));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Cylinder) == static_cast<std::uint32_t>(WeaponPartKind::Cylinder));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Chamber) == static_cast<std::uint32_t>(WeaponPartKind::Chamber));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Shell) == static_cast<std::uint32_t>(WeaponPartKind::Shell));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Round) == static_cast<std::uint32_t>(WeaponPartKind::Round));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::LaserCell) == static_cast<std::uint32_t>(WeaponPartKind::LaserCell));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Lever) == static_cast<std::uint32_t>(WeaponPartKind::Lever));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Sight) == static_cast<std::uint32_t>(WeaponPartKind::Sight));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Accessory) == static_cast<std::uint32_t>(WeaponPartKind::Accessory));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::CosmeticAmmo) == static_cast<std::uint32_t>(WeaponPartKind::CosmeticAmmo));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Other) == static_cast<std::uint32_t>(WeaponPartKind::Other));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::LaserSight) == static_cast<std::uint32_t>(WeaponPartKind::LaserSight));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Flashlight) == static_cast<std::uint32_t>(WeaponPartKind::Flashlight));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::LaserFlashlightCombo) == static_cast<std::uint32_t>(WeaponPartKind::LaserFlashlightCombo));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Scope) == static_cast<std::uint32_t>(WeaponPartKind::Scope));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::MuzzleDevice) == static_cast<std::uint32_t>(WeaponPartKind::MuzzleDevice));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::PartKind::Bipod) == static_cast<std::uint32_t>(WeaponPartKind::Bipod));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ClassificationSource::NameToken) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::NameToken));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ClassificationSource::SlotAnchor) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::SlotAnchor));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ClassificationSource::RigAnchor) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::RigAnchor));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ClassificationSource::AttachmentEvidence) ==
                  static_cast<std::uint32_t>(WeaponPartClassificationSource::AttachmentEvidence));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::None) == static_cast<std::uint32_t>(WeaponActionRole::None));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Bolt) == static_cast<std::uint32_t>(WeaponActionRole::Bolt));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Slide) == static_cast<std::uint32_t>(WeaponActionRole::Slide));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::ChargingHandle) == static_cast<std::uint32_t>(WeaponActionRole::ChargingHandle));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Pump) == static_cast<std::uint32_t>(WeaponActionRole::Pump));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::BreakAction) == static_cast<std::uint32_t>(WeaponActionRole::BreakAction));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Cylinder) == static_cast<std::uint32_t>(WeaponActionRole::Cylinder));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Lever) == static_cast<std::uint32_t>(WeaponActionRole::Lever));
    static_assert(static_cast<std::uint32_t>(api::weaponparts::ActionRole::Latch) == static_cast<std::uint32_t>(WeaponActionRole::Latch));

    struct CallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        api::core::FrameCallback callback{ nullptr };
        void* userData{ nullptr };
    };

    ProviderInstanceAccess s_physicsInteraction;
    ProviderFrameClock s_frameClock;
    std::atomic<std::uint64_t> s_nextCallbackToken{ 1 };
    std::mutex s_callbackMutex;
    std::array<CallbackSlot, 16> s_callbacks{};

    std::mutex s_snapshotMutex;
    Publication s_lastSnapshot{};
    bool s_hasSnapshot{ false };
    std::atomic<bool> s_generationStateAvailable{ false };
    std::atomic<std::uint32_t> s_currentWorldGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentSkeletonGeneration{ 0 };
    std::atomic<std::uint32_t> s_currentProviderGeneration{ 0 };
    // Published together with the frame snapshot; indexed [right, left].
    std::array<api::weaponparts::GripState, 2> s_lastPartGripStates{};
    std::array<provider::HandInteractionPublication, 2> s_lastHandInteractionStates{};
    api::weapon::WeaponState s_lastEquippedWeaponState{};

    std::mutex s_touchGrabMutex;
    TouchGrabRegistry s_touchGrabTargets{};

    struct OffhandReservationSlot
    {
        std::uint64_t ownerToken{ 0 };
        api::Hand hand{ api::Hand::None };
        api::grab::HandReservationKind reservation{
            api::grab::HandReservationKind::Normal
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
    std::atomic<std::uint32_t> s_offhandReservationHand{
        static_cast<std::uint32_t>(api::Hand::None)
    };
    std::atomic<std::uint32_t> s_offhandReservation{
        static_cast<std::uint32_t>(api::grab::HandReservationKind::Normal)
    };

    // ROCK's runtime firing hand (left-hand fire); published each frame by
    // PhysicsInteraction so primary/offhand resolution tracks who fires.
    std::atomic<bool> s_equippedWeaponFiringHandIsLeft{ false };

    constexpr std::uint64_t kRockIssuedOwnerTokenNamespace = 0xA000'0000'0000'0000ull;
    constexpr std::uint64_t kRockIssuedOwnerTokenSequenceMask = 0x0FFF'FFFF'FFFF'FFFFull;
    constexpr std::uint32_t kImplementedForceGrabFlags =
        static_cast<std::uint32_t>(api::grab::ForceGrabFlag::UsePreferredGrabPointGame) |
        provider::kForceGrabFromPlayerInventory;
    constexpr std::uint32_t kImplementedForceReleaseFlags =
        static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::RequireMatchingTarget) |
        static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::UseVelocityHavok);
    constexpr std::uint32_t kImplementedThrownDropFlags =
        static_cast<std::uint32_t>(api::grab::ThrownDropFlag::ImmediateCollisionRestore) |
        static_cast<std::uint32_t>(api::grab::ThrownDropFlag::RequireMatchingTarget) |
        static_cast<std::uint32_t>(api::grab::ThrownDropFlag::UseVelocityHavok);
    constexpr std::uint32_t kImplementedHandInputSuppressionFlags =
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressConfigModeChord) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressOpenVrGameInput) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressNativeVats) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressNativeVans) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressGrenadeQuickDraw) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::ReserveTriggerGripChord) |
        static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::ReserveButtonChord);
    std::uint32_t effectiveHandInputSuppressionFlags(api::Hand hand, std::uint32_t flags, std::uint64_t leftChord, std::uint64_t rightChord)
    {
        const auto reservation = static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::ReserveTriggerGripChord);
        const bool reserved = (flags & reservation) != 0;
        flags &= ~reservation;
        if (reserved && rock::input_remap_runtime::isTriggerGripChordHeld(hand == api::Hand::Left)) {
            flags |= static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressOpenVrGameInput);
        }
        const auto generic = static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::ReserveButtonChord);
        const bool chordReserved = (flags & generic) != 0;
        flags &= ~generic;
        if (chordReserved && (leftChord || rightChord) &&
            rock::input_remap_runtime::areRawButtonsHeld(true, leftChord) &&
            rock::input_remap_runtime::areRawButtonsHeld(false, rightChord)) {
            flags |= static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressOpenVrGameInput) |
                static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressNativeVats) |
                static_cast<std::uint32_t>(::rock::api::input::HandInputSuppressionFlag::SuppressNativeVans);
        }
        return flags;
    }
    constexpr std::uint32_t kWeaponPartTargetMatcherFlags =
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchPartKind) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchReloadRole) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSupportRole) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSocketRole) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchActionRole);
    constexpr std::uint32_t kImplementedWeaponPartTargetFlags =
        kWeaponPartTargetMatcherFlags |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::NonExclusive);
    constexpr std::uint32_t kImplementedWeaponPartDriveMatcherFlags =
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey) |
        static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName);
    constexpr std::uint32_t kProviderInvalidBodyId = 0x7FFF'FFFFu;
    // One bit per finger local transform; a hand visual request carries 15.
    constexpr std::uint16_t kAllFingerLocalTransforms = 0x7FFFu;

    // Interface IDs are 1-based; Storage is the highest.
    constexpr std::uint32_t kInterfaceSlotCount = static_cast<std::uint32_t>(rock::api::InterfaceId::Storage);

    struct ConsumerSlot
    {
        std::array<InterfaceBinding, kInterfaceSlotCount> interfaces{};
        bool revoked{ false };
        bool pendingRevoke{ false };
        std::uint64_t token{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t worldRaycastFrameIndex{ 0 };
        std::uint32_t worldRaycastCount{ 0 };
        char modName[64]{};
    };

    std::mutex s_consumerMutex;
    std::array<ConsumerSlot, api::core::kMaxOwners> s_consumers{};
    std::atomic<std::uint64_t> s_nextConsumerTokenSequence{ 1 };

    struct InteractionCommandSlot
    {
        bool active{ false };
        QueuedInteractionCommand command{};
    };

    struct InteractionCommandResultSlot
    {
        bool active{ false };
        api::grab::InteractionCommandResult result{};
    };

    std::mutex s_interactionCommandMutex;
    std::array<InteractionCommandSlot, api::grab::kMaxQueuedCommands> s_interactionCommands{};
    std::array<InteractionCommandResultSlot, api::grab::kMaxCommandResults> s_interactionResults{};
    static_assert(
        api::grab::kMaxCommandResults >= api::grab::kMaxQueuedCommands + 3,
        "Result history must retain the full queue, both deferred force-grab slots, and one dequeued command.");
    std::size_t s_nextInteractionResultSlot{ 0 };
    std::atomic<std::uint64_t> s_nextInteractionCommandId{ 1 };
    interaction_command_policy::ForceGrabReservations s_forceGrabReservations{};


    struct HandInputSuppressionSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t leftChord{ 0 }, rightChord{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        ::rock::api::input::SuppressionInvalidationReason lastInvalidationReason{
            ::rock::api::input::SuppressionInvalidationReason::None
        };
        std::uint64_t lastInvalidatedFrame{ 0 };
    };

    std::mutex s_handInputSuppressionMutex;
    std::array<HandInputSuppressionSlot, api::input::kMaxSuppressions> s_handInputSuppressions{};

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
    std::array<NativeAnimationAuthoritySlot, api::core::kMaxOwners> s_nativeAnimationAuthoritySlots{};
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityFlags{ 0 };
    std::atomic<std::uint32_t> s_nativeAnimationAuthorityOwnerCount{ 0 };

    struct AnimationPhaseCallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        api::animation::PhaseCallback callback{ nullptr };
        void* userData{ nullptr };
    };

    std::mutex s_animationPhaseCallbackMutex;
    std::array<AnimationPhaseCallbackSlot, api::animation::kMaxPhaseCallbacks>
        s_animationPhaseCallbacks{};
    std::atomic<std::uint64_t> s_nextAnimationPhaseCallbackToken{ 1 };
    std::atomic<std::uint64_t> s_activeAnimationPhaseFrameIndex{ 0 };
    thread_local bool s_presentedReadbackPhase = false;
    ProviderFrameThreadOwner s_frameThreadOwner;
    std::atomic<std::uint32_t> s_threadMismatchReportedPhases{ 0 };

    struct HandVisualAuthoritySlot
    {
        std::uint64_t ownerToken{ 0 };
        api::Hand hand{ api::Hand::None };
        std::uint32_t publishedFlags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        char tag[64]{};
    };

    std::mutex s_handVisualAuthorityMutex;
    std::array<HandVisualAuthoritySlot, api::core::kMaxOwners * 2>
        s_handVisualAuthoritySlots{};

    std::mutex s_nativeAnimationRuntimePublicationMutex;
    std::uint64_t s_nativeAnimationRuntimeProviderOwner{ 0 };
    ::rock::api::animation::NativeAnimationRuntimePublication
        s_nativeAnimationRuntimePublication{};
    bool s_hasNativeAnimationRuntimePublication{ false };
    std::uint64_t s_nativeAnimationRuntimeExpiresAfterFrame{ 0 };

    struct EquippedWeaponHandlingAuthoritySlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        api::weapon::HandlingRequest request{};
    };

    std::mutex s_equippedWeaponHandlingAuthorityMutex;
    EquippedWeaponHandlingAuthoritySlot s_equippedWeaponHandlingAuthority{};

    struct WeaponPartTargetSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        PartTarget target{};
    };

    struct WeaponPartDriveSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        PartDrive target{};
    };

    std::mutex s_weaponPartMutex;
    weapon_part_points::Registry s_weaponPartGrabPoints;
    std::array<WeaponPartTargetSlot, api::weaponparts::kMaxTargets> s_weaponPartTargets{};
    std::array<WeaponPartDriveSlot, api::weaponparts::kMaxDrives> s_weaponPartDrives{};

    api::Status validateRegisteredOwnerLocked(std::uint64_t ownerToken);
    [[nodiscard]] bool generationGuardsStale(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration);
    // An authority whose loss is reported to its owner, each through one
    // family's event stream.
    enum class LostAuthority : std::uint32_t
    {
        Owner,
        HandInputSuppression,
        WeaponPartTargets,
        WeaponPartDrive,
        NativeAnimation,
        NativeAnimationRuntime,
        HandVisual,
        WeaponHandling,
        HandReservation,
    };
    void publishAuthorityLostEvent(
        std::uint64_t ownerToken,
        LostAuthority authority,
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
        api::core::FrameCallback callback,
        const Publication* snapshot,
        void* userData)
    {
        const auto converted = toCoreSnapshot(*snapshot);
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
        api::animation::PhaseCallback callback,
        const api::animation::AnimationPhaseContext* context,
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

    // Transform is a provider or public API transform: rotate[9], translate[3], scale.
    template <class Transform>
    [[nodiscard]] bool finiteProviderTransform(const Transform& transform)
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
        const api::Point3& point)
    {
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.z);
    }

    template <class Transform>
    [[nodiscard]] RE::NiTransform toNiTransform(const Transform& source)
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

    [[nodiscard]] constexpr frik_visual_authority::Hand toVisualHand(
        const api::Hand hand)
    {
        return hand == api::Hand::Left ?
            frik_visual_authority::Hand::Left :
            frik_visual_authority::Hand::Right;
    }


    [[nodiscard]] HandVisualAuthoritySlot* findHandVisualAuthoritySlotLocked(
        const std::uint64_t ownerToken,
        const api::Hand hand)
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
            static_cast<std::uint32_t>(::rock::api::animation::HandVisualAuthorityFlag::WorldTransform);
        const auto fingerFlag =
            static_cast<std::uint32_t>(::rock::api::animation::HandVisualAuthorityFlag::FingerLocalTransforms);
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
        const api::Hand hand,
        const bool releaseSlots)
    {
        bool cleared = true;
        std::scoped_lock lock(s_handVisualAuthorityMutex);
        for (auto& slot : s_handVisualAuthoritySlots) {
            if (slot.ownerToken != ownerToken ||
                (hand != api::Hand::None && slot.hand != hand)) {
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
                LostAuthority::HandVisual,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                        ::rock::api::input::SuppressionInvalidationReason::Expired));
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
                LostAuthority::WeaponHandling,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                        ::rock::api::input::SuppressionInvalidationReason::Expired));
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

    // Runtime events carry the game frame and the current generations.
    template <class Event>
    [[nodiscard]] Event stampedEvent(const std::uint32_t kind, const std::uint64_t ownerToken = 0)
    {
        Event event{};
        event.kind = kind;
        event.ownerToken = ownerToken;
        event.frameIndex = currentGameFrameIndex();
        if (s_generationStateAvailable.load(std::memory_order_acquire)) {
            event.worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
            event.skeletonGeneration = s_currentSkeletonGeneration.load(std::memory_order_acquire);
            event.providerGeneration = s_currentProviderGeneration.load(std::memory_order_acquire);
        }
        return event;
    }

    void publishAuthorityLostEvent(
        const std::uint64_t ownerToken,
        const LostAuthority authority,
        const std::uint32_t reason)
    {
        // Animation authority codes: 1 native animation, 2 hand visual, 3 the
        // native animation runtime. WeaponParts: 1 part targets, 2 part drives.
        switch (authority) {
        case LostAuthority::NativeAnimation:
        case LostAuthority::HandVisual:
        case LostAuthority::NativeAnimationRuntime: {
            auto event = stampedEvent<api::animation::Event>(
                static_cast<std::uint32_t>(api::animation::EventKind::AuthorityLost), ownerToken);
            event.authority = authority == LostAuthority::NativeAnimation ? 1u :
                authority == LostAuthority::HandVisual ? 2u : 3u;
            event.reason = reason;
            events::publish(event, ownerToken);
            return;
        }
        case LostAuthority::HandInputSuppression: {
            auto event = stampedEvent<api::input::Event>(
                static_cast<std::uint32_t>(api::input::EventKind::AuthorityLost), ownerToken);
            event.authority = 1;
            event.reason = reason;
            events::publish(event, ownerToken);
            return;
        }
        case LostAuthority::WeaponPartTargets:
        case LostAuthority::WeaponPartDrive: {
            auto event = stampedEvent<api::weaponparts::Event>(
                static_cast<std::uint32_t>(api::weaponparts::EventKind::AuthorityLost), ownerToken);
            event.authority = authority == LostAuthority::WeaponPartTargets ? 1u : 2u;
            event.reason = reason;
            events::publish(event, ownerToken);
            return;
        }
        case LostAuthority::WeaponHandling: {
            auto event = stampedEvent<api::weapon::Event>(
                static_cast<std::uint32_t>(api::weapon::EventKind::AuthorityLost), ownerToken);
            event.authority = 1;
            event.reason = reason;
            events::publish(event, ownerToken);
            return;
        }
        case LostAuthority::HandReservation: {
            auto event = stampedEvent<api::grab::Event>(
                static_cast<std::uint32_t>(api::grab::EventKind::ReservationLost), ownerToken);
            event.failure = reason;
            events::publish(event, ownerToken);
            return;
        }
        case LostAuthority::Owner:
        default: {
            auto event = stampedEvent<api::core::Event>(
                static_cast<std::uint32_t>(api::core::EventKind::OwnerRevoked), ownerToken);
            event.reason = reason;
            events::publish(event, ownerToken);
            return;
        }
        }
    }

    void publishCommandCompletedEvent(const api::grab::InteractionCommandResult& result)
    {
        auto event = stampedEvent<api::grab::Event>(
            static_cast<std::uint32_t>(api::grab::EventKind::CommandCompleted), result.ownerToken);
        event.hand = result.hand;
        event.subjectSequence = result.commandId;
        event.commandState = static_cast<std::uint32_t>(result.state);
        event.formId = result.targetFormId;
        event.commandKind = static_cast<std::uint32_t>(result.kind);
        event.failure = static_cast<std::uint32_t>(result.failure);
        events::publish(event, result.ownerToken);
    }

    void publishOffhandReservationLocked(const OffhandReservationSlot& slot)
    {
        s_offhandReservationOwner.store(
            slot.ownerToken,
            std::memory_order_release);
        s_offhandReservation.store(
            static_cast<std::uint32_t>(slot.reservation),
            std::memory_order_release);
        s_offhandReservationHand.store(
            static_cast<std::uint32_t>(slot.hand),
            std::memory_order_release);
        s_offhandReservationExpiry.store(
            slot.expiresAfterFrame,
            std::memory_order_release);
    }

    void clearOffhandReservationLocked(
        const ::rock::api::input::SuppressionInvalidationReason reason)
    {
        const auto ownerToken = s_offhandReservationSlot.ownerToken;
        s_offhandReservationSlot = {};
        publishOffhandReservationLocked(s_offhandReservationSlot);
        if (ownerToken != 0) {
            publishAuthorityLostEvent(
                ownerToken,
                LostAuthority::HandReservation,
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
                    ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                    ::rock::api::input::SuppressionInvalidationReason::Expired);
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
                LostAuthority::NativeAnimationRuntime,
                static_cast<std::uint32_t>(
                    generationChanged ?
                        ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                        ::rock::api::input::SuppressionInvalidationReason::Expired));
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
                    ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                    ::rock::api::input::SuppressionInvalidationReason::Expired;
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    LostAuthority::HandInputSuppression,
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
        api::Hand hand,
        ::rock::api::input::SuppressionInvalidationReason reason =
            ::rock::api::input::SuppressionInvalidationReason::ExplicitClear)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active || slot.ownerToken != ownerToken) {
                continue;
            }
            if (hand == api::Hand::None || slot.hand == hand) {
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
                    LostAuthority::NativeAnimation,
                    static_cast<std::uint32_t>(
                        generationChanged ?
                            ::rock::api::input::SuppressionInvalidationReason::GenerationChanged :
                            ::rock::api::input::SuppressionInvalidationReason::Expired));
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
        s_weaponPartGrabPoints.clear(ownerToken);
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
                    LostAuthority::WeaponPartDrive,
                    static_cast<std::uint32_t>(
                        ::rock::api::input::SuppressionInvalidationReason::Expired));
                slot = {};
            }
        }
    }

    bool hasValidWeaponPartMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        // NonExclusive is a semantics flag, not a matcher: at least one match
        // flag must still be present for the target to select anything.
        if ((flags & ~kImplementedWeaponPartTargetFlags) != 0 || (flags & kWeaponPartTargetMatcherFlags) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, weapon_part_runtime::kMaxSourceName) == 0) {
            return false;
        }
        return true;
    }

    bool hasConcreteWeaponPartDriveMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        if ((flags & ~kImplementedWeaponPartDriveMatcherFlags) != 0 || (flags & kImplementedWeaponPartDriveMatcherFlags) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, weapon_part_runtime::kMaxSourceName) == 0) {
            return false;
        }
        return ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey)) != 0 && sourceRoot != 0) ||
               ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId)) != 0 && bodyId != kProviderInvalidBodyId) ||
               ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName)) != 0 &&
                   boundedStringLength(sourceName, weapon_part_runtime::kMaxSourceName) != 0);
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

    bool hasValidWeaponPartTargetSemantics(const api::weaponparts::Target& target)
    {
        const auto flags = target.flags;
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchPartKind)) != 0 &&
            !isValidWeaponPartKindValue(target.partKind)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchReloadRole)) != 0 &&
            !isValidWeaponReloadRoleValue(target.reloadRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSupportRole)) != 0 &&
            !isValidWeaponSupportRoleValue(target.supportRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSocketRole)) != 0 &&
            !isValidWeaponSocketRoleValue(target.socketRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchActionRole)) != 0 &&
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

    bool isValidWeaponPartGrabMode(api::weaponparts::GrabMode mode)
    {
        return mode == api::weaponparts::GrabMode::FullTwoHandAuthority ||
               mode == api::weaponparts::GrabMode::AttachOnly;
    }

    bool isValidWeaponPartDriveSpace(api::weaponparts::DriveSpace space)
    {
        return space == api::weaponparts::DriveSpace::WeaponRootLocal ||
               space == api::weaponparts::DriveSpace::SourceParentLocal;
    }

    bool isFiniteProviderTransform(const api::Transform& transform)
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

    weapon_part_runtime::GrabMode toRuntimeGrabMode(api::weaponparts::GrabMode mode)
    {
        switch (mode) {
        case api::weaponparts::GrabMode::FullTwoHandAuthority:
            return weapon_part_runtime::GrabMode::FullTwoHandAuthority;
        case api::weaponparts::GrabMode::AttachOnly:
            return weapon_part_runtime::GrabMode::AttachOnly;
        case api::weaponparts::GrabMode::None:
        default:
            return weapon_part_runtime::GrabMode::None;
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

    api::Status validateRegisteredOwnerLocked(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        return findConsumerSlotLocked(ownerToken) ? api::Status::Ok : api::Status::OwnerNotRegistered;
    }

    api::Status validateGenerationGuards(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration)
    {
        if (worldGeneration == 0 && skeletonGeneration == 0 && providerGeneration == 0) {
            return api::Status::Ok;
        }

        if (!s_generationStateAvailable.load(std::memory_order_acquire)) {
            return api::Status::NotReady;
        }
        if (worldGeneration != 0 &&
            worldGeneration != s_currentWorldGeneration.load(
                std::memory_order_acquire)) {
            return api::Status::WorldNotReady;
        }
        if (skeletonGeneration != 0 &&
            skeletonGeneration != s_currentSkeletonGeneration.load(
                std::memory_order_acquire)) {
            return api::Status::NotReady;
        }
        if (providerGeneration != 0 &&
            providerGeneration != s_currentProviderGeneration.load(
                std::memory_order_acquire)) {
            return api::Status::NotReady;
        }
        return api::Status::Ok;
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

    api::Hand commandHand(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.hand;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.hand;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.hand;
        default:
            return api::Hand::None;
        }
    }

    std::uint32_t commandTargetFormId(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.targetFormId;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.targetFormId;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.targetFormId;
        default:
            return 0;
        }
    }

    std::uint32_t commandTargetBodyId(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.targetBodyId;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.targetBodyId;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.targetBodyId;
        default:
            return kProviderInvalidBodyId;
        }
    }

    std::uint32_t commandWorldGeneration(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.worldGeneration;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.worldGeneration;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.worldGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandSkeletonGeneration(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.skeletonGeneration;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.skeletonGeneration;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.skeletonGeneration;
        default:
            return 0;
        }
    }

    std::uint32_t commandProviderGeneration(const QueuedInteractionCommand& command)
    {
        switch (command.kind) {
        case api::grab::InteractionCommandKind::ForceGrab:
            return command.forceGrab.providerGeneration;
        case api::grab::InteractionCommandKind::ForceRelease:
            return command.forceRelease.providerGeneration;
        case api::grab::InteractionCommandKind::ThrownDrop:
            return command.thrownDrop.providerGeneration;
        default:
            return 0;
        }
    }

    api::grab::InteractionCommandResult makeCommandResult(
        const QueuedInteractionCommand& command,
        api::grab::InteractionCommandState state,
        api::grab::InteractionFailure failure)
    {
        api::grab::InteractionCommandResult result{};
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
            api::grab::CommandStage::Terminal :
            api::grab::CommandStage::Queued;
        result.failureStage = failure;
        result.acceptedFrame = currentGameFrameIndex();
        if (result.stage == api::grab::CommandStage::Terminal) {
            result.frameIndex = result.acceptedFrame;
        }
        return result;
    }

    void storeInteractionResultLocked(const api::grab::InteractionCommandResult& result)
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
                    publishCommandCompletedEvent(merged);
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
                storedResult.stage = api::grab::CommandStage::Terminal;
                if (storedResult.frameIndex == 0) {
                    storedResult.frameIndex = currentGameFrameIndex();
                }
                if (storedResult.state ==
                        api::grab::InteractionCommandState::Succeeded &&
                    storedResult.appliedFrame == 0) {
                    storedResult.appliedFrame = storedResult.frameIndex;
                }
                if (storedResult.failureStage ==
                    api::grab::InteractionFailure::None) {
                    storedResult.failureStage = storedResult.failure;
                }
            }
            slot = InteractionCommandResultSlot{
                .active = true,
                .result = storedResult,
            };
            if (interaction_command_policy::isTerminal(storedResult.state)) {
                publishCommandCompletedEvent(storedResult);
            }
            s_nextInteractionResultSlot = (index + 1) % s_interactionResults.size();
            return;
        }
    }

    void completeInteractionCommandLocked(
        const QueuedInteractionCommand& command,
        api::grab::InteractionCommandState state,
        api::grab::InteractionFailure failure)
    {
        const auto result = makeCommandResult(command, state, failure);
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
    }

    void clearInteractionCommandsForOwnerLocked(std::uint64_t ownerToken, api::grab::InteractionFailure failure)
    {
        if (ownerToken == 0) {
            return;
        }

        for (auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken) {
                completeInteractionCommandLocked(slot.command, api::grab::InteractionCommandState::Cancelled, failure);
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
        native_interaction_service::clearOwner(ownerToken);
        loose_weapon_state::clear(ownerToken);
        physical_reload_input::clearOwner(ownerToken);
        support_trigger_input::clearOwner(ownerToken);
        {
            std::scoped_lock lock(s_consumerMutex);
            if (auto* slot = findConsumerSlotLocked(ownerToken)) slot->revoked = true;
        }
        if (ownerToken == 0) {
            return;
        }
        saved_grab_offset::clearDefaultGrabPosesForOwner(ownerToken);
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
                api::grab::InteractionFailure::InvalidRequest);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                api::Hand::None,
                ::rock::api::input::SuppressionInvalidationReason::CallbackFault);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(
                    ::rock::api::input::SuppressionInvalidationReason::CallbackFault);
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
            api::Hand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
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
            LostAuthority::Owner,
            static_cast<std::uint32_t>(
                ::rock::api::input::SuppressionInvalidationReason::CallbackFault));
    }







    api::Status validateRegisteredOwner(const std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_consumerMutex);
        return validateRegisteredOwnerLocked(ownerToken);
    }


    bool generationGuardsStale(
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration)
    {
        return validateGenerationGuards(
                   worldGeneration,
                   skeletonGeneration,
                   providerGeneration) != api::Status::Ok;
    }






    template <class Output, class Query>
    api::Status queryPhysicsInteractionValue(
        const std::uint64_t ownerToken,
        Output& output,
        Query&& query,
        const bool requireAnimationThread = false)
    {
        output = {};
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (requireAnimationThread && !onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return query(*pi, output) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }









    bool hasInteractionTargetIdentity(std::uint32_t targetFormId, std::uint32_t targetBodyId)
    {
        return targetFormId != 0 || targetBodyId != kProviderInvalidBodyId;
    }

    bool isFiniteVector3(const float values[3])
    {
        return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
    }

    api::Status validateInteractionCommandProviderReady()
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return api::Status::Ok;
    }

    api::Status enqueueInteractionCommand(QueuedInteractionCommand command, std::uint64_t* outCommandId)
    {
        if (!outCommandId) {
            return api::Status::InvalidArgument;
        }
        *outCommandId = 0;

        const auto providerReadyResult = validateInteractionCommandProviderReady();
        if (providerReadyResult != api::Status::Ok) {
            return providerReadyResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(command.ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        if (command.kind == api::grab::InteractionCommandKind::ForceGrab &&
            s_forceGrabReservations.isReserved(command.forceGrab.hand)) {
            return api::Status::HandBusy;
        }

        for (auto& slot : s_interactionCommands) {
            if (!slot.active) {
                command.commandId = nextInteractionCommandId();
                if (command.kind == api::grab::InteractionCommandKind::ForceGrab &&
                    !s_forceGrabReservations.reserve(command.forceGrab.hand, command.ownerToken, command.commandId)) {
                    return api::Status::HandBusy;
                }
                slot = InteractionCommandSlot{
                    .active = true,
                    .command = command,
                };
                *outCommandId = command.commandId;
                storeInteractionResultLocked(makeCommandResult(command, api::grab::InteractionCommandState::Queued, api::grab::InteractionFailure::None));
                return api::Status::RequestQueued;
            }
        }

        return api::Status::CapacityFull;
    }























    [[nodiscard]] bool handlingRequestValuesValid(
        const api::weapon::HandlingRequest& request)
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
               inRange(request.leftFiringAimYawDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimPitchDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[0], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[1], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[2], -15.0f, 15.0f) &&
               inRange(request.equipVisualBridgeTimeoutSeconds, 0.25f, 5.0f) &&
               inRange(request.equipVisualBridgeBlendSeconds, 0.0f, 1.0f);
    }





    api::Status validateTargetQuery(std::uint64_t ownerToken,
        const api::grab::ReferenceQuery& query)
    {
        if (!ownerToken || !query.referenceFormId || query.furnitureMarkerIndex < 0) return api::Status::InvalidArgument;
        const auto permission = validateRegisteredOwner(ownerToken);
        if (permission != api::Status::Ok) return permission;
        if (!onAnimationOwnerThread()) return api::Status::WrongThread;
        const bool ready = apiIsProviderReady();
        const auto result = ready ? validateGenerationGuards(query.worldGeneration,
            query.skeletonGeneration, query.providerGeneration) : api::Status::NotReady;
        if (result != api::Status::Ok) {
            // Animation-owner thread only. Preserve the query contract and log
            // the actual rejecting authority, including during frame resets.
            static std::uint64_t lastReportedFrame = 0;
            static bool reported = false;
            const auto frame = currentGameFrameIndex();
            if (!reported || frame < lastReportedFrame || frame - lastReportedFrame >= 600) {
                reported = true;
                lastReportedFrame = frame;
                Publication published{};
                {
                    std::scoped_lock lock(s_snapshotMutex);
                    published = s_lastSnapshot;
                }
                logger::warn("ROCK reference query rejected owner={:016X} ref={:08X} result={} stage={} frame={} requested={}/{}/{} authorityAvailable={} authority={}/{}/{} published={}/{}/{} publishedFrame={}",
                    ownerToken, query.referenceFormId, static_cast<unsigned>(result),
                    ready ? "generation-guards" : "provider-readiness", frame,
                    query.worldGeneration, query.skeletonGeneration, query.providerGeneration,
                    s_generationStateAvailable.load(std::memory_order_acquire),
                    s_currentWorldGeneration.load(std::memory_order_acquire),
                    s_currentSkeletonGeneration.load(std::memory_order_acquire),
                    s_currentProviderGeneration.load(std::memory_order_acquire),
                    published.worldGeneration, published.skeletonGeneration, published.providerGeneration, published.frameIndex);
            }
        }
        return result;
    }




























    [[nodiscard]] bool validOffhandReservationRequest(
        const api::grab::HandReservationRequest& request)
    {
        return request.reservation != api::grab::HandReservationKind::Normal &&
               (request.reservation == api::grab::HandReservationKind::ReloadReserved ||
                   request.reservation == api::grab::HandReservationKind::ReloadPoseOverride) &&
               request.leaseFrames != 0;
    }

    api::Status setHandReservationLease(
        const std::uint64_t ownerToken,
        const api::grab::HandReservationRequest& request,
        const bool requireExisting)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (!validOffhandReservationRequest(request)) {
            return api::Status::InvalidArgument;
        }
        // One weapon today: only the primary weapon's offhand can be reserved.
        if (request.hand != offhandHand()) {
            return api::Status::HandUnavailable;
        }
        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        if (requireExisting &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return api::Status::TargetUnavailable;
        }
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return api::Status::OwnerConflict;
        }
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            api::grab::kMaxReservationLeaseFrames);
        s_offhandReservationSlot = OffhandReservationSlot{
            .ownerToken = ownerToken,
            .hand = request.hand,
            .reservation = request.reservation,
            .expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames),
            .worldGeneration = request.worldGeneration,
            .skeletonGeneration = request.skeletonGeneration,
            .providerGeneration = request.providerGeneration,
        };
        publishOffhandReservationLocked(s_offhandReservationSlot);
        return api::Status::Ok;
    }






    [[nodiscard]] api::Point3 toProviderPoint(
        const RE::NiPoint3& point) noexcept
    {
        return api::Point3{ point.x, point.y, point.z };
    }









    [[nodiscard]] constexpr std::uint64_t advanceSequence(
        const std::uint64_t sequence) noexcept
    {
        return sequence == UINT64_MAX ? UINT64_MAX : sequence + 1;
    }

    [[nodiscard]] bool sameHeldBodies(
        const provider::HandInteractionPublication& left,
        const provider::HandInteractionPublication& right) noexcept
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
        const provider::HandInteractionPublication& left,
        const provider::HandInteractionPublication& right) noexcept
    {
        return left.targetKind == right.targetKind &&
               left.reservedTargetIdentity ==
                   right.reservedTargetIdentity &&
               left.targetFormId == right.targetFormId &&
               left.primaryBodyId == right.primaryBodyId &&
               sameHeldBodies(left, right);
    }

    [[nodiscard]] bool handGripActive(
        const provider::HandInteractionPublication& state) noexcept
    {
        return provider_state_policy::handHolding(state);
    }

    [[nodiscard]] bool sameHandGrip(
        const provider::HandInteractionPublication& left,
        const provider::HandInteractionPublication& right) noexcept
    {
        constexpr std::uint32_t gripFlags =
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::FiringGrip) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::PartGrip) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::PartCarry) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::LooseObject) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::LooseWeapon) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::TouchGrab) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::FixedSurfaceLatch) |
            static_cast<std::uint32_t>(
                api::grab::HandInteractionFlag::GlobalSurfaceLatch);
        return handGripActive(left) == handGripActive(right) &&
               (left.flags & gripFlags) == (right.flags & gripFlags) &&
               sameHandTarget(left, right);
    }

    [[nodiscard]] bool sameHandInteractionPayload(
        const provider::HandInteractionPublication& left,
        const provider::HandInteractionPublication& right) noexcept
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
        provider::HandInteractionPublication& current,
        const provider::HandInteractionPublication& previous,
        const bool hasPrevious)
    {
        if (!hasPrevious) {
            current.stateSequence = 1;
            current.targetSequence =
                sameHandTarget(current, provider::HandInteractionPublication{}) ?
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
        if (!(current.flags & static_cast<std::uint32_t>(api::grab::HandInteractionFlag::Valid))) {
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
        const Publication& left,
        const Publication& right) noexcept
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
        const Publication& left,
        const Publication& right) noexcept
    {
        if (left.weaponFormId != right.weaponFormId ||
            left.weapon.weaponGenerationKey != right.weapon.weaponGenerationKey ||
            left.weapon.bodyCount != right.weapon.bodyCount) {
            return false;
        }
        for (std::uint32_t index = 0; index < left.weapon.bodyCount; ++index) {
            if (left.weapon.bodyIds[index] != right.weapon.bodyIds[index]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool sameEquippedWeaponPayload(
        const api::weapon::WeaponState& left,
        const api::weapon::WeaponState& right) noexcept
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
               left.sample.worldGeneration == right.sample.worldGeneration &&
               left.sample.skeletonGeneration == right.sample.skeletonGeneration &&
               left.sample.providerGeneration == right.sample.providerGeneration;
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

}

namespace rock::provider
{
    using namespace runtime;
    bool isPowerArmorGrabOwnerRegistered(const std::uint64_t ownerToken)
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
        physical_reload_input::beginFrame(frameIndex);
        support_trigger_input::beginFrame(frameIndex);
    }

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        if(!pi) { physical_reload_input::reset(); support_trigger_input::reset(); }
        s_physicsInteraction.publish(pi);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        drainDeferredRevocations();
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::ProviderFrameDispatch);
        Publication snapshot{};
        snapshot.frameIndex = s_frameClock.publishFrame();
        pi.fillProviderFrameSnapshot(snapshot);
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
        snapshot.handReservation = currentHandReservation(snapshot.offhandHand);

        // Source keys resolve while this frame owns the live catalog; task
        // readers copy the published values without touching scene nodes.
        std::array<api::weaponparts::GripState, 2> partGripStates{};
        pi.fillProviderWeaponPartGripStates(partGripStates);

        std::array<provider::HandInteractionPublication, 2>
            handInteractionStates{};
        pi.fillProviderHandInteractionStates(handInteractionStates);
        for (auto& state : handInteractionStates) {
            state.frameIndex = snapshot.frameIndex;
        }

        api::weapon::WeaponState equippedWeaponState{};
        (void)pi.queryProviderWeaponState(equippedWeaponState);
        equippedWeaponState.sample.frameIndex = snapshot.frameIndex;

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
                    (currentHand.flags & static_cast<std::uint32_t>(api::grab::HandInteractionFlag::Valid)) &&
                    handGripActive(previousHand) &&
                    !handGripActive(currentHand) &&
                    currentHand.phase ==
                        api::grab::HandInteractionPhase::Idle) {
                    currentHand.phase =
                        api::grab::HandInteractionPhase::Releasing;
                    currentHand.targetKind = previousHand.targetKind;
                    currentHand.reservedTargetIdentity =
                        previousHand.reservedTargetIdentity;
                    currentHand.targetFormId = previousHand.targetFormId;
                    currentHand.primaryBodyId = previousHand.primaryBodyId;
                    constexpr std::uint32_t touchGrabClassificationFlags =
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::TouchGrab) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
                                FixedSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
                                GlobalSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
                                SurfaceAnchorValid) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
                                MeshSurfaceAnchor) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
                                MeshFingerPose) |
                        static_cast<std::uint32_t>(
                            api::grab::HandInteractionFlag::
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
                snapshot.handReservation !=
                    previousSnapshot.handReservation;
            const bool stateChanged = lifecycleChanged || handChanged[0] || handChanged[1] ||
                weaponChanged || transitionChanged || collisionChanged || handRolesChanged;
            snapshot.stateSequence = !hadPrevious ?
                1 :
                (stateChanged ?
                        advanceSequence(previousSnapshot.stateSequence) :
                        previousSnapshot.stateSequence);

            equippedWeaponState.sample.publicationSequence = snapshot.stateSequence;
            equippedWeaponState.sample.collisionGeneration = snapshot.collisionGeneration;
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

        // Revalidate against this publication before consumers run. The earlier
        // game-clock expiry check only has the previous weapon snapshot.
        physical_reload_input::beginFrame(snapshot.frameIndex);
        support_trigger_input::beginFrame(snapshot.frameIndex);

        if (hadPrevious && lifecycleChanged) {
            auto event = stampedEvent<api::core::Event>(
                static_cast<std::uint32_t>(api::core::EventKind::Lifecycle));
            event.reason = static_cast<std::uint32_t>(snapshot.lastLifecycleReason);
            event.lifecycleFlags = snapshot.lifecycleFlags;
            event.providerReady = snapshot.providerReady;
            events::publish(event);
        }
        if (hadPrevious) {
            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                if (!handChanged[index]) {
                    continue;
                }
                const auto& state = handInteractionStates[index];
                auto event = stampedEvent<api::grab::Event>(
                    static_cast<std::uint32_t>(api::grab::EventKind::HandStateChanged));
                event.hand = state.hand;
                event.formId = state.targetFormId;
                event.phase = static_cast<std::uint32_t>(state.phase);
                event.subjectSequence = state.stateSequence;
                event.primaryBodyId = state.primaryBodyId;
                event.flags = state.flags;
                events::publish(event);
            }
        }
        if (equippedTerminalChanged) {
            // Slot 0: ROCK keeps one weapon's transition state.
            auto event = stampedEvent<api::weapon::Event>(
                static_cast<std::uint32_t>(api::weapon::EventKind::TransitionCompleted));
            event.weaponGenerationKey =
                equippedWeaponState.terminalWeaponFormId == equippedWeaponState.weaponFormId ?
                    equippedWeaponState.weaponGenerationKey : 0;
            event.formId = equippedWeaponState.terminalWeaponFormId;
            event.terminalResult = static_cast<std::uint32_t>(
                equippedWeaponState.terminalResult);
            event.transitionSequence = equippedWeaponState.terminalSequence;
            event.source = static_cast<std::uint32_t>(
                equippedWeaponState.terminalSource);
            event.flags = equippedWeaponState.flags;
            events::publish(event);
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

    void dispatchAnimationPhaseCallbacks(
        const ::rock::api::animation::AnimationPhase phase,
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

        const bool previousReadbackPhase = std::exchange(s_presentedReadbackPhase, phase == ::rock::api::animation::AnimationPhase::Presented);
        const auto restoreReadbackPhase = F4SE::stl::scope_exit([previousReadbackPhase] {
            s_presentedReadbackPhase = previousReadbackPhase;
        });
        drainDeferredRevocations();
        const auto phaseFrameIndex = s_frameClock.beginPhase(
            static_cast<api::animation::AnimationPhase>(phase), timing.sequence);
        s_activeAnimationPhaseFrameIndex.store(phaseFrameIndex, std::memory_order_release);
        const bool startingAnimationFrame = phase == ::rock::api::animation::AnimationPhase::BeforeRock;

        if (startingAnimationFrame) {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            pruneHandVisualAuthorityLocked(currentProviderFrameIndex());
        }

        api::animation::AnimationPhaseContext context{};
        context.phase = phase;
        context.frameIndex = phaseFrameIndex;
        /*
         * Truthful timing publication: the central game clock already
         * sanitized the delta, so an unmeasurable frame publishes zero
         * elapsed time instead of a fabricated nominal-rate value.
         */
        context.deltaSeconds = timing.valid ? timing.deltaSeconds : 0.0f;

        const auto& runtime = runtime_state::currentFrame();
        context.flags |= static_cast<std::uint32_t>(
            ::rock::api::animation::AnimationPhaseContextFlag::RockEnabled);
        if (apiIsProviderReady()) {
            context.flags |= static_cast<std::uint32_t>(
                ::rock::api::animation::AnimationPhaseContextFlag::ProviderReady);
        }
        if (runtime.localSkeletonReady) {
            context.flags |= static_cast<std::uint32_t>(
                ::rock::api::animation::AnimationPhaseContextFlag::SkeletonReady);
        }
        if (runtime.localMenuBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                ::rock::api::animation::AnimationPhaseContextFlag::MenuBlocking);
        }
        if (runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                ::rock::api::animation::AnimationPhaseContextFlag::ConfigBlocking);
        }
        if (!s_presentedReadbackPhase && runtime.visualAuthorityAvailable && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking &&
            !runtime.compatibilityConfigBlocking) {
            context.flags |= static_cast<std::uint32_t>(
                ::rock::api::animation::AnimationPhaseContextFlag::VisualWritesAllowed);
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

        if (phase == ::rock::api::animation::AnimationPhase::Complete || phase == ::rock::api::animation::AnimationPhase::Presented) {
            s_activeAnimationPhaseFrameIndex.store(0, std::memory_order_release);
        }
    }

    void clearProviderStateForProviderLoss()
    {
        {
            std::scoped_lock lock(s_snapshotMutex);
            // Preserve lifecycle/terminal history, invalidate all live reads.
            s_lastSnapshot.providerReady = 0;
            s_lastSnapshot.lifecycleFlags &= ~(
                static_cast<std::uint32_t>(api::core::LifecycleFlag::ProviderReady) |
                static_cast<std::uint32_t>(api::core::LifecycleFlag::GeneratedBodiesValid) |
                static_cast<std::uint32_t>(api::core::LifecycleFlag::PhysicsWriteAllowed) |
                static_cast<std::uint32_t>(api::core::LifecycleFlag::VisualWriteAllowed));
            s_lastSnapshot.flags &= ~(
                static_cast<std::uint32_t>(::rock::provider::PublicationFlag::RightHandTransformValid) |
                static_cast<std::uint32_t>(::rock::provider::PublicationFlag::LeftHandTransformValid));
            for (auto& state : s_lastHandInteractionStates) state.flags = 0;
            s_lastPartGripStates = {};
            s_lastEquippedWeaponState.flags = 0;
        }
        std::array<std::uint64_t, api::core::kMaxOwners>
            suppressionOwners{};
        std::array<std::uint64_t, api::core::kMaxOwners>
            targetOwners{};
        std::array<std::uint64_t, api::core::kMaxOwners>
            driveOwners{};
        std::array<std::uint64_t, api::core::kMaxOwners>
            nativeAnimationOwners{};
        std::array<std::uint64_t, api::core::kMaxOwners>
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
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearAll();
        }
        clearInteractionCommandsForProviderLoss(api::grab::InteractionFailure::ProviderNotReady);
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
                    ::rock::api::input::SuppressionInvalidationReason::ProviderLost;
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
            s_weaponPartGrabPoints = {};
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
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            clearOffhandReservationLocked(
                ::rock::api::input::SuppressionInvalidationReason::ProviderLost);
        }

        constexpr auto providerLost = static_cast<std::uint32_t>(
            ::rock::api::input::SuppressionInvalidationReason::ProviderLost);
        for (std::uint32_t index = 0;
             index < suppressionOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                suppressionOwners[index],
                LostAuthority::HandInputSuppression,
                providerLost);
        }
        for (std::uint32_t index = 0; index < targetOwnerCount; ++index) {
            publishAuthorityLostEvent(
                targetOwners[index],
                LostAuthority::WeaponPartTargets,
                providerLost);
        }
        for (std::uint32_t index = 0; index < driveOwnerCount; ++index) {
            publishAuthorityLostEvent(
                driveOwners[index],
                LostAuthority::WeaponPartDrive,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < nativeAnimationOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                nativeAnimationOwners[index],
                LostAuthority::NativeAnimation,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < handVisualOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                handVisualOwners[index],
                LostAuthority::HandVisual,
                providerLost);
        }
        if (nativeRuntimeOwner != 0) {
            publishAuthorityLostEvent(
                nativeRuntimeOwner,
                LostAuthority::NativeAnimationRuntime,
                providerLost);
        }
        if (equippedHandlingOwner != 0) {
            publishAuthorityLostEvent(
                equippedHandlingOwner,
                LostAuthority::WeaponHandling,
                providerLost);
        }
        s_generationStateAvailable.store(false, std::memory_order_release);
        s_currentWorldGeneration.store(0, std::memory_order_release);
        s_currentSkeletonGeneration.store(0, std::memory_order_release);
        s_currentProviderGeneration.store(0, std::memory_order_release);
    }

    bool dequeueInteractionCommand(QueuedInteractionCommand& outCommand)
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
                        api::grab::CommandStage::Committed;
                    resultSlot.result.committedFrame = currentGameFrameIndex();
                    break;
                }
            }
            return true;
        }
        return false;
    }

    bool isInteractionCommandActive(std::uint64_t ownerToken, std::uint64_t commandId)
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

    bool completeInteractionCommand(const api::grab::InteractionCommandResult& result)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        if (result.kind == api::grab::InteractionCommandKind::ForceGrab &&
            !s_forceGrabReservations.matches(result.ownerToken, result.commandId)) {
            return false;
        }
        storeInteractionResultLocked(result);
        if (interaction_command_policy::isTerminal(result.state)) {
            s_forceGrabReservations.release(result.ownerToken, result.commandId);
        }
        return true;
    }

    void clearInteractionCommandsForProviderLoss(api::grab::InteractionFailure failure)
    {
        std::scoped_lock lock(s_interactionCommandMutex);
        /*
         * Results remain Queued after dequeue while PhysicsInteraction owns the
         * deferred commit. Cancel those as well as slots still in the bounded
         * queue so consumers never observe an immortal command.
         */
        for (auto& slot : s_interactionResults) {
            if (slot.active && slot.result.state == api::grab::InteractionCommandState::Queued) {
                auto terminal = slot.result;
                terminal.state =
                    api::grab::InteractionCommandState::Cancelled;
                terminal.failure = failure;
                terminal.failureStage = failure;
                terminal.stage = api::grab::CommandStage::Terminal;
                terminal.frameIndex = currentGameFrameIndex();
                storeInteractionResultLocked(terminal);
            }
        }
        s_interactionCommands = {};
        s_forceGrabReservations.clear();
    }

    api::grab::HandReservationKind currentHandReservation(const api::Hand hand)
    {
        if (static_cast<api::Hand>(s_offhandReservationHand.load(std::memory_order_acquire)) != hand) {
            return api::grab::HandReservationKind::Normal;
        }
        const auto expiry =
            s_offhandReservationExpiry.load(std::memory_order_acquire);
        if (expiry != 0 && !provider_lease_policy::isActive(
                currentProviderFrameIndex(),
                expiry)) {
            return api::grab::HandReservationKind::Normal;
        }
        return static_cast<api::grab::HandReservationKind>(s_offhandReservation.load(std::memory_order_acquire));
    }

    void setEquippedWeaponFiringHandIsLeft(const bool isLeft)
    {
        s_equippedWeaponFiringHandIsLeft.store(isLeft, std::memory_order_release);
    }

    bool getHandlingAuthorityRequest(
        api::weapon::HandlingRequest& outRequest)
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

    bool ownsHandlingAuthority(
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
               (s_equippedWeaponHandlingAuthority.request.features &
                   requiredFlags) == requiredFlags;
    }

    void markInteractionCommandStage(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId,
        const api::grab::CommandStage stage)
    {
        if (ownerToken == 0 || commandId == 0 ||
            stage == api::grab::CommandStage::Unknown ||
            stage == api::grab::CommandStage::Terminal) {
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
            if (stage == api::grab::CommandStage::Committed &&
                slot.result.committedFrame == 0) {
                slot.result.committedFrame = frameIndex;
            }
            if (stage == api::grab::CommandStage::Applied &&
                slot.result.appliedFrame == 0) {
                slot.result.appliedFrame = frameIndex;
            }
            return;
        }
    }

    std::uint32_t currentHandInputSuppressionFlags(api::Hand hand)
    {
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t flags = input_remap_runtime::uiInputSuppressionFlags(hand == api::Hand::Left);
        std::scoped_lock lock(s_handInputSuppressionMutex);
        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.hand == hand) {
                flags |= effectiveHandInputSuppressionFlags(hand, slot.flags, slot.leftChord, slot.rightChord);
            }
        }
        return flags;
    }

    std::uint32_t currentNativeAnimationAuthorityFlags()
    {
        return s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
    }

    void refreshNativeAnimationAuthorityLeases()
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

    bool resolveWeaponPartTarget(
        const provider::PartQuery& query,
        weapon_part_runtime::Resolution& outResolution)
    {
        std::array<weapon_part_runtime::Target, api::weaponparts::kMaxTargets> runtimeTargets{};
        const auto targetCount = copyWeaponPartTargets(runtimeTargets);

        const weapon_part_runtime::Contact contact{
            .weaponGenerationKey = query.weaponGenerationKey,
            .bodyId = query.bodyId,
            .sourceRoot = query.sourceRoot,
            .sourceName = std::string_view(query.sourceName, boundedStringLength(query.sourceName, std::size(query.sourceName))),
            .partKind = static_cast<WeaponPartKind>(query.partKind),
            .reloadRole = static_cast<WeaponReloadRole>(query.reloadRole),
            .supportRole = static_cast<WeaponSupportGripRole>(query.supportRole),
            .socketRole = static_cast<WeaponSocketRole>(query.socketRole),
            .actionRole = static_cast<WeaponActionRole>(query.actionRole),
        };
        outResolution = weapon_part_runtime::resolveTarget(
            std::span(runtimeTargets).first(targetCount), contact);
        // Non-exclusive targets can match without raising whitelistActive, so
        // a resolution is meaningful whenever either signal is set.
        return outResolution.whitelistActive || outResolution.matched;
    }

    std::uint32_t copyWeaponPartDrives(
        PartDrive* outTargets,
        std::uint32_t maxTargets,
        std::uint64_t* outOwnerTokens,bool pruneExpired)
    {
        if (!outTargets || maxTargets == 0) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t copied = 0;
        std::scoped_lock lock(s_weaponPartMutex);
        if(pruneExpired)pruneExpiredWeaponPartDrivesLocked(frameIndex);
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active || !provider_lease_policy::isActive(frameIndex,slot.expiresAfterFrame)) {
                continue;
            }
            if (copied >= maxTargets) {
                break;
            }
            outTargets[copied] = slot.target;
            outTargets[copied].sourceName[weapon_part_runtime::kMaxSourceName - 1] = '\0';
            if (outOwnerTokens) {
                outOwnerTokens[copied] = slot.ownerToken;
            }
            ++copied;
        }
        return copied;
    }

    bool resolveTouchGrabTarget(
        const std::uint32_t bodyId,
        const std::uint32_t collisionLayer,
        const TouchGrabMotionClass motionClass,
        const api::Hand hand,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatch& outMatch)
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

    bool currentTouchGrabTarget(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatch& outMatch)
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

    bool publishTouchGrabState(std::uint64_t ownerToken,std::uint64_t scopeToken,const ::rock::api::touch::TouchGrabState& state) {
        bool changed=false; bool accepted=false;
        const auto frame=currentGameFrameIndex();
        {
            std::scoped_lock lock(s_touchGrabMutex);
            accepted=s_touchGrabTargets.publishState(ownerToken,scopeToken,state,frame,&changed);
        }
        if (accepted && changed) events::publishTouch(ownerToken,scopeToken,state,frame);
        return accepted;
    }

    void acknowledgeTouchGrabYield(
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
        if (!owner || id<1 || id>kInterfaceSlotCount) return Status::InvalidArgument;
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
        if (id<1 || id>kInterfaceSlotCount) return Status::UnknownInterface;
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
    rock::api::Sample sample() {
        std::scoped_lock lock(s_snapshotMutex);
        return s_lastSnapshot.sample();
    }
    void revoke(std::uint64_t owner) { clearOwnerStateAfterCallbackFault(owner); }
    bool onOwnerThread() noexcept { return onAnimationOwnerThread(); }
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
