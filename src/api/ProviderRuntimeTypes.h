#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <ROCK/Core.h>
#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Input.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>

// Private runtime representation; never installed or returned across the DLL boundary.
namespace rock::provider {
    inline constexpr std::uint32_t ROCK_PROVIDER_API_VERSION = 1;
    inline constexpr std::uint32_t ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE = 256;
    inline constexpr std::uint32_t ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE = 112;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_BODIES = 8;
    // Evidence describes the complete classified weapon-part catalog and is
    // independent from the compact body array embedded in frame snapshots.
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1 = api::weaponparts::kMaxEvidenceDetails;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_CONSUMERS_V1 = api::core::kMaxOwners;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 = api::grab::kMaxQueuedCommands;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1 = api::grab::kMaxCommandResults;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1 = api::input::kMaxSuppressions;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1 = api::input::kMaxSuppressionLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1 = api::weaponparts::kMaxTargets;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1 = api::weaponparts::kMaxDrives;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1 = api::weaponparts::kMaxDriveLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1 = api::weapon::kMaxEmitters;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1 = api::animation::kMaxNativeAuthorityLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1 = api::animation::kMaxPhaseCallbacks;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1 = api::animation::kMaxHandVisualLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1 = api::animation::kMaxRuntimePublicationLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1 = api::weapon::kMaxHandlingLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1 = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1 = api::weaponparts::kMaxDrives;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1 = api::grab::kMaxReservationLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 = api::touch::kMaxTargets;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1 = api::touch::kMaxScopes;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1 = api::touch::kMaxLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1 = api::collision::kMaxRaycastsPerOwnerPerFrame;
    inline constexpr float ROCK_PROVIDER_MAX_WORLD_RAYCAST_DISTANCE_GAME_V1 = api::collision::kMaxRaycastDistanceGame;
    inline constexpr std::uint16_t ROCK_PROVIDER_ALL_FINGER_LOCAL_TRANSFORMS_V1 = 0x7FFFu;

    /*
     * Public frameIndex/acceptedFrame/committedFrame/appliedFrame and animation
     * phase frames use the same monotonic game-frame clock, including across
     * provider recreation. Lifecycle callbacks can share a frame index; use
     * state/event sequences to distinguish changes within that frame.
     * Lease fences use the most recent provider snapshot frame and retire at
     * the next publication boundary, after ROCK consumes that update's drives.
     * Every V1 lease uses the same exclusive expiry fence. A publication made
     * at frame F with leaseFrames N is active while currentFrame < F + N and
     * expires at F + N. Zero is invalid; values above the published family
     * limit are clamped. Refresh replaces the prior expiry and generations.
     */













    enum class RockProviderConsumerCapabilityV1 : std::uint32_t
    {
        None = 0,
        FrameSnapshots = 1u << 0,
        OffhandReservation = 1u << 3,
        InteractionCommands = 1u << 4,
        HandInputSuppression = 1u << 5,
        WeaponPartInteraction = 1u << 6,
        NativeAnimationAuthority = 1u << 7,
        AnimationPhases = 1u << 8,
        EquippedWeaponGripState = 1u << 9,
        HandVisualAuthority = 1u << 10,
        NativeAnimationRuntimeProvider = 1u << 11,
        EquippedWeaponHandlingAuthority = 1u << 12,
        // 1u << 13 retired in ROCK 1.0 (DebugOverlayPublication); never reuse.
        DebugOverlayPublication = 1u << 13,
        ProviderEvents = 1u << 14,
        HandInteractionState = 1u << 15,
        WeaponPartObservability = 1u << 17,
        WeaponComposition = 1u << 18,
        PoseReadback = 1u << 19,
        SemanticHandContacts = 1u << 20,
        PlayerColliderDescriptors = 1u << 21,
        ScopeSightState = 1u << 22,
        InputObservability = 1u << 23,
        TouchGrabTargets = 1u << 24,
        WorldRaycasts = 1u << 25,
        // 1u << 26 retired in ROCK 1.0 (ColliderVisualizationOverride); never reuse.
        ColliderVisualizationOverride = 1u << 26,
        PlayerController = 1u << 27,
        TargetDetails = 1u << 28,
        PowerArmor = 1u << 29,
    };



    /*
     * Selective Bethesda animation authority. ROCK preserves the authored
     * relationship between both arms, hands, and weapon, then rigidly anchors
     * both skeleton trees to the visible first-person weapon/controller world
     * frame while retaining the weapon's authored motion since lease acquisition.
     * Arms covers only the two
     * collarbone-to-hand chains; Hands adds the hand roots and finger/thumb
     * descendants; Weapon covers only Weapon and WeaponLeft. The character
     * root, COM, torso, head, and legs remain owned by the live VR body.
     */




    enum class RockProviderEquippedWeaponGripStateFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        TwoHandGripActive = 1u << 1,
        FiringHandLeft = 1u << 2,
        WeaponTransformOwned = 1u << 3,
        WeaponWorldValid = 1u << 4,
        RightHandInWeaponValid = 1u << 5,
        LeftHandInWeaponValid = 1u << 6,
        MuzzleWorldValid = 1u << 7,
    };

    /*
     * Owner-bound policy supplied by a standalone equipped-weapon addon.
     * ROCK remains the low-level hand, weapon-node, physics, input-routing,
     * and inventory executor. The consumer selects which optional behaviors
     * are active and supplies their bounded tuning without changing physical
     * controller identity or Fallout 4 VR's native handedness setting.
     */
    enum class RockProviderEquippedWeaponHandlingFlagV1 : std::uint32_t
    {
        None = 0,
        FiringGripOwnership = 1u << 0,
        PrimaryDetach = 1u << 1,
        AmbidextrousHandoff = 1u << 2,
        GripZoneEquip = 1u << 3,
        GripZoneHoverHaptics = 1u << 4,
        // Replaces ROCK's configured radius while this authority lease lives;
        // the core VisualOnlySupport behavior itself remains always enabled.
        FiringGripProximitySupport = 1u << 5,
        // Retained in V1 for source/ABI compatibility. Equipped-weapon
        // shoulder sheath/retrieval is configured and owned by ROCK; this
        // request bit is accepted for older consumers but has no behavior.
        EquippedWeaponShoulderStash = 1u << 6,
        // Retained in V1 for source/ABI compatibility. The Pip-Boy hand-equip
        // mode was removed; the flag is accepted and has no behavior.
        PipboyTriggerHandEquip = 1u << 7,
        // Retained in V1 for source/ABI compatibility. The visual bridge and
        // native attach recovery are now unconditional ROCK correctness
        // services; addons may still supply the bounded blend/timeout tuning.
        EquipVisualBridge = 1u << 8,
    };

    enum class RockProviderEquippedWeaponHandlingRuntimeFlagV1 : std::uint32_t
    {
        None = 0,
        AuthorityActive = 1u << 0,
        FixedHandLeft = 1u << 1,
        FiringHandLeft = 1u << 2,
        LeftFiringInfrastructureAvailable = 1u << 3,
        ManualOwnershipActive = 1u << 4,
        PartCarryActive = 1u << 5,
        // Effective firing-grip occupancy, including native right-hand carry
        // after inventory/holster equip without an explicit ROCK grip.
        // Clear during PartCarry and when WeaponPresent is clear.
        FiringGripOccupied = 1u << 6,
        WeaponPresent = 1u << 7,
    };


    enum class RockProviderForceGrabFlagV1 : std::uint32_t
    {
        None = 0,
        UsePreferredGrabPointGame = 1u << 0,
        // targetFormId is an owned ALCH/throwable base form, not a world REFR.
        // Requires hand=None and no other flags. Transfers exactly one item to
        // a free hand, without using or equipping it. Throwables prefer ROCK's
        // configured default hand; other inventory items prefer the right.
        // Query InventoryForceGrab support first. Failure before the drop leaves
        // inventory unchanged; failed attachment attempts return the exact drop.
        FromPlayerInventory = 1u << 1,
    };


    enum class RockProviderWeaponPartTargetFlagV1 : std::uint32_t
    {
        None = 0,
        MatchBodyId = 1u << 0,
        MatchSourceRoot = 1u << 1,
        MatchSourceName = 1u << 2,
        MatchPartKind = 1u << 3,
        MatchReloadRole = 1u << 4,
        MatchSupportRole = 1u << 5,
        MatchSocketRole = 1u << 6,
        MatchActionRole = 1u << 7,
        /*
         * Non-exclusive target (feature bit WeaponPartTargetNonExclusive):
         * grants its grab mode on match without activating whitelist gating,
         * so unmatched parts keep their normal grip behavior. Omit the flag
         * for reload-session semantics where every unmatched part grip is
         * rejected while the whitelist is active.
         */
        NonExclusive = 1u << 8,
    };


    enum class RockProviderEventKindV1 : std::uint32_t
    {
        Unknown = 0,
        LifecycleChanged = 1,
        EquippedWeaponTransitionTerminal = 2,
        AuthorityLost = 3,
        InteractionCommandTerminal = 4,
        GrabStateChanged = 5,
    };

    enum class RockProviderAuthorityKindV1 : std::uint32_t
    {
        Unknown = 0,
        HandInputSuppression = 1,
        WeaponPartDrive = 2,
        NativeAnimation = 3,
        NativeAnimationRuntime = 4,
        EquippedWeaponHandling = 5,
        OffhandReservation = 6,
        HandVisual = 7,
        DebugOverlay = 8, // Retired in ROCK 1.0; never emitted, never reused.
        WeaponPartTargets = 9,
        ColliderVisualization = 10, // Retired in ROCK 1.0; never emitted, never reused.
    };











    enum class RockProviderFrameEnrichmentFlagV1 : std::uint32_t
    {
        None = 0,
        DeltaSecondsValid = 1u << 0,
        HmdTransformValid = 1u << 1,
        HmdForwardValid = 1u << 2,
        CoherentHandRoles = 1u << 3,
        StateSequenceValid = 1u << 4,
        CollisionGenerationValid = 1u << 5,
        EquippedTransitionSequenceValid = 1u << 6,
        RightHandTransformValid = 1u << 7,
        LeftHandTransformValid = 1u << 8,
    };

    enum class RockProviderFrameStateChangeFlagV1 : std::uint32_t
    {
        None = 0,
        Lifecycle = 1u << 0,
        RightHand = 1u << 1,
        LeftHand = 1u << 2,
        Weapon = 1u << 3,
        EquippedTransition = 1u << 4,
        Collision = 1u << 5,
        HandRoles = 1u << 6,
    };

    [[nodiscard]] inline constexpr bool hasLifecycleFlag(std::uint32_t flags, api::core::LifecycleFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasConsumerCapabilityV1(std::uint32_t capabilities, RockProviderConsumerCapabilityV1 capability)
    {
        return (capabilities & static_cast<std::uint32_t>(capability)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasHandInputSuppressionFlagV1(
        std::uint32_t flags,
        ::rock::api::input::HandInputSuppressionFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasTouchGrabTargetFlagV1(
        std::uint32_t flags,
        ::rock::api::touch::TouchGrabTargetFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasEquippedWeaponHandlingFlagV1(
        std::uint32_t flags,
        RockProviderEquippedWeaponHandlingFlagV1 flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }


    struct RockProviderConsumerRegistrationV1
    {
        std::uint32_t size{ sizeof(RockProviderConsumerRegistrationV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        char modName[64]{};
        std::uint32_t requestedCapabilities{ 0 };
        std::uint32_t reserved[7]{};
    };

    struct RockProviderConsumerHandleV1
    {
        std::uint32_t size{ sizeof(RockProviderConsumerHandleV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t grantedCapabilities{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };


    /*
     * Extensible limits surface. Callers set size to their local structure
     * size; ROCK prefix-copies the supported bytes and returns the copied size.
     */

    /*
     * Pointer-sized fields retained by the original V1 prefix are non-owning
     * identity witnesses, never ownership or mutation authority. They may be
     * compared only on ROCK's game-thread callback/query frame while the
     * accompanying frame and generation identities still match, and must not
     * be retained or dereferenced by a consumer. Command targetRefr inputs are
     * ABI-retained but ignored; use targetFormId and/or targetBodyId. Command
     * result targetRefr is zero. New V1 structures use value identity instead.
     */

    struct RockProviderForceGrabRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderForceGrabRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float maxDistanceGame{ 0.0f };
        float preferredGrabPointGame[3]{};
        std::uint32_t reserved[5]{};
    };

    struct RockProviderForceReleaseRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderForceReleaseRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
        std::uint32_t reserved[1]{};
    };

    struct RockProviderThrownDropRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderThrownDropRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved0{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
        std::uint32_t reserved[6]{};
    };

    struct RockProviderInteractionCommandResultV1
    {
        std::uint32_t size{ sizeof(RockProviderInteractionCommandResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        api::grab::InteractionCommandKind kind{ api::grab::InteractionCommandKind::Unknown };
        api::grab::InteractionCommandState state{ api::grab::InteractionCommandState::Unknown };
        api::grab::InteractionFailure failure{ api::grab::InteractionFailure::None };
        api::Hand hand{ api::Hand::None };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        api::grab::CommandStage stage{ api::grab::CommandStage::Unknown };
        api::grab::InteractionFailure failureStage{ api::grab::InteractionFailure::None };
        std::uint64_t acceptedFrame{ 0 };
        std::uint64_t committedFrame{ 0 };
        std::uint64_t appliedFrame{ 0 };
        std::uint32_t reserved{ 0 };
    };




    struct RockProviderWeaponPartTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        api::weaponparts::GrabMode grabMode{ api::weaponparts::GrabMode::None };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[8]{};
    };

    struct RockProviderWeaponPartDriveTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartDriveTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        api::weaponparts::DriveSpace driveSpace{ api::weaponparts::DriveSpace::WeaponRootLocal };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t leaseFrames{ 1 };
        api::Transform targetTransform{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[7]{};
    };

    /*
     * Per-hand grip report: which weapon part (if any) the hand is attached
     * to this frame. Polled; gripSequence increases on every fresh capture so
     * consumers detect re-grabs without frame callbacks. sourceRoot is a
     * non-owning engine pointer valid only while weaponGenerationKey matches
     * the frame snapshot. handPartLocal is the hand frame captured at grip
     * start in handPartLocalSpace; composing it with the part's current world
     * transform yields the glued hand target, so a consumer driving the part
     * via setWeaponPartDriveTargetsV1 can also derive controller-to-part
     * displacement from it.
     */
    struct RockProviderWeaponPartGripStateV1
    {
        // Captured ROCK grip details only. Native carry can occupy the hand
        // while active is zero; use getHandInteractionStateV1 for occupancy.
        std::uint32_t size{ sizeof(RockProviderWeaponPartGripStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::weaponparts::GripKind gripKind{ api::weaponparts::GripKind::None };
        std::uint32_t active{ 0 };
        std::uint32_t attachOnly{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t providerOwnerToken{ 0 };
        std::uint32_t providerGroupId{ 0 };
        std::uint32_t providerGrabMode{ 0 };
        std::uint32_t hasHandPartLocal{ 0 };
        api::weaponparts::GripLocalSpace handPartLocalSpace{ api::weaponparts::GripLocalSpace::WeaponRootLocal };
        api::Transform handPartLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        // Record-authored identity of the gripped part; see the evidence
        // detail struct for field semantics (WeaponPartRecordIdentity bit).
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        // Exact ROCK grip provenance. Nonzero only when the active support
        // grip was acquired from the current authored support-pose capture.
        std::uint32_t authoredSupportGrip{ 0 };
        std::uint32_t reserved[5]{};
    };

    /*
     * bhkWorld, hknpWorld, and weaponNode are legacy V1 witnesses governed by
     * the pointer rule above. All other appended enrichment is copied value
     * state and remains interpretable after the callback returns.
     */
    struct RockProviderFrameSnapshot
    {
        std::uint32_t size{ sizeof(RockProviderFrameSnapshot) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uintptr_t bhkWorld{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        std::uint32_t frikSkeletonReady{ 0 };
        std::uint32_t menuBlocking{ 0 };
        std::uint32_t configBlocking{ 0 };
        std::uint32_t providerReady{ 0 };
        std::uintptr_t weaponNode{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t weaponBodyCount{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        api::Transform rightHandTransform{};
        api::Transform leftHandTransform{};
        std::uint32_t rightHandBodyId{ 0x7FFF'FFFF };
        std::uint32_t leftHandBodyId{ 0x7FFF'FFFF };
        std::uint32_t weaponBodyIds[ROCK_PROVIDER_MAX_WEAPON_BODIES]{};
        std::uint32_t rightHandState{ 0 };
        std::uint32_t leftHandState{ 0 };
        api::grab::HandReservationKind offhandReservation{ api::grab::HandReservationKind::Normal };
        float gameToHavokScale{ 0.0f };
        float havokToGameScale{ 0.0f };
        std::uint32_t physicsScaleRevision{ 0 };
        std::uint32_t lifecycleFlags{ 0 };
        api::core::LifecycleReason lastLifecycleReason{ api::core::LifecycleReason::None };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t enrichmentFlags{ 0 };
        api::Transform hmdTransform{};
        float hmdForwardWorld[3]{};
        api::Hand primaryHand{ api::Hand::Right };
        api::Hand offhandHand{ api::Hand::Left };
        std::uint64_t stateSequence{ 0 };
        std::uint32_t stateChangeMask{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint64_t equippedWeaponTransitionSequence{ 0 };
    };


    struct RockProviderWeaponContactQuery
    {
        std::uint32_t size{ sizeof(RockProviderWeaponContactQuery) };
        float pointGame[3]{};
        float radiusGame{ 0.0f };
        std::uint32_t flags{ 0 };
        std::uint32_t reserved[2]{};
    };

    struct RockProviderWeaponContactResult
    {
        std::uint32_t size{ sizeof(RockProviderWeaponContactResult) };
        std::uint32_t valid{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t interactionRoot{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float probeDistanceGame{ 0.0f };
        std::uint32_t reserved{ 0 };
    };

    /*
     * Weapon size class plus the raw keyword bitmask it was (or wasn't) derived
     * from. Consumers that only need the coarse handling bucket can read
     * sizeClass directly; consumers that need finer distinctions (e.g. a future
     * reload/scope mod picking a shotgun- or minigun-specific behavior) can
     * inspect keywordFlags (api::weapon::KeywordFlag). When no conclusive
     * authored signal exists, valid is zero and sizeClass is not authoritative.
     */
    struct RockProviderWeaponClassificationV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponClassificationV1) };
        std::uint32_t valid{ 0 };
        std::uint64_t keywordFlags{ 0 };
        api::weapon::SizeClass sizeClass{ api::weapon::SizeClass::Rifle };
        api::weapon::ClassificationSource source{ api::weapon::ClassificationSource::None };
        std::uint32_t formId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float confidence{ 0.0f };
        std::uint32_t provenanceFlags{ 0 };
    };


    /*
     * Frame-local value snapshot of FO4VR's native player character controller.
     * CheckPenetration is a reserved, ignored V1 flag. PenetrationChecked and
     * Penetrating are never reported; their absence proves no clearance.
     * No engine pointer or retained runtime identity crosses the ABI.
     */


    /*
     * One value snapshot of a weapon-mounted visual emitter. The transform and
     * forward vector are expressed in the equipped weapon root's local game-
     * unit space and contain no retained engine object. Active follows the
     * effect geometry's effective scene visibility; Visible follows the node
     * that supplied the transform. EffectStateKnown distinguishes an inactive
     * effect from an AddOnNode marker for which no live effect was found.
     */
    struct RockProviderWeaponEmitterV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponEmitterV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::weapon::EmitterKind kind{ api::weapon::EmitterKind::Unknown };
        api::weapon::EmitterSource source{ api::weapon::EmitterSource::Unknown };
        std::uint32_t flags{ 0 };
        std::uint32_t active{ 0 };
        std::uint32_t visible{ 0 };
        std::uint32_t addOnNodeValue{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        api::Transform weaponLocalTransform{};
        api::Point3 forwardWeaponLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[8]{};
    };

    /*
     * Bounded, owner-scoped closest-hit world raycast. The direction is
     * normalized by ROCK and maxDistanceGame is capped by the public limit.
     * Generation guards use the established optional-zero V1 contract.
     * Query only from ROCK's owner frame callback on the game thread.
     *
     * ROCK selects its validated far-world collision filter; consumers do not
     * inject native filter bits or retain any Havok/world pointer. A miss is a
     * successful query whose endpoint is start + direction * max distance.
     */
    struct RockProviderWorldRaycastRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderWorldRaycastRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Point3 startGame{};
        api::Point3 directionGame{};
        float maxDistanceGame{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[8]{};
    };

    struct RockProviderWorldRaycastResultV1
    {
        std::uint32_t size{ sizeof(RockProviderWorldRaycastResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t hit{ 0 };
        std::uint32_t flags{ 0 };
        float hitFraction{ 1.0f };
        float hitDistanceGame{ 0.0f };
        api::Point3 hitPointGame{};
        api::Point3 hitNormalGame{};
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };




    struct RockProviderAnimationPhaseContextV1
    {
        std::uint32_t size{ sizeof(RockProviderAnimationPhaseContextV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        ::rock::api::animation::AnimationPhase phase{ ::rock::api::animation::AnimationPhase::BeforeRock };
        std::uint32_t flags{ 0 };
        std::uint64_t frameIndex{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t activeNativeAnimationAuthorityFlags{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    /*
     * Value-only snapshot of ROCK's equipped-weapon grip solution. Query only
     * from ROCK's animation/frame callbacks on the game thread; wrong-thread
     * reads fail closed. Scene pointers are identity witnesses for the current
     * frame and must never be retained. Hand transforms are exact ROCK targets
     * in Weapon local space. When MuzzleWorldValid is set, muzzleOriginGame is
     * the equipped weapon's native fire/projectile-node barrel tip, available
     * from equip rather than first fire, and muzzleDirectionGame is its
     * normalized world-space +Y firing axis.
     */
    struct RockProviderEquippedWeaponGripStateV1
    {
        std::uint32_t size{ sizeof(RockProviderEquippedWeaponGripStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t weaponNode{ 0 };
        api::Transform weaponWorld{};
        api::Transform rightHandInWeapon{};
        api::Transform leftHandInWeapon{};
        api::Point3 muzzleOriginGame{};
        api::Point3 muzzleDirectionGame{};
        std::uint32_t reserved[2]{};
    };

    /*
     * leaseFrames must be non-zero and is clamped to the public maximum.
     * A rolling lease returns to ROCK's configured fallback handling policy if
     * the addon stops publishing, unregisters, faults, or loses the provider.
     * Generation guards use the established optional-zero V1 contract.
     */
    struct RockProviderEquippedWeaponHandlingRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderEquippedWeaponHandlingRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        float gripZoneEquipRadiusGameUnits{ 3.0f };
        float gripZoneEquipSettleSeconds{ 0.15f };
        float firingGripReattachRadiusGameUnits{ 3.0f };
        float gripZoneHoverHapticIntensity{ 0.75f };
        float firingGripProximitySupportRadiusGameUnits{ 6.0f };
        float weaponGripHapticDurationSeconds{ 0.10f };
        float firingGripAttachHapticIntensity{ 0.85f };
        float firingGripDetachHapticIntensity{ 0.30f };
        float supportGripHapticIntensity{ 0.50f };
        // Reserved V1 compatibility field; handoff uses the reattach cylinders.
        float firingGripPromotionRadiusGameUnits{ 5.0f };
        float leftFiringAimYawDegrees{ 0.0f };
        float leftFiringAimPitchDegrees{ 0.0f };
        float leftFiringAimOffsetGameUnits[3]{};
        float equipVisualBridgeTimeoutSeconds{ 2.0f };
        float equipVisualBridgeBlendSeconds{ 0.15f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[8]{};
    };

    struct RockProviderEquippedWeaponHandlingStateV1
    {
        std::uint32_t size{ sizeof(RockProviderEquippedWeaponHandlingStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t authorityFlags{ 0 };
        std::uint32_t runtimeFlags{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        api::Hand fixedFiringHand{ api::Hand::Right };
        api::Hand currentFiringHand{ api::Hand::Right };
        std::uint32_t reserved[9]{};
    };






    /*
     * Detailed weapon evidence carries semantic body identity, local generated
     * bounds, and total point count without making the fixed function table own
     * variable-length buffers. Callers fetch the local mesh point cloud through
     * the body-id keyed copy function below.
     */
    struct RockProviderWeaponEvidenceDetailV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponEvidenceDetailV1) };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t fallbackGripPose{ 0 };
        std::uintptr_t interactionRoot{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        api::Bounds3 localBoundsGame{};
        std::uint32_t pointCount{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        /*
         * Record-authored identity (feature bit WeaponPartRecordIdentity;
         * fields were reserved zeros before it). omodFormId is the installed
         * OMOD occupying this part's slot (0 when unpaired); attachPointFormId
         * is the vanilla attach-point keyword of that slot; classification-
         * Source is api::weaponparts::ClassificationSource.
         */
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct RockProviderBodyContactV1
    {
        std::uint32_t size{ sizeof(RockProviderBodyContactV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t bodyLayer{ 0xFFFF'FFFF };
        std::uint32_t targetLayer{ 0xFFFF'FFFF };
        api::collision::BodyZoneKind zone{ api::collision::BodyZoneKind::Unknown };
        api::collision::BodyZoneSide side{ api::collision::BodyZoneSide::Center };
        std::uint32_t role{ 0 };
        std::uint32_t descriptorIndex{ 0 };
        api::grab::BodyContactTargetKind targetKind{ api::grab::BodyContactTargetKind::Unknown };
        api::collision::BodyZoneKind targetZone{ api::collision::BodyZoneKind::Unknown };
        api::collision::BodyZoneSide targetSide{ api::collision::BodyZoneSide::Center };
        std::uint32_t targetRole{ 0 };
        std::uint32_t targetDescriptorIndex{ 0 };
        std::uint32_t inPowerArmor{ 0 };
        std::uint32_t targetInPowerArmor{ 0 };
        std::uint32_t hasContactPointGame{ 0 };
        std::uint32_t reserved0{ 0 };
        api::Point3 contactPointGame{};
        std::uint32_t reserved[8]{};
    };



    struct RockProviderHandInteractionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderHandInteractionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        api::Hand hand{ api::Hand::None };
        api::grab::HandInteractionPhase phase{ api::grab::HandInteractionPhase::Idle };
        api::grab::BodyContactTargetKind targetKind{ api::grab::BodyContactTargetKind::Unknown };
        std::uint32_t flags{ 0 };
        // TouchGrab: zero-extended referenceNativeHandle. Weapon: generation
        // key. Interpret only with targetKind; zero means unavailable.
        std::uint64_t reservedTargetIdentity{ 0 };
        std::uint32_t targetFormId{ 0 };
        // For TouchGrab, always identifies the target, including global world surfaces.
        std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
        std::uint32_t heldBodyCount{ 0 };
        std::uint32_t heldBodyIds[ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1]{};
        std::uint32_t effectiveInputSuppressionFlags{ 0 };
        std::uint32_t collisionAvailabilityFlags{ 0 };
        std::uint64_t stateSequence{ 0 };
        std::uint64_t targetSequence{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t releaseSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        api::Point3 surfaceAnchorGame{};
        ::rock::api::touch::SurfaceGripMode surfaceGripMode{
            ::rock::api::touch::SurfaceGripMode::CollisionAnchor
        };
    };

    struct RockProviderEventV1
    {
        std::uint32_t size{ sizeof(RockProviderEventV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t sequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        RockProviderEventKindV1 kind{ RockProviderEventKindV1::Unknown };
        api::Hand hand{ api::Hand::None };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t formId{ 0 };
        std::uint32_t result{ 0 };
        std::uint64_t subjectSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t data[5]{};
    };

    struct RockProviderEquippedWeaponStateV1
    {
        std::uint32_t size{ sizeof(RockProviderEquippedWeaponStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t transitionSequence{ 0 };
        std::uint64_t terminalSequence{ 0 };
        api::weapon::TransitionSource transitionSource{
            api::weapon::TransitionSource::Unknown
        };
        api::weapon::TransitionResult terminalResult{
            api::weapon::TransitionResult::None
        };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // Current weapon fields above never contain a historical identity.
        // Transition identity exists only while TransitionActive; terminal
        // identity/source describe terminalResult even during a newer transition.
        std::uint32_t transitionWeaponFormId{ 0 };
        std::uint32_t terminalWeaponFormId{ 0 };
        api::weapon::TransitionSource terminalSource{
            api::weapon::TransitionSource::Unknown
        };
        std::uint32_t reserved[4]{};
    };



    struct RockProviderWeaponPartResolutionQueryV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartResolutionQueryV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartResolutionResultV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartResolutionResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        api::weaponparts::GrabMode grabMode{ api::weaponparts::GrabMode::None };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t reserved0{ 0 };
        std::uint64_t winningOwnerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartPoseV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartPoseV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t flags{ 0 };
        // api::weaponparts::ActionRole for this semantic source.
        std::uint32_t actionRole{ 0 };
        api::Transform sourceParentLocal{};
        api::Transform weaponRootLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderWeaponPartDriveApplicationResultV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartDriveApplicationResultV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        api::weaponparts::DriveApplication result{
            api::weaponparts::DriveApplication::Unknown
        };
        api::Transform appliedSourceParentLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderScopeSightStateV1
    {
        std::uint32_t size{ sizeof(RockProviderScopeSightStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t publicationSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t flags{ 0 };
        api::weapon::ScopeActivationSource activationSource{
            api::weapon::ScopeActivationSource::None
        };
        std::uint32_t nativeScopeOverlayIndex{ 0 };
        api::Point3 anchorWeaponLocal{};
        api::Bounds3 sightBoundsWeaponLocal{};
        std::uint32_t sightBodyCount{ 0 };
        std::uint32_t sightBodyId{ 0x7FFF'FFFF };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderWeaponCompositionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponCompositionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t compositionSignature{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t entryCount{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
        std::uint64_t missingCoverageMask{ 0 };
        std::uint64_t publicationSequence{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct RockProviderWeaponCompositionEntryV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponCompositionEntryV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t stableIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct RockProviderAuthoredGripPoseV1
    {
        std::uint32_t size{ sizeof(RockProviderAuthoredGripPoseV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        api::weapon::AuthoredGripSource source{ api::weapon::AuthoredGripSource::Unknown };
        std::uint64_t variantKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t flags{ 0 };
        std::uint16_t rightFingerLocalTransformMask{ 0 };
        std::uint16_t leftFingerLocalTransformMask{ 0 };
        api::Transform rightHandInWeapon{};
        api::Transform leftHandInWeapon{};
        api::Transform rightFingerLocalTransforms[15]{};
        api::Transform leftFingerLocalTransforms[15]{};
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };


    struct RockProviderSemanticHandContactV1
    {
        std::uint32_t size{ sizeof(RockProviderSemanticHandContactV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        api::Hand hand{ api::Hand::None };
        std::uint32_t role{ 0 };
        std::uint32_t finger{ 0 };
        std::uint32_t segment{ 0 };
        std::uint32_t handBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t flags{ 0 };
        api::collision::SemanticContactState contactState{
            api::collision::SemanticContactState::Continued
        };
        std::uint32_t framesSinceContact{ 0 };
        std::uint32_t contactSequence{ 0 };
        api::Point3 contactPointGame{};
        api::Point3 contactNormalGame{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderPlayerColliderDescriptorV1
    {
        std::uint32_t size{ sizeof(RockProviderPlayerColliderDescriptorV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        api::collision::PlayerColliderKind kind{ api::collision::PlayerColliderKind::Hand };
        api::Hand hand{ api::Hand::None };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t role{ 0 };
        api::collision::BodyZoneKind zone{ api::collision::BodyZoneKind::Unknown };
        api::collision::BodyZoneSide side{ api::collision::BodyZoneSide::Center };
        std::uint32_t descriptorIndex{ 0 };
        std::uint32_t flags{ 0 };
        float lengthGameUnits{ 0.0f };
        float radiusGameUnits{ 0.0f };
        api::Transform transform{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderHandCollisionAvailabilityV1
    {
        std::uint32_t size{ sizeof(RockProviderHandCollisionAvailabilityV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        api::Hand hand{ api::Hand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t collisionSequence{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t handBodyCount{ 0 };
        std::uint32_t dynamicTwinCount{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // reserved[0] = contacted dynamic-twin slot mask for the other hand.
        // reserved[1] = contacted dynamic-twin slot mask for the weapon proxy.
        // reserved[2] = stable dynamic interaction collision layer (48/52).
        // reserved[3] = active exact hand/weapon pair-suppression lease count.
        std::uint32_t reserved[4]{};
        std::uint32_t collisionEnabledBodyCount{ 0 };
        std::uint32_t filterKnownBodyCount{ 0 };
    };


    struct RockProviderOffhandReservationRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::grab::HandReservationKind reservation{ api::grab::HandReservationKind::Normal };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct RockProviderOffhandReservationStateV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        api::Hand hand{ api::Hand::None };
        api::grab::HandReservationKind reservation{ api::grab::HandReservationKind::Normal };
        std::uint32_t active{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t remainingFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    /*
     * Publications are copied into ROCK's bounded registry and replace one
     * owner/scope transactionally. Every target must carry the current,
     * nonzero world/skeleton/provider generations and a nonzero owner-defined
     * targetGeneration. Bump targetGeneration whenever the resolved body or
     * any behavioral geometry/policy changes; refresh the unchanged value only
     * to renew its lease.
     *
     * LimitedHinge coordinates and limits are radians. LimitedPrismatic
     * coordinates and limits are game units. Their pivot and normalized axis
     * are in current world/game space. ROCK temporarily converts a keyframed
     * mechanism body to dynamic while held, owns all constraints, and restores
     * the original motion class before publishing Latched/Yielded/Invalidated.
     *
     * FixedAnchor either names one body or uses MatchAnyBody plus a nonzero
     * allowedLayerMask. A dynamic palm/fingertip touch plus grip locks the
     * rendered hand and its dynamic proxy bodies relative to the matched body
     * until grip release. ROCK never changes, activates, constrains, or writes
     * velocity to the matched body. Held state reports the resolved bodyId,
     * active hand mask, and contact point/normal when the manifold supplies
     * them; referenceFormId/referenceNativeHandle remain provider-authored.
     * Exact body registrations are resolved before wildcard registrations.
     * One wildcard descriptor owns at most one resolved body concurrently;
     * publish disjoint right/left wildcard descriptors when a consumer needs
     * two independent surfaces at once (for example, two-hand climbing).
     */



    // Availability is independent of boolean values. Node names are descriptive,
    // not globally unique part IDs. All transforms use game units.
    struct RockProviderReferenceQueryV1
    {
        std::uint32_t size{ sizeof(RockProviderReferenceQueryV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t referenceFormId{ 0 };
        // Optional additional identity check; zero means unspecified.
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // Native furniture use is indexed. Zero is marker 0, not "any marker".
        std::int32_t furnitureMarkerIndex{ 0 };
    };

    struct RockProviderReferenceInteractionV1
    {
        std::uint32_t size{ sizeof(RockProviderReferenceInteractionV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t baseFormType{ 0 };
        std::uint32_t activationBlocked{ 0 };
        api::grab::NativeOpenState openState{};
        std::uint32_t furnitureInUse{ 0 };
        std::uint32_t furnitureInUseIncludingReservations{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint64_t frameIndex{ 0 };
    };

    struct RockProviderHandTargetDetailsV1
    {
        std::uint32_t size{ sizeof(RockProviderHandTargetDetailsV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHandInteractionStateV1 handState{};
        RockProviderReferenceInteractionV1 reference{};
        std::uint32_t flags{ 0 };
        std::uint32_t collisionLayer{ 0 };
        api::Point3 anchorGame{};
        api::Point3 normalGame{};
        api::PowerArmorPoint powerArmorPoint{};
        std::uint32_t sourceTriangleIndex{ 0xFFFF'FFFFu };
        char collisionNodeName[64]{};
        char meshPartName[64]{};
    };

    struct RockProviderPowerArmorPointPoseV1
    {
        api::PowerArmorPoint point{};
        // Animated armor-bone origin, not a sampled human palm pose.
        std::uint32_t valid{ 0 };
        api::Transform world{};
        api::Transform frameLocal{};
    };

    struct RockProviderPowerArmorTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderPowerArmorTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderReferenceInteractionV1 touchedReference{};
        RockProviderReferenceInteractionV1 frameReference{};
        std::uint32_t flags{ 0 };
        // Nonzero only when the queried reference is a verified PA actor.
        // Furniture reservations do not imply a wearer.
        std::uint32_t actorFormId{ 0 };
        RockProviderPowerArmorPointPoseV1 points[2]{};
    };

    struct RockProviderPowerArmorGrabRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderPowerArmorGrabRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderReferenceQueryV1 target{};
        api::Hand hand{ api::Hand::None };
        api::PowerArmorPoint point{};
        // Zero uses the native proximity radius. Maximum 32 game units.
        float maxDistanceGame{ 0.0f };
    };


using RockProviderFrameCallback = rock::api::core::FrameCallback;
using RockProviderAnimationPhaseCallbackV1 = rock::api::animation::PhaseCallback;
}
