#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <ROCK/Core.h>
#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Diagnostics.h>
#include <ROCK/Grab.h>
#include <ROCK/Input.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#define ROCK_PROVIDER_CALL __cdecl

// Private runtime representation; never installed or returned across the DLL boundary.
namespace rock::provider {
    inline constexpr std::uint32_t ROCK_PROVIDER_API_VERSION = 1;
    inline constexpr std::uint32_t ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE = 256;
    inline constexpr std::uint32_t ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE = 112;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_BODIES = 8;
    // Evidence describes the complete classified weapon-part catalog and is
    // independent from the compact body array embedded in frame snapshots.
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1 = api::weaponparts::kMaxEvidenceDetails;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_POINTS_PER_DETAIL_V1 = api::weaponparts::kMaxEvidencePoints;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1 = api::collision::kMaxExternalBodies;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1 = api::collision::kMaxExternalContacts;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_BODY_CONTACTS_V1 = api::collision::kMaxBodyContacts;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1 = api::core::kMaxFrameCallbacks;
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
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1 = api::core::kMaxPhaseCallbacks;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1 = api::animation::kMaxHandVisualLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1 = api::animation::kMaxRuntimePublicationLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1 = api::weapon::kMaxHandlingLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLICATION_LEASE_FRAMES_V1 = api::diagnostics::kMaxLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1 = api::diagnostics::kMaxOverlayPublishers;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_PER_PUBLISHER_V1 = api::diagnostics::kMaxLinesPerPublisher;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_PER_PUBLISHER_V1 = api::diagnostics::kMaxTextPerPublisher;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1 = api::diagnostics::kMaxCombinedLines;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1 = api::diagnostics::kMaxCombinedText;
    inline constexpr std::uint32_t ROCK_PROVIDER_DEBUG_OVERLAY_TEXT_CAPACITY_V1 = 128;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_COLLIDER_VISUALIZATION_OVERRIDE_LEASE_FRAMES_V1 = api::diagnostics::kMaxLeaseFrames;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1 = 256;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1 = api::collision::kMaxExternalScopes;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1 = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_COMPOSITION_ENTRIES_V1 = api::weapon::kMaxCompositionEntries;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_SEMANTIC_HAND_CONTACTS_V1 = api::collision::kMaxSemanticContacts;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_PLAYER_COLLIDER_DESCRIPTORS_V1 = api::collision::kMaxColliderDescriptors;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_POSES_V1 = api::weaponparts::kMaxPoses;
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

    enum class RockProviderHand : std::uint32_t
    {
        None = 0,
        Right = 1,
        Left = 2,
    };

    enum class RockProviderHandStateFlag : std::uint32_t
    {
        None = 0,
        Touching = 1u << 0,
        // Effective occupancy: loose hold, weapon grip/native carry, or touch grab.
        Holding = 1u << 1,
        PhysicsDisabled = 1u << 2,
        // State is known independently of hand-transform availability.
        Valid = 1u << 3,
    };

    enum class RockProviderHandFrameFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Left = 1u << 1,
        Primary = 1u << 2,
        Offhand = 1u << 3,
        HasSceneNode = 1u << 4,
        RootFlattenedAuthority = 1u << 5,
        PresentedVisual = 1u << 6,
    };

    enum class RockProviderExternalBodyRole : std::uint32_t
    {
        Unknown = 0,
        ReloadMobile = 1,
        ReloadSocket = 2,
        ReloadAction = 3,
        ReloadVisualProxy = 4,
        ActorRagdollBone = 100,
    };

    enum class RockProviderExternalBodyContactPolicy : std::uint32_t
    {
        None = 0,
        ReportHandContacts = 1u << 0,
        ReportAllSourceKinds = 1u << 1,
        SuppressRockDynamicPush = 1u << 2,
    };

    enum class RockProviderExternalSourceKind : std::uint32_t
    {
        Unknown = 0,
        Hand = 1,
        Weapon = 2,
        HeldObject = 3,
    };

    enum class RockProviderExternalContactQuality : std::uint32_t
    {
        BodyPairOnly = 0,
        AggregateImpulse = 1,
        RawPoint = 2,
    };

    /*
     * Touch-grab targets are a separate opt-in authority from ROCK's ordinary
     * loose-object grab. Limited mechanisms may name one live dynamic or
     * keyframed body. FixedAnchor may additionally match a bounded collision
     * layer/motion mask without changing that body's motion type; this is the
     * hand-to-surface latch primitive used by climbing consumers.
     */
    enum class RockProviderTouchGrabKindV1 : std::uint32_t
    {
        FixedAnchor = 0,
        LimitedHinge = 1,
        LimitedPrismatic = 2,
    };

    enum class RockProviderTouchGrabTargetFlagV1 : std::uint32_t
    {
        None = 0,
        AllowRightHand = 1u << 0,
        AllowLeftHand = 1u << 1,
        AllowTwoHands = 1u << 2,
        LatchOnRelease = 1u << 3,
        MatchAnyBody = 1u << 4,
        MatchStaticMotion = 1u << 5,
        MatchKeyframedMotion = 1u << 6,
        MatchDynamicMotion = 1u << 7,
        // Wildcard FixedAnchor only. Considered after explicit bodies, native
        // authored points, close objects and ordinary wildcard registrations.
        FallbackOnly = 1u << 8,
        // Exclude native PA furniture and PA actors from this target's surface.
        ExcludePowerArmor = 1u << 9,
    };

    enum class RockProviderTouchGrabPhaseV1 : std::uint32_t
    {
        Inactive = 0,
        Armed = 1,
        Held = 2,
        Latched = 3,
        Yielded = 4,
        Invalidated = 5,
    };

    enum class RockProviderTouchGrabReleaseReasonV1 : std::uint32_t
    {
        None = 0,
        GripReleased = 1,
        OwnerYield = 2,
        TargetRemoved = 3,
        RegistrationExpired = 4,
        GenerationChanged = 5,
        WorldLost = 6,
        TargetInvalid = 7,
        HandUnavailable = 8,
    };

    enum class RockProviderTouchGrabStateFlagV1 : std::uint32_t
    {
        None = 0,
        ContactPointValid = 1u << 0,
        ContactNormalValid = 1u << 1,
        CoordinateValid = 1u << 2,
        FixedAnchor = 1u << 3,
        OriginalMotionKeyframed = 1u << 4,
        OriginalMotionDynamic = 1u << 5,
        MeshSurfaceAnchor = 1u << 6,
        MeshFingerPose = 1u << 7,
        MeshCollisionFallback = 1u << 8,
    };

    enum class RockProviderSurfaceGripModeV1 : std::uint32_t
    {
        CollisionAnchor = 0,
        MeshAnchor = 1,
        CollisionFallback = 2,
        AnimatedArmorBone = 3,
    };

    enum class RockProviderTouchGrabHandMaskV1 : std::uint32_t
    {
        None = 0,
        Right = 1u << 0,
        Left = 1u << 1,
    };

    enum class RockProviderOffhandReservation : std::uint32_t
    {
        Normal = 0,
        ReloadReserved = 1,
        ReloadPoseOverride = 2,
    };

    enum class RockProviderBodyZoneSide : std::uint32_t
    {
        Center = 0,
        Left = 1,
        Right = 2,
    };

    enum class RockProviderBodyZoneKind : std::uint32_t
    {
        Unknown = 0,
        Pelvis = 1,
        SpineLower = 2,
        SpineUpper = 3,
        Chest = 4,
        NeckHead = 5,
        LeftShoulder = 6,
        LeftUpperArm = 7,
        LeftForearmUpper = 8,
        LeftForearmLower = 9,
        LeftHand = 10,
        RightShoulder = 11,
        RightUpperArm = 12,
        RightForearmUpper = 13,
        RightForearmLower = 14,
        RightHand = 15,
        LeftHip = 16,
        LeftThigh = 17,
        LeftCalf = 18,
        LeftFoot = 19,
        RightHip = 20,
        RightThigh = 21,
        RightCalf = 22,
        RightFoot = 23,
    };

    enum class RockProviderBodyContactTargetKind : std::uint32_t
    {
        Unknown = 0,
        Hand = 1,
        Weapon = 2,
        HeldObject = 3,
        Body = 4,
        External = 5,
        WorldSurface = 6,
        DynamicProp = 7,
        Actor = 8,
        QueryOnly = 9,
    };

    enum class RockProviderLifecycleFlag : std::uint32_t
    {
        None = 0,
        WorldAvailable = 1u << 0,
        SkeletonReady = 1u << 1,
        ProviderReady = 1u << 2,
        MenuBlocking = 1u << 3,
        ConfigBlocking = 1u << 4,
        LoadingOrWorldTransition = 1u << 5,
        GeneratedBodiesValid = 1u << 6,
        PhysicsWriteAllowed = 1u << 7,
        VisualWriteAllowed = 1u << 8,
    };

    enum class RockProviderLifecycleReason : std::uint32_t
    {
        None = 0,
        GameLoaded = 1,
        SkeletonReady = 2,
        SkeletonDestroying = 3,
        PowerArmorChanged = 4,
        WorldAvailable = 5,
        WorldChanged = 6,
        WorldUnavailable = 7,
        ProviderReady = 8,
        ProviderLost = 9,
        MenuBlocked = 10,
        ConfigBlocked = 11,
        GeneratedBodiesRebuilt = 12,
        GeneratedBodiesInvalidated = 13,
        TransitionSettled = 14,
        Shutdown = 15,
    };

    enum class RockProviderResultV1 : std::uint32_t
    {
        Ok = 0,
        NotReady = 1,
        InvalidArgument = 2,
        InvalidSize = 3,
        UnsupportedVersion = 4,
        CapacityFull = 5,
        OwnerNotRegistered = 6,
        OwnerConflict = 7,
        PermissionDenied = 8,
        WorldNotReady = 9,
        TargetInvalid = 10,
        TargetUnavailable = 11,
        HandUnavailable = 12,
        HandBusy = 13,
        ObjectAlreadyOwned = 14,
        RequestQueued = 15,
        RequestNotFound = 16,
        WrongThread = 17,
        AlreadyCommitted = 18,
    };

    enum class RockProviderConsumerCapabilityV1 : std::uint32_t
    {
        None = 0,
        FrameSnapshots = 1u << 0,
        ExternalBodies = 1u << 1,
        ExternalContacts = 1u << 2,
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
        DebugOverlayPublication = 1u << 13,
        ProviderEvents = 1u << 14,
        HandInteractionState = 1u << 15,
        ExternalBodyScopes = 1u << 16,
        WeaponPartObservability = 1u << 17,
        WeaponComposition = 1u << 18,
        PoseReadback = 1u << 19,
        SemanticHandContacts = 1u << 20,
        PlayerColliderDescriptors = 1u << 21,
        ScopeSightState = 1u << 22,
        InputObservability = 1u << 23,
        TouchGrabTargets = 1u << 24,
        WorldRaycasts = 1u << 25,
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
    enum class RockProviderNativeAnimationAuthorityFlagV1 : std::uint32_t
    {
        None = 0,
        Arms = 1u << 0,
        Hands = 1u << 1,
        Weapon = 1u << 2,
        ReloadPose = (1u << 0) | (1u << 1) | (1u << 2),
    };

    enum class RockProviderNativeAnimationAuthorityStatusFlagV1 : std::uint32_t
    {
        None = 0,
        HookInstalled = 1u << 0,
        RuntimeEnabled = 1u << 1,
        CaptureValid = 1u << 2,
        LocalReloadTestLeaseActive = 1u << 3,
        HookInstallFailed = 1u << 4,
        ThreadMismatch = 1u << 5,
        CaptureFault = 1u << 6,
        RuntimeProviderAvailable = 1u << 7,
    };

    enum class RockProviderAnimationPhaseV1 : std::uint32_t
    {
        BeforeRock = 1,
        AfterRock = 2,
        Complete = 3,
        NativeGraphOutput = 4,
        Presented = 5,
    };

    enum class RockProviderAnimationPhaseContextFlagV1 : std::uint32_t
    {
        None = 0,
        RockEnabled = 1u << 0,
        ProviderReady = 1u << 1,
        SkeletonReady = 1u << 2,
        MenuBlocking = 1u << 3,
        ConfigBlocking = 1u << 4,
        VisualWritesAllowed = 1u << 5,
    };

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

    enum class RockProviderHandVisualAuthorityFlagV1 : std::uint32_t
    {
        None = 0,
        WorldTransform = 1u << 0,
        FingerLocalTransforms = 1u << 1,
    };

    enum class RockProviderDebugOverlayTextFlagV1 : std::uint32_t
    {
        None = 0,
        WorldAnchored = 1u << 0,
    };

    enum class RockProviderWeaponEmitterKindV1 : std::uint32_t
    {
        Unknown = 0,
        Flashlight = 1,
        Laser = 2,
        Reticle = 3,
    };

    enum class RockProviderWeaponEmitterSourceV1 : std::uint32_t
    {
        Unknown = 0,
        EffectGeometry = 1,
        AddOnNode = 2,
    };

    enum class RockProviderWeaponEmitterFlagV1 : std::uint32_t
    {
        None = 0,
        TransformValid = 1u << 0,
        DirectionValid = 1u << 1,
        EffectStateKnown = 1u << 2,
        HasAddOnNodeValue = 1u << 3,
        HasOmod = 1u << 4,
        HasAttachPoint = 1u << 5,
    };

    enum class RockProviderInteractionCommandKindV1 : std::uint32_t
    {
        Unknown = 0,
        ForceGrab = 1,
        ForceRelease = 2,
        ThrownDrop = 3,
    };

    enum class RockProviderInteractionCommandStateV1 : std::uint32_t
    {
        Unknown = 0,
        Queued = 1,
        Succeeded = 2,
        Rejected = 3,
        Cancelled = 4,
    };

    enum class RockProviderInteractionFailureV1 : std::uint32_t
    {
        None = 0,
        ProviderNotReady = 1,
        PhysicsWritesBlocked = 2,
        OwnerNotRegistered = 3,
        InvalidRequest = 4,
        StaleWorldGeneration = 5,
        StaleSkeletonGeneration = 6,
        StaleProviderGeneration = 7,
        TargetMissing = 8,
        TargetUnavailable = 9,
        TargetBodyMissing = 10,
        TargetAlreadyOwned = 11,
        HandInvalid = 12,
        HandDisabled = 13,
        HandBusy = 14,
        HandNotHolding = 15,
        HeldObjectMismatch = 16,
    };

    enum class RockProviderForceGrabFlagV1 : std::uint32_t
    {
        None = 0,
        UsePreferredGrabPointGame = 1u << 0,
        // targetFormId is an owned ALCH/throwable base form, not a world REFR.
        // Requires hand=None and no other flags. Transfers exactly one item to
        // the first free hand (right preferred), without using or equipping it.
        // Query InventoryForceGrab support first. Failure before the drop leaves
        // inventory unchanged; failed attachment attempts return the exact drop.
        FromPlayerInventory = 1u << 1,
    };

    enum class RockProviderForceReleaseFlagV1 : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    enum class RockProviderThrownDropFlagV1 : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    enum class RockProviderHandInputSuppressionFlagV1 : std::uint32_t
    {
        None = 0,
        SuppressNormalGrabPress = 1u << 0,
        SuppressGrabRelease = 1u << 1,
        SuppressHeldWeaponTriggerEquip = 1u << 2,
        SuppressGameplayCandidates = 1u << 3,
        SuppressOpenVrGameInput = 1u << 4,
        /*
         * The native primary-wand Pause binding has two independent actions:
         * ordinary VATS opens on release, while V.A.N.S. starts from held
         * samples after Bethesda's threshold. These flags suppress only their
         * named phase. Set both to consume the complete gesture. The action is
         * a single primary-wand path, so ROCK aggregates these two flags across
         * active right- and left-hand suppression leases.
         */
        SuppressNativeVats = 1u << 5,
        SuppressNativeVans = 1u << 6,
        // Grenade mode is enabled without a claim. A ready UI consumer renews
        // this flag (on either hand) to disable Pip-Boy-equipped B-hold draw.
        // Remove the flag or clear the lease to enable it again; expiry,
        // unregister and lifecycle loss also restore it. Other owners retain
        // their claims. Never renew merely because the consumer DLL loaded.
        // This flag alone leaves native taps, OpenVR input and grabs unchanged.
        SuppressGrenadeQuickDraw = 1u << 7,
        // Reserve this hand's trigger+grip chord. Alone, either button remains
        // native. When both raw buttons are held, ROCK applies ConfigModeChord
        // and OpenVrGameInput suppression before returning input to the game.
        // Renew only while the consumer can accept a new chord. After capture,
        // also request unconditional suppression through both physical releases.
        ReserveTriggerGripChord = 1u << 8,
        // General physical button/chord reservation. Uses the two masks in the
        // request; suppression applies only while every specified button is down.
        ReserveButtonChord = 1u << 9,
        SuppressConfigModeChord =
            static_cast<std::uint32_t>(SuppressNormalGrabPress) |
            static_cast<std::uint32_t>(SuppressGrabRelease) |
            static_cast<std::uint32_t>(SuppressHeldWeaponTriggerEquip) |
            static_cast<std::uint32_t>(SuppressGameplayCandidates),
    };

    enum class RockProviderWeaponPartGrabModeV1 : std::uint32_t
    {
        None = 0,
        FullTwoHandAuthority = 1,
        AttachOnly = 2,
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

    enum class RockProviderWeaponPartDriveSpaceV1 : std::uint32_t
    {
        WeaponRootLocal = 0,
        SourceParentLocal = 1,
    };

    /*
     * What a physical hand currently holds on the equipped weapon.
     * FiringGrip: the hand owns the firing grip (weapon rides this hand).
     * SupportFullAuthority: offhand support grip driving the two-hand solver.
     * SupportVisualOnly: visual-only support (sidearm shooting cup).
     * PartCarry: carry-authority part grip while the firing hand is detached.
     * AttachOnly: whitelist-mandated glue â€” the hand follows the part
     * (including provider part drives) but never steers the weapon.
     */
    enum class RockProviderWeaponPartGripKindV1 : std::uint32_t
    {
        None = 0,
        FiringGrip = 1,
        SupportFullAuthority = 2,
        SupportVisualOnly = 3,
        PartCarry = 4,
        AttachOnly = 5,
    };

    /*
     * Coarse whole-weapon handling class retained by the V1 contract. This is
     * resolved from authored runtime data, not from weight or mesh dimensions.
     */
    enum class RockProviderWeaponSizeClassV1 : std::uint32_t
    {
        Melee = 0,
        Pistol = 1,
        Rifle = 2,
        Heavy = 3,
    };

    /*
     * Records the authored runtime signal that produced the handling class.
     */
    enum class RockProviderWeaponClassificationSourceV1 : std::uint32_t
    {
        None = 0,
        Keyword = 1,
        WeaponData = 2,
        EquipSlot = 3,
    };

    /*
     * One bit per Fallout4.esm WeaponType* keyword found through the equipped
     * weapon's effective instance keyword form. Installed OMOD keyword changes
     * are already applied by the engine. A bitmask preserves valid combinations
     * such as CombatShotgun's Rifle and Shotgun keywords.
     */
    enum class RockProviderWeaponKeywordFlagV1 : std::uint64_t
    {
        None = 0,
        Pistol = 1ull << 0,
        Rifle = 1ull << 1,
        Shotgun = 1ull << 2,
        AssaultRifle = 1ull << 3,
        Sniper = 1ull << 4,
        GaussRifle = 1ull << 5,
        LaserMusket = 1ull << 6,
        HeavyGun = 1ull << 7,
        HandToHand = 1ull << 8,
        Melee1H = 1ull << 9,
        Melee2H = 1ull << 10,
        Unarmed = 1ull << 11,
        Minigun = 1ull << 12,
        Fatman = 1ull << 13,
        MissileLauncher = 1ull << 14,
        GatlingLaser = 1ull << 15,
        Flamer = 1ull << 16,
        Cryolater = 1ull << 17,
        JunkJet = 1ull << 18,
        RailwayRifle = 1ull << 19,
        Broadsider = 1ull << 20,
        Syringer = 1ull << 21,
        FlareGun = 1ull << 22,
        GammaGun = 1ull << 23,
        AlienBlaster = 1ull << 24,
        Ripper = 1ull << 25,
        Shishkebab = 1ull << 26,
        Laser = 1ull << 27,
        Plasma = 1ull << 28,
        Ballistic = 1ull << 29,
        Thrown = 1ull << 30,
        Grenade = 1ull << 31,
        Mine = 1ull << 32,
        Explosive = 1ull << 33,
        Automatic = 1ull << 34,
    };


    enum class RockProviderHandInteractionPhaseV1 : std::uint32_t
    {
        Idle = 0,
        Touching = 1,
        Selecting = 2,
        Pulling = 3,
        Catching = 4,
        // Includes native equipped carry and attachment-only weapon grips.
        Holding = 5,
        Releasing = 6,
        StashCandidate = 7,
        ConsumeCandidate = 8,
    };

    enum class RockProviderHandInteractionFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Primary = 1u << 1,
        Offhand = 1u << 2,
        LooseObject = 1u << 3,
        LooseWeapon = 1u << 4,
        // Effective firing-grip occupancy, independent of ROCK capture state.
        FiringGrip = 1u << 5,
        PartGrip = 1u << 6,
        PartCarry = 1u << 7,
        InputSuppressed = 1u << 8,
        CollisionAvailable = 1u << 9,
        TransitionSuppressed = 1u << 10,
        HeldBodyListTruncated = 1u << 11,
        TouchGrab = 1u << 12,
        FixedSurfaceLatch = 1u << 13,
        GlobalSurfaceLatch = 1u << 14,
        SurfaceAnchorValid = 1u << 15,
        MeshSurfaceAnchor = 1u << 16,
        MeshFingerPose = 1u << 17,
        MeshCollisionFallback = 1u << 18,
        DynamicOtherHandContact = 1u << 19,
        DynamicWeaponContact = 1u << 20,
        DynamicWeaponPairSuppressed = 1u << 21,
        // Occupancy provenance; these do not grant manipulation authority.
        NativeWeaponCarry = 1u << 22,
        RockGripActive = 1u << 23,
        AttachOnly = 1u << 24,
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

    enum class RockProviderEventStreamFlagV1 : std::uint32_t
    {
        None = 0,
        GapBeforeFirstCopied = 1u << 0,
        RingOverwroteRecords = 1u << 1,
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
        DebugOverlay = 8,
        WeaponPartTargets = 9,
        ColliderVisualization = 10,
    };

    enum class RockProviderEquippedWeaponTransitionSourceV1 : std::uint32_t
    {
        Unknown = 0,
        ObservedEquip = 1,
        HeldTriggerEquip = 2,
        HeldGripZoneEquip = 3,
        MenuExit = 4,
        WorkbenchExit = 5,
    };

    enum class RockProviderEquippedWeaponTransitionResultV1 : std::uint32_t
    {
        None = 0,
        Completed = 1,
        WeaponUnequipped = 2,
        IdentityLost = 3,
        ExpectedIdentityTimeout = 4,
        NativeAnimationHandoff = 5,
        WeaponNoLongerDrawn = 6,
        RecoveryExhausted = 7,
        ProviderLost = 8,
        Shutdown = 9,
        IntentionalShoulderSheathe = 10,
    };

    enum class RockProviderEquippedWeaponStateFlagV1 : std::uint32_t
    {
        None = 0,
        // Current observation is available; check WeaponPresent separately.
        Valid = 1u << 0,
        IdentityPending = 1u << 1,
        DrawPending = 1u << 2,
        BridgePresented = 1u << 3,
        NativeRenderable = 1u << 4,
        HandPoseHandoffComplete = 1u << 5,
        RecoveryExhausted = 1u << 6,
        TransitionActive = 1u << 7,
        WeaponPresent = 1u << 8,
        WeaponDrawn = 1u << 9,
        // NativeRenderable/HandPoseHandoffComplete describe the latest current
        // weapon observation, not the retained transition terminal result.
        PresentationKnown = 1u << 10,
        // Equipped identity can exist while holstered. WeaponPresent matches
        // handling-state WeaponPresent: drawn with a resolved weapon root.
        WeaponEquipped = 1u << 11,
        // Bipod mode is enabled and the physical right-stick click belongs to
        // a current weapon/surface contact, an active latch, or its release
        // this frame. False when bBipodMode is off. Consumers must reject the
        // complete opening gesture, not replay its release when this clears.
        BipodInputReserved = 1u << 12,
    };

    enum class RockProviderExternalContactFlagV1 : std::uint32_t
    {
        None = 0,
        SourceVelocityValid = 1u << 0,
        ContactPointValid = 1u << 1,
        ContactNormalValid = 1u << 2,
        ContactPointMeasured = 1u << 3,
        ContactPointEstimated = 1u << 4,
        CollisionAvailable = 1u << 5,
        TransitionSuppressed = 1u << 6,
    };

    enum class RockProviderExternalContactStreamFlagV1 : std::uint32_t
    {
        None = 0,
        GapBeforeFirstCopied = 1u << 0,
        RingOverwroteRecords = 1u << 1,
    };

    enum class RockProviderWeaponPartDriveApplicationV1 : std::uint32_t
    {
        Unknown = 0,
        Applied = 1,
        Unresolved = 2,
        StaleGeneration = 3,
        MissingParent = 4,
        LostPriority = 5,
        InvalidTransform = 6,
        CapacityRejected = 7,
        Restored = 8,
    };

    enum class RockProviderScopeActivationSourceV1 : std::uint32_t
    {
        None = 0,
        NativeGeometry = 1,
        RockGeometry = 2,
        ManualInput = 3,
    };

    enum class RockProviderScopeSightFlagV1 : std::uint32_t
    {
        None = 0,
        Available = 1u << 0,
        Active = 1u << 1,
        MenuOpen = 1u << 2,
        AnchorValid = 1u << 3,
        BoundsValid = 1u << 4,
        NativeOverlayValid = 1u << 5,
        ManualDirectTransitionRequired = 1u << 6,
    };

    enum class RockProviderWeaponPartPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        SourceParentLocalValid = 1u << 1,
        WeaponRootLocalValid = 1u << 2,
    };

    enum class RockProviderWeaponClassificationProvenanceFlagV1 : std::uint32_t
    {
        None = 0,
        KeywordEvidence = 1u << 0,
        EffectiveInstanceKeywordEvidence = 1u << 1,
        WeaponDataEvidence = 1u << 2,
        EquipSlotEvidence = 1u << 3,
        GenerationBound = 1u << 4,
    };

    enum class RockProviderWeaponCompositionFlagV1 : std::uint32_t
    {
        None = 0,
        Active = 1u << 0,
        Disabled = 1u << 1,
        AttachPointResolved = 1u << 2,
        SemanticEvidenceMatched = 1u << 3,
    };

    enum class RockProviderAuthoredGripSourceV1 : std::uint32_t
    {
        Unknown = 0,
        LiveEquippedGraph = 1,
        NativeIdlePreharvest = 2,
        RuntimeCanonical = 3,
    };

    enum class RockProviderAuthoredGripPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        RightHandValid = 1u << 1,
        LeftHandValid = 1u << 2,
        RightFingersValid = 1u << 3,
        LeftFingersValid = 1u << 4,
    };

    enum class RockProviderPresentedHandPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        HandWorldValid = 1u << 1,
        FingerLocalsValid = 1u << 2,
        RootFlattenedReadback = 1u << 3,
    };

    enum class RockProviderSemanticHandContactFlagV1 : std::uint32_t
    {
        None = 0,
        ContactPointValid = 1u << 0,
        ContactNormalValid = 1u << 1,
        TargetFormResolved = 1u << 2,
        HeldObjectRelation = 1u << 3,
        CollisionAvailable = 1u << 4,
        TransitionSuppressed = 1u << 5,
    };

    enum class RockProviderSemanticContactStateV1 : std::uint32_t
    {
        Begin = 1,
        Continued = 2,
        End = 3,
    };

    enum class RockProviderPlayerColliderKindV1 : std::uint32_t
    {
        Hand = 1,
        Body = 2,
    };

    enum class RockProviderPlayerColliderFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Enabled = 1u << 1,
        PrimaryPalmAnchor = 1u << 2,
        TransformValid = 1u << 3,
        InPowerArmor = 1u << 4,
        // Enabled means lifecycle-allowed and a readable, unsuppressed filter.
        LifecycleAllowed = 1u << 5,
        FilterKnown = 1u << 6,
        CollisionSuppressed = 1u << 7,
    };

    enum class RockProviderHandCollisionAvailabilityFlagV1 : std::uint32_t
    {
        None = 0,
        BodiesReady = 1u << 0,
        DynamicTwinsReady = 1u << 1,
        PhysicsWritesAllowed = 1u << 2,
        CollisionAvailable = 1u << 3,
        TransitionSuppressed = 1u << 4,
        MenuSuppressed = 1u << 5,
        HandDisabled = 1u << 6,
        DynamicInteractionsEnabled = 1u << 7,
        DynamicOtherHandContact = 1u << 8,
        DynamicWeaponContact = 1u << 9,
        DynamicWeaponPairSuppressed = 1u << 10,
        DynamicPairFilterReady = 1u << 11,
        FilterKnown = 1u << 12,
        CollisionSuppressed = 1u << 13,
    };

    enum class RockProviderInputAvailabilityReasonV1 : std::uint32_t
    {
        Available = 0,
        HookNotSampled = 1,
        BlockingMenu = 2,
        ReleaseToRearm = 3,
        InvalidButton = 4,
    };

    enum class RockProviderLogicalInputActionV1 : std::uint32_t
    {
        Jump = 1,
    };

    enum class RockProviderPlayerControllerImplementationV1 : std::uint32_t
    {
        Unknown = 0,
        Proxy = 1,
        RigidBody = 2,
    };

    enum class RockProviderPlayerSupportStateV1 : std::uint32_t
    {
        Unsupported = 0,
        Sliding = 1,
        Supported = 2,
    };

    enum class RockProviderPlayerControllerStateFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        PositionValid = 1u << 1,
        VelocityValid = 1u << 2,
        ShapeValid = 1u << 3,
        SupportNormalValid = 1u << 4,
        Supported = 1u << 5,
        Sliding = 1u << 6,
        PenetrationChecked = 1u << 7,
        Penetrating = 1u << 8,
        Proxy = 1u << 9,
        RigidBody = 1u << 10,
    };

    enum class RockProviderPlayerControllerQueryFlagV1 : std::uint32_t
    {
        None = 0,
        CheckPenetration = 1u << 0,
    };

    enum class RockProviderSuppressionInvalidationReasonV1 : std::uint32_t
    {
        None = 0,
        Expired = 1,
        GenerationChanged = 2,
        OwnerUnregistered = 3,
        ProviderLost = 4,
        ExplicitClear = 5,
        CallbackFault = 6,
    };

    enum class RockProviderCommandStageV1 : std::uint32_t
    {
        Unknown = 0,
        Accepted = 1,
        Queued = 2,
        Committed = 3,
        Applied = 4,
        Terminal = 5,
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

    [[nodiscard]] inline constexpr bool hasLifecycleFlag(std::uint32_t flags, RockProviderLifecycleFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasConsumerCapabilityV1(std::uint32_t capabilities, RockProviderConsumerCapabilityV1 capability)
    {
        return (capabilities & static_cast<std::uint32_t>(capability)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasHandInputSuppressionFlagV1(
        std::uint32_t flags,
        RockProviderHandInputSuppressionFlagV1 flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasWeaponPartTargetFlagV1(
        std::uint32_t flags,
        RockProviderWeaponPartTargetFlagV1 flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasWeaponKeywordFlagV1(std::uint64_t flags, RockProviderWeaponKeywordFlagV1 flag)
    {
        return (flags & static_cast<std::uint64_t>(flag)) != 0;
    }


    [[nodiscard]] inline constexpr bool hasTouchGrabTargetFlagV1(
        std::uint32_t flags,
        RockProviderTouchGrabTargetFlagV1 flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasNativeAnimationAuthorityFlagV1(
        std::uint32_t flags,
        RockProviderNativeAnimationAuthorityFlagV1 flag)
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
        RockProviderHand hand{ RockProviderHand::None };
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
        RockProviderHand hand{ RockProviderHand::None };
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
        RockProviderHand hand{ RockProviderHand::None };
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
        RockProviderInteractionCommandKindV1 kind{ RockProviderInteractionCommandKindV1::Unknown };
        RockProviderInteractionCommandStateV1 state{ RockProviderInteractionCommandStateV1::Unknown };
        RockProviderInteractionFailureV1 failure{ RockProviderInteractionFailureV1::None };
        RockProviderHand hand{ RockProviderHand::None };
        std::uintptr_t targetRefr{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        RockProviderCommandStageV1 stage{ RockProviderCommandStageV1::Unknown };
        RockProviderInteractionFailureV1 failureStage{ RockProviderInteractionFailureV1::None };
        std::uint64_t acceptedFrame{ 0 };
        std::uint64_t committedFrame{ 0 };
        std::uint64_t appliedFrame{ 0 };
        std::uint32_t reserved{ 0 };
    };

    struct RockProviderHandInputSuppressionRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderHandInputSuppressionRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // Physical left/right OpenVR button masks, split to preserve V1 alignment.
        // Used only with ReserveButtonChord. Renew unconditional flags after
        // capture until every member is released; expiry/clear restores input.
        std::uint32_t chordButtonsLow[2]{};
        std::uint32_t chordButtonsHigh[2]{};
        std::uint32_t reserved[4]{};
    };

    /*
     * Raw OpenVR wand button state as sampled by ROCK's controller-state hook
     * (feature bit RawWandButtonState). Level state only: press/release edge
     * tracking is deliberately not exposed because ROCK consumes its edge
     * queues internally every frame (grab and trigger-equip logic); consumers
     * derive their own edges from held transitions. available reads 0 until
     * the hook has sampled that wand; held reads 0 while a game-stopping menu
     * owns input, with the same release-to-rearm gating ROCK applies to its
     * own gameplay reads. This state stays readable while ROCK suppresses the
     * matching native game action (e.g. the pipboy trigger) - that is the
     * point: the game action is silenced, the physical button is not.
     */
    enum class RockProviderNativeInputContextFlagV1 : std::uint32_t
    {
        Available = 1u << 0,
        MenuActive = 1u << 1,
        PrimaryActivationTarget = 1u << 2,
    };

    struct RockProviderRawWandButtonStateV1
    {
        std::uint32_t size{ sizeof(RockProviderRawWandButtonStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        std::uint64_t sampleSequence{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        RockProviderInputAvailabilityReasonV1 availabilityReason{
            RockProviderInputAvailabilityReasonV1::HookNotSampled
        };
    };

    struct RockProviderWeaponPartTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        RockProviderWeaponPartGrabModeV1 grabMode{ RockProviderWeaponPartGrabModeV1::None };
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

    struct RockProviderTransform
    {
        float rotate[9]{};
        float translate[3]{};
        float scale{ 1.0f };
    };

    struct RockProviderWeaponPartDriveTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponPartDriveTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        RockProviderWeaponPartDriveSpaceV1 driveSpace{ RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t leaseFrames{ 1 };
        RockProviderTransform targetTransform{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[7]{};
    };

    enum class RockProviderWeaponPartGripLocalSpaceV1 : std::uint32_t
    {
        WeaponRootLocal = 0,
        PartSourceLocal = 1,
    };

    /*
     * Which signal classified a weapon part. NameToken is NIF-name matching
     * (author discretion, least trustworthy); SlotAnchor means the part sits
     * under a connect-point slot and carries record-authored identity
     * (attach point, owning OMOD); RigAnchor means an engine-animated rig
     * node (bolt, magazine display) supplied the function;
     * AttachmentEvidence means the installed OMOD or its discovered emitter
     * capabilities refined the physical module kind.
     */
    enum class RockProviderWeaponPartClassificationSourceV1 : std::uint32_t
    {
        NameToken = 0,
        SlotAnchor = 1,
        RigAnchor = 2,
        AttachmentEvidence = 3,
    };

    /*
     * Numeric contract for the partKind / actionRole payloads that already
     * flow through weapon evidence details, part targets, drive targets, and
     * grip states as raw uint32 values. Values mirror ROCK's internal
     * classification enums one-to-one (static_asserted inside ROCK, so drift
     * breaks ROCK's build, never a consumer at runtime). External consumers
     * use these to build part whitelists and gate grips without including
     * ROCK internals. Scope is reserved for an installed OMOD carrying
     * Fallout's native scope-overlay property; Sight covers every other optic,
     * including red-dot and holographic sights. LaserFlashlightCombo means one
     * physical module owns both emitter capabilities. MuzzleDevice covers the
     * dedicated muzzle attachment slot (suppressors, compensators, brakes, and
     * flash hiders). Bipod identifies an authored bipod component without
     * implying deployed/folded state.
     */
    enum class RockProviderWeaponPartKindV1 : std::uint32_t
    {
        Receiver = 0,
        Barrel = 1,
        Handguard = 2,
        Foregrip = 3,
        Pump = 4,
        Stock = 5,
        Grip = 6,
        Magazine = 7,
        Magwell = 8,
        Bolt = 9,
        Slide = 10,
        ChargingHandle = 11,
        BreakAction = 12,
        Cylinder = 13,
        Chamber = 14,
        Shell = 15,
        Round = 16,
        LaserCell = 17,
        Lever = 18,
        Sight = 19,
        Accessory = 20,
        CosmeticAmmo = 21,
        Other = 22,
        LaserSight = 23,
        Flashlight = 24,
        LaserFlashlightCombo = 25,
        Scope = 26,
        MuzzleDevice = 27,
        Bipod = 28,
    };

    enum class RockProviderWeaponActionRoleV1 : std::uint32_t
    {
        None = 0,
        Bolt = 1,
        Slide = 2,
        ChargingHandle = 3,
        Pump = 4,
        BreakAction = 5,
        Cylinder = 6,
        Lever = 7,
        Latch = 8,
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
        RockProviderHand hand{ RockProviderHand::None };
        RockProviderWeaponPartGripKindV1 gripKind{ RockProviderWeaponPartGripKindV1::None };
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
        RockProviderWeaponPartGripLocalSpaceV1 handPartLocalSpace{ RockProviderWeaponPartGripLocalSpaceV1::WeaponRootLocal };
        RockProviderTransform handPartLocal{};
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
        RockProviderTransform rightHandTransform{};
        RockProviderTransform leftHandTransform{};
        std::uint32_t rightHandBodyId{ 0x7FFF'FFFF };
        std::uint32_t leftHandBodyId{ 0x7FFF'FFFF };
        std::uint32_t weaponBodyIds[ROCK_PROVIDER_MAX_WEAPON_BODIES]{};
        std::uint32_t rightHandState{ 0 };
        std::uint32_t leftHandState{ 0 };
        RockProviderOffhandReservation offhandReservation{ RockProviderOffhandReservation::Normal };
        std::uint32_t externalBodyCount{ 0 };
        float gameToHavokScale{ 0.0f };
        float havokToGameScale{ 0.0f };
        std::uint32_t physicsScaleRevision{ 0 };
        std::uint32_t lifecycleFlags{ 0 };
        RockProviderLifecycleReason lastLifecycleReason{ RockProviderLifecycleReason::None };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t enrichmentFlags{ 0 };
        RockProviderTransform hmdTransform{};
        float hmdForwardWorld[3]{};
        RockProviderHand primaryHand{ RockProviderHand::Right };
        RockProviderHand offhandHand{ RockProviderHand::Left };
        std::uint64_t stateSequence{ 0 };
        std::uint32_t stateChangeMask{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint64_t equippedWeaponTransitionSequence{ 0 };
    };

    struct RockProviderHandFrameV1
    {
        std::uint32_t size{ sizeof(RockProviderHandFrameV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uintptr_t node{ 0 };
        RockProviderTransform transform{};
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t state{ 0 };
        std::uint32_t reserved[7]{};
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint64_t stateSequence{ 0 };
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
     * inspect keywordFlags with hasWeaponKeywordFlagV1. When no conclusive
     * authored signal exists, valid is zero and sizeClass is not authoritative.
     */
    struct RockProviderWeaponClassificationV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponClassificationV1) };
        std::uint32_t valid{ 0 };
        std::uint64_t keywordFlags{ 0 };
        RockProviderWeaponSizeClassV1 sizeClass{ RockProviderWeaponSizeClassV1::Rifle };
        RockProviderWeaponClassificationSourceV1 source{ RockProviderWeaponClassificationSourceV1::None };
        std::uint32_t formId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float confidence{ 0.0f };
        std::uint32_t provenanceFlags{ 0 };
    };

    struct RockProviderPoint3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    /*
     * Semantic input observed at the native gameplay handler. pressSequence
     * changes only on a new press and lets consumers detect the configured
     * logical Jump action without guessing an OpenVR axis or button mapping.
     */
    struct RockProviderLogicalInputActionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderLogicalInputActionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderLogicalInputActionV1 action{
            RockProviderLogicalInputActionV1::Jump
        };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        RockProviderInputAvailabilityReasonV1 availabilityReason{
            RockProviderInputAvailabilityReasonV1::HookNotSampled
        };
        std::uint64_t sampleSequence{ 0 };
        std::uint64_t pressSequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };

    /*
     * Frame-local value snapshot of FO4VR's native player character controller.
     * CheckPenetration is a reserved, ignored V1 flag. PenetrationChecked and
     * Penetrating are never reported; their absence proves no clearance.
     * No engine pointer or retained runtime identity crosses the ABI.
     */
    struct RockProviderPlayerControllerStateV1
    {
        std::uint32_t size{ sizeof(RockProviderPlayerControllerStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t flags{ 0 };
        RockProviderPlayerControllerImplementationV1 implementation{
            RockProviderPlayerControllerImplementationV1::Unknown
        };
        RockProviderPlayerSupportStateV1 supportState{
            RockProviderPlayerSupportStateV1::Unsupported
        };
        std::uint32_t reserved0{ 0 };
        RockProviderPoint3 positionGame{};
        RockProviderPoint3 velocityGame{};
        RockProviderPoint3 supportNormalGame{};
        float radiusGame{ 0.0f };
        float heightGame{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct RockProviderPlayerControllerJumpRequestV1
    {
        std::uint32_t size{
            sizeof(RockProviderPlayerControllerJumpRequestV1)
        };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        float heightGameUnits{ 0.0f };
        std::uint32_t reserved0{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderBounds3
    {
        RockProviderPoint3 min{};
        RockProviderPoint3 max{};
        std::uint32_t valid{ 0 };
        std::uint32_t reserved{ 0 };
    };

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
        RockProviderWeaponEmitterKindV1 kind{ RockProviderWeaponEmitterKindV1::Unknown };
        RockProviderWeaponEmitterSourceV1 source{ RockProviderWeaponEmitterSourceV1::Unknown };
        std::uint32_t flags{ 0 };
        std::uint32_t active{ 0 };
        std::uint32_t visible{ 0 };
        std::uint32_t addOnNodeValue{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        RockProviderTransform weaponLocalTransform{};
        RockProviderPoint3 forwardWeaponLocal{};
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[8]{};
    };

    enum class RockProviderWorldRaycastResultFlagV1 : std::uint32_t
    {
        None = 0,
        Hit = 1u << 0,
        NormalValid = 1u << 1,
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
        RockProviderPoint3 startGame{};
        RockProviderPoint3 directionGame{};
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
        RockProviderPoint3 hitPointGame{};
        RockProviderPoint3 hitNormalGame{};
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    /*
     * Temporarily replaces ROCK's complete debug-overlay presentation with one
     * exact collider from the active equipped-weapon generation. The body ID
     * must occur in ROCK's complete matching-generation weapon body catalog;
     * it is not restricted to the compact weaponBodyIds frame-snapshot array.
     * The optional partKind is descriptive metadata only; body identity
     * remains the authoritative selection key. Refresh from ROCK's owner frame
     * callback while focus is desired. Clearing or lease invalidation restores
     * the unchanged config-driven overlay on the next frame.
     */
    struct RockProviderColliderVisualizationRequestV1
    {
        std::uint32_t size{
            sizeof(RockProviderColliderVisualizationRequestV1)
        };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    /*
     * Authority is always a rolling bounded lease. leaseFrames must be nonzero,
     * is bounded by
     * ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1 and should
     * be refreshed by a consumer that wants rolling temporary authority.
     * Generation guards follow the same optional-zero contract as the other
     * V1 request surfaces.
     */
    struct RockProviderNativeAnimationAuthorityRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderNativeAnimationAuthorityRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[9]{};
    };

    struct RockProviderNativeAnimationAuthorityStateV1
    {
        std::uint32_t size{ sizeof(RockProviderNativeAnimationAuthorityStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t activeFlags{ 0 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t activeOwnerCount{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t reserved[8]{};
    };

    /*
     * Owner-bound main-thread animation phases expose stable capture and
     * presentation boundaries without exposing ROCK's native detours.
     * NativeGraphOutput runs at the byte-validated player graph-output entry,
     * before downstream native scene and hFRIK presentation writers; callbacks
     * at that phase may capture data but must not mutate the engine graph.
     * BeforeRock runs before ROCK mutates weapon/hand presentation, AfterRock
     * runs after the interaction update, and Complete closes the frame after
     * all visual writers. Unregister prevents future dispatch copies but does
     * not wait for an already copied invocation; callback and userData storage
     * must remain alive through that invocation. Faulting callbacks revoke all
     * stateful resources and callbacks owned by that consumer.
     */
    struct RockProviderAnimationPhaseContextV1
    {
        std::uint32_t size{ sizeof(RockProviderAnimationPhaseContextV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderAnimationPhaseV1 phase{ RockProviderAnimationPhaseV1::BeforeRock };
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
        RockProviderTransform weaponWorld{};
        RockProviderTransform rightHandInWeapon{};
        RockProviderTransform leftHandInWeapon{};
        RockProviderPoint3 muzzleOriginGame{};
        RockProviderPoint3 muzzleDirectionGame{};
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
        RockProviderHand fixedFiringHand{ RockProviderHand::Right };
        RockProviderHand currentFiringHand{ RockProviderHand::Right };
        std::uint32_t reserved[9]{};
    };

    /*
     * Requests that ROCK assign the currently equipped weapon's firing grip
     * to one exact physical hand. This does not select or equip an arbitrary
     * inventory stack: weaponFormId and weaponGenerationKey are optional-zero
     * identity guards for the weapon that is already equipped. The caller
     * must own the active equipped-weapon handling lease with firing-grip
     * ownership. Explicit right and left assignments have the same authority
     * requirement; AmbidextrousHandoff governs in-world role swaps, not this
     * direct assignment. Call only on ROCK's animation owner thread.
     * RequestQueued means the canonical
     * native-right or persistent-left carry is armed but not yet settled;
     * observe getEquippedWeaponHandlingStateV1 for the effective hand.
     */
    struct RockProviderEquippedWeaponHandRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderEquippedWeaponHandRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t reserved[3]{};
    };

    /*
     * A consumer publishes one hand world target and/or an exact 15-bone
     * finger-local pose through ROCK's FRIK authority bridge. Set/clear only
     * from ROCK's animation/frame callbacks on the game thread; wrong-thread
     * writes are rejected. ROCK derives a unique tag from ownerToken. Every
     * publication is a rolling bounded lease with generation guards and is
     * cleared on expiry, generation change, explicit clear, consumer
     * unregister, provider loss, or callback fault.
     */
    struct RockProviderHandVisualAuthorityRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderHandVisualAuthorityRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::int32_t priority{ 0 };
        std::uint16_t fingerLocalTransformMask{ 0 };
        std::uint16_t reserved0{ 0 };
        RockProviderTransform worldTransform{};
        RockProviderTransform fingerLocalTransforms[15]{};
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };

    /*
     * The addon that actually executes native animation authority publishes
     * capture health here. ROCK remains the V1 lease coordinator and folds
     * this status into getNativeAnimationAuthorityStateV1.
     */
    struct RockProviderNativeAnimationRuntimePublicationV1
    {
        std::uint32_t size{ sizeof(RockProviderNativeAnimationRuntimePublicationV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    /*
     * Diagnostic-only colored geometry submitted to ROCK's single OpenVR/D3D
     * overlay renderer. Consumer memory is copied during publish and is never
     * retained. Publications are owner-scoped, bounded, game-thread-only, and
     * retained. Every publication is a rolling bounded lease and is cleared on
     * expiry, generation change, explicit clear, unregister, callback fault,
     * or provider loss.
     */
    struct RockProviderDebugOverlayLineV1
    {
        std::uint32_t size{ sizeof(RockProviderDebugOverlayLineV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        float startGame[3]{};
        float endGame[3]{};
        float color[4]{ 1.0f, 1.0f, 1.0f, 1.0f };
        std::uint32_t reserved[2]{};
    };

    struct RockProviderDebugOverlayTextV1
    {
        std::uint32_t size{ sizeof(RockProviderDebugOverlayTextV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t flags{ 0 };
        std::uint32_t reserved0{ 0 };
        char text[ROCK_PROVIDER_DEBUG_OVERLAY_TEXT_CAPACITY_V1]{};
        float x{ 18.0f };
        float y{ 18.0f };
        float textSize{ 2.0f };
        float color[4]{ 0.90f, 1.0f, 0.95f, 0.92f };
        float worldAnchorGame[3]{};
        std::uint32_t reserved[4]{};
    };

    struct RockProviderDebugOverlayPublicationV1
    {
        std::uint32_t size{ sizeof(RockProviderDebugOverlayPublicationV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t lineCount{ 0 };
        std::uint32_t textCount{ 0 };
        const RockProviderDebugOverlayLineV1* lines{ nullptr };
        const RockProviderDebugOverlayTextV1* textEntries{ nullptr };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t reserved[4]{};
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
        RockProviderBounds3 localBoundsGame{};
        std::uint32_t pointCount{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        /*
         * Record-authored identity (feature bit WeaponPartRecordIdentity;
         * fields were reserved zeros before it). omodFormId is the installed
         * OMOD occupying this part's slot (0 when unpaired); attachPointFormId
         * is the vanilla attach-point keyword of that slot; classification-
         * Source is RockProviderWeaponPartClassificationSourceV1.
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
        RockProviderBodyZoneKind zone{ RockProviderBodyZoneKind::Unknown };
        RockProviderBodyZoneSide side{ RockProviderBodyZoneSide::Center };
        std::uint32_t role{ 0 };
        std::uint32_t descriptorIndex{ 0 };
        RockProviderBodyContactTargetKind targetKind{ RockProviderBodyContactTargetKind::Unknown };
        RockProviderBodyZoneKind targetZone{ RockProviderBodyZoneKind::Unknown };
        RockProviderBodyZoneSide targetSide{ RockProviderBodyZoneSide::Center };
        std::uint32_t targetRole{ 0 };
        std::uint32_t targetDescriptorIndex{ 0 };
        std::uint32_t inPowerArmor{ 0 };
        std::uint32_t targetInPowerArmor{ 0 };
        std::uint32_t hasContactPointGame{ 0 };
        std::uint32_t reserved0{ 0 };
        RockProviderPoint3 contactPointGame{};
        std::uint32_t reserved[8]{};
    };

    struct RockProviderExternalBodyRegistration
    {
        std::uint32_t size{ sizeof(RockProviderExternalBodyRegistration) };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t generation{ 0 };
        RockProviderExternalBodyRole role{ RockProviderExternalBodyRole::Unknown };
        RockProviderExternalBodyContactPolicy contactPolicy{ RockProviderExternalBodyContactPolicy::None };
        RockProviderHand ownerHand{ RockProviderHand::None };
    };

    struct RockProviderExternalContactV1
    {
        std::uint32_t size{ sizeof(RockProviderExternalContactV1) };
        std::uint32_t sourceBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetExternalBodyId{ 0x7FFF'FFFF };
        std::uint32_t generation{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        RockProviderExternalSourceKind sourceKind{ RockProviderExternalSourceKind::Unknown };
        RockProviderHand sourceHand{ RockProviderHand::None };
        RockProviderExternalBodyRole targetRole{ RockProviderExternalBodyRole::Unknown };
        RockProviderExternalContactQuality quality{ RockProviderExternalContactQuality::BodyPairOnly };
        float sourceVelocityHavok[4]{};
        float contactPointHavok[4]{};
        float contactNormalHavok[4]{};
        // Sum of Bethesda contact point weights at contact-signal +0x30; this is not an impulse magnitude.
        union
        {
            float contactPointWeightSum{ 0.0f };
            float aggregateImpulseMagnitude;
        };
        std::uint32_t sourcePartKind{ 0 };
        std::uint32_t sourceRole{ 0 };
        std::uint32_t sourceSubRole{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t collisionGeneration{ 0 };
    };

    struct RockProviderHandInteractionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderHandInteractionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        RockProviderHandInteractionPhaseV1 phase{ RockProviderHandInteractionPhaseV1::Idle };
        RockProviderBodyContactTargetKind targetKind{ RockProviderBodyContactTargetKind::Unknown };
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
        RockProviderPoint3 surfaceAnchorGame{};
        RockProviderSurfaceGripModeV1 surfaceGripMode{
            RockProviderSurfaceGripModeV1::CollisionAnchor
        };
    };

    struct RockProviderEventV1
    {
        std::uint32_t size{ sizeof(RockProviderEventV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t sequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        RockProviderEventKindV1 kind{ RockProviderEventKindV1::Unknown };
        RockProviderHand hand{ RockProviderHand::None };
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

    struct RockProviderEventStreamStateV1
    {
        std::uint32_t size{ sizeof(RockProviderEventStreamStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t oldestRetainedSequence{ 0 };
        std::uint64_t latestEmittedSequence{ 0 };
        std::uint64_t firstCopiedSequence{ 0 };
        std::uint64_t lastCopiedSequence{ 0 };
        std::uint32_t copiedCount{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t overwrittenCount{ 0 };
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
        RockProviderEquippedWeaponTransitionSourceV1 transitionSource{
            RockProviderEquippedWeaponTransitionSourceV1::Unknown
        };
        RockProviderEquippedWeaponTransitionResultV1 terminalResult{
            RockProviderEquippedWeaponTransitionResultV1::None
        };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        // Current weapon fields above never contain a historical identity.
        // Transition identity exists only while TransitionActive; terminal
        // identity/source describe terminalResult even during a newer transition.
        std::uint32_t transitionWeaponFormId{ 0 };
        std::uint32_t terminalWeaponFormId{ 0 };
        RockProviderEquippedWeaponTransitionSourceV1 terminalSource{
            RockProviderEquippedWeaponTransitionSourceV1::Unknown
        };
        std::uint32_t reserved[4]{};
    };

    struct RockProviderExternalContactRecordV1
    {
        std::uint32_t size{ sizeof(RockProviderExternalContactRecordV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t parentOwnerToken{ 0 };
        std::uint64_t scopeToken{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t sourceBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetExternalBodyId{ 0x7FFF'FFFF };
        std::uint32_t bodyGeneration{ 0 };
        RockProviderExternalSourceKind sourceKind{ RockProviderExternalSourceKind::Unknown };
        RockProviderHand sourceHand{ RockProviderHand::None };
        RockProviderExternalBodyRole targetRole{ RockProviderExternalBodyRole::Unknown };
        RockProviderExternalContactQuality quality{ RockProviderExternalContactQuality::BodyPairOnly };
        std::uint32_t flags{ 0 };
        float sourceVelocityHavok[3]{};
        float contactPointHavok[3]{};
        float contactNormalHavok[3]{};
        float contactPointWeightSum{ 0.0f };
        std::uint32_t sourcePartKind{ 0 };
        std::uint32_t sourceRole{ 0 };
        std::uint32_t sourceSubRole{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[3]{};
    };

    struct RockProviderExternalContactStreamStateV1
    {
        std::uint32_t size{ sizeof(RockProviderExternalContactStreamStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        // The sequence domain survives provider loss and scope replacement.
        // Retained evidence is cleared on loss; cursors need not be rewound.
        std::uint64_t oldestRetainedSequence{ 0 };
        std::uint64_t latestEmittedSequence{ 0 };
        std::uint64_t firstCopiedSequence{ 0 };
        std::uint64_t lastCopiedSequence{ 0 };
        std::uint64_t overwrittenCount{ 0 };
        std::uint32_t copiedCount{ 0 };
        std::uint32_t flags{ 0 };
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
        RockProviderWeaponPartGrabModeV1 grabMode{ RockProviderWeaponPartGrabModeV1::None };
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
        // RockProviderWeaponActionRoleV1 for this semantic source.
        std::uint32_t actionRole{ 0 };
        RockProviderTransform sourceParentLocal{};
        RockProviderTransform weaponRootLocal{};
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
        RockProviderWeaponPartDriveApplicationV1 result{
            RockProviderWeaponPartDriveApplicationV1::Unknown
        };
        RockProviderTransform appliedSourceParentLocal{};
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
        RockProviderScopeActivationSourceV1 activationSource{
            RockProviderScopeActivationSourceV1::None
        };
        std::uint32_t nativeScopeOverlayIndex{ 0 };
        RockProviderPoint3 anchorWeaponLocal{};
        RockProviderBounds3 sightBoundsWeaponLocal{};
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
        RockProviderAuthoredGripSourceV1 source{ RockProviderAuthoredGripSourceV1::Unknown };
        std::uint64_t variantKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t flags{ 0 };
        std::uint16_t rightFingerLocalTransformMask{ 0 };
        std::uint16_t leftFingerLocalTransformMask{ 0 };
        RockProviderTransform rightHandInWeapon{};
        RockProviderTransform leftHandInWeapon{};
        RockProviderTransform rightFingerLocalTransforms[15]{};
        RockProviderTransform leftFingerLocalTransforms[15]{};
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderPresentedHandPoseV1
    {
        std::uint32_t size{ sizeof(RockProviderPresentedHandPoseV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        RockProviderTransform handWorld{};
        std::uint16_t fingerLocalTransformMask{ 0 };
        std::uint16_t reserved0{ 0 };
        RockProviderTransform fingerLocalTransforms[15]{};
        std::uint64_t presentationSequence{ 0 };
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
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t role{ 0 };
        std::uint32_t finger{ 0 };
        std::uint32_t segment{ 0 };
        std::uint32_t handBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t flags{ 0 };
        RockProviderSemanticContactStateV1 contactState{
            RockProviderSemanticContactStateV1::Continued
        };
        std::uint32_t framesSinceContact{ 0 };
        std::uint32_t contactSequence{ 0 };
        RockProviderPoint3 contactPointGame{};
        RockProviderPoint3 contactNormalGame{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderPlayerColliderDescriptorV1
    {
        std::uint32_t size{ sizeof(RockProviderPlayerColliderDescriptorV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        RockProviderPlayerColliderKindV1 kind{ RockProviderPlayerColliderKindV1::Hand };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t role{ 0 };
        RockProviderBodyZoneKind zone{ RockProviderBodyZoneKind::Unknown };
        RockProviderBodyZoneSide side{ RockProviderBodyZoneSide::Center };
        std::uint32_t descriptorIndex{ 0 };
        std::uint32_t flags{ 0 };
        float lengthGameUnits{ 0.0f };
        float radiusGameUnits{ 0.0f };
        RockProviderTransform transform{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderHandCollisionAvailabilityV1
    {
        std::uint32_t size{ sizeof(RockProviderHandCollisionAvailabilityV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
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

    struct RockProviderHandInputSuppressionStateV1
    {
        std::uint32_t size{ sizeof(RockProviderHandInputSuppressionStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t frameIndex{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t callerFlags{ 0 };
        std::uint32_t effectiveFlags{ 0 };
        std::uint32_t callerLeaseActive{ 0 };
        std::uint64_t callerExpiresAfterFrame{ 0 };
        std::uint32_t callerRemainingFrames{ 0 };
        RockProviderSuppressionInvalidationReasonV1 lastInvalidationReason{
            RockProviderSuppressionInvalidationReasonV1::None
        };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct RockProviderOffhandReservationRequestV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationRequestV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderOffhandReservation reservation{ RockProviderOffhandReservation::Normal };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    struct RockProviderOffhandReservationStateV1
    {
        std::uint32_t size{ sizeof(RockProviderOffhandReservationStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        RockProviderOffhandReservation reservation{ RockProviderOffhandReservation::Normal };
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
    struct RockProviderTouchGrabTargetV1
    {
        std::uint32_t size{ sizeof(RockProviderTouchGrabTargetV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        RockProviderTouchGrabKindV1 kind{
            RockProviderTouchGrabKindV1::FixedAnchor
        };
        std::uint32_t flags{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint64_t allowedLayerMask{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        RockProviderPoint3 pivotWorldGame{};
        RockProviderPoint3 axisWorldGame{};
        float minimumCoordinate{ 0.0f };
        float maximumCoordinate{ 1.0f };
        float currentCoordinate{ 0.0f };
        std::uint32_t reserved0{ 0 };
        std::uint64_t reserved[3]{};
    };

    struct RockProviderTouchGrabStateV1
    {
        std::uint32_t size{ sizeof(RockProviderTouchGrabStateV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        RockProviderTouchGrabKindV1 kind{
            RockProviderTouchGrabKindV1::FixedAnchor
        };
        RockProviderTouchGrabPhaseV1 phase{
            RockProviderTouchGrabPhaseV1::Inactive
        };
        RockProviderTouchGrabReleaseReasonV1 releaseReason{
            RockProviderTouchGrabReleaseReasonV1::None
        };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t activeHandMask{ 0 };
        std::uint32_t flags{ 0 };
        RockProviderSurfaceGripModeV1 surfaceGripMode{
            RockProviderSurfaceGripModeV1::CollisionAnchor
        };
        float currentCoordinate{ 0.0f };
        float coordinateVelocity{ 0.0f };
        RockProviderPoint3 contactPointGame{};
        RockProviderPoint3 contactNormalGame{};
        std::uint64_t frameIndex{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint64_t reserved[2]{};
    };


    enum class RockProviderPowerArmorPointV1 : std::uint32_t
    {
        None = 0,
        LeftArmorHand = 1,
        RightArmorHand = 2,
    };

    enum class RockProviderNativeOpenStateV1 : std::uint32_t
    {
        NotApplicable = 0, Open = 1, Opening = 2, Closed = 3, Closing = 4,
    };

    // Availability is independent of boolean values. Node names are descriptive,
    // not globally unique part IDs. All transforms use game units.
    enum class RockProviderTargetDetailFlagV1 : std::uint32_t
    {
        Reference = 1u << 0, Body = 1u << 1, Anchor = 1u << 2,
        Normal = 1u << 3, MeshPart = 1u << 4,
        ActivationBlocked = 1u << 5, OpenState = 1u << 6,
        FurnitureUse = 1u << 7, PowerArmorClassification = 1u << 8,
        PowerArmorFrame = 1u << 9, PowerArmorActor = 1u << 10,
    };

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
        RockProviderNativeOpenStateV1 openState{};
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
        RockProviderPoint3 anchorGame{};
        RockProviderPoint3 normalGame{};
        RockProviderPowerArmorPointV1 powerArmorPoint{};
        std::uint32_t sourceTriangleIndex{ 0xFFFF'FFFFu };
        char collisionNodeName[64]{};
        char meshPartName[64]{};
    };

    struct RockProviderPowerArmorPointPoseV1
    {
        RockProviderPowerArmorPointV1 point{};
        // Animated armor-bone origin, not a sampled human palm pose.
        std::uint32_t valid{ 0 };
        RockProviderTransform world{};
        RockProviderTransform frameLocal{};
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
        RockProviderHand hand{ RockProviderHand::None };
        RockProviderPowerArmorPointV1 point{};
        // Zero uses the native proximity radius. Maximum 32 game units.
        float maxDistanceGame{ 0.0f };
    };


using RockProviderFrameCallback = rock::api::core::FrameCallbackV1;
using RockProviderAnimationPhaseCallbackV1 = rock::api::core::PhaseCallbackV1;
}
