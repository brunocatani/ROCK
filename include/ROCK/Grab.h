#pragma once
#include "Abi.h"
#include "Touch.h"

#pragma pack(push, 8)
namespace rock::api::grab {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Grab;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxQueuedCommands = 32;
    inline constexpr std::uint32_t kMaxCommandResults = 64;
    inline constexpr std::uint32_t kMaxReservationLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;
    inline constexpr std::uint32_t kPowerArmorPointCount = 2;
    inline constexpr float kMaxPowerArmorGrabDistanceGame = 32.0f;
    inline constexpr std::uint32_t kNoBody = 0x7FFF'FFFF;

    enum class HandStateFlag : std::uint32_t
    {
        None = 0,
        Touching = 1u << 0,
        Holding = 1u << 1,
        PhysicsDisabled = 1u << 2,
        Valid = 1u << 3,
    };

    // 5 (External) is retired with external body registration; never reuse.
    enum class BodyContactTargetKind : std::uint32_t
    {
        Unknown = 0,
        Hand = 1,
        Weapon = 2,
        HeldObject = 3,
        Body = 4,
        WorldSurface = 6,
        DynamicProp = 7,
        Actor = 8,
        QueryOnly = 9,
    };

    // ------------------------------------------------------------------
    // Interaction commands
    // ------------------------------------------------------------------

    enum class InteractionCommandKind : std::uint32_t
    {
        Unknown = 0,
        ForceGrab = 1,
        ForceRelease = 2,
        ThrownDrop = 3,
    };

    enum class InteractionCommandState : std::uint32_t
    {
        Unknown = 0,
        Queued = 1,
        Succeeded = 2,
        Rejected = 3,
        Cancelled = 4,
    };

    enum class InteractionFailure : std::uint32_t
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

    enum class CommandStage : std::uint32_t
    {
        Unknown = 0,
        Accepted = 1,
        Queued = 2,
        Committed = 3,
        Applied = 4,
        Terminal = 5,
    };

    enum class ForceGrabFlag : std::uint32_t
    {
        None = 0,
        UsePreferredGrabPointGame = 1u << 0,
    };

    enum class ForceReleaseFlag : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    enum class ThrownDropFlag : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    // The generation fields of a request guard it; zero skips that guard.
    struct ForceGrabRequest
    {
        std::uint32_t size{ sizeof(ForceGrabRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ kNoBody };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float maxDistanceGame{ 0.0f };
        float preferredGrabPointGame[3]{};
    };

    struct ForceReleaseRequest
    {
        std::uint32_t size{ sizeof(ForceReleaseRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ kNoBody };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
    };

    struct ThrownDropRequest
    {
        std::uint32_t size{ sizeof(ThrownDropRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ kNoBody };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
    };

    struct PowerArmorGrabRequest
    {
        std::uint32_t size{ sizeof(PowerArmorGrabRequest) };
        std::uint32_t version{ 1 };
        ReferenceKey target{};
        Hand hand{ Hand::None };
        PowerArmorPoint point{};
        float maxDistanceGame{ 0.0f };
    };

    // Puts an item of baseFormId from the player's inventory into the hand.
    struct InventoryGrabRequest
    {
        std::uint32_t size{ sizeof(InventoryGrabRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct InteractionCommandResult
    {
        std::uint32_t size{ sizeof(InteractionCommandResult) };
        std::uint32_t version{ 1 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        InteractionCommandKind kind{ InteractionCommandKind::Unknown };
        InteractionCommandState state{ InteractionCommandState::Unknown };
        InteractionFailure failure{ InteractionFailure::None };
        Hand hand{ Hand::None };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ kNoBody };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        CommandStage stage{ CommandStage::Unknown };
        InteractionFailure failureStage{ InteractionFailure::None };
        std::uint64_t acceptedFrame{ 0 };
        std::uint64_t committedFrame{ 0 };
        std::uint64_t appliedFrame{ 0 };
    };

    // ------------------------------------------------------------------
    // Hand interaction state
    // ------------------------------------------------------------------

    enum class HandInteractionPhase : std::uint32_t
    {
        Idle = 0,
        Touching = 1,
        Selecting = 2,
        Pulling = 3,
        Catching = 4,
        Holding = 5,
        Releasing = 6,
        StashCandidate = 7,
        ConsumeCandidate = 8,
    };

    // Primary and Offhand describe the hand's role for the primary weapon.
    enum class HandInteractionFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Primary = 1u << 1,
        Offhand = 1u << 2,
        LooseObject = 1u << 3,
        LooseWeapon = 1u << 4,
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
        NativeWeaponCarry = 1u << 22,
        RockGripActive = 1u << 23,
        AttachOnly = 1u << 24,
    };

    // A value copy of the latest publication. The record carries its own
    // frame, generations and per-hand sequences (state, target, grip,
    // release), which advance only when that part of the hand changes.
    struct HandInteractionState
    {
        std::uint32_t size{ sizeof(HandInteractionState) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        HandInteractionPhase phase{ HandInteractionPhase::Idle };
        BodyContactTargetKind targetKind{ BodyContactTargetKind::Unknown };
        std::uint32_t flags{ 0 };
        std::uint64_t reservedTargetIdentity{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t primaryBodyId{ kNoBody };
        std::uint32_t heldBodyCount{ 0 };
        std::uint32_t heldBodyIds[8]{};
        std::uint64_t stateSequence{ 0 };
        std::uint64_t targetSequence{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t releaseSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        Point3 surfaceAnchorGame{};
        touch::SurfaceGripMode surfaceGripMode{ touch::SurfaceGripMode::CollisionAnchor };
    };

    enum class TargetDetailFlag : std::uint32_t
    {
        Reference = 1u << 0,
        Body = 1u << 1,
        Anchor = 1u << 2,
        Normal = 1u << 3,
        MeshPart = 1u << 4,
        ActivationBlocked = 1u << 5,
        OpenState = 1u << 6,
        FurnitureUse = 1u << 7,
        PowerArmorClassification = 1u << 8,
        PowerArmorFrame = 1u << 9,
        PowerArmorActor = 1u << 10,
    };

    struct HandTargetDetails
    {
        std::uint32_t size{ sizeof(HandTargetDetails) };
        std::uint32_t version{ 1 };
        HandInteractionState handState{};
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t collisionLayer{ 0 };
        Point3 anchorGame{};
        Point3 normalGame{};
        PowerArmorPoint powerArmorPoint{};
        std::uint32_t sourceTriangleIndex{ 0xFFFF'FFFFu };
        char collisionNodeName[64]{};
        char meshPartName[64]{};
    };

    // ------------------------------------------------------------------
    // Hand reservation
    // ------------------------------------------------------------------

    enum class HandReservationKind : std::uint32_t
    {
        Normal = 0,
        // The hand may not take a support grip on the reserved weapon.
        ReloadReserved = 1,
        ReloadPoseOverride = 2,
    };

    /*
     * One owner reserves one hand at a time. The reserved hand must be the
     * offhand of the weapon in weaponSlot (HandUnavailable otherwise); while
     * the roles change, the reservation applies only while its hand is that
     * offhand. The lease runs 1 to kMaxReservationLeaseFrames frames from
     * acquire or renew. Expiry, generation changes and owner loss end it with
     * an Event of kind ReservationLost.
     */
    struct HandReservationRequest
    {
        std::uint32_t size{ sizeof(HandReservationRequest) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        Hand hand{ Hand::None };
        HandReservationKind reservation{ HandReservationKind::Normal };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct HandReservationState
    {
        std::uint32_t size{ sizeof(HandReservationState) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        HandReservationKind reservation{ HandReservationKind::Normal };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t active{ 0 };
        std::uint32_t remainingFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t weaponSlot{ 0 };
    };

    // ------------------------------------------------------------------
    // Held placement (right-stick click places a held loose object)
    // ------------------------------------------------------------------

    enum class HeldPlacementFlag : std::uint32_t { LooseObject = 1, FixedSurfaceContact = 2 };
    struct HeldPlacementHand
    {
        std::uint32_t formId{ 0 };
        std::uint32_t flags{ 0 };
    };
    enum class PlacementResult : std::uint32_t { None, Pending, Anchored, Rejected };

    // The physical right Axis0 click (32), sampled without consuming it.
    struct HeldPlacementState
    {
        std::uint32_t size{ sizeof(HeldPlacementState) };
        std::uint32_t version{ 1 };
        std::uint64_t frameToken{ 0 };
        Sample sample{};
        HeldPlacementHand right{};
        HeldPlacementHand left{};
        std::uint32_t buttonAvailable{ 0 };
        std::uint32_t buttonHeld{ 0 };
        std::uint32_t buttonPressed{ 0 };
        std::uint32_t buttonAgeMilliseconds{ 0 };
        std::uint64_t lastRequestToken{ 0 };
        PlacementResult lastResult{ PlacementResult::None };
        std::uint32_t lastRequestFormId{ 0 };
    };

    enum class PlacementIntentFlag : std::uint32_t { ReserveClick = 1, Anchor = 2 };

    /*
     * Read the state and submit its unchanged frameToken and sample in the
     * same phase (Animation BeforeRock preferred). The intent expires on the
     * next ROCK update; one owner per update (Busy for a competing owner).
     * formId is the add-on's eligible held reference, or zero. ReserveClick
     * covers the whole press and release; Anchor needs ReserveClick and a
     * nonzero formId. Acceptance is not completion: lastRequestToken and
     * lastResult report the native placement. ROCK validates both hands, the
     * reference's lifetime and its whole body set, releases the constraints
     * and saves a native fixed placement; static parts stay static. Pending
     * work cancels on lost intent, owner or world. Clearing or unregistering
     * never thaws a completed placement.
     */
    struct HeldPlacementIntent
    {
        std::uint32_t size{ sizeof(HeldPlacementIntent) };
        std::uint32_t version{ 1 };
        std::uint64_t frameToken{ 0 };
        Sample sample{};
        std::uint32_t formId{ 0 };
        std::uint32_t flags{ 0 };
    };

    // ------------------------------------------------------------------
    // Default grab poses per base form
    // ------------------------------------------------------------------

    enum class DefaultPoseFlag : std::uint32_t { Present = 1, FingerValues = 2, FingerJoints = 4 };

    // Pose relative to the physical hand's grab authority proxy: game-unit
    // translation, row-major proper orthonormal rotation, unit scale. Finger
    // order thumb, index, middle, ring, pinky; joints proximal to distal.
    // Thumb values accept [0,2], other fingers [0,1].
    struct DefaultHandPose
    {
        std::uint32_t flags{ 0 };
        float translateGame[3]{};
        float rotate[9]{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        float fingerValues[5]{};
        float fingerJointValues[15]{};
    };

    /*
     * Default poses fill missing per-hand saved offsets on ROCK's saved-offset
     * path (loose non-weapons and throwables); authored poses and firearm or
     * melee attachment keep priority. Present is required for any hand flags;
     * FingerJoints requires FingerValues; at least one hand must be present.
     * One owner per base form, 64 forms in all; replacing the same owner's
     * pair is atomic, a competing owner gets OwnerConflict, CapacityFull never
     * evicts. Changes affect later grabs only. Nothing is written to disk;
     * entries retire with their owner.
     */
    struct DefaultGrabPoses
    {
        std::uint32_t size{ sizeof(DefaultGrabPoses) };
        std::uint32_t version{ 1 };
        std::uint32_t baseFormId{ 0 };
        DefaultHandPose right{};
        DefaultHandPose left{};
    };

    // ------------------------------------------------------------------
    // Held object target
    // ------------------------------------------------------------------

    /*
     * Guides the currently held loose reference through ROCK's finite-force
     * grab constraint; it stays dynamic and collidable (no teleport, keyframe
     * or hide). rootWorld is in game units with Ni basis rows; its scale must
     * equal the reference's scale. Supply the current Grab sample (at most one
     * frame old) and gripSequence. One owner per hand; leaseFrames 1 to 2.
     * Release, a new grab, owner loss or expiry return the hand to ordinary
     * grabbing. Two-handed, fixed and loose-firearm holds are refused.
     * connectedWeaponFormId plus connectedWeaponGeneration, when nonzero, name
     * the weapon in connectedWeaponSlot: while renewed only its collision
     * pairs with this object are suppressed (insertion into a socket); zero
     * both to restore.
     */
    struct HeldObjectTarget
    {
        std::uint32_t size{ sizeof(HeldObjectTarget) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t leaseFrames{ 2 };
        std::uint32_t connectedWeaponSlot{ 0 };
        std::uint32_t connectedWeaponFormId{ 0 };
        Transform rootWorld{};
        Sample sample{};
        std::uint64_t gripSequence{ 0 };
        std::uint64_t connectedWeaponGeneration{ 0 };
    };

    // The solved rigid body pose reconstructed at the reference root, without
    // render prediction.
    struct HeldObjectPose
    {
        std::uint32_t size{ sizeof(HeldObjectPose) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t bodyId{ kNoBody };
        Sample sample{};
        std::uint64_t gripSequence{ 0 };
        Transform rootWorld{};
    };

    // ------------------------------------------------------------------
    // Reference queries
    // ------------------------------------------------------------------

    enum class NativeOpenState : std::uint32_t
    {
        NotApplicable = 0,
        Open = 1,
        Opening = 2,
        Closed = 3,
        Closing = 4,
    };

    struct ReferenceQuery
    {
        std::uint32_t size{ sizeof(ReferenceQuery) };
        std::uint32_t version{ 1 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
    };

    // frameIndex and worldGeneration are those at the time of the query.
    struct ReferenceInteraction
    {
        std::uint32_t size{ sizeof(ReferenceInteraction) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t baseFormType{ 0 };
        std::uint32_t activationBlocked{ 0 };
        NativeOpenState openState{ NativeOpenState::NotApplicable };
        std::uint32_t furnitureInUse{ 0 };
        std::uint32_t furnitureInUseIncludingReservations{ 0 };
        std::int32_t furnitureMarkerIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint64_t frameIndex{ 0 };
    };

    struct PowerArmorPointPose
    {
        PowerArmorPoint point{};
        std::uint32_t valid{ 0 };
        Transform world{};
        Transform frameLocal{};
    };

    struct PowerArmorTarget
    {
        std::uint32_t size{ sizeof(PowerArmorTarget) };
        std::uint32_t version{ 1 };
        ReferenceInteraction touchedReference{};
        ReferenceInteraction frameReference{};
        std::uint32_t flags{ 0 };
        std::uint32_t actorFormId{ 0 };
        PowerArmorPointPose points[kPowerArmorPointCount]{};
    };

    // ------------------------------------------------------------------
    // Events
    // ------------------------------------------------------------------

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct Event
    {
        std::uint32_t size{ sizeof(Event) };
        std::uint32_t kind{};
        std::uint64_t sequence{};
        std::uint64_t frameIndex{};
        std::uint64_t ownerToken{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        Hand hand{ Hand::None };
        std::uint32_t formId{};
        std::uint32_t primaryBodyId{ kNoBody };
        std::uint32_t secondaryBodyId{ kNoBody };
        std::uint32_t collisionLayer{};
        std::uint32_t flags{};
        std::uint32_t sourceKind{};
        std::uint64_t subjectSequence{};
        std::uint32_t commandKind{};
        std::uint32_t commandState{};
        std::uint32_t failure{};
        std::uint32_t phase{};
        float positionGame[3]{};
        float velocityGame[3]{};
        float mass{};
        float speedGameUnitsPerSecond{};
        float intensityHint{};
    };
    enum class EventKind : std::uint32_t
    {
        SelectionLocked = 1, PullStarted = 2, PullArrived = 3, PullCatchAttempt = 4, PullCatchSucceeded = 5,
        GrabCommitted = 6, HeldImpact = 7, Released = 8, TwoHandStarted = 9, TwoHandStopped = 10,
        StashCandidate = 11, ConsumeCandidate = 12, Stashed = 13, Consumed = 14, LootStarted = 15,
        LootCompleted = 16, SelectionUnlocked = 17,
        HandStateChanged = 100, CommandCompleted = 101, GrabNotification = 102, ReleaseNotification = 103,
        ReservationLost = 104,
    };

    // Synchronous observation on the producing thread. Calling ROCK inside
    // the callback is forbidden.
    using EventCallback = void(ROCK_CALL*)(const Event*, void*);

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread (Write). Commands queue; read the outcome with
        // getInteractionCommandResult or the CommandCompleted event.
        Status(ROCK_CALL* requestForceGrab)(OwnerToken owner, const ForceGrabRequest* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* requestForceRelease)(OwnerToken owner, const ForceReleaseRequest* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* requestThrownDrop)(OwnerToken owner, const ThrownDropRequest* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* requestPowerArmorGrab)(OwnerToken owner, const PowerArmorGrabRequest* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* requestInventoryGrab)(OwnerToken owner, const InventoryGrabRequest* request, std::uint64_t* outCommandId) noexcept;
        // Game thread; Read for the result, Write for cancel.
        Status(ROCK_CALL* getInteractionCommandResult)(OwnerToken owner, std::uint64_t commandId, InteractionCommandResult* outResult) noexcept;
        Status(ROCK_CALL* cancelInteractionCommand)(OwnerToken owner, std::uint64_t commandId) noexcept;
        // Any thread (Read), including F4SE tasks: a value copy that grants no
        // live pointer or write authority. Check Status and the Valid flag.
        Status(ROCK_CALL* getHandInteractionState)(OwnerToken owner, Hand hand, HandInteractionState* outState) noexcept;
        // Game thread (Read).
        Status(ROCK_CALL* getHandTargetDetails)(OwnerToken owner, Hand hand, HandTargetDetails* outDetails) noexcept;
        // Game thread; Write for acquire/renew/release, Read for the state.
        Status(ROCK_CALL* acquireHandReservation)(OwnerToken owner, const HandReservationRequest* request) noexcept;
        Status(ROCK_CALL* renewHandReservation)(OwnerToken owner, const HandReservationRequest* request) noexcept;
        Status(ROCK_CALL* releaseHandReservation)(OwnerToken owner) noexcept;
        Status(ROCK_CALL* getHandReservationState)(OwnerToken owner, HandReservationState* outState) noexcept;
        // Game thread; Read for the state, Write for submit/clear.
        Status(ROCK_CALL* getHeldPlacementState)(OwnerToken owner, HeldPlacementState* outState) noexcept;
        Status(ROCK_CALL* submitHeldPlacementIntent)(OwnerToken owner, const HeldPlacementIntent* intent) noexcept;
        Status(ROCK_CALL* clearHeldPlacementIntent)(OwnerToken owner) noexcept;
        // Game thread (Write). Clear is idempotent when absent.
        Status(ROCK_CALL* setDefaultGrabPoses)(OwnerToken owner, const DefaultGrabPoses* poses) noexcept;
        Status(ROCK_CALL* clearDefaultGrabPoses)(OwnerToken owner, std::uint32_t baseFormId) noexcept;
        // Game thread; Write for set/clear, Read for the pose. HandUnavailable
        // means the observed grab no longer owns that hand.
        Status(ROCK_CALL* setHeldObjectTarget)(OwnerToken owner, const HeldObjectTarget* target) noexcept;
        Status(ROCK_CALL* clearHeldObjectTarget)(OwnerToken owner, Hand hand) noexcept;
        Status(ROCK_CALL* getHeldObjectPose)(OwnerToken owner, Hand hand, HeldObjectPose* outPose) noexcept;
        // Game thread (Read).
        Status(ROCK_CALL* queryReferenceInteraction)(OwnerToken owner, const ReferenceQuery* query, ReferenceInteraction* outInteraction) noexcept;
        Status(ROCK_CALL* queryPowerArmorTarget)(OwnerToken owner, const ReferenceQuery* query, PowerArmorTarget* outTarget) noexcept;
        // Game thread (Read).
        Status(ROCK_CALL* setEventCallback)(OwnerToken owner, EventCallback callback, void* userData) noexcept;
        Status(ROCK_CALL* clearEventCallback)(OwnerToken owner) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
        // Any thread. Copies events after afterSequence.
        Status(ROCK_CALL* copyEvents)(OwnerToken owner, std::uint64_t afterSequence, Event* outEvents, std::uint32_t capacity, Stream* outStream) noexcept;
    };
}

#pragma pack(pop)
