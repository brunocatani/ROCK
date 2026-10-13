#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::grab {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Grab;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxQueuedCommands = 32;
    inline constexpr std::uint32_t kMaxCommandResults = 64;
    inline constexpr std::uint32_t kMaxReservationLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;
    inline constexpr float kMaxPowerArmorGrabDistanceGame = 32.0f;


    enum class HandStateFlag : std::uint32_t
    {
        None = 0,
        Touching = 1u << 0,
        Holding = 1u << 1,
        PhysicsDisabled = 1u << 2,
        Valid = 1u << 3,
    };

    enum class SurfaceGripModeV1 : std::uint32_t
    {
        CollisionAnchor = 0,
        MeshAnchor = 1,
        CollisionFallback = 2,
        AnimatedArmorBone = 3,
    };

    enum class OffhandReservation : std::uint32_t
    {
        Normal = 0,
        ReloadReserved = 1,
        ReloadPoseOverride = 2,
    };

    enum class BodyContactTargetKind : std::uint32_t
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

    enum class InteractionCommandKindV1 : std::uint32_t
    {
        Unknown = 0,
        ForceGrab = 1,
        ForceRelease = 2,
        ThrownDrop = 3,
    };

    enum class InteractionCommandStateV1 : std::uint32_t
    {
        Unknown = 0,
        Queued = 1,
        Succeeded = 2,
        Rejected = 3,
        Cancelled = 4,
    };

    enum class InteractionFailureV1 : std::uint32_t
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

    enum class ForceGrabFlagV1 : std::uint32_t
    {
        None = 0,
        UsePreferredGrabPointGame = 1u << 0,
    };

    enum class ForceReleaseFlagV1 : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    enum class ThrownDropFlagV1 : std::uint32_t
    {
        None = 0,
        ImmediateCollisionRestore = 1u << 0,
        RequireMatchingTarget = 1u << 1,
        UseVelocityHavok = 1u << 2,
    };

    enum class HandInteractionPhaseV1 : std::uint32_t
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

    enum class HandInteractionFlagV1 : std::uint32_t
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

    enum class CommandStageV1 : std::uint32_t
    {
        Unknown = 0,
        Accepted = 1,
        Queued = 2,
        Committed = 3,
        Applied = 4,
        Terminal = 5,
    };

    struct ForceGrabRequestV1
    {
        std::uint32_t size{ sizeof(ForceGrabRequestV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float maxDistanceGame{ 0.0f };
        float preferredGrabPointGame[3]{};
        std::uint32_t reserved[5]{};
    };

    struct ForceReleaseRequestV1
    {
        std::uint32_t size{ sizeof(ForceReleaseRequestV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        float linearVelocityHavok[3]{};
        float angularVelocityRadiansPerSecond[3]{};
        std::uint32_t reserved[1]{};
    };

    struct ThrownDropRequestV1
    {
        std::uint32_t size{ sizeof(ThrownDropRequestV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
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

    enum class TargetDetailFlagV1 : std::uint32_t
    {
        Reference = 1u << 0, Body = 1u << 1, Anchor = 1u << 2,
        Normal = 1u << 3, MeshPart = 1u << 4,
        ActivationBlocked = 1u << 5, OpenState = 1u << 6,
        FurnitureUse = 1u << 7, PowerArmorClassification = 1u << 8,
        PowerArmorFrame = 1u << 9, PowerArmorActor = 1u << 10,
    };

    struct PowerArmorGrabRequestV1
    {
        std::uint32_t size{ sizeof(PowerArmorGrabRequestV1) };
        std::uint32_t version{ 1 };
        ReferenceKey target{};
        Hand hand{ Hand::None };
        PowerArmorPointV1 point{};
        float maxDistanceGame{ 0.0f };
    };

    struct InteractionCommandResultV1
    {
        std::uint32_t size{ sizeof(InteractionCommandResultV1) };
        std::uint32_t version{ 1 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        InteractionCommandKindV1 kind{ InteractionCommandKindV1::Unknown };
        InteractionCommandStateV1 state{ InteractionCommandStateV1::Unknown };
        InteractionFailureV1 failure{ InteractionFailureV1::None };
        Hand hand{ Hand::None };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        CommandStageV1 stage{ CommandStageV1::Unknown };
        InteractionFailureV1 failureStage{ InteractionFailureV1::None };
        std::uint64_t acceptedFrame{ 0 };
        std::uint64_t committedFrame{ 0 };
        std::uint64_t appliedFrame{ 0 };
        std::uint32_t reserved{ 0 };
    };

    struct HandInteractionStateV1
    {
        std::uint32_t size{ sizeof(HandInteractionStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        HandInteractionPhaseV1 phase{ HandInteractionPhaseV1::Idle };
        BodyContactTargetKind targetKind{ BodyContactTargetKind::Unknown };
        std::uint32_t flags{ 0 };
        std::uint64_t reservedTargetIdentity{ 0 };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
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
        SurfaceGripModeV1 surfaceGripMode{
            SurfaceGripModeV1::CollisionAnchor
        };
    };

    struct OffhandReservationRequestV1
    {
        std::uint32_t size{ sizeof(OffhandReservationRequestV1) };
        std::uint32_t version{ 1 };
        OffhandReservation reservation{ OffhandReservation::Normal };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    struct OffhandReservationStateV1
    {
        std::uint32_t size{ sizeof(OffhandReservationStateV1) };
        std::uint32_t version{ 1 };
        OffhandReservation reservation{ OffhandReservation::Normal };
        std::uint32_t active{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t remainingFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct HandTargetDetailsV1
    {
        std::uint32_t size{ sizeof(HandTargetDetailsV1) };
        std::uint32_t version{ 1 };
        HandInteractionStateV1 handState{};
        std::uint32_t referenceFormId{};
        std::uint32_t referenceNativeHandle{};
        std::uint32_t flags{ 0 };
        std::uint32_t collisionLayer{ 0 };
        Point3 anchorGame{};
        Point3 normalGame{};
        PowerArmorPointV1 powerArmorPoint{};
        std::uint32_t sourceTriangleIndex{ 0xFFFF'FFFFu };
        char collisionNodeName[64]{};
        char meshPartName[64]{};
    };

    struct InventoryGrabRequestV1 { std::uint32_t size{sizeof(InventoryGrabRequestV1)}; Hand hand{Hand::None}; std::uint32_t baseFormId{}; std::uint32_t worldGeneration{}; std::uint32_t skeletonGeneration{}; std::uint32_t providerGeneration{}; };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1, Write=2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct EventV1 {
        std::uint32_t size{sizeof(EventV1)};
        std::uint32_t kind{};
        std::uint64_t sequence{};
        std::uint64_t frameIndex{};
        std::uint64_t ownerToken{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        Hand hand{Hand::None}; std::uint32_t formId{}; std::uint32_t primaryBodyId{0x7FFFFFFF}; std::uint32_t secondaryBodyId{0x7FFFFFFF}; std::uint32_t collisionLayer{}; std::uint32_t flags{}; std::uint32_t sourceKind{}; std::uint64_t subjectSequence{}; std::uint32_t commandKind{}; std::uint32_t commandState{}; std::uint32_t failure{}; std::uint32_t phase{}; float positionGame[3]{}; float velocityGame[3]{}; float mass{}; float speedGameUnitsPerSecond{}; float intensityHint{};
    };
    enum class EventKindV1 : std::uint32_t { SelectionLocked=1, PullStarted=2, PullArrived=3, PullCatchAttempt=4, PullCatchSucceeded=5, GrabCommitted=6, HeldImpact=7, Released=8, TwoHandStarted=9, TwoHandStopped=10, StashCandidate=11, ConsumeCandidate=12, Stashed=13, Consumed=14, LootStarted=15, LootCompleted=16, SelectionUnlocked=17, HandStateChanged=100, CommandCompleted=101, GrabNotification=102, ReleaseNotification=103, ReservationLost=104 };

    using EventCallbackV1=void(ROCK_CALL*)(const EventV1*,void*);
    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* requestForceGrabV1)(std::uint64_t ownerToken, const ForceGrabRequestV1* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* getInteractionCommandResultV1)(std::uint64_t ownerToken, std::uint64_t commandId, InteractionCommandResultV1* outResult) noexcept;
        Status(ROCK_CALL* requestForceReleaseV1)(std::uint64_t ownerToken, const ForceReleaseRequestV1* request, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* requestThrownDropV1)(std::uint64_t ownerToken, const ThrownDropRequestV1* request, std::uint64_t* outCommandId) noexcept;
        // Any-thread synchronized value snapshot, including F4SE tasks. Check
        // Status and Valid; the result grants no live pointer or write authority.
        Status(ROCK_CALL* getHandInteractionStateV1)(std::uint64_t ownerToken, Hand hand, HandInteractionStateV1* outState) noexcept;
        Status(ROCK_CALL* cancelInteractionCommandV1)(std::uint64_t ownerToken, std::uint64_t commandId) noexcept;
        Status(ROCK_CALL* acquireOffhandReservationV1)(std::uint64_t ownerToken, const OffhandReservationRequestV1* request) noexcept;
        Status(ROCK_CALL* renewOffhandReservationV1)(std::uint64_t ownerToken, const OffhandReservationRequestV1* request) noexcept;
        Status(ROCK_CALL* releaseOffhandReservationV1)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* getOffhandReservationStateV1)(std::uint64_t ownerToken, OffhandReservationStateV1* outState) noexcept;
        Status(ROCK_CALL* getHandTargetDetailsV1)(std::uint64_t ownerToken, Hand hand, HandTargetDetailsV1* outDetails) noexcept;
        Status(ROCK_CALL* requestPowerArmorGrabV1)(std::uint64_t ownerToken, const PowerArmorGrabRequestV1* request, std::uint64_t* outCommandId) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* requestInventoryGrab)(OwnerToken, const InventoryGrabRequestV1*, std::uint64_t* outCommandId) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
        // Synchronous producer-thread observation. ROCK calls are forbidden inside this callback.
        Status(ROCK_CALL* setEventCallback)(OwnerToken, EventCallbackV1, void*) noexcept;
        Status(ROCK_CALL* clearEventCallback)(OwnerToken) noexcept;
    };
}

#pragma pack(pop)
