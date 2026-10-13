#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::collision {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Collision;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxExternalBodies = 2048;
    inline constexpr std::uint32_t kMaxExternalScopes = 256;
    inline constexpr std::uint32_t kMaxExternalContacts = 512;
    inline constexpr std::uint32_t kMaxBodyContacts = 128;
    inline constexpr std::uint32_t kMaxSemanticContacts = 20;
    inline constexpr std::uint32_t kMaxColliderDescriptors = 96;
    inline constexpr std::uint32_t kMaxRaycastsPerOwnerPerFrame = 8;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;
    inline constexpr float kMaxRaycastDistanceGame = 8192.0f;


    enum class ExternalBodyRole : std::uint32_t
    {
        Unknown = 0,
        ReloadMobile = 1,
        ReloadSocket = 2,
        ReloadAction = 3,
        ReloadVisualProxy = 4,
        ActorRagdollBone = 100,
    };

    enum class ExternalBodyContactPolicy : std::uint32_t
    {
        None = 0,
        ReportHandContacts = 1u << 0,
        ReportAllSourceKinds = 1u << 1,
        // 1u << 2 was SuppressRockDynamicPush, removed with ROCK's push
        // assist; never reuse the bit.
    };

    enum class ExternalSourceKind : std::uint32_t
    {
        Unknown = 0,
        Hand = 1,
        Weapon = 2,
        HeldObject = 3,
    };

    enum class ExternalContactQuality : std::uint32_t
    {
        BodyPairOnly = 0,
        AggregateImpulse = 1,
        RawPoint = 2,
    };

    enum class BodyZoneSide : std::uint32_t
    {
        Center = 0,
        Left = 1,
        Right = 2,
    };

    enum class BodyZoneKind : std::uint32_t
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

    enum class ExternalContactFlagV1 : std::uint32_t
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

    enum class ExternalContactStreamFlagV1 : std::uint32_t
    {
        None = 0,
        GapBeforeFirstCopied = 1u << 0,
        RingOverwroteRecords = 1u << 1,
    };

    enum class SemanticHandContactFlagV1 : std::uint32_t
    {
        None = 0,
        ContactPointValid = 1u << 0,
        ContactNormalValid = 1u << 1,
        TargetFormResolved = 1u << 2,
        HeldObjectRelation = 1u << 3,
        CollisionAvailable = 1u << 4,
        TransitionSuppressed = 1u << 5,
    };

    enum class SemanticContactStateV1 : std::uint32_t
    {
        Begin = 1,
        Continued = 2,
        End = 3,
    };

    enum class PlayerColliderKindV1 : std::uint32_t
    {
        Hand = 1,
        Body = 2,
    };

    enum class PlayerColliderFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Enabled = 1u << 1,
        PrimaryPalmAnchor = 1u << 2,
        TransformValid = 1u << 3,
        InPowerArmor = 1u << 4,
        LifecycleAllowed = 1u << 5,
        FilterKnown = 1u << 6,
        CollisionSuppressed = 1u << 7,
    };

    enum class HandCollisionAvailabilityFlagV1 : std::uint32_t
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

    enum class WorldRaycastResultFlagV1 : std::uint32_t
    {
        None = 0,
        Hit = 1u << 0,
        NormalValid = 1u << 1,
    };

    struct WorldRaycastRequestV1
    {
        std::uint32_t size{ sizeof(WorldRaycastRequestV1) };
        std::uint32_t version{ 1 };
        Point3 startGame{};
        Point3 directionGame{};
        float maxDistanceGame{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[8]{};
    };

    struct WorldRaycastResultV1
    {
        std::uint32_t size{ sizeof(WorldRaycastResultV1) };
        std::uint32_t version{ 1 };
        std::uint32_t hit{ 0 };
        std::uint32_t flags{ 0 };
        float hitFraction{ 1.0f };
        float hitDistanceGame{ 0.0f };
        Point3 hitPointGame{};
        Point3 hitNormalGame{};
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    struct ExternalContactStreamStateV1
    {
        std::uint32_t size{ sizeof(ExternalContactStreamStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t oldestRetainedSequence{ 0 };
        std::uint64_t latestEmittedSequence{ 0 };
        std::uint64_t firstCopiedSequence{ 0 };
        std::uint64_t lastCopiedSequence{ 0 };
        std::uint64_t overwrittenCount{ 0 };
        std::uint32_t copiedCount{ 0 };
        std::uint32_t flags{ 0 };
    };

    struct HandCollisionAvailabilityV1
    {
        std::uint32_t size{ sizeof(HandCollisionAvailabilityV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t collisionSequence{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t handBodyCount{ 0 };
        std::uint32_t dynamicTwinCount{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t otherHandContactSlots{};
        std::uint32_t weaponContactSlots{};
        std::uint32_t dynamicCollisionLayer{};
        std::uint32_t pairSuppressionLeaseCount{};
        std::uint32_t collisionEnabledBodyCount{ 0 };
        std::uint32_t filterKnownBodyCount{ 0 };
    };

    struct BodyContactV1
    {
        std::uint32_t size{ sizeof(BodyContactV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t bodyLayer{ 0xFFFF'FFFF };
        std::uint32_t targetLayer{ 0xFFFF'FFFF };
        BodyZoneKind zone{ BodyZoneKind::Unknown };
        BodyZoneSide side{ BodyZoneSide::Center };
        std::uint32_t role{ 0 };
        std::uint32_t descriptorIndex{ 0 };
        BodyContactTargetKind targetKind{ BodyContactTargetKind::Unknown };
        BodyZoneKind targetZone{ BodyZoneKind::Unknown };
        BodyZoneSide targetSide{ BodyZoneSide::Center };
        std::uint32_t targetRole{ 0 };
        std::uint32_t targetDescriptorIndex{ 0 };
        std::uint32_t inPowerArmor{ 0 };
        std::uint32_t targetInPowerArmor{ 0 };
        std::uint32_t hasContactPointGame{ 0 };
        std::uint32_t reserved0{ 0 };
        Point3 contactPointGame{};
        std::uint32_t reserved[8]{};
    };

    struct ExternalBodyRegistration
    {
        std::uint32_t size{ sizeof(ExternalBodyRegistration) };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t worldGeneration{};
        std::uint32_t generation{ 0 };
        ExternalBodyRole role{ ExternalBodyRole::Unknown };
        ExternalBodyContactPolicy contactPolicy{ ExternalBodyContactPolicy::None };
        Hand ownerHand{ Hand::None };
    };

    struct ExternalContactRecordV1
    {
        std::uint32_t size{ sizeof(ExternalContactRecordV1) };
        std::uint32_t version{ 1 };
        std::uint64_t parentOwnerToken{ 0 };
        std::uint64_t scopeToken{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t sourceBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetExternalBodyId{ 0x7FFF'FFFF };
        std::uint32_t bodyGeneration{ 0 };
        ExternalSourceKind sourceKind{ ExternalSourceKind::Unknown };
        Hand sourceHand{ Hand::None };
        ExternalBodyRole targetRole{ ExternalBodyRole::Unknown };
        ExternalContactQuality quality{ ExternalContactQuality::BodyPairOnly };
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

    struct SemanticHandContactV1
    {
        std::uint32_t size{ sizeof(SemanticHandContactV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t role{ 0 };
        std::uint32_t finger{ 0 };
        std::uint32_t segment{ 0 };
        std::uint32_t handBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t flags{ 0 };
        SemanticContactStateV1 contactState{
            SemanticContactStateV1::Continued
        };
        std::uint32_t framesSinceContact{ 0 };
        std::uint32_t contactSequence{ 0 };
        Point3 contactPointGame{};
        Point3 contactNormalGame{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct PlayerColliderDescriptorV1
    {
        std::uint32_t size{ sizeof(PlayerColliderDescriptorV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        PlayerColliderKindV1 kind{ PlayerColliderKindV1::Hand };
        Hand hand{ Hand::None };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t role{ 0 };
        BodyZoneKind zone{ BodyZoneKind::Unknown };
        BodyZoneSide side{ BodyZoneSide::Center };
        std::uint32_t descriptorIndex{ 0 };
        std::uint32_t flags{ 0 };
        float lengthGameUnits{ 0.0f };
        float radiusGameUnits{ 0.0f };
        Transform transform{};
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct EnvironmentV1 { std::uint32_t size{sizeof(EnvironmentV1)}; std::uint32_t externalBodyCount{}; SampleV1 sample{}; float gameToHavokScale{}; float havokToGameScale{}; std::uint32_t physicsScaleRevision{}; std::uint32_t weaponBodyCount{}; std::uint64_t weaponGenerationKey{}; std::uint32_t weaponBodyIds[8]{}; };

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
        Hand hand{Hand::None}; std::uint32_t formId{}; std::uint32_t collisionLayer{};
    };
    enum class EventKindV1 : std::uint32_t { Touch=1, TouchEnd=2 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* clearExternalBodies)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* getBodyContactSnapshotV1)(OwnerToken ownerToken, BodyContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outCopied) noexcept;
        Status(ROCK_CALL* registerExternalBodiesForScopeV1)(std::uint64_t ownerToken, std::uint64_t scopeToken, const ExternalBodyRegistration* bodies, std::uint32_t bodyCount) noexcept;
        Status(ROCK_CALL* clearExternalBodiesForScopeV1)(std::uint64_t ownerToken, std::uint64_t scopeToken) noexcept;
        Status(ROCK_CALL* copyExternalContactsSinceV1)(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t afterSequence, ExternalContactRecordV1* outContacts, std::uint32_t maxContacts, ExternalContactStreamStateV1* outStreamState) noexcept;
        Status(ROCK_CALL* copySemanticHandContactsV1)(std::uint64_t ownerToken, Hand hand, std::uint32_t maxFramesSinceContact, SemanticHandContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outContactCount) noexcept;
        Status(ROCK_CALL* copyPlayerColliderDescriptorsV1)(std::uint64_t ownerToken, PlayerColliderDescriptorV1* outDescriptors, std::uint32_t maxDescriptors, std::uint32_t* outDescriptorCount) noexcept;
        Status(ROCK_CALL* getHandCollisionAvailabilityV1)(std::uint64_t ownerToken, Hand hand, HandCollisionAvailabilityV1* outState) noexcept;
        Status(ROCK_CALL* queryWorldRaycastV1)(std::uint64_t ownerToken, const WorldRaycastRequestV1* request, WorldRaycastResultV1* outResult) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        // Any-thread copy; payload and SampleV1 identify the same publication.
        Status(ROCK_CALL* getEnvironment)(OwnerToken, EnvironmentV1*) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)
