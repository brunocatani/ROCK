#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::collision {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Collision;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxBodyContacts = 128;
    inline constexpr std::uint32_t kMaxSemanticContacts = 20;
    inline constexpr std::uint32_t kMaxColliderDescriptors = 96;
    inline constexpr std::uint32_t kMaxRaycastsPerOwnerPerFrame = 8;
    inline constexpr std::uint32_t kMaxEjectionClearances = 8;
    inline constexpr std::uint32_t kMaxObjectRestWatches = 16;
    inline constexpr float kMaxRaycastDistanceGame = 8192.0f;
    inline constexpr std::uint32_t kNoBody = 0x7FFF'FFFF;

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
        // 16-23 were the retired leg and foot zones; never reuse them.
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
    // Hand and body contacts
    // ------------------------------------------------------------------

    enum class SemanticHandContactFlag : std::uint32_t
    {
        None = 0,
        ContactPointValid = 1u << 0,
        ContactNormalValid = 1u << 1,
        TargetFormResolved = 1u << 2,
        HeldObjectRelation = 1u << 3,
        CollisionAvailable = 1u << 4,
        TransitionSuppressed = 1u << 5,
    };

    enum class SemanticContactState : std::uint32_t
    {
        Begin = 1,
        Continued = 2,
        End = 3,
    };

    // One hand collider's contact: role, finger and segment name the part of
    // the hand that touched.
    struct SemanticHandContact
    {
        std::uint32_t size{ sizeof(SemanticHandContact) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t role{ 0 };
        std::uint32_t finger{ 0 };
        std::uint32_t segment{ 0 };
        std::uint32_t handBodyId{ kNoBody };
        std::uint32_t targetBodyId{ kNoBody };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t flags{ 0 };
        SemanticContactState contactState{ SemanticContactState::Continued };
        std::uint32_t framesSinceContact{ 0 };
        std::uint32_t contactSequence{ 0 };
        Point3 contactPointGame{};
        Point3 contactNormalGame{};
        std::uint32_t collisionGeneration{ 0 };
    };

    // A contact of one of the player's body colliders.
    struct BodyContact
    {
        std::uint32_t size{ sizeof(BodyContact) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t targetBodyId{ kNoBody };
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
        Point3 contactPointGame{};
    };

    // ------------------------------------------------------------------
    // Player colliders
    // ------------------------------------------------------------------

    enum class PlayerColliderKind : std::uint32_t
    {
        Hand = 1,
        Body = 2,
    };

    enum class PlayerColliderFlag : std::uint32_t
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

    struct PlayerColliderDescriptor
    {
        std::uint32_t size{ sizeof(PlayerColliderDescriptor) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        PlayerColliderKind kind{ PlayerColliderKind::Hand };
        Hand hand{ Hand::None };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t role{ 0 };
        BodyZoneKind zone{ BodyZoneKind::Unknown };
        BodyZoneSide side{ BodyZoneSide::Center };
        std::uint32_t descriptorIndex{ 0 };
        std::uint32_t flags{ 0 };
        float lengthGameUnits{ 0.0f };
        float radiusGameUnits{ 0.0f };
        Transform transform{};
        std::uint32_t collisionGeneration{ 0 };
    };

    enum class HandCollisionAvailabilityFlag : std::uint32_t
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

    struct HandCollisionAvailability
    {
        std::uint32_t size{ sizeof(HandCollisionAvailability) };
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
        std::uint32_t otherHandContactSlots{ 0 };
        std::uint32_t weaponContactSlots{ 0 };
        std::uint32_t dynamicCollisionLayer{ 0 };
        std::uint32_t pairSuppressionLeaseCount{ 0 };
        std::uint32_t collisionEnabledBodyCount{ 0 };
        std::uint32_t filterKnownBodyCount{ 0 };
    };

    // ------------------------------------------------------------------
    // World raycast
    // ------------------------------------------------------------------

    enum class WorldRaycastResultFlag : std::uint32_t
    {
        None = 0,
        Hit = 1u << 0,
        NormalValid = 1u << 1,
    };

    // Nonzero generation fields guard the request.
    struct WorldRaycastRequest
    {
        std::uint32_t size{ sizeof(WorldRaycastRequest) };
        std::uint32_t version{ 1 };
        Point3 startGame{};
        Point3 directionGame{};
        float maxDistanceGame{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // frameIndex and the generations are those at the time of the cast.
    struct WorldRaycastResult
    {
        std::uint32_t size{ sizeof(WorldRaycastResult) };
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
    };

    // ------------------------------------------------------------------
    // Environment
    // ------------------------------------------------------------------

    // The generated bodies of one weapon slot (bodyCount 0 without one).
    struct WeaponBodies
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyCount{ 0 };
        std::uint32_t bodyIds[8]{};
    };

    // Physics scale and each weapon slot's generated bodies for the
    // publication named by sample. Only slot 0 is published yet.
    struct Environment
    {
        std::uint32_t size{ sizeof(Environment) };
        std::uint32_t version{ 1 };
        Sample sample{};
        float gameToHavokScale{ 0.0f };
        float havokToGameScale{ 0.0f };
        std::uint32_t physicsScaleRevision{ 0 };
        WeaponBodies weapons[kMaxWeaponSlots]{};
    };

    // ------------------------------------------------------------------
    // Ejection clearance
    // ------------------------------------------------------------------

    enum class EjectionClearanceFlag : std::uint32_t
    {
        // Also exclude the object's contacts with both physical hands.
        IncludeHands = 1u << 0,
    };

    /*
     * For one second after the reference's bodies become available (up to
     * 1.5 s waiting for them to load), the reference does not collide with
     * the weapon in weaponSlot, nor with the hands under IncludeHands. World
     * and other objects still collide; ray queries and grabbing are
     * unaffected. reference names the object; expectedBase its base form;
     * weaponFormId and weaponGenerationKey witness the slot's weapon. Repeating the same request
     * observes it and never extends the deadline: RequestQueued while waiting,
     * Ok once active or completed. At most kMaxEjectionClearances references;
     * owner, generation or weapon changes revoke a clearance.
     */
    struct EjectionClearanceRequest
    {
        std::uint32_t size{ sizeof(EjectionClearanceRequest) };
        std::uint32_t version{ 1 };
        ReferenceKey reference{};
        std::uint32_t expectedBase{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t flags{ 0 };
    };

    // ------------------------------------------------------------------
    // Object rest
    // ------------------------------------------------------------------

    enum class ObjectRestState : std::uint32_t { Waiting, Watching, Resting };

    /*
     * Watches a dropped MISC reference until it rests. RequestQueued admits a
     * watch (at most kMaxObjectRestWatches); poll with the same request: Ok
     * reports Watching or Resting. Resting acknowledges completion and frees
     * the watch. Retry CapacityFull, NotReady or TargetUnavailable later. Held
     * items cannot rest. Two seconds of supported rest freezes the native
     * bodies and saves their final pose; ordinary grabbing wakes them. Owner,
     * lifecycle or scene loss cancels unfinished watches; submit again after a
     * regrab, drop or load. A completed placement is not undone.
     */
    struct ObjectRestRequest
    {
        std::uint32_t size{ sizeof(ObjectRestRequest) };
        std::uint32_t version{ 1 };
        ReferenceKey reference{};
        std::uint32_t expectedBase{ 0 };
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread (Read). Contacts no older than maxFramesSinceContact.
        Status(ROCK_CALL* copySemanticHandContacts)(OwnerToken owner, Hand hand, std::uint32_t maxFramesSinceContact, SemanticHandContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* copyPlayerColliderDescriptors)(OwnerToken owner, PlayerColliderDescriptor* outDescriptors, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* copyBodyContacts)(OwnerToken owner, BodyContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* getHandCollisionAvailability)(OwnerToken owner, Hand hand, HandCollisionAvailability* outState) noexcept;
        // Game thread (Read); at most kMaxRaycastsPerOwnerPerFrame per frame.
        Status(ROCK_CALL* queryWorldRaycast)(OwnerToken owner, const WorldRaycastRequest* request, WorldRaycastResult* outResult) noexcept;
        // Any thread (Read).
        Status(ROCK_CALL* getEnvironment)(OwnerToken owner, Environment* outEnvironment) noexcept;
        // Game thread; Write to request, owner only to cancel. reference 0
        // cancels all of the owner's clearances or watches.
        Status(ROCK_CALL* requestEjectionClearance)(OwnerToken owner, const EjectionClearanceRequest* request) noexcept;
        Status(ROCK_CALL* cancelEjectionClearance)(OwnerToken owner, std::uint32_t referenceFormId) noexcept;
        Status(ROCK_CALL* watchObjectRest)(OwnerToken owner, const ObjectRestRequest* request, ObjectRestState* outState) noexcept;
        Status(ROCK_CALL* cancelObjectRest)(OwnerToken owner, std::uint32_t referenceFormId) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}

#pragma pack(pop)
