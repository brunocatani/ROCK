#pragma once
#include "Abi.h"

// Weapon parts of the equipped weapons, addressed by weapon slot: contacts,
// evidence, part targets and drives, grip state, poses, sources, collision
// suppression, grab points, and authored state for loose weapon items.
#pragma pack(push, 8)
namespace rock::api::weaponparts {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::WeaponParts;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxEvidenceDetails = 100;
    inline constexpr std::uint32_t kMaxEvidencePoints = 252;
    inline constexpr std::uint32_t kMaxPoses = 128;
    inline constexpr std::uint32_t kMaxTargets = 128;
    inline constexpr std::uint32_t kMaxDrives = 64;
    inline constexpr std::uint32_t kMaxDriveLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxSources = 4096;
    inline constexpr std::uint32_t kMaxCollisionSources = 16;
    inline constexpr std::uint32_t kMaxGrabPoints = 32;
    inline constexpr std::uint32_t kMaxItemParts = 96;
    inline constexpr std::uint32_t kNoBody = 0x7FFF'FFFF;

    // ------------------------------------------------------------------
    // Part vocabulary
    // ------------------------------------------------------------------

    enum class PartKind : std::uint32_t
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

    enum class ActionRole : std::uint32_t
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

    enum class ClassificationSource : std::uint32_t
    {
        NameToken = 0,
        SlotAnchor = 1,
        RigAnchor = 2,
        AttachmentEvidence = 3,
    };

    // ------------------------------------------------------------------
    // Contacts and evidence
    // ------------------------------------------------------------------

    struct ContactQuery
    {
        std::uint32_t size{ sizeof(ContactQuery) };
        std::uint32_t version{ 1 };
        Point3 pointGame{};
        float radiusGame{ 0.0f };
        std::uint32_t flags{ 0 };
    };

    // The weapon part nearest the query point. Keys are opaque source keys of
    // the weapon generation.
    struct ContactResult
    {
        std::uint32_t size{ sizeof(ContactResult) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t valid{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint64_t interactionKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float probeDistanceGame{ 0.0f };
    };

    // One generated collider and how ROCK classified its part.
    struct EvidenceDetail
    {
        std::uint32_t size{ sizeof(EvidenceDetail) };
        std::uint32_t version{ 1 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t fallbackGripPose{ 0 };
        std::uint32_t pointCount{ 0 };
        std::uint64_t interactionKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        Bounds3 localBoundsGame{};
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        char sourceName[64]{};
    };

    // ------------------------------------------------------------------
    // Part targets and drives
    // ------------------------------------------------------------------

    enum class GrabMode : std::uint32_t
    {
        None = 0,
        FullTwoHandAuthority = 1,
        AttachOnly = 2,
    };

    enum class TargetFlag : std::uint32_t
    {
        None = 0,
        MatchBodyId = 1u << 0,
        MatchSourceKey = 1u << 1,
        MatchSourceName = 1u << 2,
        MatchPartKind = 1u << 3,
        MatchReloadRole = 1u << 4,
        MatchSupportRole = 1u << 5,
        MatchSocketRole = 1u << 6,
        MatchActionRole = 1u << 7,
        NonExclusive = 1u << 8,
    };

    // Selects which parts the owner's group may grab, and how.
    struct Target
    {
        std::uint32_t size{ sizeof(Target) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        GrabMode grabMode{ GrabMode::None };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        char sourceName[64]{};
    };

    enum class DriveSpace : std::uint32_t
    {
        WeaponRootLocal = 0,
        SourceParentLocal = 1,
    };

    // Poses one part source for 1..kMaxDriveLeaseFrames frames.
    struct DriveTarget
    {
        std::uint32_t size{ sizeof(DriveTarget) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        DriveSpace driveSpace{ DriveSpace::WeaponRootLocal };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t leaseFrames{ 1 };
        Transform targetTransform{};
        char sourceName[64]{};
    };

    struct ResolutionQuery
    {
        std::uint32_t size{ sizeof(ResolutionQuery) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint64_t sourceKey{ 0 };
        char sourceName[64]{};
    };

    // Which owner's target wins for the queried part.
    struct ResolutionResult
    {
        std::uint32_t size{ sizeof(ResolutionResult) };
        std::uint32_t version{ 1 };
        OwnerToken winningOwner{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        GrabMode grabMode{ GrabMode::None };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
    };

    enum class DriveApplication : std::uint32_t
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

    struct DriveResult
    {
        std::uint32_t size{ sizeof(DriveResult) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        OwnerToken owner{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        DriveApplication result{ DriveApplication::Unknown };
        Transform appliedSourceParentLocal{};
        char sourceName[64]{};
    };

    // ------------------------------------------------------------------
    // Grip state and poses
    // ------------------------------------------------------------------

    enum class GripKind : std::uint32_t
    {
        None = 0,
        FiringGrip = 1,
        SupportFullAuthority = 2,
        SupportVisualOnly = 3,
        PartCarry = 4,
        AttachOnly = 5,
    };

    enum class GripLocalSpace : std::uint32_t
    {
        WeaponRootLocal = 0,
        PartSourceLocal = 1,
    };

    // What one hand grips on which weapon slot's part.
    struct GripState
    {
        std::uint32_t size{ sizeof(GripState) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        GripKind gripKind{ GripKind::None };
        std::uint32_t active{ 0 };
        std::uint32_t attachOnly{ 0 };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint64_t sourceKey{ 0 };
        OwnerToken targetOwner{ 0 };
        std::uint32_t targetGroupId{ 0 };
        GrabMode targetGrabMode{ GrabMode::None };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        std::uint32_t authoredSupportGrip{ 0 };
        std::uint32_t hasHandPartLocal{ 0 };
        GripLocalSpace handPartLocalSpace{ GripLocalSpace::WeaponRootLocal };
        Transform handPartLocal{};
        char sourceName[64]{};
    };

    enum class PoseFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        SourceParentLocalValid = 1u << 1,
        WeaponRootLocalValid = 1u << 2,
    };

    struct PartPose
    {
        std::uint32_t size{ sizeof(PartPose) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t bodyId{ kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t actionRole{ 0 };
        Transform sourceParentLocal{};
        Transform weaponRootLocal{};
        char sourceName[64]{};
    };

    // ------------------------------------------------------------------
    // Sources
    // ------------------------------------------------------------------

    // One node of the weapon model. parentKey 0 is the weapon root.
    struct Source
    {
        std::uint32_t size{ sizeof(Source) };
        std::uint32_t version{ 1 };
        std::uint32_t bodyId{ kNoBody };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint64_t parentKey{ 0 };
        Transform sourceParentLocal{};
        char name[64]{};
    };

    struct SourcePose
    {
        std::uint32_t size{ sizeof(SourcePose) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t valid{ 0 };
        std::uint32_t weaponSlot{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        Transform sourceParentLocal{};
        Transform weaponRootLocal{};
        Transform world{};
    };

    // ------------------------------------------------------------------
    // Collision suppression
    // ------------------------------------------------------------------

    /*
     * Stops the weapon's generated bodies whose source is one of sourceKeys,
     * or a descendant, from colliding. Visibility, transforms and body
     * existence are unchanged. One owner per weapon; the owner's whole
     * selection is replaced atomically. 1..kMaxCollisionSources sources, lease
     * 1..2 frames, sample at most one frame old. Clear, expiry, owner,
     * generation or weapon loss restores the bodies. Renew each frame.
     */
    struct CollisionSuppression
    {
        std::uint32_t size{ sizeof(CollisionSuppression) };
        std::uint32_t version{ 1 };
        std::uint32_t leaseFrames{ 2 };
        std::uint32_t sourceCount{ 0 };
        Sample sample{};
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKeys[kMaxCollisionSources]{};
    };

    // ------------------------------------------------------------------
    // Grab points
    // ------------------------------------------------------------------

    /*
     * A point in a part source where a hand may take an AttachOnly target of
     * the owner's group. The palm or the index, middle or pinky finger probes
     * select the nearest point within both its radius and the hand-probe
     * radius; the hand seats at the nearest collider surface of the group's
     * part within the radius, using the currently driven pose, keeping the
     * approaching hand rotation, and the relation follows the moving part.
     * No matching surface means no acquisition. Configured groups never
     * acquire through whole-part surfaces. handMask: left 1, right 2.
     */
    struct GrabPoint
    {
        std::uint32_t size{ sizeof(GrabPoint) };
        std::uint32_t version{ 1 };
        std::uint32_t pointId{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t handMask{ 3 };
        float radiusGame{ 3.0f };
        Point3 pointInSource{};
    };

    // ------------------------------------------------------------------
    // Loose weapon item state
    // ------------------------------------------------------------------

    // Exact native item identity: weapon form, plus the inventory owner and
    // ExtraUniqueID, or the dropped reference and 0.
    struct ItemKey
    {
        std::uint32_t weapon{ 0 };
        std::uint32_t owner{ 0 };
        std::uint32_t unique{ 0 };
        bool operator==(const ItemKey&) const = default;
    };

    // path is relative to the model's Weapon node; local is the parent-local
    // transform. hidden applies to the subtree and its generated collision.
    struct ItemPart
    {
        char path[256]{};
        Transform local{};
        std::uint32_t hidden{ 0 };
    };

    struct ItemState
    {
        std::uint32_t size{ sizeof(ItemState) };
        std::uint32_t version{ 1 };
        std::uint32_t count{ 0 };
        ItemPart parts[kMaxItemParts]{};
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    /*
     * Per-weapon calls take weaponSlot < kMaxWeaponSlots (InvalidArgument
     * otherwise). A slot ROCK does not publish yet returns TargetUnavailable;
     * clearing it is a no-op. Copy calls report the total in outCount and copy
     * min(capacity, total). Game thread unless noted.
     */
    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Read.
        Status(ROCK_CALL* queryContact)(OwnerToken owner, std::uint32_t weaponSlot, const ContactQuery* query, ContactResult* outResult) noexcept;
        Status(ROCK_CALL* copyEvidenceDetails)(OwnerToken owner, std::uint32_t weaponSlot, EvidenceDetail* outDetails, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* copyEvidencePoints)(OwnerToken owner, std::uint32_t weaponSlot, std::uint32_t bodyId, Point3* outPoints, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        // Write. Replaces the owner's whole selection on the slot.
        Status(ROCK_CALL* setPartTargets)(OwnerToken owner, std::uint32_t weaponSlot, const Target* targets, std::uint32_t count) noexcept;
        Status(ROCK_CALL* clearPartTargets)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        Status(ROCK_CALL* setDriveTargets)(OwnerToken owner, std::uint32_t weaponSlot, const DriveTarget* targets, std::uint32_t count) noexcept;
        Status(ROCK_CALL* clearDriveTargets)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        // Read, any thread. The latest published grip of a hand.
        Status(ROCK_CALL* getGripState)(OwnerToken owner, Hand hand, GripState* outState) noexcept;
        // Read.
        Status(ROCK_CALL* queryTargetResolution)(OwnerToken owner, std::uint32_t weaponSlot, const ResolutionQuery* query, ResolutionResult* outResult) noexcept;
        Status(ROCK_CALL* copyPartPoses)(OwnerToken owner, std::uint32_t weaponSlot, PartPose* outPoses, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* copyDriveResults)(OwnerToken owner, std::uint32_t weaponSlot, DriveResult* outResults, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* copySources)(OwnerToken owner, std::uint32_t weaponSlot, std::uint64_t weaponGenerationKey, Source* outSources, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* querySourcePose)(OwnerToken owner, std::uint32_t weaponSlot, std::uint64_t weaponGenerationKey, std::uint64_t sourceKey, SourcePose* outPose) noexcept;
        Status(ROCK_CALL* querySourcePath)(OwnerToken owner, std::uint32_t weaponSlot, std::uint64_t weaponGenerationKey, std::uint64_t sourceKey, std::uint64_t* outParentKey, std::uint32_t* outChildIndex) noexcept;
        // Write.
        Status(ROCK_CALL* setCollisionSuppression)(OwnerToken owner, std::uint32_t weaponSlot, const CollisionSuppression* request) noexcept;
        Status(ROCK_CALL* clearCollisionSuppression)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        // Write. Register the AttachOnly targets first; 1..kMaxGrabPoints,
        // ids unique within a group, replaced atomically. A grip in progress
        // keeps its baseline.
        Status(ROCK_CALL* setGrabPoints)(OwnerToken owner, std::uint32_t weaponSlot, const GrabPoint* points, std::uint32_t count) noexcept;
        Status(ROCK_CALL* clearGrabPoints)(OwnerToken owner, std::uint32_t weaponSlot) noexcept;
        // Write, any thread (also native serialization callbacks). A state
        // stays until moved, removed (move to owner 0), cleared or its owner
        // is lost; republish after a load.
        Status(ROCK_CALL* setItemState)(OwnerToken owner, const ItemKey* key, const ItemState* state) noexcept;
        Status(ROCK_CALL* moveItemState)(OwnerToken owner, const ItemKey* from, const ItemKey* to) noexcept;
        Status(ROCK_CALL* clearItemStates)(OwnerToken owner) noexcept;
        // Any thread.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}
#pragma pack(pop)
