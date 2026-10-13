#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::WeaponParts;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 1;
    inline constexpr std::uint32_t kMaxEvidenceDetails = 100;
    inline constexpr std::uint32_t kMaxEvidencePoints = 252;
    inline constexpr std::uint32_t kMaxPoses = 128;
    inline constexpr std::uint32_t kMaxTargets = 128;
    inline constexpr std::uint32_t kMaxDrives = 64;
    inline constexpr std::uint32_t kMaxDriveLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxSources = 4096;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;


    enum class WeaponPartGrabModeV1 : std::uint32_t
    {
        None = 0,
        FullTwoHandAuthority = 1,
        AttachOnly = 2,
    };

    enum class WeaponPartTargetFlagV1 : std::uint32_t
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

    enum class WeaponPartDriveSpaceV1 : std::uint32_t
    {
        WeaponRootLocal = 0,
        SourceParentLocal = 1,
    };

    enum class WeaponPartGripKindV1 : std::uint32_t
    {
        None = 0,
        FiringGrip = 1,
        SupportFullAuthority = 2,
        SupportVisualOnly = 3,
        PartCarry = 4,
        AttachOnly = 5,
    };

    enum class WeaponPartDriveApplicationV1 : std::uint32_t
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

    enum class WeaponPartPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        SourceParentLocalValid = 1u << 1,
        WeaponRootLocalValid = 1u << 2,
    };

    enum class WeaponPartGripLocalSpaceV1 : std::uint32_t
    {
        WeaponRootLocal = 0,
        PartSourceLocal = 1,
    };

    enum class WeaponPartClassificationSourceV1 : std::uint32_t
    {
        NameToken = 0,
        SlotAnchor = 1,
        RigAnchor = 2,
        AttachmentEvidence = 3,
    };

    enum class WeaponPartKindV1 : std::uint32_t
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

    enum class WeaponActionRoleV1 : std::uint32_t
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

    struct WeaponContactQuery
    {
        std::uint32_t size{ sizeof(WeaponContactQuery) };
        float pointGame[3]{};
        float radiusGame{ 0.0f };
        std::uint32_t flags{ 0 };
        std::uint32_t reserved[2]{};
    };

    struct WeaponContactResult
    {
        std::uint32_t size{ sizeof(WeaponContactResult) };
        std::uint32_t valid{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint64_t interactionKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float probeDistanceGame{ 0.0f };
        std::uint32_t reserved{ 0 };
    };

    struct WeaponEvidenceDetailV1
    {
        std::uint32_t size{ sizeof(WeaponEvidenceDetailV1) };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t fallbackGripPose{ 0 };
        std::uint64_t interactionKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        Bounds3 localBoundsGame{};
        std::uint32_t pointCount{ 0 };
        char sourceName[64]{};
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct WeaponPartResolutionQueryV1
    {
        std::uint32_t size{ sizeof(WeaponPartResolutionQueryV1) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint64_t sourceKey{ 0 };
        char sourceName[64]{};
        std::uint32_t reserved[4]{};
    };

    struct WeaponPartPoseV1
    {
        std::uint32_t size{ sizeof(WeaponPartPoseV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{};
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t actionRole{ 0 };
        Transform sourceParentLocal{};
        Transform weaponRootLocal{};
        char sourceName[64]{};
        std::uint32_t reserved[4]{};
    };

    struct WeaponPartTargetV1
    {
        std::uint32_t size{ sizeof(WeaponPartTargetV1) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        WeaponPartGrabModeV1 grabMode{ WeaponPartGrabModeV1::None };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        char sourceName[64]{};
        std::uint32_t reserved[8]{};
    };

    struct WeaponPartDriveTargetV1
    {
        std::uint32_t size{ sizeof(WeaponPartDriveTargetV1) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        WeaponPartDriveSpaceV1 driveSpace{ WeaponPartDriveSpaceV1::WeaponRootLocal };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t sourceKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t leaseFrames{ 1 };
        Transform targetTransform{};
        char sourceName[64]{};
        std::uint32_t reserved[7]{};
    };

    struct WeaponPartGripStateV1
    {
        std::uint32_t size{ sizeof(WeaponPartGripStateV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        WeaponPartGripKindV1 gripKind{ WeaponPartGripKindV1::None };
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
        std::uint64_t sourceKey{ 0 };
        std::uint64_t providerOwnerToken{ 0 };
        std::uint32_t providerGroupId{ 0 };
        std::uint32_t providerGrabMode{ 0 };
        std::uint32_t hasHandPartLocal{ 0 };
        WeaponPartGripLocalSpaceV1 handPartLocalSpace{ WeaponPartGripLocalSpaceV1::WeaponRootLocal };
        Transform handPartLocal{};
        char sourceName[64]{};
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
        std::uint32_t authoredSupportGrip{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct WeaponPartResolutionResultV1
    {
        std::uint32_t size{ sizeof(WeaponPartResolutionResultV1) };
        std::uint32_t version{ 1 };
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        WeaponPartGrabModeV1 grabMode{ WeaponPartGrabModeV1::None };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        std::uint32_t reserved0{ 0 };
        std::uint64_t winningOwnerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct WeaponPartDriveApplicationResultV1
    {
        std::uint32_t size{ sizeof(WeaponPartDriveApplicationResultV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
        WeaponPartDriveApplicationV1 result{
            WeaponPartDriveApplicationV1::Unknown
        };
        Transform appliedSourceParentLocal{};
        char sourceName[64]{};
        std::uint32_t reserved[4]{};
    };

    struct SourceV1 { std::uint32_t size{sizeof(SourceV1)}; std::uint32_t bodyId{0x7FFFFFFF}; std::uint64_t weaponGenerationKey{}; std::uint64_t sourceKey{}; std::uint64_t parentKey{}; Transform sourceParentLocal{}; char name[64]{}; };

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
        std::uint32_t authority{}; std::uint32_t reason{};
    };
    enum class EventKindV1 : std::uint32_t { AuthorityLost=1 };

    struct SourcePoseV1 {
        std::uint32_t size{sizeof(SourcePoseV1)};
        std::uint32_t flags{};
        SampleV1 sample{};
        std::uint64_t weaponGenerationKey{};
        std::uint64_t sourceKey{};
        Transform sourceParentLocal{};
        Transform weaponRootLocal{};
        Transform world{};
    };
    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* queryWeaponContactAtPoint)(OwnerToken ownerToken, const WeaponContactQuery* query, WeaponContactResult* outResult) noexcept;
        Status(ROCK_CALL* getWeaponEvidenceDetailCountV1)(OwnerToken ownerToken, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* copyWeaponEvidenceDetailsV1)(OwnerToken ownerToken, WeaponEvidenceDetailV1* outDetails, std::uint32_t maxDetails, std::uint32_t* outCopied) noexcept;
        Status(ROCK_CALL* getWeaponEvidenceDetailPointCountV1)(OwnerToken ownerToken, std::uint32_t bodyId, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* copyWeaponEvidenceDetailPointsV1)(OwnerToken ownerToken, std::uint32_t bodyId, Point3* outPoints, std::uint32_t maxPoints, std::uint32_t* outCopied) noexcept;
        Status(ROCK_CALL* setWeaponPartTargetsV1)(std::uint64_t ownerToken, const WeaponPartTargetV1* targets, std::uint32_t targetCount) noexcept;
        Status(ROCK_CALL* clearWeaponPartTargetsV1)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* setWeaponPartDriveTargetsV1)(std::uint64_t ownerToken, const WeaponPartDriveTargetV1* targets, std::uint32_t targetCount) noexcept;
        Status(ROCK_CALL* clearWeaponPartDriveTargetsV1)(std::uint64_t ownerToken) noexcept;
        // Any-thread synchronized value snapshot, without live scene access.
        Status(ROCK_CALL* getWeaponPartGripStateV1)(OwnerToken ownerToken, Hand hand, WeaponPartGripStateV1* outState) noexcept;
        Status(ROCK_CALL* queryWeaponPartTargetResolutionV1)(std::uint64_t ownerToken, const WeaponPartResolutionQueryV1* query, WeaponPartResolutionResultV1* outResolution) noexcept;
        Status(ROCK_CALL* copyWeaponPartPoseSnapshotV1)(std::uint64_t ownerToken, WeaponPartPoseV1* outParts, std::uint32_t maxParts, std::uint32_t* outPartCount) noexcept;
        Status(ROCK_CALL* copyWeaponPartDriveApplicationResultsV1)(std::uint64_t ownerToken, WeaponPartDriveApplicationResultV1* outResults, std::uint32_t maxResults, std::uint32_t* outResultCount) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* copySources)(OwnerToken, std::uint64_t weaponGenerationKey, SourceV1*, std::uint32_t capacity, std::uint32_t* copied, std::uint32_t* total) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
        Status(ROCK_CALL* querySourcePose)(OwnerToken, std::uint64_t weaponGenerationKey, std::uint64_t sourceKey, SourcePoseV1*) noexcept;
        // Added in minor 1. Resolves native hierarchy position without exposing a node address.
        Status(ROCK_CALL* querySourcePath)(OwnerToken, std::uint64_t weaponGenerationKey, std::uint64_t sourceKey, std::uint64_t* parentKey, std::uint32_t* childIndex) noexcept;
    };
}

#pragma pack(pop)
