#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef NOMMNOSOUND
#define NOMMNOSOUND
#endif

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#if !defined(_WINDOWS_) && !defined(_INC_WINDOWS)
struct HINSTANCE__;
using HMODULE = HINSTANCE__*;
using LPCSTR = const char*;
using FARPROC = std::intptr_t(__stdcall*)();
extern "C" __declspec(dllimport) HMODULE __stdcall GetModuleHandleA(LPCSTR lpModuleName);
extern "C" __declspec(dllimport) FARPROC __stdcall GetProcAddress(HMODULE hModule, LPCSTR lpProcName);
#endif

namespace rock::provider
{
#if defined(ROCK_API_EXPORTS)
#define ROCK_PROVIDER_API extern "C" __declspec(dllexport)
#else
#define ROCK_PROVIDER_API extern "C" __declspec(dllimport)
#endif

#define ROCK_PROVIDER_CALL __cdecl

    inline constexpr std::uint32_t ROCK_PROVIDER_API_VERSION = 1;
    inline constexpr std::uint32_t ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE = 256;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_BODIES = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V1 = 2048;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1 = 512;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_BODY_CONTACTS_V1 = 128;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V1 = 16;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_CONSUMERS_V1 = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 = 32;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1 = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1 = 32;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1 = 120;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1 = 128;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1 = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1 = 120;

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
        Holding = 1u << 1,
        PhysicsDisabled = 1u << 2,
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
    };

    enum class RockProviderFeatureBitV1 : std::uint32_t
    {
        None = 0,
        FrameCallbacks = 1u << 0,
        LifecycleFields = 1u << 1,
        HandFrames = 1u << 2,
        WeaponEvidence = 1u << 3,
        BodyContacts = 1u << 4,
        ExternalContacts = 1u << 5,
        ConsumerRegistrationV1 = 1u << 8,
        OwnerFilteredExternalContactsV1 = 1u << 9,
        InteractionCommandQueue = 1u << 10,
        ForceGrabCommand = 1u << 11,
        ForceReleaseCommand = 1u << 12,
        ThrownDropCommand = 1u << 13,
        HandInputSuppression = 1u << 14,
        WeaponPartInteraction = 1u << 15,
        WeaponPartGripState = 1u << 16,
        WeaponPartRecordIdentity = 1u << 17,
        WeaponPartTargetNonExclusive = 1u << 18,
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
     * AttachOnly: whitelist-mandated glue — the hand follows the part
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
     * Coarse physical size class for the currently equipped weapon. ROCK uses
     * this to pick a generated-collision max-distance-from-origin budget;
     * exposed here so other providers (reload/scope logic) don't need to
     * reimplement weapon-size classification.
     */
    enum class RockProviderWeaponSizeClassV1 : std::uint32_t
    {
        Melee = 0,
        Pistol = 1,
        Rifle = 2,
        Heavy = 3,
    };

    /*
     * Fallout4.esm's WeaponType* keyword tagging is reliable on vanilla weapons
     * but author-discretion on mods (verified directly: of two installed real
     * pistol mods, one tags every weapon, the other tags none). This records
     * which signal actually produced the size class so a consumer can tell a
     * confident keyword match from a weight-based guess.
     */
    enum class RockProviderWeaponClassificationSourceV1 : std::uint32_t
    {
        None = 0,
        Keyword = 1,
        WeightFallback = 2,
        Default = 3,
    };

    /*
     * One bit per Fallout4.esm WeaponType* keyword found on the equipped
     * weapon's own form. A bitmask rather than a single value because vanilla
     * weapons can legitimately carry more than one bucket keyword at once
     * (e.g. CombatShotgun carries both Rifle, the grip/animation category, and
     * Shotgun, the specific family) - callers that need the more specific tag
     * (e.g. a future reload/scope mod picking a shotgun-specific animation)
     * should prefer the most specific flag present rather than assuming
     * mutual exclusivity.
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

    [[nodiscard]] inline constexpr bool hasLifecycleFlag(std::uint32_t flags, RockProviderLifecycleFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasConsumerCapabilityV1(std::uint32_t capabilities, RockProviderConsumerCapabilityV1 capability)
    {
        return (capabilities & static_cast<std::uint32_t>(capability)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasFeatureBitV1(std::uint32_t featureBits, RockProviderFeatureBitV1 feature)
    {
        return (featureBits & static_cast<std::uint32_t>(feature)) != 0;
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

    struct RockProviderLimitsV1
    {
        std::uint32_t size{ sizeof(RockProviderLimitsV1) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t featureBits{ 0 };
        std::uint32_t maxFrameCallbacks{ 0 };
        std::uint32_t maxConsumers{ 0 };
        std::uint32_t maxExternalBodies{ 0 };
        std::uint32_t maxExternalContacts{ 0 };
        std::uint32_t maxBodyContacts{ 0 };
        std::uint32_t maxWeaponBodies{ 0 };
        std::uint32_t maxInteractionCommands{ 0 };
        std::uint32_t maxCompletedInteractionCommands{ 0 };
        std::uint32_t providerApiByteSize{ 0 };
        std::uint32_t reserved[6]{};
    };

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
        std::uint32_t reserved0{ 0 };
        std::uint32_t reserved[8]{};
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
        std::uint32_t reserved[8]{};
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

    struct RockProviderWeaponPartTargetQueryV1
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
    };

    struct RockProviderWeaponPartTargetResolutionV1
    {
        std::uint32_t whitelistActive{ 0 };
        std::uint32_t matched{ 0 };
        RockProviderWeaponPartGrabModeV1 grabMode{ RockProviderWeaponPartGrabModeV1::None };
        std::uint32_t groupId{ 0 };
        std::uint64_t ownerToken{ 0 };
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
     * node (bolt, magazine display) supplied the function.
     */
    enum class RockProviderWeaponPartClassificationSourceV1 : std::uint32_t
    {
        NameToken = 0,
        SlotAnchor = 1,
        RigAnchor = 2,
    };

    /*
     * Numeric contract for the partKind / actionRole payloads that already
     * flow through weapon evidence details, part targets, drive targets, and
     * grip states as raw uint32 values. Values mirror ROCK's internal
     * classification enums one-to-one (static_asserted inside ROCK, so drift
     * breaks ROCK's build, never a consumer at runtime). External consumers
     * use these to build part whitelists and gate grips without including
     * ROCK internals.
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
        std::uint32_t reserved[6]{};
    };

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
     * from. Consumers that only need the collider-style bucket can read
     * sizeClass directly; consumers that need finer distinctions (e.g. a future
     * reload/scope mod picking a shotgun- or minigun-specific behavior) can
     * inspect keywordFlags with hasWeaponKeywordFlagV1.
     */
    struct RockProviderWeaponClassificationV1
    {
        std::uint32_t size{ sizeof(RockProviderWeaponClassificationV1) };
        std::uint32_t valid{ 0 };
        std::uint64_t keywordFlags{ 0 };
        RockProviderWeaponSizeClassV1 sizeClass{ RockProviderWeaponSizeClassV1::Rifle };
        RockProviderWeaponClassificationSourceV1 source{ RockProviderWeaponClassificationSourceV1::None };
        std::uint32_t formId{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct RockProviderPoint3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct RockProviderBounds3
    {
        RockProviderPoint3 min{};
        RockProviderPoint3 max{};
        std::uint32_t valid{ 0 };
        std::uint32_t reserved{ 0 };
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
        std::uint32_t reserved[2]{};
    };

    using RockProviderFrameCallback = void(ROCK_PROVIDER_CALL*)(const RockProviderFrameSnapshot* snapshot, void* userData);

    struct RockProviderApi
    {
        static constexpr auto ROCK_F4SE_MOD_NAME = "ROCK";

        std::uint32_t(ROCK_PROVIDER_CALL* getVersion)();
        const char*(ROCK_PROVIDER_CALL* getModVersion)();
        bool(ROCK_PROVIDER_CALL* isProviderReady)();
        std::uint64_t(ROCK_PROVIDER_CALL* registerFrameCallback)(RockProviderFrameCallback callback, void* userData);
        bool(ROCK_PROVIDER_CALL* unregisterFrameCallback)(std::uint64_t callbackToken);
        bool(ROCK_PROVIDER_CALL* getFrameSnapshot)(RockProviderFrameSnapshot* outSnapshot);
        bool(ROCK_PROVIDER_CALL* queryWeaponContactAtPoint)(const RockProviderWeaponContactQuery* query, RockProviderWeaponContactResult* outResult);
        void(ROCK_PROVIDER_CALL* clearExternalBodies)(std::uint64_t ownerToken);
        bool(ROCK_PROVIDER_CALL* setOffhandInteractionReservation)(std::uint64_t ownerToken, RockProviderOffhandReservation reservation);
        bool(ROCK_PROVIDER_CALL* registerExternalBodiesV1)(
            std::uint64_t ownerToken,
            const RockProviderExternalBodyRegistration* bodies,
            std::uint32_t bodyCount);
        std::uint32_t(ROCK_PROVIDER_CALL* getWeaponEvidenceDetailCountV1)();
        std::uint32_t(ROCK_PROVIDER_CALL* copyWeaponEvidenceDetailsV1)(RockProviderWeaponEvidenceDetailV1* outDetails, std::uint32_t maxDetails);
        std::uint32_t(ROCK_PROVIDER_CALL* getWeaponEvidenceDetailPointCountV1)(std::uint32_t bodyId);
        std::uint32_t(ROCK_PROVIDER_CALL* copyWeaponEvidenceDetailPointsV1)(
            std::uint32_t bodyId,
            RockProviderPoint3* outPoints,
            std::uint32_t maxPoints);
        std::uint32_t(ROCK_PROVIDER_CALL* getBodyContactSnapshotV1)(RockProviderBodyContactV1* outContacts, std::uint32_t maxContacts);
        RockProviderHand(ROCK_PROVIDER_CALL* getPrimaryHandV1)();
        RockProviderHand(ROCK_PROVIDER_CALL* getOffhandHandV1)();
        bool(ROCK_PROVIDER_CALL* getHandFrameV1)(RockProviderHand hand, RockProviderHandFrameV1* outFrame);
        RockProviderResultV1(ROCK_PROVIDER_CALL* registerConsumerV1)(
            const RockProviderConsumerRegistrationV1* registration,
            RockProviderConsumerHandleV1* outHandle);
        RockProviderResultV1(ROCK_PROVIDER_CALL* unregisterConsumerV1)(std::uint64_t ownerToken);
        std::uint32_t(ROCK_PROVIDER_CALL* getGrantedCapabilitiesV1)(std::uint64_t ownerToken);
        bool(ROCK_PROVIDER_CALL* getProviderLimitsV1)(RockProviderLimitsV1* outLimits);
        std::uint32_t(ROCK_PROVIDER_CALL* getExternalContactSnapshotForOwnerV1)(
            std::uint64_t ownerToken,
            RockProviderExternalContactV1* outContacts,
            std::uint32_t maxContacts);
        RockProviderResultV1(ROCK_PROVIDER_CALL* requestForceGrabV1)(
            std::uint64_t ownerToken,
            const RockProviderForceGrabRequestV1* request,
            std::uint64_t* outCommandId);
        RockProviderResultV1(ROCK_PROVIDER_CALL* getInteractionCommandResultV1)(
            std::uint64_t ownerToken,
            std::uint64_t commandId,
            RockProviderInteractionCommandResultV1* outResult);
        RockProviderResultV1(ROCK_PROVIDER_CALL* requestForceReleaseV1)(
            std::uint64_t ownerToken,
            const RockProviderForceReleaseRequestV1* request,
            std::uint64_t* outCommandId);
        RockProviderResultV1(ROCK_PROVIDER_CALL* requestThrownDropV1)(
            std::uint64_t ownerToken,
            const RockProviderThrownDropRequestV1* request,
            std::uint64_t* outCommandId);
        RockProviderResultV1(ROCK_PROVIDER_CALL* setHandInputSuppressionV1)(
            std::uint64_t ownerToken,
            const RockProviderHandInputSuppressionRequestV1* request);
        RockProviderResultV1(ROCK_PROVIDER_CALL* clearHandInputSuppressionV1)(
            std::uint64_t ownerToken,
            RockProviderHand hand);
        RockProviderResultV1(ROCK_PROVIDER_CALL* setWeaponPartTargetsV1)(
            std::uint64_t ownerToken,
            const RockProviderWeaponPartTargetV1* targets,
            std::uint32_t targetCount);
        RockProviderResultV1(ROCK_PROVIDER_CALL* clearWeaponPartTargetsV1)(std::uint64_t ownerToken);
        RockProviderResultV1(ROCK_PROVIDER_CALL* setWeaponPartDriveTargetsV1)(
            std::uint64_t ownerToken,
            const RockProviderWeaponPartDriveTargetV1* targets,
            std::uint32_t targetCount);
        RockProviderResultV1(ROCK_PROVIDER_CALL* clearWeaponPartDriveTargetsV1)(std::uint64_t ownerToken);
        bool(ROCK_PROVIDER_CALL* queryEquippedWeaponClassificationV1)(RockProviderWeaponClassificationV1* outResult);
        bool(ROCK_PROVIDER_CALL* getWeaponPartGripStateV1)(RockProviderHand hand, RockProviderWeaponPartGripStateV1* outState);

        [[nodiscard]] static int initialize(
            const std::uint32_t minVersion = ROCK_PROVIDER_API_VERSION,
            const std::uint32_t minProviderApiByteSize = 0)
        {
            const auto hasMinimumTableSize = [](const RockProviderApi* api, std::uint32_t requiredByteSize) {
                if (requiredByteSize == 0) {
                    return true;
                }
                if (!api || !api->getProviderLimitsV1) {
                    return false;
                }
                RockProviderLimitsV1 limits{};
                return api->getProviderLimitsV1(&limits) && limits.providerApiByteSize >= requiredByteSize;
            };

            if (inst) {
                if (inst->getVersion() < minVersion) {
                    return 4;
                }
                return hasMinimumTableSize(inst, minProviderApiByteSize) ? 0 : 5;
            }

            const auto rockDll = GetModuleHandleA("ROCK.dll");
            if (!rockDll) {
                return 1;
            }

            const auto getApi = reinterpret_cast<const RockProviderApi*(ROCK_PROVIDER_CALL*)()>(GetProcAddress(rockDll, "ROCKAPI_GetProviderApi"));
            if (!getApi) {
                return 2;
            }

            const auto api = getApi();
            if (!api) {
                return 3;
            }

            if (api->getVersion() < minVersion) {
                return 4;
            }
            if (!hasMinimumTableSize(api, minProviderApiByteSize)) {
                return 5;
            }

            inst = api;
            return 0;
        }

        inline static const RockProviderApi* inst = nullptr;
    };

    ROCK_PROVIDER_API const RockProviderApi* ROCK_PROVIDER_CALL ROCKAPI_GetProviderApi();

    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_FORCE_GRAB_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, getInteractionCommandResultV1) + sizeof(std::declval<RockProviderApi>().getInteractionCommandResultV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_FORCE_RELEASE_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, requestForceReleaseV1) + sizeof(std::declval<RockProviderApi>().requestForceReleaseV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_THROWN_DROP_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, requestThrownDropV1) + sizeof(std::declval<RockProviderApi>().requestThrownDropV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_HAND_INPUT_SUPPRESSION_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, clearHandInputSuppressionV1) + sizeof(std::declval<RockProviderApi>().clearHandInputSuppressionV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_WEAPON_PART_INTERACTION_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, clearWeaponPartDriveTargetsV1) + sizeof(std::declval<RockProviderApi>().clearWeaponPartDriveTargetsV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_WEAPON_CLASSIFICATION_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, queryEquippedWeaponClassificationV1) + sizeof(std::declval<RockProviderApi>().queryEquippedWeaponClassificationV1));
    inline constexpr std::uint32_t ROCK_PROVIDER_API_V1_WEAPON_PART_GRIP_STATE_TABLE_BYTES = static_cast<std::uint32_t>(
        offsetof(RockProviderApi, getWeaponPartGripStateV1) + sizeof(std::declval<RockProviderApi>().getWeaponPartGripStateV1));

    [[nodiscard]] inline bool queryProviderLimitsV1(RockProviderLimitsV1& outLimits)
    {
        if (!RockProviderApi::inst || !RockProviderApi::inst->getProviderLimitsV1) {
            return false;
        }

        outLimits = {};
        return RockProviderApi::inst->getProviderLimitsV1(&outLimits);
    }

    [[nodiscard]] inline bool providerApiTableSupportsV1(const RockProviderLimitsV1& limits, std::uint32_t requiredByteSize)
    {
        return requiredByteSize != 0 && limits.providerApiByteSize >= requiredByteSize;
    }

    [[nodiscard]] inline bool providerApiTableSupportsV1(std::uint32_t requiredByteSize)
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && providerApiTableSupportsV1(limits, requiredByteSize);
    }

    [[nodiscard]] inline bool supportsForceGrabCommandV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_FORCE_GRAB_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::InteractionCommandQueue) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::ForceGrabCommand);
    }

    [[nodiscard]] inline bool supportsForceGrabCommandV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsForceGrabCommandV1(limits);
    }

    [[nodiscard]] inline bool supportsForceReleaseCommandV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_FORCE_RELEASE_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::InteractionCommandQueue) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::ForceReleaseCommand);
    }

    [[nodiscard]] inline bool supportsForceReleaseCommandV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsForceReleaseCommandV1(limits);
    }

    [[nodiscard]] inline bool supportsThrownDropCommandV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_THROWN_DROP_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::InteractionCommandQueue) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::ThrownDropCommand);
    }

    [[nodiscard]] inline bool supportsThrownDropCommandV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsThrownDropCommandV1(limits);
    }

    [[nodiscard]] inline bool supportsHandInputSuppressionV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_HAND_INPUT_SUPPRESSION_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::HandInputSuppression);
    }

    [[nodiscard]] inline bool supportsHandInputSuppressionV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsHandInputSuppressionV1(limits);
    }

    [[nodiscard]] inline bool supportsWeaponPartInteractionV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_WEAPON_PART_INTERACTION_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::WeaponPartInteraction);
    }

    [[nodiscard]] inline bool supportsWeaponPartInteractionV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsWeaponPartInteractionV1(limits);
    }

    [[nodiscard]] inline bool supportsWeaponPartGripStateV1(const RockProviderLimitsV1& limits)
    {
        return providerApiTableSupportsV1(limits, ROCK_PROVIDER_API_V1_WEAPON_PART_GRIP_STATE_TABLE_BYTES) &&
               hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::WeaponPartGripState);
    }

    [[nodiscard]] inline bool supportsWeaponPartGripStateV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsWeaponPartGripStateV1(limits);
    }

    [[nodiscard]] inline bool supportsWeaponPartRecordIdentityV1(const RockProviderLimitsV1& limits)
    {
        return hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::WeaponPartRecordIdentity);
    }

    [[nodiscard]] inline bool supportsWeaponPartRecordIdentityV1()
    {
        RockProviderLimitsV1 limits{};
        return queryProviderLimitsV1(limits) && supportsWeaponPartRecordIdentityV1(limits);
    }

    static_assert(std::is_standard_layout_v<RockProviderTransform>);
    static_assert(std::is_trivially_copyable_v<RockProviderTransform>);
    static_assert(sizeof(RockProviderConsumerRegistrationV1) == 104);
    static_assert(alignof(RockProviderConsumerRegistrationV1) == 4);
    static_assert(std::is_standard_layout_v<RockProviderConsumerRegistrationV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderConsumerRegistrationV1>);
    static_assert(sizeof(RockProviderConsumerHandleV1) == 48);
    static_assert(alignof(RockProviderConsumerHandleV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderConsumerHandleV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderConsumerHandleV1>);
    static_assert(sizeof(RockProviderLimitsV1) == 72);
    static_assert(alignof(RockProviderLimitsV1) == 4);
    static_assert(std::is_standard_layout_v<RockProviderLimitsV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderLimitsV1>);
    static_assert(sizeof(RockProviderForceGrabRequestV1) == 80);
    static_assert(alignof(RockProviderForceGrabRequestV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderForceGrabRequestV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderForceGrabRequestV1>);
    static_assert(sizeof(RockProviderForceReleaseRequestV1) == 72);
    static_assert(alignof(RockProviderForceReleaseRequestV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderForceReleaseRequestV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderForceReleaseRequestV1>);
    static_assert(sizeof(RockProviderThrownDropRequestV1) == 96);
    static_assert(alignof(RockProviderThrownDropRequestV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderThrownDropRequestV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderThrownDropRequestV1>);
    static_assert(sizeof(RockProviderInteractionCommandResultV1) == 112);
    static_assert(alignof(RockProviderInteractionCommandResultV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderInteractionCommandResultV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderInteractionCommandResultV1>);
    static_assert(sizeof(RockProviderHandInputSuppressionRequestV1) == 64);
    static_assert(alignof(RockProviderHandInputSuppressionRequestV1) == 4);
    static_assert(std::is_standard_layout_v<RockProviderHandInputSuppressionRequestV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderHandInputSuppressionRequestV1>);
    static_assert(sizeof(RockProviderWeaponPartTargetV1) == 160);
    static_assert(alignof(RockProviderWeaponPartTargetV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponPartTargetV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponPartTargetV1>);
    static_assert(sizeof(RockProviderTransform) == 52);
    static_assert(sizeof(RockProviderWeaponPartDriveTargetV1) == 192);
    static_assert(alignof(RockProviderWeaponPartDriveTargetV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponPartDriveTargetV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponPartDriveTargetV1>);
    static_assert(sizeof(RockProviderWeaponPartTargetQueryV1) == 104);
    static_assert(alignof(RockProviderWeaponPartTargetQueryV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponPartTargetQueryV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponPartTargetQueryV1>);
    static_assert(sizeof(RockProviderWeaponPartTargetResolutionV1) == 24);
    static_assert(alignof(RockProviderWeaponPartTargetResolutionV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponPartTargetResolutionV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponPartTargetResolutionV1>);
    static_assert(sizeof(RockProviderWeaponClassificationV1) == 48);
    static_assert(alignof(RockProviderWeaponClassificationV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponClassificationV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponClassificationV1>);
    static_assert(sizeof(RockProviderWeaponPartGripStateV1) == 248);
    static_assert(alignof(RockProviderWeaponPartGripStateV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponPartGripStateV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponPartGripStateV1>);
    static_assert(sizeof(RockProviderFrameSnapshot) == 272);
    static_assert(alignof(RockProviderFrameSnapshot) == 8);
    static_assert(sizeof(RockProviderHandFrameV1) == 112);
    static_assert(alignof(RockProviderHandFrameV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderHandFrameV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderHandFrameV1>);
    static_assert(sizeof(RockProviderExternalBodyRegistration) == 32);
    static_assert(sizeof(RockProviderExternalContactV1) == 128);
    static_assert(alignof(RockProviderExternalContactV1) == 8);
    static_assert(sizeof(RockProviderPoint3) == 12);
    static_assert(sizeof(RockProviderBounds3) == 32);
    static_assert(sizeof(RockProviderWeaponEvidenceDetailV1) == 192);
    static_assert(alignof(RockProviderWeaponEvidenceDetailV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponEvidenceDetailV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponEvidenceDetailV1>);
    static_assert(sizeof(RockProviderBodyContactV1) == 128);
    static_assert(alignof(RockProviderBodyContactV1) == 8);
    static_assert(std::is_standard_layout_v<RockProviderBodyContactV1>);
    static_assert(std::is_trivially_copyable_v<RockProviderBodyContactV1>);
}

namespace rock
{
    class PhysicsInteraction;
}

namespace rock::provider
{
    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi);
    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi);
    void clearExternalBodiesForProviderLoss();
    bool isExternalBodyId(std::uint32_t bodyId);
    bool isExternalBodyDynamicPushSuppressed(std::uint32_t bodyId);
    bool recordExternalHandContact(bool isLeft, std::uint32_t handBodyId, std::uint32_t externalBodyId, std::uint64_t frameIndex);
    bool recordExternalContact(const RockProviderExternalContactV1& contact);
    RockProviderOffhandReservation currentOffhandReservation();
    std::uint32_t currentHandInputSuppressionFlagsV1(RockProviderHand hand);
    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution);
    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets);
    std::uint32_t currentExternalBodyCount();
}
