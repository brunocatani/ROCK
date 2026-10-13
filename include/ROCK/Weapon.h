#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::weapon {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Weapon;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxEmitters = 32;
    inline constexpr std::uint32_t kMaxCompositionEntries = 64;
    inline constexpr std::uint32_t kMaxHandlingLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;


    enum class EquippedWeaponGripStateFlagV1 : std::uint32_t
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

    enum class EquippedWeaponHandlingFlagV1 : std::uint32_t
    {
        None = 0,
        FiringGripOwnership = 1u << 0,
        PrimaryDetach = 1u << 1,
        AmbidextrousHandoff = 1u << 2,
        GripZoneEquip = 1u << 3,
        GripZoneHoverHaptics = 1u << 4,
        FiringGripProximitySupport = 1u << 5,
    };

    enum class EquippedWeaponHandlingRuntimeFlagV1 : std::uint32_t
    {
        None = 0,
        AuthorityActive = 1u << 0,
        FiringHandLeft = 1u << 2,
        LeftFiringInfrastructureAvailable = 1u << 3,
        ManualOwnershipActive = 1u << 4,
        PartCarryActive = 1u << 5,
        FiringGripOccupied = 1u << 6,
        WeaponPresent = 1u << 7,
    };

    enum class WeaponEmitterKindV1 : std::uint32_t
    {
        Unknown = 0,
        Flashlight = 1,
        Laser = 2,
        Reticle = 3,
    };

    enum class WeaponEmitterSourceV1 : std::uint32_t
    {
        Unknown = 0,
        EffectGeometry = 1,
        AddOnNode = 2,
    };

    enum class WeaponEmitterFlagV1 : std::uint32_t
    {
        None = 0,
        TransformValid = 1u << 0,
        DirectionValid = 1u << 1,
        EffectStateKnown = 1u << 2,
        HasAddOnNodeValue = 1u << 3,
        HasOmod = 1u << 4,
        HasAttachPoint = 1u << 5,
    };

    enum class WeaponSizeClassV1 : std::uint32_t
    {
        Melee = 0,
        Pistol = 1,
        Rifle = 2,
        Heavy = 3,
    };

    enum class WeaponClassificationSourceV1 : std::uint32_t
    {
        None = 0,
        Keyword = 1,
        WeaponData = 2,
        EquipSlot = 3,
    };

    enum class WeaponKeywordFlagV1 : std::uint64_t
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

    enum class EquippedWeaponTransitionSourceV1 : std::uint32_t
    {
        Unknown = 0,
        ObservedEquip = 1,
        HeldTriggerEquip = 2,
        HeldGripZoneEquip = 3,
        MenuExit = 4,
        WorkbenchExit = 5,
    };

    enum class EquippedWeaponTransitionResultV1 : std::uint32_t
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

    enum class EquippedWeaponStateFlagV1 : std::uint32_t
    {
        None = 0,
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
        PresentationKnown = 1u << 10,
        WeaponEquipped = 1u << 11,
        BipodInputReserved = 1u << 12,
    };

    enum class ScopeActivationSourceV1 : std::uint32_t
    {
        None = 0,
        NativeGeometry = 1,
        RockGeometry = 2,
        ManualInput = 3,
    };

    enum class ScopeSightFlagV1 : std::uint32_t
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

    enum class WeaponClassificationProvenanceFlagV1 : std::uint32_t
    {
        None = 0,
        KeywordEvidence = 1u << 0,
        EffectiveInstanceKeywordEvidence = 1u << 1,
        WeaponDataEvidence = 1u << 2,
        EquipSlotEvidence = 1u << 3,
        GenerationBound = 1u << 4,
    };

    enum class WeaponCompositionFlagV1 : std::uint32_t
    {
        None = 0,
        Active = 1u << 0,
        Disabled = 1u << 1,
        AttachPointResolved = 1u << 2,
        SemanticEvidenceMatched = 1u << 3,
    };

    enum class AuthoredGripSourceV1 : std::uint32_t
    {
        Unknown = 0,
        LiveEquippedGraph = 1,
        NativeIdlePreharvest = 2,
        RuntimeCanonical = 3,
    };

    enum class AuthoredGripPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        RightHandValid = 1u << 1,
        LeftHandValid = 1u << 2,
        RightFingersValid = 1u << 3,
        LeftFingersValid = 1u << 4,
    };

    struct EquippedWeaponGripStateV1
    {
        std::uint32_t size{ sizeof(EquippedWeaponGripStateV1) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        Transform weaponWorld{};
        Transform rightHandInWeapon{};
        Transform leftHandInWeapon{};
        Point3 muzzleOriginGame{};
        Point3 muzzleDirectionGame{};
        std::uint32_t reserved[2]{};
    };

    struct EquippedWeaponHandlingRequestV1
    {
        std::uint32_t size{ sizeof(EquippedWeaponHandlingRequestV1) };
        std::uint32_t version{ 1 };
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

    struct EquippedWeaponHandlingStateV1
    {
        std::uint32_t size{ sizeof(EquippedWeaponHandlingStateV1) };
        std::uint32_t version{ 1 };
        std::uint32_t authorityFlags{ 0 };
        std::uint32_t runtimeFlags{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        Hand currentFiringHand{ Hand::Right };
        std::uint32_t reserved[9]{};
    };

    struct WeaponCompositionStateV1
    {
        std::uint32_t size{ sizeof(WeaponCompositionStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t compositionSignature{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t entryCount{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
        std::uint64_t missingCoverageMask{ 0 };
        std::uint64_t publicationSequence{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct WeaponCompositionEntryV1
    {
        std::uint32_t size{ sizeof(WeaponCompositionEntryV1) };
        std::uint32_t version{ 1 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t stableIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct WeaponClassificationV1
    {
        std::uint32_t size{ sizeof(WeaponClassificationV1) };
        std::uint32_t valid{ 0 };
        std::uint64_t keywordFlags{ 0 };
        WeaponSizeClassV1 sizeClass{ WeaponSizeClassV1::Rifle };
        WeaponClassificationSourceV1 source{ WeaponClassificationSourceV1::None };
        std::uint32_t formId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        float confidence{ 0.0f };
        std::uint32_t provenanceFlags{ 0 };
    };

    struct WeaponEmitterV1
    {
        std::uint32_t size{ sizeof(WeaponEmitterV1) };
        std::uint32_t version{ 1 };
        WeaponEmitterKindV1 kind{ WeaponEmitterKindV1::Unknown };
        WeaponEmitterSourceV1 source{ WeaponEmitterSourceV1::Unknown };
        std::uint32_t flags{ 0 };
        std::uint32_t active{ 0 };
        std::uint32_t visible{ 0 };
        std::uint32_t addOnNodeValue{ 0 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        Transform weaponLocalTransform{};
        Point3 forwardWeaponLocal{};
        char sourceName[64]{};
        std::uint32_t reserved[8]{};
    };

    struct EquippedWeaponStateV1
    {
        std::uint32_t size{ sizeof(EquippedWeaponStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t transitionSequence{ 0 };
        std::uint64_t terminalSequence{ 0 };
        EquippedWeaponTransitionSourceV1 transitionSource{
            EquippedWeaponTransitionSourceV1::Unknown
        };
        EquippedWeaponTransitionResultV1 terminalResult{
            EquippedWeaponTransitionResultV1::None
        };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t transitionWeaponFormId{ 0 };
        std::uint32_t terminalWeaponFormId{ 0 };
        EquippedWeaponTransitionSourceV1 terminalSource{
            EquippedWeaponTransitionSourceV1::Unknown
        };
        std::uint32_t reserved[4]{};
    };

    struct ScopeSightStateV1
    {
        std::uint32_t size{ sizeof(ScopeSightStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t publicationSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t flags{ 0 };
        ScopeActivationSourceV1 activationSource{
            ScopeActivationSourceV1::None
        };
        std::uint32_t nativeScopeOverlayIndex{ 0 };
        Point3 anchorWeaponLocal{};
        Bounds3 sightBoundsWeaponLocal{};
        std::uint32_t sightBodyCount{ 0 };
        std::uint32_t sightBodyId{ 0x7FFF'FFFF };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct AuthoredGripPoseV1
    {
        std::uint32_t size{ sizeof(AuthoredGripPoseV1) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        AuthoredGripSourceV1 source{ AuthoredGripSourceV1::Unknown };
        std::uint64_t variantKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t flags{ 0 };
        std::uint16_t rightFingerLocalTransformMask{ 0 };
        std::uint16_t leftFingerLocalTransformMask{ 0 };
        Transform rightHandInWeapon{};
        Transform leftHandInWeapon{};
        Transform rightFingerLocalTransforms[15]{};
        Transform leftFingerLocalTransforms[15]{};
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };


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
        Hand hand{Hand::None}; std::uint32_t formId{}; std::uint64_t weaponGenerationKey{}; std::uint64_t transitionSequence{}; std::uint32_t source{}; std::uint32_t terminalResult{}; std::uint32_t flags{}; std::uint32_t authority{}; std::uint32_t reason{};
    };
    enum class EventKindV1 : std::uint32_t { TransitionCompleted=1, AuthorityLost=2 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* getPrimaryHandV1)(OwnerToken ownerToken, Hand* outValue) noexcept;
        Status(ROCK_CALL* getOffhandHandV1)(OwnerToken ownerToken, Hand* outValue) noexcept;
        Status(ROCK_CALL* queryEquippedWeaponClassificationV1)(OwnerToken ownerToken, WeaponClassificationV1* outResult) noexcept;
        Status(ROCK_CALL* getWeaponEmitterCountV1)(OwnerToken ownerToken, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* copyWeaponEmittersV1)(OwnerToken ownerToken, WeaponEmitterV1* outEmitters, std::uint32_t maxEmitters, std::uint32_t* outCopied) noexcept;
        Status(ROCK_CALL* getEquippedWeaponGripStateV1)(std::uint64_t ownerToken, EquippedWeaponGripStateV1* outState) noexcept;
        Status(ROCK_CALL* setEquippedWeaponHandlingAuthorityV1)(std::uint64_t ownerToken, const EquippedWeaponHandlingRequestV1* request) noexcept;
        Status(ROCK_CALL* clearEquippedWeaponHandlingAuthorityV1)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* getEquippedWeaponHandlingStateV1)(OwnerToken ownerToken, EquippedWeaponHandlingStateV1* outState) noexcept;
        // Any-thread synchronized value snapshot; grants no mutation authority.
        Status(ROCK_CALL* getEquippedWeaponStateV1)(std::uint64_t ownerToken, EquippedWeaponStateV1* outState) noexcept;
        Status(ROCK_CALL* getScopeSightStateV1)(std::uint64_t ownerToken, ScopeSightStateV1* outState) noexcept;
        Status(ROCK_CALL* getWeaponCompositionStateV1)(std::uint64_t ownerToken, WeaponCompositionStateV1* outState) noexcept;
        Status(ROCK_CALL* copyWeaponCompositionEntriesV1)(std::uint64_t ownerToken, WeaponCompositionEntryV1* outEntries, std::uint32_t maxEntries, std::uint32_t* outEntryCount) noexcept;
        Status(ROCK_CALL* getSelectedAuthoredGripPoseV1)(std::uint64_t ownerToken, AuthoredGripPoseV1* outPose) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)
