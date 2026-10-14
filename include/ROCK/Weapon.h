#pragma once
#include "Abi.h"

// Equipped weapons, addressed by weapon slot (see kMaxWeaponSlots): state and
// hand assignment, grip, classification, emitters, composition, authored grip,
// scope sight, the handling authority lease and inventory equips.
#pragma pack(push, 8)
namespace rock::api::weapon {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Weapon;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxEmitters = 32;
    inline constexpr std::uint32_t kMaxCompositionEntries = 64;
    inline constexpr std::uint32_t kMaxHandlingLeaseFrames = 120;
    inline constexpr std::uint32_t kFingerLocalTransformCount = 15;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;

    // ------------------------------------------------------------------
    // Weapon state
    // ------------------------------------------------------------------

    enum class WeaponStateFlag : std::uint32_t
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

    enum class HandlingStateFlag : std::uint32_t
    {
        None = 0,
        // A hand holds the weapon's firing grip.
        FiringGripHeld = 1u << 0,
        // Grips follow the hands' own grab ownership.
        ManualOwnership = 1u << 1,
        // The firing hand carries the weapon by a part, vacating the firing grip.
        PartCarry = 1u << 2,
        // The left hand can take the firing grip.
        LeftFiringAvailable = 1u << 3,
    };

    enum class TransitionSource : std::uint32_t
    {
        Unknown = 0,
        ObservedEquip = 1,
        HeldTriggerEquip = 2,
        HeldGripZoneEquip = 3,
        MenuExit = 4,
        WorkbenchExit = 5,
    };

    enum class TransitionResult : std::uint32_t
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

    /*
     * One weapon slot. firingHand is the hand assigned the weapon's firing
     * role (FiringGripHeld says whether it holds the grip now); supportHand
     * holds its support grip, or None. Both are None without a weapon. The
     * transition fields follow the slot's latest equip transition and its
     * last terminal result.
     */
    struct WeaponState
    {
        std::uint32_t size{ sizeof(WeaponState) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t weaponFormId{ 0 };
        Hand firingHand{ Hand::None };
        Hand supportHand{ Hand::None };
        std::uint32_t handlingFlags{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t transitionSequence{ 0 };
        std::uint64_t terminalSequence{ 0 };
        TransitionSource transitionSource{ TransitionSource::Unknown };
        TransitionResult terminalResult{ TransitionResult::None };
        TransitionSource terminalSource{ TransitionSource::Unknown };
        std::uint32_t transitionWeaponFormId{ 0 };
        std::uint32_t terminalWeaponFormId{ 0 };
    };

    // ------------------------------------------------------------------
    // Grip
    // ------------------------------------------------------------------

    // Bits 1 (two-hand grip) and 2 (left firing hand) are retired: the hands
    // are in WeaponState. Never reuse.
    enum class GripFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        WeaponTransformOwned = 1u << 3,
        WeaponWorldValid = 1u << 4,
        RightHandInWeaponValid = 1u << 5,
        LeftHandInWeaponValid = 1u << 6,
        MuzzleWorldValid = 1u << 7,
    };

    // The weapon's solved world transform, each hand's grip in weapon space and
    // the projectile origin.
    struct Grip
    {
        std::uint32_t size{ sizeof(Grip) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        Transform weaponWorld{};
        Transform rightHandInWeapon{};
        Transform leftHandInWeapon{};
        Point3 muzzleOriginGame{};
        Point3 muzzleDirectionGame{};
    };

    // ------------------------------------------------------------------
    // Classification
    // ------------------------------------------------------------------

    enum class SizeClass : std::uint32_t
    {
        Melee = 0,
        Pistol = 1,
        Rifle = 2,
        Heavy = 3,
    };

    enum class ClassificationSource : std::uint32_t
    {
        None = 0,
        Keyword = 1,
        WeaponData = 2,
        EquipSlot = 3,
    };

    enum class KeywordFlag : std::uint64_t
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

    enum class ClassificationProvenanceFlag : std::uint32_t
    {
        None = 0,
        KeywordEvidence = 1u << 0,
        EffectiveInstanceKeywordEvidence = 1u << 1,
        WeaponDataEvidence = 1u << 2,
        EquipSlotEvidence = 1u << 3,
        GenerationBound = 1u << 4,
    };

    struct Classification
    {
        std::uint32_t size{ sizeof(Classification) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t valid{ 0 };
        std::uint64_t keywordFlags{ 0 };
        SizeClass sizeClass{ SizeClass::Rifle };
        ClassificationSource source{ ClassificationSource::None };
        std::uint32_t weaponFormId{ 0 };
        float confidence{ 0.0f };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t provenanceFlags{ 0 };
    };

    // ------------------------------------------------------------------
    // Emitters
    // ------------------------------------------------------------------

    enum class EmitterKind : std::uint32_t
    {
        Unknown = 0,
        Flashlight = 1,
        Laser = 2,
        Reticle = 3,
    };

    enum class EmitterSource : std::uint32_t
    {
        Unknown = 0,
        EffectGeometry = 1,
        AddOnNode = 2,
    };

    enum class EmitterFlag : std::uint32_t
    {
        None = 0,
        TransformValid = 1u << 0,
        DirectionValid = 1u << 1,
        EffectStateKnown = 1u << 2,
        HasAddOnNodeValue = 1u << 3,
        HasOmod = 1u << 4,
        HasAttachPoint = 1u << 5,
    };

    struct Emitter
    {
        std::uint32_t size{ sizeof(Emitter) };
        std::uint32_t version{ 1 };
        EmitterKind kind{ EmitterKind::Unknown };
        EmitterSource source{ EmitterSource::Unknown };
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
    };

    // ------------------------------------------------------------------
    // Composition
    // ------------------------------------------------------------------

    enum class CompositionFlag : std::uint32_t
    {
        None = 0,
        Active = 1u << 0,
        Disabled = 1u << 1,
        AttachPointResolved = 1u << 2,
        SemanticEvidenceMatched = 1u << 3,
    };

    struct CompositionState
    {
        std::uint32_t size{ sizeof(CompositionState) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t compositionSignature{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
        std::uint64_t missingCoverageMask{ 0 };
        std::uint64_t publicationSequence{ 0 };
        std::uint32_t entryCount{ 0 };
    };

    // One installed mod (OMOD) and the attach point it occupies.
    struct CompositionEntry
    {
        std::uint32_t size{ sizeof(CompositionEntry) };
        std::uint32_t version{ 1 };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t stableIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t semanticCoverageMask{ 0 };
    };

    // ------------------------------------------------------------------
    // Authored grip
    // ------------------------------------------------------------------

    enum class AuthoredGripSource : std::uint32_t
    {
        Unknown = 0,
        LiveEquippedGraph = 1,
        NativeIdlePreharvest = 2,
        RuntimeCanonical = 3,
    };

    enum class AuthoredGripPoseFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        RightHandValid = 1u << 1,
        LeftHandValid = 1u << 2,
        RightFingersValid = 1u << 3,
        LeftFingersValid = 1u << 4,
    };

    // The authored hand and finger poses ROCK selected for the weapon. The
    // generations are those at capture.
    struct AuthoredGripPose
    {
        std::uint32_t size{ sizeof(AuthoredGripPose) };
        std::uint32_t version{ 1 };
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t variantKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        AuthoredGripSource source{ AuthoredGripSource::Unknown };
        std::uint32_t flags{ 0 };
        std::uint16_t rightFingerLocalTransformMask{ 0 };
        std::uint16_t leftFingerLocalTransformMask{ 0 };
        Transform rightHandInWeapon{};
        Transform leftHandInWeapon{};
        Transform rightFingerLocalTransforms[kFingerLocalTransformCount]{};
        Transform leftFingerLocalTransforms[kFingerLocalTransformCount]{};
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // ------------------------------------------------------------------
    // Scope sight
    // ------------------------------------------------------------------

    enum class ScopeActivationSource : std::uint32_t
    {
        None = 0,
        NativeGeometry = 1,
        RockGeometry = 2,
        ManualInput = 3,
    };

    enum class ScopeSightFlag : std::uint32_t
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

    struct ScopeSightState
    {
        std::uint32_t size{ sizeof(ScopeSightState) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t weaponSlot{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t weaponFormId{ 0 };
        ScopeActivationSource activationSource{ ScopeActivationSource::None };
        std::uint32_t nativeScopeOverlayIndex{ 0 };
        Point3 anchorWeaponLocal{};
        Bounds3 sightBoundsWeaponLocal{};
        std::uint32_t sightBodyCount{ 0 };
        std::uint32_t sightBodyId{ 0x7FFF'FFFF };
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
    };

    // ------------------------------------------------------------------
    // Handling authority
    // ------------------------------------------------------------------

    enum class HandlingFeature : std::uint32_t
    {
        None = 0,
        FiringGripOwnership = 1u << 0,
        PrimaryDetach = 1u << 1,
        AmbidextrousHandoff = 1u << 2,
        GripZoneEquip = 1u << 3,
        GripZoneHoverHaptics = 1u << 4,
        FiringGripProximitySupport = 1u << 5,
    };

    // One owner at a time overrides ROCK's weapon handling tuning for
    // 1..kMaxHandlingLeaseFrames frames; renew to keep it.
    struct HandlingRequest
    {
        std::uint32_t size{ sizeof(HandlingRequest) };
        std::uint32_t version{ 1 };
        std::uint32_t features{ 0 };
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
        float leftFiringAimYawDegrees{ 0.0f };
        float leftFiringAimPitchDegrees{ 0.0f };
        float leftFiringAimOffsetGameUnits[3]{};
        float equipVisualBridgeTimeoutSeconds{ 2.0f };
        float equipVisualBridgeBlendSeconds{ 0.15f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct HandlingAuthority
    {
        std::uint32_t size{ sizeof(HandlingAuthority) };
        std::uint32_t version{ 1 };
        std::uint32_t active{ 0 };
        std::uint32_t features{ 0 };
        OwnerToken owner{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
    };

    // ------------------------------------------------------------------
    // Inventory equip
    // ------------------------------------------------------------------

    // An inventory weapon stack captured on the frame owner immediately before
    // use. The keys are opaque comparison witnesses, never pointers or saved
    // identities; stackIndex is the native inventory stack index.
    struct InventoryWeapon
    {
        std::uint32_t size{ sizeof(InventoryWeapon) };
        std::uint32_t version{ 1 };
        std::uint32_t baseFormId{ 0 };
        std::uint32_t stackIndex{ 0 };
        std::uint32_t count{ 0 };
        std::uint64_t stackKey{ 0 };
        std::uint64_t instanceKey{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct EquipRequest
    {
        std::uint32_t size{ sizeof(EquipRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        InventoryWeapon item{};
    };

    enum class EquipState : std::uint32_t
    {
        Unknown = 0,
        Queued = 1,
        Switching = 2,
        Succeeded = 3,
        Failed = 4,
        Cancelled = 5,
    };

    enum class EquipFailure : std::uint32_t
    {
        None = 0,
        LifecycleLost = 1,
        OwnerLost = 2,
        HandUnavailable = 3,
        HandBusy = 4,
        InventoryChanged = 5,
        InfrastructureUnavailable = 6,
        OutgoingTransferFailed = 7,
        NativeEquipFailed = 8,
        GripOrPresentationFailed = 9,
        CancelledBeforeCommit = 10,
        EquippedWeaponChanged = 11,
        MenuBlocked = 12,
    };

    enum class EquipResultFlag : std::uint32_t
    {
        OutgoingRemoved = 1u << 0,
        OutgoingRetained = 1u << 1,
        IncomingEquipAccepted = 1u << 2,
        IncomingGripAcquired = 1u << 3,
        IncomingPresented = 1u << 4,
    };

    struct EquipResult
    {
        std::uint32_t size{ sizeof(EquipResult) };
        std::uint32_t version{ 1 };
        std::uint64_t commandId{ 0 };
        std::uint64_t transferSequence{ 0 };
        EquipState state{ EquipState::Unknown };
        EquipFailure failure{ EquipFailure::None };
        std::uint32_t flags{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t incomingFormId{ 0 };
        std::uint32_t outgoingFormId{ 0 };
        std::uint32_t outgoingReferenceFormId{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    // TransitionCompleted: a slot finished an equip transition (formId,
    // transitionSequence, source, terminalResult, flags). AuthorityLost: the
    // owner's handling authority ended (reason).
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
        std::uint32_t weaponSlot{};
        Hand hand{ Hand::None };
        std::uint32_t formId{};
        std::uint64_t weaponGenerationKey{};
        std::uint64_t transitionSequence{};
        std::uint32_t source{};
        std::uint32_t terminalResult{};
        std::uint32_t flags{};
        std::uint32_t authority{};
        std::uint32_t reason{};
    };
    enum class EventKind : std::uint32_t { TransitionCompleted = 1, AuthorityLost = 2 };

    /*
     * Per-weapon calls take weaponSlot < kMaxWeaponSlots (InvalidArgument
     * otherwise). A slot ROCK does not publish yet returns TargetUnavailable.
     * Game thread unless noted.
     */
    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Read.
        Status(ROCK_CALL* getWeaponState)(OwnerToken owner, std::uint32_t weaponSlot, WeaponState* outState) noexcept;
        Status(ROCK_CALL* getGrip)(OwnerToken owner, std::uint32_t weaponSlot, Grip* outGrip) noexcept;
        Status(ROCK_CALL* getClassification)(OwnerToken owner, std::uint32_t weaponSlot, Classification* outClassification) noexcept;
        // Read. outCount is the weapon's emitter count; min(capacity, count)
        // records are copied.
        Status(ROCK_CALL* copyEmitters)(OwnerToken owner, std::uint32_t weaponSlot, Emitter* outEmitters, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* getCompositionState)(OwnerToken owner, std::uint32_t weaponSlot, CompositionState* outState) noexcept;
        Status(ROCK_CALL* copyCompositionEntries)(OwnerToken owner, std::uint32_t weaponSlot, CompositionEntry* outEntries, std::uint32_t capacity, std::uint32_t* outCount) noexcept;
        Status(ROCK_CALL* getAuthoredGripPose)(OwnerToken owner, std::uint32_t weaponSlot, AuthoredGripPose* outPose) noexcept;
        Status(ROCK_CALL* getScopeSightState)(OwnerToken owner, std::uint32_t weaponSlot, ScopeSightState* outState) noexcept;
        // Write to set or clear the owner's lease (OwnerConflict while another
        // owner holds it); Read for the current authority.
        Status(ROCK_CALL* setHandlingAuthority)(OwnerToken owner, const HandlingRequest* request) noexcept;
        Status(ROCK_CALL* clearHandlingAuthority)(OwnerToken owner) noexcept;
        Status(ROCK_CALL* getHandlingAuthority)(OwnerToken owner, HandlingAuthority* outAuthority) noexcept;
        // Read. Capture immediately before requestInventoryEquip.
        Status(ROCK_CALL* captureInventoryWeapon)(OwnerToken owner, std::uint32_t baseFormId, std::uint32_t stackIndex, InventoryWeapon* outItem) noexcept;
        // Write. Equips into the requested physical hand; a weapon carried only
        // by the other hand stays there. A busy destination is refused. One
        // switch may be outstanding; RequestQueued is admission, not success.
        Status(ROCK_CALL* requestInventoryEquip)(OwnerToken owner, const EquipRequest* request, std::uint64_t* outCommandId) noexcept;
        // Read. The latest 64 terminal results are kept; inspect flags even
        // when the switch fails.
        Status(ROCK_CALL* getInventoryEquipResult)(OwnerToken owner, std::uint64_t commandId, EquipResult* outResult) noexcept;
        // Write. Accepted only before the outgoing removal or native equip;
        // AlreadyCommitted afterwards, and ROCK finishes the cleanup.
        Status(ROCK_CALL* cancelInventoryEquip)(OwnerToken owner, std::uint64_t commandId) noexcept;
        // Read, any thread; for a native pickup interceptor. outMatches is 1
        // only while ROCK acquires exactly this reference and count for an
        // equip, on the acquiring thread. It is intent, not a promise that
        // the equip succeeds.
        Status(ROCK_CALL* isEquipPickup)(OwnerToken owner, std::uint32_t referenceFormId, std::uint32_t count, std::uint32_t* outMatches) noexcept;
        // Any thread.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken owner, std::uint64_t afterSequence, Event* outEvents, std::uint32_t capacity, Stream* outStream) noexcept;
    };
}
#pragma pack(pop)
