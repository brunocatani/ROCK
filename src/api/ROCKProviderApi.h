#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef NOMMNOSOUND
#define NOMMNOSOUND
#endif

#include <cstdint>
#include <type_traits>

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

    inline constexpr std::uint32_t ROCK_PROVIDER_API_VERSION = 9;
    inline constexpr std::uint32_t ROCK_PROVIDER_FRAME_SNAPSHOT_V6_SIZE = 256;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_WEAPON_BODIES = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EVIDENCE_NAME = 64;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_BODIES_V2 = 2048;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V2 = 512;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DIAGNOSTIC_AXES_V4 = 16;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DIAGNOSTIC_MARKERS_V4 = 16;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_V4 = 8;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_CHARS_V4 = 128;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_BODY_CONTACTS_V6 = 128;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_FRAME_CALLBACKS_V9 = 16;
    inline constexpr std::uint32_t ROCK_PROVIDER_MAX_CONSUMERS_V9 = 64;

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

    enum class RockProviderHandFrameFlagV8 : std::uint32_t
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

    enum class RockProviderResultV9 : std::uint32_t
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
        RequestRejected = 16,
        RequestNotFound = 17,
    };

    enum class RockProviderConsumerCapabilityV9 : std::uint32_t
    {
        None = 0,
        FrameSnapshots = 1u << 0,
        ExternalBodies = 1u << 1,
        ExternalContacts = 1u << 2,
        OffhandReservation = 1u << 3,
        DiagnosticOverlay = 1u << 4,
        DiagnosticInput = 1u << 5,
        InteractionCommands = 1u << 6,
    };

    enum class RockProviderFeatureBitV9 : std::uint32_t
    {
        None = 0,
        FrameCallbacks = 1u << 0,
        LifecycleFields = 1u << 1,
        HandFramesV8 = 1u << 2,
        WeaponEvidenceV3 = 1u << 3,
        BodyContactsV6 = 1u << 4,
        ExternalContactsV2 = 1u << 5,
        DiagnosticOverlayV4 = 1u << 6,
        DiagnosticInputV5 = 1u << 7,
        ConsumerRegistrationV9 = 1u << 8,
        OwnerFilteredExternalContactsV9 = 1u << 9,
        InteractionCommandQueue = 1u << 10,
        ForceGrabCommand = 1u << 11,
        ForceReleaseCommand = 1u << 12,
    };

    [[nodiscard]] inline constexpr bool hasLifecycleFlag(std::uint32_t flags, RockProviderLifecycleFlag flag)
    {
        return (flags & static_cast<std::uint32_t>(flag)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasConsumerCapabilityV9(std::uint32_t capabilities, RockProviderConsumerCapabilityV9 capability)
    {
        return (capabilities & static_cast<std::uint32_t>(capability)) != 0;
    }

    [[nodiscard]] inline constexpr bool hasFeatureBitV9(std::uint32_t featureBits, RockProviderFeatureBitV9 feature)
    {
        return (featureBits & static_cast<std::uint32_t>(feature)) != 0;
    }

    struct RockProviderConsumerRegistrationV9
    {
        std::uint32_t size{ sizeof(RockProviderConsumerRegistrationV9) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        char modName[64]{};
        std::uint32_t requestedCapabilities{ 0 };
        std::uint32_t reserved[7]{};
    };

    struct RockProviderConsumerHandleV9
    {
        std::uint32_t size{ sizeof(RockProviderConsumerHandleV9) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t grantedCapabilities{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct RockProviderLimitsV9
    {
        std::uint32_t size{ sizeof(RockProviderLimitsV9) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint32_t featureBits{ 0 };
        std::uint32_t maxFrameCallbacks{ 0 };
        std::uint32_t maxConsumers{ 0 };
        std::uint32_t maxExternalBodies{ 0 };
        std::uint32_t maxExternalContacts{ 0 };
        std::uint32_t maxBodyContacts{ 0 };
        std::uint32_t maxWeaponBodies{ 0 };
        std::uint32_t maxInteractionCommands{ 0 };
        std::uint32_t reserved[8]{};
    };

    struct RockProviderTransform
    {
        float rotate[9]{};
        float translate[3]{};
        float scale{ 1.0f };
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

    struct RockProviderHandFrameV8
    {
        std::uint32_t size{ sizeof(RockProviderHandFrameV8) };
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

    struct RockProviderWeaponEvidenceDescriptor
    {
        std::uint32_t size{ sizeof(RockProviderWeaponEvidenceDescriptor) };
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
        char sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};
        std::uint32_t reserved[2]{};
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
     * V3 keeps generated reload evidence detailed without making the fixed
     * function table own variable-length buffers. The detail row carries the
     * semantic body identity, local generated bounds, and total point count;
     * callers fetch the local mesh point cloud through the body-id keyed copy
     * function below. The older v2 descriptor remains available for consumers
     * that only need contact classification.
     */
    struct RockProviderWeaponEvidenceDetailV3
    {
        std::uint32_t size{ sizeof(RockProviderWeaponEvidenceDetailV3) };
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
        std::uint32_t reserved[9]{};
    };

    struct RockProviderBodyContactV6
    {
        std::uint32_t size{ sizeof(RockProviderBodyContactV6) };
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

    struct RockProviderExternalContact
    {
        std::uint32_t size{ sizeof(RockProviderExternalContact) };
        std::uint32_t handBodyId{ 0x7FFF'FFFF };
        std::uint32_t externalBodyId{ 0x7FFF'FFFF };
        std::uint32_t generation{ 0 };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t sequence{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        RockProviderExternalBodyRole role{ RockProviderExternalBodyRole::Unknown };
    };

    struct RockProviderExternalContactV2
    {
        std::uint32_t size{ sizeof(RockProviderExternalContactV2) };
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

    enum class RockProviderDiagnosticOverlayFlagsV4 : std::uint32_t
    {
        None = 0,
        DrawAxes = 1u << 0,
        DrawMarkers = 1u << 1,
        DrawScreenText = 1u << 2,
    };

    struct RockProviderDiagnosticOverlayAxisV4
    {
        std::uint32_t size{ sizeof(RockProviderDiagnosticOverlayAxisV4) };
        std::uint32_t role{ 0 };
        RockProviderTransform transform{};
        float translationStart[3]{};
        std::uint32_t drawTranslationLine{ 0 };
        std::uint32_t reserved[3]{};
    };

    struct RockProviderDiagnosticOverlayMarkerV4
    {
        std::uint32_t size{ sizeof(RockProviderDiagnosticOverlayMarkerV4) };
        std::uint32_t role{ 0 };
        float position[3]{};
        float lineEnd[3]{};
        float sizeGame{ 2.0f };
        std::uint32_t drawPoint{ 1 };
        std::uint32_t drawLine{ 0 };
        std::uint32_t reserved[3]{};
    };

    struct RockProviderDiagnosticOverlayTextV4
    {
        std::uint32_t size{ sizeof(RockProviderDiagnosticOverlayTextV4) };
        std::uint32_t role{ 0 };
        char text[ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_CHARS_V4]{};
        float x{ 18.0f };
        float y{ 18.0f };
        float sizeScale{ 2.0f };
        float color[4]{ 0.90f, 1.0f, 0.95f, 0.92f };
        float worldAnchor[3]{};
        std::uint32_t worldAnchored{ 0 };
        std::uint32_t reserved[3]{};
    };

    struct RockProviderDiagnosticOverlayFrameV4
    {
        std::uint32_t size{ sizeof(RockProviderDiagnosticOverlayFrameV4) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t axisCount{ 0 };
        std::uint32_t markerCount{ 0 };
        std::uint32_t textCount{ 0 };
        RockProviderDiagnosticOverlayAxisV4 axes[ROCK_PROVIDER_MAX_DIAGNOSTIC_AXES_V4]{};
        RockProviderDiagnosticOverlayMarkerV4 markers[ROCK_PROVIDER_MAX_DIAGNOSTIC_MARKERS_V4]{};
        RockProviderDiagnosticOverlayTextV4 texts[ROCK_PROVIDER_MAX_DIAGNOSTIC_TEXT_V4]{};
        std::uint32_t reserved[8]{};
    };

    enum class RockProviderDiagnosticInputFlagsV5 : std::uint32_t
    {
        None = 0,
        PrimaryTriggerHeld = 1u << 0,
        PrimaryTriggerPressed = 1u << 1,
        PrimaryTriggerReleased = 1u << 2,
        RightThumbstickLeftPressed = 1u << 3,
        RightThumbstickRightPressed = 1u << 4,
        RightThumbstickUpPressed = 1u << 5,
        RightThumbstickDownPressed = 1u << 6,
    };

    enum class RockProviderDiagnosticSuppressionFlagsV5 : std::uint32_t
    {
        None = 0,
        PrimaryTrigger = 1u << 0,
        RightThumbstick = 1u << 1,
    };

    struct RockProviderDiagnosticInputSnapshotV5
    {
        std::uint32_t size{ sizeof(RockProviderDiagnosticInputSnapshotV5) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint32_t flags{ 0 };
        std::uint32_t reserved0{ 0 };
        float rightThumbstickX{ 0.0f };
        float rightThumbstickY{ 0.0f };
        float primaryTriggerAxisX{ 0.0f };
        std::uint32_t reserved[5]{};
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
        std::uint32_t(ROCK_PROVIDER_CALL* getWeaponEvidenceDescriptors)(RockProviderWeaponEvidenceDescriptor* outDescriptors, std::uint32_t maxDescriptors);
        bool(ROCK_PROVIDER_CALL* registerExternalBodies)(
            std::uint64_t ownerToken,
            const RockProviderExternalBodyRegistration* bodies,
            std::uint32_t bodyCount);
        void(ROCK_PROVIDER_CALL* clearExternalBodies)(std::uint64_t ownerToken);
        std::uint32_t(ROCK_PROVIDER_CALL* getExternalContactSnapshot)(RockProviderExternalContact* outContacts, std::uint32_t maxContacts);
        bool(ROCK_PROVIDER_CALL* setOffhandInteractionReservation)(std::uint64_t ownerToken, RockProviderOffhandReservation reservation);
        bool(ROCK_PROVIDER_CALL* registerExternalBodiesV2)(
            std::uint64_t ownerToken,
            const RockProviderExternalBodyRegistration* bodies,
            std::uint32_t bodyCount);
        std::uint32_t(ROCK_PROVIDER_CALL* getExternalContactSnapshotV2)(RockProviderExternalContactV2* outContacts, std::uint32_t maxContacts);
        std::uint32_t(ROCK_PROVIDER_CALL* getWeaponEvidenceDetailCountV3)();
        std::uint32_t(ROCK_PROVIDER_CALL* copyWeaponEvidenceDetailsV3)(RockProviderWeaponEvidenceDetailV3* outDetails, std::uint32_t maxDetails);
        std::uint32_t(ROCK_PROVIDER_CALL* getWeaponEvidenceDetailPointCountV3)(std::uint32_t bodyId);
        std::uint32_t(ROCK_PROVIDER_CALL* copyWeaponEvidenceDetailPointsV3)(
            std::uint32_t bodyId,
            RockProviderPoint3* outPoints,
            std::uint32_t maxPoints);
        bool(ROCK_PROVIDER_CALL* publishDiagnosticOverlayV4)(const RockProviderDiagnosticOverlayFrameV4* frame);
        bool(ROCK_PROVIDER_CALL* setDiagnosticInputSuppressionV4)(std::uint64_t ownerToken, bool suppressPrimaryTrigger);
        bool(ROCK_PROVIDER_CALL* getDiagnosticInputSnapshotV5)(std::uint64_t ownerToken, RockProviderDiagnosticInputSnapshotV5* outSnapshot);
        bool(ROCK_PROVIDER_CALL* setDiagnosticInputSuppressionV5)(std::uint64_t ownerToken, std::uint32_t suppressionFlags);
        std::uint32_t(ROCK_PROVIDER_CALL* getBodyContactSnapshotV6)(RockProviderBodyContactV6* outContacts, std::uint32_t maxContacts);
        RockProviderHand(ROCK_PROVIDER_CALL* getPrimaryHandV8)();
        RockProviderHand(ROCK_PROVIDER_CALL* getOffhandHandV8)();
        bool(ROCK_PROVIDER_CALL* getHandFrameV8)(RockProviderHand hand, RockProviderHandFrameV8* outFrame);
        RockProviderResultV9(ROCK_PROVIDER_CALL* registerConsumerV9)(
            const RockProviderConsumerRegistrationV9* registration,
            RockProviderConsumerHandleV9* outHandle);
        RockProviderResultV9(ROCK_PROVIDER_CALL* unregisterConsumerV9)(std::uint64_t ownerToken);
        std::uint32_t(ROCK_PROVIDER_CALL* getGrantedCapabilitiesV9)(std::uint64_t ownerToken);
        bool(ROCK_PROVIDER_CALL* getProviderLimitsV9)(RockProviderLimitsV9* outLimits);
        std::uint32_t(ROCK_PROVIDER_CALL* getExternalContactSnapshotForOwnerV9)(
            std::uint64_t ownerToken,
            RockProviderExternalContactV2* outContacts,
            std::uint32_t maxContacts);

        [[nodiscard]] static int initialize(const std::uint32_t minVersion = ROCK_PROVIDER_API_VERSION)
        {
            if (inst) {
                return inst->getVersion() < minVersion ? 4 : 0;
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

            inst = api;
            return 0;
        }

        inline static const RockProviderApi* inst = nullptr;
    };

    static_assert(std::is_standard_layout_v<RockProviderTransform>);
    static_assert(std::is_trivially_copyable_v<RockProviderTransform>);
    static_assert(sizeof(RockProviderConsumerRegistrationV9) == 104);
    static_assert(alignof(RockProviderConsumerRegistrationV9) == 4);
    static_assert(std::is_standard_layout_v<RockProviderConsumerRegistrationV9>);
    static_assert(std::is_trivially_copyable_v<RockProviderConsumerRegistrationV9>);
    static_assert(sizeof(RockProviderConsumerHandleV9) == 48);
    static_assert(alignof(RockProviderConsumerHandleV9) == 8);
    static_assert(std::is_standard_layout_v<RockProviderConsumerHandleV9>);
    static_assert(std::is_trivially_copyable_v<RockProviderConsumerHandleV9>);
    static_assert(sizeof(RockProviderLimitsV9) == 72);
    static_assert(alignof(RockProviderLimitsV9) == 4);
    static_assert(std::is_standard_layout_v<RockProviderLimitsV9>);
    static_assert(std::is_trivially_copyable_v<RockProviderLimitsV9>);
    static_assert(sizeof(RockProviderTransform) == 52);
    static_assert(sizeof(RockProviderFrameSnapshot) == 272);
    static_assert(alignof(RockProviderFrameSnapshot) == 8);
    static_assert(sizeof(RockProviderHandFrameV8) == 112);
    static_assert(alignof(RockProviderHandFrameV8) == 8);
    static_assert(std::is_standard_layout_v<RockProviderHandFrameV8>);
    static_assert(std::is_trivially_copyable_v<RockProviderHandFrameV8>);
    static_assert(sizeof(RockProviderExternalBodyRegistration) == 32);
    static_assert(sizeof(RockProviderExternalContact) == 40);
    static_assert(sizeof(RockProviderExternalContactV2) == 128);
    static_assert(alignof(RockProviderExternalContactV2) == 8);
    static_assert(sizeof(RockProviderWeaponEvidenceDescriptor) == 128);
    static_assert(sizeof(RockProviderPoint3) == 12);
    static_assert(sizeof(RockProviderBounds3) == 32);
    static_assert(sizeof(RockProviderWeaponEvidenceDetailV3) == 192);
    static_assert(alignof(RockProviderWeaponEvidenceDetailV3) == 8);
    static_assert(std::is_standard_layout_v<RockProviderWeaponEvidenceDetailV3>);
    static_assert(std::is_trivially_copyable_v<RockProviderWeaponEvidenceDetailV3>);
    static_assert(sizeof(RockProviderBodyContactV6) == 128);
    static_assert(alignof(RockProviderBodyContactV6) == 8);
    static_assert(std::is_standard_layout_v<RockProviderBodyContactV6>);
    static_assert(std::is_trivially_copyable_v<RockProviderBodyContactV6>);
    static_assert(sizeof(RockProviderDiagnosticOverlayAxisV4) == 88);
    static_assert(sizeof(RockProviderDiagnosticOverlayMarkerV4) == 56);
    static_assert(sizeof(RockProviderDiagnosticOverlayTextV4) == 192);
    static_assert(sizeof(RockProviderDiagnosticOverlayFrameV4) == 3920);
    static_assert(alignof(RockProviderDiagnosticOverlayFrameV4) == 8);
    static_assert(std::is_standard_layout_v<RockProviderDiagnosticOverlayFrameV4>);
    static_assert(std::is_trivially_copyable_v<RockProviderDiagnosticOverlayFrameV4>);
    static_assert(sizeof(RockProviderDiagnosticInputSnapshotV5) == 64);
    static_assert(alignof(RockProviderDiagnosticInputSnapshotV5) == 8);
    static_assert(std::is_standard_layout_v<RockProviderDiagnosticInputSnapshotV5>);
    static_assert(std::is_trivially_copyable_v<RockProviderDiagnosticInputSnapshotV5>);
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
    bool recordExternalContact(const RockProviderExternalContactV2& contact);
    RockProviderOffhandReservation currentOffhandReservation();
    std::uint32_t currentExternalBodyCount();
}
