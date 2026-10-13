#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::core {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Core;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxOwners = 64;
    inline constexpr std::uint32_t kMaxFrameCallbacks = 16;
    inline constexpr std::uint32_t kMaxPhaseCallbacks = 16;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;



    enum class LifecycleFlag : std::uint32_t
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

    enum class LifecycleReason : std::uint32_t
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

    enum class AnimationPhaseV1 : std::uint32_t
    {
        BeforeRock = 1,
        AfterRock = 2,
        Complete = 3,
        NativeGraphOutput = 4,
        // Final readback after FRIK's world final, once per skeleton frame.
        // Observation only: publish control/animation writes in the earlier
        // phases. Complete retains its existing control-completion meaning.
        Presented = 5,
    };

    enum class AnimationPhaseContextFlagV1 : std::uint32_t
    {
        None = 0,
        RockEnabled = 1u << 0,
        ProviderReady = 1u << 1,
        SkeletonReady = 1u << 2,
        MenuBlocking = 1u << 3,
        ConfigBlocking = 1u << 4,
        VisualWritesAllowed = 1u << 5,
    };



    struct RegistrationV1
    {
        std::uint32_t size{ sizeof(RegistrationV1) };
        std::uint32_t version{ 1 };
        char modName[64]{};
        std::uint32_t reserved[7]{};
    };

    struct OwnerV1
    {
        std::uint32_t size{ sizeof(OwnerV1) };
        std::uint32_t version{ 1 };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
    };

    struct SnapshotV1
    {
        std::uint32_t size{ sizeof(SnapshotV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t frikSkeletonReady{ 0 };
        std::uint32_t menuBlocking{ 0 };
        std::uint32_t configBlocking{ 0 };
        std::uint32_t providerReady{ 0 };
        std::uint32_t lifecycleFlags{ 0 };
        LifecycleReason lastLifecycleReason{ LifecycleReason::None };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint64_t stateSequence{ 0 };
    };

    struct AnimationPhaseContextV1
    {
        std::uint32_t size{ sizeof(AnimationPhaseContextV1) };
        std::uint32_t version{ 1 };
        AnimationPhaseV1 phase{ AnimationPhaseV1::BeforeRock };
        std::uint32_t flags{ 0 };
        std::uint64_t frameIndex{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[7]{};
    };

    using FrameCallbackV1 = void(ROCK_CALL*)(const SnapshotV1*, void*);
    using PhaseCallbackV1 = void(ROCK_CALL*)(const AnimationPhaseContextV1*, void*);

    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1, Callbacks=4 };
    inline constexpr std::uint32_t kSupportedPermissions = 5;

    struct EventV1 {
        std::uint32_t size{sizeof(EventV1)};
        std::uint32_t kind{};
        std::uint64_t sequence{};
        std::uint64_t frameIndex{};
        std::uint64_t ownerToken{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t lifecycleFlags{}; std::uint32_t reason{}; std::uint32_t providerReady{};
    };
    enum class EventKindV1 : std::uint32_t { Lifecycle=1, OwnerRevoked=2, PhysicsInitialized=3, PhysicsShutdown=4 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* getModVersion)(OwnerToken ownerToken, const char** outValue) noexcept;
        Status(ROCK_CALL* isProviderReady)(OwnerToken ownerToken, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* getFrameSnapshot)(OwnerToken ownerToken, SnapshotV1* outSnapshot) noexcept;
        Status(ROCK_CALL* registerConsumerV1)(const RegistrationV1* registration, OwnerV1* outHandle) noexcept;
        Status(ROCK_CALL* unregisterConsumerV1)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* registerAnimationPhaseCallbackV1)(std::uint64_t ownerToken, PhaseCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
        Status(ROCK_CALL* unregisterAnimationPhaseCallbackV1)(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept;
        Status(ROCK_CALL* registerFrameCallbackForOwnerV1)(std::uint64_t ownerToken, FrameCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
        Status(ROCK_CALL* unregisterFrameCallbackForOwnerV1)(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* bindInterface)(OwnerToken, InterfaceId, std::uint32_t exactMajor, std::uint32_t permissions) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)
