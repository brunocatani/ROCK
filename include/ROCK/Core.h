#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::core {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Core;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxOwners = 64;
    inline constexpr std::uint32_t kMaxFrameCallbacks = 16;

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

    // modName: nonempty UTF-8, NUL-terminated within the array.
    struct Registration
    {
        std::uint32_t size{ sizeof(Registration) };
        std::uint32_t version{ 1 };
        char modName[64]{};
    };

    struct Owner
    {
        std::uint32_t size{ sizeof(Owner) };
        std::uint32_t version{ 1 };
        OwnerToken ownerToken{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // ROCK's lifecycle state for one publication. The readiness fields are the
    // provider's own view this frame; lifecycleFlags (LifecycleFlag) is the
    // lifecycle state machine, which can lag them during transitions.
    struct Snapshot
    {
        std::uint32_t size{ sizeof(Snapshot) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t lifecycleFlags{ 0 };
        LifecycleReason lastLifecycleReason{ LifecycleReason::None };
        std::uint32_t frikSkeletonReady{ 0 };
        std::uint32_t menuBlocking{ 0 };
        std::uint32_t configBlocking{ 0 };
        std::uint32_t providerReady{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        float deltaSeconds{ 0.0f };
    };

    // Game thread, once per ROCK publication. The snapshot is borrowed for the
    // call. Unregister stops future calls but does not wait for one already
    // started; callback and userData must stay valid through it. A faulting
    // callback revokes everything its owner holds.
    using FrameCallback = void(ROCK_CALL*)(const Snapshot*, void*);

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Callbacks = 4 };
    inline constexpr std::uint32_t kSupportedPermissions = 5;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Any thread (Read). outValue stays valid for the process lifetime.
        Status(ROCK_CALL* getModVersion)(OwnerToken owner, const char** outValue) noexcept;
        Status(ROCK_CALL* isProviderReady)(OwnerToken owner, std::uint32_t* outValue) noexcept;
        // Any thread (Read). The latest publication's lifecycle state.
        Status(ROCK_CALL* getSnapshot)(OwnerToken owner, Snapshot* outSnapshot) noexcept;
        // Any thread outside a ROCK callback. One owner per mod name; a name
        // already registered is refused with OwnerConflict.
        Status(ROCK_CALL* registerConsumer)(const Registration* registration, Owner* outOwner) noexcept;
        // Game thread. Releases every resource and callback of the owner.
        Status(ROCK_CALL* unregisterConsumer)(OwnerToken owner) noexcept;
        // Game thread (Callbacks).
        Status(ROCK_CALL* registerFrameCallback)(OwnerToken owner, FrameCallback callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
        Status(ROCK_CALL* unregisterFrameCallback)(OwnerToken owner, std::uint64_t callbackToken) noexcept;
        // Binds an interface major discovered through ROCKAPI_QueryInterfaceV1
        // to the owner with the requested permission bits.
        Status(ROCK_CALL* bindInterface)(OwnerToken owner, InterfaceId interfaceId, std::uint32_t exactMajor, std::uint32_t permissions) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}

#pragma pack(pop)
