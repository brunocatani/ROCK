#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::animation {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Animation;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxNativeAuthorityOwners = 64;
    inline constexpr std::uint32_t kMaxNativeAuthorityLeaseFrames = 1200;
    inline constexpr std::uint32_t kMaxHandVisualLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxRuntimePublicationLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxPhaseCallbacks = 16;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;
    inline constexpr std::uint32_t kFingerLocalTransformCount = 15;

    enum class NativeAnimationAuthorityFlag : std::uint32_t
    {
        None = 0,
        Arms = 1u << 0,
        Hands = 1u << 1,
        Weapon = 1u << 2,
        ReloadPose = (1u << 0) | (1u << 1) | (1u << 2),
    };

    enum class NativeAnimationAuthorityStatusFlag : std::uint32_t
    {
        None = 0,
        HookInstalled = 1u << 0,
        RuntimeEnabled = 1u << 1,
        CaptureValid = 1u << 2,
        LocalReloadTestLeaseActive = 1u << 3,
        HookInstallFailed = 1u << 4,
        ThreadMismatch = 1u << 5,
        CaptureFault = 1u << 6,
        RuntimeProviderAvailable = 1u << 7,
    };

    enum class HandVisualAuthorityFlag : std::uint32_t
    {
        None = 0,
        WorldTransform = 1u << 0,
        FingerLocalTransforms = 1u << 1,
    };

    /*
     * Game-thread phases of each animation frame, for capture and
     * presentation without exposing ROCK's native detours.
     * - NativeGraphOutput: at the validated player graph-output entry, before
     *   native scene and FRIK presentation writers. Capture only; do not
     *   mutate the engine graph.
     * - BeforeRock: before ROCK writes weapon and hand presentation.
     * - AfterRock: after ROCK's interaction update.
     * - Complete: after all visual writers; closes the control frame.
     * - Presented: final readback after FRIK's world final, once per skeleton
     *   frame. Observation only; publish control and animation writes in the
     *   earlier phases.
     */
    enum class AnimationPhase : std::uint32_t
    {
        BeforeRock = 1,
        AfterRock = 2,
        Complete = 3,
        NativeGraphOutput = 4,
        Presented = 5,
    };

    enum class AnimationPhaseContextFlag : std::uint32_t
    {
        None = 0,
        RockEnabled = 1u << 0,
        ProviderReady = 1u << 1,
        SkeletonReady = 1u << 2,
        MenuBlocking = 1u << 3,
        ConfigBlocking = 1u << 4,
        VisualWritesAllowed = 1u << 5,
    };

    struct AnimationPhaseContext
    {
        std::uint32_t size{ sizeof(AnimationPhaseContext) };
        std::uint32_t version{ 1 };
        AnimationPhase phase{ AnimationPhase::BeforeRock };
        std::uint32_t flags{ 0 };
        std::uint64_t frameIndex{ 0 };
        float deltaSeconds{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // The context is borrowed for the call. Unregister stops future calls but
    // does not wait for one already started; callback and userData must stay
    // valid through it. A faulting callback revokes everything its owner holds.
    using PhaseCallback = void(ROCK_CALL*)(const AnimationPhaseContext*, void*);

    struct NativeAnimationAuthorityRequest
    {
        std::uint32_t size{ sizeof(NativeAnimationAuthorityRequest) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct NativeAnimationAuthorityState
    {
        std::uint32_t size{ sizeof(NativeAnimationAuthorityState) };
        std::uint32_t version{ 1 };
        std::uint32_t activeFlags{ 0 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t activeOwnerCount{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
    };

    // Bit N of fingerLocalTransformMask marks fingerLocalTransforms[N] (thumb,
    // index, middle, ring, pinky; three joints each, parent-local).
    struct HandVisualAuthorityRequest
    {
        std::uint32_t size{ sizeof(HandVisualAuthorityRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::int32_t priority{ 0 };
        std::uint32_t fingerLocalTransformMask{ 0 };
        Transform worldTransform{};
        Transform fingerLocalTransforms[kFingerLocalTransformCount]{};
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // Published by the mod that executes native animation authority; ROCK
    // coordinates the leases and folds this status into the authority state.
    struct NativeAnimationRuntimePublication
    {
        std::uint32_t size{ sizeof(NativeAnimationRuntimePublication) };
        std::uint32_t version{ 1 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2, Callbacks = 4 };
    inline constexpr std::uint32_t kSupportedPermissions = 7;

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
        std::uint32_t authority{};
        std::uint32_t reason{};
    };
    enum class EventKind : std::uint32_t { AuthorityLost = 1 };

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread (Write).
        Status(ROCK_CALL* setNativeAnimationAuthority)(OwnerToken owner, const NativeAnimationAuthorityRequest* request) noexcept;
        Status(ROCK_CALL* clearNativeAnimationAuthority)(OwnerToken owner) noexcept;
        // Game thread (Read).
        Status(ROCK_CALL* getNativeAnimationAuthorityState)(OwnerToken owner, NativeAnimationAuthorityState* outState) noexcept;
        // Game thread (Write).
        Status(ROCK_CALL* setHandVisualAuthority)(OwnerToken owner, const HandVisualAuthorityRequest* request) noexcept;
        Status(ROCK_CALL* clearHandVisualAuthority)(OwnerToken owner, Hand hand) noexcept;
        Status(ROCK_CALL* publishNativeAnimationRuntime)(OwnerToken owner, const NativeAnimationRuntimePublication* publication) noexcept;
        Status(ROCK_CALL* clearNativeAnimationRuntime)(OwnerToken owner) noexcept;
        // Game thread (Callbacks).
        Status(ROCK_CALL* registerAnimationPhaseCallback)(OwnerToken owner, PhaseCallback callback, void* userData, std::uint64_t* outCallbackToken) noexcept;
        Status(ROCK_CALL* unregisterAnimationPhaseCallback)(OwnerToken owner, std::uint64_t callbackToken) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
        // Any thread. Copies authority-lost events after afterSequence.
        Status(ROCK_CALL* copyEvents)(OwnerToken owner, std::uint64_t afterSequence, Event* outEvents, std::uint32_t capacity, Stream* outStream) noexcept;
    };
}

#pragma pack(pop)
