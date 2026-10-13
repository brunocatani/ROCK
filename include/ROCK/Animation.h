#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::animation {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Animation;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxNativeAuthorityOwners = 64;
    inline constexpr std::uint32_t kMaxNativeAuthorityLeaseFrames = 1200;
    inline constexpr std::uint32_t kMaxHandVisualLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxRuntimePublicationLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;


    enum class NativeAnimationAuthorityFlagV1 : std::uint32_t
    {
        None = 0,
        Arms = 1u << 0,
        Hands = 1u << 1,
        Weapon = 1u << 2,
        ReloadPose = (1u << 0) | (1u << 1) | (1u << 2),
    };

    enum class NativeAnimationAuthorityStatusFlagV1 : std::uint32_t
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

    enum class HandVisualAuthorityFlagV1 : std::uint32_t
    {
        None = 0,
        WorldTransform = 1u << 0,
        FingerLocalTransforms = 1u << 1,
    };

    struct NativeAnimationAuthorityRequestV1
    {
        std::uint32_t size{ sizeof(NativeAnimationAuthorityRequestV1) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[9]{};
    };

    struct NativeAnimationAuthorityStateV1
    {
        std::uint32_t size{ sizeof(NativeAnimationAuthorityStateV1) };
        std::uint32_t version{ 1 };
        std::uint32_t activeFlags{ 0 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t activeOwnerCount{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t reserved[8]{};
    };

    struct HandVisualAuthorityRequestV1
    {
        std::uint32_t size{ sizeof(HandVisualAuthorityRequestV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::int32_t priority{ 0 };
        std::uint16_t fingerLocalTransformMask{ 0 };
        std::uint16_t reserved0{ 0 };
        Transform worldTransform{};
        Transform fingerLocalTransforms[15]{};
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct NativeAnimationRuntimePublicationV1
    {
        std::uint32_t size{ sizeof(NativeAnimationRuntimePublicationV1) };
        std::uint32_t version{ 1 };
        std::uint32_t statusFlags{ 0 };
        std::uint32_t capturedTransformCount{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[6]{};
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
        std::uint32_t authority{}; std::uint32_t reason{};
    };
    enum class EventKindV1 : std::uint32_t { AuthorityLost=1 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* setNativeAnimationAuthorityV1)(std::uint64_t ownerToken, const NativeAnimationAuthorityRequestV1* request) noexcept;
        Status(ROCK_CALL* clearNativeAnimationAuthorityV1)(std::uint64_t ownerToken) noexcept;
        Status(ROCK_CALL* getNativeAnimationAuthorityStateV1)(OwnerToken ownerToken, NativeAnimationAuthorityStateV1* outState) noexcept;
        Status(ROCK_CALL* setHandVisualAuthorityV1)(std::uint64_t ownerToken, const HandVisualAuthorityRequestV1* request) noexcept;
        Status(ROCK_CALL* clearHandVisualAuthorityV1)(std::uint64_t ownerToken, Hand hand) noexcept;
        Status(ROCK_CALL* publishNativeAnimationRuntimeV1)(std::uint64_t ownerToken, const NativeAnimationRuntimePublicationV1* publication) noexcept;
        Status(ROCK_CALL* clearNativeAnimationRuntimeV1)(std::uint64_t ownerToken) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)
