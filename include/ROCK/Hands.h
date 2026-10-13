#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::hands {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Hands;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 1;
    inline constexpr std::uint32_t kFingerLocalTransformCount = 15;


    enum class HandFrameFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Left = 1u << 1,
        HasSceneNode = 1u << 4,
        RootFlattenedAuthority = 1u << 5,
        PresentedVisual = 1u << 6,
    };

    enum class PresentedHandPoseFlagV1 : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        HandWorldValid = 1u << 1,
        FingerLocalsValid = 1u << 2,
        RootFlattenedReadback = 1u << 3,
    };

    struct HandFrameV1
    {
        std::uint32_t size{ sizeof(HandFrameV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        Transform transform{};
        std::uint32_t reserved[7]{};
        std::uint64_t frameIndex{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t stateSequence{ 0 };
    };

    struct PresentedHandPoseV1
    {
        std::uint32_t size{ sizeof(PresentedHandPoseV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        Transform handWorld{};
        std::uint16_t fingerLocalTransformMask{ 0 };
        std::uint16_t reserved0{ 0 };
        Transform fingerLocalTransforms[15]{};
        std::uint64_t presentationSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct HeadPoseV1 { std::uint32_t size{sizeof(HeadPoseV1)}; std::uint32_t valid{}; SampleV1 sample{}; Transform transform{}; float forwardWorld[3]{}; };

    // Roles identify the physical hands assigned to ROCK's firing/support roles
    // in this publication. They follow firing-hand transfers; they are not the
    // player's configured dominant hand. Left/Right identities never change.
    struct RolesV1 {
        std::uint32_t size{sizeof(RolesV1)};
        std::uint32_t version{1};
        SampleV1 sample{};
        Hand primary{Hand::None};
        Hand offhand{Hand::None};
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1 };
    inline constexpr std::uint32_t kSupportedPermissions = 1;

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        // Any-thread copied control snapshot. Check Status, flags and generations.
        Status(ROCK_CALL* getHandFrameV1)(OwnerToken ownerToken, Hand hand, HandFrameV1* outFrame) noexcept;
        // Last completed presentation, with its capture frame/generations.
        // Early phases may return the preceding frame; before the first final
        // capture, readback is unavailable. Presented supplies same-frame data.
        Status(ROCK_CALL* getPresentedHandFrameV1)(OwnerToken ownerToken, Hand hand, HandFrameV1* outFrame) noexcept;
        Status(ROCK_CALL* getPresentedHandPoseV1)(std::uint64_t ownerToken, Hand hand, PresentedHandPoseV1* outPose) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        // Any-thread copies; payload and SampleV1 identify the same publication.
        Status(ROCK_CALL* getHeadPose)(OwnerToken, HeadPoseV1*) noexcept;
        Status(ROCK_CALL* getRoles)(OwnerToken, RolesV1*) noexcept;
    };
}

#pragma pack(pop)
