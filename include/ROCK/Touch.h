#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::touch {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Touch;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxTargets = 256;
    inline constexpr std::uint32_t kMaxScopes = 64;
    inline constexpr std::uint32_t kMaxLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;


    enum class TouchGrabKindV1 : std::uint32_t
    {
        FixedAnchor = 0,
        LimitedHinge = 1,
        LimitedPrismatic = 2,
    };

    enum class TouchGrabTargetFlagV1 : std::uint32_t
    {
        None = 0,
        AllowRightHand = 1u << 0,
        AllowLeftHand = 1u << 1,
        AllowTwoHands = 1u << 2,
        LatchOnRelease = 1u << 3,
        MatchAnyBody = 1u << 4,
        MatchStaticMotion = 1u << 5,
        MatchKeyframedMotion = 1u << 6,
        MatchDynamicMotion = 1u << 7,
        FallbackOnly = 1u << 8,
        ExcludePowerArmor = 1u << 9,
    };

    enum class TouchGrabPhaseV1 : std::uint32_t
    {
        Inactive = 0,
        Armed = 1,
        Held = 2,
        Latched = 3,
        Yielded = 4,
        Invalidated = 5,
    };

    enum class TouchGrabReleaseReasonV1 : std::uint32_t
    {
        None = 0,
        GripReleased = 1,
        OwnerYield = 2,
        TargetRemoved = 3,
        RegistrationExpired = 4,
        GenerationChanged = 5,
        WorldLost = 6,
        TargetInvalid = 7,
        HandUnavailable = 8,
    };

    enum class TouchGrabStateFlagV1 : std::uint32_t
    {
        None = 0,
        ContactPointValid = 1u << 0,
        ContactNormalValid = 1u << 1,
        CoordinateValid = 1u << 2,
        FixedAnchor = 1u << 3,
        OriginalMotionKeyframed = 1u << 4,
        OriginalMotionDynamic = 1u << 5,
        MeshSurfaceAnchor = 1u << 6,
        MeshFingerPose = 1u << 7,
        MeshCollisionFallback = 1u << 8,
    };

    enum class SurfaceGripModeV1 : std::uint32_t
    {
        CollisionAnchor = 0,
        MeshAnchor = 1,
        CollisionFallback = 2,
        AnimatedArmorBone = 3,
    };

    enum class TouchGrabHandMaskV1 : std::uint32_t
    {
        None = 0,
        Right = 1u << 0,
        Left = 1u << 1,
    };

    struct TouchGrabTargetV1
    {
        std::uint32_t size{ sizeof(TouchGrabTargetV1) };
        std::uint32_t version{ 1 };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        TouchGrabKindV1 kind{
            TouchGrabKindV1::FixedAnchor
        };
        std::uint32_t flags{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint64_t allowedLayerMask{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        Point3 pivotWorldGame{};
        Point3 axisWorldGame{};
        float minimumCoordinate{ 0.0f };
        float maximumCoordinate{ 1.0f };
        float currentCoordinate{ 0.0f };
        std::uint32_t reserved0{ 0 };
        std::uint64_t reserved[3]{};
    };

    struct TouchGrabStateV1
    {
        std::uint32_t size{ sizeof(TouchGrabStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        TouchGrabKindV1 kind{
            TouchGrabKindV1::FixedAnchor
        };
        TouchGrabPhaseV1 phase{
            TouchGrabPhaseV1::Inactive
        };
        TouchGrabReleaseReasonV1 releaseReason{
            TouchGrabReleaseReasonV1::None
        };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t activeHandMask{ 0 };
        std::uint32_t flags{ 0 };
        SurfaceGripModeV1 surfaceGripMode{
            SurfaceGripModeV1::CollisionAnchor
        };
        float currentCoordinate{ 0.0f };
        float coordinateVelocity{ 0.0f };
        Point3 contactPointGame{};
        Point3 contactNormalGame{};
        std::uint64_t frameIndex{ 0 };
        std::uint64_t sequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint64_t reserved[2]{};
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
        std::uint64_t scopeToken{}; std::uint64_t targetId{}; std::uint32_t targetGeneration{}; std::uint32_t phase{}; std::uint32_t releaseReason{}; std::uint32_t activeHandMask{};
    };
    enum class EventKindV1 : std::uint32_t { StateChanged=1 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* setTouchGrabTargetsForScopeV1)(std::uint64_t ownerToken, std::uint64_t scopeToken, const TouchGrabTargetV1* targets, std::uint32_t targetCount) noexcept;
        Status(ROCK_CALL* clearTouchGrabTargetsForScopeV1)(std::uint64_t ownerToken, std::uint64_t scopeToken) noexcept;
        Status(ROCK_CALL* copyTouchGrabStatesForScopeV1)(std::uint64_t ownerToken, std::uint64_t scopeToken, TouchGrabStateV1* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount) noexcept;
        Status(ROCK_CALL* requestTouchGrabYieldV1)(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)
