#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::touch {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Touch;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxTargets = 256;
    inline constexpr std::uint32_t kMaxScopes = 64;
    inline constexpr std::uint32_t kMaxLeaseFrames = 120;

    /*
     * A touch-grab target turns a body or surface into something a hand can
     * hold instead of a loose-object grab. Limited mechanisms name one live
     * dynamic or keyframed body. FixedAnchor may instead match a bounded
     * collision layer/motion mask without changing that body's motion type;
     * this is the hand-to-surface latch primitive climbing consumers use.
     */
    enum class TouchGrabKind : std::uint32_t
    {
        FixedAnchor = 0,
        LimitedHinge = 1,
        LimitedPrismatic = 2,
    };

    enum class TouchGrabTargetFlag : std::uint32_t
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
        // Wildcard FixedAnchor only. Considered after explicit bodies, native
        // authored points, close objects and ordinary wildcard registrations.
        FallbackOnly = 1u << 8,
        // Exclude native power-armor furniture and power-armor actors.
        ExcludePowerArmor = 1u << 9,
    };

    enum class TouchGrabPhase : std::uint32_t
    {
        Inactive = 0,
        Armed = 1,
        Held = 2,
        Latched = 3,
        Yielded = 4,
        Invalidated = 5,
    };

    enum class TouchGrabReleaseReason : std::uint32_t
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

    enum class TouchGrabStateFlag : std::uint32_t
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

    enum class SurfaceGripMode : std::uint32_t
    {
        CollisionAnchor = 0,
        MeshAnchor = 1,
        CollisionFallback = 2,
        AnimatedArmorBone = 3,
    };

    enum class TouchGrabHandMask : std::uint32_t
    {
        None = 0,
        Right = 1u << 0,
        Left = 1u << 1,
    };

    // The generations must match the current publication; leaseFrames is
    // 1..kMaxLeaseFrames and the target expires unless set again.
    struct TouchGrabTarget
    {
        std::uint32_t size{ sizeof(TouchGrabTarget) };
        std::uint32_t version{ 1 };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        TouchGrabKind kind{ TouchGrabKind::FixedAnchor };
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
    };

    // sequence increases on every change of this target's state.
    struct TouchGrabState
    {
        std::uint32_t size{ sizeof(TouchGrabState) };
        std::uint32_t version{ 1 };
        std::uint64_t targetId{ 0 };
        std::uint32_t targetGeneration{ 0 };
        TouchGrabKind kind{ TouchGrabKind::FixedAnchor };
        TouchGrabPhase phase{ TouchGrabPhase::Inactive };
        TouchGrabReleaseReason releaseReason{ TouchGrabReleaseReason::None };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t referenceNativeHandle{ 0 };
        std::uint32_t activeHandMask{ 0 };
        std::uint32_t flags{ 0 };
        SurfaceGripMode surfaceGripMode{ SurfaceGripMode::CollisionAnchor };
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
    };

    enum class AuthoredFlag : std::uint32_t
    {
        Indicator = 1,
        ReserveGrab = 2,
        Attached = 4,
    };

    /*
     * One lease per physical hand; a competing owner is rejected. Refresh it
     * every frame (leaseFrames 1..3), with sample taken from the current
     * publication. Indicator shows ROCK's authored marker at contactWorldGame.
     * ReserveGrab consumes new ROCK grabs before static/climbing acquisition;
     * it does not revoke an existing object or climbing hold. Attached
     * requires ReserveGrab and suppresses only the owned dynamic hand's
     * simulation pairs against world-surface layers; the other hand, actors,
     * tools and queries are unaffected. Clear, expiry, owner loss and
     * generation or lifecycle loss remove the lease. The caller owns reach and
     * occupancy checks, the grip pose and the mechanism's movement.
     */
    struct AuthoredHand
    {
        std::uint32_t size{ sizeof(AuthoredHand) };
        std::uint32_t version{ 1 };
        Sample sample{};
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t leaseFrames{ 2 };
        Point3 contactWorldGame{};
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread. Replaces the scope's targets (Write).
        Status(ROCK_CALL* setTouchGrabTargetsForScope)(OwnerToken owner, std::uint64_t scopeToken, const TouchGrabTarget* targets, std::uint32_t targetCount) noexcept;
        Status(ROCK_CALL* clearTouchGrabTargetsForScope)(OwnerToken owner, std::uint64_t scopeToken) noexcept;
        // Game thread (Read). Writes up to maxStates states and the count copied.
        Status(ROCK_CALL* copyTouchGrabStatesForScope)(OwnerToken owner, std::uint64_t scopeToken, TouchGrabState* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount) noexcept;
        Status(ROCK_CALL* requestTouchGrabYield)(OwnerToken owner, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration) noexcept;
        // Game thread, including BeforeRock (Write).
        Status(ROCK_CALL* setAuthoredHand)(OwnerToken owner, const AuthoredHand* request) noexcept;
        Status(ROCK_CALL* clearAuthoredHand)(OwnerToken owner, Hand hand) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}

#pragma pack(pop)
