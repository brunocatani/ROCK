#include "api/TouchGrabRegistry.h"

#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <memory>

namespace
{
    using namespace rock::provider;

    constexpr std::uint32_t flag(
        const RockProviderTouchGrabTargetFlagV1 value)
    {
        return static_cast<std::uint32_t>(value);
    }

    RockProviderTouchGrabTargetV1 hinge(
        const std::uint64_t targetId,
        const std::uint32_t bodyId,
        const std::uint32_t targetGeneration = 1)
    {
        RockProviderTouchGrabTargetV1 target{};
        target.targetId = targetId;
        target.targetGeneration = targetGeneration;
        target.kind = RockProviderTouchGrabKindV1::LimitedHinge;
        target.flags =
            flag(RockProviderTouchGrabTargetFlagV1::AllowRightHand) |
            flag(RockProviderTouchGrabTargetFlagV1::AllowLeftHand) |
            flag(RockProviderTouchGrabTargetFlagV1::AllowTwoHands) |
            flag(RockProviderTouchGrabTargetFlagV1::LatchOnRelease) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchDynamicMotion);
        target.bodyId = bodyId;
        target.referenceFormId = 0x1234;
        target.referenceNativeHandle = 0x5678;
        target.leaseFrames = 3;
        target.worldGeneration = 11;
        target.skeletonGeneration = 12;
        target.providerGeneration = 13;
        target.pivotWorldGame = { 1.0f, 2.0f, 3.0f };
        target.axisWorldGame = { 0.0f, 0.0f, 1.0f };
        target.minimumCoordinate = -1.0f;
        target.maximumCoordinate = 1.0f;
        target.currentCoordinate = 0.25f;
        return target;
    }

    RockProviderTouchGrabTargetV1 wildcardAnchor(
        const std::uint64_t targetId,
        const std::uint64_t layerMask)
    {
        RockProviderTouchGrabTargetV1 target{};
        target.targetId = targetId;
        target.targetGeneration = 1;
        target.kind = RockProviderTouchGrabKindV1::FixedAnchor;
        target.flags =
            flag(RockProviderTouchGrabTargetFlagV1::AllowRightHand) |
            flag(RockProviderTouchGrabTargetFlagV1::AllowLeftHand) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchAnyBody) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchStaticMotion) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion);
        target.allowedLayerMask = layerMask;
        target.leaseFrames = 2;
        target.worldGeneration = 11;
        target.skeletonGeneration = 12;
        target.providerGeneration = 13;
        return target;
    }

    void testValidationAndExplicitPriority()
    {
        constexpr std::uint64_t owner = 0xA001;
        constexpr std::uint64_t explicitScope = 0xB001;
        constexpr std::uint64_t wildcardScope = 0xB002;
        auto registry = std::make_unique<TouchGrabRegistry>();

        auto explicitTarget = hinge(1, 100);
        assert(registry->setScope(
                   owner,
                   explicitScope,
                   &explicitTarget,
                   1,
                   40) == TouchGrabRegistry::RegistrationResult::Ok);

        auto wildcard = wildcardAnchor(2, std::uint64_t{ 1 } << 5);
        assert(registry->setScope(
                   owner,
                   wildcardScope,
                   &wildcard,
                   1,
                   40) == TouchGrabRegistry::RegistrationResult::Ok);

        auto match = registry->resolve(
            100,
            5,
            TouchGrabMotionClassV1::Keyframed,
            RockProviderHand::Right,
            11,
            12,
            13,
            40);
        assert(match.matched);
        assert(!match.wildcard);
        assert(match.target.targetId == 1);

        match = registry->resolve(
            101,
            5,
            TouchGrabMotionClassV1::Static,
            RockProviderHand::Left,
            11,
            12,
            13,
            40);
        assert(match.matched);
        assert(match.wildcard);
        assert(match.target.targetId == 2);

        match = registry->resolve(
            101,
            6,
            TouchGrabMotionClassV1::Static,
            RockProviderHand::Left,
            11,
            12,
            13,
            40);
        assert(!match.matched);

        auto invalidHinge = hinge(3, 102);
        invalidHinge.flags |=
            flag(RockProviderTouchGrabTargetFlagV1::MatchStaticMotion);
        assert(registry->setScope(
                   owner,
                   0xB003,
                   &invalidHinge,
                   1,
                   40) ==
               TouchGrabRegistry::RegistrationResult::InvalidArgument);

        auto unguardedHinge = hinge(4, 103);
        unguardedHinge.worldGeneration = 0;
        unguardedHinge.skeletonGeneration = 0;
        unguardedHinge.providerGeneration = 0;
        assert(registry->setScope(
                   owner,
                   0xB004,
                   &unguardedHinge,
                   1,
                   40) ==
               TouchGrabRegistry::RegistrationResult::InvalidArgument);

        auto duplicateBody = hinge(5, 100);
        assert(registry->setScope(
                   owner,
                   0xB005,
                   &duplicateBody,
                   1,
                   40) ==
               TouchGrabRegistry::RegistrationResult::OwnerConflict);
        assert(registry->targetCount() == 2);
    }

    void testLeaseRefreshStateAndYield()
    {
        constexpr std::uint64_t owner = 0xA010;
        constexpr std::uint64_t scope = 0xB010;
        auto registry = std::make_unique<TouchGrabRegistry>();
        auto target = hinge(10, 200, 7);
        target.leaseFrames = 2;
        assert(registry->setScope(
                   owner,
                   scope,
                   &target,
                   1,
                   100) == TouchGrabRegistry::RegistrationResult::Ok);

        std::array<RockProviderTouchGrabStateV1, 1> states{};
        assert(registry->copyStates(
                   owner,
                   scope,
                   states.data(),
                   static_cast<std::uint32_t>(states.size()),
                   100) == 1);
        assert(states[0].phase == RockProviderTouchGrabPhaseV1::Armed);
        assert(states[0].currentCoordinate == 0.25f);

        RockProviderTouchGrabStateV1 held{};
        held.targetId = target.targetId;
        held.targetGeneration = target.targetGeneration;
        held.kind = target.kind;
        held.phase = RockProviderTouchGrabPhaseV1::Held;
        held.bodyId = target.bodyId;
        held.currentCoordinate = 0.5f;
        held.activeHandMask = 1;
        assert(registry->publishState(owner, scope, held, 101));
        assert(registry->requestYield(
            owner,
            scope,
            target.targetId,
            target.targetGeneration,
            101));

        target.currentCoordinate = 0.75f;
        assert(registry->setScope(
                   owner,
                   scope,
                   &target,
                   1,
                   101) == TouchGrabRegistry::RegistrationResult::Ok);

        const auto resolved = registry->resolve(
            target.bodyId,
            0,
            TouchGrabMotionClassV1::Dynamic,
            RockProviderHand::Right,
            11,
            12,
            13,
            102);
        assert(!resolved.matched);

        TouchGrabTargetMatchV1 current{};
        assert(registry->currentTarget(
            owner,
            scope,
            target.targetId,
            target.targetGeneration,
            11,
            12,
            13,
            102,
            current));
        assert(current.yieldRequested);

        states = {};
        assert(registry->copyStates(
                   owner,
                   scope,
                   states.data(),
                   static_cast<std::uint32_t>(states.size()),
                   102) == 1);
        assert(states[0].phase == RockProviderTouchGrabPhaseV1::Held);
        assert(states[0].currentCoordinate == 0.5f);
        assert(states[0].sequence != 0);

        assert(registry->acknowledgeYield(
            owner,
            scope,
            target.targetId,
            target.targetGeneration));
        assert(registry->currentTarget(
            owner,
            scope,
            target.targetId,
            target.targetGeneration,
            11,
            12,
            13,
            102,
            current));
        assert(!current.yieldRequested);

        RockProviderTouchGrabStateV1 invalidated = held;
        invalidated.phase =
            RockProviderTouchGrabPhaseV1::Invalidated;
        assert(registry->publishState(
            owner,
            scope,
            invalidated,
            102));
        assert(!registry->resolve(
            target.bodyId,
            0,
            TouchGrabMotionClassV1::Dynamic,
            RockProviderHand::Right,
            11,
            12,
            13,
            102).matched);

        assert(!registry->currentTarget(
            owner,
            scope,
            target.targetId,
            target.targetGeneration,
            11,
            12,
            13,
            103,
            current));
        assert(registry->targetCount() == 0);
    }

    void testScopeReplacementIsTransactional()
    {
        constexpr std::uint64_t owner = 0xA020;
        constexpr std::uint64_t scope = 0xB020;
        auto registry = std::make_unique<TouchGrabRegistry>();
        auto original = hinge(20, 300);
        assert(registry->setScope(
                   owner,
                   scope,
                   &original,
                   1,
                   1) == TouchGrabRegistry::RegistrationResult::Ok);

        std::array<RockProviderTouchGrabTargetV1, 2> invalidBatch{
            hinge(21, 301),
            hinge(22, 301),
        };
        assert(registry->setScope(
                   owner,
                   scope,
                   invalidBatch.data(),
                   static_cast<std::uint32_t>(invalidBatch.size()),
                   1) ==
               TouchGrabRegistry::RegistrationResult::InvalidArgument);

        const auto match = registry->resolve(
            300,
            0,
            TouchGrabMotionClassV1::Dynamic,
            RockProviderHand::Right,
            11,
            12,
            13,
            1);
        assert(match.matched);
        assert(match.target.targetId == original.targetId);
    }
}

int main()
{
    testValidationAndExplicitPriority();
    testLeaseRefreshStateAndYield();
    testScopeReplacementIsTransactional();
    return 0;
}
