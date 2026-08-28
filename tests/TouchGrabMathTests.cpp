#include "physics-interaction/grab/TouchGrabMath.h"
#include "physics-interaction/grab/TouchGrabJoinPolicy.h"

#include <cmath>
#include <cstdint>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

namespace
{
    using namespace rock::provider;

    constexpr std::uint32_t flag(
        const RockProviderTouchGrabTargetFlagV1 value)
    {
        return static_cast<std::uint32_t>(value);
    }

    RockProviderTouchGrabTargetV1 targetForHand(
        const std::uint64_t targetId,
        const RockProviderTouchGrabTargetFlagV1 hand)
    {
        RockProviderTouchGrabTargetV1 target{};
        target.targetId = targetId;
        target.targetGeneration = 7;
        target.kind = RockProviderTouchGrabKindV1::FixedAnchor;
        target.flags =
            flag(hand) |
            flag(RockProviderTouchGrabTargetFlagV1::AllowTwoHands) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchAnyBody) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchStaticMotion) |
            flag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion);
        target.allowedLayerMask = std::uint64_t{ 1 } << 1;
        target.leaseFrames = 4;
        target.worldGeneration = 2;
        target.skeletonGeneration = 4;
        target.providerGeneration = 3;
        return target;
    }
}

int main()
{
    using namespace rock::touch_grab_math;
    constexpr float halfPi = 1.57079632679f;

    Vector3 witness{};
    assert(makePerpendicularWitness({ 0.0f, 0.0f, 1.0f }, witness));
    assert(std::abs(dot(witness, { 0.0f, 0.0f, 1.0f })) <
           1.0e-6f);

    const float positive = hingeCoordinate(
        0.25f,
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f });
    assert(std::abs(positive - (0.25f + halfPi)) < 1.0e-5f);

    const float negative = hingeCoordinate(
        0.25f,
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f });
    assert(std::abs(negative - (0.25f - halfPi)) < 1.0e-5f);

    const float slider = prismaticCoordinate(
        2.0f,
        { 5.0f, 6.0f, 7.0f },
        { 5.0f, 1.0f, 9.0f },
        { 0.0f, -1.0f, 0.0f });
    assert(std::abs(slider - 7.0f) < 1.0e-6f);

    using rock::touch_grab_join_policy::canJoinSameBody;
    auto right = targetForHand(
        100,
        RockProviderTouchGrabTargetFlagV1::AllowRightHand);
    auto left = targetForHand(
        101,
        RockProviderTouchGrabTargetFlagV1::AllowLeftHand);
    assert(canJoinSameBody(10, 20, right, 10, 20, left));
    assert(!canJoinSameBody(10, 20, right, 11, 20, left));
    assert(!canJoinSameBody(10, 20, right, 10, 21, left));

    left.flags &= ~flag(
        RockProviderTouchGrabTargetFlagV1::AllowTwoHands);
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    left = targetForHand(
        101,
        RockProviderTouchGrabTargetFlagV1::AllowLeftHand);
    ++left.targetGeneration;
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    --left.targetGeneration;
    left.allowedLayerMask = std::uint64_t{ 1 } << 2;
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    return 0;
}
