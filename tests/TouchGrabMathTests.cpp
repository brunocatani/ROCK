#include "physics-interaction/grab/TouchGrabMath.h"
#include "physics-interaction/grab/TouchGrabJoinPolicy.h"

#include <cmath>
#include <cstdint>
#include <cassert>

namespace
{
    using namespace rock::provider;

    constexpr std::uint32_t flag(
        const ::rock::api::touch::TouchGrabTargetFlag value)
    {
        return static_cast<std::uint32_t>(value);
    }

    ::rock::api::touch::TouchGrabTarget targetForHand(
        const std::uint64_t targetId,
        const ::rock::api::touch::TouchGrabTargetFlag hand)
    {
        ::rock::api::touch::TouchGrabTarget target{};
        target.targetId = targetId;
        target.targetGeneration = 7;
        target.kind = ::rock::api::touch::TouchGrabKind::FixedAnchor;
        target.flags =
            flag(hand) |
            flag(::rock::api::touch::TouchGrabTargetFlag::AllowTwoHands) |
            flag(::rock::api::touch::TouchGrabTargetFlag::MatchAnyBody) |
            flag(::rock::api::touch::TouchGrabTargetFlag::MatchStaticMotion) |
            flag(::rock::api::touch::TouchGrabTargetFlag::MatchKeyframedMotion);
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
        ::rock::api::touch::TouchGrabTargetFlag::AllowRightHand);
    auto left = targetForHand(
        101,
        ::rock::api::touch::TouchGrabTargetFlag::AllowLeftHand);
    assert(canJoinSameBody(10, 20, right, 10, 20, left));
    assert(!canJoinSameBody(10, 20, right, 11, 20, left));
    assert(!canJoinSameBody(10, 20, right, 10, 21, left));

    left.flags &= ~flag(
        ::rock::api::touch::TouchGrabTargetFlag::AllowTwoHands);
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    left = targetForHand(
        101,
        ::rock::api::touch::TouchGrabTargetFlag::AllowLeftHand);
    ++left.targetGeneration;
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    --left.targetGeneration;
    left.allowedLayerMask = std::uint64_t{ 1 } << 2;
    assert(!canJoinSameBody(10, 20, right, 10, 20, left));
    return 0;
}
