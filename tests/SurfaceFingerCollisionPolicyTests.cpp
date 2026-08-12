#include "physics-interaction/hand/DynamicHandCollisionTelemetry.h"
#include "physics-interaction/hand/SurfaceFingerCollisionPolicy.h"

#ifdef NDEBUG
#    undef NDEBUG
#endif
#include <cassert>
#include <cmath>

namespace
{
    bool nearlyEqual(const float lhs, const float rhs)
    {
        return std::abs(lhs - rhs) < 0.0001f;
    }
}

int main()
{
    namespace policy = rock::surface_finger_collision_policy;
    namespace telemetry = rock::dynamic_hand_collision_telemetry;

    static_assert(telemetry::kPalmSlot == 0);
    static_assert(telemetry::kFirstFingerSlot == 1);
    static_assert(telemetry::kFingerSlotCount == 15);
    static_assert(telemetry::kForearmSlot == 16);
    static_assert(telemetry::kBodiesPerHand == 17);
    for (std::size_t finger = 0; finger < policy::kFingerCount; ++finger) {
        for (std::size_t segment = 0; segment < policy::kSegmentCount;
             ++segment) {
            const std::size_t bodyIndex =
                telemetry::bodyIndexForFingerSegment(finger, segment);
            assert(telemetry::isFingerSlot(bodyIndex));
            assert(telemetry::fingerIndexForBodyIndex(bodyIndex) == finger);
            assert(
                telemetry::fingerSegmentIndexForBodyIndex(bodyIndex) ==
                segment);
            assert(
                telemetry::isSurfaceGrabSourceSlot(bodyIndex) ==
                (segment == 2));
        }
    }
    assert(telemetry::isSurfaceGrabSourceSlot(telemetry::kPalmSlot));
    assert(!telemetry::isSurfaceGrabSourceSlot(telemetry::kForearmSlot));

    std::array<float, policy::kFingerCount> baseline{
        1.0f, 0.9f, 0.8f, 0.7f, 0.6f
    };
    std::array<std::array<policy::SegmentContact, policy::kSegmentCount>,
        policy::kFingerCount>
        contacts{};
    contacts[1][2] = {
        .blockedDepthGameUnits = 0.4f,
        .closingProbeTravelGameUnits = 0.2f,
        .openingProbeTravelGameUnits = -0.1f,
        .active = true,
    };
    const auto helpful = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(helpful.targetOpenValues[1], 0.7f));
    assert(nearlyEqual(helpful.targetOpenValues[0], 1.0f));
    assert(helpful.directions[1] == -1);
    assert(helpful.helpfulSegmentMask == (1u << 5));
    assert(helpful.anyHelpfulContact);

    contacts[1][2].closingProbeTravelGameUnits = -0.2f;
    contacts[1][2].openingProbeTravelGameUnits = 0.0f;
    const auto movesIntoSurface = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(movesIntoSurface.targetOpenValues[1], 0.9f));
    assert(movesIntoSurface.helpfulSegmentMask == 0);
    assert(!movesIntoSurface.anyHelpfulContact);

    contacts[1][2].openingProbeTravelGameUnits = 0.2f;
    contacts[1][2].blockedDepthGameUnits = 0.1f;
    const auto palmSideOpening = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(palmSideOpening.targetOpenValues[1], 0.95f));
    assert(palmSideOpening.directions[1] == 1);
    assert(palmSideOpening.helpfulSegmentMask == (1u << 5));

    contacts = {};
    contacts[2][0] = {
        .blockedDepthGameUnits = 0.2f,
        .closingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    contacts[2][1] = {
        .blockedDepthGameUnits = 0.4f,
        .closingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    const auto multiSegment = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(multiSegment.targetOpenValues[2], 0.6f));
    assert((multiSegment.helpfulSegmentMask & (1u << 6)) != 0);
    assert((multiSegment.helpfulSegmentMask & (1u << 7)) != 0);

    contacts = {};
    contacts[3][0] = {
        .blockedDepthGameUnits = 10.0f,
        .closingProbeTravelGameUnits = 0.1f,
        .active = true,
    };
    const auto clamped = policy::solve(
        baseline,
        contacts,
        {},
        policy::Config{ .maximumDeflectionOpenUnits = 0.25f });
    assert(nearlyEqual(clamped.targetOpenValues[3], 0.45f));

    baseline[4] = 0.0f;
    contacts = {};
    contacts[4][2] = {
        .blockedDepthGameUnits = 0.2f,
        .closingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    const auto alreadyClosed = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(alreadyClosed.targetOpenValues[4], 0.0f));
    assert(alreadyClosed.helpfulSegmentMask == 0);
    assert(!alreadyClosed.anyHelpfulContact);

    baseline[0] = 1.0f;
    contacts = {};
    contacts[0][2] = {
        .blockedDepthGameUnits = 0.2f,
        .openingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    const auto alreadyOpen = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(alreadyOpen.targetOpenValues[0], 1.0f));
    assert(alreadyOpen.directions[0] == 0);
    assert(alreadyOpen.helpfulSegmentMask == 0);

    baseline[4] = 0.5f;
    contacts = {};
    contacts[4][0] = {
        .blockedDepthGameUnits = 0.1f,
        .closingProbeTravelGameUnits = 0.2f,
        .openingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    std::array<std::int8_t, policy::kFingerCount> previousDirections{};
    previousDirections[4] = 1;
    const auto stableDirection = policy::solve(
        baseline,
        contacts,
        previousDirections,
        {});
    assert(stableDirection.directions[4] == 1);
    assert(nearlyEqual(stableDirection.targetOpenValues[4], 0.55f));

    baseline[0] = 0.5f;
    contacts = {};
    contacts[0][0] = {
        .blockedDepthGameUnits = 0.1f,
        .closingProbeTravelGameUnits = 0.2f,
        .openingProbeTravelGameUnits = -0.1f,
        .active = true,
    };
    contacts[0][1] = {
        .blockedDepthGameUnits = 0.4f,
        .closingProbeTravelGameUnits = -0.1f,
        .openingProbeTravelGameUnits = 0.4f,
        .active = true,
    };
    const auto coherentFinger = policy::solve(baseline, contacts, {}, {});
    assert(coherentFinger.directions[0] == 1);
    assert(nearlyEqual(coherentFinger.targetOpenValues[0], 0.5882353f));
    assert((coherentFinger.helpfulSegmentMask & 0x02u) != 0);
    assert((coherentFinger.helpfulSegmentMask & 0x01u) == 0);

    const std::array<float, policy::kFingerCount> current{
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f
    };
    const std::array<float, policy::kFingerCount> target{
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    const auto smoothed =
        policy::advanceOpenValues(current, target, 30.0f, 1.0f / 90.0f);
    for (const float value : smoothed) {
        assert(value > 0.0f && value < 1.0f);
    }
    const auto immediate =
        policy::advanceOpenValues(current, target, 0.0f, 1.0f / 90.0f);
    for (const float value : immediate) {
        assert(nearlyEqual(value, 0.0f));
    }

    return 0;
}
