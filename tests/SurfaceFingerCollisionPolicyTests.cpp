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
    std::array<std::int8_t, policy::kFingerCount> directions{};
    contacts[1][2] = {
        .blockedDepthGameUnits = 0.4f,
        .closingProbeTravelGameUnits = 0.2f,
        .openingProbeTravelGameUnits = -0.1f,
        .active = true,
    };

    // No commanded direction means no response, even with helpful contact.
    const auto uncommanded = policy::solve(baseline, contacts, {}, {});
    assert(nearlyEqual(uncommanded.targetOpenValues[1], 0.9f));
    assert(uncommanded.directions[1] == 0);
    assert(uncommanded.helpfulSegmentMask == 0);
    assert(!uncommanded.anyHelpfulContact);

    directions[1] = -1;
    const auto helpful = policy::solve(baseline, contacts, directions, {});
    assert(nearlyEqual(helpful.targetOpenValues[1], 0.7f));
    assert(nearlyEqual(helpful.targetOpenValues[0], 1.0f));
    assert(helpful.directions[1] == -1);
    assert(helpful.helpfulSegmentMask == (1u << 5));
    assert(helpful.anyHelpfulContact);

    // A commanded direction whose probe travel moves INTO the surface must
    // not deflect the finger.
    contacts[1][2].closingProbeTravelGameUnits = -0.2f;
    contacts[1][2].openingProbeTravelGameUnits = 0.0f;
    const auto movesIntoSurface = policy::solve(
        baseline,
        contacts,
        directions,
        {});
    assert(nearlyEqual(movesIntoSurface.targetOpenValues[1], 0.9f));
    assert(movesIntoSurface.helpfulSegmentMask == 0);
    assert(!movesIntoSurface.anyHelpfulContact);

    // The solver never flips to the other direction on its own: opening
    // travel would help here, but the command stays closed and unhelpful.
    contacts[1][2].openingProbeTravelGameUnits = 0.2f;
    contacts[1][2].blockedDepthGameUnits = 0.1f;
    const auto noSelfFlip = policy::solve(baseline, contacts, directions, {});
    assert(noSelfFlip.directions[1] == 0);
    assert(noSelfFlip.helpfulSegmentMask == 0);

    directions[1] = 1;
    const auto palmSideOpening = policy::solve(
        baseline,
        contacts,
        directions,
        {});
    assert(nearlyEqual(palmSideOpening.targetOpenValues[1], 0.95f));
    assert(palmSideOpening.directions[1] == 1);
    assert(palmSideOpening.helpfulSegmentMask == (1u << 5));

    directions = {};
    directions[2] = -1;
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
    const auto multiSegment = policy::solve(baseline, contacts, directions, {});
    assert(nearlyEqual(multiSegment.targetOpenValues[2], 0.6f));
    assert((multiSegment.helpfulSegmentMask & (1u << 6)) != 0);
    assert((multiSegment.helpfulSegmentMask & (1u << 7)) != 0);

    directions = {};
    directions[3] = -1;
    contacts = {};
    contacts[3][0] = {
        .blockedDepthGameUnits = 10.0f,
        .closingProbeTravelGameUnits = 0.1f,
        .active = true,
    };
    const auto clamped = policy::solve(
        baseline,
        contacts,
        directions,
        policy::Config{ .maximumDeflectionOpenUnits = 0.25f });
    assert(nearlyEqual(clamped.targetOpenValues[3], 0.45f));

    baseline[4] = 0.0f;
    directions = {};
    directions[4] = -1;
    contacts = {};
    contacts[4][2] = {
        .blockedDepthGameUnits = 0.2f,
        .closingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    const auto alreadyClosed = policy::solve(baseline, contacts, directions, {});
    assert(nearlyEqual(alreadyClosed.targetOpenValues[4], 0.0f));
    assert(alreadyClosed.helpfulSegmentMask == 0);
    assert(!alreadyClosed.anyHelpfulContact);

    baseline[0] = 1.0f;
    directions = {};
    directions[0] = 1;
    contacts = {};
    contacts[0][2] = {
        .blockedDepthGameUnits = 0.2f,
        .openingProbeTravelGameUnits = 0.2f,
        .active = true,
    };
    const auto alreadyOpen = policy::solve(baseline, contacts, directions, {});
    assert(nearlyEqual(alreadyOpen.targetOpenValues[0], 1.0f));
    assert(alreadyOpen.directions[0] == 0);
    assert(alreadyOpen.helpfulSegmentMask == 0);

    /*
     * Incremental physical curl contract: the runtime re-baselines the solve
     * on the current open values each frame, so persistent blocked contact
     * must accumulate through repeated solve+advance iterations all the way
     * to the anatomical stop (a full fist), and stop stepping there.
     */
    {
        std::array<float, policy::kFingerCount> curlCurrent{
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f
        };
        std::array<std::int8_t, policy::kFingerCount> curlDirections{};
        curlDirections[1] = -1;
        std::array<std::array<policy::SegmentContact, policy::kSegmentCount>,
            policy::kFingerCount>
            curlContacts{};
        curlContacts[1][2] = {
            .blockedDepthGameUnits = 5.0f,
            .closingProbeTravelGameUnits = 0.2f,
            .active = true,
        };
        const policy::Config curlConfig{ .maximumDeflectionOpenUnits = 0.3f };
        for (int frame = 0; frame < 240; ++frame) {
            const auto step = policy::solve(
                curlCurrent,
                curlContacts,
                curlDirections,
                curlConfig);
            curlCurrent = policy::advanceOpenValues(
                curlCurrent,
                step.targetOpenValues,
                30.0f,
                1.0f / 90.0f);
        }
        assert(curlCurrent[1] < 0.01f);
        assert(nearlyEqual(curlCurrent[0], 1.0f));
        const auto parked = policy::solve(
            curlCurrent,
            curlContacts,
            curlDirections,
            curlConfig);
        // At the stop the anatomical capacity is exhausted; the solve must
        // not report progress it cannot make.
        assert(parked.targetOpenValues[1] <= curlCurrent[1] + 0.0001f);
    }

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
