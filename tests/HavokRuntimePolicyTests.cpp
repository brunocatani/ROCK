#include "physics-interaction/HavokRuntime.h"
#include "physics-interaction/HavokPhysicsTiming.h"

#include <cmath>
#include <cstdio>
#include <cstdint>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected)
    {
        if (std::fabs(actual - expected) < 0.0001f) {
            return true;
        }

        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace frik::rock;

    bool ok = true;

    ok &= expectFalse("invalid body id is rejected", havok_runtime::isValidBodyId(RE::hknpBodyId{ body_frame::kInvalidBodyId }));
    ok &= expectTrue("normal body id is accepted", havok_runtime::isValidBodyId(RE::hknpBodyId{ 42 }));

    ok &= expectFalse("invalid body slot is rejected", havok_runtime::bodySlotCanBeRead(body_frame::kInvalidBodyId, 128));
    ok &= expectFalse("body id above max readable body index is rejected",
        havok_runtime::bodySlotCanBeRead(body_frame::kMaxReadableBodyIndex + 1, body_frame::kMaxReadableBodyIndex));
    ok &= expectFalse("high-water mark above max readable body index is rejected",
        havok_runtime::bodySlotCanBeRead(42, body_frame::kMaxReadableBodyIndex + 1));
    ok &= expectFalse("body id above high-water mark is rejected", havok_runtime::bodySlotCanBeRead(43, 42));
    ok &= expectTrue("body id equal to high-water mark is accepted", havok_runtime::bodySlotCanBeRead(42, 42));
    ok &= expectTrue("max readable body index equal to high-water mark is accepted",
        havok_runtime::bodySlotCanBeRead(body_frame::kMaxReadableBodyIndex, body_frame::kMaxReadableBodyIndex));

    havok_runtime::ContactSignalPointSelectionInput weightedContact{};
    weightedContact.pointCount = 2;
    weightedContact.contactIndex = 1;
    weightedContact.pointWeights[0] = 1.0f;
    weightedContact.pointWeights[1] = 3.0f;
    weightedContact.contactNormalHavok[0] = 0.0f;
    weightedContact.contactNormalHavok[1] = 2.0f;
    weightedContact.contactNormalHavok[2] = 0.0f;
    weightedContact.contactPointsHavok[0][0] = 2.0f;
    weightedContact.contactPointsHavok[0][1] = 0.0f;
    weightedContact.contactPointsHavok[0][2] = 0.0f;
    weightedContact.contactPointsHavok[1][0] = 6.0f;
    weightedContact.contactPointsHavok[1][1] = 4.0f;
    weightedContact.contactPointsHavok[1][2] = 0.0f;

    havok_runtime::ContactSignalPointResult selectedContact{};
    ok &= expectTrue("weighted raw contact point is selected",
        havok_runtime::selectContactSignalPoint(weightedContact, selectedContact));
    ok &= expectNear("weighted point x", selectedContact.contactPointHavok[0], 5.0f);
    ok &= expectNear("weighted point y", selectedContact.contactPointHavok[1], 3.0f);
    ok &= expectNear("normalized normal y", selectedContact.contactNormalHavok[1], 1.0f);
    ok &= expectNear("contact point weight sum", selectedContact.contactPointWeightSum, 4.0f);

    const auto multiSubstepTiming = havok_physics_timing::makeTimingSample(
        1.0f / 90.0f,
        1.0f / 180.0f,
        0.0f,
        1.0f / 45.0f,
        2);
    ok &= expectNear("timing records simulated substeps", multiSubstepTiming.simulatedDeltaSeconds, 1.0f / 90.0f);
    ok &= expectNear("keyframe drive uses whole frame raw delta", havok_physics_timing::driveDeltaSeconds(multiSubstepTiming), 1.0f / 90.0f);

    const auto fallbackTiming = havok_physics_timing::makeTimingSample(
        -1.0f,
        1.0f / 180.0f,
        0.0f,
        1.0f / 90.0f,
        1);
    ok &= expectNear("invalid raw delta falls back before keyframe drive", havok_physics_timing::driveDeltaSeconds(fallbackTiming),
        havok_physics_timing::kFallbackPhysicsDeltaSeconds);

    havok_runtime::ContactSignalPointSelectionInput fallbackContact{};
    fallbackContact.pointCount = 2;
    fallbackContact.contactIndex = 1;
    fallbackContact.contactNormalHavok[2] = 1.0f;
    fallbackContact.contactPointsHavok[1][0] = 8.0f;
    fallbackContact.contactPointsHavok[1][1] = 9.0f;
    fallbackContact.contactPointsHavok[1][2] = 10.0f;
    ok &= expectTrue("unweighted raw contact point falls back to contact index",
        havok_runtime::selectContactSignalPoint(fallbackContact, selectedContact));
    ok &= expectNear("fallback point x", selectedContact.contactPointHavok[0], 8.0f);
    ok &= expectNear("fallback point z", selectedContact.contactPointHavok[2], 10.0f);

    return ok ? 0 : 1;
}
