#include "physics-interaction/grab/GrabThreePhase.h"

#include <cmath>
#include <cstdio>
#include <cstring>

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

    bool expectPhase(
        const char* label,
        rock::grab_three_phase::AcquisitionPhase actual,
        rock::grab_three_phase::AcquisitionPhase expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %s got %s\n",
            label,
            rock::grab_three_phase::phaseName(expected),
            rock::grab_three_phase::phaseName(actual));
        return false;
    }

    bool expectReason(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected %s got %s\n", label, expected, actual ? actual : "null");
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    float pointDot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    rock::grab_three_phase::GrabPocketFrame makePocket()
    {
        rock::grab_three_phase::GrabPocketFrame pocket{};
        pocket.valid = true;
        pocket.palmCenterWorld = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        pocket.palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        pocket.pocketCenterWorld = RE::NiPoint3{ 0.0f, 0.0f, 4.0f };
        pocket.pocketRadiusGameUnits = 9.0f;
        pocket.pocketDepthGameUnits = 4.0f;
        return pocket;
    }
}

int main()
{
    using namespace rock::grab_three_phase;

    bool ok = true;

    const RE::NiTransform identityHand = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    const auto identityPocket = buildGrabPocketFrameWithPalmCenter(
        identityHand,
        false,
        RE::NiPoint3{ 10.0f, 20.0f, 30.0f },
        7.0f,
        9.0f);
    ok &= expectTrue("proxy-basis pocket frame valid", identityPocket.valid);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal X", identityPocket.palmNormalWorld.x, 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal Y", identityPocket.palmNormalWorld.y, -1.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps reversed palm normal Z", identityPocket.palmNormalWorld.z, 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket projects finger off palm normal", pointDot(identityPocket.fingerForwardWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket projects cross-palm off palm normal", pointDot(identityPocket.crossPalmWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("proxy-basis pocket keeps tangent and bitangent orthogonal", pointDot(identityPocket.fingerForwardWorld, identityPocket.crossPalmWorld), 0.0f, 0.001f);

    const auto closeSelection = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .hasFreshTouchContact = false,
        .isFarSelection = false,
        .requireEvidenceForTouchHeld = false,
        .hasTouchHeldAuthorityEvidence = false,
        .touchAcquireDistanceGameUnits = 4.0f,
        .touchContactMaxDistanceGameUnits = 9.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectTrue("normal close selection remains accepted", closeSelection.accepted);
    ok &= expectPhase("normal close selection can touch-hold from touch envelope", closeSelection.phase, AcquisitionPhase::TouchHeld);
    ok &= expectReason("normal close selection reason", closeSelection.reason, "insideTouchEnvelope");

    const auto pullArrivalWithoutEvidence = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .hasFreshTouchContact = false,
        .isFarSelection = false,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = false,
        .touchAcquireDistanceGameUnits = 4.0f,
        .touchContactMaxDistanceGameUnits = 9.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectTrue("pull arrival without evidence remains accepted", pullArrivalWithoutEvidence.accepted);
    ok &= expectPhase("pull arrival without evidence waits near", pullArrivalWithoutEvidence.phase, AcquisitionPhase::NearConverging);
    ok &= expectReason("pull arrival without evidence reason", pullArrivalWithoutEvidence.reason, "touchEnvelopeAwaitingAuthorityEvidence");

    const auto pullArrivalFreshTouchWithoutAuthority = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 6.0f, 0.0f, 0.0f },
        .hasFreshTouchContact = true,
        .isFarSelection = false,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = false,
        .touchAcquireDistanceGameUnits = 4.0f,
        .touchContactMaxDistanceGameUnits = 9.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectPhase("pull arrival fresh touch still needs explicit evidence", pullArrivalFreshTouchWithoutAuthority.phase, AcquisitionPhase::NearConverging);
    ok &= expectReason("pull arrival fresh touch without authority reason", pullArrivalFreshTouchWithoutAuthority.reason, "touchEnvelopeAwaitingAuthorityEvidence");

    const auto pullArrivalFreshTouchWithAuthority = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 6.0f, 0.0f, 0.0f },
        .hasFreshTouchContact = true,
        .isFarSelection = false,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = true,
        .touchAcquireDistanceGameUnits = 4.0f,
        .touchContactMaxDistanceGameUnits = 9.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectPhase("pull arrival fresh touch with authority can touch-hold", pullArrivalFreshTouchWithAuthority.phase, AcquisitionPhase::TouchHeld);
    ok &= expectReason("pull arrival fresh touch with authority reason", pullArrivalFreshTouchWithAuthority.reason, "freshTouchContactInPocketEnvelope");

    const auto pullArrivalWithEvidence = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .hasFreshTouchContact = false,
        .isFarSelection = false,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = true,
        .touchAcquireDistanceGameUnits = 4.0f,
        .touchContactMaxDistanceGameUnits = 9.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectPhase("pull arrival with evidence can touch-hold", pullArrivalWithEvidence.phase, AcquisitionPhase::TouchHeld);

    const auto behindPalm = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 0.0f, 0.0f, -3.0f },
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = true,
        .behindPalmToleranceGameUnits = 1.0f,
    });
    ok &= expectFalse("behind palm remains rejected even with authority evidence", behindPalm.accepted);
    ok &= expectReason("behind palm reason", behindPalm.reason, "behindPalm");

    const auto normalCloseSeat = evaluatePullCatchSeatSafety(PullCatchSeatSafetyInput{
        .grabbedFromPullCatch = false,
        .usingPinchPocket = false,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .pivotAuthorityNormalTrusted = false,
        .pivotAuthorityPositionOnly = true,
        .gripToPocketDistanceGameUnits = 1.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .pulledAdjustDistanceGameUnits = 10.5f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectTrue("normal close grabs ignore pull-catch seat gate", normalCloseSeat.allowImmediateTouchHeld);
    ok &= expectFalse("normal close grabs do not get pulled adjust", normalCloseSeat.allowPulledAdjust);
    ok &= expectReason("normal close seat reason", normalCloseSeat.reason, "notPullCatch");

    const auto pinchPullSeat = evaluatePullCatchSeatSafety(PullCatchSeatSafetyInput{
        .grabbedFromPullCatch = true,
        .usingPinchPocket = true,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .pivotAuthorityNormalTrusted = false,
        .gripToPocketDistanceGameUnits = 1.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .pulledAdjustDistanceGameUnits = 10.5f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectTrue("pinch pocket pull-catch keeps immediate touch-held", pinchPullSeat.allowImmediateTouchHeld);
    ok &= expectFalse("pinch pocket pull-catch never uses pulled adjust", pinchPullSeat.allowPulledAdjust);
    ok &= expectReason("pinch pocket seat reason", pinchPullSeat.reason, "pinchPocket");

    const auto unsafeNormalPullSeat = evaluatePullCatchSeatSafety(PullCatchSeatSafetyInput{
        .grabbedFromPullCatch = true,
        .usingPinchPocket = false,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .stablePocketTouchContact = true,
        .pivotAuthorityNormalTrusted = true,
        .pivotAuthorityPositionOnly = false,
        .gripToPocketDistanceGameUnits = 3.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .behindPalmToleranceGameUnits = 1.5f,
        .touchAcquireDistanceGameUnits = 4.0f,
        .pocketRadiusGameUnits = 9.0f,
        .pulledAdjustDistanceGameUnits = 10.5f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectFalse("unsafe pull-catch normal defers touch-held", unsafeNormalPullSeat.allowImmediateTouchHeld);
    ok &= expectTrue("unsafe pull-catch normal requires settled visual relation", unsafeNormalPullSeat.requireSettledVisualRelation);
    ok &= expectFalse("unsafe pull-catch normal disables pulled adjust", unsafeNormalPullSeat.allowPulledAdjust);
    ok &= expectReason("unsafe pull-catch normal reason", unsafeNormalPullSeat.reason, "pullCatchSeatNormalWrongSide");

    const auto safeStablePullSeat = evaluatePullCatchSeatSafety(PullCatchSeatSafetyInput{
        .grabbedFromPullCatch = true,
        .usingPinchPocket = false,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .stablePocketTouchContact = true,
        .pivotAuthorityNormalTrusted = true,
        .pivotAuthorityPositionOnly = false,
        .gripToPocketDistanceGameUnits = 6.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .behindPalmToleranceGameUnits = 1.5f,
        .touchAcquireDistanceGameUnits = 4.0f,
        .pocketRadiusGameUnits = 9.0f,
        .pulledAdjustDistanceGameUnits = 10.5f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, -1.0f },
    });
    ok &= expectTrue("safe stable pull-catch preserves immediate touch-held", safeStablePullSeat.allowImmediateTouchHeld);
    ok &= expectFalse("safe stable pull-catch does not require settled relation", safeStablePullSeat.requireSettledVisualRelation);
    ok &= expectTrue("safe stable pull-catch allows pulled adjust", safeStablePullSeat.allowPulledAdjust);
    ok &= expectNear("safe stable pull-catch applies configured adjust", safeStablePullSeat.adjustDistanceGameUnits, 10.5f, 0.001f);
    ok &= expectNear("safe stable pull-catch normal faces palm", safeStablePullSeat.normalDotPalm, -1.0f, 0.001f);
    ok &= expectReason("safe stable pull-catch reason", safeStablePullSeat.reason, "pullCatchSeatSafe");

    const auto behindPullSeat = evaluatePullCatchSeatSafety(PullCatchSeatSafetyInput{
        .grabbedFromPullCatch = true,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .stablePocketTouchContact = true,
        .pivotAuthorityNormalTrusted = true,
        .gripToPocketDistanceGameUnits = 3.0f,
        .signedPalmDistanceGameUnits = -2.0f,
        .behindPalmToleranceGameUnits = 1.0f,
        .pulledAdjustDistanceGameUnits = 10.5f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, -1.0f },
    });
    ok &= expectFalse("behind-palm pull-catch defers touch-held", behindPullSeat.allowImmediateTouchHeld);
    ok &= expectFalse("behind-palm pull-catch disables pulled adjust", behindPullSeat.allowPulledAdjust);
    ok &= expectReason("behind-palm pull-catch reason", behindPullSeat.reason, "pullCatchSeatBehindPalm");

    return ok ? 0 : 1;
}
