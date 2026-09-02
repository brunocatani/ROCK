#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabPoseCandidateSelector.h"

#include <array>
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

    namespace poseSelector = rock::grab_pose_candidate_selector;
    ok &= expectTrue("grab pose selector candidate budget is fixed", poseSelector::kRotationCandidates.size() == 13);
    ok &= expectTrue("grab pose selector proxy budget is fixed", poseSelector::kMaxProxyTriangles == 64);
    ok &= expectTrue("grab pose selector hand budget is fixed", poseSelector::kMaxHandCapsules == 19);

    std::array<rock::GrabLocalTriangle, 70> selectorSourceTriangles{};
    for (std::size_t i = 0; i < selectorSourceTriangles.size(); ++i) {
        const float x = static_cast<float>(i);
        selectorSourceTriangles[i] = rock::GrabLocalTriangle{
            RE::NiPoint3{ x, -1.0f, 0.0f },
            RE::NiPoint3{ x, 1.0f, 0.0f },
            RE::NiPoint3{ x + 0.25f, 0.0f, 0.0f },
        };
    }
    const auto boundedSelectorProxy = poseSelector::buildTriangleProxy(
        selectorSourceTriangles,
        RE::NiPoint3{});
    ok &= expectTrue("grab pose selector fills its proxy budget", boundedSelectorProxy.count == poseSelector::kMaxProxyTriangles);
    ok &= expectTrue("grab pose selector keeps the nearest triangle", boundedSelectorProxy.triangles[0].sourceIndex == 0);

    poseSelector::TriangleProxy contactProxy{};
    contactProxy.sourceCount = 1;
    contactProxy.count = 1;
    contactProxy.triangles[0] = poseSelector::detail::makeProxyTriangle(
        rock::GrabLocalTriangle{
            RE::NiPoint3{ -10.0f, -10.0f, 0.0f },
            RE::NiPoint3{ 10.0f, -10.0f, 0.0f },
            RE::NiPoint3{ 0.0f, 10.0f, 0.0f },
        },
        0);
    poseSelector::HandModel selectorHand{};
    selectorHand.capsules[0] = poseSelector::HandCapsule{
        .aWorld = RE::NiPoint3{ -1.0f, 0.0f, 1.0f },
        .bWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .radiusGameUnits = 1.0f,
    };
    selectorHand.capsuleCount = 1;
    selectorHand.palmCenterWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
    selectorHand.palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, -1.0f };
    selectorHand.palmRadiusGameUnits = 1.0f;
    selectorHand.tipCentersWorld[0] = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
    selectorHand.tipRadiiGameUnits[0] = 1.0f;
    selectorHand.tipCount = 1;
    selectorHand.valid = true;
    const auto contactEvaluation = poseSelector::evaluateCandidate(
        contactProxy,
        selectorHand,
        identityHand,
        RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
        true,
        0.0f);
    RE::NiTransform floatingObject = identityHand;
    floatingObject.translate = RE::NiPoint3{ 0.0f, 0.0f, -5.0f };
    const auto floatingEvaluation = poseSelector::evaluateCandidate(
        contactProxy,
        selectorHand,
        floatingObject,
        RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
        true,
        0.0f);
    ok &= expectTrue("grab pose selector scores a contact seat", contactEvaluation.valid);
    ok &= expectTrue("grab pose selector rejects a floating seat", floatingEvaluation.valid && floatingEvaluation.totalScore > contactEvaluation.totalScore);
    const std::array<poseSelector::CandidateEvaluation, 2> selectorEvaluations{
        floatingEvaluation,
        contactEvaluation,
    };
    const auto selectorDecision = poseSelector::selectBestCandidate(selectorEvaluations);
    ok &= expectTrue("grab pose selector chooses the bounded improvement", selectorDecision.applied && selectorDecision.candidateIndex == 1);

    constexpr float kTiltDegrees = 15.0f;
    constexpr float kExpectedFingerComponent = 0.2588190451f;
    constexpr float kExpectedCrossPalmComponent = 0.9659258263f;
    const RE::NiPoint3 fingerForward{ 1.0f, 0.0f, 0.0f };
    const auto rightPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, fingerForward, kTiltDegrees);
    const auto leftPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, -1.0f }, fingerForward, kTiltDegrees);
    ok &= expectNear("right grip tilt points toward fingertips", rightPresentationAxis.x, kExpectedFingerComponent, 0.001f);
    ok &= expectNear("right grip tilt keeps signed cross-palm Z", rightPresentationAxis.z, kExpectedCrossPalmComponent, 0.001f);
    ok &= expectNear("left grip tilt points toward fingertips", leftPresentationAxis.x, kExpectedFingerComponent, 0.001f);
    ok &= expectNear("left grip tilt keeps reversed cross-palm Z", leftPresentationAxis.z, -kExpectedCrossPalmComponent, 0.001f);
    ok &= expectNear(
        "grip tilt keeps the same fingertip angle across hands",
        pointDot(rightPresentationAxis, fingerForward),
        pointDot(leftPresentationAxis, fingerForward),
        0.001f);
    const auto rejectedSignedPresentationAxis = buildGripPresentationAxisTowardFingertips(
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, fingerForward, -kTiltDegrees);
    ok &= expectNear(
        "grip tilt rejects the retired signed-angle convention",
        lengthSquared(rejectedSignedPresentationAxis),
        0.0f,
        0.001f);

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

    const auto programmaticBehindPalm = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 0.0f, 0.0f, -30.0f },
        .programmaticArrival = true,
        .behindPalmToleranceGameUnits = 1.0f,
    });
    ok &= expectTrue("programmatic behind-palm arrival remains accepted", programmaticBehindPalm.accepted);
    ok &= expectFalse("programmatic arrival retains behind-palm telemetry", programmaticBehindPalm.frontHemisphere);
    ok &= expectPhase("programmatic arrival converges dynamically", programmaticBehindPalm.phase, AcquisitionPhase::NearConverging);
    ok &= expectReason("programmatic arrival reason", programmaticBehindPalm.reason, "programmaticArrivalBehindPalm");

    const auto programmaticFrontTouch = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .programmaticArrival = true,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = false,
        .touchAcquireDistanceGameUnits = 4.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectTrue("front-side programmatic arrival remains accepted", programmaticFrontTouch.accepted);
    ok &= expectTrue("front-side programmatic arrival retains hemisphere telemetry", programmaticFrontTouch.frontHemisphere);
    ok &= expectPhase("programmatic arrival without stable contact converges", programmaticFrontTouch.phase, AcquisitionPhase::NearConverging);
    ok &= expectReason("programmatic arrival without stable contact reason", programmaticFrontTouch.reason, "touchEnvelopeAwaitingAuthorityEvidence");

    const auto programmaticFrontTouchWithEvidence = classifyAcquisitionPhase(PhaseClassificationInput{
        .pocket = makePocket(),
        .gripSeedWorld = RE::NiPoint3{ 1.0f, 0.0f, 1.0f },
        .programmaticArrival = true,
        .requireEvidenceForTouchHeld = true,
        .hasTouchHeldAuthorityEvidence = true,
        .touchAcquireDistanceGameUnits = 4.0f,
        .nearConvergeDistanceGameUnits = 28.0f,
    });
    ok &= expectPhase("programmatic arrival with stable contact can touch-hold", programmaticFrontTouchWithEvidence.phase, AcquisitionPhase::TouchHeld);

    const auto normalCloseSeat = evaluateProgrammaticArrivalSeatSafety(ProgrammaticArrivalSeatSafetyInput{
        .programmaticArrival = false,
        .usingPinchPocket = false,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .pivotAuthorityNormalTrusted = false,
        .pivotAuthorityPositionOnly = true,
        .gripToPocketDistanceGameUnits = 1.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectTrue("normal close grabs ignore arrival seat gate", normalCloseSeat.allowImmediateTouchHeld);
    ok &= expectReason("normal close seat reason", normalCloseSeat.reason, "notProgrammaticArrival");

    const auto pinchPullSeat = evaluateProgrammaticArrivalSeatSafety(ProgrammaticArrivalSeatSafetyInput{
        .programmaticArrival = true,
        .usingPinchPocket = true,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .pivotAuthorityNormalTrusted = false,
        .gripToPocketDistanceGameUnits = 1.0f,
        .signedPalmDistanceGameUnits = 1.0f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectTrue("pinch pocket pull-catch keeps immediate touch-held", pinchPullSeat.allowImmediateTouchHeld);
    ok &= expectReason("pinch pocket seat reason", pinchPullSeat.reason, "pinchPocket");

    const auto unsafeNormalPullSeat = evaluateProgrammaticArrivalSeatSafety(ProgrammaticArrivalSeatSafetyInput{
        .programmaticArrival = true,
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
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
    });
    ok &= expectFalse("unsafe programmatic normal defers touch-held", unsafeNormalPullSeat.allowImmediateTouchHeld);
    ok &= expectTrue("unsafe programmatic normal requires settled visual relation", unsafeNormalPullSeat.requireSettledVisualRelation);
    ok &= expectReason("unsafe programmatic normal reason", unsafeNormalPullSeat.reason, "arrivalSeatNormalWrongSide");

    const auto safeStablePullSeat = evaluateProgrammaticArrivalSeatSafety(ProgrammaticArrivalSeatSafetyInput{
        .programmaticArrival = true,
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
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, -1.0f },
    });
    ok &= expectTrue("safe stable programmatic arrival preserves immediate touch-held", safeStablePullSeat.allowImmediateTouchHeld);
    ok &= expectFalse("safe stable programmatic arrival does not require settled relation", safeStablePullSeat.requireSettledVisualRelation);
    ok &= expectNear("safe stable programmatic normal faces palm", safeStablePullSeat.normalDotPalm, -1.0f, 0.001f);
    ok &= expectReason("safe stable programmatic reason", safeStablePullSeat.reason, "arrivalSeatSafe");

    const auto behindPullSeat = evaluateProgrammaticArrivalSeatSafety(ProgrammaticArrivalSeatSafetyInput{
        .programmaticArrival = true,
        .capturePhase = AcquisitionPhase::TouchHeld,
        .pocketValid = true,
        .stablePocketTouchContact = true,
        .pivotAuthorityNormalTrusted = true,
        .gripToPocketDistanceGameUnits = 3.0f,
        .signedPalmDistanceGameUnits = -2.0f,
        .behindPalmToleranceGameUnits = 1.0f,
        .palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
        .gripNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, -1.0f },
    });
    ok &= expectFalse("behind-palm programmatic arrival defers touch-held", behindPullSeat.allowImmediateTouchHeld);
    ok &= expectReason("behind-palm programmatic reason", behindPullSeat.reason, "arrivalSeatBehindPalm");

    return ok ? 0 : 1;
}
