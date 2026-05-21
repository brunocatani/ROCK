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
    ok &= expectTrue("raw-roll pocket frame valid", identityPocket.valid);
    ok &= expectNear("raw-roll pocket keeps reversed palm normal X", identityPocket.palmNormalWorld.x, 0.0f, 0.001f);
    ok &= expectNear("raw-roll pocket keeps reversed palm normal Y", identityPocket.palmNormalWorld.y, -1.0f, 0.001f);
    ok &= expectNear("raw-roll pocket keeps reversed palm normal Z", identityPocket.palmNormalWorld.z, 0.0f, 0.001f);
    ok &= expectNear("raw-roll pocket projects finger off palm normal", pointDot(identityPocket.fingerForwardWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("raw-roll pocket projects thumb side off palm normal", pointDot(identityPocket.thumbSideWorld, identityPocket.palmNormalWorld), 0.0f, 0.001f);
    ok &= expectNear("raw-roll pocket keeps tangent and bitangent orthogonal", pointDot(identityPocket.fingerForwardWorld, identityPocket.thumbSideWorld), 0.0f, 0.001f);

    const RE::NiPoint3 savedRightProxyOffset = rock::g_rockConfig.rockRightGrabAuthorityProxyOffsetGameUnits;
    const RE::NiPoint3 savedLeftProxyOffset = rock::g_rockConfig.rockLeftGrabAuthorityProxyOffsetGameUnits;
    rock::g_rockConfig.rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3{ 0.0f, -2.0f, 0.0f };
    rock::g_rockConfig.rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3{ 0.0f, -2.0f, 0.0f };
    const auto rightSemanticProxy = rock::makeGrabAuthorityProxyFrameFromSemanticPalmPocket(
        identityHand,
        RE::NiPoint3{ 10.0f, 20.0f, 30.0f },
        false);
    const auto leftSemanticProxy = rock::makeGrabAuthorityProxyFrameFromSemanticPalmPocket(
        identityHand,
        RE::NiPoint3{ 10.0f, 20.0f, 30.0f },
        true);
    ok &= expectNear("right semantic proxy Y=-2 moves palm-side X", rightSemanticProxy.translate.x, 10.0f, 0.001f);
    ok &= expectNear("right semantic proxy Y=-2 moves palm-side Y", rightSemanticProxy.translate.y, 18.0f, 0.001f);
    ok &= expectNear("right semantic proxy Y=-2 moves palm-side Z", rightSemanticProxy.translate.z, 30.0f, 0.001f);
    ok &= expectNear("left semantic proxy Y=-2 matches right palm-side X", leftSemanticProxy.translate.x, rightSemanticProxy.translate.x, 0.001f);
    ok &= expectNear("left semantic proxy Y=-2 matches right palm-side Y", leftSemanticProxy.translate.y, rightSemanticProxy.translate.y, 0.001f);
    ok &= expectNear("left semantic proxy Y=-2 matches right palm-side Z", leftSemanticProxy.translate.z, rightSemanticProxy.translate.z, 0.001f);

    RE::NiTransform rotatedHand = identityHand;
    rotatedHand.rotate.entry[0][0] = 0.0f;
    rotatedHand.rotate.entry[0][1] = 1.0f;
    rotatedHand.rotate.entry[0][2] = 0.0f;
    rotatedHand.rotate.entry[1][0] = -1.0f;
    rotatedHand.rotate.entry[1][1] = 0.0f;
    rotatedHand.rotate.entry[1][2] = 0.0f;
    rotatedHand.rotate.entry[2][0] = 0.0f;
    rotatedHand.rotate.entry[2][1] = 0.0f;
    rotatedHand.rotate.entry[2][2] = 1.0f;
    const auto rotatedSemanticProxy = rock::makeGrabAuthorityProxyFrameFromSemanticPalmPocket(
        rotatedHand,
        RE::NiPoint3{ 10.0f, 20.0f, 30.0f },
        true);
    ok &= expectNear("semantic proxy preserves raw rotation 00", rotatedSemanticProxy.rotate.entry[0][0], rotatedHand.rotate.entry[0][0], 0.001f);
    ok &= expectNear("semantic proxy preserves raw rotation 01", rotatedSemanticProxy.rotate.entry[0][1], rotatedHand.rotate.entry[0][1], 0.001f);
    ok &= expectNear("semantic proxy preserves raw rotation 10", rotatedSemanticProxy.rotate.entry[1][0], rotatedHand.rotate.entry[1][0], 0.001f);
    ok &= expectNear("semantic proxy preserves raw rotation 11", rotatedSemanticProxy.rotate.entry[1][1], rotatedHand.rotate.entry[1][1], 0.001f);
    rock::g_rockConfig.rockRightGrabAuthorityProxyOffsetGameUnits = savedRightProxyOffset;
    rock::g_rockConfig.rockLeftGrabAuthorityProxyOffsetGameUnits = savedLeftProxyOffset;

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

    return ok ? 0 : 1;
}
