#include "physics-interaction/grab/GrabThreePhase.h"

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
