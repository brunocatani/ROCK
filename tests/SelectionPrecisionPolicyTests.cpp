#include "physics-interaction/hand/HandSelection.h"

#include <cstdio>
#include <limits>

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

    rock::selection_query_policy::ShapeCastCandidateScore scoreClose(float lateral, float along, float normalDotDirection = -1.0f)
    {
        return rock::selection_query_policy::scoreShapeCastCandidate(
            rock::selection_query_policy::ShapeCastCandidateScoringInput{
                .isFarSelection = false,
                .lateralDistance = lateral,
                .alongDistance = along,
                .lateralScale = 6.0f,
                .alongScale = 25.0f,
                .normalDotDirection = normalDotDirection,
                .hasHitNormal = true });
    }

    rock::selection_query_policy::ShapeCastCandidateScore scoreFar(float lateral, float along, float normalDotDirection = -1.0f)
    {
        return rock::selection_query_policy::scoreShapeCastCandidate(
            rock::selection_query_policy::ShapeCastCandidateScoringInput{
                .isFarSelection = true,
                .lateralDistance = lateral,
                .alongDistance = along,
                .lateralScale = 21.0f,
                .alongScale = 1000.0f,
                .normalDotDirection = normalDotDirection,
                .hasHitNormal = true });
    }
}

int main()
{
    using namespace rock::selection_query_policy;

    bool ok = true;

    {
        const auto centeredLater = scoreClose(0.20f, 8.0f);
        const auto nearerButOffPalm = scoreClose(5.0f, 2.0f);
        ok &= expectTrue("close selection prefers palm-centered evidence over nearest-along clutter",
            isBetterShapeCastCandidateScore(centeredLater, nearerButOffPalm));
    }

    {
        const auto closerSameLine = scoreClose(0.20f, 3.0f);
        const auto fartherSameLine = scoreClose(0.20f, 12.0f);
        ok &= expectTrue("close selection still uses depth when lateral evidence ties",
            isBetterShapeCastCandidateScore(closerSameLine, fartherSameLine));
    }

    {
        const auto pointedFar = scoreFar(0.30f, 500.0f);
        const auto nearButOffRay = scoreFar(8.0f, 50.0f);
        ok &= expectTrue("far selection remains pointing-ray first in crowded scenes",
            isBetterShapeCastCandidateScore(pointedFar, nearButOffRay));
    }

    {
        const auto farScaleScore = scoreFar(1.0f, 8.0f);
        const auto closeScaleScore = scoreClose(1.0f, 8.0f);
        ok &= expectTrue("far-promoted close selections need close-scale score recomputation",
            closeScaleScore.valid && farScaleScore.valid && closeScaleScore.score > farScaleScore.score);
    }

    {
        const auto frontFacing = scoreClose(1.0f, 6.0f, -0.95f);
        const auto backFacing = scoreClose(1.0f, 6.0f, 0.40f);
        ok &= expectTrue("front-facing hit normal wins equal distance evidence",
            isBetterShapeCastCandidateScore(frontFacing, backFacing));
    }

    {
        const auto invalid = scoreShapeCastCandidate(ShapeCastCandidateScoringInput{
            .isFarSelection = false,
            .lateralDistance = (std::numeric_limits<float>::infinity)(),
            .alongDistance = 1.0f,
            .lateralScale = 6.0f,
            .alongScale = 25.0f });
        ok &= expectFalse("non-finite selection metric cannot produce valid score", invalid.valid);
    }

    {
        ok &= expectTrue("close selection stickiness keeps weak score improvement",
            shouldKeepCurrentCloseSelectionAgainstCandidate(0.20f, 0.18f, 12.0f, 6.0f));
        ok &= expectFalse("close selection stickiness releases for decisive score improvement",
            shouldKeepCurrentCloseSelectionAgainstCandidate(0.20f, 0.10f, 12.0f, 6.0f));
    }

    {
        ok &= expectTrue("close selection stickiness falls back to distance when score is missing",
            shouldKeepCurrentCloseSelectionAgainstCandidate((std::numeric_limits<float>::infinity)(), 0.10f, 12.0f, 10.0f));
        ok &= expectFalse("close selection distance fallback releases for much closer candidate",
            shouldKeepCurrentCloseSelectionAgainstCandidate((std::numeric_limits<float>::infinity)(), 0.10f, 12.0f, 6.0f));
    }

    return ok ? 0 : 1;
}
