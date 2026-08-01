/*
 * Parity + invariant tests for the grab pose objective and deterministic
 * solve (src/physics-interaction/grab/GrabPoseObjective.h).
 *
 * The fixture goldens in GrabPoseObjectiveFixtures.h were computed by the
 * mirrored offline harness (tools/generate_grab_pose_objective_fixtures.py)
 * from real ground-truth captures. If the header's math drifts from the
 * offline math the weights were fitted against, these comparisons fail -
 * that lockstep is the point, not an implementation detail.
 */
#include "physics-interaction/grab/GrabPoseObjective.h"

#include "GrabPoseObjectiveFixtures.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace
{
    bool expectNear(const std::string& label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f (eps %.5f)\n", label.c_str(), expected, actual, epsilon);
        return false;
    }

    bool expectTrue(const std::string& label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label.c_str());
        return false;
    }

    float termEpsilon(float expected)
    {
        const float magnitude = expected >= 0.0f ? expected : -expected;
        return (std::max)(0.02f, 0.03f * magnitude);
    }

    RE::NiPoint3 toPoint(const float (&value)[3])
    {
        return RE::NiPoint3{ value[0], value[1], value[2] };
    }
}

int main()
{
    namespace objective = rock::grab_pose_objective;
    namespace fixtures = rock::grab_pose_objective_fixtures;

    bool ok = true;

    for (const auto& fixture : fixtures::kFixtures) {
        const std::string prefix = std::string("[") + fixture.name + "] ";

        std::vector<rock::GrabLocalTriangle> triangles;
        triangles.reserve(fixture.triangleCount);
        for (std::size_t i = 0; i < fixture.triangleCount; ++i) {
            const float* v = fixture.vertices + i * 9;
            triangles.push_back(rock::GrabLocalTriangle{
                RE::NiPoint3{ v[0], v[1], v[2] },
                RE::NiPoint3{ v[3], v[4], v[5] },
                RE::NiPoint3{ v[6], v[7], v[8] },
            });
        }

        objective::HandVolumeModel hand{};
        hand.valid = true;
        hand.capsules.reserve(fixture.capsuleCount);
        for (std::size_t i = 0; i < fixture.capsuleCount; ++i) {
            const float* c = fixture.capsules + i * 7;
            hand.capsules.push_back(objective::HandCapsule{
                RE::NiPoint3{ c[0], c[1], c[2] },
                RE::NiPoint3{ c[3], c[4], c[5] },
                c[6],
            });
        }
        hand.palmCenter = toPoint(fixture.palmCenter);
        hand.palmRadius = fixture.palmRadius;
        hand.tipCount = fixture.tipCount;
        for (std::size_t i = 0; i < fixture.tipCount; ++i) {
            const float* t = fixture.tips + i * 4;
            hand.tipCenters[i] = RE::NiPoint3{ t[0], t[1], t[2] };
            hand.tipRadii[i] = t[3];
        }
        hand.palmNormal = toPoint(fixture.palmNormal);

        // --- shape model parity -------------------------------------------------
        const auto model = objective::buildShapeModel(triangles);
        ok &= expectTrue(prefix + "model valid", model.valid);
        ok &= expectNear(prefix + "centroid.x", model.centroid.x, fixture.centroid[0], 0.01f);
        ok &= expectNear(prefix + "centroid.y", model.centroid.y, fixture.centroid[1], 0.01f);
        ok &= expectNear(prefix + "centroid.z", model.centroid.z, fixture.centroid[2], 0.01f);
        for (std::size_t i = 0; i < 3; ++i) {
            ok &= expectNear(prefix + "eigenvalue" + std::to_string(i),
                model.eigenvalues[i], fixture.eigenvalues[i],
                (std::max)(0.01f, 0.002f * fixture.eigenvalues[i]));
        }
        ok &= expectNear(prefix + "elongationRatio", model.elongationRatio, fixture.elongationRatio, 0.01f);
        ok &= expectNear(prefix + "secondElongationRatio", model.secondElongationRatio, fixture.secondElongationRatio, 0.01f);
        const char* shapeName =
            model.shape == objective::ShapeClass::Rod ? "rod" : (model.shape == objective::ShapeClass::Plate ? "plate" : "compact");
        ok &= expectTrue(prefix + "shape==" + fixture.shape, std::string(shapeName) == fixture.shape);
        ok &= expectNear(prefix + "girthProfileMin", model.girthProfileMinGameUnits, fixture.girthProfileMin, 0.02f);

        // --- palm normal derivation parity --------------------------------------
        const RE::NiPoint3 derivedNormal = objective::derivePalmNormal(
            toPoint(fixture.indexBase), toPoint(fixture.middleBase), toPoint(fixture.ringBase),
            toPoint(fixture.pinkyBase), toPoint(fixture.palmHeel), toPoint(fixture.palmCenter),
            model.centroid);
        ok &= expectNear(prefix + "palmNormal.x", derivedNormal.x, fixture.palmNormal[0], 0.002f);
        ok &= expectNear(prefix + "palmNormal.y", derivedNormal.y, fixture.palmNormal[1], 0.002f);
        ok &= expectNear(prefix + "palmNormal.z", derivedNormal.z, fixture.palmNormal[2], 0.002f);

        // --- term parity at the three golden poses ------------------------------
        struct GoldenPose
        {
            const char* label;
            objective::PoseDelta pose;
            const fixtures::FixtureTerms* terms;
        };
        objective::PoseDelta poseA{};
        poseA.rotate = objective::axisAngle(RE::NiPoint3{ 1.0f, 2.0f, 3.0f }, 20.0f * 0.01745329252f);
        poseA.translate = RE::NiPoint3{ 1.0f, -0.5f, 0.25f };
        objective::PoseDelta poseB{};
        poseB.rotate = objective::axisAngle(RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, 45.0f * 0.01745329252f);
        poseB.translate = RE::NiPoint3{ 0.0f, 2.0f, 0.0f };
        const GoldenPose goldenPoses[]{
            { "identity", objective::PoseDelta{}, &fixture.termsIdentity },
            { "poseA", poseA, &fixture.termsPoseA },
            { "poseB", poseB, &fixture.termsPoseB },
        };
        for (const auto& golden : goldenPoses) {
            const auto terms = objective::scorePose(triangles, model, hand, golden.pose);
            const std::string poseLabel = prefix + golden.label + ".";
            ok &= expectNear(poseLabel + "touch", terms.touch, golden.terms->touch, termEpsilon(golden.terms->touch));
            ok &= expectNear(poseLabel + "palmProx", terms.palmProx, golden.terms->palmProx, termEpsilon(golden.terms->palmProx));
            ok &= expectNear(poseLabel + "overPen", terms.overPen, golden.terms->overPen, termEpsilon(golden.terms->overPen));
            ok &= expectNear(poseLabel + "wrap", terms.wrap, golden.terms->wrap, termEpsilon(golden.terms->wrap));
            ok &= expectNear(poseLabel + "rodAxis", terms.rodAxis, golden.terms->rodAxis, termEpsilon(golden.terms->rodAxis));
            ok &= expectNear(poseLabel + "girth", terms.girth, golden.terms->girth, termEpsilon(golden.terms->girth));
        }

        // --- solve invariants ---------------------------------------------------
        const auto solved = objective::solvePose(triangles, model, hand);
        const auto solvedAgain = objective::solvePose(triangles, model, hand);
        ok &= expectTrue(prefix + "solve deterministic",
            solved.pose.translate.x == solvedAgain.pose.translate.x &&
            solved.pose.translate.y == solvedAgain.pose.translate.y &&
            solved.pose.translate.z == solvedAgain.pose.translate.z &&
            solved.totalScore == solvedAgain.totalScore);

        const auto seedTerms = objective::scorePose(triangles, model, hand, objective::PoseDelta{});
        const float seedScore = seedTerms.weightedTotal(objective::ObjectiveWeights{});
        ok &= expectTrue(prefix + "solve does not worsen the seed", solved.totalScore <= seedScore + 1.0e-3f);

        /*
         * Label stability: these fixtures ARE verified holds, so the manifold
         * plus the arrival regularizer must keep the solve near the seed.
         * Bounds derive from the offline validation (ident drift p90
         * 13.6deg/1.52gu) with slack for the deterministic search variant.
         */
        ok &= expectTrue(prefix + "solve stays near a verified hold (rot " +
                std::to_string(solved.rotationDegrees) + "deg)",
            solved.rotationDegrees <= 25.0f);
        ok &= expectTrue(prefix + "solve stays near a verified hold (trans " +
                std::to_string(solved.translationGameUnits) + "gu)",
            solved.translationGameUnits <= 3.5f);

        // The solve must never buy contact by over-penetrating the hand.
        ok &= expectTrue(prefix + "solve does not over-penetrate",
            solved.terms.overPen <= seedTerms.overPen + 0.01f);
    }

    if (!ok) {
        std::printf("GrabPoseObjectiveTests FAILED\n");
        return 1;
    }
    std::printf("GrabPoseObjectiveTests passed\n");
    return 0;
}
