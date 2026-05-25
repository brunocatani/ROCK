#include "physics-interaction/grab/GrabCore.h"

#include <array>
#include <cstdio>
#include <limits>

namespace
{
    RE::NiTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

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

    bool expectPointNear(const char* label, const RE::NiPoint3& actual, const RE::NiPoint3& expected, float epsilon)
    {
        bool ok = true;
        ok &= expectNear(label, actual.x, expected.x, epsilon);
        ok &= expectNear(label, actual.y, expected.y, epsilon);
        ok &= expectNear(label, actual.z, expected.z, epsilon);
        return ok;
    }

    bool expectRotationNear(const char* label, const RE::NiMatrix3& actual, const RE::NiMatrix3& expected, float epsilon)
    {
        bool ok = true;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                char entryLabel[128]{};
                std::snprintf(entryLabel, sizeof(entryLabel), "%s[%d][%d]", label, row, column);
                ok &= expectNear(entryLabel, actual.entry[row][column], expected.entry[row][column], epsilon);
            }
        }
        return ok;
    }

    RE::NiMatrix3 zRotation90()
    {
        RE::NiMatrix3 matrix{};
        matrix.entry[0][0] = 0.0f;
        matrix.entry[0][1] = 1.0f;
        matrix.entry[0][2] = 0.0f;
        matrix.entry[1][0] = -1.0f;
        matrix.entry[1][1] = 0.0f;
        matrix.entry[1][2] = 0.0f;
        matrix.entry[2][0] = 0.0f;
        matrix.entry[2][1] = 0.0f;
        matrix.entry[2][2] = 1.0f;
        return matrix;
    }
}

int main()
{
    namespace authority = rock::grab_authority_frame_math;

    bool ok = true;

    {
        using Candidate = authority::GrabAuthorityPivotCandidate<RE::NiPoint3>;
        const std::array<Candidate, 5> candidates{
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::CollisionFallback,
                .pointWorld = RE::NiPoint3{ 50.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "collision",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::SelectionMeshSnap,
                .pointWorld = RE::NiPoint3{ 40.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "selection",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
                .pointWorld = RE::NiPoint3{ 30.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "palm",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
                .pointWorld = RE::NiPoint3{ 20.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "support",
            },
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::PinchPocket,
                .pointWorld = RE::NiPoint3{ 10.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "pinch",
            },
        };

        const auto resolved = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, true);
        ok &= expectTrue("resolver accepts best candidate", resolved.valid);
        ok &= expectTrue("pinch pocket has highest priority", resolved.source == authority::GrabAuthorityPivotSource::PinchPocket);
        ok &= expectPointNear("pinch point selected", resolved.pointWorld, RE::NiPoint3{ 10.0f, 0.0f, 0.0f }, 0.001f);
    }

    {
        using Candidate = authority::GrabAuthorityPivotCandidate<RE::NiPoint3>;
        const std::array<Candidate, 1> candidates{
            Candidate{
                .valid = true,
                .source = authority::GrabAuthorityPivotSource::CollisionFallback,
                .pointWorld = RE::NiPoint3{ 50.0f, 0.0f, 0.0f },
                .bodyId = 7,
                .reason = "collision",
            },
        };

        const auto disabled = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, false);
        ok &= expectFalse("collision fallback is rejected when disabled", disabled.valid);

        const auto enabled = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates, true);
        ok &= expectTrue("collision fallback is accepted when enabled", enabled.valid);
        ok &= expectTrue("collision fallback source selected", enabled.source == authority::GrabAuthorityPivotSource::CollisionFallback);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        rawHandWorld.translate = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };

        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 5.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        const RE::NiPoint3 pivotAWorld{ 5.0f, 0.0f, 0.0f };
        const RE::NiPoint3 gripPointWorld{ 11.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = rawHandWorld,
                .proxyWorld = proxyWorld,
                .rawRotationProxyFrameWorld = proxyWorld,
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = pivotAWorld,
                .gripPointWorld = gripPointWorld,
                .visualNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
                .visualNormalValid = true,
            });

        ok &= expectTrue("freeze creates a valid authority frame", frozen.valid);
        ok &= expectPointNear("body-local pivot B is frozen from selected world point",
            frozen.pivotBBodyLocalGame,
            RE::NiPoint3{ -1.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("constraint-local pivot B matches selected world point",
            frozen.pivotBConstraintLocalGame,
            RE::NiPoint3{ -1.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("desired object shifts selected point to pivot A",
            frozen.desiredObjectWorld.translate,
            RE::NiPoint3{ 4.0f, 0.0f, 0.0f },
            0.001f);
        ok &= expectPointNear("desired body preserves body-local relation",
            frozen.desiredBodyWorld.translate,
            RE::NiPoint3{ 6.0f, 0.0f, 0.0f },
            0.001f);

        const RE::NiTransform recomposedDesiredBody =
            rock::transform_math::composeTransforms(proxyWorld, frozen.rawRotationProxyBodyHandSpace);
        ok &= expectPointNear("frozen body relation recomposes through proxy frame",
            recomposedDesiredBody.translate,
            frozen.desiredBodyWorld.translate,
            0.001f);

        const RE::NiPoint3 targetGripPoint =
            rock::transform_math::localPointToWorld(frozen.desiredBodyWorld, frozen.pivotBConstraintLocalGame);
        ok &= expectPointNear("frozen pivot B reaches pivot A through desired body",
            targetGripPoint,
            pivotAWorld,
            0.001f);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();

        RE::NiTransform actualProxyWorld = identityTransform();
        actualProxyWorld.rotate = zRotation90();
        actualProxyWorld.translate = RE::NiPoint3{ 2.0f, 3.0f, 4.0f };

        RE::NiTransform rawAuthorityBodyRelation = identityTransform();
        rawAuthorityBodyRelation.translate = RE::NiPoint3{ 5.0f, 0.0f, 0.0f };

        const RE::NiTransform desiredBodyWorld =
            authority::composeDesiredBodyWorldFromRawAuthority(rawHandWorld, actualProxyWorld, rawAuthorityBodyRelation);
        const RE::NiTransform solverRelation =
            authority::computeBodyRelationForActualProxyFrame(rawHandWorld, actualProxyWorld, rawAuthorityBodyRelation);
        const RE::NiTransform recomposedSolverBody =
            rock::transform_math::composeTransforms(actualProxyWorld, solverRelation);

        ok &= expectPointNear("actual proxy conversion preserves raw-authority desired body translation",
            recomposedSolverBody.translate,
            desiredBodyWorld.translate,
            0.001f);
        ok &= expectRotationNear("actual proxy conversion preserves raw-authority desired body rotation",
            recomposedSolverBody.rotate,
            desiredBodyWorld.rotate,
            0.001f);
        const bool solverRelationDiffersFromRaw =
            std::abs(solverRelation.rotate.entry[0][0] - rawAuthorityBodyRelation.rotate.entry[0][0]) > 0.01f ||
            std::abs(solverRelation.rotate.entry[0][1] - rawAuthorityBodyRelation.rotate.entry[0][1]) > 0.01f;
        ok &= expectTrue("actual proxy conversion changes only solver-space relation when proxy axes diverge",
            solverRelationDiffersFromRaw);
    }

    {
        RE::NiTransform invalidHand = identityTransform();
        invalidHand.rotate.entry[0][0] = std::numeric_limits<float>::infinity();

        RE::NiTransform frame = identityTransform();
        frame.translate = RE::NiPoint3{ 5.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = invalidHand,
                .proxyWorld = frame,
                .rawRotationProxyFrameWorld = frame,
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = frame.translate,
                .gripPointWorld = RE::NiPoint3{ 11.0f, 0.0f, 0.0f },
                .source = authority::GrabAuthorityPivotSource::PalmPocketMesh,
            });
        ok &= expectFalse("freeze rejects non-finite authority rotation", frozen.valid);
    }

    return ok ? 0 : 1;
}
