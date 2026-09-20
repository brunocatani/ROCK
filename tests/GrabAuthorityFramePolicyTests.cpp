#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabOffsetAcquisition.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/hand/HandFrame.h"

#include "RE/NetImmerse/NiMatrix3.h"

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
}

int main()
{
    namespace authority = rock::grab_authority_frame_math;

    bool ok = true;
    {
        namespace acquisition = rock::grab_offset_acquisition;
        namespace frames = rock::grab_frame_math;
        namespace math = rock::transform_math;
        const auto rotationDegrees = [](const auto& a, const auto& b) {
            return acquisition::rotationAngleRadians(a.rotate, b.rotate) * 57.295779513f;
        };
        const auto rotated = [](float radians) {
            auto result = identityTransform();
            const float q[4]{ 0.0f, 0.0f, std::sin(radians / 2.0f), std::cos(radians / 2.0f) };
            result.rotate = math::havokQuaternionToNiRows<RE::NiMatrix3>(q);
            return result;
        };
        auto palm = rotated(0.7f);
        palm.rotate = math::transposeRotation(palm.rotate);
        palm.translate = { -76000.0f, 82000.0f, 7700.0f };
        palm.scale = 1.5f;
        const RE::NiPoint3 grip{ 8.0f, -3.0f, 5.0f };
        const RE::NiPoint3 pivotA{ 2.0f, 1.0f, -1.0f };
        auto seated = rotated(-1.5f);
        seated.scale = 1.2f;
        seated.translate = pivotA - math::localPointToWorld(seated, grip);
        auto initial = rotated(0.6f);
        initial.scale = 1.2f;
        initial.translate = { 20.0f, -10.0f, 7.0f };
        const auto initialWorld = frames::objectFromGeneratedProxyLocalSpace(palm, initial);
        auto transition = acquisition::begin(palm, initialWorld, seated, grip);
        ok &= expectTrue("acquisition captures a scaled rotated palm", transition.active);
        const auto firstProxy = acquisition::advance(transition, palm, seated, grip, 0.0f);
        const auto firstBody = frames::objectFromGeneratedProxyLocalSpace(firstProxy, seated);
        ok &= expectPointNear("initial constraint target keeps live body", firstBody.translate, initialWorld.translate, 0.02f);
        ok &= expectNear("initial target keeps live rotation", rotationDegrees(firstBody, initialWorld), 0.0f, 0.06f);
        ok &= expectPointNear("fixed pivots meet at initial grip",
            rock::generatedProxyLocalPointToWorld(firstProxy, pivotA), math::localPointToWorld(firstBody, grip), 0.02f);

        auto movedPalm = rotated(-0.4f);
        movedPalm.rotate = math::transposeRotation(movedPalm.rotate);
        movedPalm.translate = palm.translate + RE::NiPoint3{ 10.0f, 20.0f, -5.0f };
        movedPalm.scale = palm.scale;
        const auto halfProxy = acquisition::advance(transition, movedPalm, seated, grip, transition.durationSeconds * 0.5f);
        const auto halfBody = frames::objectFromGeneratedProxyLocalSpace(halfProxy, seated);
        const auto firstGrip = math::localPointToWorld(initial, grip);
        const auto lastGrip = math::localPointToWorld(seated, grip);
        const auto middleGrip = (firstGrip + lastGrip) * 0.5f;
        ok &= expectPointNear("grip moves on straight segment in current hand space",
            math::localPointToWorld(halfBody, grip), rock::generatedProxyLocalPointToWorld(movedPalm, middleGrip), 0.03f);
        const auto halfLocal = frames::objectInGeneratedProxyLocalSpace(movedPalm, halfBody);
        ok &= expectNear("rotation shares grip progress", rotationDegrees(initial, halfLocal),
            rotationDegrees(initial, seated) * 0.5f, 0.02f);
        ok &= expectPointNear("nonzero pivot A stays coherent during acquisition",
            rock::generatedProxyLocalPointToWorld(halfProxy, pivotA), math::localPointToWorld(halfBody, grip), 0.03f);
        const float beforePause = transition.elapsedSeconds;
        (void)acquisition::advance(transition, movedPalm, seated, grip, std::numeric_limits<float>::quiet_NaN());
        ok &= expectNear("invalid delta does not advance acquisition", transition.elapsedSeconds, beforePause, 0.0f);
        const auto lastProxy = acquisition::advance(transition, movedPalm, seated, grip, transition.durationSeconds);
        ok &= expectPointNear("finished target returns exactly to live palm", lastProxy.translate, movedPalm.translate, 0.0f);

        ok &= expectFalse("completed acquisition stops advancing", transition.active);
        initial.translate = {};
        initial.translate = pivotA - math::localPointToWorld(initial, grip);
        transition = acquisition::begin(palm, frames::objectFromGeneratedProxyLocalSpace(palm, initial), seated, grip);
        ok &= expectTrue("pure rotation begins", transition.active);
        // Use a representable zero-error capture for the no-op case. The large
        // world-coordinate round trip above carries float position quantization.
        const auto origin = identityTransform();
        transition = acquisition::begin(origin, frames::objectFromGeneratedProxyLocalSpace(origin, seated), seated, grip);
        ok &= expectFalse("already seated resets acquisition", transition.active);
        auto invalid = palm;
        invalid.translate.x = std::numeric_limits<float>::quiet_NaN();
        ok &= expectFalse("nonfinite capture fails closed", acquisition::begin(invalid, initialWorld, seated, grip).active);
    }

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

        const auto resolved = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates);
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

        const auto resolved = authority::resolveGrabAuthorityPivot<RE::NiPoint3>(candidates);
        ok &= expectFalse("collision fallback cannot be final authority", resolved.valid);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        rawHandWorld.translate = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };

        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };

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
                .proxyAuthorityFrameWorld = rock::makeGeneratedProxyAuthorityRelationFrame(proxyWorld),
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = pivotAWorld,
                .gripPointWorld = gripPointWorld,
                .visualNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
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
        ok &= expectPointNear("pivot A is preserved as a non-origin proxy-local point",
            frozen.pivotAHandBodyLocalGame,
            RE::NiPoint3{ 2.0f, 0.0f, 0.0f },
            0.001f);

        const RE::NiTransform recomposedDesiredBody =
            rock::transform_math::composeTransforms(rock::makeGeneratedProxyAuthorityRelationFrame(proxyWorld), frozen.proxyAuthorityBodyHandSpace);
        ok &= expectPointNear("frozen body relation recomposes through proxy frame",
            recomposedDesiredBody.translate,
            frozen.desiredBodyWorld.translate,
            0.001f);
        const RE::NiPoint3 relationPivotA =
            rock::transform_math::localPointToWorld(frozen.proxyAuthorityBodyHandSpace, frozen.pivotBConstraintLocalGame);
        ok &= expectPointNear("proxy BODY relation maps selected pivot B to pivot A local",
            relationPivotA,
            frozen.pivotAHandBodyLocalGame,
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
        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        RE::NiTransform desiredObjectWorld = identityTransform();
        desiredObjectWorld.translate = RE::NiPoint3{ 100.0f, 0.0f, 0.0f };

        RE::NiTransform desiredBodyWorld = identityTransform();
        desiredBodyWorld.translate = RE::NiPoint3{ 6.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = rawHandWorld,
                .proxyWorld = proxyWorld,
                .proxyAuthorityFrameWorld = rock::makeGeneratedProxyAuthorityRelationFrame(proxyWorld),
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .desiredObjectWorld = desiredObjectWorld,
                .desiredBodyWorld = desiredBodyWorld,
                .pivotAWorld = RE::NiPoint3{ 5.0f, 0.0f, 0.0f },
                .gripPointWorld = RE::NiPoint3{ 11.0f, 0.0f, 0.0f },
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
                .hasDesiredObjectWorld = true,
                .hasDesiredBodyWorld = true,
            });

        ok &= expectTrue("explicit desired body authority frame is valid", frozen.valid);
        ok &= expectPointNear("explicit desired body target is solver authority",
            frozen.desiredBodyWorld.translate,
            desiredBodyWorld.translate,
            0.001f);
        const RE::NiTransform oldVisualDerivedBody =
            rock::transform_math::composeTransforms(frozen.desiredObjectWorld, frozen.bodyLocal);
        ok &= expectPointNear("visual target is derived from BODY authority",
            oldVisualDerivedBody.translate,
            frozen.desiredBodyWorld.translate,
            0.001f);
    }

    {
        RE::NiTransform rawHandWorld = identityTransform();
        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };

        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        RE::NiTransform bodyWorld = identityTransform();
        bodyWorld.translate = RE::NiPoint3{ 12.0f, 0.0f, 0.0f };

        RE::NiTransform desiredObjectWorld = identityTransform();
        desiredObjectWorld.translate = RE::NiPoint3{ 100.0f, 0.0f, 0.0f };

        RE::NiTransform desiredBodyWorld = identityTransform();
        desiredBodyWorld.translate = RE::NiPoint3{ 9.0f, 0.0f, 0.0f };

        const auto frozen = authority::freezeGrabAuthorityFrame<RE::NiTransform>(
            authority::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = rawHandWorld,
                .proxyWorld = proxyWorld,
                .proxyAuthorityFrameWorld = rock::makeGeneratedProxyAuthorityRelationFrame(proxyWorld),
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .desiredObjectWorld = desiredObjectWorld,
                .desiredBodyWorld = desiredBodyWorld,
                .pivotAWorld = RE::NiPoint3{ 5.0f, 0.0f, 0.0f },
                .gripPointWorld = RE::NiPoint3{ 11.0f, 0.0f, 0.0f },
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
                .hasDesiredObjectWorld = true,
                .hasDesiredBodyWorld = true,
            });

        ok &= expectTrue("misaligned explicit body authority frame is valid", frozen.valid);
        ok &= expectPointNear("BODY target translation is canonicalized around selected pivot",
            frozen.desiredBodyWorld.translate,
            RE::NiPoint3{ 6.0f, 0.0f, 0.0f },
            0.001f);
        const RE::NiPoint3 relationPivotA =
            rock::transform_math::localPointToWorld(frozen.proxyAuthorityBodyHandSpace, frozen.pivotBConstraintLocalGame);
        ok &= expectPointNear("canonicalized BODY relation keeps transform-B on selected pivot",
            relationPivotA,
            frozen.pivotAHandBodyLocalGame,
            0.001f);
    }

    {
        const RE::NiPoint3 xAxis{ 0.0f, 1.0f, 0.0f };
        const RE::NiPoint3 yAxis{ 0.0f, 0.0f, 1.0f };
        const RE::NiPoint3 zAxis{ 1.0f, 0.0f, 0.0f };

        RE::NiTransform proxyWorld = identityTransform();
        proxyWorld.translate = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
        proxyWorld.rotate = rock::hand_bone_collider_geometry_math::matrixFromAxes<RE::NiMatrix3>(xAxis, yAxis, zAxis);

        const RE::NiPoint3 proxyLocalPivot{ 0.0f, -2.0f, 0.0f };
        const RE::NiPoint3 pivotAWorld =
            proxyWorld.translate + rock::hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(proxyWorld, proxyLocalPivot);

        ok &= expectPointNear("generated proxy pivot A freezes through stored column local space",
            rock::grab_frame_math::computePivotAHandBodyLocal(proxyWorld, pivotAWorld),
            proxyLocalPivot,
            0.001f);
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
                .proxyAuthorityFrameWorld = rock::makeGeneratedProxyAuthorityRelationFrame(frame),
                .objectWorld = objectWorld,
                .bodyWorld = bodyWorld,
                .constraintBodyWorld = bodyWorld,
                .pivotAWorld = frame.translate,
                .gripPointWorld = RE::NiPoint3{ 11.0f, 0.0f, 0.0f },
                .source = authority::GrabAuthorityPivotSource::GripSupportModel,
            });
        ok &= expectFalse("freeze rejects non-finite authority rotation", frozen.valid);
    }

    return ok ? 0 : 1;
}
