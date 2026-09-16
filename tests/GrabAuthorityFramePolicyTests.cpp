#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/weapon/WeaponSupport.h"
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

    // A long off-center grip exposes origin-based interpolation: rotating a
    // 60-unit lever must not bow a two-unit grip correction away from its line.
    for (int axis = 0; axis < 3; ++axis) {
        namespace acquisition = rock::grab_offset_acquisition;
        auto start = identityTransform();
        start.translate = { 12.0f, -5.0f, 8.0f };
        start.scale = 1.3f;
        auto target = start;
        float quaternion[4]{ 0, 0, 0, std::cos(1.45f) };
        quaternion[axis] = std::sin(1.45f);
        target.rotate = rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
        const RE::NiPoint3 grip{ 60.0f, -30.0f, 10.0f };
        const auto firstGrip = rock::transform_math::localPointToWorld(start, grip);
        const RE::NiPoint3 lastGrip = firstGrip + RE::NiPoint3{ 2.0f, 0.0f, 0.0f };
        target = rock::grab_frame_math::shiftObjectToAlignGripWithPocket(target, lastGrip,
            rock::transform_math::localPointToWorld(target, grip));
        float lastAngle = 0.0f;
        for (int step = 0; step <= 20; ++step) {
            const float fraction = step / 20.0f;
            const auto blended = acquisition::interpolateAtGrip(start, target, grip, fraction);
            const auto actualGrip = rock::transform_math::localPointToWorld(blended, grip);
            ok &= expectNear("offset grip advances along the segment", actualGrip.x, firstGrip.x + 2.0f * fraction, 0.001f);
            ok &= expectNear("offset grip never bows sideways", actualGrip.y, firstGrip.y, 0.001f);
            ok &= expectNear("offset grip never bows vertically", actualGrip.z, firstGrip.z, 0.001f);
            const float angle = acquisition::rotationAngleRadians(start.rotate, blended.rotate);
            ok &= expectTrue("orientation advances monotonically along the short arc", angle + 0.001f >= lastAngle && angle <= 2.901f);
            lastAngle = angle;
        }

        auto proxy = identityTransform();
        float proxyQuaternion[4]{ 0.2f, -0.3f, 0.1f, 0.92736185f };
        proxy.rotate = rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(proxyQuaternion);
        proxy.translate = { 100.0f, 200.0f, -30.0f };
        proxy.scale = 0.85f;
        const auto targetLocal = rock::grab_frame_math::objectInGeneratedProxyLocalSpace(proxy, target);
        auto transition = acquisition::begin(proxy, start, targetLocal, grip);
        ok &= expectTrue("rotation and translation initialize acquisition", transition.active);
        const auto initialProxy = acquisition::advance(transition, proxy, targetLocal, grip, 0.0f);
        const auto initialBody = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(initialProxy, targetLocal);
        ok &= expectNear("constraint starts at actual body x", initialBody.translate.x, start.translate.x, 0.001f);
        ok &= expectNear("constraint starts at actual body y", initialBody.translate.y, start.translate.y, 0.001f);
        ok &= expectNear("constraint starts at actual body z", initialBody.translate.z, start.translate.z, 0.001f);
        ok &= expectNear("constraint starts at actual rotation", acquisition::rotationAngleRadians(initialBody.rotate, start.rotate), 0.0f, 0.001f);

        proxy.translate.x += 25.0f;
        const auto movingProxy = acquisition::advance(transition, proxy, targetLocal, grip, 0.0f);
        const auto movingBody = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(movingProxy, targetLocal);
        ok &= expectNear("acquisition follows hand translation without world-space lag", movingBody.translate.x, start.translate.x + 25.0f, 0.001f);
        const auto endedProxy = acquisition::advance(transition, proxy, targetLocal, grip, transition.durationSeconds);
        ok &= expectFalse("acquisition ends deterministically", transition.active);
        ok &= expectNear("completion returns exact physical proxy", endedProxy.translate.x, proxy.translate.x, 0.0f);
        const auto alignedBody = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(endedProxy, targetLocal);
        const auto authoredTarget = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(proxy, targetLocal);
        ok &= expectNear("arbitrary released orientation converges to authored rotation",
            acquisition::rotationAngleRadians(alignedBody.rotate, authoredTarget.rotate), 0.0f, 0.001f);
        ok &= expectPointNear("arbitrary released position converges to authored grip",
            rock::transform_math::localPointToWorld(alignedBody, grip),
            rock::transform_math::localPointToWorld(authoredTarget, grip), 0.001f);
        const auto noCorrection = acquisition::begin(proxy,
            rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(proxy, targetLocal), targetLocal, grip);
        ok &= expectFalse("an already seated pose does not restart correction", noCorrection.active);
    }

    {
        // Both motor chains must reconstruct one root target even when their
        // bodies, local pivots, and column-authored proxies differ.
        auto root = identityTransform();
        root.translate = {100.0f, -70.0f, 20.0f};
        root.scale = 1.3f;
        root.rotate = rock::weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({0.0f, 0.0f, 1.0f}, 0.7f);
        std::array<RE::NiTransform, 2> proxy{identityTransform(), identityTransform()};
        proxy[0].translate = {105.0f, -80.0f, 12.0f};
        proxy[0].rotate = rock::weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({1.0f, 0.0f, 0.0f}, 0.4f);
        proxy[1].translate = {102.0f, -42.0f, 15.0f};
        proxy[1].rotate = rock::weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({0.0f, 1.0f, 0.0f}, -0.8f);
        proxy[1].scale = 0.9f;
        std::array<RE::NiTransform, 2> bodyLocal{identityTransform(), identityTransform()};
        bodyLocal[0].translate = {2.0f, -1.0f, 3.0f};
        bodyLocal[1].translate = {-3.0f, 8.0f, 1.0f};
        bodyLocal[1].rotate = rock::weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({0.0f, 1.0f, 0.0f}, 0.5f);
        std::array<RE::NiTransform, 2> relation{};
        for (std::size_t i = 0; i < 2; ++i) {
            relation[i] = rock::grab_frame_math::objectInGeneratedProxyLocalSpace(proxy[i],
                rock::transform_math::composeTransforms(root, bodyLocal[i]));
        }
        const RE::NiPoint3 primaryLocal{-1.0f, 2.0f, 0.0f}, supportLocal{2.0f, 25.0f, 1.0f};
        const auto primaryTarget = rock::transform_math::localPointToWorld(root, primaryLocal);
        const auto initialSupport = rock::transform_math::localPointToWorld(root, supportLocal);
        const auto span = rock::weaponSolverLength(initialSupport - primaryTarget);
        const std::array<RE::NiPoint3, 4> directions{{{1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
            {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
        for (const auto& direction : directions) {
            for (const float handSeparation : {10.0f, 70.0f}) {
                const auto supportTarget = rock::makeLockedSupportGripTarget(primaryTarget,
                    primaryTarget + direction * handSeparation, initialSupport, span, 0.001f);
                const auto solve = rock::solveTwoHandedWeaponTransformFrikPivot(
                    rock::WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>{
                        .weaponWorldTransform = root, .primaryGripLocal = primaryLocal,
                        .supportGripLocal = supportLocal, .primaryTargetWorld = primaryTarget,
                        .supportTargetWorld = supportTarget,
                    });
                ok &= expectTrue("loose two-hand target solves", solve.solved);
                ok &= expectPointNear("primary wrist pivot stays fixed",
                    rock::transform_math::localPointToWorld(solve.weaponWorldTransform, primaryLocal), primaryTarget, 0.001f);
                ok &= expectPointNear("support follows fixed-length aim ray",
                    rock::transform_math::localPointToWorld(solve.weaponWorldTransform, supportLocal), supportTarget, 0.001f);
                ok &= expectNear("two-hand turn preserves scale", solve.weaponWorldTransform.scale, root.scale, 0.0001f);
                for (std::size_t i = 0; i < 2; ++i) {
                    const auto body = rock::transform_math::composeTransforms(solve.weaponWorldTransform, bodyLocal[i]);
                    const auto targetProxy = rock::grab_frame_math::generatedProxyFromObjectWorld(body, relation[i]);
                    const auto readback = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(targetProxy, relation[i]);
                    ok &= expectPointNear("both proxies reconstruct shared body", readback.translate, body.translate, 0.001f);
                    for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
                        ok &= expectNear("shared proxy orientation", readback.rotate.entry[row][col], body.rotate.entry[row][col], 0.0001f);
                    }
                    auto physicalNi = proxy[i], targetNi = targetProxy;
                    physicalNi.rotate = rock::transform_math::transposeRotation(physicalNi.rotate);
                    targetNi.rotate = rock::transform_math::transposeRotation(targetNi.rotate);
                    const auto correction = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(physicalNi), targetNi);
                    physicalNi.translate.x += 5.0f;
                    auto releasedProxy = rock::transform_math::composeTransforms(physicalNi, correction);
                    releasedProxy.rotate = rock::transform_math::transposeRotation(releasedProxy.rotate);
                    const auto releasedBody = rock::grab_frame_math::objectFromGeneratedProxyLocalSpace(releasedProxy, relation[i]);
                    ok &= expectPointNear("remaining hand keeps seat while moving", releasedBody.translate,
                        body.translate + RE::NiPoint3{5.0f, 0.0f, 0.0f}, 0.001f);
                }
            }
        }
    }

    return ok ? 0 : 1;
}
