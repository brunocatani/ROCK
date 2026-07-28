#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
    struct TestVector3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct TestMatrix3
    {
        float entry[3][4]{};
    };

    struct TestTransform
    {
        TestMatrix3 rotate{};
        TestVector3 translate{};
        float scale{ 1.0f };
    };

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

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance = 0.0001f)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }

        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    bool expectTransformNear(const char* label, const TestTransform& actual, const TestTransform& expected)
    {
        bool ok = true;
        ok &= expectNear(label, actual.translate.x, expected.translate.x);
        ok &= expectNear(label, actual.translate.y, expected.translate.y);
        ok &= expectNear(label, actual.translate.z, expected.translate.z);
        ok &= expectNear(label, actual.scale, expected.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(label, actual.rotate.entry[row][column], expected.rotate.entry[row][column]);
            }
        }
        return ok;
    }

    TestMatrix3 makeAxisAngleRotation(
        const TestVector3& axis,
        float degrees)
    {
        return rock::weaponSolverAxisAngleStored<
            TestMatrix3,
            TestVector3>(
            axis,
            degrees * 0.01745329251994329577f);
    }
}

int main()
{
    bool ok = true;

    {
        TestTransform liveHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        liveHandWorld.translate = { 10.0f, 5.0f, -2.0f };
        const TestVector3 livePalmPivot{ 12.0f, 8.0f, 1.0f };
        const TestVector3 selectedGripPoint{ 18.0f, 6.0f, 5.0f };

        TestTransform partWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        partWorld.translate = { 30.0f, 40.0f, 50.0f };
        partWorld.rotate.entry[0][0] = 0.0f;
        partWorld.rotate.entry[0][1] = 1.0f;
        partWorld.rotate.entry[1][0] = -1.0f;
        partWorld.rotate.entry[1][1] = 0.0f;
        partWorld.scale = 1.25f;

        const TestTransform seatedHandWorld = rock::weapon_two_handed_grip_math::alignHandFrameToGripPoint(
            liveHandWorld,
            livePalmPivot,
            selectedGripPoint);
        const TestTransform virtualPartWorld = rock::weapon_two_handed_grip_math::virtualizeMeshForTranslatedHandSeat(
            partWorld,
            livePalmPivot,
            selectedGripPoint);
        const TestVector3 virtualGripPoint = rock::weapon_two_handed_grip_math::virtualizeGripPointForTranslatedHandSeat(
            livePalmPivot,
            selectedGripPoint);

        ok &= expectNear("support grip seats hand translation x", seatedHandWorld.translate.x, 16.0f);
        ok &= expectNear("support grip seats hand translation y", seatedHandWorld.translate.y, 3.0f);
        ok &= expectNear("support grip seats hand translation z", seatedHandWorld.translate.z, 2.0f);
        ok &= expectNear("support grip virtual mesh applies inverse seat x", virtualPartWorld.translate.x, 24.0f);
        ok &= expectNear("support grip virtual mesh applies inverse seat y", virtualPartWorld.translate.y, 42.0f);
        ok &= expectNear("support grip virtual mesh applies inverse seat z", virtualPartWorld.translate.z, 46.0f);
        ok &= expectNear("support grip virtual seat resolves to live palm x", virtualGripPoint.x, livePalmPivot.x);
        ok &= expectNear("support grip virtual seat resolves to live palm y", virtualGripPoint.y, livePalmPivot.y);
        ok &= expectNear("support grip virtual seat resolves to live palm z", virtualGripPoint.z, livePalmPivot.z);
        ok &= expectNear("support grip virtual mesh preserves scale", virtualPartWorld.scale, partWorld.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear("support grip virtual mesh preserves rotation", virtualPartWorld.rotate.entry[row][column], partWorld.rotate.entry[row][column]);
            }
        }

        const TestVector3 finalGripFromHand{
            selectedGripPoint.x - seatedHandWorld.translate.x,
            selectedGripPoint.y - seatedHandWorld.translate.y,
            selectedGripPoint.z - seatedHandWorld.translate.z,
        };
        const TestVector3 virtualGripFromHand{
            virtualGripPoint.x - liveHandWorld.translate.x,
            virtualGripPoint.y - liveHandWorld.translate.y,
            virtualGripPoint.z - liveHandWorld.translate.z,
        };
        ok &= expectNear("support grip frozen solve preserves final hand/mesh relation x", virtualGripFromHand.x, finalGripFromHand.x);
        ok &= expectNear("support grip frozen solve preserves final hand/mesh relation y", virtualGripFromHand.y, finalGripFromHand.y);
        ok &= expectNear("support grip frozen solve preserves final hand/mesh relation z", virtualGripFromHand.z, finalGripFromHand.z);
    }

    {
        TestTransform oneHandWeapon =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        const TestVector3 primaryGripLocal{ 2.0f, 1.0f, -1.0f };
        const TestVector3 supportGripLocal{ 2.0f, 11.0f, -1.0f };
        const TestVector3 primaryTarget{ 100.0f, 50.0f, 20.0f };
        const TestVector3 supportTarget{ 110.0f, 50.0f, 20.0f };
        oneHandWeapon.translate = { 98.0f, 49.0f, 21.0f };

        rock::WeaponTwoHandedSolverInput<TestTransform, TestVector3>
            solverInput{};
        solverInput.weaponWorldTransform = oneHandWeapon;
        solverInput.primaryGripLocal = primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryTarget;
        solverInput.supportTargetWorld = supportTarget;
        solverInput.supportNormalLocal = { 0.0f, 0.0f, 1.0f };
        solverInput.supportNormalTargetWorld = { 0.0f, 1.0f, 0.0f };
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = 0.5f;

        const auto fullSolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
        auto axisOnlyInput = solverInput;
        axisOnlyInput.useSupportNormalTwist = false;
        axisOnlyInput.supportNormalTwistFactor = 0.0f;
        const auto axisOnlySolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(axisOnlyInput);
        ok &= expectTrue(
            "dynamic acquisition fixture full solve succeeds",
            fullSolve.solved);
        ok &= expectTrue(
            "dynamic acquisition fixture axis solve succeeds",
            axisOnlySolve.solved);

        const float fullCorrectionRadians =
            rock::weapon_support_acquisition_math::
                rotationAngleRadians(fullSolve.rotationDelta);
        const float twistContributionRadians =
            rock::weapon_support_acquisition_math::
                rotationDistanceRadians(
                    axisOnlySolve.rotationDelta,
                    fullSolve.rotationDelta);
        ok &= expectTrue(
            "dynamic acquisition composite includes support normal twist",
            twistContributionRadians >
                1.0f * 0.01745329251994329577f);

        constexpr std::array<float, 5> kAcquisitionAlphas{
            0.0f,
            0.25f,
            0.5f,
            0.75f,
            1.0f,
        };
        for (const float alpha : kAcquisitionAlphas) {
            const auto acquired =
                rock::weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        TestTransform,
                        TestVector3>(
                        oneHandWeapon,
                        fullSolve.rotationDelta,
                        primaryGripLocal,
                        primaryTarget,
                        alpha);
            ok &= expectTrue(
                "dynamic acquisition partial solve stays valid",
                acquired.valid);
            const TestVector3 primaryWorld =
                rock::transform_math::localPointToWorld(
                    acquired.weaponWorldTransform,
                    primaryGripLocal);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot x exact",
                primaryWorld.x,
                primaryTarget.x);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot y exact",
                primaryWorld.y,
                primaryTarget.y);
            ok &= expectNear(
                "dynamic acquisition keeps primary pivot z exact",
                primaryWorld.z,
                primaryTarget.z);
            ok &= expectNear(
                "dynamic acquisition slerps the complete correction",
                acquired.appliedRotationRadians,
                fullCorrectionRadians * alpha,
                0.001f);
            if (alpha == 0.0f) {
                ok &= expectTransformNear(
                    "dynamic acquisition alpha zero preserves one-hand frame",
                    acquired.weaponWorldTransform,
                    oneHandWeapon);
            }
            if (alpha == 1.0f) {
                ok &= expectTransformNear(
                    "dynamic acquisition alpha one matches full solver",
                    acquired.weaponWorldTransform,
                    fullSolve.weaponWorldTransform);
            }
        }

        const TestVector3 movingPrimaryTarget{
            primaryTarget.x + 7.0f,
            primaryTarget.y - 4.0f,
            primaryTarget.z + 3.0f,
        };
        const auto movingPrimaryAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    fullSolve.rotationDelta,
                    primaryGripLocal,
                    movingPrimaryTarget,
                    0.35f);
        const TestVector3 movingPrimaryWorld =
            rock::transform_math::localPointToWorld(
                movingPrimaryAcquire.weaponWorldTransform,
                primaryGripLocal);
        ok &= expectNear(
            "dynamic acquisition follows moving primary x immediately",
            movingPrimaryWorld.x,
            movingPrimaryTarget.x);
        ok &= expectNear(
            "dynamic acquisition follows moving primary y immediately",
            movingPrimaryWorld.y,
            movingPrimaryTarget.y);
        ok &= expectNear(
            "dynamic acquisition follows moving primary z immediately",
            movingPrimaryWorld.z,
            movingPrimaryTarget.z);

        const TestMatrix3 almostFullTurn =
            makeAxisAngleRotation(
                TestVector3{ 0.0f, 0.0f, 1.0f },
                350.0f);
        const auto shortestArcAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    almostFullTurn,
                    primaryGripLocal,
                    primaryTarget,
                    0.5f);
        ok &= expectNear(
            "dynamic acquisition uses quaternion shortest arc",
            shortestArcAcquire.appliedRotationRadians,
            5.0f * 0.01745329251994329577f,
            0.001f);

        ok &= expectNear(
            "dynamic acquisition smoothstep quarter",
            rock::weapon_support_acquisition_math::smoothStepAlpha(
                0.25f),
            0.15625f);
        ok &= expectNear(
            "dynamic acquisition zero duration reaches endpoint",
            rock::weapon_support_acquisition_math::
                timedSmoothStepAlpha(0.0f, 0.0f),
            1.0f);

        TestMatrix3 invalidRotation = fullSolve.rotationDelta;
        invalidRotation.entry[0][0] =
            (std::numeric_limits<float>::quiet_NaN)();
        const auto invalidAcquire =
            rock::weapon_support_acquisition_math::
                applyRotationAroundPrimaryPivot<
                    TestTransform,
                    TestVector3>(
                    oneHandWeapon,
                    invalidRotation,
                    primaryGripLocal,
                    primaryTarget,
                    0.5f);
        ok &= expectFalse(
            "dynamic acquisition rejects non-finite correction",
            invalidAcquire.valid);
        ok &= expectTransformNear(
            "dynamic acquisition non-finite correction fails closed",
            invalidAcquire.weaponWorldTransform,
            oneHandWeapon);
    }

    {
        TestTransform weaponBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        weaponBefore.translate = { 10.0f, 20.0f, 30.0f };
        TestTransform scopeBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        scopeBefore.translate = { 12.0f, 24.0f, 35.0f };
        TestTransform weaponAfter = weaponBefore;
        weaponAfter.translate = { 17.0f, 16.0f, 32.0f };
        weaponAfter.rotate.entry[0][0] = 0.0f;
        weaponAfter.rotate.entry[0][1] = 1.0f;
        weaponAfter.rotate.entry[1][0] = -1.0f;
        weaponAfter.rotate.entry[1][1] = 0.0f;
        const TestTransform relativeBefore = rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(weaponBefore),
            scopeBefore);

        const TestVector3 sightBoundsMin{ -4.0f, 8.0f, 7.0f };
        const TestVector3 sightBoundsMax{ 6.0f, 38.0f, 15.0f };
        const TestVector3 sightAnchor = rock::native_scope_camera_follow_math::rearPlaneCenterFromSightBounds(sightBoundsMin, sightBoundsMax);
        ok &= expectNear("native scope sight anchor centers lateral bounds", sightAnchor.x, 1.0f);
        ok &= expectNear("native scope sight anchor uses rear forward plane", sightAnchor.y, 8.0f);
        ok &= expectNear("native scope sight anchor centers vertical bounds", sightAnchor.z, 11.0f);

        const TestVector3 firingGripWeaponLocal{ -2.0f, 3.0f, 1.0f };
        const TestVector3 fallbackOffsetWeaponLocal{ 1.5f, 12.0f, 7.0f };
        const auto generatedResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                true,
                sightAnchor,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("valid generated sight remains the preferred scope anchor",
            generatedResolution.valid &&
                generatedResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::GeneratedSight);
        ok &= expectNear("generated scope anchor keeps sight x", generatedResolution.weaponLocal.x, sightAnchor.x);
        ok &= expectNear("generated scope anchor keeps sight y", generatedResolution.weaponLocal.y, sightAnchor.y);
        ok &= expectNear("generated scope anchor keeps sight z", generatedResolution.weaponLocal.z, sightAnchor.z);

        const auto missingGeometryResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                false,
                TestVector3{},
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("missing scope geometry selects the firing-grip fallback",
            missingGeometryResolution.valid &&
                missingGeometryResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        ok &= expectNear("firing-grip fallback adds lateral offset", missingGeometryResolution.weaponLocal.x, -0.5f);
        ok &= expectNear("firing-grip fallback adds forward offset", missingGeometryResolution.weaponLocal.y, 15.0f);
        ok &= expectNear("firing-grip fallback adds vertical offset", missingGeometryResolution.weaponLocal.z, 8.0f);

        const auto forcedFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                true,
                sightAnchor,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("forced fallback bypasses an incorrectly accepted sight collider",
            forcedFallbackResolution.valid &&
                forcedFallbackResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        const auto unavailableFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                true,
                sightAnchor,
                false,
                TestVector3{},
                fallbackOffsetWeaponLocal);
        ok &= expectFalse("forced fallback fails closed without a current firing grip",
            unavailableFallbackResolution.valid);
        TestVector3 invalidGeneratedSight = sightAnchor;
        invalidGeneratedSight.x =
            (std::numeric_limits<float>::quiet_NaN)();
        const auto invalidGeometryResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                false,
                true,
                invalidGeneratedSight,
                true,
                firingGripWeaponLocal,
                fallbackOffsetWeaponLocal);
        ok &= expectTrue("non-finite generated sight falls back to the firing grip",
            invalidGeometryResolution.valid &&
                invalidGeometryResolution.source ==
                    rock::native_scope_sight_anchor_policy::AnchorSource::FiringGripFallback);
        TestVector3 invalidFallbackOffset = fallbackOffsetWeaponLocal;
        invalidFallbackOffset.z =
            (std::numeric_limits<float>::infinity)();
        const auto invalidFallbackResolution =
            rock::native_scope_sight_anchor_policy::resolve(
                true,
                false,
                TestVector3{},
                true,
                firingGripWeaponLocal,
                invalidFallbackOffset);
        ok &= expectFalse("non-finite firing-grip offset fails closed",
            invalidFallbackResolution.valid);

        const TestTransform rigidSightFrameLocal =
            rock::native_scope_camera_follow_math::captureRigidAnchorFrameWeaponLocal(
                weaponBefore,
                scopeBefore,
                sightAnchor);
        const TestTransform anchoredScopeAfter =
            rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(
                weaponAfter,
                rigidSightFrameLocal);
        const TestTransform anchoredRelativeAfter = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(weaponAfter), anchoredScopeAfter);
        ok &= expectNear("native scope camera replaces controller-relative lateral position", anchoredRelativeAfter.translate.x, sightAnchor.x);
        ok &= expectNear("native scope camera replaces controller-relative forward position", anchoredRelativeAfter.translate.y, sightAnchor.y);
        ok &= expectNear("native scope camera replaces controller-relative vertical position", anchoredRelativeAfter.translate.z, sightAnchor.z);
        ok &= expectNear("native scope camera preserves calibrated scale", anchoredRelativeAfter.scale, relativeBefore.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear("native scope camera preserves calibrated rotation", anchoredRelativeAfter.rotate.entry[row][column], relativeBefore.rotate.entry[row][column]);
            }
        }

        TestTransform fallbackCameraBase = rigidSightFrameLocal;
        fallbackCameraBase.translate =
            missingGeometryResolution.weaponLocal;
        const TestTransform zeroFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    0.0f,
                    0.0f);
        ok &= expectTransformNear(
            "zero fallback rotation preserves native camera calibration",
            zeroFallbackRotation,
            fallbackCameraBase);

        const TestTransform pitchFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    90.0f,
                    0.0f,
                    0.0f);
        ok &= expectNear("fallback pitch keeps firing-grip anchor x", pitchFallbackRotation.translate.x, fallbackCameraBase.translate.x);
        ok &= expectNear("fallback pitch keeps firing-grip anchor y", pitchFallbackRotation.translate.y, fallbackCameraBase.translate.y);
        ok &= expectNear("fallback pitch keeps firing-grip anchor z", pitchFallbackRotation.translate.z, fallbackCameraBase.translate.z);
        ok &= expectNear("fallback pitch preserves native camera scale", pitchFallbackRotation.scale, fallbackCameraBase.scale);
        ok &= expectNear("fallback pitch rotates local Y toward Z", pitchFallbackRotation.rotate.entry[1][2], 1.0f);
        ok &= expectNear("fallback pitch rotates local Z toward negative Y", pitchFallbackRotation.rotate.entry[2][1], -1.0f);

        const TestTransform yawFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    90.0f,
                    0.0f);
        ok &= expectNear("fallback yaw rotates local X toward Y", yawFallbackRotation.rotate.entry[0][1], 1.0f);
        ok &= expectNear("fallback yaw rotates local Y toward negative X", yawFallbackRotation.rotate.entry[1][0], -1.0f);

        const TestTransform rollFallbackRotation =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    fallbackCameraBase,
                    0.0f,
                    0.0f,
                    90.0f);
        ok &= expectNear("fallback roll rotates local X toward negative Z", rollFallbackRotation.rotate.entry[0][2], -1.0f);
        ok &= expectNear("fallback roll rotates local Z toward X", rollFallbackRotation.rotate.entry[2][0], 1.0f);

        TestTransform calibratedFallbackCamera = fallbackCameraBase;
        calibratedFallbackCamera.rotate =
            yawFallbackRotation.rotate;
        const TestTransform weaponAxisPitchFallback =
            rock::native_scope_camera_follow_math::
                applyWeaponLocalRotationOffset(
                    calibratedFallbackCamera,
                    90.0f,
                    0.0f,
                    0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 x", weaponAxisPitchFallback.rotate.entry[0][0], 0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 y", weaponAxisPitchFallback.rotate.entry[0][1], 0.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row0 z", weaponAxisPitchFallback.rotate.entry[0][2], 1.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row1 x", weaponAxisPitchFallback.rotate.entry[1][0], -1.0f);
        ok &= expectNear("fallback pitch uses weapon X after nonidentity native calibration row2 y", weaponAxisPitchFallback.rotate.entry[2][1], -1.0f);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor x", weaponAxisPitchFallback.translate.x, fallbackCameraBase.translate.x);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor y", weaponAxisPitchFallback.translate.y, fallbackCameraBase.translate.y);
        ok &= expectNear("weapon-axis fallback rotation never orbits the anchor z", weaponAxisPitchFallback.translate.z, fallbackCameraBase.translate.z);

        ok &= expectNear("rigid scope frame stores generated sight x", rigidSightFrameLocal.translate.x, sightAnchor.x);
        ok &= expectNear("rigid scope frame stores generated sight y", rigidSightFrameLocal.translate.y, sightAnchor.y);
        ok &= expectNear("rigid scope frame stores generated sight z", rigidSightFrameLocal.translate.z, sightAnchor.z);
        const std::array<TestTransform, 3> handModeWeaponFrames{
            weaponBefore,
            weaponAfter,
            [] {
                TestTransform leftFiring = rock::transform_math::makeIdentityTransform<TestTransform>();
                leftFiring.translate = { -18.0f, 6.0f, 42.0f };
                leftFiring.rotate.entry[0][0] = -1.0f;
                leftFiring.rotate.entry[1][1] = -1.0f;
                return leftFiring;
            }(),
        };
        for (const TestTransform& handModeWeaponFrame : handModeWeaponFrames) {
            const TestTransform rigidScopeWorld = rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(handModeWeaponFrame, rigidSightFrameLocal);
            const TestTransform resolvedLocal = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(handModeWeaponFrame), rigidScopeWorld);
            ok &= expectTransformNear("one-hand, two-hand, and left-hand modes preserve one rigid scope frame", resolvedLocal, rigidSightFrameLocal);
        }

        const rock::native_scope_activation_geometry::ConeThresholds coneThresholds{};
        TestTransform coneWeapon = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform coneHmd = rock::transform_math::makeIdentityTransform<TestTransform>();
        const TestVector3 zeroOffset{};
        const auto centeredConeSample = rock::native_scope_activation_geometry::sample(coneWeapon, TestVector3{ 0.0f, 30.0f, 0.0f }, coneHmd, zeroOffset, coneThresholds);
        ok &= expectTrue("generated sight rear-center produces a valid native cone sample", centeredConeSample.valid);
        ok &= expectNear("native cone uses HMD-to-sight distance", centeredConeSample.distanceGameUnits, 30.0f);
        ok &= expectNear("native cone uses HMD +Y", centeredConeSample.hmdAngleDegrees, 0.0f);
        ok &= expectNear("native cone uses weapon +Y", centeredConeSample.weaponAngleDegrees, 0.0f);
        ok &= expectNear("native cone preserves shipped widening formula", centeredConeSample.weaponAngleWidening, 4.0f);
        ok &= expectTrue("centered generated sight enters native scope", rock::native_scope_activation_geometry::isInsideCone(centeredConeSample, false, coneThresholds));

        const auto hysteresisDistanceSample = rock::native_scope_activation_geometry::sample(coneWeapon, TestVector3{ 0.0f, 39.0f, 0.0f }, coneHmd, zeroOffset, coneThresholds);
        ok &= expectFalse("39 game units stays outside the strict enter distance",
            rock::native_scope_activation_geometry::isInsideCone(hysteresisDistanceSample, false, coneThresholds));
        ok &=
            expectTrue("39 game units stays inside the wider exit distance", rock::native_scope_activation_geometry::isInsideCone(hysteresisDistanceSample, true, coneThresholds));
        auto strictBoundarySample = centeredConeSample;
        strictBoundarySample.distanceGameUnits = coneThresholds.distanceEnterGameUnits;
        ok &= expectFalse("native enter boundary remains a strict comparison", rock::native_scope_activation_geometry::isInsideCone(strictBoundarySample, false, coneThresholds));

        constexpr std::uint32_t exitConfirmationFrames = 3;
        const auto immediateEntry = rock::native_scope_activation_geometry::stabilizeExitDecision(true, false, 0, exitConfirmationFrames);
        ok &= expectTrue("native scope entry remains immediate", immediateEntry.decision);
        ok &= expectTrue("entry clears pending outside samples", immediateEntry.consecutiveOutsideFrames == 0);
        const auto inactiveOutside = rock::native_scope_activation_geometry::stabilizeExitDecision(false, false, 2, exitConfirmationFrames);
        ok &= expectFalse("outside sample does not activate an inactive scope", inactiveOutside.decision);
        ok &= expectTrue("inactive scope clears pending exit state", inactiveOutside.consecutiveOutsideFrames == 0);
        const auto firstOutside = rock::native_scope_activation_geometry::stabilizeExitDecision(false, true, 0, exitConfirmationFrames);
        ok &= expectTrue("first active outside sample is held", firstOutside.decision);
        const auto recoveredInside = rock::native_scope_activation_geometry::stabilizeExitDecision(true, true, firstOutside.consecutiveOutsideFrames, exitConfirmationFrames);
        ok &= expectTrue("inside sample cancels a pending scope exit", recoveredInside.decision);
        ok &= expectTrue("inside recovery resets outside count", recoveredInside.consecutiveOutsideFrames == 0);
        const auto secondOutside = rock::native_scope_activation_geometry::stabilizeExitDecision(
            false, true, firstOutside.consecutiveOutsideFrames, exitConfirmationFrames);
        ok &= expectTrue("second active outside sample is held", secondOutside.decision);
        const auto confirmedExit = rock::native_scope_activation_geometry::stabilizeExitDecision(
            false, true, secondOutside.consecutiveOutsideFrames, exitConfirmationFrames);
        ok &= expectFalse("third consecutive active outside sample confirms scope exit", confirmedExit.decision);

        const TestTransform equippedSightBaseline =
            rock::native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(
                weaponBefore,
                rigidSightFrameLocal);
        const TestVector3 expectedSightAnchorWorld = rock::transform_math::localPointToWorld(weaponBefore, sightAnchor);
        ok &= expectNear("equipped scope baseline uses sight world position x", equippedSightBaseline.translate.x, expectedSightAnchorWorld.x);
        ok &= expectNear("equipped scope baseline uses sight world position y", equippedSightBaseline.translate.y, expectedSightAnchorWorld.y);
        ok &= expectNear("equipped scope baseline uses sight world position z", equippedSightBaseline.translate.z, expectedSightAnchorWorld.z);
        ok &= expectNear("equipped scope baseline preserves hFRIK camera scale", equippedSightBaseline.scale, scopeBefore.scale);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear("equipped scope baseline preserves hFRIK camera rotation", equippedSightBaseline.rotate.entry[row][column], scopeBefore.rotate.entry[row][column]);
            }
        }

        TestTransform nativeModelRootInCameraLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        nativeModelRootInCameraLocal.translate = { 6.0f, -30.0f, 2.0f };
        nativeModelRootInCameraLocal.rotate.entry[1][1] = 0.0f;
        nativeModelRootInCameraLocal.rotate.entry[1][2] = 1.0f;
        nativeModelRootInCameraLocal.rotate.entry[2][1] = -1.0f;
        nativeModelRootInCameraLocal.rotate.entry[2][2] = 0.0f;
        const TestTransform nativeScopeModelRootWorld = rock::transform_math::composeTransforms(
            scopeBefore,
            nativeModelRootInCameraLocal);
        const TestTransform modelRootCalibration =
            rock::native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(
                scopeBefore,
                nativeScopeModelRootWorld);
        ok &= expectNear("native scope overlay discards obsolete lateral calibration", modelRootCalibration.translate.x, 0.0f);
        ok &= expectNear("native scope overlay discards obsolete depth calibration", modelRootCalibration.translate.y, 0.0f);
        ok &= expectNear("native scope overlay discards obsolete vertical calibration", modelRootCalibration.translate.z, 0.0f);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(
                    "native scope overlay preserves Bethesda model orientation",
                    modelRootCalibration.rotate.entry[row][column],
                    nativeModelRootInCameraLocal.rotate.entry[row][column]);
            }
        }

        const TestTransform zeroFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        const TestTransform targetScopeModelRoot =
            rock::native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                anchoredScopeAfter,
                modelRootCalibration,
                zeroFineTune);
        const TestTransform expectedScopeModelRoot = rock::transform_math::composeTransforms(
            anchoredScopeAfter,
            modelRootCalibration);
        ok &= expectTransformNear(
            "native scope overlay applies Bethesda orientation at the generated sight",
            targetScopeModelRoot,
            expectedScopeModelRoot);
        ok &= expectNear("native scope overlay keeps generated sight lateral anchor", targetScopeModelRoot.translate.x, anchoredScopeAfter.translate.x);
        ok &= expectNear("native scope overlay keeps generated sight depth anchor", targetScopeModelRoot.translate.y, anchoredScopeAfter.translate.y);
        ok &= expectNear("native scope overlay keeps generated sight vertical anchor", targetScopeModelRoot.translate.z, anchoredScopeAfter.translate.z);

        TestTransform scopeModelRootLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        scopeModelRootLocal.translate = { 0.0f, -12.0f, 0.0f };
        const TestTransform correctedScopeParent =
            rock::native_scope_overlay_follow_math::resolveScopeParentWorldForModelRoot(
                targetScopeModelRoot,
                scopeModelRootLocal);
        const TestTransform correctedScopeModelRoot = rock::transform_math::composeTransforms(
            correctedScopeParent,
            scopeModelRootLocal);
        ok &= expectTransformNear(
            "native scope overlay compensates model-root depth after calibrated orientation",
            correctedScopeModelRoot,
            targetScopeModelRoot);

        const TestTransform fineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                1.0f, 2.0f, 3.0f, 90.0f, 0.0f, 0.0f);
        ok &= expectNear("native scope overlay INI lateral offset", fineTune.translate.x, 1.0f);
        ok &= expectNear("native scope overlay INI depth offset", fineTune.translate.y, 2.0f);
        ok &= expectNear("native scope overlay INI vertical offset", fineTune.translate.z, 3.0f);
        ok &= expectNear("native scope overlay INI pitch rotates local Y toward Z", fineTune.rotate.entry[1][2], 1.0f);
        ok &= expectNear("native scope overlay INI pitch rotates local Z toward negative Y", fineTune.rotate.entry[2][1], -1.0f);

        const TestTransform yawFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f);
        ok &= expectNear("native scope overlay INI yaw rotates local X toward Y", yawFineTune.rotate.entry[0][1], 1.0f);
        ok &= expectNear("native scope overlay INI yaw rotates local Y toward negative X", yawFineTune.rotate.entry[1][0], -1.0f);

        const TestTransform rollFineTune =
            rock::native_scope_overlay_follow_math::makeModelRootFineTuneLocal<TestTransform>(
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90.0f);
        ok &= expectNear("native scope overlay INI roll rotates local X toward negative Z", rollFineTune.rotate.entry[0][2], -1.0f);
        ok &= expectNear("native scope overlay INI roll rotates local Z toward X", rollFineTune.rotate.entry[2][0], 1.0f);
    }

    {
        TestTransform driverBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        driverBefore.translate = { 10.0f, -4.0f, 7.0f };
        TestTransform handBefore = rock::transform_math::makeIdentityTransform<TestTransform>();
        handBefore.translate = { 12.0f, -1.0f, 8.5f };

        const TestTransform driverToHand = rock::scope_safe_hand_frame_math::captureDriverToHandLocal(
            driverBefore,
            handBefore);

        TestTransform driverAfter = rock::transform_math::makeIdentityTransform<TestTransform>();
        driverAfter.translate = { -3.0f, 14.0f, 11.0f };
        driverAfter.rotate.entry[0][0] = 0.0f;
        driverAfter.rotate.entry[0][1] = 1.0f;
        driverAfter.rotate.entry[1][0] = -1.0f;
        driverAfter.rotate.entry[1][1] = 0.0f;

        const TestTransform handAfter = rock::scope_safe_hand_frame_math::resolveHandWorld(
            driverAfter,
            driverToHand);
        const TestTransform resolvedDriverToHand = rock::transform_math::composeTransforms(
            rock::transform_math::invertTransform(driverAfter),
            handAfter);
        ok &= expectTransformNear("scope-safe hand preserves hFRIK driver-local calibration", resolvedDriverToHand, driverToHand);
        using rock::scope_safe_hand_frame_math::ResolutionMode;
        ok &= expectEqual("visible body uses root-flattened hand authority",
            rock::scope_safe_hand_frame_math::resolveMode(false, true, true, true, 0, 3),
            ResolutionMode::RootFlattened);
        ok &= expectEqual("native scope ignores even finite root data and reconstructs from the hFRIK driver",
            rock::scope_safe_hand_frame_math::resolveMode(true, true, true, true, 0, 3),
            ResolutionMode::DriverReconstructed);
        ok &= expectEqual("native scope briefly freezes the last valid frame across a transient driver miss",
            rock::scope_safe_hand_frame_math::resolveMode(true, false, false, true, 0, 3),
            ResolutionMode::LastKnown);
        ok &= expectEqual("native scope stops freezing after the bounded driver-miss grace",
            rock::scope_safe_hand_frame_math::resolveMode(true, false, false, true, 3, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("native scope without calibration or history fails closed",
            rock::scope_safe_hand_frame_math::resolveMode(true, true, false, false, 0, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("ordinary aiming never substitutes the scope driver for a missing canonical hand frame",
            rock::scope_safe_hand_frame_math::resolveMode(false, false, true, true, 0, 3), ResolutionMode::Unavailable);
        ok &= expectTrue("open ScopeMenu selects driver-frame weapon authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, false, false));
        ok &= expectTrue("manual grip retains driver-frame authority across a transient ScopeMenu close",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, true));
        ok &= expectTrue("manual grip retains driver-frame authority across a later ScopeMenu reopen",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, true, true));
        ok &= expectFalse("driver-frame authority returns to the visible root only after manual ownership ends",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, false, true));
        ok &= expectFalse("ordinary unscoped aiming does not acquire driver-frame authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, false));
        ok &= expectTrue("locked hand IK publishes outside native scope", rock::scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(false));
        ok &= expectFalse("locked hand IK is suppressed while native scope hides the body", rock::scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(true));
        ok &= expectTrue("visible stable carry may refresh the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(false, false));
        ok &= expectFalse("ScopeMenu cannot overwrite the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(true, false));
        ok &= expectFalse("scope-exit hand rebase cannot overwrite the right firing canonical", rock::scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(false, true));
        ok &= expectTrue("scoped right firing grip reuses the matching pre-scope canonical",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, false, true, 0x1234u, 0x1234u));
        ok &= expectFalse("scoped grip rejects a canonical from a stale weapon generation",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, false, true, 0x1234u, 0x5678u));
        ok &= expectFalse("left firing grip keeps its established mirrored hold",
            rock::scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(true, true, true, 0x1234u, 0x1234u));

        using rock::scope_safe_hand_frame_math::DeferredClearAction;
        using rock::scope_safe_hand_frame_math::DesiredHandAuthorityInput;
        using rock::scope_safe_hand_frame_math::HandAuthorityRole;
        const auto primaryRole = rock::scope_safe_hand_frame_math::roleMask(HandAuthorityRole::PrimaryGrip);
        const auto supportRole = rock::scope_safe_hand_frame_math::roleMask(HandAuthorityRole::SupportGrip);

        const DesiredHandAuthorityInput rightPrimaryOnly{
            .firingHandIsLeft = false,
        };
        ok &= expectEqual("right-hand-only scope owns no ROCK wrist tag",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightPrimaryOnly, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("right-hand-only scope leaves the left wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightPrimaryOnly, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));

        const DesiredHandAuthorityInput leftPrimaryOnly{
            .firingHandIsLeft = true,
        };
        ok &= expectEqual("left-hand-only scope uses mirrored carry without a ROCK wrist tag",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftPrimaryOnly, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("left-hand-only scope leaves the right wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftPrimaryOnly, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));

        const DesiredHandAuthorityInput rightFiringTwoHand{
            .gripping = true,
            .primaryHandAuthorityEnabled = true,
            .firingHandIsLeft = false,
            .leftPartGripActive = true,
        };
        ok &= expectEqual("right-fire two-hand scope retains the right primary wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringTwoHand, false), primaryRole);
        ok &= expectEqual("right-fire two-hand scope retains the left support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringTwoHand, true), supportRole);

        const DesiredHandAuthorityInput rightFiringVisualSupport{
            .gripping = true,
            .primaryHandAuthorityEnabled = false,
            .firingHandIsLeft = false,
            .leftPartGripActive = true,
        };
        ok &= expectEqual("right-fire visual support keeps the native firing wrist unowned",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringVisualSupport, false),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("right-fire visual support retains only the left support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(rightFiringVisualSupport, true), supportRole);

        const DesiredHandAuthorityInput leftFiringTwoHand{
            .gripping = true,
            .primaryHandAuthorityEnabled = true,
            .firingHandIsLeft = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("left-fire two-hand scope retains the left primary wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringTwoHand, true), primaryRole);
        ok &= expectEqual("left-fire two-hand scope retains the right support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringTwoHand, false), supportRole);

        const DesiredHandAuthorityInput leftFiringVisualSupport{
            .gripping = true,
            .primaryHandAuthorityEnabled = false,
            .firingHandIsLeft = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("left-fire visual support keeps the mirrored firing wrist native",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringVisualSupport, true),
            static_cast<rock::scope_safe_hand_frame_math::HandAuthorityRoleMask>(0));
        ok &= expectEqual("left-fire visual support retains only the right support wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(leftFiringVisualSupport, false), supportRole);

        const DesiredHandAuthorityInput twoHandPartCarry{
            .leftPartGripActive = true,
            .rightPartGripActive = true,
        };
        ok &= expectEqual("part carry retains the left part-grip wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(twoHandPartCarry, true), supportRole);
        ok &= expectEqual("part carry retains the right part-grip wrist",
            rock::scope_safe_hand_frame_math::desiredRolesForHand(twoHandPartCarry, false), supportRole);

        ok &= expectEqual("scope exit retains a role still owned by the same hand",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, primaryRole, 0),
            DeferredClearAction::RetainLiveRole);
        ok &= expectEqual("scope exit waits before replacing primary with support on one hand",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, supportRole, 0),
            DeferredClearAction::WaitForReplacementPublication);
        ok &= expectEqual("scope exit clears the old primary after support publishes",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::PrimaryGrip, supportRole, supportRole),
            DeferredClearAction::ClearStaleRole);
        ok &= expectEqual("scope exit clears released authority when no ROCK role remains",
            rock::scope_safe_hand_frame_math::resolveDeferredClearAction(
                HandAuthorityRole::SupportGrip, 0, 0),
            DeferredClearAction::ClearStaleRole);

        TestTransform rebaseStart = rock::transform_math::makeIdentityTransform<TestTransform>();
        rebaseStart.translate = { 1.5f, -2.0f, 0.75f };
        rebaseStart.rotate.entry[0][0] = 0.0f;
        rebaseStart.rotate.entry[0][1] = 1.0f;
        rebaseStart.rotate.entry[1][0] = -1.0f;
        rebaseStart.rotate.entry[1][1] = 0.0f;
        const TestTransform rebaseIdentity = rock::transform_math::makeIdentityTransform<TestTransform>();
        ok &= expectTransformNear("scope-exit rebase starts at the prior ROCK hand frame",
            rock::scope_safe_hand_frame_math::interpolateRebaseTransform(rebaseStart, rebaseIdentity, 0.0f),
            rebaseStart);
        ok &= expectTransformNear("scope-exit rebase finishes at the restored hFRIK root frame",
            rock::scope_safe_hand_frame_math::interpolateRebaseTransform(rebaseStart, rebaseIdentity, 1.0f),
            rebaseIdentity);
        ok &= expectNear("scope-exit rebase timing clamps at completion",
            rock::scope_safe_hand_frame_math::rebaseAlpha(0.10f, 0.075f),
            1.0f);
    }

    using namespace rock::contact_pipeline_policy;

    const ContactEndpoint weapon{
        .bodyId = 100,
        .layer = 44,
        .kind = ContactEndpointKind::Weapon,
    };

    const auto leftWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 10, .layer = 43, .kind = ContactEndpointKind::LeftHand },
        weapon);
    ok &= expectEqual("left hand weapon contact routes as hand weapon", leftWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("left hand weapon contact drives support evidence", leftWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("left hand remains contact source", leftWeapon.source.kind, ContactEndpointKind::LeftHand);

    const auto rightWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 20, .layer = 43, .kind = ContactEndpointKind::RightHand },
        weapon);
    ok &= expectEqual("right hand weapon contact routes as hand weapon", rightWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("right hand weapon contact drives support evidence", rightWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("right hand remains contact source", rightWeapon.source.kind, ContactEndpointKind::RightHand);

    using rock::weapon_two_handed_grip_math::canProcessNormalGrabInput;
    using rock::weapon_two_handed_grip_math::resolveSupportReleaseManualAction;
    using rock::weapon_two_handed_grip_math::SupportReleaseOwnershipInput;
    using rock::weapon_two_handed_grip_math::SupportReleaseManualAction;
    ok &= expectFalse("support-hand normal grab is blocked while its part grip is active", canProcessNormalGrabInput(false, true, true, false));
    ok &= expectTrue("support-hand normal grab stays available without a part grip", canProcessNormalGrabInput(false, true, false, false));
    ok &= expectFalse("firing-hand normal grab is blocked while a weapon is equipped", canProcessNormalGrabInput(true, true, false, false));
    ok &= expectTrue("firing-hand normal grab is restored while detached and free", canProcessNormalGrabInput(true, true, false, true));
    ok &= expectTrue("firing-hand normal grab stays available without equipped weapon", canProcessNormalGrabInput(true, false, false, false));
    ok &= expectTrue("full two-handed support still owns weapon transform",
        rock::weapon_support_authority_policy::supportGripOwnsWeaponTransform(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("full two-handed support applies primary hand authority while active",
        rock::weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("support grip continues to apply offhand visual authority",
        rock::weapon_support_authority_policy::supportGripAppliesSupportHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("only a normal dynamic full-authority support grip uses synchronized acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            false,
            false));
    ok &= expectFalse("authored support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            true,
            false,
            false));
    ok &= expectFalse("provider support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            true,
            false));
    ok &= expectFalse("attach-only support bypasses synchronized dynamic acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver,
            false,
            false,
            true));
    ok &= expectFalse("visual-only support never gains synchronized weapon acquisition",
        rock::weapon_support_authority_policy::shouldUseDynamicSupportAcquisition(
            rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport,
            false,
            false,
            false));

    const rock::RockEquippedWeaponHandlingBaseline coreWeaponHandlingBaseline{
        .ambidextrousHandoffEnabled = true,
        .firingGripProximitySupportRadiusGameUnits = 7.0f,
        .firingGripPromotionRadiusGameUnits = 5.5f,
        .leftFiringAimYawDegrees = 1.5f,
        .leftFiringAimPitchDegrees = -2.5f,
        .leftFiringAimOffsetXGameUnits = 0.5f,
        .leftFiringAimOffsetYGameUnits = -0.75f,
        .leftFiringAimOffsetZGameUnits = 1.25f,
    };
    const auto coreWeaponHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        nullptr);
    ok &= expectFalse("base ROCK has no external equipped-weapon authority",
        coreWeaponHandling.externalAuthorityActive);
    ok &= expectNear("base ROCK owns the proximity support radius",
        coreWeaponHandling.firingGripProximitySupportRadiusGameUnits,
        7.0f);
    ok &= expectTrue("base ROCK enables configured firing-grip ownership",
        coreWeaponHandling.firingGripOwnershipEnabled);
    ok &= expectTrue("base ROCK enables configured ambidextrous handoff",
        coreWeaponHandling.ambidextrousHandoffEnabled);
    ok &= expectFalse("base ROCK ambidextrous mode does not enable realistic detach",
        coreWeaponHandling.primaryDetachEnabled);
    ok &= expectNear("base ROCK owns firing-grip promotion tuning",
        coreWeaponHandling.firingGripPromotionRadiusGameUnits,
        5.5f);
    ok &= expectNear("base ROCK owns left firing aim yaw",
        coreWeaponHandling.leftFiringAimYawDegrees,
        1.5f);
    ok &= expectNear("base ROCK owns left firing aim pitch",
        coreWeaponHandling.leftFiringAimPitchDegrees,
        -2.5f);
    ok &= expectNear("base ROCK owns left firing aim offset X",
        coreWeaponHandling.leftFiringAimOffsetXGameUnits,
        0.5f);
    ok &= expectNear("base ROCK owns left firing aim offset Y",
        coreWeaponHandling.leftFiringAimOffsetYGameUnits,
        -0.75f);
    ok &= expectNear("base ROCK owns left firing aim offset Z",
        coreWeaponHandling.leftFiringAimOffsetZGameUnits,
        1.25f);

    auto fixedOnlyBaseline = coreWeaponHandlingBaseline;
    fixedOnlyBaseline.ambidextrousHandoffEnabled = false;
    const auto fixedOnlyHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        nullptr);
    ok &= expectFalse("disabled ROCK ambidextrous mode does not claim firing ownership",
        fixedOnlyHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("disabled ROCK ambidextrous mode keeps handoff disabled",
        fixedOnlyHandling.ambidextrousHandoffEnabled);

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 externalWeaponHandling{};
    externalWeaponHandling.flags = static_cast<std::uint32_t>(
        rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
    externalWeaponHandling.firingGripProximitySupportRadiusGameUnits = 8.0f;
    auto externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an equipped-weapon request activates external authority",
        externalHandling.externalAuthorityActive);
    ok &= expectFalse("an active addon request may suppress ROCK ambidextrous handoff",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("external authority preserves ROCK's radius without an override",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        7.0f);
    externalWeaponHandling.flags |=
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport);
    externalWeaponHandling.firingGripPromotionRadiusGameUnits = 9.0f;
    externalWeaponHandling.leftFiringAimYawDegrees = -4.0f;
    externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an active addon request may enable handoff through the ROCK executor",
        externalHandling.ambidextrousHandoffEnabled);
    ok &= expectNear("an active owner supplies proximity tuning",
        externalHandling.firingGripProximitySupportRadiusGameUnits,
        8.0f);
    ok &= expectNear("an active owner supplies handoff promotion tuning",
        externalHandling.firingGripPromotionRadiusGameUnits,
        9.0f);
    ok &= expectNear("an active owner supplies left firing aim tuning",
        externalHandling.leftFiringAimYawDegrees,
        -4.0f);

    ok &= expectFalse("compatible addon activation does not tear down ROCK handoff",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            externalHandling,
            false));
    auto addonDetachHandling = externalHandling;
    addonDetachHandling.primaryDetachEnabled = true;
    ok &= expectTrue("removing addon detach capability reconciles manual weapon state",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            addonDetachHandling,
            coreWeaponHandling,
            false));
    ok &= expectTrue("an addon override that disables handoff reconciles the live switch",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            fixedOnlyHandling,
            false));
    ok &= expectFalse("gaining ROCK handoff capability does not require teardown",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            fixedOnlyHandling,
            coreWeaponHandling,
            false));
    ok &= expectTrue("changing the fixed firing hand always reconciles carry ownership",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            coreWeaponHandling,
            true));
    auto addonPipboyHandling = externalHandling;
    addonPipboyHandling.pipboyTriggerHandEquipEnabled = true;
    ok &= expectTrue("removing addon Pip-Boy hand assignment reconciles its carry",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            addonPipboyHandling,
            coreWeaponHandling,
            false));

    using rock::weapon_support_authority_policy::canApplyFiringGripProximityAuthority;
    using rock::weapon_support_authority_policy::canPromoteSupportGripToFiringGrip;
    using rock::weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode;
    using rock::weapon_support_authority_policy::WeaponSupportAuthorityMode;
    ok &= expectTrue("firing-grip proximity contract always applies to eligible equipped weapons",
        canApplyFiringGripProximityAuthority(false));
    ok &= expectFalse("firing-grip proximity never changes a provider-mandated grab mode",
        canApplyFiringGripProximityAuthority(true));
    ok &= expectEqual("any weapon grab near the firing grip stays visual-only",
        resolveFiringGripProximityAuthorityMode(5.5f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab at the firing-grip radius stays visual-only",
        resolveFiringGripProximityAuthorityMode(6.0f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab away from the firing grip takes full authority",
        resolveFiringGripProximityAuthorityMode(6.5f, 6.0f),
        WeaponSupportAuthorityMode::FullTwoHandedSolver);
    ok &= expectFalse("authored visual-only support never promotes into firing authority",
        canPromoteSupportGripToFiringGrip(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true));
    ok &= expectTrue("dynamic touch visual-only support retains explicit handoff",
        canPromoteSupportGripToFiringGrip(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            false));
    ok &= expectTrue("full-authority support keeps its existing promotion contract",
        canPromoteSupportGripToFiringGrip(
            WeaponSupportAuthorityMode::FullTwoHandedSolver,
            true));

    using rock::weapon_interaction_probe_math::isBetterProbeCandidate;
    using rock::weapon_interaction_probe_math::ProbeCandidateRank;
    ok &= expectTrue("closer weapon part wins outside dual containment",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 }));
    ok &= expectFalse("farther weapon part loses outside dual containment",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("contained tiny part beats the engulfing receiver hull",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("containment tolerance lets a near-miss tiny part beat the engulfing hull",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.81f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("semantic priority breaks equal-size containment ties",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectFalse("equal candidates keep the current best",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectEqual("support release keeps realistic primary ownership while its grip is held",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{
            .firingGripOwnershipEnabled = true,
            .primaryDetachEnabled = true,
            .primaryGripHeld = true,
        }),
        SupportReleaseManualAction::KeepPrimaryOwnership);
    ok &= expectEqual("support release drops realistically detached weapon when primary grip is open",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{
            .firingGripOwnershipEnabled = true,
            .primaryDetachEnabled = true,
            .primaryGripHeld = false,
        }),
        SupportReleaseManualAction::DropEquippedWeapon);
    ok &= expectEqual("support release preserves ambidextrous firing ownership without realistic detach",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{
            .firingGripOwnershipEnabled = true,
            .primaryDetachEnabled = false,
            .primaryGripHeld = false,
        }),
        SupportReleaseManualAction::KeepPrimaryOwnership);
    ok &= expectEqual("support release ends support when firing-grip ownership is disabled",
        resolveSupportReleaseManualAction(SupportReleaseOwnershipInput{}),
        SupportReleaseManualAction::EndSupportOnly);

    using rock::weapon_two_handed_grip_math::canStartFreeHandPartGrip;
    using rock::weapon_two_handed_grip_math::canAttemptFiringGripReattach;
    using rock::weapon_two_handed_grip_math::FiringGripReattachInput;
    using rock::weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab;
    ok &= expectTrue("firing grip reattach is eligible during part carry",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
        }));
    ok &= expectFalse("firing grip reattach requires part-carry state",
        canAttemptFiringGripReattach(FiringGripReattachInput{}));
    ok &= expectFalse("firing grip reattach is blocked while a menu owns input",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .menuInputActive = true,
        }));
    ok &= expectFalse("firing grip reattach is blocked while the hand holds an object",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .handHoldingObject = true,
        }));

    ok &= expectTrue("held grab with the palm on the grip re-takes the firing grip",
        shouldReattachFiringGripOnGrab(true, 2.9f, 3.0f));
    ok &= expectFalse("grab reattach requires the palm inside the radius",
        shouldReattachFiringGripOnGrab(true, 3.5f, 3.0f));
    ok &= expectFalse("an open hand never re-takes the firing grip",
        shouldReattachFiringGripOnGrab(false, 0.1f, 3.0f));

    using rock::weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate;
    ok &= expectTrue("open palm inside the reattach radius is a hover candidate",
        isFiringGripReattachHoverCandidate(false, 2.9f, 3.0f));
    ok &= expectFalse("hover candidate requires the palm inside the radius",
        isFiringGripReattachHoverCandidate(false, 3.5f, 3.0f));
    ok &= expectFalse("a held grab is the reattach itself, never a hover",
        isFiringGripReattachHoverCandidate(true, 2.9f, 3.0f));

    ok &= expectTrue("free hand part grip starts on grab press over a routed support part",
        canStartFreeHandPartGrip(true, true, false, false));
    ok &= expectFalse("free hand part grip requires a routed support-grip contact",
        canStartFreeHandPartGrip(false, true, false, false));
    ok &= expectFalse("free hand part grip requires a grab press edge",
        canStartFreeHandPartGrip(true, false, false, false));
    ok &= expectFalse("free hand part grip is blocked while the hand holds an object",
        canStartFreeHandPartGrip(true, true, true, false));
    ok &= expectFalse("free hand part grip does not restart while already gripping",
        canStartFreeHandPartGrip(true, true, false, true));

    using namespace rock::equipped_weapon_manual_ownership_policy;
    ok &= expectTrue("addon detach mode enables firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .primaryDetachEnabled = true,
        }));
    ok &= expectTrue("addon handoff mode independently enables firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .ambidextrousHandoffAvailable = true,
        }));
    ok &= expectFalse("firing-grip ownership is disabled when both modes are off",
        firingGripOwnershipEnabled(FiringGripModeAvailability{}));
    ok &= expectTrue("left-hand trigger equip starts addon handoff ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .ambidextrousHandoffAvailable = true,
            },
            .handIsLeft = true,
            .gripHeld = true,
        }));
    ok &= expectFalse("right-hand trigger equip cannot start handoff-only ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .ambidextrousHandoffAvailable = true,
            },
            .gripHeld = true,
        }));
    ok &= expectTrue("addon detach mode can start right-hand ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .primaryDetachEnabled = true,
            },
            .gripHeld = true,
        }));
    ok &= expectFalse("trigger equip ownership requires the same hand grip",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .primaryDetachEnabled = true,
                .ambidextrousHandoffAvailable = true,
            },
            .handIsLeft = true,
        }));
    ok &= expectTrue("addon grip-zone flag enables settle equip",
        canSettleEquipInGripZone(true));
    ok &= expectFalse("grip-zone settle equip stays off without addon authority",
        canSettleEquipInGripZone(false));
    ok &= expectTrue("non-detaching ownership ignores an open firing grip",
        shouldRetainPrimaryOnlyOwnership(false, false));
    ok &= expectTrue("detaching ownership remains while the firing grip is held",
        shouldRetainPrimaryOnlyOwnership(true, true));
    ok &= expectFalse("detaching ownership releases when the firing grip opens",
        shouldRetainPrimaryOnlyOwnership(true, false));
    RuntimeState ambidextrousLifecycleState{
        .active = true,
        .ownershipKey = 0x21u,
    };
    const auto ambidextrousWeaponChanged = update(ambidextrousLifecycleState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 0x22u,
            .primaryGripRetained = shouldRetainPrimaryOnlyOwnership(false, false),
        });
    ok &= expectTrue("ambidextrous-only ownership still clears when equipped identity changes",
        ambidextrousWeaponChanged.cleared);
    ok &= expectFalse("ambidextrous identity cleanup never drops the newly equipped weapon",
        ambidextrousWeaponChanged.dropRequested);
    ok &= expectTrue("manual grip feature is available for an equipped instance", featureAvailable(true, true, true, 20));
    ok &= expectFalse("manual grip feature is unavailable without active weapon node", featureAvailable(true, true, false, 20));
    ok &= expectTrue("manual primary ownership is available while colliders build", featureAvailable(true, true, true, 20));
    ok &= expectFalse("manual grip feature requires an equipped instance witness", featureAvailable(true, true, true, 0));
    ok &= expectTrue("pending trigger-equip grip waits while runtime weapon is not ready",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectFalse("pending trigger-equip grip clears on release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectTrue("pending trigger-equip grip is retained for visual-only sidearm release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));

    ok &= expectTrue("same equipped weapon preserves ownership across collision rebuild",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0xBBu));
    ok &= expectFalse("different equipped weapon cannot inherit manual ownership",
        canPreserveManualOwnership(0xAAu, 0xCCu, 0xBBu));
    ok &= expectFalse("unpublished collision generation cannot preserve ownership",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0u));
    ok &= expectTrue("primary-only ownership survives unpublished collision generation",
        canPreserveManualOwnership(0xAAu, 0xAAu, 0u, false));
    ok &= expectFalse("provisional primary-only ownership still rejects a different equipped identity",
        canPreserveManualOwnership(0xAAu, 0xCCu, 0u, false));

    GripReleaseDebounceState primaryReleaseDebounce{};
    auto primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    ok &= expectTrue("one open primary sample retains firing grip", primaryReleaseDecision.retained);
    ok &= expectFalse("one open primary sample does not confirm release", primaryReleaseDecision.releaseConfirmed);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, true);
    ok &= expectTrue("held primary sample resets release debounce", primaryReleaseDecision.retained);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    primaryReleaseDecision = debouncePrimaryGripRelease(primaryReleaseDebounce, false);
    ok &= expectFalse("stable open primary samples release firing grip", primaryReleaseDecision.retained);
    ok &= expectTrue("stable open primary samples confirm release", primaryReleaseDecision.releaseConfirmed);

    ok &= expectTrue("release confirmed on a just-captured support grip is deferred",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(0));
    ok &= expectTrue("release confirmed on the earliest confirmable frame after a grab is deferred",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(kPrimaryReleaseConfirmFrames));
    ok &= expectTrue("release confirmed at the defer window edge is still deferred",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(kFreshSupportGripPrimaryReleaseDeferFrames));
    ok &= expectFalse("release confirmed on an aged support grip acts normally",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(kFreshSupportGripPrimaryReleaseDeferFrames + 1));

    RuntimeState manualState{};
    auto manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectFalse("native equip alone does not start manual ownership", manualDecision.active);
    ok &= expectFalse("native equip alone does not request drop", manualDecision.dropRequested);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = true,
            .primaryGripRetained = true,
            .supportGripRetained = false,
        });
    ok &= expectTrue("first retained grip starts manual ownership", manualDecision.started);
    ok &= expectTrue("manual ownership remains active while primary grip retained", manualDecision.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("manual ownership requests drop when all grips release", manualDecision.dropRequested);
    ok &= expectFalse("drop request clears manual ownership state", manualState.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 11,
            .startRequested = true,
            .primaryGripRetained = false,
            .supportGripRetained = true,
        });
    ok &= expectTrue("support grip can start manual ownership", manualDecision.started);
    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .ownershipKey = 12,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("equipped instance change clears manual ownership", manualDecision.cleared);
    ok &= expectFalse("equipped instance change does not drop the newly equipped weapon", manualDecision.dropRequested);

    using namespace rock::equipped_weapon_drop_policy;
    ok &= expectEqual("support release normally drops from left hand", sourceForSupportRelease(false), SourceHand::Left);
    ok &= expectEqual("same-frame primary release drops from right hand", sourceForSupportRelease(true), SourceHand::Right);
    ok &= expectTrue("equipped shoulder stash is available when realistic handling and its setting are enabled",
        equippedWeaponShoulderStashAvailable(true, true));
    ok &= expectFalse("realistic handling off fully disables equipped shoulder stash",
        equippedWeaponShoulderStashAvailable(false, true));
    ok &= expectFalse("equipped shoulder stash setting remains authoritative under realistic handling",
        equippedWeaponShoulderStashAvailable(true, false));
    ok &= expectFalse("equipped shoulder stash stays disabled when both gates are off",
        equippedWeaponShoulderStashAvailable(false, false));

    ok &= expectEqual("primary-only carry stashes from the firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, false),
        SourceHand::Right);
    ok &= expectEqual("primary-only carry follows a left firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, true),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the left grip stashes from the left hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, false, false),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the right grip stashes from the right hand",
        resolveEquippedWeaponStashCarryHand(false, true, false, true, false),
        SourceHand::Right);
    ok &= expectTrue("ordinary release without a stash commit routes to physical drop",
        shouldAttemptPhysicalDrop(false));
    ok &= expectFalse("selected stash never falls through to physical drop",
        shouldAttemptPhysicalDrop(true));
    ok &= expectTrue("successful physical drop commits collider retirement",
        physicalDropCommitted(PhysicalDropCommitInput{ .dropSucceeded = true }));
    ok &= expectTrue("unresolved dropped reference still commits collider retirement",
        physicalDropCommitted(PhysicalDropCommitInput{ .droppedReferenceUnavailable = true }));
    ok &= expectFalse("failed physical drop preserves equipped collision",
        physicalDropCommitted(PhysicalDropCommitInput{}));
    ok &= expectEqual("part carry with both grips has no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, true, false),
        SourceHand::None);
    ok &= expectEqual("inactive grip states have no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, false, false, false, false),
        SourceHand::None);

    {
        using namespace rock::weapon_part_grip_report_policy;
        ok &= expectTrue("active non-attach part grip counts as carry", partGripCountsAsCarry(true, false));
        ok &= expectFalse("attach-only part grip never counts as carry", partGripCountsAsCarry(true, true));
        ok &= expectFalse("inactive part grip never counts as carry", partGripCountsAsCarry(false, false));

        ok &= expectTrue("provider AttachOnly grab mode resolves attach-only",
            providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));
        ok &= expectFalse("provider full-authority grab mode is not attach-only",
            providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::FullTwoHandAuthority)));
        ok &= expectFalse("attach-only requires an active provider authority",
            providerGrabModeIsAttachOnly(false, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));

        ok &= expectEqual("firing hand in gripping state reports the firing grip",
            resolveHandGripKind(true, false, false, true, false, false, false),
            HandGripKind::FiringGrip);
        ok &= expectEqual("firing hand in primary-only state reports the firing grip",
            resolveHandGripKind(false, false, true, true, false, false, false),
            HandGripKind::FiringGrip);
        ok &= expectEqual("offhand full-authority support grip reports full authority",
            resolveHandGripKind(true, false, false, false, true, false, false),
            HandGripKind::SupportFullAuthority);
        ok &= expectEqual("offhand visual-only support grip reports visual-only",
            resolveHandGripKind(true, false, false, false, true, false, true),
            HandGripKind::SupportVisualOnly);
        ok &= expectEqual("attach-only grip reports attach-only in gripping state",
            resolveHandGripKind(true, false, false, false, true, true, true),
            HandGripKind::AttachOnly);
        ok &= expectEqual("carry part grip in part-carry reports part carry",
            resolveHandGripKind(false, true, false, false, true, false, false),
            HandGripKind::PartCarry);
        ok &= expectEqual("detached firing hand attach-only grip reports attach-only",
            resolveHandGripKind(false, true, false, true, true, true, false),
            HandGripKind::AttachOnly);
        ok &= expectEqual("detached firing hand carry grip reports part carry",
            resolveHandGripKind(false, true, false, true, true, false, false),
            HandGripKind::PartCarry);
        ok &= expectEqual("idle hand reports no grip",
            resolveHandGripKind(false, false, false, false, false, false, false),
            HandGripKind::None);
        ok &= expectEqual("firing hand without part grip in part-carry reports no grip",
            resolveHandGripKind(false, true, false, true, false, false, false),
            HandGripKind::None);
    }

    {
        using namespace rock::weapon_part_record_identity_policy;
        ok &= expectEqual("P-Mag resolves the magazine slot anchor",
            resolveStructureAnchor("P-Mag"), StructureAnchor::SlotMagazine);
        ok &= expectEqual("P-Stock resolves the rear-furniture slot anchor",
            resolveStructureAnchor("P-Stock"), StructureAnchor::SlotRearFurniture);
        ok &= expectEqual("P-Barrel resolves the barrel slot anchor",
            resolveStructureAnchor("P-Barrel"), StructureAnchor::SlotBarrel);
        ok &= expectEqual("P-Compensator resolves the muzzle slot anchor",
            resolveStructureAnchor("P-Compensator"), StructureAnchor::SlotMuzzle);
        ok &= expectEqual("P-Bipod resolves the dedicated bipod slot anchor",
            resolveStructureAnchor("P-Bipod"), StructureAnchor::SlotBipod);
        ok &= expectEqual("mod-prefixed bipod connect point resolves the bipod slot anchor",
            resolveStructureAnchor("P-SV98Bipod"), StructureAnchor::SlotBipod);
        ok &= expectEqual("WeaponBolt resolves the bolt rig anchor",
            resolveStructureAnchor("WeaponBolt"), StructureAnchor::RigBolt);
        ok &= expectEqual("WeaponMagazineChild3 resolves the magazine display rig anchor",
            resolveStructureAnchor("WeaponMagazineChild3"), StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("dedicated magazine slot outranks its internal display rig",
            chooseStructureAnchor(StructureAnchor::SlotMagazine, StructureAnchor::RigMagazineDisplay), StructureAnchor::SlotMagazine);
        ok &= expectEqual("bolt rig outranks the catch-all receiver slot",
            chooseStructureAnchor(StructureAnchor::SlotReceiver, StructureAnchor::RigBolt), StructureAnchor::RigBolt);
        ok &= expectEqual("unknown mod-added connect point resolves no anchor",
            resolveStructureAnchor("P-CustomThing"), StructureAnchor::None);
        ok &= expectEqual("plain mesh name resolves no anchor",
            resolveStructureAnchor("AK74M_Body"), StructureAnchor::None);
        ok &= expectTrue("standard barrel attach point recovers a P-Barrel node",
            canonicalConnectPointForAttachPoint(kAttachPointBarrel) == "P-Barrel");
        ok &= expectTrue("muzzle recovery reads parent metadata from the installed barrel",
            recoveryProviderAttachPointForAttachPoint(kAttachPointMuzzle) == kAttachPointBarrel);
        ok &= expectTrue("magazine recovery reads parent metadata from the installed receiver",
            recoveryProviderAttachPointForAttachPoint(kAttachPointMagazine) == kAttachPointReceiver);
        ok &= expectTrue("receiver recovery precedes dependent barrel recovery",
            recoveryDependencyRank(kAttachPointReceiver) < recoveryDependencyRank(kAttachPointBarrel));
        ok &= expectTrue("barrel recovery precedes dependent muzzle recovery",
            recoveryDependencyRank(kAttachPointBarrel) < recoveryDependencyRank(kAttachPointMuzzle));
        ok &= expectTrue("custom attach points do not synthesize guessed nodes",
            canonicalConnectPointForAttachPoint(0xFE123456).empty());

        const auto otherByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Other);
        const auto magFromSlot = applyStructureAnchor(otherByName, StructureAnchor::SlotMagazine);
        ok &= expectEqual("magazine slot classifies an unnamed part as magazine",
            magFromSlot.partKind, rock::WeaponPartKind::Magazine);
        ok &= expectEqual("magazine slot classification is slot-sourced",
            magFromSlot.classificationSource, rock::WeaponPartClassificationSource::SlotAnchor);
        ok &= expectEqual("magazine slot carries the vanilla attach-point form id",
            magFromSlot.attachPointFormId, kAttachPointMagazine);
        const auto cartridgeKeptInMagazineSlot = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Round), StructureAnchor::SlotMagazine);
        ok &= expectEqual("magazine slot preserves an explicitly named cartridge",
            cartridgeKeptInMagazineSlot.partKind, rock::WeaponPartKind::Round);
        const auto cosmeticBulletKeptInMagazineSlot = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::CosmeticAmmo), StructureAnchor::SlotMagazine);
        ok &= expectEqual("magazine slot preserves explicitly cosmetic bullet geometry",
            cosmeticBulletKeptInMagazineSlot.partKind, rock::WeaponPartKind::CosmeticAmmo);

        const auto receiverByWeakToken = rock::classifyWeaponPartKind(rock::WeaponPartKind::Receiver);
        const auto barrelOverride = applyStructureAnchor(receiverByWeakToken, StructureAnchor::SlotBarrel);
        ok &= expectEqual("barrel slot overrides a weak receiver name match",
            barrelOverride.partKind, rock::WeaponPartKind::Barrel);
        const auto muzzleOverride = applyStructureAnchor(receiverByWeakToken, StructureAnchor::SlotMuzzle);
        ok &= expectEqual("muzzle slot classifies its physical module separately from the barrel",
            muzzleOverride.partKind, rock::WeaponPartKind::MuzzleDevice);
        ok &= expectEqual("muzzle slot carries the vanilla attach-point form id",
            muzzleOverride.attachPointFormId, kAttachPointMuzzle);

        const auto slideByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Slide);
        const auto slideKept = applyStructureAnchor(slideByName, StructureAnchor::RigBolt);
        ok &= expectEqual("action-named part keeps its name under the bolt rig",
            slideKept.partKind, rock::WeaponPartKind::Slide);
        ok &= expectEqual("kept action name stays name-sourced",
            slideKept.classificationSource, rock::WeaponPartClassificationSource::NameToken);
        const auto pumpKept = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Pump), StructureAnchor::SlotHandguard);
        ok &= expectEqual("pump keeps its action role inside the handguard slot",
            pumpKept.partKind, rock::WeaponPartKind::Pump);

        const auto receiverFill = applyStructureAnchor(otherByName, StructureAnchor::SlotReceiver);
        ok &= expectEqual("receiver slot fills unclassified parts",
            receiverFill.partKind, rock::WeaponPartKind::Receiver);
        const auto stockKeptOverReceiver = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Stock), StructureAnchor::SlotReceiver);
        ok &= expectEqual("receiver slot never overrides a critical name match",
            stockKeptOverReceiver.partKind, rock::WeaponPartKind::Stock);

        const auto roundKept = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Round), StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("named ammo round keeps its reload role under the magazine rig",
            roundKept.partKind, rock::WeaponPartKind::Round);
        const auto followerFill = applyStructureAnchor(otherByName, StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("unnamed magazine-rig part fills as cosmetic ammo",
            followerFill.partKind, rock::WeaponPartKind::CosmeticAmmo);

        const auto noAnchor = applyStructureAnchor(otherByName, StructureAnchor::None);
        ok &= expectEqual("no anchor keeps the name classification",
            noAnchor.partKind, rock::WeaponPartKind::Other);
        ok &= expectEqual("no anchor keeps the name source",
            noAnchor.classificationSource, rock::WeaponPartClassificationSource::NameToken);

        ok &= expectEqual("authored suppressor name classifies as a muzzle device",
            rock::classifyWeaponPartName("AK_Suppressor_Mesh").partKind,
            rock::WeaponPartKind::MuzzleDevice);
        ok &= expectEqual("barrel remains distinct from its installed muzzle device",
            rock::classifyWeaponPartName("WeaponBarrel").partKind,
            rock::WeaponPartKind::Barrel);
        ok &= expectEqual("authored bipod name classifies without deployment inference",
            rock::classifyWeaponPartName("Rifle_BiPod").partKind,
            rock::WeaponPartKind::Bipod);
        ok &= expectEqual("bipod identity outranks incidental cylinder export token",
            rock::classifyWeaponPartName("bipod_Cylinder_009_Bipod").partKind,
            rock::WeaponPartKind::Bipod);
        const auto bipodOverActionName = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Cylinder), StructureAnchor::SlotBipod);
        ok &= expectEqual("dedicated bipod slot overrides incidental action-name classification",
            bipodOverActionName.partKind, rock::WeaponPartKind::Bipod);
    }

    {
        using namespace rock::weapon_accessory_part_kind_policy;

        auto sight = rock::classifyWeaponPartKind(rock::WeaponPartKind::Sight);
        sight.attachPointFormId = rock::weapon_part_record_identity_policy::kAttachPointSight;
        const auto unchangedSight = applyAttachmentEvidence(sight, {});
        ok &= expectEqual("reticle-only optic remains Sight",
            unchangedSight.partKind, rock::WeaponPartKind::Sight);
        ok &= expectEqual("unchanged sight retains its original classification source",
            unchangedSight.classificationSource, rock::WeaponPartClassificationSource::NameToken);

        const auto laser = applyAttachmentEvidence(sight, Evidence{ .laserEmitter = true });
        ok &= expectEqual("laser emitter refines the physical module to LaserSight",
            laser.partKind, rock::WeaponPartKind::LaserSight);
        ok &= expectEqual("laser module reports attachment-backed classification",
            laser.classificationSource, rock::WeaponPartClassificationSource::AttachmentEvidence);
        ok &= expectEqual("attachment refinement retains the owning slot FormID",
            laser.attachPointFormId, sight.attachPointFormId);

        const auto flashlight = applyAttachmentEvidence(sight, Evidence{ .flashlightEmitter = true });
        ok &= expectEqual("flashlight emitter refines the physical module to Flashlight",
            flashlight.partKind, rock::WeaponPartKind::Flashlight);

        const auto combo = applyAttachmentEvidence(sight, Evidence{ .laserEmitter = true, .flashlightEmitter = true });
        ok &= expectEqual("co-owned laser and flashlight emitters produce the combo kind",
            combo.partKind, rock::WeaponPartKind::LaserFlashlightCombo);

        const auto nativeScope = applyAttachmentEvidence(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Barrel),
            Evidence{ .nativeScopeOverlay = true, .laserEmitter = true, .flashlightEmitter = true });
        ok &= expectEqual("native overlay OMOD evidence is authoritative for Scope",
            nativeScope.partKind, rock::WeaponPartKind::Scope);

        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Other) == 22);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::LaserSight) == 23);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Flashlight) == 24);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::LaserFlashlightCombo) == 25);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Scope) == 26);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::MuzzleDevice) == 27);
        static_assert(static_cast<std::uint32_t>(rock::WeaponPartKind::Bipod) == 28);
    }

    using namespace rock::weapon_part_runtime;
    std::array<Target, 3> weaponPartTargets{};
    weaponPartTargets[0].active = true;
    weaponPartTargets[0].ownerToken = 10;
    weaponPartTargets[0].weaponGenerationKey = 0xABC;
    weaponPartTargets[0].flags = MatchBodyId;
    weaponPartTargets[0].grabMode = GrabMode::AttachOnly;
    weaponPartTargets[0].bodyId = 42;
    weaponPartTargets[0].priority = 1;
    weaponPartTargets[0].groupId = 7;
    weaponPartTargets[1].active = true;
    weaponPartTargets[1].ownerToken = 11;
    weaponPartTargets[1].weaponGenerationKey = 0xABC;
    weaponPartTargets[1].flags = MatchBodyId;
    weaponPartTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    weaponPartTargets[1].bodyId = 42;
    weaponPartTargets[1].priority = 2;
    weaponPartTargets[2].active = true;
    weaponPartTargets[2].ownerToken = 12;
    weaponPartTargets[2].weaponGenerationKey = 0xABC;
    weaponPartTargets[2].flags = MatchSourceName;
    weaponPartTargets[2].grabMode = GrabMode::AttachOnly;
    std::memcpy(weaponPartTargets[2].sourceName.data(), "BoltNode", 8);
    weaponPartTargets[2].priority = 3;

    const auto unmatchedWhitelist = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part whitelist becomes active for matching generation", unmatchedWhitelist.whitelistActive);
    ok &= expectFalse("weapon part whitelist fails closed for unregistered contact", unmatchedWhitelist.matched);

    const auto matchedBody = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part body target matches", matchedBody.matched);
    ok &= expectEqual("higher priority matching target selects full authority",
        matchedBody.grabMode,
        GrabMode::FullTwoHandAuthority);
    ok &= expectEqual("matching target owner is preserved",
        matchedBody.ownerToken,
        static_cast<std::uint64_t>(11));

    const auto matchedName = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "BoltNode",
        });
    ok &= expectTrue("weapon part source-name target matches", matchedName.matched);
    ok &= expectEqual("source-name target selects attach-only mode",
        matchedName.grabMode,
        GrabMode::AttachOnly);

    const auto otherGeneration = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xDEF,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("weapon part whitelist does not apply to other generation", otherGeneration.whitelistActive);

    std::array<Target, 1> strictWeaponPartTarget{};
    strictWeaponPartTarget[0].active = true;
    strictWeaponPartTarget[0].ownerToken = 20;
    strictWeaponPartTarget[0].weaponGenerationKey = 0xABC;
    strictWeaponPartTarget[0].flags = MatchBodyId | MatchSourceName | MatchPartKind;
    strictWeaponPartTarget[0].grabMode = GrabMode::AttachOnly;
    strictWeaponPartTarget[0].bodyId = 42;
    strictWeaponPartTarget[0].partKind = rock::WeaponPartKind::Bolt;
    std::memcpy(strictWeaponPartTarget[0].sourceName.data(), "BoltNode", 8);

    const auto strictMatched = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectTrue("weapon part target requires and accepts all requested match fields", strictMatched.matched);

    const auto strictWrongName = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "Receiver",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong source name", strictWrongName.matched);

    const auto strictWrongPart = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Receiver,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong semantic part", strictWrongPart.matched);

    // Non-exclusive whitelist targets grant grab modes without activating
    // whitelist gating for everything else.
    std::array<Target, 2> mixedExclusivityTargets{};
    mixedExclusivityTargets[0].active = true;
    mixedExclusivityTargets[0].ownerToken = 30;
    mixedExclusivityTargets[0].flags = MatchActionRole | NonExclusive;
    mixedExclusivityTargets[0].grabMode = GrabMode::AttachOnly;
    mixedExclusivityTargets[0].actionRole = rock::WeaponActionRole::Bolt;

    const auto nonExclusiveBolt = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .actionRole = rock::WeaponActionRole::Bolt,
        });
    ok &= expectTrue("non-exclusive bolt target matches bolt contact", nonExclusiveBolt.matched);
    ok &= expectEqual("non-exclusive bolt target grants attach-only", nonExclusiveBolt.grabMode, GrabMode::AttachOnly);
    ok &= expectFalse("non-exclusive target does not activate whitelist gating", nonExclusiveBolt.whitelistActive);

    const auto nonExclusiveMiss = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("non-bolt contact stays unmatched under non-exclusive target", nonExclusiveMiss.matched);
    ok &= expectFalse("non-bolt contact is not whitelist-gated by non-exclusive target", nonExclusiveMiss.whitelistActive);

    mixedExclusivityTargets[1].active = true;
    mixedExclusivityTargets[1].ownerToken = 31;
    mixedExclusivityTargets[1].flags = MatchBodyId;
    mixedExclusivityTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    mixedExclusivityTargets[1].bodyId = 77;
    const auto mixedUnmatched = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("exclusive target still activates whitelist gating alongside non-exclusive", mixedUnmatched.whitelistActive);
    ok &= expectFalse("mixed whitelist still fails closed for unmatched contact", mixedUnmatched.matched);

    std::array<Target, 1> semanticsOnlyTarget{};
    semanticsOnlyTarget[0].active = true;
    semanticsOnlyTarget[0].ownerToken = 32;
    semanticsOnlyTarget[0].flags = NonExclusive;
    semanticsOnlyTarget[0].grabMode = GrabMode::AttachOnly;
    const auto semanticsOnly = resolveTarget(semanticsOnlyTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
        });
    ok &= expectFalse("NonExclusive without a matcher is unusable", semanticsOnly.matched);
    ok &= expectFalse("NonExclusive without a matcher activates nothing", semanticsOnly.whitelistActive);

    {
        using namespace rock::weapon_interaction_acquisition_policy;
        State acquisitionState{};
        ok &= expectEqual("legacy-palm overlap publishes touch acquisition provenance",
            resolve(acquisitionState, true, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("first overlap-gap frame retains touch provenance",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("second overlap-gap frame retains touch provenance",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::PhysicalContact);
        ok &= expectEqual("probe provenance begins after the bounded touch lease",
            resolve(acquisitionState, false, true),
            rock::WeaponInteractionAcquisitionSource::ProximityProbe);
        ok &= expectEqual("provenance never manufactures a contact candidate",
            resolve(acquisitionState, false, false),
            rock::WeaponInteractionAcquisitionSource::None);
    }

    using namespace rock::hand_collision_suppression_math;
    SuppressionSet<2> postDropSuppression{};
    const auto postDropSuppressionResult = beginSuppression(postDropSuppression, 42, 0);
    ok &= expectTrue("post-drop suppression stores release body", postDropSuppressionResult.stored);
    DelayedRestoreState postDropRestore{};
    ok &= expectTrue("post-drop suppression uses grab release delay seconds", beginDelayedRestore(postDropRestore, postDropSuppression, 0.8f));
    ok &= expectFalse("post-drop suppression remains active before configured delay", advanceDelayedRestore(postDropRestore, postDropSuppression, 0.79f));
    ok &= expectTrue("post-drop suppression expires at configured delay", advanceDelayedRestore(postDropRestore, postDropSuppression, 0.01f));

    {
        using namespace rock::weapon_collision_geometry_math;
        ok &= expectFalse("small unscaled source-local hull stays below the build threshold",
            scaledHullDiagonalCanBuild(0.04f, 1.0f, 0.5f));
        ok &= expectTrue("authored node scale participates in source-local hull validation",
            scaledHullDiagonalCanBuild(0.04f, 78.0f, 0.5f));
        ok &= expectFalse("zero-scale source-local hull fails closed",
            scaledHullDiagonalCanBuild(4.0f, 0.0f, 0.5f));

        std::vector<HullSelectionInput> balancedInputs{
            { .center = { 0.0f, -100.0f, 0.0f }, .min = { -2.0f, -102.0f, -2.0f }, .max = { 2.0f, -98.0f, 2.0f }, .pointCount = 40, .coverageClass = 1, .priority = 62 },
            { .center = { 0.0f, 100.0f, 0.0f }, .min = { -2.0f, 98.0f, -2.0f }, .max = { 2.0f, 102.0f, 2.0f }, .pointCount = 40, .coverageClass = 1, .priority = 62 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 3, .priority = 56 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 3, .priority = 80 },
            { .center = { 0.0f, 0.0f, 0.0f }, .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f }, .pointCount = 20, .coverageClass = 7, .priority = 12, .cosmetic = true },
        };
        const auto balancedSelection = selectBalancedHullIndices(balancedInputs, 3);
        ok &= expectTrue("balanced hull selection keeps the higher-priority magazine shell",
            std::find(balancedSelection.begin(), balancedSelection.end(), 3) != balancedSelection.end());
        ok &= expectFalse("balanced hull selection drops the lower-priority same-class candidate first",
            std::find(balancedSelection.begin(), balancedSelection.end(), 2) != balancedSelection.end());
        ok &= expectFalse("balanced hull selection does not spend structural capacity on cosmetic ammunition",
            std::find(balancedSelection.begin(), balancedSelection.end(), 4) != balancedSelection.end());
    }

    return ok ? 0 : 1;
}
