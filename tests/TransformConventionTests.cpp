#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

#include "physics-interaction/DebugAxisMath.h"
#include "physics-interaction/DebugPivotMath.h"
#include "physics-interaction/DebugOverlayPolicy.h"
#include "physics-interaction/CollisionLayerPolicy.h"
#include "physics-interaction/GrabAnchorMath.h"
#include "physics-interaction/GrabFingerPoseMath.h"
#include "physics-interaction/HandVisualLerpMath.h"
#include "physics-interaction/HeldObjectDampingMath.h"
#include "physics-interaction/HeldObjectPhysicsMath.h"
#include "physics-interaction/HandCollisionSuppressionMath.h"
#include "physics-interaction/HandspaceConvention.h"
#include "physics-interaction/NativeMeleeSuppressionPolicy.h"
#include "physics-interaction/PointingDirectionMath.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/WeaponCollisionAdjustmentMath.h"
#include "physics-interaction/WeaponCollisionGeometryMath.h"
#include "physics-interaction/WeaponCollisionLimits.h"
#include "physics-interaction/WeaponInteractionProbeMath.h"
#include "physics-interaction/WeaponInteractionRouter.h"
#include "physics-interaction/WeaponPartClassifier.h"
#include "physics-interaction/WeaponReloadStageObserver.h"
#include "physics-interaction/WeaponSemanticHullBudget.h"
#include "physics-interaction/WeaponTwoHandedSolver.h"

struct TestMatrix
{
    float entry[3][4]{};
};

struct TestVector
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct TestTransform
{
    TestMatrix rotate{};
    TestVector translate{};
    float scale = 1.0f;
};

namespace
{
    constexpr float kEpsilon = 0.0001f;

    bool nearlyEqual(float a, float b)
    {
        return std::fabs(a - b) <= kEpsilon;
    }

    TestMatrix makeNonSymmetricRotation()
    {
        TestMatrix matrix{};
        matrix.entry[0][0] = 0.0f;
        matrix.entry[0][1] = -1.0f;
        matrix.entry[0][2] = 0.0f;
        matrix.entry[1][0] = 1.0f;
        matrix.entry[1][1] = 0.0f;
        matrix.entry[1][2] = 0.0f;
        matrix.entry[2][0] = 0.0f;
        matrix.entry[2][1] = 0.0f;
        matrix.entry[2][2] = 1.0f;
        return matrix;
    }

    bool expectFloat(const char* label, float actual, float expected)
    {
        if (nearlyEqual(actual, expected)) {
            return true;
        }

        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectContainsIndex(const char* label, const std::vector<std::size_t>& values, std::size_t expected)
    {
        if (std::find(values.begin(), values.end(), expected) != values.end()) {
            return true;
        }

        std::printf("%s expected selected index %zu\n", label, expected);
        return false;
    }
}

int main()
{
    const TestMatrix niRotation = makeNonSymmetricRotation();
    const TestMatrix hkRotation = frik::rock::transform_math::niRowsToHavokColumns(niRotation);
    const auto* hkFloats = reinterpret_cast<const float*>(&hkRotation);

    bool ok = true;
    ok &= expectFloat("hk[0]", hkFloats[0], niRotation.entry[0][0]);
    ok &= expectFloat("hk[1]", hkFloats[1], niRotation.entry[1][0]);
    ok &= expectFloat("hk[2]", hkFloats[2], niRotation.entry[2][0]);
    ok &= expectFloat("hk[4]", hkFloats[4], niRotation.entry[0][1]);
    ok &= expectFloat("hk[5]", hkFloats[5], niRotation.entry[1][1]);
    ok &= expectFloat("hk[6]", hkFloats[6], niRotation.entry[2][1]);
    ok &= expectFloat("hk[8]", hkFloats[8], niRotation.entry[0][2]);
    ok &= expectFloat("hk[9]", hkFloats[9], niRotation.entry[1][2]);
    ok &= expectFloat("hk[10]", hkFloats[10], niRotation.entry[2][2]);

    const TestMatrix roundTrip = frik::rock::transform_math::havokColumnsToNiRows<TestMatrix>(hkFloats);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            char label[32];
            std::snprintf(label, sizeof(label), "roundTrip[%d][%d]", row, col);
            ok &= expectFloat(label, roundTrip.entry[row][col], niRotation.entry[row][col]);
        }
    }

    TestTransform rootTransform{};
    rootTransform.rotate = niRotation;
    rootTransform.translate = TestVector{ 10.0f, 20.0f, 30.0f };
    rootTransform.scale = 2.0f;

    const auto sharedWorldPoint = frik::rock::transform_math::localPointToWorld(rootTransform, TestVector{ 5.0f, 0.0f, 0.0f });
    ok &= expectFloat("sharedLocalToWorld.x", sharedWorldPoint.x, 10.0f);
    ok &= expectFloat("sharedLocalToWorld.y", sharedWorldPoint.y, 10.0f);
    ok &= expectFloat("sharedLocalToWorld.z", sharedWorldPoint.z, 30.0f);

    const auto sharedLocalRoundTrip = frik::rock::transform_math::worldPointToLocal(rootTransform, sharedWorldPoint);
    ok &= expectFloat("sharedWorldToLocal.x", sharedLocalRoundTrip.x, 5.0f);
    ok &= expectFloat("sharedWorldToLocal.y", sharedLocalRoundTrip.y, 0.0f);
    ok &= expectFloat("sharedWorldToLocal.z", sharedLocalRoundTrip.z, 0.0f);

    const auto sharedLocalDelta = frik::rock::transform_math::worldVectorToLocal(rootTransform, TestVector{ 0.0f, -10.0f, 0.0f });
    ok &= expectFloat("sharedWorldVectorToLocal.x", sharedLocalDelta.x, 5.0f);
    ok &= expectFloat("sharedWorldVectorToLocal.y", sharedLocalDelta.y, 0.0f);
    ok &= expectFloat("sharedWorldVectorToLocal.z", sharedLocalDelta.z, 0.0f);

    TestTransform childTransform{};
    childTransform.translate = TestVector{ 5.0f, 1.0f, 0.0f };
    childTransform.scale = 3.0f;
    childTransform.rotate.entry[0][0] = 1.0f;
    childTransform.rotate.entry[1][2] = -1.0f;
    childTransform.rotate.entry[2][1] = 1.0f;

    const auto composedTransform = frik::rock::transform_math::composeTransforms(rootTransform, childTransform);
    ok &= expectFloat("sharedComposeTranslate.x", composedTransform.translate.x, 12.0f);
    ok &= expectFloat("sharedComposeTranslate.y", composedTransform.translate.y, 10.0f);
    ok &= expectFloat("sharedComposeTranslate.z", composedTransform.translate.z, 30.0f);
    ok &= expectFloat("sharedComposeScale", composedTransform.scale, 6.0f);
    ok &= expectFloat("sharedComposeRot[0][0]", composedTransform.rotate.entry[0][0], 0.0f);
    ok &= expectFloat("sharedComposeRot[0][1]", composedTransform.rotate.entry[0][1], -1.0f);
    ok &= expectFloat("sharedComposeRot[0][2]", composedTransform.rotate.entry[0][2], 0.0f);
    ok &= expectFloat("sharedComposeRot[1][0]", composedTransform.rotate.entry[1][0], 0.0f);
    ok &= expectFloat("sharedComposeRot[1][1]", composedTransform.rotate.entry[1][1], 0.0f);
    ok &= expectFloat("sharedComposeRot[1][2]", composedTransform.rotate.entry[1][2], -1.0f);
    ok &= expectFloat("sharedComposeRot[2][0]", composedTransform.rotate.entry[2][0], 1.0f);
    ok &= expectFloat("sharedComposeRot[2][1]", composedTransform.rotate.entry[2][1], 0.0f);
    ok &= expectFloat("sharedComposeRot[2][2]", composedTransform.rotate.entry[2][2], 0.0f);

    const auto inverseRootTransform = frik::rock::transform_math::invertTransform(rootTransform);
    const auto inverseRoundTrip = frik::rock::transform_math::localPointToWorld(inverseRootTransform, sharedWorldPoint);
    ok &= expectFloat("sharedInverseRoundTrip.x", inverseRoundTrip.x, 5.0f);
    ok &= expectFloat("sharedInverseRoundTrip.y", inverseRoundTrip.y, 0.0f);
    ok &= expectFloat("sharedInverseRoundTrip.z", inverseRoundTrip.z, 0.0f);

    float quat[4]{};
    frik::rock::transform_math::niRowsToHavokQuaternion(niRotation, quat);
    ok &= expectFloat("quat.x", quat[0], 0.0f);
    ok &= expectFloat("quat.y", quat[1], 0.0f);
    ok &= expectFloat("quat.z", quat[2], 0.7071067f);
    ok &= expectFloat("quat.w", quat[3], 0.7071067f);

    const auto axisX = frik::rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 1.0f, 0.0f, 0.0f });
    const auto axisY = frik::rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 0.0f, 1.0f, 0.0f });
    const auto axisZ = frik::rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 0.0f, 0.0f, 1.0f });
    ok &= expectFloat("axisX.x", axisX.x, 0.0f);
    ok &= expectFloat("axisX.y", axisX.y, -1.0f);
    ok &= expectFloat("axisX.z", axisX.z, 0.0f);
    ok &= expectFloat("axisY.x", axisY.x, 1.0f);
    ok &= expectFloat("axisY.y", axisY.y, 0.0f);
    ok &= expectFloat("axisY.z", axisY.z, 0.0f);
    ok &= expectFloat("axisZ.x", axisZ.x, 0.0f);
    ok &= expectFloat("axisZ.y", axisZ.y, 0.0f);
    ok &= expectFloat("axisZ.z", axisZ.z, 1.0f);

    const auto mode1RightOffset = frik::rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.0f }, false, 1);
    const auto mode1LeftOffset = frik::rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.0f }, true, 1);
    ok &= expectFloat("mode1RightOffset.x", mode1RightOffset.x, 0.086f);
    ok &= expectFloat("mode1RightOffset.y", mode1RightOffset.y, 0.0f);
    ok &= expectFloat("mode1RightOffset.z", mode1RightOffset.z, 0.005f);
    ok &= expectFloat("mode1LeftOffset.x", mode1LeftOffset.x, 0.086f);
    ok &= expectFloat("mode1LeftOffset.y", mode1LeftOffset.y, 0.0f);
    ok &= expectFloat("mode1LeftOffset.z", mode1LeftOffset.z, 0.005f);

    const auto mode1AuthoredX = frik::rock::handspace_convention::authoredToRaw(TestVector{ 1.0f, 0.0f, 0.0f }, 1);
    const auto mode1AuthoredY = frik::rock::handspace_convention::authoredToRaw(TestVector{ 0.0f, 1.0f, 0.0f }, 1);
    const auto mode1AuthoredZ = frik::rock::handspace_convention::authoredToRaw(TestVector{ 0.0f, 0.0f, 1.0f }, 1);
    ok &= expectFloat("mode1AuthoredX.x", mode1AuthoredX.x, 1.0f);
    ok &= expectFloat("mode1AuthoredX.y", mode1AuthoredX.y, 0.0f);
    ok &= expectFloat("mode1AuthoredX.z", mode1AuthoredX.z, 0.0f);
    ok &= expectFloat("mode1AuthoredY.x", mode1AuthoredY.x, 0.0f);
    ok &= expectFloat("mode1AuthoredY.y", mode1AuthoredY.y, 0.0f);
    ok &= expectFloat("mode1AuthoredY.z", mode1AuthoredY.z, -1.0f);
    ok &= expectFloat("mode1AuthoredZ.x", mode1AuthoredZ.x, 0.0f);
    ok &= expectFloat("mode1AuthoredZ.y", mode1AuthoredZ.y, 1.0f);
    ok &= expectFloat("mode1AuthoredZ.z", mode1AuthoredZ.z, 0.0f);

    const auto mode0LeftOffset = frik::rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.0f }, true, 0);
    ok &= expectFloat("mode0LeftOffset.x", mode0LeftOffset.x, -0.086f);
    ok &= expectFloat("mode0LeftOffset.y", mode0LeftOffset.y, -0.005f);
    ok &= expectFloat("mode0LeftOffset.z", mode0LeftOffset.z, 0.0f);

    const auto mode2LeftOffset = frik::rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.25f }, true, 2);
    ok &= expectFloat("mode2LeftOffset.x", mode2LeftOffset.x, -0.25f);
    ok &= expectFloat("mode2LeftOffset.y", mode2LeftOffset.y, 0.086f);
    ok &= expectFloat("mode2LeftOffset.z", mode2LeftOffset.z, -0.005f);

    float bodyFloats[16]{};
    bodyFloats[0] = 0.0f;
    bodyFloats[1] = 1.0f;
    bodyFloats[2] = 0.0f;
    bodyFloats[4] = -1.0f;
    bodyFloats[5] = 0.0f;
    bodyFloats[6] = 0.0f;
    bodyFloats[8] = 0.0f;
    bodyFloats[9] = 0.0f;
    bodyFloats[10] = 1.0f;
    bodyFloats[12] = 1.0f;
    bodyFloats[13] = 2.0f;
    bodyFloats[14] = 3.0f;
    const float localPivot[4]{ 2.0f, 3.0f, 4.0f, 0.0f };
    const auto pivotWorld = frik::rock::debug_pivot_math::bodyLocalPointToWorldGamePoint<TestVector>(bodyFloats, localPivot, 70.0f);
    ok &= expectFloat("pivotWorld.x", pivotWorld.x, -140.0f);
    ok &= expectFloat("pivotWorld.y", pivotWorld.y, 280.0f);
    ok &= expectFloat("pivotWorld.z", pivotWorld.z, 490.0f);

    const TestVector palmBase{ 0.0f, -2.4f, 6.0f };
    const TestVector grabOffset{ 10.9f, 12.1f, 0.1f };
    const auto forwardGrabAnchor = frik::rock::grab_anchor_math::makeGrabAnchorHandspace(palmBase, grabOffset, false);
    ok &= expectFloat("forwardGrabAnchor.x", forwardGrabAnchor.x, 10.9f);
    ok &= expectFloat("forwardGrabAnchor.y", forwardGrabAnchor.y, 9.7f);
    ok &= expectFloat("forwardGrabAnchor.z", forwardGrabAnchor.z, 6.1f);

    const auto reversedGrabAnchor = frik::rock::grab_anchor_math::makeGrabAnchorHandspace(palmBase, grabOffset, true);
    ok &= expectFloat("reversedGrabAnchor.x", reversedGrabAnchor.x, -10.9f);
    ok &= expectFloat("reversedGrabAnchor.y", reversedGrabAnchor.y, -14.5f);
    ok &= expectFloat("reversedGrabAnchor.z", reversedGrabAnchor.z, 5.9f);

    const TestVector tunedGrabOffset{ 5.0f, 2.0f, -8.0f };
    const auto rightTunedGrabAnchor = frik::rock::grab_anchor_math::makeGrabAnchorHandspaceForHand(palmBase, tunedGrabOffset, false, false);
    const auto leftTunedGrabAnchor = frik::rock::grab_anchor_math::makeGrabAnchorHandspaceForHand(palmBase, tunedGrabOffset, false, true);
    ok &= expectFloat("rightTunedGrabAnchor.x", rightTunedGrabAnchor.x, 5.0f);
    ok &= expectFloat("rightTunedGrabAnchor.y", rightTunedGrabAnchor.y, -0.4f);
    ok &= expectFloat("rightTunedGrabAnchor.z", rightTunedGrabAnchor.z, -2.0f);
    ok &= expectFloat("leftTunedGrabAnchor.x", leftTunedGrabAnchor.x, 5.0f);
    ok &= expectFloat("leftTunedGrabAnchor.y", leftTunedGrabAnchor.y, -0.4f);
    ok &= expectFloat("leftTunedGrabAnchor.z", leftTunedGrabAnchor.z, -2.0f);

    const auto reversedLeftTunedGrabAnchor = frik::rock::grab_anchor_math::makeGrabAnchorHandspaceForHand(palmBase, tunedGrabOffset, true, true);
    ok &= expectFloat("reversedLeftTunedGrabAnchor.x", reversedLeftTunedGrabAnchor.x, -5.0f);
    ok &= expectFloat("reversedLeftTunedGrabAnchor.y", reversedLeftTunedGrabAnchor.y, -4.4f);
    ok &= expectFloat("reversedLeftTunedGrabAnchor.z", reversedLeftTunedGrabAnchor.z, 14.0f);

    const TestVector palmNormal{ 0.0f, 0.0f, 1.0f };
    const auto rightCloseNormal = frik::rock::handspace_convention::authoredToRawForHand(palmNormal, false, 1);
    const auto leftCloseNormal = frik::rock::handspace_convention::authoredToRawForHand(palmNormal, true, 1);
    ok &= expectFloat("rightCloseNormal.x", rightCloseNormal.x, 0.0f);
    ok &= expectFloat("rightCloseNormal.y", rightCloseNormal.y, 1.0f);
    ok &= expectFloat("rightCloseNormal.z", rightCloseNormal.z, 0.0f);
    ok &= expectFloat("leftCloseNormal.x", leftCloseNormal.x, 0.0f);
    ok &= expectFloat("leftCloseNormal.y", leftCloseNormal.y, 1.0f);
    ok &= expectFloat("leftCloseNormal.z", leftCloseNormal.z, 0.0f);

    const TestVector farDirection{ 0.25f, -0.5f, 1.0f };
    const auto normalFarDirection = frik::rock::pointing_direction_math::applyFarGrabNormalReversal(farDirection, false);
    ok &= expectFloat("normalFarDirection.x", normalFarDirection.x, 0.25f);
    ok &= expectFloat("normalFarDirection.y", normalFarDirection.y, -0.5f);
    ok &= expectFloat("normalFarDirection.z", normalFarDirection.z, 1.0f);

    const auto reversedFarDirection = frik::rock::pointing_direction_math::applyFarGrabNormalReversal(farDirection, true);
    ok &= expectFloat("reversedFarDirection.x", reversedFarDirection.x, -0.25f);
    ok &= expectFloat("reversedFarDirection.y", reversedFarDirection.y, 0.5f);
    ok &= expectFloat("reversedFarDirection.z", reversedFarDirection.z, -1.0f);

    TestMatrix identity{};
    identity.entry[0][0] = 1.0f;
    identity.entry[1][1] = 1.0f;
    identity.entry[2][2] = 1.0f;
    const TestVector weaponRotX90{ 90.0f, 0.0f, 0.0f };
    const auto disabledWeaponRotation = frik::rock::weapon_collision_adjustment_math::applyLocalEulerCorrection(identity, weaponRotX90, false);
    ok &= expectFloat("weaponDisabledRotation[1][1]", disabledWeaponRotation.entry[1][1], 1.0f);
    ok &= expectFloat("weaponDisabledRotation[2][2]", disabledWeaponRotation.entry[2][2], 1.0f);

    const auto correctedWeaponRotation = frik::rock::weapon_collision_adjustment_math::applyLocalEulerCorrection(identity, weaponRotX90, true);
    ok &= expectFloat("weaponCorrectedRotation[0][0]", correctedWeaponRotation.entry[0][0], 1.0f);
    ok &= expectFloat("weaponCorrectedRotation[1][1]", correctedWeaponRotation.entry[1][1], 0.0f);
    ok &= expectFloat("weaponCorrectedRotation[1][2]", correctedWeaponRotation.entry[1][2], -1.0f);
    ok &= expectFloat("weaponCorrectedRotation[2][1]", correctedWeaponRotation.entry[2][1], 1.0f);
    ok &= expectFloat("weaponCorrectedRotation[2][2]", correctedWeaponRotation.entry[2][2], 0.0f);

    const auto localCorrectedWeaponRotation = frik::rock::weapon_collision_adjustment_math::applyLocalEulerCorrection(niRotation, weaponRotX90, true);
    ok &= expectFloat("weaponLocalCorrectedRotation[0][0]", localCorrectedWeaponRotation.entry[0][0], 0.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[0][1]", localCorrectedWeaponRotation.entry[0][1], -1.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[0][2]", localCorrectedWeaponRotation.entry[0][2], 0.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[1][0]", localCorrectedWeaponRotation.entry[1][0], 0.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[1][1]", localCorrectedWeaponRotation.entry[1][1], 0.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[1][2]", localCorrectedWeaponRotation.entry[1][2], -1.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[2][0]", localCorrectedWeaponRotation.entry[2][0], 1.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[2][1]", localCorrectedWeaponRotation.entry[2][1], 0.0f);
    ok &= expectFloat("weaponLocalCorrectedRotation[2][2]", localCorrectedWeaponRotation.entry[2][2], 0.0f);

    TestMatrix rootRotation{};
    rootRotation.entry[0][1] = -1.0f;
    rootRotation.entry[1][0] = 1.0f;
    rootRotation.entry[2][2] = 1.0f;
    const TestVector weaponRoot{ 10.0f, 20.0f, 30.0f };
    const TestVector localBarrel{ 5.0f, 0.0f, 0.0f };
    const auto barrelWorld = frik::rock::weapon_collision_geometry_math::localPointToWorld(rootRotation, weaponRoot, 2.0f, localBarrel);
    ok &= expectFloat("weaponPackageWorld.x", barrelWorld.x, 10.0f);
    ok &= expectFloat("weaponPackageWorld.y", barrelWorld.y, 10.0f);
    ok &= expectFloat("weaponPackageWorld.z", barrelWorld.z, 30.0f);

    const auto barrelLocalRoundTrip = frik::rock::weapon_collision_geometry_math::worldPointToLocal(rootRotation, weaponRoot, 2.0f, barrelWorld);
    ok &= expectFloat("weaponPackageRoundTrip.x", barrelLocalRoundTrip.x, 5.0f);
    ok &= expectFloat("weaponPackageRoundTrip.y", barrelLocalRoundTrip.y, 0.0f);
    ok &= expectFloat("weaponPackageRoundTrip.z", barrelLocalRoundTrip.z, 0.0f);

    const auto generatedPackageRotation = frik::rock::weapon_collision_geometry_math::transposeRotation(rootRotation);
    ok &= expectFloat("weaponPackageRotation[0][1]", generatedPackageRotation.entry[0][1], 1.0f);
    ok &= expectFloat("weaponPackageRotation[1][0]", generatedPackageRotation.entry[1][0], -1.0f);
    ok &= expectFloat("weaponPackageRotation[2][2]", generatedPackageRotation.entry[2][2], 1.0f);

    const TestMatrix correctionZ90 = frik::rock::weapon_collision_adjustment_math::makeLocalEulerCorrection<TestMatrix>(TestVector{ 0.0f, 0.0f, 90.0f });
    const auto correctedLocalBarrel = frik::rock::weapon_collision_geometry_math::rotateLocalPoint(correctionZ90, localBarrel);
    ok &= expectFloat("weaponPackageCorrectedLocal.x", correctedLocalBarrel.x, 0.0f);
    ok &= expectFloat("weaponPackageCorrectedLocal.y", correctedLocalBarrel.y, -5.0f);
    ok &= expectFloat("weaponPackageCorrectedLocal.z", correctedLocalBarrel.z, 0.0f);

    std::vector<TestVector> linePoints;
    for (int i = 0; i < 300; ++i) {
        linePoints.push_back(TestVector{ static_cast<float>(i), 0.0f, 0.0f });
    }
    const auto reducedLine = frik::rock::weapon_collision_geometry_math::limitPointCloud(linePoints, 64);
    ok &= expectFloat("weaponReducedLineCount", static_cast<float>(reducedLine.size()), 64.0f);
    ok &= expectFloat("weaponReducedLineMin", reducedLine.front().x, 0.0f);
    ok &= expectFloat("weaponReducedLineMax", reducedLine.back().x, 299.0f);

    std::vector<frik::rock::weapon_collision_geometry_math::HullSelectionInput> generatedHullInputs{
        { { -1.0f, 0.0f, 0.0f }, { -2.0f, -1.0f, -1.0f }, { 0.0f, 1.0f, 1.0f }, 220, 2, 10, false },
        { { -1.0f, 8.0f, 0.0f }, { -2.0f, 7.0f, -1.0f }, { 0.0f, 9.0f, 1.0f }, 221, 2, 10, false },
        { { -1.0f, 16.0f, 0.0f }, { -2.0f, 15.0f, -1.0f }, { 0.0f, 17.0f, 1.0f }, 222, 2, 10, false },
        { { -1.0f, 24.0f, 0.0f }, { -2.0f, 23.0f, -1.0f }, { 0.0f, 25.0f, 1.0f }, 223, 2, 10, false },
        { { -1.0f, 32.0f, 0.0f }, { -2.0f, 31.0f, -1.0f }, { 0.0f, 33.0f, 1.0f }, 224, 2, 10, false },
        { { 0.0f, -24.0f, 0.0f }, { -3.0f, -27.0f, -2.0f }, { 3.0f, -21.0f, 2.0f }, 120, 0, 10, false },
        { { 0.0f, -6.0f, 0.0f }, { -4.0f, -8.0f, -2.0f }, { 4.0f, -4.0f, 2.0f }, 180, 1, 15, false },
        { { 0.0f, 4.0f, -4.0f }, { -2.0f, 2.0f, -7.0f }, { 2.0f, 6.0f, -1.0f }, 90, 3, 20, false },
        { { 0.0f, 3.0f, 4.0f }, { -2.0f, 1.0f, 3.0f }, { 2.0f, 5.0f, 5.0f }, 75, 4, 30, false },
        { { 0.0f, 2.0f, -3.0f }, { -1.0f, 1.0f, -4.0f }, { 1.0f, 3.0f, -2.0f }, 70, 8, 90, true },
    };
    const auto balancedHulls = frik::rock::weapon_collision_geometry_math::selectBalancedHullIndices(generatedHullInputs, 6);
    ok &= expectFloat("weaponBalancedHullCount", static_cast<float>(balancedHulls.size()), 6.0f);
    ok &= expectContainsIndex("weaponBalancedHullKeepsRearStock", balancedHulls, 5);
    ok &= expectContainsIndex("weaponBalancedHullKeepsFrontBarrel", balancedHulls, 4);
    ok &= expectContainsIndex("weaponBalancedHullKeepsReceiver", balancedHulls, 6);
    ok &= expectContainsIndex("weaponBalancedHullKeepsMagazine", balancedHulls, 7);
    ok &= expectContainsIndex("weaponBalancedHullKeepsTop", balancedHulls, 8);

    frik::rock::hand_collision_suppression_math::SuppressionState collisionState{};
    constexpr std::uint32_t activeFilter = 0x000B002B;
    constexpr std::uint32_t bodyId = 42;
    const auto suppressedFilter = frik::rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, activeFilter);
    ok &= expectFloat("suppression active", collisionState.active ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("suppression bodyId", static_cast<float>(collisionState.bodyId), static_cast<float>(bodyId));
    ok &= expectFloat("suppressed bit", (suppressedFilter & frik::rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 1.0f);

    const auto repeatedFilter = frik::rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, suppressedFilter);
    ok &= expectFloat("repeat keeps original disabled state", collisionState.wasNoCollideBeforeSuppression ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("repeat filter stable", static_cast<float>(repeatedFilter), static_cast<float>(suppressedFilter));

    const auto restoredFilter = frik::rock::hand_collision_suppression_math::restoreFilter(collisionState, repeatedFilter);
    ok &= expectFloat("restore clears bit", (restoredFilter & frik::rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("restore preserves layer/group", static_cast<float>(restoredFilter), static_cast<float>(activeFilter));

    frik::rock::hand_collision_suppression_math::clear(collisionState);
    constexpr std::uint32_t preDisabledFilter = activeFilter | frik::rock::hand_collision_suppression_math::kNoCollideBit;
    frik::rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, preDisabledFilter);
    ok &= expectFloat("pre-disabled recorded", collisionState.wasNoCollideBeforeSuppression ? 1.0f : 0.0f, 1.0f);
    const auto restoredPreDisabledFilter = frik::rock::hand_collision_suppression_math::restoreFilter(collisionState, preDisabledFilter);
    ok &= expectFloat("restore keeps pre-disabled bit", (restoredPreDisabledFilter & frik::rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 1.0f);

    ok &= expectFloat("debug hand draw enabled",
        frik::rock::debug_overlay_policy::shouldDrawHandBody(true, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug hand draw disabled",
        frik::rock::debug_overlay_policy::shouldDrawHandBody(true, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug weapon index inside cap",
        frik::rock::debug_overlay_policy::shouldDrawWeaponBody(true, true, 5, 6) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug weapon index outside cap",
        frik::rock::debug_overlay_policy::shouldDrawWeaponBody(true, true, 6, 6) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug shape generation clamp low", static_cast<float>(frik::rock::debug_overlay_policy::clampShapeGenerationsPerFrame(-2)), 0.0f);
    ok &= expectFloat("debug shape generation clamp high", static_cast<float>(frik::rock::debug_overlay_policy::clampShapeGenerationsPerFrame(99)), 32.0f);
    ok &= expectFloat("debug max convex vertex clamp low", static_cast<float>(frik::rock::debug_overlay_policy::clampMaxConvexSupportVertices(0)), 4.0f);
    ok &= expectFloat("debug max convex vertex clamp high", static_cast<float>(frik::rock::debug_overlay_policy::clampMaxConvexSupportVertices(999)), 256.0f);
    ok &= expectFloat("debug heavy convex uses bounds",
        frik::rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(192, 64, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug small convex keeps detail",
        frik::rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(32, 64, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug heavy convex bounds disabled",
        frik::rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(192, 64, false) ? 1.0f : 0.0f,
        0.0f);

    ok &= expectFloat("weapon collision body cap", static_cast<float>(frik::rock::MAX_WEAPON_COLLISION_BODIES), 64.0f);

    const auto magazinePart = frik::rock::classifyWeaponPartName("AssaultRifle:MagazineLarge");
    ok &= expectFloat("weapon classifier magazine kind", static_cast<float>(magazinePart.partKind), static_cast<float>(frik::rock::WeaponPartKind::Magazine));
    ok &= expectFloat("weapon classifier magazine reload role", static_cast<float>(magazinePart.reloadRole), static_cast<float>(frik::rock::WeaponReloadRole::MagazineBody));

    const auto magwellPart = frik::rock::classifyWeaponPartName("CombatRifle:MagwellSocket");
    ok &= expectFloat("weapon classifier magwell kind", static_cast<float>(magwellPart.partKind), static_cast<float>(frik::rock::WeaponPartKind::Magwell));
    ok &= expectFloat("weapon classifier magwell socket role", static_cast<float>(magwellPart.socketRole), static_cast<float>(frik::rock::WeaponSocketRole::Magwell));

    const auto handguardPart = frik::rock::classifyWeaponPartName("LaserRifle:Handguard");
    ok &= expectFloat("weapon classifier handguard support", static_cast<float>(handguardPart.supportGripRole), static_cast<float>(frik::rock::WeaponSupportGripRole::SupportSurface));
    ok &= expectFloat("weapon classifier handguard pose", static_cast<float>(handguardPart.fallbackGripPose), static_cast<float>(frik::rock::WeaponGripPoseId::HandguardClamp));

    std::vector<frik::rock::WeaponSemanticHullBudgetInput> semanticBudgetInputs;
    semanticBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Other), .sourceIndex = 0, .pointCount = 300 });
    semanticBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::CosmeticAmmo), .sourceIndex = 1, .pointCount = 900 });
    semanticBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Magazine), .sourceIndex = 2, .pointCount = 20 });
    semanticBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Magwell), .sourceIndex = 3, .pointCount = 25 });
    semanticBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Bolt), .sourceIndex = 4, .pointCount = 15 });
    const auto semanticBudgetIndices = frik::rock::selectSemanticWeaponHullIndices(semanticBudgetInputs, 3);
    ok &= expectFloat("weapon semantic budget count", static_cast<float>(semanticBudgetIndices.size()), 3.0f);
    ok &= expectFloat("weapon semantic budget keeps magwell first", static_cast<float>(semanticBudgetIndices[0]), 3.0f);
    ok &= expectFloat("weapon semantic budget keeps magazine second", static_cast<float>(semanticBudgetIndices[1]), 2.0f);
    ok &= expectFloat("weapon semantic budget keeps bolt third", static_cast<float>(semanticBudgetIndices[2]), 4.0f);

    std::vector<frik::rock::WeaponSemanticHullBudgetInput> duplicateSupportBudgetInputs;
    duplicateSupportBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Handguard), .sourceIndex = 10, .pointCount = 900 });
    duplicateSupportBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Handguard), .sourceIndex = 11, .pointCount = 800 });
    duplicateSupportBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Handguard), .sourceIndex = 12, .pointCount = 700 });
    duplicateSupportBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Magazine), .sourceIndex = 13, .pointCount = 20 });
    duplicateSupportBudgetInputs.push_back({ .semantic = frik::rock::classifyWeaponPartKind(frik::rock::WeaponPartKind::Bolt), .sourceIndex = 14, .pointCount = 15 });
    const auto duplicateSupportBudgetIndices = frik::rock::selectSemanticWeaponHullIndices(duplicateSupportBudgetInputs, 3);
    const auto hasBudgetIndex = [&duplicateSupportBudgetIndices](std::size_t index) {
        return std::find(duplicateSupportBudgetIndices.begin(), duplicateSupportBudgetIndices.end(), index) != duplicateSupportBudgetIndices.end();
    };
    ok &= expectFloat("weapon semantic budget keeps one handguard", (hasBudgetIndex(10) || hasBudgetIndex(11) || hasBudgetIndex(12)) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("weapon semantic budget keeps magazine despite lower priority duplicate", hasBudgetIndex(13) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("weapon semantic budget keeps bolt despite lower priority duplicate", hasBudgetIndex(14) ? 1.0f : 0.0f, 1.0f);

    frik::rock::WeaponInteractionContact handguardContact{};
    handguardContact.valid = true;
    handguardContact.bodyId = 200;
    handguardContact.partKind = handguardPart.partKind;
    handguardContact.supportGripRole = handguardPart.supportGripRole;
    handguardContact.reloadRole = handguardPart.reloadRole;
    handguardContact.socketRole = handguardPart.socketRole;
    handguardContact.actionRole = handguardPart.actionRole;
    const auto supportRoute = frik::rock::routeWeaponInteraction(handguardContact, frik::rock::WeaponReloadRuntimeState{});
    ok &= expectFloat("weapon router support grip", static_cast<float>(supportRoute.kind), static_cast<float>(frik::rock::WeaponInteractionKind::SupportGrip));

    frik::rock::WeaponInteractionContact magazineContact{};
    magazineContact.valid = true;
    magazineContact.bodyId = 201;
    magazineContact.partKind = magazinePart.partKind;
    magazineContact.supportGripRole = magazinePart.supportGripRole;
    magazineContact.reloadRole = magazinePart.reloadRole;
    magazineContact.socketRole = magazinePart.socketRole;
    magazineContact.actionRole = magazinePart.actionRole;
    frik::rock::WeaponReloadRuntimeState activeReload{};
    activeReload.state = frik::rock::WeaponReloadState::ReloadRequested;
    const auto reloadRoute = frik::rock::routeWeaponInteraction(magazineContact, activeReload);
    ok &= expectFloat("weapon router magazine reload", static_cast<float>(reloadRoute.kind), static_cast<float>(frik::rock::WeaponInteractionKind::RemoveMagazine));

    frik::rock::WeaponReloadObserverState reloadObserver{};
    const auto idleReloadStage =
        frik::rock::advanceWeaponReloadStageObserver(reloadObserver, frik::rock::WeaponReloadNativeSnapshot{ .weaponEquipped = true, .sequence = 1 });
    ok &= expectFloat("reload observer starts idle", static_cast<float>(idleReloadStage.stage), static_cast<float>(frik::rock::WeaponVanillaReloadStage::Idle));

    const auto reloadStartedStage = frik::rock::advanceWeaponReloadStageObserver(
        reloadObserver,
        frik::rock::WeaponReloadNativeSnapshot{ .weaponEquipped = true, .reloadEventReceived = true, .reloadEventValue = true, .sequence = 2 });
    ok &= expectFloat("reload observer native start", static_cast<float>(reloadStartedStage.stage),
        static_cast<float>(frik::rock::WeaponVanillaReloadStage::VanillaReloadStarted));
    ok &= expectFloat("reload observer native source", static_cast<float>(reloadStartedStage.source),
        static_cast<float>(frik::rock::WeaponReloadStageSource::NativeReloadEvent));

    const auto ammoCommittedStage = frik::rock::advanceWeaponReloadStageObserver(
        reloadObserver,
        frik::rock::WeaponReloadNativeSnapshot{ .weaponEquipped = true, .ammoEventReceived = true, .clipAmmo = 30, .reserveAmmo = 90, .sequence = 3 });
    ok &= expectFloat("reload observer ammo commit", static_cast<float>(ammoCommittedStage.stage),
        static_cast<float>(frik::rock::WeaponVanillaReloadStage::AmmoCommitted));

    frik::rock::WeaponReloadObserverState combinedReloadObserver{};
    const auto combinedReloadAmmoStage = frik::rock::advanceWeaponReloadStageObserver(
        combinedReloadObserver,
        frik::rock::WeaponReloadNativeSnapshot{ .weaponEquipped = true,
            .reloadEventReceived = true,
            .reloadEventValue = true,
            .ammoEventReceived = true,
            .clipAmmo = 12,
            .reserveAmmo = 48,
            .sequence = 30 });
    ok &= expectFloat("reload observer same frame ammo commit", static_cast<float>(combinedReloadAmmoStage.stage),
        static_cast<float>(frik::rock::WeaponVanillaReloadStage::AmmoCommitted));
    ok &= expectFloat("reload observer same frame ammo source", static_cast<float>(combinedReloadAmmoStage.source),
        static_cast<float>(frik::rock::WeaponReloadStageSource::NativeAmmoCountEvent));

    const auto reloadCompleteStage = frik::rock::advanceWeaponReloadStageObserver(
        reloadObserver,
        frik::rock::WeaponReloadNativeSnapshot{ .weaponEquipped = true, .reloadEventReceived = true, .reloadEventValue = false, .sequence = 4 });
    ok &= expectFloat("reload observer native complete", static_cast<float>(reloadCompleteStage.stage), static_cast<float>(frik::rock::WeaponVanillaReloadStage::Complete));

    frik::rock::WeaponReloadCoordinatorState reloadCoordinator{};
    const auto coordinatorStart = frik::rock::advanceWeaponReloadCoordinator(
        reloadCoordinator,
        frik::rock::WeaponReloadObserverOutput{ .stage = frik::rock::WeaponVanillaReloadStage::VanillaReloadStarted,
            .source = frik::rock::WeaponReloadStageSource::NativeReloadEvent,
            .sequence = 10 },
        false);
    ok &= expectFloat("reload coordinator request", static_cast<float>(coordinatorStart.runtime.state), static_cast<float>(frik::rock::WeaponReloadState::ReloadRequested));
    ok &= expectFloat("reload coordinator blocks support", coordinatorStart.runtime.supportGripAllowed ? 1.0f : 0.0f, 0.0f);

    const auto coordinatorAmmo = frik::rock::advanceWeaponReloadCoordinator(
        reloadCoordinator,
        frik::rock::WeaponReloadObserverOutput{ .stage = frik::rock::WeaponVanillaReloadStage::AmmoCommitted,
            .source = frik::rock::WeaponReloadStageSource::NativeAmmoCountEvent,
            .sequence = 11 },
        false);
    ok &= expectFloat("reload coordinator unloaded", static_cast<float>(coordinatorAmmo.runtime.state), static_cast<float>(frik::rock::WeaponReloadState::WeaponUnloaded));

    const auto coordinatorBlockedComplete = frik::rock::advanceWeaponReloadCoordinator(
        reloadCoordinator,
        frik::rock::WeaponReloadObserverOutput{ .stage = frik::rock::WeaponVanillaReloadStage::Complete,
            .source = frik::rock::WeaponReloadStageSource::NativeReloadEvent,
            .sequence = 12 },
        false,
        true);
    ok &= expectFloat("reload coordinator blocks vanilla complete without physical gate", static_cast<float>(coordinatorBlockedComplete.runtime.state),
        static_cast<float>(frik::rock::WeaponReloadState::WeaponUnloaded));
    ok &= expectFloat("reload coordinator reports physical gate", coordinatorBlockedComplete.vanillaCompletionNeedsPhysicalGate ? 1.0f : 0.0f, 1.0f);

    const auto coordinatorPhysical = frik::rock::advanceWeaponReloadCoordinator(
        reloadCoordinator,
        frik::rock::WeaponReloadObserverOutput{ .stage = frik::rock::WeaponVanillaReloadStage::AmmoCommitted,
            .source = frik::rock::WeaponReloadStageSource::NativeAmmoCountEvent,
            .sequence = 13 },
        true);
    ok &= expectFloat("reload coordinator inserted", static_cast<float>(coordinatorPhysical.runtime.state), static_cast<float>(frik::rock::WeaponReloadState::AmmoInserted));

    frik::rock::WeaponReloadCoordinatorState fallbackReloadCoordinator{};
    const auto coordinatorFallbackComplete = frik::rock::advanceWeaponReloadCoordinator(
        fallbackReloadCoordinator,
        frik::rock::WeaponReloadObserverOutput{ .stage = frik::rock::WeaponVanillaReloadStage::AmmoCommitted,
            .source = frik::rock::WeaponReloadStageSource::NativeAmmoCountEvent,
            .sequence = 14 },
        true,
        false);
    ok &= expectFloat("reload coordinator ammo commit fallback completes", static_cast<float>(coordinatorFallbackComplete.runtime.state),
        static_cast<float>(frik::rock::WeaponReloadState::Idle));
    ok &= expectFloat("reload coordinator ammo commit fallback allows support", coordinatorFallbackComplete.runtime.supportGripAllowed ? 1.0f : 0.0f, 1.0f);

    frik::rock::WeaponInteractionContact magwellReloadContact{};
    magwellReloadContact.valid = true;
    magwellReloadContact.bodyId = 202;
    magwellReloadContact.partKind = magwellPart.partKind;
    magwellReloadContact.supportGripRole = magwellPart.supportGripRole;
    magwellReloadContact.socketRole = magwellPart.socketRole;
    const auto socketRoute = frik::rock::routeWeaponInteraction(magwellReloadContact, coordinatorPhysical.runtime);
    ok &= expectFloat("weapon router reload socket wins over support", static_cast<float>(socketRoute.kind), static_cast<float>(frik::rock::WeaponInteractionKind::SocketInsert));

    ok &= expectFloat("reload physical gate falls back when unsupported",
        frik::rock::shouldRequireWeaponReloadPhysicalCompletion(true, true, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("reload physical gate remains strict when fallback disabled",
        frik::rock::shouldRequireWeaponReloadPhysicalCompletion(true, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("reload physical gate remains strict when supported",
        frik::rock::shouldRequireWeaponReloadPhysicalCompletion(true, true, true) ? 1.0f : 0.0f,
        1.0f);

    ok &= expectFloat("stale reload fallback inactive when strict physical gate",
        frik::rock::shouldFallbackCompleteStaleReload(true, true, true, 240, 180) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("stale reload fallback active when unsupported fallback times out",
        frik::rock::shouldFallbackCompleteStaleReload(true, false, true, 180, 180) ? 1.0f : 0.0f,
        1.0f);

    const float probeInside = frik::rock::weapon_interaction_probe_math::pointAabbDistanceSquared(
        TestVector{ 1.0f, 2.0f, 3.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 2.0f, 4.0f, 6.0f });
    ok &= expectFloat("weapon interaction probe point inside aabb", probeInside, 0.0f);
    const float probeOutside = frik::rock::weapon_interaction_probe_math::pointAabbDistanceSquared(
        TestVector{ 5.0f, 2.0f, 3.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 2.0f, 4.0f, 6.0f });
    ok &= expectFloat("weapon interaction probe point outside aabb", probeOutside, 9.0f);
    ok &= expectFloat("weapon interaction probe radius check",
        frik::rock::weapon_interaction_probe_math::isWithinProbeRadiusSquared(probeOutside, 3.0f) ? 1.0f : 0.0f,
        1.0f);

    TestTransform twoHandWeapon{};
    twoHandWeapon.rotate = identity;
    twoHandWeapon.scale = 1.0f;
    twoHandWeapon.translate = TestVector{ 3.0f, 4.0f, 5.0f };
    frik::rock::WeaponTwoHandedSolverInput<TestTransform, TestVector> solverInput{};
    solverInput.weaponWorldTransform = twoHandWeapon;
    solverInput.primaryGripLocal = TestVector{ 0.5f, 0.0f, 0.0f };
    solverInput.supportGripLocal = TestVector{ 0.5f, 2.0f, 0.0f };
    solverInput.primaryTargetWorld = TestVector{ 10.0f, 20.0f, 30.0f };
    solverInput.supportTargetWorld = TestVector{ 16.0f, 20.0f, 30.0f };

    const auto solvedTwoHand = frik::rock::solveTwoHandedWeaponTransform(solverInput);
    ok &= expectFloat("two handed solver solved", solvedTwoHand.solved ? 1.0f : 0.0f, 1.0f);
    const auto solvedPrimaryWorld = frik::rock::transform_math::localPointToWorld(solvedTwoHand.weaponWorldTransform, solverInput.primaryGripLocal);
    const auto solvedSupportWorld = frik::rock::transform_math::localPointToWorld(solvedTwoHand.weaponWorldTransform, solverInput.supportGripLocal);
    ok &= expectFloat("two handed solver primary x", solvedPrimaryWorld.x, 10.0f);
    ok &= expectFloat("two handed solver primary y", solvedPrimaryWorld.y, 20.0f);
    ok &= expectFloat("two handed solver primary z", solvedPrimaryWorld.z, 30.0f);

    TestTransform twoHandTwistWeapon{};
    twoHandTwistWeapon.rotate = identity;
    twoHandTwistWeapon.scale = 1.0f;
    twoHandTwistWeapon.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    frik::rock::WeaponTwoHandedSolverInput<TestTransform, TestVector> twistSolverInput{};
    twistSolverInput.weaponWorldTransform = twoHandTwistWeapon;
    twistSolverInput.primaryGripLocal = TestVector{ 0.0f, 0.0f, 0.0f };
    twistSolverInput.supportGripLocal = TestVector{ 2.0f, 0.0f, 0.0f };
    twistSolverInput.primaryTargetWorld = TestVector{ 0.0f, 0.0f, 0.0f };
    twistSolverInput.supportTargetWorld = TestVector{ 2.0f, 0.0f, 0.0f };
    twistSolverInput.supportNormalLocal = TestVector{ 0.0f, 0.0f, 1.0f };
    twistSolverInput.supportNormalTargetWorld = TestVector{ 0.0f, 1.0f, 0.0f };
    twistSolverInput.useSupportNormalTwist = true;
    twistSolverInput.supportNormalTwistFactor = 1.0f;
    const auto solvedTwist = frik::rock::solveTwoHandedWeaponTransform(twistSolverInput);
    const auto solvedTwistNormal = frik::rock::transform_math::localVectorToWorld(solvedTwist.weaponWorldTransform, twistSolverInput.supportNormalLocal);
    ok &= expectFloat("two handed solver twist solved", solvedTwist.solved ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("two handed solver twists support normal x", solvedTwistNormal.x, 0.0f);
    ok &= expectFloat("two handed solver twists support normal y", solvedTwistNormal.y, 1.0f);
    ok &= expectFloat("two handed solver twists support normal z", solvedTwistNormal.z, 0.0f);
    ok &= expectFloat("two handed solver support x", solvedSupportWorld.x, 12.0f);
    ok &= expectFloat("two handed solver support y", solvedSupportWorld.y, 20.0f);
    ok &= expectFloat("two handed solver support z", solvedSupportWorld.z, 30.0f);

    ok &= expectFloat("held velocity damping clamp low", frik::rock::held_object_damping_math::clampVelocityDamping(-2.0f), 0.0f);
    ok &= expectFloat("held velocity damping clamp high", frik::rock::held_object_damping_math::clampVelocityDamping(1.5f), 1.0f);
    ok &= expectFloat("held velocity damping keep factor", frik::rock::held_object_damping_math::velocityKeepFactor(0.25f), 0.75f);

    const auto dampedVelocity = frik::rock::held_object_damping_math::applyVelocityDamping(TestVector{ 10.0f, -20.0f, 5.0f }, 0.25f);
    ok &= expectFloat("held damped velocity x", dampedVelocity.x, 7.5f);
    ok &= expectFloat("held damped velocity y", dampedVelocity.y, -15.0f);
    ok &= expectFloat("held damped velocity z", dampedVelocity.z, 3.75f);

    const auto playerVelocity = frik::rock::held_object_physics_math::gameUnitsDeltaToHavokVelocity(TestVector{ 70.0f, -140.0f, 35.0f }, 0.5f);
    ok &= expectFloat("held player velocity x", playerVelocity.x, 2.0f);
    ok &= expectFloat("held player velocity y", playerVelocity.y, -4.0f);
    ok &= expectFloat("held player velocity z", playerVelocity.z, 1.0f);

    const auto residualDampedVelocity =
        frik::rock::held_object_physics_math::applyResidualVelocityDamping(TestVector{ 12.0f, 0.0f, 0.0f }, TestVector{ 8.0f, 0.0f, 0.0f }, 0.25f);
    ok &= expectFloat("held residual damping preserves player x", residualDampedVelocity.x, 11.0f);
    ok &= expectFloat("held residual damping preserves player y", residualDampedVelocity.y, 0.0f);
    ok &= expectFloat("held residual damping preserves player z", residualDampedVelocity.z, 0.0f);

    ok &= expectFloat("held player-space warp below threshold",
        frik::rock::held_object_physics_math::shouldWarpPlayerSpaceDelta(TestVector{ 35.0f, 0.0f, 0.0f }, 35.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held player-space warp above threshold",
        frik::rock::held_object_physics_math::shouldWarpPlayerSpaceDelta(TestVector{ 36.0f, 0.0f, 0.0f }, 35.0f) ? 1.0f : 0.0f,
        1.0f);

    const auto releaseVelocity =
        frik::rock::held_object_physics_math::composeReleaseVelocity(TestVector{ 3.0f, 0.0f, 0.0f }, TestVector{ 8.0f, 1.0f, 0.0f }, 1.5f);
    ok &= expectFloat("held release velocity x", releaseVelocity.x, 12.5f);
    ok &= expectFloat("held release velocity y", releaseVelocity.y, 1.0f);
    ok &= expectFloat("held release velocity z", releaseVelocity.z, 0.0f);

    std::array<TestVector, 5> releaseHistory{};
    releaseHistory[0] = TestVector{ 1.0f, 0.0f, 0.0f };
    releaseHistory[1] = TestVector{ 4.0f, 0.0f, 0.0f };
    releaseHistory[2] = TestVector{ 7.0f, 0.0f, 0.0f };
    releaseHistory[3] = TestVector{ 4.0f, 0.0f, 0.0f };
    releaseHistory[4] = TestVector{ 1.0f, 0.0f, 0.0f };
    const auto maxReleaseVelocity = frik::rock::held_object_physics_math::maxMagnitudeVelocity(releaseHistory, 5);
    ok &= expectFloat("held release max history x", maxReleaseVelocity.x, 5.0f);
    ok &= expectFloat("held release max history y", maxReleaseVelocity.y, 0.0f);
    ok &= expectFloat("held release max history z", maxReleaseVelocity.z, 0.0f);

    ok &= expectFloat("held lerp duration min",
        frik::rock::held_object_physics_math::computeHandLerpDuration(7.0f, 0.10f, 0.20f, 7.0f, 14.0f),
        0.10f);
    ok &= expectFloat("held lerp duration midpoint",
        frik::rock::held_object_physics_math::computeHandLerpDuration(10.5f, 0.10f, 0.20f, 7.0f, 14.0f),
        0.15f);
    ok &= expectFloat("held tau advance",
        frik::rock::held_object_physics_math::advanceToward(0.03f, 0.01f, 0.5f, 0.02f),
        0.02f);
    ok &= expectFloat("held angular force ratio",
        frik::rock::held_object_physics_math::angularForceFromRatio(2000.0f, 12.5f),
        160.0f);

    TestTransform currentHandVisual{};
    currentHandVisual.rotate = identity;
    currentHandVisual.scale = 1.0f;
    currentHandVisual.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    TestTransform targetHandVisual = currentHandVisual;
    targetHandVisual.translate = TestVector{ 10.0f, 0.0f, 0.0f };
    const auto advancedHandVisual = frik::rock::hand_visual_lerp_math::advanceTransform(currentHandVisual, targetHandVisual, 25.0f, 360.0f, 0.2f);
    ok &= expectFloat("hand visual lerp still advancing", advancedHandVisual.reachedTarget ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("hand visual lerp x", advancedHandVisual.transform.translate.x, 5.0f);

    const auto completedHandVisual =
        frik::rock::hand_visual_lerp_math::advanceTransform(advancedHandVisual.transform, targetHandVisual, 25.0f, 360.0f, 0.2f);
    ok &= expectFloat("hand visual lerp reached", completedHandVisual.reachedTarget ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand visual lerp target x", completedHandVisual.transform.translate.x, 10.0f);

    std::vector<frik::rock::grab_finger_pose_math::Triangle<TestVector>> fingerTriangles;
    fingerTriangles.push_back({ TestVector{ 4.0f, -1.0f, -1.0f }, TestVector{ 4.0f, 1.0f, -1.0f }, TestVector{ 4.0f, 0.0f, 1.0f } });
    const auto fingerValue = frik::rock::grab_finger_pose_math::solveFingerCurlValue(
        fingerTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        2.0f);
    ok &= expectFloat("finger curl triangle hit", fingerValue.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("finger curl half extended", fingerValue.value, 0.5f);

    std::vector<frik::rock::grab_finger_pose_math::Triangle<TestVector>> nearFingerTriangles;
    nearFingerTriangles.push_back({ TestVector{ 4.0f, 0.20f, -1.0f }, TestVector{ 4.0f, 0.20f, 1.0f }, TestVector{ 4.0f, 1.20f, 0.0f } });
    const auto nearFingerValue = frik::rock::grab_finger_pose_math::solveFingerCurlValue(
        nearFingerTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        0.25f);
    ok &= expectFloat("finger curl probe-radius near hit", nearFingerValue.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("finger curl probe-radius distance", nearFingerValue.distance, 4.0f);

    const auto missedFingerValue = frik::rock::grab_finger_pose_math::solveFingerCurlValue(
        fingerTriangles,
        TestVector{ 0.0f, 5.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        0.25f);
    ok &= expectFloat("finger curl miss closes", missedFingerValue.hit ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("finger curl miss min", missedFingerValue.value, 0.2f);

    std::uint64_t weaponLayerMatrix[48]{};
    for (std::uint32_t i = 0; i < 48; ++i) {
        weaponLayerMatrix[i] = ~0ULL;
    }
    frik::rock::collision_layer_policy::applyWeaponProjectileBlockingPolicy(
        weaponLayerMatrix, frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON, false, false);
    ok &= expectFloat("weapon projectile layer disabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("projectile reciprocal disabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::FO4_LAYER_PROJECTILE] & (1ULL << frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon cone projectile layer disabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_CONEPROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon spell layer disabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_SPELL)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon clutter/world layer unchanged",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << 1)) ? 1.0f : 0.0f,
        1.0f);

    frik::rock::collision_layer_policy::applyWeaponProjectileBlockingPolicy(
        weaponLayerMatrix, frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON, true, true);
    ok &= expectFloat("weapon projectile layer enabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon cone projectile layer enabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_CONEPROJECTILE)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon spell layer enabled",
        (weaponLayerMatrix[frik::rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << frik::rock::collision_layer_policy::FO4_LAYER_SPELL)) ? 1.0f : 0.0f,
        1.0f);

    using frik::rock::native_melee_suppression::NativeMeleeEvent;
    using frik::rock::native_melee_suppression::NativeMeleePolicyInput;
    using frik::rock::native_melee_suppression::NativeMeleeSuppressionAction;
    using frik::rock::native_melee_suppression::evaluateNativeMeleeSuppression;

    const NativeMeleePolicyInput npcSwingInput{ .rockEnabled = true,
        .suppressionEnabled = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = false,
        .physicalSwingActive = false };
    ok &= expectFloat("native melee npc swing calls native",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, npcSwingInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::CallNative));

    const NativeMeleePolicyInput playerSwingInput{ .rockEnabled = true,
        .suppressionEnabled = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = false };
    ok &= expectFloat("native melee player weapon swing suppressed",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, playerSwingInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::ReturnUnhandled));

    const NativeMeleePolicyInput playerHitFrameInput{ .rockEnabled = true,
        .suppressionEnabled = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = false };
    ok &= expectFloat("native melee player hitframe suppressed without physical swing",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, playerHitFrameInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::ReturnUnhandled));

    const NativeMeleePolicyInput physicalSwingHitFrameInput{ .rockEnabled = true,
        .suppressionEnabled = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = true };
    ok &= expectFloat("native melee player hitframe handled by physical swing",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, physicalSwingHitFrameInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::ReturnHandled));

    const NativeMeleePolicyInput disabledSuppressionInput{ .rockEnabled = true,
        .suppressionEnabled = false,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = false };
    ok &= expectFloat("native melee disabled policy calls native",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, disabledSuppressionInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::CallNative));
    ok &= expectFloat("native melee swing lease inactive with no expiry", frik::rock::native_melee_suppression::isPhysicalSwingLeaseActive(100, 0) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("native melee swing lease active before expiry", frik::rock::native_melee_suppression::isPhysicalSwingLeaseActive(100, 110) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("native melee swing lease inactive after expiry", frik::rock::native_melee_suppression::isPhysicalSwingLeaseActive(111, 110) ? 1.0f : 0.0f, 0.0f);

    return ok ? 0 : 1;
}
