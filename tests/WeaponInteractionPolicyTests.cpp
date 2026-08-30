#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
    namespace toggle_grab =
        rock::equipped_weapon_toggle_grab_policy;

    enum class TestWeaponType
    {
        kHandToHand = 0,
        kOneHandSword = 1,
        kOneHandDagger = 2,
        kOneHandAxe = 3,
        kOneHandMace = 4,
        kTwoHandSword = 5,
        kTwoHandAxe = 6,
        kBow = 7,
        kStaff = 8,
        kGun = 9,
        kGrenade = 10,
        kMine = 11,
    };

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

    bool expectVectorNear(
        const char* label,
        const TestVector3& actual,
        const TestVector3& expected,
        float tolerance = 0.0001f)
    {
        bool ok = true;
        ok &= expectNear(label, actual.x, expected.x, tolerance);
        ok &= expectNear(label, actual.y, expected.y, tolerance);
        ok &= expectNear(label, actual.z, expected.z, tolerance);
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
        TestTransform rightNativeWeaponInWand =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        rightNativeWeaponInWand.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.23f, 0.51f, 0.83f }),
            31.0f);
        rightNativeWeaponInWand.translate = { 8.0f, -3.0f, 4.0f };
        rightNativeWeaponInWand.scale = 2.0f;

        const TestTransform leftWeaponInWand =
            rock::left_firing_position_only_math::
                mirrorRightWeaponInWandOrientation(
                    rightNativeWeaponInWand);
        const TestVector3 weaponForward{ 0.0f, 1.0f, 0.0f };
        const TestTransform rightNativeWeaponOrientation =
            rock::left_firing_position_only_math::orientationOnly(
                rightNativeWeaponInWand);
        const TestVector3 rightBarrel =
            rock::transform_math::localVectorToWorld(
                rightNativeWeaponOrientation,
                weaponForward);
        const TestVector3 leftBarrel =
            rock::transform_math::localVectorToWorld(
                leftWeaponInWand,
                weaponForward);
        ok &= expectVectorNear(
            "left native weapon mirror negates only barrel lateral component",
            leftBarrel,
            TestVector3{ -rightBarrel.x, rightBarrel.y, rightBarrel.z });
        const TestVector3 rightLateral =
            rock::transform_math::localVectorToWorld(
                rightNativeWeaponOrientation,
                TestVector3{ 1.0f, 0.0f, 0.0f });
        const TestVector3 leftCorrespondingLateral =
            rock::transform_math::localVectorToWorld(
                leftWeaponInWand,
                TestVector3{ -1.0f, 0.0f, 0.0f });
        ok &= expectVectorNear(
            "left native weapon mirror maps right +X to left -X",
            leftCorrespondingLateral,
            TestVector3{
                -rightLateral.x,
                rightLateral.y,
                rightLateral.z });
        const TestVector3 rightUp =
            rock::transform_math::localVectorToWorld(
                rightNativeWeaponOrientation,
                TestVector3{ 0.0f, 0.0f, 1.0f });
        const TestVector3 leftUp =
            rock::transform_math::localVectorToWorld(
                leftWeaponInWand,
                TestVector3{ 0.0f, 0.0f, 1.0f });
        ok &= expectVectorNear(
            "left native weapon mirror maps right +Z to left +Z",
            leftUp,
            TestVector3{ -rightUp.x, rightUp.y, rightUp.z });
        ok &= expectVectorNear(
            "left native weapon orientation discards right-wand translation",
            leftWeaponInWand.translate,
            TestVector3{});
        ok &= expectNear(
            "left native weapon orientation is unit scale",
            leftWeaponInWand.scale,
            1.0f);
        const auto& mirroredRotation = leftWeaponInWand.rotate.entry;
        const float mirroredDeterminant =
            mirroredRotation[0][0] *
                (mirroredRotation[1][1] * mirroredRotation[2][2] -
                    mirroredRotation[1][2] * mirroredRotation[2][1]) -
            mirroredRotation[0][1] *
                (mirroredRotation[1][0] * mirroredRotation[2][2] -
                    mirroredRotation[1][2] * mirroredRotation[2][0]) +
            mirroredRotation[0][2] *
                (mirroredRotation[1][0] * mirroredRotation[2][1] -
                    mirroredRotation[1][1] * mirroredRotation[2][0]);
        ok &= expectNear(
            "bilateral weapon mirror remains a proper rotation",
            mirroredDeterminant,
            1.0f);

        TestTransform rawLeftWandWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        rawLeftWandWorld.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ -0.4f, 0.7f, 0.2f }),
            28.0f);
        rawLeftWandWorld.translate = { 15.0f, -6.0f, 11.0f };
        TestTransform referenceHandInWand =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        referenceHandInWand.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.6f, 0.1f, -0.5f }),
            -19.0f);
        referenceHandInWand.translate = { 2.0f, -5.0f, 3.0f };
        const TestTransform rawReferenceHandWorld =
            rock::transform_math::composeTransforms(
                rawLeftWandWorld,
                referenceHandInWand);

        TestTransform hfrikDampingWorldDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        hfrikDampingWorldDelta.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.2f, -0.3f, 0.8f }),
            7.0f);
        hfrikDampingWorldDelta.translate = { -4.0f, 9.0f, 1.0f };
        const TestTransform dampedPhysicalHandWorld =
            rock::transform_math::composeTransforms(
                hfrikDampingWorldDelta,
                rawReferenceHandWorld);
        const TestTransform dampedAimCarrierWorld =
            rock::left_firing_position_only_math::
                resolveDampedAimCarrierWorld(
                    rawLeftWandWorld,
                    referenceHandInWand,
                    dampedPhysicalHandWorld);
        TestTransform expectedDampedCarrierWorld = rawLeftWandWorld;
        expectedDampedCarrierWorld.rotate =
            rock::transform_math::composeTransforms(
                rock::left_firing_position_only_math::orientationOnly(
                    hfrikDampingWorldDelta),
                rock::left_firing_position_only_math::orientationOnly(
                    rawLeftWandWorld))
                .rotate;
        ok &= expectTransformNear(
            "hFRIK damped follow applies only the observed hand rotation delta",
            dampedAimCarrierWorld,
            expectedDampedCarrierWorld);
        ok &= expectTransformNear(
            "zero hFRIK damping delta preserves the raw aim carrier",
            rock::left_firing_position_only_math::
                resolveDampedAimCarrierWorld(
                    rawLeftWandWorld,
                    referenceHandInWand,
                    rawReferenceHandWorld),
            rawLeftWandWorld);

        const TestTransform weaponOnDampedCarrier =
            rock::transform_math::composeTransforms(
                dampedAimCarrierWorld,
                leftWeaponInWand);
        const TestTransform weaponBackInDampedCarrier =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(
                    dampedAimCarrierWorld),
                weaponOnDampedCarrier);
        ok &= expectTransformNear(
            "shared damping preserves mirrored weapon aim in the corrected carrier",
            weaponBackInDampedCarrier,
            leftWeaponInWand);

        TestTransform leftWandWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        leftWandWorld.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.3f, 0.7f, -0.2f }),
            23.0f);
        leftWandWorld.translate = { 40.0f, -12.0f, 9.0f };

        TestTransform liveWeaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        liveWeaponWorld.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            -67.0f);
        liveWeaponWorld.translate = { -100.0f, 55.0f, 13.0f };
        liveWeaponWorld.scale = 1.25f;
        const TestVector3 firingGripWeaponLocal{ 2.0f, 6.0f, -1.0f };
        const TestVector3 physicalLeftGripTargetWorld{ 18.0f, 27.0f, 33.0f };
        const TestTransform solvedLeftWeapon =
            rock::left_firing_position_only_math::
                resolveWeaponWorldPositionOnly(
                    leftWandWorld,
                    leftWeaponInWand,
                    liveWeaponWorld,
                    firingGripWeaponLocal,
                    physicalLeftGripTargetWorld);
        const TestVector3 solvedGripWorld =
            rock::transform_math::localPointToWorld(
                solvedLeftWeapon,
                firingGripWeaponLocal);
        ok &= expectVectorNear(
            "left position-only weapon solve seats authored firing point",
            solvedGripWorld,
            physicalLeftGripTargetWorld);
        ok &= expectNear(
            "left position-only weapon solve preserves live weapon scale",
            solvedLeftWeapon.scale,
            liveWeaponWorld.scale);

        TestTransform expectedLeftWeaponOrientation =
            rock::transform_math::composeTransforms(
                leftWandWorld,
                leftWeaponInWand);
        expectedLeftWeaponOrientation.translate = solvedLeftWeapon.translate;
        expectedLeftWeaponOrientation.scale = liveWeaponWorld.scale;
        ok &= expectTransformNear(
            "left weapon orientation depends only on native aim, not authored wrist",
            solvedLeftWeapon,
            expectedLeftWeaponOrientation);

        TestTransform authoredLeftWristA =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        authoredLeftWristA.translate = { 1.5f, -2.0f, 0.75f };
        TestTransform authoredLeftWristB = authoredLeftWristA;
        authoredLeftWristB.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            47.0f);
        const TestTransform solvedAfterAuthoredWristChange =
            rock::left_firing_position_only_math::
                resolveWeaponWorldPositionOnly(
                    leftWandWorld,
                    leftWeaponInWand,
                    liveWeaponWorld,
                    firingGripWeaponLocal,
                    physicalLeftGripTargetWorld);
        ok &= expectTransformNear(
            "authored wrist rotation cannot alter left weapon aim",
            solvedAfterAuthoredWristChange,
            solvedLeftWeapon);

        const TestTransform presentedLeftWristA =
            rock::transform_math::composeTransforms(
                solvedLeftWeapon,
                authoredLeftWristA);
        const TestTransform presentedLeftWristB =
            rock::transform_math::composeTransforms(
                solvedLeftWeapon,
                authoredLeftWristB);
        const TestVector3 presentedFingerAxisA =
            rock::transform_math::localVectorToWorld(
                presentedLeftWristA,
                TestVector3{ 0.0f, 1.0f, 0.0f });
        const TestVector3 presentedFingerAxisB =
            rock::transform_math::localVectorToWorld(
                presentedLeftWristB,
                TestVector3{ 0.0f, 1.0f, 0.0f });
        const TestVector3 presentedAxisDelta{
            presentedFingerAxisA.x - presentedFingerAxisB.x,
            presentedFingerAxisA.y - presentedFingerAxisB.y,
            presentedFingerAxisA.z - presentedFingerAxisB.z,
        };
        ok &= expectTrue(
            "authored wrist rotation remains visible on the presented hand",
            std::sqrt(
                presentedAxisDelta.x * presentedAxisDelta.x +
                presentedAxisDelta.y * presentedAxisDelta.y +
                presentedAxisDelta.z * presentedAxisDelta.z) >
                0.1f);
    }

    {
        TestTransform supportInputAtAttach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportInputAtAttach.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.3f, -0.4f, 0.8f }),
            37.0f);
        supportInputAtAttach.translate = { 11.0f, -4.0f, 8.0f };

        TestTransform supportGripTargetAtAttach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportGripTargetAtAttach.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ -0.2f, 0.9f, 0.3f }),
            -24.0f);
        supportGripTargetAtAttach.translate = { -7.0f, 13.0f, 5.0f };

        TestTransform inputToGripTargetLocal{};
        ok &= expectTrue(
            "support baseline captures damped support input to authored grip baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    supportInputAtAttach,
                    supportGripTargetAtAttach,
                    inputToGripTargetLocal));

        TestTransform resolvedAttachTarget{};
        ok &= expectTrue(
            "support baseline resolves captured support baseline",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    supportInputAtAttach,
                    inputToGripTargetLocal,
                    resolvedAttachTarget));
        ok &= expectTransformNear(
            "support baseline unchanged support input reproduces exact authored target",
            resolvedAttachTarget,
            supportGripTargetAtAttach);

        TestTransform laterWorldDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        laterWorldDelta.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.7f, 0.1f, -0.5f }),
            29.0f);
        laterWorldDelta.translate = { 2.0f, -3.0f, 1.5f };
        const TestTransform movedSupportInput =
            rock::transform_math::composeTransforms(
                laterWorldDelta,
                supportInputAtAttach);
        const TestTransform expectedMovedTarget =
            rock::transform_math::composeTransforms(
                laterWorldDelta,
                supportGripTargetAtAttach);
        TestTransform resolvedMovedTarget{};
        ok &= expectTrue(
            "support baseline resolves post-attach support delta",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedSupportInput,
                    inputToGripTargetLocal,
                    resolvedMovedTarget));
        ok &= expectTransformNear(
            "support baseline carries only the post-attach rigid support delta",
            resolvedMovedTarget,
            expectedMovedTarget);

        TestTransform degenerateInput = supportInputAtAttach;
        degenerateInput.scale = 0.0f;
        ok &= expectFalse(
            "support baseline rejects degenerate support input at capture",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    degenerateInput,
                    supportGripTargetAtAttach,
                    inputToGripTargetLocal));

        TestTransform nonFiniteRelation = inputToGripTargetLocal;
        nonFiniteRelation.translate.x =
            (std::numeric_limits<float>::quiet_NaN)();
        ok &= expectFalse(
            "support baseline rejects non-finite captured support baseline",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    supportInputAtAttach,
                    nonFiniteRelation,
                    resolvedMovedTarget));
    }

    {
        TestTransform primaryDriver =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        primaryDriver.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            14.0f);
        primaryDriver.translate = { 2.0f, -5.0f, 7.0f };
        TestTransform supportDriver =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportDriver.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 1.0f, 0.0f },
            -31.0f);
        supportDriver.translate = { -4.0f, 12.0f, 3.0f };

        TestTransform primaryTarget =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        primaryTarget.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            22.0f);
        primaryTarget.translate = { 8.0f, 1.0f, -2.0f };
        TestTransform supportTarget =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportTarget.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.3f, 0.7f, -0.2f }),
            -19.0f);
        supportTarget.translate = { -9.0f, 6.0f, 11.0f };

        TestTransform primaryRelation{};
        TestTransform supportRelation{};
        ok &= expectTrue(
            "dynamic support captures both controller-driver relations atomically",
            rock::weapon_support_acquisition_math::
                tryCaptureDynamicSupportDriverBaseline(
                    primaryDriver,
                    primaryTarget,
                    supportDriver,
                    supportTarget,
                    primaryRelation,
                    supportRelation));

        TestTransform resolvedPrimary{};
        TestTransform resolvedSupport{};
        ok &= expectTrue(
            "dynamic support resolves both controller-driver targets atomically",
            rock::weapon_support_acquisition_math::
                tryResolveDynamicSupportDriverTargets(
                    primaryDriver,
                    primaryRelation,
                    supportDriver,
                    supportRelation,
                    resolvedPrimary,
                    resolvedSupport));
        ok &= expectTransformNear(
            "unchanged primary driver reproduces captured primary target",
            resolvedPrimary,
            primaryTarget);
        ok &= expectTransformNear(
            "unchanged support driver reproduces captured support target",
            resolvedSupport,
            supportTarget);

        TestTransform primaryDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        primaryDelta.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 1.0f, 0.0f },
            9.0f);
        primaryDelta.translate = { 3.0f, 0.0f, -1.0f };
        TestTransform supportDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportDelta.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            -12.0f);
        supportDelta.translate = { -2.0f, 4.0f, 0.5f };
        const TestTransform movedPrimaryDriver =
            rock::transform_math::composeTransforms(
                primaryDelta,
                primaryDriver);
        const TestTransform movedSupportDriver =
            rock::transform_math::composeTransforms(
                supportDelta,
                supportDriver);
        ok &= expectTrue(
            "dynamic support resolves independent post-capture driver deltas",
            rock::weapon_support_acquisition_math::
                tryResolveDynamicSupportDriverTargets(
                    movedPrimaryDriver,
                    primaryRelation,
                    movedSupportDriver,
                    supportRelation,
                    resolvedPrimary,
                    resolvedSupport));
        ok &= expectTransformNear(
            "primary target follows only primary driver delta",
            resolvedPrimary,
            rock::transform_math::composeTransforms(
                primaryDelta,
                primaryTarget));
        ok &= expectTransformNear(
            "support target follows only support driver delta",
            resolvedSupport,
            rock::transform_math::composeTransforms(
                supportDelta,
                supportTarget));

        TestTransform invalidSupportDriver = supportDriver;
        invalidSupportDriver.scale = 0.0f;
        ok &= expectFalse(
            "dynamic support fails closed when either current driver is invalid",
            rock::weapon_support_acquisition_math::
                tryResolveDynamicSupportDriverTargets(
                    primaryDriver,
                    primaryRelation,
                    invalidSupportDriver,
                    supportRelation,
                    resolvedPrimary,
                    resolvedSupport));
    }

    {
        TestTransform weaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        weaponWorld.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            18.0f);
        weaponWorld.translate = { 4.0f, -6.0f, 2.0f };
        const TestVector3 primaryGripLocal{ 0.0f, 0.0f, 0.0f };
        const TestVector3 supportGripLocal{ 0.0f, 12.0f, 0.0f };
        const TestVector3 supportNormalLocal{ 1.0f, 0.0f, 0.0f };
        const TestVector3 primaryTargetWorld =
            rock::transform_math::localPointToWorld(
                weaponWorld,
                primaryGripLocal);
        const TestVector3 supportGripWorld =
            rock::transform_math::localPointToWorld(
                weaponWorld,
                supportGripLocal);

        TestTransform supportGripHandWorld = weaponWorld;
        supportGripHandWorld.translate = supportGripWorld;
        TestTransform dampedSupportInput =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        dampedSupportInput.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 1.0f, 0.0f },
            -41.0f);
        dampedSupportInput.translate = { 20.0f, 7.0f, -3.0f };

        TestTransform inputToGripTargetLocal{};
        TestTransform calibratedSupportTarget{};
        ok &= expectTrue(
            "support baseline solver test captures authored hand target",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    dampedSupportInput,
                    supportGripHandWorld,
                    inputToGripTargetLocal));
        ok &= expectTrue(
            "support baseline solver test resolves authored hand target",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    dampedSupportInput,
                    inputToGripTargetLocal,
                    calibratedSupportTarget));

        const TestVector3 lockedSupportTarget =
            rock::makeLockedSupportGripTarget(
                primaryTargetWorld,
                calibratedSupportTarget.translate,
                supportGripWorld,
                rock::weaponSolverLength(
                    rock::weaponSolverSub(
                        supportGripWorld,
                        primaryTargetWorld)),
                0.001f);
        rock::WeaponTwoHandedSolverInput<
            TestTransform,
            TestVector3> solverInput{};
        solverInput.weaponWorldTransform = weaponWorld;
        solverInput.primaryGripLocal = primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryTargetWorld;
        solverInput.supportTargetWorld = lockedSupportTarget;
        solverInput.supportNormalLocal = supportNormalLocal;
        solverInput.supportNormalTargetWorld =
            rock::transform_math::localVectorToWorld(
                calibratedSupportTarget,
                supportNormalLocal);
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = 0.5f;

        const auto attachSolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
        ok &= expectTrue(
            "support baseline calibrated attach target solves",
            attachSolve.solved);
        ok &= expectTransformNear(
            "support baseline calibrated attach target leaves weapon unchanged",
            attachSolve.weaponWorldTransform,
            weaponWorld);
    }

    {
        const TestTransform weaponAtAttach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        const TestVector3 primaryGripLocal{ 0.0f, 0.0f, 0.0f };
        const TestVector3 supportGripLocal{ 0.0f, 10.0f, 0.0f };
        const TestVector3 supportNormalLocal{ 1.0f, 0.0f, 0.0f };

        TestTransform supportGripHandAtAttach = weaponAtAttach;
        supportGripHandAtAttach.translate = supportGripLocal;
        TestTransform supportInputAtAttach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportInputAtAttach.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            45.0f);
        supportInputAtAttach.translate = { 7.0f, 3.0f, -2.0f };

        TestTransform inputToGripTargetLocal{};
        ok &= expectTrue(
            "support baseline tandem test captures attach baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    supportInputAtAttach,
                    supportGripHandAtAttach,
                    inputToGripTargetLocal));

        TestTransform supportDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        supportDelta.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            30.0f);
        const TestTransform movedSupportInput =
            rock::transform_math::composeTransforms(
                supportDelta,
                supportInputAtAttach);
        TestTransform movedCalibratedTarget{};
        ok &= expectTrue(
            "support baseline tandem test resolves moved support input",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedSupportInput,
                    inputToGripTargetLocal,
                    movedCalibratedTarget));

        rock::WeaponTwoHandedSolverInput<
            TestTransform,
            TestVector3> solverInput{};
        solverInput.weaponWorldTransform = weaponAtAttach;
        solverInput.primaryGripLocal = primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryGripLocal;
        solverInput.supportTargetWorld =
            rock::makeLockedSupportGripTarget(
                primaryGripLocal,
                movedCalibratedTarget.translate,
                supportGripLocal,
                10.0f,
                0.001f);
        solverInput.supportNormalLocal = supportNormalLocal;
        solverInput.supportNormalTargetWorld =
            rock::transform_math::localVectorToWorld(
                movedCalibratedTarget,
                supportNormalLocal);
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = 0.5f;

        const auto movedSolve =
            rock::solveTwoHandedWeaponTransformFrikPivot(solverInput);
        ok &= expectTrue(
            "support baseline post-attach tandem delta solves",
            movedSolve.solved);
        ok &= expectTransformNear(
            "support baseline post-attach support delta drives existing tandem solver",
            movedSolve.weaponWorldTransform,
            supportDelta);
        ok &= expectVectorNear(
            "support baseline tandem delta keeps primary pivot fixed",
            rock::transform_math::localPointToWorld(
                movedSolve.weaponWorldTransform,
                primaryGripLocal),
            primaryGripLocal);
    }

    ok &= expectTrue("one-hand sword is melee", rock::weapon_type_policy::isMelee(TestWeaponType::kOneHandSword));
    ok &= expectTrue("two-hand axe is melee", rock::weapon_type_policy::isMelee(TestWeaponType::kTwoHandAxe));
    ok &= expectFalse("gun does not bit-alias melee", rock::weapon_type_policy::isMelee(TestWeaponType::kGun));
    ok &= expectFalse("grenade is not melee", rock::weapon_type_policy::isMelee(TestWeaponType::kGrenade));

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
        TestTransform rawHandWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        rawHandWorld.translate = { 10.0f, -5.0f, 3.0f };
        const TestVector3 palmPivotWorld{ 12.0f, -5.0f, 3.0f };
        const TestVector3 palmNormalWorld{ 1.0f, 0.0f, 0.0f };
        const TestVector3 surfaceNormalWorld{ 0.0f, -1.0f, 0.0f };
        const TestVector3 targetGripPointWorld{ 30.0f, 9.0f, -4.0f };
        constexpr float kThirtyDegreesRadians =
            0.52359877559829887308f;

        const auto seated =
            rock::weapon_support_acquisition_math::
                alignHandFrameToGripSurface<
                    TestTransform,
                    TestVector3>(
                    rawHandWorld,
                    palmPivotWorld,
                    palmNormalWorld,
                    targetGripPointWorld,
                    surfaceNormalWorld,
                    kThirtyDegreesRadians);
        ok &= expectTrue(
            "support surface seat produces a finite hand frame",
            seated.valid);
        ok &= expectNear(
            "support surface seat clamps wrist swing",
            seated.appliedRotationRadians,
            kThirtyDegreesRadians);

        const TestVector3 rawOriginToPalm =
            rock::weaponSolverSub(
                palmPivotWorld,
                rawHandWorld.translate);
        const TestVector3 seatedPalmPoint =
            rock::weaponSolverAdd(
                seated.handWorld.translate,
                rock::weaponSolverApplyStoredWorldRotationToVector<
                    TestMatrix3,
                    TestVector3>(
                    seated.handWorld.rotate,
                    rawOriginToPalm));
        ok &= expectVectorNear(
            "support surface seat preserves exact palm contact pivot",
            seatedPalmPoint,
            targetGripPointWorld);

        const TestVector3 seatedPalmNormal =
            rock::weaponSolverNormalize(
                rock::weaponSolverApplyStoredWorldRotationToVector<
                    TestMatrix3,
                    TestVector3>(
                    seated.handWorld.rotate,
                    palmNormalWorld));
        ok &= expectNear(
            "support surface seat rotates palm toward inward normal",
            rock::weaponSolverDot(
                seatedPalmNormal,
                TestVector3{ 0.0f, 1.0f, 0.0f }),
            0.5f);

        TestTransform weaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        weaponWorld.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.2f, 0.7f, -0.4f }),
            23.0f);
        weaponWorld.translate = { -40.0f, 70.0f, 18.0f };
        weaponWorld.scale = 1.3f;
        const TestVector3 meshPointLocal{ 4.0f, -3.0f, 2.0f };
        const TestVector3 originalMeshPointWorld =
            rock::transform_math::localPointToWorld(
                weaponWorld,
                meshPointLocal);
        const TestTransform virtualWeaponWorld =
            rock::weapon_two_handed_grip_math::
                virtualizeMeshForSeatedHand(
                    weaponWorld,
                    rawHandWorld,
                    seated.handWorld);
        const TestVector3 virtualMeshPointWorld =
            rock::transform_math::localPointToWorld(
                virtualWeaponWorld,
                meshPointLocal);
        ok &= expectVectorNear(
            "full surface seat virtualization preserves hand-mesh relation",
            rock::transform_math::worldPointToLocal(
                rawHandWorld,
                virtualMeshPointWorld),
            rock::transform_math::worldPointToLocal(
                seated.handWorld,
                originalMeshPointWorld));
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
        TestTransform oldWeaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        oldWeaponWorld.translate = { 8.0f, -3.0f, 4.0f };
        oldWeaponWorld.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            25.0f);
        oldWeaponWorld.scale = 1.2f;

        TestTransform newWeaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        newWeaponWorld.translate = { -6.0f, 12.0f, 2.0f };
        newWeaponWorld.rotate = makeAxisAngleRotation(
            TestVector3{ 1.0f, 0.0f, 0.0f },
            -35.0f);
        newWeaponWorld.scale = 0.85f;

        TestTransform animatedPresentationWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        animatedPresentationWorld.translate = { 15.0f, 7.0f, -2.0f };
        animatedPresentationWorld.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 1.0f, 0.0f },
            48.0f);
        animatedPresentationWorld.scale = 0.9f;

        const TestTransform presentationWorldDelta =
            rock::weapon_visual_authority_math::makePresentationWorldDelta(
                oldWeaponWorld,
                newWeaponWorld);
        const TestTransform reframedPresentationWorld =
            rock::weapon_visual_authority_math::applyPresentationWorldDelta(
                presentationWorldDelta,
                animatedPresentationWorld);
        const TestTransform directReframedPresentationWorld =
            rock::weapon_visual_authority_math::reframePresentationWorld(
                oldWeaponWorld,
                newWeaponWorld,
                animatedPresentationWorld);
        ok &= expectTransformNear(
            "weapon presentation precomputed delta matches direct reframe",
            reframedPresentationWorld,
            directReframedPresentationWorld);

        const TestTransform oldWeaponRelativePresentation =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(oldWeaponWorld),
                animatedPresentationWorld);
        const TestTransform newWeaponRelativePresentation =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(newWeaponWorld),
                reframedPresentationWorld);
        ok &= expectTransformNear(
            "weapon presentation reframe preserves evaluated root-relative world",
            newWeaponRelativePresentation,
            oldWeaponRelativePresentation);

        TestTransform staleAuthoredLocal =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        staleAuthoredLocal.translate = { 100.0f, 200.0f, 300.0f };
        const TestTransform staleLocalRebuild =
            rock::transform_math::composeTransforms(
                newWeaponWorld,
                staleAuthoredLocal);
        ok &= expectTrue(
            "weapon presentation reframe ignores stale descendant local",
            std::fabs(
                reframedPresentationWorld.translate.x -
                staleLocalRebuild.translate.x) > 1.0f);
    }

    {
        TestTransform nativeKickLocal =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        nativeKickLocal.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            60.0f);
        nativeKickLocal.translate = { 10.0f, -4.0f, 2.0f };

        TestTransform controlledKickLocal{};
        ok &= expectTrue(
            "visual-only support builds a controlled rigid recoil sample",
            rock::weapon_recoil_authority_math::
                tryBuildVisualOnlySupportKick(
                    nativeKickLocal,
                    controlledKickLocal));
        TestTransform expectedControlledKick =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        expectedControlledKick.rotate = makeAxisAngleRotation(
            TestVector3{ 0.0f, 0.0f, 1.0f },
            18.0f);
        expectedControlledKick.translate = { 4.5f, -1.8f, 0.9f };
        ok &= expectTransformNear(
            "visual-only support reduces angular recoil more than linear impulse",
            controlledKickLocal,
            expectedControlledKick);

        TestTransform invalidKick = nativeKickLocal;
        invalidKick.rotate.entry[0][0] =
            (std::numeric_limits<float>::quiet_NaN)();
        TestTransform rejectedKick = nativeKickLocal;
        ok &= expectFalse(
            "visual-only support rejects a non-finite native kick",
            rock::weapon_recoil_authority_math::
                tryBuildVisualOnlySupportKick(
                    invalidKick,
                    rejectedKick));
        ok &= expectTransformNear(
            "invalid visual-only recoil fails closed to identity",
            rejectedKick,
            rock::transform_math::makeIdentityTransform<TestTransform>());
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

        TestTransform fineTunedRotatedWeapon = weaponAfter;
        fineTunedRotatedWeapon.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.2f, 0.9f, 0.3f }),
            -10.0f);
        const TestTransform fineTunedScopeWorld =
            rock::native_scope_camera_follow_math::
                resolveRigidAnchorFrameWorld(
                    fineTunedRotatedWeapon,
                    rigidSightFrameLocal);
        const TestTransform fineTunedScopeResolvedLocal =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(
                    fineTunedRotatedWeapon),
                fineTunedScopeWorld);
        ok &= expectTransformNear(
            "fine-tuned rotated weapon preserves retained rigid scope frame",
            fineTunedScopeResolvedLocal,
            rigidSightFrameLocal);

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
        ok &= expectEqual("prior collision presentation reconstructs from the physical driver even while unscoped",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(true, false, true, true, true, 0, 3),
            ResolutionMode::DriverReconstructed);
        ok &= expectEqual("prior collision presentation never falls back to a contaminated root or cached output",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(true, false, true, false, true, 0, 3),
            ResolutionMode::Unavailable);
        ok &= expectEqual("ordinary frames retain the visible-root policy",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(false, false, true, true, true, 0, 3),
            ResolutionMode::RootFlattened);
        ok &= expectEqual("ordinary scoped frames retain bounded last-known continuity",
            rock::scope_safe_hand_frame_math::resolveCollisionIsolatedMode(false, true, false, false, true, 0, 3),
            ResolutionMode::LastKnown);
        ok &= expectTrue("open ScopeMenu selects driver-frame weapon authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, false, false, false));
        ok &= expectTrue("held scope button retains driver-frame authority across a transient ScopeMenu close",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, true, true));
        ok &= expectFalse("released scope button returns to the restored root before manual grip teardown",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, false, true, true));
        ok &= expectTrue("manual grip retains driver-frame authority across a later ScopeMenu reopen",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(true, true, true, true));
        ok &= expectFalse("driver-frame authority stops if manual ownership ends despite a held scope button",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, false, true));
        ok &= expectFalse("ordinary unscoped aiming does not acquire driver-frame authority",
            rock::scope_safe_hand_frame_math::retainDriverFrameAuthority(false, true, true, false));
        ok &= expectFalse("button release never rebases the visible root from a stale hidden-scope hand",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(false, true, true, true));
        ok &= expectTrue("a non-release root handoff may preserve the last scoped hand continuously",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, true, false, true));
        ok &= expectFalse("root rebase requires an actual driver-authority stop edge",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, false, true, true));
        ok &= expectFalse("root rebase requires a valid reconstructed or recent scoped hand",
            rock::scope_safe_hand_frame_math::shouldStartRootRebase(true, true, false, false));
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

    {
        TestTransform renderedTwoHandWeaponWorld =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        renderedTwoHandWeaponWorld.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.31f, -0.44f, 0.72f }),
            37.0f);
        renderedTwoHandWeaponWorld.translate =
            TestVector3{ 18.0f, -7.0f, 23.0f };
        renderedTwoHandWeaponWorld.scale = 1.25f;

        TestTransform authoredLeftHandWeaponLocal =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        authoredLeftHandWeaponLocal.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.0f, 1.0f, 0.0f }),
            90.0f);
        authoredLeftHandWeaponLocal.translate =
            TestVector3{ -4.0f, 22.0f, 1.5f };
        const TestTransform authoredLeftHandTargetAtDetach =
            rock::transform_math::composeTransforms(
                renderedTwoHandWeaponWorld,
                authoredLeftHandWeaponLocal);

        TestTransform rawLeftDriverAtDetach =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        rawLeftDriverAtDetach.translate =
            TestVector3{ -11.0f, 26.0f, 9.0f };
        rawLeftDriverAtDetach.scale = 1.25f;

        TestTransform driverToAuthoredTargetLocal{};
        TestTransform driverToWeaponLocal{};
        ok &= expectTrue(
            "integrated right detach captures raw-driver to authored-target baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    rawLeftDriverAtDetach,
                    authoredLeftHandTargetAtDetach,
                    driverToAuthoredTargetLocal));
        ok &= expectTrue(
            "integrated right detach captures raw-driver to weapon baseline",
            rock::weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    rawLeftDriverAtDetach,
                    renderedTwoHandWeaponWorld,
                    driverToWeaponLocal));

        TestTransform firstAuthoredLeftHandTarget{};
        TestTransform firstPartCarryWeaponWorld{};
        ok &= expectTrue(
            "unchanged raw driver resolves the authored support target",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    rawLeftDriverAtDetach,
                    driverToAuthoredTargetLocal,
                    firstAuthoredLeftHandTarget));
        ok &= expectTrue(
            "unchanged raw driver resolves the rendered weapon pose",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    rawLeftDriverAtDetach,
                    driverToWeaponLocal,
                    firstPartCarryWeaponWorld));
        ok &= expectTransformNear(
            "integrated right detach preserves the authored support target",
            firstAuthoredLeftHandTarget,
            authoredLeftHandTargetAtDetach);
        ok &= expectTransformNear(
            "integrated right detach preserves the rendered two-hand pose",
            firstPartCarryWeaponWorld,
            renderedTwoHandWeaponWorld);

        TestTransform postDetachHandDelta =
            rock::transform_math::makeIdentityTransform<TestTransform>();
        postDetachHandDelta.rotate = makeAxisAngleRotation(
            rock::weaponSolverNormalize(
                TestVector3{ 0.56f, 0.14f, -0.63f }),
            18.0f);
        postDetachHandDelta.translate =
            TestVector3{ 6.0f, -2.0f, 9.0f };
        const TestTransform movedLeftHand =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                rawLeftDriverAtDetach);
        TestTransform movedAuthoredLeftHandTarget{};
        TestTransform movedPartCarryWeaponWorld{};
        ok &= expectTrue(
            "moved raw driver resolves the calibrated authored target",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedLeftHand,
                    driverToAuthoredTargetLocal,
                    movedAuthoredLeftHandTarget));
        ok &= expectTrue(
            "moved raw driver resolves the calibrated weapon",
            rock::weapon_support_acquisition_math::
                tryResolveSupportInputTarget(
                    movedLeftHand,
                    driverToWeaponLocal,
                    movedPartCarryWeaponWorld));
        const TestTransform expectedMovedAuthoredTarget =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                authoredLeftHandTargetAtDetach);
        const TestTransform expectedMovedWeaponWorld =
            rock::transform_math::composeTransforms(
                postDetachHandDelta,
                renderedTwoHandWeaponWorld);
        ok &= expectTransformNear(
            "part carry retains the controller-to-authored-hand orientation offset",
            movedAuthoredLeftHandTarget,
            expectedMovedAuthoredTarget);
        ok &= expectTransformNear(
            "part carry applies only post-detach left-hand rigid motion",
            movedPartCarryWeaponWorld,
            expectedMovedWeaponWorld);
        const TestTransform movedAuthoredHandWeaponLocal =
            rock::transform_math::composeTransforms(
                rock::transform_math::invertTransform(
                    movedPartCarryWeaponWorld),
                movedAuthoredLeftHandTarget);
        ok &= expectTransformNear(
            "part carry never rewrites the authored hand-in-weapon relation",
            movedAuthoredHandWeaponLocal,
            authoredLeftHandWeaponLocal);
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
        .authoredOnlySupportGrabsEnabled = true,
        .toggleGrabEnabled = true,
        .equippedWeaponShoulderStashEnabled = true,
        .immersiveWeapon = {
            .physicalRightFiringGripDetachEnabled = true,
            .physicalRightFiringGripDetachPosePreservationEnabled = true,
            .physicalRightFiringGripReattachRadiusGameUnits = 4.0f,
            .physicalRightFiringGripHapticDurationSeconds = 0.11f,
            .physicalRightFiringGripAttachHapticIntensity = 0.81f,
            .physicalRightFiringGripDetachHapticIntensity = 0.31f,
        },
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
    ok &= expectTrue("base ROCK enables authored-only support acquisition",
        coreWeaponHandling.authoredOnlySupportGrabsEnabled);
    ok &= expectTrue("base ROCK enables configured equipped-weapon toggle grab",
        coreWeaponHandling.toggleGrabEnabled);
    ok &= expectFalse("base ROCK shoulder stash never enables physical detach",
        coreWeaponHandling.primaryDetachEnabled);
    const auto integratedRightDetach =
        rock::resolveEquippedWeaponDetachDecision(
            coreWeaponHandling,
            false);
    ok &= expectEqual("physical right resolves integrated detach authority",
        integratedRightDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::
            IntegratedPhysicalRight);
    ok &= expectTrue("integrated physical right enables firing ownership",
        integratedRightDetach.firingGripOwnershipEnabled);
    ok &= expectTrue("integrated physical right enables detach",
        integratedRightDetach.primaryDetachEnabled);
    ok &= expectTrue("integrated physical right selects detach pose preservation",
        integratedRightDetach.preserveWeaponPoseOnDetach);
    ok &= expectNear("integrated physical right owns reattach radius",
        integratedRightDetach.reattachRadiusGameUnits,
        4.0f);
    ok &= expectNear("integrated physical right owns haptic duration",
        integratedRightDetach.gripHapticDurationSeconds,
        0.11f);
    ok &= expectNear("integrated physical right owns attach haptic",
        integratedRightDetach.gripAttachHapticIntensity,
        0.81f);
    ok &= expectNear("integrated physical right owns detach haptic",
        integratedRightDetach.gripDetachHapticIntensity,
        0.31f);
    ok &= expectEqual(
        "authored-only integrated right rejects an ordinary detached part grab",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedPhysicalRight,
                .detachedHandIsLeft = false,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Reject);
    ok &= expectEqual(
        "authored-only integrated right accepts an exact provider part target",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedPhysicalRight,
                .detachedHandIsLeft = false,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = true,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::ExactProviderTarget);
    ok &= expectEqual(
        "authored-only off preserves unrestricted detached right selection",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedPhysicalRight,
                .detachedHandIsLeft = false,
                .authoredOnlySupportGrabsEnabled = false,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Standard);
    ok &= expectEqual(
        "external detach preserves its established part selection",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        ExternalProvider,
                .detachedHandIsLeft = false,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Standard);
    ok &= expectEqual(
        "integrated policy never restricts a physical-left detached hand",
        rock::immersive_weapon_policy::
            resolveDetachedFiringHandPartGrab({
                .partCarryAuthority =
                    rock::immersive_weapon_policy::DetachAuthority::
                        IntegratedPhysicalRight,
                .detachedHandIsLeft = true,
                .authoredOnlySupportGrabsEnabled = true,
                .exactProviderPartTargetActive = false,
            }),
        rock::immersive_weapon_policy::
            DetachedFiringHandPartGrabSelection::Standard);
    const auto integratedLeftDetach =
        rock::resolveEquippedWeaponDetachDecision(
            coreWeaponHandling,
            true);
    ok &= expectEqual("integrated feature grants no physical left detach authority",
        integratedLeftDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::None);
    ok &= expectFalse("integrated feature cannot detach the physical left hand",
        integratedLeftDetach.primaryDetachEnabled);
    ok &= expectFalse("integrated feature cannot preserve a physical-left detach pose",
        integratedLeftDetach.preserveWeaponPoseOnDetach);
    ok &= expectTrue("base ROCK owns equipped-weapon shoulder stash",
        coreWeaponHandling.equippedWeaponShoulderStashEnabled);
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

    auto stashOnlyBaseline = coreWeaponHandlingBaseline;
    stashOnlyBaseline.ambidextrousHandoffEnabled = false;
    const auto stashOnlyHandling = rock::makeEquippedWeaponHandlingSettings(
        stashOnlyBaseline,
        nullptr);
    ok &= expectFalse("ROCK stash does not create firing ownership with handoff off",
        stashOnlyHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("ROCK stash does not create physical detach with handoff off",
        stashOnlyHandling.primaryDetachEnabled);
    ok &= expectFalse("disabled ROCK ambidextrous mode keeps handoff disabled",
        stashOnlyHandling.ambidextrousHandoffEnabled);
    ok &= expectTrue("authored-only support remains independent of handoff",
        stashOnlyHandling.authoredOnlySupportGrabsEnabled);

    auto fixedOnlyBaseline = stashOnlyBaseline;
    fixedOnlyBaseline.equippedWeaponShoulderStashEnabled = false;
    const auto fixedOnlyHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        nullptr);
    ok &= expectFalse("disabled ROCK handoff and stash release firing ownership",
        fixedOnlyHandling.firingGripOwnershipEnabled);
    ok &= expectFalse("disabled ROCK stash releases primary detach",
        fixedOnlyHandling.primaryDetachEnabled);
    ok &= expectFalse("disabled ROCK stash remains disabled",
        fixedOnlyHandling.equippedWeaponShoulderStashEnabled);

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 externalWeaponHandling{};
    externalWeaponHandling.flags = static_cast<std::uint32_t>(
        rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
    externalWeaponHandling.firingGripProximitySupportRadiusGameUnits = 8.0f;
    auto externalHandling = rock::makeEquippedWeaponHandlingSettings(
        coreWeaponHandlingBaseline,
        &externalWeaponHandling);
    ok &= expectTrue("an equipped-weapon request activates external authority",
        externalHandling.externalAuthorityActive);
    ok &= expectTrue("an addon lease cannot suppress ROCK shoulder stash",
        externalHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("an addon lease cannot suppress ROCK toggle grab",
        externalHandling.toggleGrabEnabled);
    ok &= expectTrue("an addon handling lease cannot suppress authored-only support",
        externalHandling.authoredOnlySupportGrabsEnabled);
    ok &= expectFalse("ROCK stash cannot add detach to a non-detach addon lease",
        externalHandling.primaryDetachEnabled);
    const auto integratedRightUnderNonDetachProvider =
        rock::resolveEquippedWeaponDetachDecision(
            externalHandling,
            false);
    ok &= expectEqual("non-detach provider cannot suppress integrated physical-right detach",
        integratedRightUnderNonDetachProvider.authority,
        rock::immersive_weapon_policy::DetachAuthority::
            IntegratedPhysicalRight);
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
    auto authoredOnlyDisabledHandling = coreWeaponHandling;
    authoredOnlyDisabledHandling.authoredOnlySupportGrabsEnabled = false;
    ok &= expectFalse("authored-only hot reload applies to the next acquisition",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            authoredOnlyDisabledHandling,
            false));
    auto detachPosePreservationDisabled = coreWeaponHandling;
    detachPosePreservationDisabled.immersiveWeapon.
        physicalRightFiringGripDetachPosePreservationEnabled = false;
    const auto integratedRightWithoutPosePreservation =
        rock::resolveEquippedWeaponDetachDecision(
            detachPosePreservationDisabled,
            false);
    ok &= expectTrue("pose preservation off keeps integrated physical-right detach",
        integratedRightWithoutPosePreservation.primaryDetachEnabled);
    ok &= expectFalse("pose preservation off selects the legacy carry relation",
        integratedRightWithoutPosePreservation.preserveWeaponPoseOnDetach);
    ok &= expectFalse("pose-preservation hot reload does not tear down a legal carry",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            detachPosePreservationDisabled,
            false));
    auto addonDetachHandling = fixedOnlyHandling;
    addonDetachHandling.firingGripOwnershipEnabled = true;
    addonDetachHandling.primaryDetachEnabled = true;
    ok &= expectTrue("removing provider detach capability reconciles manual weapon state",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            addonDetachHandling,
            fixedOnlyHandling,
            false));
    auto integratedDetachDisabled = coreWeaponHandling;
    integratedDetachDisabled.immersiveWeapon.
        physicalRightFiringGripDetachEnabled = false;
    ok &= expectTrue("removing integrated physical-right detach reconciles manual weapon state",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            integratedDetachDisabled,
            false));
    const auto disabledIntegratedRightDetach =
        rock::resolveEquippedWeaponDetachDecision(
            integratedDetachDisabled,
            false);
    ok &= expectEqual("disabled integrated policy grants no detach authority",
        disabledIntegratedRightDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::None);
    ok &= expectFalse("disabled integrated policy cannot detach physical right",
        disabledIntegratedRightDetach.primaryDetachEnabled);
    ok &= expectFalse("disabled integrated policy cannot preserve a detach pose",
        disabledIntegratedRightDetach.preserveWeaponPoseOnDetach);
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

    rock::provider::RockProviderEquippedWeaponHandlingRequestV1 legacyStashRequest{};
    legacyStashRequest.flags =
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
        static_cast<std::uint32_t>(
            rock::provider::RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
    legacyStashRequest.firingGripReattachRadiusGameUnits = 9.0f;
    legacyStashRequest.weaponGripHapticDurationSeconds = 0.20f;
    legacyStashRequest.firingGripAttachHapticIntensity = 0.40f;
    legacyStashRequest.firingGripDetachHapticIntensity = 0.50f;
    const auto legacyStashHandling = rock::makeEquippedWeaponHandlingSettings(
        fixedOnlyBaseline,
        &legacyStashRequest);
    ok &= expectFalse("legacy provider stash bit cannot enable ROCK shoulder stash",
        legacyStashHandling.equippedWeaponShoulderStashEnabled);
    ok &= expectTrue("legacy provider detach remains available to its other handling paths",
        legacyStashHandling.primaryDetachEnabled);
    const auto providerRightDetach =
        rock::resolveEquippedWeaponDetachDecision(
            legacyStashHandling,
            false);
    ok &= expectEqual("provider PrimaryDetach retains physical-right authority",
        providerRightDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::ExternalProvider);
    ok &= expectFalse("provider right detach does not inherit integrated pose preservation",
        providerRightDetach.preserveWeaponPoseOnDetach);
    const auto providerLeftDetach =
        rock::resolveEquippedWeaponDetachDecision(
            legacyStashHandling,
            true);
    ok &= expectEqual("provider PrimaryDetach retains physical-left authority",
        providerLeftDetach.authority,
        rock::immersive_weapon_policy::DetachAuthority::ExternalProvider);
    ok &= expectTrue("provider PrimaryDetach remains all-firing-hand capable",
        providerLeftDetach.primaryDetachEnabled);
    ok &= expectFalse("provider PrimaryDetach does not inherit integrated pose preservation",
        providerLeftDetach.preserveWeaponPoseOnDetach);
    ok &= expectNear("provider PrimaryDetach owns reattach tuning",
        providerLeftDetach.reattachRadiusGameUnits,
        9.0f);
    ok &= expectNear("provider PrimaryDetach owns attach haptic tuning",
        providerLeftDetach.gripAttachHapticIntensity,
        0.40f);
    ok &= expectNear("provider PrimaryDetach owns detach haptic tuning",
        providerLeftDetach.gripDetachHapticIntensity,
        0.50f);
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

    auto holdToGrabHandling = coreWeaponHandling;
    holdToGrabHandling.toggleGrabEnabled = false;
    ok &= expectTrue("changing equipped-weapon grab input mode reconciles live grips",
        rock::requiresEquippedWeaponHandlingModeReconcile(
            coreWeaponHandling,
            holdToGrabHandling,
            false));

    {
        toggle_grab::RuntimeState toggleState{};
        toggle_grab::Input toggleInput{
            .enabled = true,
            .inputAllowed = true,
            .weaponOwnershipKey = 0x1234u,
            .occupancy = {},
            .left = {
                .held = true,
                .pressed = true,
                .released = false,
            },
            .right = {
                .held = true,
                .pressed = true,
                .released = false,
            },
        };
        auto toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("open left weapon grip passes its acquisition press",
            toggleDecision.left.held && toggleDecision.left.pressed);
        ok &= expectTrue("open right weapon grip passes its acquisition press",
            toggleDecision.right.held && toggleDecision.right.pressed);
        ok &= expectFalse("acquisition presses are not consumed as releases",
            toggleDecision.leftReleasePressConsumed ||
                toggleDecision.rightReleasePressConsumed);

        const auto toggleAcquisition = toggle_grab::reconcile(
            toggleState,
            true,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = true, .right = true });
        ok &= expectTrue("new left weapon occupancy owns its acquisition press",
            toggleAcquisition.leftGripAcquired);
        ok &= expectTrue("new right weapon occupancy owns its acquisition press",
            toggleAcquisition.rightGripAcquired);
        toggleInput.occupancy = { .left = true, .right = true };
        toggleInput.left = { .released = true };
        toggleInput.right = { .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("left weapon grip stays latched after physical release",
            toggleDecision.left.held && !toggleDecision.left.released);
        ok &= expectTrue("right weapon grip stays latched after physical release",
            toggleDecision.right.held && !toggleDecision.right.released);

        toggleInput.left = { .held = true, .pressed = true };
        toggleInput.right = {};
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("second left press requests a logical release",
            !toggleDecision.left.held && toggleDecision.left.released);
        ok &= expectTrue("second left press is reserved for the weapon release",
            toggleDecision.leftReleasePressConsumed);
        ok &= expectTrue("right weapon grip remains independently latched",
            toggleDecision.right.held &&
                !toggleDecision.rightReleasePressConsumed);

        static_cast<void>(toggle_grab::reconcile(
            toggleState,
            true,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = true, .right = true }));
        toggleInput.occupancy = { .left = true, .right = true };
        toggleInput.left = { .held = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("pending left release stays logically open for debounce",
            !toggleDecision.left.held && !toggleDecision.left.released);

        static_cast<void>(toggle_grab::reconcile(
            toggleState,
            true,
            toggleInput.weaponOwnershipKey,
            toggle_grab::GripOccupancy{ .left = false, .right = true }));
        toggleInput.occupancy = { .left = false, .right = true };
        toggleInput.left = { .held = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectFalse("release press tail cannot re-acquire the left weapon grip",
            toggleDecision.left.held || toggleDecision.left.pressed);
        ok &= expectTrue("right latch survives the peer hand release",
            toggleDecision.right.held);

        toggleInput.left = { .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectEqual("physical release rearms only the released hand",
            toggleState.hands[toggle_grab::handIndex(true)],
            toggle_grab::HandState::Open);
        ok &= expectEqual("peer hand remains latched after left rearm",
            toggleState.hands[toggle_grab::handIndex(false)],
            toggle_grab::HandState::Latched);

        toggleInput.enabled = false;
        toggleInput.right = { .held = false, .released = true };
        toggleDecision = toggle_grab::prepare(toggleState, toggleInput);
        ok &= expectTrue("disabled toggle mode passes physical release input",
            toggleDecision.right.released);
        ok &= expectEqual("disabled toggle mode clears weapon identity",
            toggleState.weaponOwnershipKey,
            std::uint64_t{ 0 });
    }

    using rock::weapon_support_authority_policy::canApplyFiringGripProximityAuthority;
    using rock::weapon_support_authority_policy::canCarryAfterFiringGripDetach;
    using rock::weapon_support_authority_policy::canPromoteSupportGripToFiringGrip;
    using rock::weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode;
    using rock::weapon_support_authority_policy::shouldApplyVisualOnlySupportRecoilAssist;
    using rock::weapon_support_authority_policy::WeaponSupportAuthorityMode;
    ok &= expectTrue("firing-grip proximity contract always applies to eligible equipped weapons",
        canApplyFiringGripProximityAuthority(false));
    ok &= expectFalse("firing-grip proximity never changes a provider-mandated grab mode",
        canApplyFiringGripProximityAuthority(true));
    ok &= expectTrue("full two-hand support can carry after firing-grip detach",
        canCarryAfterFiringGripDetach(
            WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectFalse("visual-only support cannot carry after firing-grip detach",
        canCarryAfterFiringGripDetach(
            WeaponSupportAuthorityMode::VisualOnlySupport));
    ok &= expectEqual("any weapon grab near the firing grip stays visual-only",
        resolveFiringGripProximityAuthorityMode(5.5f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab at the firing-grip radius stays visual-only",
        resolveFiringGripProximityAuthorityMode(6.0f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("any weapon grab away from the firing grip takes full authority",
        resolveFiringGripProximityAuthorityMode(6.5f, 6.0f),
        WeaponSupportAuthorityMode::FullTwoHandedSolver);
    ok &= expectTrue("active core visual-only support receives recoil-only authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            false,
            false));
    ok &= expectFalse("full two-handed support keeps its geometric recoil solve",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::FullTwoHandedSolver,
            true,
            false,
            false));
    ok &= expectFalse("inactive visual support never changes recoil",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            false,
            false,
            false));
    ok &= expectFalse("provider visual glue never inherits recoil authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            true,
            false));
    ok &= expectFalse("AttachOnly visual glue never inherits recoil authority",
        shouldApplyVisualOnlySupportRecoilAssist(
            WeaponSupportAuthorityMode::VisualOnlySupport,
            true,
            false,
            true));
    ok &= expectTrue("active support may attempt handoff regardless of authored or dynamic pose selection",
        canPromoteSupportGripToFiringGrip(true, false));
    ok &= expectFalse("inactive support cannot attempt handoff",
        canPromoteSupportGripToFiringGrip(false, false));
    ok &= expectFalse("AttachOnly support never inherits firing-grip ownership",
        canPromoteSupportGripToFiringGrip(true, true));

    using rock::weapon_interaction_probe_math::isBetterProbeCandidate;
    using rock::weapon_interaction_probe_math::ProbeCandidateRank;
    ok &= expectTrue("closer exact weapon surface always wins",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 }));
    ok &= expectFalse("farther exact weapon surface always loses",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("smaller part breaks an exact surface-distance tie",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectFalse("tiny overlapping AABB cannot beat a closer rendered surface",
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
    ok &= expectTrue("provider detach mode enables right firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .primaryDetachEnabled = true,
        }, false));
    ok &= expectTrue("provider detach mode enables left firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .primaryDetachEnabled = true,
        }, true));
    ok &= expectTrue("ambidextrous handoff independently enables firing-grip ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .ambidextrousHandoffAvailable = true,
        }, true));
    ok &= expectTrue("integrated detach enables physical-right firing ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .physicalRightDetachEnabled = true,
        }, false));
    ok &= expectFalse("integrated detach grants no physical-left firing ownership",
        firingGripOwnershipEnabled(FiringGripModeAvailability{
            .physicalRightDetachEnabled = true,
        }, true));
    ok &= expectFalse("firing-grip ownership is disabled when all modes are off",
        firingGripOwnershipEnabled(FiringGripModeAvailability{}, false));
    ok &= expectTrue("left-hand trigger equip starts ambidextrous handoff ownership",
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
    ok &= expectTrue("provider detach mode can start right-hand ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .primaryDetachEnabled = true,
            },
            .gripHeld = true,
        }));
    ok &= expectTrue("integrated detach can start physical-right ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .physicalRightDetachEnabled = true,
            },
            .gripHeld = true,
        }));
    ok &= expectFalse("integrated detach cannot start physical-left ownership",
        shouldStartHeldWeaponEquipOwnership(HeldWeaponEquipOwnershipInput{
            .modes = FiringGripModeAvailability{
                .physicalRightDetachEnabled = true,
            },
            .handIsLeft = true,
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
    ok &= expectFalse("grip-zone settle equip stays off without an explicit grip-zone capability",
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
    ok &= expectTrue("committed shoulder retrieval survives release while the weapon node resolves",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .committedTransfer = true,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = true,
        }));
    ok &= expectFalse("committed shoulder retrieval still fails closed without pose authority",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .committedTransfer = true,
            .ownershipModeEnabled = true,
            .primaryPoseBlockerAvailable = false,
        }));
    ok &= expectTrue("committed shoulder retrieval starts after squeeze release",
        shouldStartPendingPrimaryOnlyGrip(true, false, true));
    ok &= expectFalse("ordinary held-weapon transfer does not start after squeeze release",
        shouldStartPendingPrimaryOnlyGrip(true, false, false));
    ok &= expectFalse("committed transfer cannot start on a different equipped identity",
        shouldStartPendingPrimaryOnlyGrip(false, true, true));
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
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(0.0f));
    // The confirm debounce is a publication count; the defer window must
    // outlast it at the slowest supported rate (2 frames at 45 FPS).
    ok &= expectTrue("release confirmed on the earliest confirmable frame after a grab is deferred",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(static_cast<float>(kPrimaryReleaseConfirmFrames) / 45.0f));
    ok &= expectTrue("release confirmed at the defer window edge is still deferred",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(kFreshSupportGripPrimaryReleaseDeferSeconds));
    ok &= expectFalse("release confirmed on an aged support grip acts normally",
        shouldDeferPrimaryReleaseActionForFreshSupportGrip(kFreshSupportGripPrimaryReleaseDeferSeconds + 0.001f));

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
    ok &= expectTrue("ROCK shoulder stash is available without realistic detach",
        equippedWeaponShoulderStashAvailable(true));
    ok &= expectFalse("ROCK shoulder stash setting remains authoritative",
        equippedWeaponShoulderStashAvailable(false));
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

    rock::collision_suppression_registry::DelayedRestoreTimer postDropRestore{};
    ok &= expectTrue("post-drop suppression uses grab release delay seconds", postDropRestore.begin(42, 1, 0.8f));
    ok &= expectFalse("post-drop suppression remains active before configured delay", postDropRestore.advance(true, 0.79f));
    ok &= expectTrue("post-drop suppression expires at configured delay", postDropRestore.advance(true, 0.01f));

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

        const std::vector<DetachedComponentInput> basReloadMagazineInputs{
            { .min = { -8.0f, -4.0f, -6.0f }, .max = { 8.0f, 4.0f, 6.0f }, .assembledAnchor = true },
            { .min = { -3.0f, -6.0f, -12.0f }, .max = { 3.0f, 1.0f, -4.0f } },
            { .min = { -4.0f, -28.0f, -122.0f }, .max = { 4.0f, -14.0f, -108.0f } },
            { .min = { -2.0f, -25.0f, -116.0f }, .max = { 2.0f, -18.0f, -110.0f } },
        };
        const auto basReloadMagazineFilter = findDetachedSourceComponentIndices(basReloadMagazineInputs, 2.0f, 24.0f);
        ok &= expectEqual("remote reload magazine component is rejected before collider budgeting",
            basReloadMagazineFilter.verdict,
            DetachedComponentVerdict::Filtered);
        ok &= expectEqual("remote reload magazine component rejects every member", basReloadMagazineFilter.excludedIndices.size(), std::size_t{ 2 });
        ok &= expectTrue("remote reload magazine shell is rejected",
            std::find(basReloadMagazineFilter.excludedIndices.begin(), basReloadMagazineFilter.excludedIndices.end(), 2) !=
                basReloadMagazineFilter.excludedIndices.end());
        ok &= expectTrue("remote reload magazine ammunition is rejected",
            std::find(basReloadMagazineFilter.excludedIndices.begin(), basReloadMagazineFilter.excludedIndices.end(), 3) !=
                basReloadMagazineFilter.excludedIndices.end());
        ok &= expectTrue("remote reload magazine has a measured empty separation",
            basReloadMagazineFilter.minimumExcludedGap >= 24.0f);

        auto nearbyMagazineInputs = basReloadMagazineInputs;
        nearbyMagazineInputs[2].min = { -3.0f, -5.0f, -27.0f };
        nearbyMagazineInputs[2].max = { 3.0f, 1.0f, -19.0f };
        nearbyMagazineInputs[3].min = { -2.0f, -4.0f, -25.0f };
        nearbyMagazineInputs[3].max = { 2.0f, 0.0f, -21.0f };
        const auto nearbyMagazineFilter = findDetachedSourceComponentIndices(nearbyMagazineInputs, 2.0f, 24.0f);
        ok &= expectTrue("nearby disconnected magazine fails open for authored mesh gaps", nearbyMagazineFilter.excludedIndices.empty());

        auto distantStructuralInputs = basReloadMagazineInputs;
        distantStructuralInputs[2].assembledAnchor = true;
        const auto distantStructuralFilter = findDetachedSourceComponentIndices(distantStructuralInputs, 2.0f, 24.0f);
        ok &= expectTrue("distant structural weapon geometry is never rejected by origin distance", distantStructuralFilter.excludedIndices.empty());

        auto coherentAuthoredSourceInputs = basReloadMagazineInputs;
        coherentAuthoredSourceInputs[0].coherenceGroup = 17;
        coherentAuthoredSourceInputs[2].coherenceGroup = 17;
        const auto coherentAuthoredSourceFilter = findDetachedSourceComponentIndices(coherentAuthoredSourceInputs, 2.0f, 24.0f);
        ok &= expectTrue("spatial policy does not split hulls from one authored source", coherentAuthoredSourceFilter.excludedIndices.empty());

        auto ambiguousInputs = basReloadMagazineInputs;
        ambiguousInputs[0].assembledAnchor = false;
        const auto ambiguousFilter = findDetachedSourceComponentIndices(ambiguousInputs, 2.0f, 24.0f);
        ok &= expectEqual("inventory without assembled anchor evidence fails open",
            ambiguousFilter.verdict,
            DetachedComponentVerdict::FailOpenNoAssembledAnchor);
        ok &= expectTrue("ambiguous inventory preserves all sources", ambiguousFilter.excludedIndices.empty());
    }

    return ok ? 0 : 1;
}
