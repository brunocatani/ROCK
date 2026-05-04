#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "api/FRIKApi.h"
#include "physics-interaction/NativeMeleeSuppressionPolicy.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/PhysicsShapeCastMath.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/debug/DebugAxisMath.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/debug/DebugPivotMath.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabVisualAuthorityPolicy.h"
#include "physics-interaction/grab/HeldPlayerSpaceRegistry.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/object/GeometryBodyResolver.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponDebug.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponTypes.h"

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

    TestMatrix makeRotationX90()
    {
        TestMatrix matrix{};
        matrix.entry[0][0] = 1.0f;
        matrix.entry[1][1] = 0.0f;
        matrix.entry[1][2] = -1.0f;
        matrix.entry[2][1] = 1.0f;
        matrix.entry[2][2] = 0.0f;
        return matrix;
    }

    TestMatrix makeRotationY90()
    {
        TestMatrix matrix{};
        matrix.entry[0][0] = 0.0f;
        matrix.entry[0][2] = 1.0f;
        matrix.entry[1][1] = 1.0f;
        matrix.entry[2][0] = -1.0f;
        matrix.entry[2][2] = 0.0f;
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

    bool expectMatrix(const char* label, const TestMatrix& actual, const TestMatrix& expected)
    {
        bool ok = true;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!nearlyEqual(actual.entry[row][column], expected.entry[row][column])) {
                    std::printf("%s[%d][%d] expected %.4f got %.4f\n",
                        label,
                        row,
                        column,
                        expected.entry[row][column],
                        actual.entry[row][column]);
                    ok = false;
                }
            }
        }
        return ok;
    }

    bool expectContainsIndex(const char* label, const std::vector<std::size_t>& values, std::size_t expected)
    {
        if (std::find(values.begin(), values.end(), expected) != values.end()) {
            return true;
        }

        std::printf("%s expected selected index %zu\n", label, expected);
        return false;
    }

    bool expectString(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected '%s' got '%s'\n", label, expected, actual ? actual : "(null)");
        return false;
    }

    bool expectString(const char* label, const std::string& actual, const char* expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected '%s' got '%s'\n", label, expected, actual.c_str());
        return false;
    }

    bool expectUint32(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected 0x%08X got 0x%08X\n", label, expected, actual);
        return false;
    }

    bool expectCandidateKind(const char* label, rock::origin_diagnostics::OriginCandidateKind actual, rock::origin_diagnostics::OriginCandidateKind expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected candidate %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }
}

int main()
{
    bool ok = true;

    {
        using namespace rock::selection_highlight_policy;

        ok &= expectUint32("selection highlight does not enable VATS image space",
            kVatsHighlightEnableImageSpaceEffect ? 1u : 0u,
            0u);
        ok &= expectUint32("selection highlight uses object rollover render state",
            kVatsHighlightUseObjectRolloverState ? 1u : 0u,
            1u);

        ok &= expectUint32("selection highlight starts only for enabled new ref",
            shouldStartVatsHighlight(true, true, false) ? 1u : 0u,
            1u);
        ok &= expectUint32("selection highlight blocks disabled",
            shouldStartVatsHighlight(false, true, false) ? 1u : 0u,
            0u);
        ok &= expectUint32("selection highlight blocks null ref",
            shouldStartVatsHighlight(true, false, false) ? 1u : 0u,
            0u);
        ok &= expectUint32("selection highlight keeps existing target stable",
            shouldStartVatsHighlight(true, true, true) ? 1u : 0u,
            0u);
    }

    {
        using namespace rock::origin_diagnostics;

        const RE::NiPoint3 raw{ 10.0f, 20.0f, 30.0f };
        const RE::NiPoint3 zero{};

        const auto zeroOrigin = evaluateOriginCandidates(raw, raw, zero, zero, zero, 5.0f);
        bool originOk = true;
        originOk &= expectCandidateKind("origin zero chooses raw", zeroOrigin.best.kind, OriginCandidateKind::Raw);
        originOk &= expectFloat("origin zero distance", zeroOrigin.best.distanceToVisual, 0.0f);
        originOk &= !zeroOrigin.exceedsWarningThreshold;

        const RE::NiPoint3 bhkOrigin{ 100.0f, -50.0f, 25.0f };
        const auto bhkPlus = evaluateOriginCandidates(raw, RE::NiPoint3{ 110.0f, -30.0f, 55.0f }, bhkOrigin, zero, zero, 5.0f);
        originOk &= expectCandidateKind("origin bhk plus chooses add", bhkPlus.best.kind, OriginCandidateKind::BhkStoredOriginAdd);
        originOk &= expectFloat("origin bhk plus distance", bhkPlus.best.distanceToVisual, 0.0f);
        originOk &= !bhkPlus.exceedsWarningThreshold;

        const RE::NiPoint3 hknpOrigin{ -40.0f, 80.0f, 5.0f };
        const auto hknpMinus = evaluateOriginCandidates(raw, RE::NiPoint3{ 50.0f, -60.0f, 25.0f }, zero, hknpOrigin, hknpOrigin, 5.0f);
        originOk &= expectCandidateKind("origin hknp minus chooses subtract", hknpMinus.best.kind, OriginCandidateKind::HknpBroadphaseMinShiftSubtract);
        originOk &= expectFloat("origin hknp minus distance", hknpMinus.best.distanceToVisual, 0.0f);
        originOk &= !hknpMinus.exceedsWarningThreshold;

        const auto mismatch = evaluateOriginCandidates(raw, RE::NiPoint3{ 1000.0f, 1000.0f, 1000.0f }, zero, zero, zero, 5.0f);
        originOk &= expectCandidateKind("origin mismatch keeps raw", mismatch.best.kind, OriginCandidateKind::Raw);
        originOk &= mismatch.exceedsWarningThreshold;

        ok &= originOk;
    }

    {
        using namespace rock::physics_scale;

        const float engineHavokToGame = 69.99125f;
        const auto engineScale = makeSnapshot(1.0f / engineHavokToGame, engineHavokToGame, 71.0f, 7);
        const auto hardcodedSeventy = makeSnapshot(1.0f / 70.0f, 70.0f, 71.0f, 1);
        const TestVector redRocketVisual{ -69799.97f, 79818.46f, 7479.38f };
        const auto redRocketBodyHavok = gameToHavokPoint(redRocketVisual, engineScale);
        const auto engineRoundTrip = havokToGamePoint(redRocketBodyHavok, engineScale);
        const auto hardcodedRoundTrip = havokToGamePoint(redRocketBodyHavok, hardcodedSeventy);
        const auto engineDrift = distanceGame(engineRoundTrip, redRocketVisual);
        const auto hardcodedDrift = distanceGame(hardcodedRoundTrip, redRocketVisual);

        ok &= expectFloat("physics scale keeps frik vr scale separate", engineScale.vrScale, 71.0f);
        ok &= expectFloat("physics scale keeps engine havok scale separate", engineScale.havokToGame, engineHavokToGame);
        ok &= expectFloat("physics scale red rocket engine drift is sub-centimeter", engineDrift < 0.05f ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("physics scale red rocket hardcoded seventy drift is visible", hardcodedDrift > 10.0f && hardcodedDrift < 20.0f ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("physics scale revision invalidates caches", shouldInvalidateCachedScaleData(hardcodedSeventy, engineScale) ? 1.0f : 0.0f, 1.0f);
    }

    const TestMatrix niRotation = makeNonSymmetricRotation();
    const TestMatrix hkRotation = rock::transform_math::niRowsToHavokColumns(niRotation);
    const auto* hkFloats = reinterpret_cast<const float*>(&hkRotation);

    ok &= expectFloat("hk[0]", hkFloats[0], niRotation.entry[0][0]);
    ok &= expectFloat("hk[1]", hkFloats[1], niRotation.entry[1][0]);
    ok &= expectFloat("hk[2]", hkFloats[2], niRotation.entry[2][0]);
    ok &= expectFloat("hk[4]", hkFloats[4], niRotation.entry[0][1]);
    ok &= expectFloat("hk[5]", hkFloats[5], niRotation.entry[1][1]);
    ok &= expectFloat("hk[6]", hkFloats[6], niRotation.entry[2][1]);
    ok &= expectFloat("hk[8]", hkFloats[8], niRotation.entry[0][2]);
    ok &= expectFloat("hk[9]", hkFloats[9], niRotation.entry[1][2]);
    ok &= expectFloat("hk[10]", hkFloats[10], niRotation.entry[2][2]);

    TestTransform desiredGrabFrame{};
    desiredGrabFrame.rotate = niRotation;
    desiredGrabFrame.scale = 1.0f;
    alignas(16) float initialTransformB[12]{};
    alignas(16) float initialTargetBRca[12]{};
    rock::grab_constraint_math::writeInitialGrabAngularFrame(initialTransformB, initialTargetBRca, desiredGrabFrame);
    const TestMatrix expectedBodyToHandRotation = rock::transform_math::transposeRotation(niRotation);
    ok &= expectFloat("grab initial transformB inverse col0.x", initialTransformB[0], expectedBodyToHandRotation.entry[0][0]);
    ok &= expectFloat("grab initial transformB inverse col0.y", initialTransformB[1], expectedBodyToHandRotation.entry[1][0]);
    ok &= expectFloat("grab initial transformB inverse col0.z", initialTransformB[2], expectedBodyToHandRotation.entry[2][0]);
    ok &= expectFloat("grab initial transformB inverse col1.x", initialTransformB[4], expectedBodyToHandRotation.entry[0][1]);
    ok &= expectFloat("grab initial transformB inverse col1.y", initialTransformB[5], expectedBodyToHandRotation.entry[1][1]);
    ok &= expectFloat("grab initial transformB inverse col1.z", initialTransformB[6], expectedBodyToHandRotation.entry[2][1]);
    ok &= expectFloat("grab initial transformB inverse col2.x", initialTransformB[8], expectedBodyToHandRotation.entry[0][2]);
    ok &= expectFloat("grab initial transformB inverse col2.y", initialTransformB[9], expectedBodyToHandRotation.entry[1][2]);
    ok &= expectFloat("grab initial transformB inverse col2.z", initialTransformB[10], expectedBodyToHandRotation.entry[2][2]);
    for (int columnStart = 0; columnStart <= 8; columnStart += 4) {
        for (int component = 0; component < 4; ++component) {
            char label[64];
            std::snprintf(label, sizeof(label), "grab initial target matches transformB[%d]", columnStart + component);
            ok &= expectFloat(label, initialTargetBRca[columnStart + component], initialTransformB[columnStart + component]);
        }
    }

    {
        TestTransform desiredBodyInHand = rock::transform_math::makeIdentityTransform<TestTransform>();
        desiredBodyInHand.rotate = niRotation;
        desiredBodyInHand.translate = TestVector{ 3.0f, -2.0f, 5.0f };
        const TestVector initialPivotAHand{ 8.0f, -1.0f, 6.0f };
        const TestVector movedPivotAHand{ 12.0f, -4.0f, 9.0f };

        const TestVector initialPivotBBody =
            rock::grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyInHand, initialPivotAHand);
        const TestVector movedPivotBBody =
            rock::grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyInHand, movedPivotAHand);

        const TestVector initialPivotRoundTrip = rock::transform_math::localPointToWorld(desiredBodyInHand, initialPivotBBody);
        const TestVector movedPivotRoundTrip = rock::transform_math::localPointToWorld(desiredBodyInHand, movedPivotBBody);
        ok &= expectFloat("grab dynamic transformB initial maps to pivotA.x", initialPivotRoundTrip.x, initialPivotAHand.x);
        ok &= expectFloat("grab dynamic transformB initial maps to pivotA.y", initialPivotRoundTrip.y, initialPivotAHand.y);
        ok &= expectFloat("grab dynamic transformB initial maps to pivotA.z", initialPivotRoundTrip.z, initialPivotAHand.z);
        ok &= expectFloat("grab dynamic transformB moved maps to pivotA.x", movedPivotRoundTrip.x, movedPivotAHand.x);
        ok &= expectFloat("grab dynamic transformB moved maps to pivotA.y", movedPivotRoundTrip.y, movedPivotAHand.y);
        ok &= expectFloat("grab dynamic transformB moved maps to pivotA.z", movedPivotRoundTrip.z, movedPivotAHand.z);
        ok &= expectFloat("grab dynamic transformB moved changes local x", movedPivotBBody.x, 2.0f);
        ok &= expectFloat("grab dynamic transformB moved changes local y", movedPivotBBody.y, 9.0f);
        ok &= expectFloat("grab dynamic transformB moved changes local z", movedPivotBBody.z, 4.0f);

        alignas(16) float dynamicTransformBPos[4]{};
        rock::grab_constraint_math::writeDynamicTransformBTranslation(dynamicTransformBPos, desiredBodyInHand, movedPivotAHand, 1.0f / 70.0f);
        ok &= expectFloat("grab dynamic transformB hk x", dynamicTransformBPos[0], movedPivotBBody.x / 70.0f);
        ok &= expectFloat("grab dynamic transformB hk y", dynamicTransformBPos[1], movedPivotBBody.y / 70.0f);
        ok &= expectFloat("grab dynamic transformB hk z", dynamicTransformBPos[2], movedPivotBBody.z / 70.0f);
        ok &= expectFloat("grab dynamic transformB hk w", dynamicTransformBPos[3], 0.0f);

        TestTransform liveHandBody = rock::transform_math::makeIdentityTransform<TestTransform>();
        liveHandBody.rotate = niRotation;
        liveHandBody.translate = TestVector{ 50.0f, 100.0f, -25.0f };
        const TestVector expectedPivotALocal{ 2.0f, -3.0f, 4.0f };
        const TestVector livePivotWorld = rock::transform_math::localPointToWorld(liveHandBody, expectedPivotALocal);
        const TestVector pivotALocal =
            rock::grab_constraint_math::computeConstraintPivotLocalGame(liveHandBody, livePivotWorld);
        const TestVector pivotARoundTrip = rock::transform_math::localPointToWorld(liveHandBody, pivotALocal);

        ok &= expectFloat("grab transformA live body local x", pivotALocal.x, expectedPivotALocal.x);
        ok &= expectFloat("grab transformA live body local y", pivotALocal.y, expectedPivotALocal.y);
        ok &= expectFloat("grab transformA live body local z", pivotALocal.z, expectedPivotALocal.z);
        ok &= expectFloat("grab transformA live body roundtrip x", pivotARoundTrip.x, livePivotWorld.x);
        ok &= expectFloat("grab transformA live body roundtrip y", pivotARoundTrip.y, livePivotWorld.y);
        ok &= expectFloat("grab transformA live body roundtrip z", pivotARoundTrip.z, livePivotWorld.z);

        alignas(16) float pivotAHavok[4]{};
        rock::grab_constraint_math::writeConstraintPivotLocalTranslation(pivotAHavok, liveHandBody, livePivotWorld, 1.0f / 70.0f);
        ok &= expectFloat("grab transformA live body hk x", pivotAHavok[0], expectedPivotALocal.x / 70.0f);
        ok &= expectFloat("grab transformA live body hk y", pivotAHavok[1], expectedPivotALocal.y / 70.0f);
        ok &= expectFloat("grab transformA live body hk z", pivotAHavok[2], expectedPivotALocal.z / 70.0f);
        ok &= expectFloat("grab transformA live body hk w", pivotAHavok[3], 0.0f);
    }

    {
        alignas(16) unsigned char setupAtom[16]{};
        setupAtom[2] = 1;
        rock::grab_constraint_math::writeSetupStabilizationDefaults(setupAtom);

        auto readU32 = [&](std::size_t offset) {
            std::uint32_t value = 0;
            std::memcpy(&value, setupAtom + offset, sizeof(value));
            return value;
        };

        ok &= expectFloat("grab setup stabilization disabled", setupAtom[2] ? 1.0f : 0.0f, 0.0f);
        ok &= expectUint32("grab setup stabilization max linear impulse bits", readU32(4), 0x7f7fffee);
        ok &= expectUint32("grab setup stabilization max angular impulse bits", readU32(8), 0x7f7fffee);
        ok &= expectUint32("grab setup stabilization max angle bits", readU32(12), 0x5f7ffff0);
    }

    const TestMatrix roundTrip = rock::transform_math::havokColumnsToNiRows<TestMatrix>(hkFloats);
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

    const auto sharedWorldPoint = rock::transform_math::localPointToWorld(rootTransform, TestVector{ 5.0f, 0.0f, 0.0f });
    ok &= expectFloat("sharedLocalToWorld.x", sharedWorldPoint.x, 10.0f);
    ok &= expectFloat("sharedLocalToWorld.y", sharedWorldPoint.y, 10.0f);
    ok &= expectFloat("sharedLocalToWorld.z", sharedWorldPoint.z, 30.0f);

    const auto sharedLocalRoundTrip = rock::transform_math::worldPointToLocal(rootTransform, sharedWorldPoint);
    ok &= expectFloat("sharedWorldToLocal.x", sharedLocalRoundTrip.x, 5.0f);
    ok &= expectFloat("sharedWorldToLocal.y", sharedLocalRoundTrip.y, 0.0f);
    ok &= expectFloat("sharedWorldToLocal.z", sharedLocalRoundTrip.z, 0.0f);

    const auto sharedLocalDelta = rock::transform_math::worldVectorToLocal(rootTransform, TestVector{ 0.0f, -10.0f, 0.0f });
    ok &= expectFloat("sharedWorldVectorToLocal.x", sharedLocalDelta.x, 5.0f);
    ok &= expectFloat("sharedWorldVectorToLocal.y", sharedLocalDelta.y, 0.0f);
    ok &= expectFloat("sharedWorldVectorToLocal.z", sharedLocalDelta.z, 0.0f);

    TestTransform childTransform{};
    childTransform.translate = TestVector{ 5.0f, 1.0f, 0.0f };
    childTransform.scale = 3.0f;
    childTransform.rotate.entry[0][0] = 1.0f;
    childTransform.rotate.entry[1][2] = -1.0f;
    childTransform.rotate.entry[2][1] = 1.0f;

    const auto composedTransform = rock::transform_math::composeTransforms(rootTransform, childTransform);
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

    const auto inverseRootTransform = rock::transform_math::invertTransform(rootTransform);
    const auto inverseRoundTrip = rock::transform_math::localPointToWorld(inverseRootTransform, sharedWorldPoint);
    ok &= expectFloat("sharedInverseRoundTrip.x", inverseRoundTrip.x, 5.0f);
    ok &= expectFloat("sharedInverseRoundTrip.y", inverseRoundTrip.y, 0.0f);
    ok &= expectFloat("sharedInverseRoundTrip.z", inverseRoundTrip.z, 0.0f);

    float quat[4]{};
    rock::transform_math::niRowsToHavokQuaternion(niRotation, quat);
    ok &= expectFloat("quat.x", quat[0], 0.0f);
    ok &= expectFloat("quat.y", quat[1], 0.0f);
    ok &= expectFloat("quat.z", quat[2], 0.7071067f);
    ok &= expectFloat("quat.w", quat[3], 0.7071067f);

    const auto axisX = rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 1.0f, 0.0f, 0.0f });
    const auto axisY = rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 0.0f, 1.0f, 0.0f });
    const auto axisZ = rock::debug_axis_math::rotateNiLocalToWorld(niRotation, TestVector{ 0.0f, 0.0f, 1.0f });
    ok &= expectFloat("axisX.x", axisX.x, 0.0f);
    ok &= expectFloat("axisX.y", axisX.y, -1.0f);
    ok &= expectFloat("axisX.z", axisX.z, 0.0f);
    ok &= expectFloat("axisY.x", axisY.x, 1.0f);
    ok &= expectFloat("axisY.y", axisY.y, 0.0f);
    ok &= expectFloat("axisY.z", axisY.z, 0.0f);
    ok &= expectFloat("axisZ.x", axisZ.x, 0.0f);
    ok &= expectFloat("axisZ.y", axisZ.y, 0.0f);
    ok &= expectFloat("axisZ.z", axisZ.z, 1.0f);

    const auto activeRightOffset = rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.0f }, false);
    const auto activeLeftOffset = rock::handspace_convention::authoredToRawForHand(TestVector{ 0.086f, -0.005f, 0.0f }, true);
    ok &= expectFloat("activeRightOffset.x", activeRightOffset.x, 0.086f);
    ok &= expectFloat("activeRightOffset.y", activeRightOffset.y, 0.0f);
    ok &= expectFloat("activeRightOffset.z", activeRightOffset.z, 0.005f);
    ok &= expectFloat("activeLeftOffset.x", activeLeftOffset.x, 0.086f);
    ok &= expectFloat("activeLeftOffset.y", activeLeftOffset.y, 0.0f);
    ok &= expectFloat("activeLeftOffset.z", activeLeftOffset.z, 0.005f);

    const auto activeAuthoredX = rock::handspace_convention::authoredToRaw(TestVector{ 1.0f, 0.0f, 0.0f });
    const auto activeAuthoredY = rock::handspace_convention::authoredToRaw(TestVector{ 0.0f, 1.0f, 0.0f });
    const auto activeAuthoredZ = rock::handspace_convention::authoredToRaw(TestVector{ 0.0f, 0.0f, 1.0f });
    ok &= expectFloat("activeAuthoredX.x", activeAuthoredX.x, 1.0f);
    ok &= expectFloat("activeAuthoredX.y", activeAuthoredX.y, 0.0f);
    ok &= expectFloat("activeAuthoredX.z", activeAuthoredX.z, 0.0f);
    ok &= expectFloat("activeAuthoredY.x", activeAuthoredY.x, 0.0f);
    ok &= expectFloat("activeAuthoredY.y", activeAuthoredY.y, 0.0f);
    ok &= expectFloat("activeAuthoredY.z", activeAuthoredY.z, -1.0f);
    ok &= expectFloat("activeAuthoredZ.x", activeAuthoredZ.x, 0.0f);
    ok &= expectFloat("activeAuthoredZ.y", activeAuthoredZ.y, 1.0f);
    ok &= expectFloat("activeAuthoredZ.z", activeAuthoredZ.z, 0.0f);

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
    const auto pivotWorld = rock::debug_pivot_math::bodyLocalPointToWorldGamePoint<TestVector>(bodyFloats, localPivot, 70.0f);
    ok &= expectFloat("pivotWorld.x", pivotWorld.x, -140.0f);
    ok &= expectFloat("pivotWorld.y", pivotWorld.y, 280.0f);
    ok &= expectFloat("pivotWorld.z", pivotWorld.z, 490.0f);

    ok &= expectString("right grab node default is ROCK-owned", std::string(rock::grab_node_name_policy::defaultGrabNodeName(false)), "ROCK:GrabR");
    ok &= expectString("left grab node default is ROCK-owned", std::string(rock::grab_node_name_policy::defaultGrabNodeName(true)), "ROCK:GrabL");
    ok &= expectString(
        "empty right grab node falls back to ROCK default",
        rock::grab_node_name_policy::sanitizeConfiguredGrabNodeName("", false).c_str(),
        "ROCK:GrabR");
    ok &= expectString(
        "stale HIGGS right grab node falls back to ROCK default",
        rock::grab_node_name_policy::sanitizeConfiguredGrabNodeName("HIGGS:GrabR", false).c_str(),
        "ROCK:GrabR");
    ok &= expectString(
        "custom ROCK grab node is preserved",
        rock::grab_node_name_policy::sanitizeConfiguredGrabNodeName("ROCK:CustomBottleGrabR", false).c_str(),
        "ROCK:CustomBottleGrabR");

    {
        TestTransform objectWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        objectWorld.translate = TestVector{ 100.0f, 200.0f, -50.0f };
        objectWorld.rotate = makeRotationY90();

        TestTransform authoredGrabNodeLocal = rock::transform_math::makeIdentityTransform<TestTransform>();
        authoredGrabNodeLocal.translate = TestVector{ 4.0f, -2.0f, 9.0f };
        authoredGrabNodeLocal.rotate = makeNonSymmetricRotation();

        const TestTransform authoredGrabNodeWorld = rock::transform_math::composeTransforms(objectWorld, authoredGrabNodeLocal);

        TestTransform handWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        handWorld.translate = TestVector{ -12.0f, 30.0f, 44.0f };
        handWorld.rotate = makeRotationX90();
        const TestVector grabPivotWorld{ -10.0f, 31.0f, 42.0f };

        const auto desiredObjectWorld = rock::grab_node_info_math::buildDesiredObjectWorldFromAuthoredGrabNode(
            objectWorld,
            authoredGrabNodeWorld,
            handWorld,
            grabPivotWorld);
        const auto recoveredGrabNodeLocal = rock::grab_node_info_math::computeGrabNodeLocalTransformForCurrentGrab(
            desiredObjectWorld,
            handWorld,
            grabPivotWorld);

        ok &= expectFloat("grab node info recovers local x", recoveredGrabNodeLocal.translate.x, authoredGrabNodeLocal.translate.x);
        ok &= expectFloat("grab node info recovers local y", recoveredGrabNodeLocal.translate.y, authoredGrabNodeLocal.translate.y);
        ok &= expectFloat("grab node info recovers local z", recoveredGrabNodeLocal.translate.z, authoredGrabNodeLocal.translate.z);
        ok &= expectMatrix("grab node info recovers local rotation", recoveredGrabNodeLocal.rotate, authoredGrabNodeLocal.rotate);

        const auto identityEuler = rock::grab_node_info_math::nifskopeMatrixToEulerDegrees(
            rock::transform_math::makeIdentityRotation<TestMatrix>());
        ok &= expectFloat("grab node info identity euler x", identityEuler.x, 0.0f);
        ok &= expectFloat("grab node info identity euler y", identityEuler.y, 0.0f);
        ok &= expectFloat("grab node info identity euler z", identityEuler.z, 0.0f);

        const auto z90Euler = rock::grab_node_info_math::nifskopeMatrixToEulerDegrees(makeNonSymmetricRotation());
        ok &= expectFloat("grab node info z90 euler x", z90Euler.x, 0.0f);
        ok &= expectFloat("grab node info z90 euler y", z90Euler.y, 0.0f);
        ok &= expectFloat("grab node info z90 euler z", z90Euler.z, 90.0f);
    }

    {
        using namespace rock::grab_surface_frame_math;
        const TestVector v0{ 0.0f, -5.0f, 0.0f };
        const TestVector v1{ 0.0f, 5.0f, 0.0f };
        const TestVector v2{ 0.0f, 0.0f, 2.0f };
        const auto sideFrame = buildSurfaceFrameFromTriangle(v0, v1, v2, TestVector{ 0.0f, 1.0f, 0.0f }, TestVector{ 0.0f, 1.0f, 0.0f });
        ok &= expectFloat("surface side normal x", sideFrame.normal.x, 1.0f);
        ok &= expectFloat("surface side tangent y", sideFrame.tangent.y, 1.0f);
        ok &= expectFloat("surface side bitangent z", sideFrame.bitangent.z, 1.0f);
        ok &= expectFloat("surface side face kind", static_cast<float>(sideFrame.faceKind), static_cast<float>(GrabSurfaceFaceKind::Side));
        ok &= expectFloat("surface side tangent source", static_cast<float>(sideFrame.tangentSource), static_cast<float>(GrabSurfaceTangentSource::ObjectLongAxis));
        ok &= expectFloat("surface side confidence", sideFrame.confidence, 1.0f);

        const auto longPropSideFrame = buildSurfaceFrameFromTriangle(
            TestVector{ 0.0f, -0.5f, 0.0f },
            TestVector{ 0.0f, 0.5f, 0.0f },
            TestVector{ 0.0f, 0.0f, 5.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            TestVector{ 0.0f, 1.0f, 0.0f });
        ok &= expectFloat("long prop side tangent uses object axis x", longPropSideFrame.tangent.x, 0.0f);
        ok &= expectFloat("long prop side tangent uses object axis y", longPropSideFrame.tangent.y, 1.0f);
        ok &= expectFloat("long prop side tangent source", static_cast<float>(longPropSideFrame.tangentSource), static_cast<float>(GrabSurfaceTangentSource::ObjectLongAxis));

        TestTransform objectWorld{};
        objectWorld.rotate = rock::transform_math::makeIdentityRotation<TestMatrix>();
        objectWorld.scale = 2.0f;
        objectWorld.translate = TestVector{ 100.0f, 50.0f, 25.0f };
        const TestVector localSurfacePoint{ 0.0f, 3.0f, 1.0f };
        const TestVector grabPivotWorld{ 10.0f, 20.0f, 30.0f };
        const auto desired = buildDesiredObjectWorldFromSurfaceFrame(objectWorld,
            localSurfacePoint,
            sideFrame,
            grabPivotWorld,
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 1.0f, 0.0f },
            GrabOrientationMode::SurfaceNormalAuto);
        const auto desiredNormal = rock::transform_math::rotateLocalVectorToWorld(desired.transform.rotate, sideFrame.normal);
        const auto desiredTangent = rock::transform_math::rotateLocalVectorToWorld(desired.transform.rotate, sideFrame.tangent);
        const auto desiredSurface = rock::transform_math::localPointToWorld(desired.transform, localSurfacePoint);
        ok &= expectFloat("surface normal opposes palm x", desiredNormal.x, -1.0f);
        ok &= expectFloat("surface normal opposes palm y", desiredNormal.y, 0.0f);
        ok &= expectFloat("surface tangent follows palm tangent y", desiredTangent.y, 1.0f);
        ok &= expectFloat("surface pivot align x", desiredSurface.x, grabPivotWorld.x);
        ok &= expectFloat("surface pivot align y", desiredSurface.y, grabPivotWorld.y);
        ok &= expectFloat("surface pivot align z", desiredSurface.z, grabPivotWorld.z);
        ok &= expectFloat("surface mode used auto", static_cast<float>(desired.modeUsed), static_cast<float>(GrabOrientationMode::SurfaceNormalAuto));

        const auto handParallelDesired = buildDesiredObjectWorldFromSurfaceFrame(objectWorld,
            localSurfacePoint,
            longPropSideFrame,
            grabPivotWorld,
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            GrabOrientationMode::SurfaceNormalAuto);
        const auto handParallelTangent = rock::transform_math::rotateLocalVectorToWorld(handParallelDesired.transform.rotate, longPropSideFrame.tangent);
        ok &= expectFloat("long prop tangent becomes hand parallel x", handParallelTangent.x, 0.0f);
        ok &= expectFloat("long prop tangent becomes hand parallel z", handParallelTangent.z, 1.0f);

        const auto acceptedAlignment = evaluateSurfaceAlignmentGate(longPropSideFrame,
            GrabOrientationMode::SurfaceNormalAuto,
            true,
            true,
            3.0f,
            2.0f,
            8.0f,
            8.0f,
            true);
        ok &= expectFloat("surface alignment accepts coherent long prop side",
            static_cast<float>(acceptedAlignment),
            static_cast<float>(GrabSurfaceAlignmentDecision::Accepted));
        const auto rejectedDistance = evaluateSurfaceAlignmentGate(longPropSideFrame,
            GrabOrientationMode::SurfaceNormalAuto,
            true,
            true,
            9.0f,
            2.0f,
            8.0f,
            8.0f,
            true);
        ok &= expectFloat("surface alignment rejects far palm contact",
            static_cast<float>(rejectedDistance),
            static_cast<float>(GrabSurfaceAlignmentDecision::RejectedPivotDistance));
        const auto rejectedOwner = evaluateSurfaceAlignmentGate(longPropSideFrame,
            GrabOrientationMode::SurfaceNormalAuto,
            true,
            false,
            3.0f,
            2.0f,
            8.0f,
            8.0f,
            true);
        ok &= expectFloat("surface alignment rejects owner mismatch",
            static_cast<float>(rejectedOwner),
            static_cast<float>(GrabSurfaceAlignmentDecision::RejectedOwnerMismatch));

        {
            const TestVector rawPivot{ 6.0f, -2.0f, -0.2f };
            const TestVector rawPalmNormal{ 0.0f, -1.0f, 0.0f };
            const TestVector rawFingerAxis{ 1.0f, 0.0f, 0.0f };
            const TestVector contactWorld{ 100.0f, 50.0f, 25.0f };
            const TestVector surfaceNormalWorld{ 0.0f, 0.0f, 1.0f };
            const TestVector surfaceTangentWorld{ 0.0f, 1.0f, 0.0f };

            const auto handTarget = rock::hand_visual_authority_math::buildHandBoneWorldFromContactFrame<TestTransform>(
                rawPivot,
                rawPalmNormal,
                rawFingerAxis,
                contactWorld,
                surfaceNormalWorld,
                surfaceTangentWorld,
                1.0f);

            const auto solvedPivot = rock::transform_math::localPointToWorld(handTarget, rawPivot);
            const auto solvedPalmNormal = rock::hand_visual_authority_math::normalizeOrZero(
                rock::transform_math::localVectorToWorld(handTarget, rawPalmNormal));
            const auto solvedFingerAxis = rock::hand_visual_authority_math::normalizeOrZero(
                rock::transform_math::localVectorToWorld(handTarget, rawFingerAxis));

            ok &= expectFloat("visual hand target pivots calibrated palm x", solvedPivot.x, contactWorld.x);
            ok &= expectFloat("visual hand target pivots calibrated palm y", solvedPivot.y, contactWorld.y);
            ok &= expectFloat("visual hand target pivots calibrated palm z", solvedPivot.z, contactWorld.z);
            ok &= expectFloat("visual hand target palm normal opposes surface x", solvedPalmNormal.x, 0.0f);
            ok &= expectFloat("visual hand target palm normal opposes surface y", solvedPalmNormal.y, 0.0f);
            ok &= expectFloat("visual hand target palm normal opposes surface z", solvedPalmNormal.z, -1.0f);
            ok &= expectFloat("visual hand target finger axis follows tangent x", solvedFingerAxis.x, 0.0f);
            ok &= expectFloat("visual hand target finger axis follows tangent y", solvedFingerAxis.y, 1.0f);
            ok &= expectFloat("visual hand target finger axis follows tangent z", solvedFingerAxis.z, 0.0f);
        }

        {
            const TestVector rawPivot{ 2.0f, 0.0f, 0.0f };
            const auto handTarget = rock::hand_visual_authority_math::buildHandBoneWorldFromContactFrame<TestTransform>(
                rawPivot,
                TestVector{ 0.0f, -1.0f, 0.0f },
                TestVector{ 1.0f, 0.0f, 0.0f },
                TestVector{ 10.0f, 0.0f, 0.0f },
                TestVector{ 0.0f, 0.0f, 1.0f },
                TestVector{ 0.0f, 1.0f, 0.0f },
                2.0f);
            const auto solvedPivot = rock::transform_math::localPointToWorld(handTarget, rawPivot);
            ok &= expectFloat("visual hand target honors hand scale in pivot solve", solvedPivot.x, 10.0f);
        }

        const auto capFrame = buildSurfaceFrameFromTriangle(v0, v1, v2, TestVector{ 0.0f, 1.0f, 0.0f }, TestVector{ 1.0f, 0.0f, 0.0f }, 0.85f, true);
        ok &= expectFloat("surface cap face kind", static_cast<float>(capFrame.faceKind), static_cast<float>(GrabSurfaceFaceKind::CapTopBottom));
        ok &= expectFloat("surface cap keeps preserved tangent", static_cast<float>(capFrame.tangentSource), static_cast<float>(GrabSurfaceTangentSource::PreservedObjectRoll));

        const auto collisionNormalFrame = buildSurfaceFrameFromNormal(
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            TestVector{ 0.0f, 1.0f, 0.0f });
        ok &= expectFloat("collision normal frame normal x", collisionNormalFrame.normal.x, 1.0f);
        ok &= expectFloat("collision normal frame tangent y", collisionNormalFrame.tangent.y, 1.0f);
        ok &= expectFloat("collision normal frame source", static_cast<float>(collisionNormalFrame.tangentSource), static_cast<float>(GrabSurfaceTangentSource::ObjectLongAxis));
        ok &= expectFloat("collision normal frame confidence", collisionNormalFrame.confidence, 1.0f);

        const auto invalidCollisionNormalFrame = buildSurfaceFrameFromNormal(
            TestVector{ 0.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            TestVector{ 0.0f, 1.0f, 0.0f });
        ok &= expectFloat("invalid collision normal frame ambiguous", static_cast<float>(invalidCollisionNormalFrame.faceKind), static_cast<float>(GrabSurfaceFaceKind::Ambiguous));

        const auto ambiguousFrame =
            buildSurfaceFrameFromTriangle(TestVector{ 0.0f, 0.0f, 0.0f }, TestVector{ 0.0f, 0.0f, 0.0f }, TestVector{ 0.0f, 0.0f, 0.0f }, TestVector{ 0.0f, 1.0f, 0.0f }, TestVector{ 1.0f, 0.0f, 0.0f });
        const auto ambiguousDesired = buildDesiredObjectWorldFromSurfaceFrame(objectWorld,
            localSurfacePoint,
            ambiguousFrame,
            grabPivotWorld,
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 1.0f, 0.0f },
            GrabOrientationMode::SurfaceNormalAuto);
        ok &= expectFloat("ambiguous frame kind", static_cast<float>(ambiguousFrame.faceKind), static_cast<float>(GrabSurfaceFaceKind::Ambiguous));
        ok &= expectFloat("ambiguous preserves mode", static_cast<float>(ambiguousDesired.modeUsed), static_cast<float>(GrabOrientationMode::PreserveObjectRotation));

        const auto mirroredRight = buildDesiredObjectWorldFromSurfaceFrame(objectWorld,
            localSurfacePoint,
            sideFrame,
            grabPivotWorld,
            TestVector{ 0.0f, 0.0f, -1.0f },
            TestVector{ 0.0f, 1.0f, 0.0f },
            GrabOrientationMode::SurfaceNormalAuto);
        const auto mirroredLeft = buildDesiredObjectWorldFromSurfaceFrame(objectWorld,
            localSurfacePoint,
            sideFrame,
            grabPivotWorld,
            TestVector{ 0.0f, 0.0f, -1.0f },
            TestVector{ 0.0f, -1.0f, 0.0f },
            GrabOrientationMode::SurfaceNormalAuto);
        const auto rightNormal = rock::transform_math::rotateLocalVectorToWorld(mirroredRight.transform.rotate, sideFrame.normal);
        const auto leftNormal = rock::transform_math::rotateLocalVectorToWorld(mirroredLeft.transform.rotate, sideFrame.normal);
        ok &= expectFloat("right mirrored normal z", rightNormal.z, 1.0f);
        ok &= expectFloat("left mirrored normal z", leftNormal.z, 1.0f);
    }

    {
        using namespace rock::grab_contact_patch_math;

        std::vector<GrabContactPatchSample<TestVector>> samples{
            GrabContactPatchSample<TestVector>{ .bodyId = 10, .point = TestVector{ -2.0f, -1.0f, 0.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
            GrabContactPatchSample<TestVector>{ .bodyId = 10, .point = TestVector{ 2.0f, -1.0f, 0.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
            GrabContactPatchSample<TestVector>{ .bodyId = 10, .point = TestVector{ 0.0f, 3.0f, 0.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
        };
        const auto patch = fitContactPatch(samples, TestVector{ 0.0f, 0.0f, 5.0f }, TestVector{ 0.0f, 0.0f, 1.0f }, TestVector{ 1.0f, 0.0f, 0.0f }, 35.0f);
        ok &= expectFloat("contact patch valid", patch.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("contact patch coherent hit count", static_cast<float>(patch.hitCount), 3.0f);
        ok &= expectFloat("contact patch normal z", patch.normal.z, -1.0f);
        ok &= expectFloat("contact patch point z", patch.contactPoint.z, 0.0f);
        ok &= expectFloat("contact patch tangent y prefers longest span", std::fabs(patch.tangent.y), 1.0f);
        ok &= expectFloat("contact patch orientation reliable", patch.orientationReliable ? 1.0f : 0.0f, 1.0f);

        samples.push_back(GrabContactPatchSample<TestVector>{
            .bodyId = 10,
            .point = TestVector{ 50.0f, 50.0f, 50.0f },
            .normal = TestVector{ 1.0f, 0.0f, 0.0f },
            .accepted = true,
        });
        const auto outlierPatch = fitContactPatch(samples, TestVector{ 0.0f, 0.0f, 5.0f }, TestVector{ 0.0f, 0.0f, 1.0f }, TestVector{ 1.0f, 0.0f, 0.0f }, 35.0f);
        ok &= expectFloat("contact patch rejects normal outlier", static_cast<float>(outlierPatch.rejectedSampleCount), 1.0f);
        ok &= expectFloat("contact patch keeps coherent samples", static_cast<float>(outlierPatch.hitCount), 3.0f);
        ok &= expectFloat("contact patch outlier normal z", outlierPatch.normal.z, -1.0f);

        const std::vector<GrabContactPatchSample<TestVector>> twoHitSamples{
            GrabContactPatchSample<TestVector>{ .bodyId = 11, .point = TestVector{ -4.0f, 0.0f, 0.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
            GrabContactPatchSample<TestVector>{ .bodyId = 11, .point = TestVector{ 4.0f, 0.0f, 0.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
        };
        const auto twoHitPatch = fitContactPatch(twoHitSamples, TestVector{ 0.0f, 2.0f, 5.0f }, TestVector{ 0.0f, 0.0f, 1.0f }, TestVector{ 0.0f, 1.0f, 0.0f }, 35.0f);
        ok &= expectFloat("two-hit patch valid", twoHitPatch.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("two-hit patch tangent x", std::fabs(twoHitPatch.tangent.x), 1.0f);
        ok &= expectFloat("two-hit patch orientation reliable", twoHitPatch.orientationReliable ? 1.0f : 0.0f, 1.0f);

        const std::vector<GrabContactPatchSample<TestVector>> singleHitSample{
            GrabContactPatchSample<TestVector>{ .bodyId = 12, .point = TestVector{ 1.0f, 2.0f, 3.0f }, .normal = TestVector{ 0.0f, 0.0f, -1.0f }, .accepted = true },
        };
        const auto singleHitPatch = fitContactPatch(singleHitSample, TestVector{ 1.0f, 2.0f, 7.0f }, TestVector{ 0.0f, 0.0f, 1.0f }, TestVector{ 1.0f, 0.0f, 0.0f }, 35.0f);
        ok &= expectFloat("single-hit patch keeps pivot evidence", singleHitPatch.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("single-hit patch orientation not reliable", singleHitPatch.orientationReliable ? 1.0f : 0.0f, 0.0f);

        const auto singleHitPivot = chooseContactPatchPivotPoint(singleHitPatch,
            singleHitSample,
            TestVector{ 10.0f, 20.0f, 30.0f },
            true,
            TestVector{},
            false,
            8.0f);
        ok &= expectFloat("single-hit patch does not replace selected pivot", singleHitPivot.replaceSelectedPoint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("single-hit patch keeps selected pivot x", singleHitPivot.point.x, 10.0f);

        const auto farSamplePivot = chooseContactPatchPivotPoint(twoHitPatch,
            twoHitSamples,
            TestVector{ 100.0f, 0.0f, 0.0f },
            true,
            TestVector{},
            false,
            8.0f);
        ok &= expectFloat("far patch samples do not replace selected pivot", farSamplePivot.replaceSelectedPoint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("far patch samples keep selected pivot x", farSamplePivot.point.x, 100.0f);

        const auto meshSnapPivot = chooseContactPatchPivotPoint(twoHitPatch,
            twoHitSamples,
            TestVector{ 0.0f, 0.0f, 0.0f },
            true,
            TestVector{ 0.5f, 0.0f, 0.0f },
            true,
            8.0f);
        ok &= expectFloat("mesh snap replaces selected pivot", meshSnapPivot.replaceSelectedPoint ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("mesh snap pivot x", meshSnapPivot.point.x, 0.5f);

        const auto samplePivot = chooseContactPatchPivotPoint(twoHitPatch,
            twoHitSamples,
            TestVector{ 0.0f, 0.0f, 0.0f },
            true,
            TestVector{},
            false,
            8.0f);
        ok &= expectFloat("two-hit sample replaces selected pivot with real sample", samplePivot.replaceSelectedPoint ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("two-hit sample pivot is real sample x", std::fabs(samplePivot.point.x), 4.0f);

        ok &= expectFloat("patch normal matches selected normal",
            contactPatchNormalMatchesSelection(patch, TestVector{ 0.0f, 0.0f, -1.0f }, true, TestVector{ 0.0f, 0.0f, 1.0f }, 35.0f) ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("patch normal rejects different selected normal",
            contactPatchNormalMatchesSelection(patch, TestVector{ 1.0f, 0.0f, 0.0f }, true, TestVector{ 0.0f, 0.0f, 1.0f }, 35.0f) ? 1.0f : 0.0f,
            0.0f);

        TestTransform bodyWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        bodyWorld.translate = TestVector{ 100.0f, 20.0f, -10.0f };
        bodyWorld.scale = 2.0f;
        const TestVector surfaceWorld{ 104.0f, 24.0f, -6.0f };
        const auto frozenPivotB = freezePivotBBodyLocal(bodyWorld, surfaceWorld);
        ok &= expectFloat("frozen pivot b local x", frozenPivotB.x, 2.0f);
        ok &= expectFloat("frozen pivot b local y", frozenPivotB.y, 2.0f);
        ok &= expectFloat("frozen pivot b local z", frozenPivotB.z, 2.0f);

        TestTransform rotatedHandSpace = rock::transform_math::makeIdentityTransform<TestTransform>();
        rotatedHandSpace.rotate = makeNonSymmetricRotation();
        rotatedHandSpace.translate = TestVector{ 3.0f, 4.0f, 5.0f };
        const auto stillFrozen = freezePivotBBodyLocal(bodyWorld, surfaceWorld);
        ok &= expectFloat("frozen pivot b ignores hand rotation x", stillFrozen.x, frozenPivotB.x);
        ok &= expectFloat("frozen pivot b ignores hand rotation y", stillFrozen.y, frozenPivotB.y);
        ok &= expectFloat("frozen pivot b ignores hand rotation z", stillFrozen.z, frozenPivotB.z);
    }

    {
        using namespace rock::grab_contact_source_policy;

        const auto meshOnlyMissing = evaluateGrabContactSourcePolicy(true, true, false, false);
        ok &= expectFloat("mesh-only disallows collision grab point", meshOnlyMissing.allowCollisionGrabPoint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("mesh-only allows contact patch probing", meshOnlyMissing.allowContactPatchPivot ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("mesh-only contact patch must mesh snap", meshOnlyMissing.requireContactPatchMeshSnap ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("mesh-only require mesh fails without mesh", meshOnlyMissing.failWithoutMesh ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("mesh-only missing reason", meshOnlyMissing.reason, "meshContactRequired");

        const auto meshOnlyFound = evaluateGrabContactSourcePolicy(true, true, true, false);
        ok &= expectFloat("mesh-only accepts mesh contact", meshOnlyFound.failWithoutMesh ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("mesh-only still disallows collision with mesh", meshOnlyFound.allowCollisionGrabPoint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("mesh-only rejects unsnapped contact patch pivot",
            shouldAcceptContactPatchPivot(meshOnlyFound, true, false) ? 1.0f : 0.0f,
            0.0f);
        ok &= expectFloat("mesh-only accepts mesh-snapped contact patch pivot",
            shouldAcceptContactPatchPivot(meshOnlyFound, true, true) ? 1.0f : 0.0f,
            1.0f);

        const auto authoredNode = evaluateGrabContactSourcePolicy(true, true, false, true);
        ok &= expectFloat("mesh-only accepts authored grab node", authoredNode.failWithoutMesh ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("mesh-only authored reason", authoredNode.reason, "authoredGrabNode");

        const auto legacy = evaluateGrabContactSourcePolicy(false, false, false, false);
        ok &= expectFloat("legacy allows collision grab point", legacy.allowCollisionGrabPoint ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("legacy allows contact patch pivot", legacy.allowContactPatchPivot ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("legacy accepts valid unsnapped contact patch pivot",
            shouldAcceptContactPatchPivot(legacy, true, false) ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("mesh-only rejects owner mismatch",
            shouldRejectMeshOwnerMismatch(true, true, true, false, false) ? 1.0f : 0.0f,
            1.0f);
        ok &= expectFloat("authored node bypasses owner mismatch rejection",
            shouldRejectMeshOwnerMismatch(true, true, true, true, false) ? 1.0f : 0.0f,
            0.0f);
    }

    const TestVector palmNormal{ 0.0f, 0.0f, 1.0f };
    const auto rightCloseNormal = rock::handspace_convention::authoredToRawForHand(palmNormal, false);
    const auto leftCloseNormal = rock::handspace_convention::authoredToRawForHand(palmNormal, true);
    ok &= expectFloat("rightCloseNormal.x", rightCloseNormal.x, 0.0f);
    ok &= expectFloat("rightCloseNormal.y", rightCloseNormal.y, 1.0f);
    ok &= expectFloat("rightCloseNormal.z", rightCloseNormal.z, 0.0f);
    ok &= expectFloat("leftCloseNormal.x", leftCloseNormal.x, 0.0f);
    ok &= expectFloat("leftCloseNormal.y", leftCloseNormal.y, 1.0f);
    ok &= expectFloat("leftCloseNormal.z", leftCloseNormal.z, 0.0f);

    const TestVector farDirection{ 0.25f, -0.5f, 1.0f };
    const auto normalFarDirection = rock::pointing_direction_math::applyFarGrabNormalReversal(farDirection, false);
    ok &= expectFloat("normalFarDirection.x", normalFarDirection.x, 0.25f);
    ok &= expectFloat("normalFarDirection.y", normalFarDirection.y, -0.5f);
    ok &= expectFloat("normalFarDirection.z", normalFarDirection.z, 1.0f);

    const auto reversedFarDirection = rock::pointing_direction_math::applyFarGrabNormalReversal(farDirection, true);
    ok &= expectFloat("reversedFarDirection.x", reversedFarDirection.x, -0.25f);
    ok &= expectFloat("reversedFarDirection.y", reversedFarDirection.y, 0.5f);
    ok &= expectFloat("reversedFarDirection.z", reversedFarDirection.z, -1.0f);

    TestMatrix rootRotation{};
    rootRotation.entry[0][1] = -1.0f;
    rootRotation.entry[1][0] = 1.0f;
    rootRotation.entry[2][2] = 1.0f;
    const TestVector weaponRoot{ 10.0f, 20.0f, 30.0f };
    const TestVector localBarrel{ 5.0f, 0.0f, 0.0f };
    const auto barrelWorld = rock::weapon_collision_geometry_math::localPointToWorld(rootRotation, weaponRoot, 2.0f, localBarrel);
    ok &= expectFloat("weaponPackageWorld.x", barrelWorld.x, 10.0f);
    ok &= expectFloat("weaponPackageWorld.y", barrelWorld.y, 10.0f);
    ok &= expectFloat("weaponPackageWorld.z", barrelWorld.z, 30.0f);

    const auto barrelLocalRoundTrip = rock::weapon_collision_geometry_math::worldPointToLocal(rootRotation, weaponRoot, 2.0f, barrelWorld);
    ok &= expectFloat("weaponPackageRoundTrip.x", barrelLocalRoundTrip.x, 5.0f);
    ok &= expectFloat("weaponPackageRoundTrip.y", barrelLocalRoundTrip.y, 0.0f);
    ok &= expectFloat("weaponPackageRoundTrip.z", barrelLocalRoundTrip.z, 0.0f);

    const auto generatedPackageRotation = rootRotation;
    ok &= expectFloat("weaponPackageRotation[0][1]", generatedPackageRotation.entry[0][1], -1.0f);
    ok &= expectFloat("weaponPackageRotation[1][0]", generatedPackageRotation.entry[1][0], 1.0f);
    ok &= expectFloat("weaponPackageRotation[2][2]", generatedPackageRotation.entry[2][2], 1.0f);

    std::vector<TestVector> linePoints;
    for (int i = 0; i < 300; ++i) {
        linePoints.push_back(TestVector{ static_cast<float>(i), 0.0f, 0.0f });
    }
    const auto reducedLine = rock::weapon_collision_geometry_math::limitPointCloud(linePoints, 64);
    ok &= expectFloat("weaponReducedLineCount", static_cast<float>(reducedLine.size()), 64.0f);
    ok &= expectFloat("weaponReducedLineMin", reducedLine.front().x, 0.0f);
    ok &= expectFloat("weaponReducedLineMax", reducedLine.back().x, 299.0f);

    std::vector<rock::weapon_collision_geometry_math::HullSelectionInput> generatedHullInputs{
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
    const auto balancedHulls = rock::weapon_collision_geometry_math::selectBalancedHullIndices(generatedHullInputs, 6);
    ok &= expectFloat("weaponBalancedHullCount", static_cast<float>(balancedHulls.size()), 6.0f);
    ok &= expectContainsIndex("weaponBalancedHullKeepsRearStock", balancedHulls, 5);
    ok &= expectContainsIndex("weaponBalancedHullKeepsFrontBarrel", balancedHulls, 4);
    ok &= expectContainsIndex("weaponBalancedHullKeepsReceiver", balancedHulls, 6);
    ok &= expectContainsIndex("weaponBalancedHullKeepsMagazine", balancedHulls, 7);
    ok &= expectContainsIndex("weaponBalancedHullKeepsTop", balancedHulls, 8);

    rock::hand_collision_suppression_math::SuppressionState collisionState{};
    constexpr std::uint32_t activeFilter = 0x000B002B;
    constexpr std::uint32_t bodyId = 42;
    const auto suppressedFilter = rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, activeFilter);
    ok &= expectFloat("suppression active", collisionState.active ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("suppression bodyId", static_cast<float>(collisionState.bodyId), static_cast<float>(bodyId));
    ok &= expectFloat("suppressed bit", (suppressedFilter & rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 1.0f);

    const auto repeatedFilter = rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, suppressedFilter);
    ok &= expectFloat("repeat keeps original disabled state", collisionState.wasNoCollideBeforeSuppression ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("repeat filter stable", static_cast<float>(repeatedFilter), static_cast<float>(suppressedFilter));

    const auto restoredFilter = rock::hand_collision_suppression_math::restoreFilter(collisionState, repeatedFilter);
    ok &= expectFloat("restore clears bit", (restoredFilter & rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("restore preserves layer/group", static_cast<float>(restoredFilter), static_cast<float>(activeFilter));

    rock::hand_collision_suppression_math::clear(collisionState);
    constexpr std::uint32_t preDisabledFilter = activeFilter | rock::hand_collision_suppression_math::kNoCollideBit;
    rock::hand_collision_suppression_math::beginSuppression(collisionState, bodyId, preDisabledFilter);
    ok &= expectFloat("pre-disabled recorded", collisionState.wasNoCollideBeforeSuppression ? 1.0f : 0.0f, 1.0f);
    const auto restoredPreDisabledFilter = rock::hand_collision_suppression_math::restoreFilter(collisionState, preDisabledFilter);
    ok &= expectFloat("restore keeps pre-disabled bit", (restoredPreDisabledFilter & rock::hand_collision_suppression_math::kNoCollideBit) ? 1.0f : 0.0f, 1.0f);

    rock::hand_collision_suppression_math::DelayedRestoreState delayedRestore{};
    const bool delayScheduled = rock::hand_collision_suppression_math::beginDelayedRestore(delayedRestore, collisionState, 0.10f);
    ok &= expectFloat("release collision delay scheduled", delayScheduled ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("release collision delay tracks body", static_cast<float>(delayedRestore.bodyId), static_cast<float>(bodyId));
    ok &= expectFloat("release collision delay waits before expiry",
        rock::hand_collision_suppression_math::advanceDelayedRestore(delayedRestore, collisionState, 0.05f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("release collision delay remaining", delayedRestore.remainingSeconds, 0.05f);
    ok &= expectFloat("release collision delay ready after expiry",
        rock::hand_collision_suppression_math::advanceDelayedRestore(delayedRestore, collisionState, 0.05f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("invalid release collision delay does not schedule",
        rock::hand_collision_suppression_math::beginDelayedRestore(delayedRestore, collisionState, -1.0f) ? 1.0f : 0.0f,
        0.0f);

    rock::hand_collision_suppression_math::SuppressionSet<4> collisionSet{};
    const auto firstSetSuppress = rock::hand_collision_suppression_math::beginSuppression(collisionSet, 101, activeFilter);
    const auto secondSetSuppress = rock::hand_collision_suppression_math::beginSuppression(collisionSet, 202, preDisabledFilter);
    ok &= expectFloat("suppression set tracks first body", firstSetSuppress.stored ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("suppression set tracks second body", secondSetSuppress.stored ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("suppression set active count", static_cast<float>(rock::hand_collision_suppression_math::activeCount(collisionSet)), 2.0f);
    ok &= expectFloat("suppression set first body restore clears bit",
        (rock::hand_collision_suppression_math::restoreFilter(collisionSet, 101, firstSetSuppress.disabledFilter) &
            rock::hand_collision_suppression_math::kNoCollideBit)
            ? 1.0f
            : 0.0f,
        0.0f);
    ok &= expectFloat("suppression set second body restore keeps bit",
        (rock::hand_collision_suppression_math::restoreFilter(collisionSet, 202, secondSetSuppress.disabledFilter) &
            rock::hand_collision_suppression_math::kNoCollideBit)
            ? 1.0f
            : 0.0f,
        1.0f);
    rock::hand_collision_suppression_math::DelayedRestoreState delayedSetRestore{};
    ok &= expectFloat("suppression set delayed restore schedules",
        rock::hand_collision_suppression_math::beginDelayedRestore(delayedSetRestore, collisionSet, 0.10f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("suppression set delayed restore tracks count", static_cast<float>(delayedSetRestore.bodyCount), 2.0f);

    {
        using rock::active_grab_body_lifecycle::BodyLifecycleRecord;
        using rock::active_grab_body_lifecycle::BodyLifecycleSnapshot;
        using rock::active_grab_body_lifecycle::BodyRestorePolicy;
        using rock::active_grab_body_lifecycle::BodyRestoreReason;
        using rock::active_grab_body_lifecycle::MotionRole;
        using rock::active_grab_body_lifecycle::makeLifecycleAudit;

        BodyLifecycleSnapshot lifecycle{};
        lifecycle.capture(BodyLifecycleRecord{
            .bodyId = 501,
            .motionIndex = 11,
            .motionPropertiesId = 2,
            .bodyFlags = 0x00000006,
            .filterInfo = 0x000B002B,
            .ownerKey = 0xAA01,
            .motionRole = MotionRole::SystemOwnedNonDynamic,
            .acceptedBeforePrep = true,
        });
        lifecycle.capture(BodyLifecycleRecord{
            .bodyId = 502,
            .motionIndex = 12,
            .motionPropertiesId = 1,
            .bodyFlags = 0x00000002,
            .filterInfo = 0x000B002C,
            .ownerKey = 0xAA02,
            .motionRole = MotionRole::LooseDynamic,
            .acceptedBeforePrep = true,
        });
        lifecycle.markPreparedBody(501, true);
        lifecycle.markPreparedBody(502, true);

        const auto complexRelease = lifecycle.restorePlanForRelease(BodyRestorePolicy::ProtectComplexSystemOwned);
        ok &= expectFloat("lifecycle release restores complex body count", static_cast<float>(complexRelease.motionRestoreCount), 1.0f);
        ok &= expectFloat("lifecycle release restores filters for all captured bodies", static_cast<float>(complexRelease.filterRestoreCount), 2.0f);
        ok &= expectFloat(
            "lifecycle release keeps originally dynamic motion", complexRelease.shouldRestoreMotion(502) ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat(
            "lifecycle release restores system-owned non-dynamic motion", complexRelease.shouldRestoreMotion(501) ? 1.0f : 0.0f, 1.0f);

        const auto failedSetup = lifecycle.restorePlanForFailure();
        ok &= expectFloat("lifecycle failed setup restores every changed motion", static_cast<float>(failedSetup.motionRestoreCount), 2.0f);
        ok &= expectFloat(
            "lifecycle failed setup reason", static_cast<float>(failedSetup.reason), static_cast<float>(BodyRestoreReason::FailedGrabSetup));

        const auto audit = makeLifecycleAudit(lifecycle, complexRelease, 501);
        ok &= expectFloat("lifecycle audit body count", static_cast<float>(audit.bodyCount), 2.0f);
        ok &= expectFloat("lifecycle audit converted count", static_cast<float>(audit.convertedCount), 2.0f);
        ok &= expectFloat("lifecycle audit primary body", static_cast<float>(audit.primaryBodyId), 501.0f);
        ok &= expectFloat("lifecycle audit reports no verified damping snapshots", static_cast<float>(audit.dampingSnapshotCount), 0.0f);
        ok &= expectFloat("lifecycle audit reports no verified inertia snapshots", static_cast<float>(audit.inertiaSnapshotCount), 0.0f);

        const auto restoreCommands = lifecycle.makeMotionRestoreCommands(complexRelease);
        ok &= expectFloat("lifecycle restore commands are per restored body", static_cast<float>(restoreCommands.size()), 1.0f);
        ok &= expectFloat("lifecycle restore command targets owner key", static_cast<float>(restoreCommands[0].ownerKey), static_cast<float>(0xAA01));
        ok &= expectFloat("lifecycle restore command does not recurse", restoreCommands[0].recursive ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("lifecycle restore command uses classified motion", static_cast<float>(restoreCommands[0].motionType),
            static_cast<float>(rock::physics_body_classifier::BodyMotionType::Keyframed));
    }

    {
        using rock::collision_suppression_registry::CollisionSuppressionOwner;
        using rock::collision_suppression_registry::PureCollisionSuppressionRegistry;
        using rock::collision_suppression_registry::kSuppressionNoCollideBit;

        PureCollisionSuppressionRegistry registry{};
        const auto grabAcquire = registry.acquire(901, CollisionSuppressionOwner::Grab, activeFilter);
        const auto weaponAcquire = registry.acquire(901, CollisionSuppressionOwner::WeaponDominantHand, grabAcquire.filterAfter);
        const auto repeatedGrabAcquire = registry.acquire(901, CollisionSuppressionOwner::Grab, weaponAcquire.filterAfter);
        ok &= expectFloat("suppression registry first lease changes filter", grabAcquire.filterChanged ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("suppression registry keeps original pre-disabled state", grabAcquire.wasNoCollideBeforeSuppression ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("suppression registry second owner does not overwrite original filter", weaponAcquire.firstLeaseForBody ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("suppression registry repeated owner acquire is idempotent", repeatedGrabAcquire.ownerAlreadyHeld ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("suppression registry repeated owner acquire does not change filter", repeatedGrabAcquire.filterChanged ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("suppression registry active lease count", static_cast<float>(registry.activeLeaseCount(901)), 2.0f);
        const auto releaseGrab = registry.release(901, CollisionSuppressionOwner::Grab, weaponAcquire.filterAfter);
        ok &= expectFloat("suppression registry releasing one owner keeps body suppressed", releaseGrab.bodyFullyReleased ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("suppression registry filter still disabled with one owner",
            (releaseGrab.filterAfter & kSuppressionNoCollideBit) ? 1.0f : 0.0f,
            1.0f);
        const auto releaseWeapon = registry.release(901, CollisionSuppressionOwner::WeaponDominantHand, releaseGrab.filterAfter);
        ok &= expectFloat("suppression registry last owner releases body", releaseWeapon.bodyFullyReleased ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("suppression registry final filter restores enabled",
            (releaseWeapon.filterAfter & kSuppressionNoCollideBit) ? 1.0f : 0.0f,
            0.0f);

        registry.acquire(902, CollisionSuppressionOwner::WeaponSupportHand, preDisabledFilter);
        const auto releasePreDisabled = registry.release(902, CollisionSuppressionOwner::WeaponSupportHand, preDisabledFilter);
        ok &= expectFloat("suppression registry preserves pre-disabled bit",
            (releasePreDisabled.filterAfter & kSuppressionNoCollideBit) ? 1.0f : 0.0f,
            1.0f);
    }

    {
        using rock::held_player_space_registry::HeldMotionSample;
        using rock::held_player_space_registry::HeldPlayerSpaceRegistry;
        using rock::held_player_space_registry::PureVec3;
        using rock::held_player_space_registry::WriterKind;

        HeldPlayerSpaceRegistry registry{};
        registry.beginFrame(PureVec3{ 1.0f, 0.0f, 0.0f }, PureVec3{ 0.25f, 0.0f, 0.0f }, 0.50f);
        registry.registerBody(1001, 31);
        registry.registerBody(1002, 31);
        registry.registerBody(1003, 32);

        const auto body1001 = registry.solveBodyVelocity(
            HeldMotionSample{ .bodyId = 1001, .motionIndex = 31, .linearVelocity = PureVec3{ 2.25f, 0.0f, 0.0f }, .angularVelocity = PureVec3{ 0.0f, 4.0f, 0.0f } });
        const auto body1002 = registry.solveBodyVelocity(
            HeldMotionSample{ .bodyId = 1002, .motionIndex = 31, .linearVelocity = PureVec3{ 9.0f, 0.0f, 0.0f }, .angularVelocity = PureVec3{ 0.0f, 9.0f, 0.0f } });
        const auto body1003 = registry.solveBodyVelocity(
            HeldMotionSample{ .bodyId = 1003, .motionIndex = 32, .linearVelocity = PureVec3{ 0.25f, 4.0f, 0.0f }, .angularVelocity = PureVec3{ 0.0f, 0.0f, 6.0f } });

        ok &= expectFloat("held player-space first motion writes", body1001.shouldWrite ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("held player-space duplicate motion skipped", body1002.shouldWrite ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("held player-space preserves local x", body1001.linearVelocity.x, 2.0f);
        ok &= expectFloat("held player-space composes y residual", body1003.linearVelocity.y, 2.0f);
        ok &= expectFloat("held player-space damps angular velocity", body1003.angularVelocity.z, 3.0f);
        registry.recordWriter(WriterKind::ConstraintTarget);
        registry.recordWriter(WriterKind::PlayerSpaceCentral);
        ok &= expectFloat("held player-space accepted writer mask", registry.writerMaskIsSteadyStateExpected() ? 1.0f : 0.0f, 1.0f);
        registry.recordWriter(WriterKind::HeldLoopVelocity);
        ok &= expectFloat("held player-space rejects held-loop writer mask", registry.writerMaskIsSteadyStateExpected() ? 1.0f : 0.0f, 0.0f);

        HeldPlayerSpaceRegistry ccWriterRegistry{};
        ccWriterRegistry.beginFrame(PureVec3{}, PureVec3{}, 1.0f);
        ccWriterRegistry.recordWriter(WriterKind::ConstraintTarget);
        ccWriterRegistry.recordWriter(WriterKind::PlayerSpaceCentral);
        ccWriterRegistry.recordWriter(WriterKind::CharacterControllerHook);
        ok &= expectFloat("held player-space rejects character-controller hook writer mask", ccWriterRegistry.writerMaskIsSteadyStateExpected() ? 1.0f : 0.0f, 0.0f);

        using rock::held_grab_cc_policy::HeldGrabContactPolicyInput;
        using rock::held_grab_cc_policy::HeldGrabContactIntervention;
        using rock::held_grab_cc_policy::filterGeneratedContactBuffers;
        using rock::held_grab_cc_policy::makeGeneratedContactBufferView;
        using rock::held_grab_cc_policy::evaluateHeldGrabContactPolicy;

        const auto normalContactPolicy = evaluateHeldGrabContactPolicy(HeldGrabContactPolicyInput{
            .hooksEnabled = true,
            .holdingHeldObject = true,
            .diagnosticsEnabled = false,
        });
        ok &= expectFloat("held grab CC policy disables post-constraint mutation", normalContactPolicy.mayMutateGeneratedConstraint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("held grab CC policy normal runtime filters before original", normalContactPolicy.mayFilterBeforeOriginal ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("held grab CC policy normal runtime does not inspect", normalContactPolicy.mayInspectGeneratedConstraint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("held grab CC policy normal runtime pre-original intervention",
            static_cast<float>(normalContactPolicy.intervention),
            static_cast<float>(HeldGrabContactIntervention::PreOriginalFilter));
        ok &= expectString("held grab CC policy normal reason", normalContactPolicy.reason, "preOriginalHeldBodyFilter");

        const auto diagnosticContactPolicy = evaluateHeldGrabContactPolicy(HeldGrabContactPolicyInput{
            .hooksEnabled = true,
            .holdingHeldObject = true,
            .diagnosticsEnabled = true,
        });
        ok &= expectFloat("held grab CC policy diagnostic path can inspect", diagnosticContactPolicy.mayInspectGeneratedConstraint ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("held grab CC policy diagnostic path still filters before original", diagnosticContactPolicy.mayFilterBeforeOriginal ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("held grab CC policy diagnostic path cannot mutate", diagnosticContactPolicy.mayMutateGeneratedConstraint ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("held grab CC policy diagnostic intervention",
            static_cast<float>(diagnosticContactPolicy.intervention),
            static_cast<float>(HeldGrabContactIntervention::PreOriginalFilter));
        ok &= expectString("held grab CC policy diagnostic reason", diagnosticContactPolicy.reason, "preOriginalHeldBodyFilter");

        ok &= expectFloat("held grab CC buffers reject null manifold", makeGeneratedContactBufferView(nullptr, reinterpret_cast<void*>(0x1000)).valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("held grab CC buffers null manifold reason", makeGeneratedContactBufferView(nullptr, reinterpret_cast<void*>(0x1000)).reason, "missingManifold");
        ok &= expectFloat("held grab CC buffers reject null simplex", makeGeneratedContactBufferView(reinterpret_cast<void*>(0x1000), nullptr).valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("held grab CC buffers null simplex reason", makeGeneratedContactBufferView(reinterpret_cast<void*>(0x1000), nullptr).reason, "missingSimplexInput");

        alignas(void*) char generatedManifoldMissingRows[sizeof(void*) * 2]{};
        alignas(void*) char generatedEntriesMissingRows[0x40]{};
        alignas(void*) char generatedSimplexMissingRows[0x58]{};
        *reinterpret_cast<char**>(generatedManifoldMissingRows) = generatedEntriesMissingRows;
        *reinterpret_cast<int*>(generatedManifoldMissingRows + sizeof(void*)) = 1;
        *reinterpret_cast<int*>(generatedSimplexMissingRows + 0x50) = 1;
        const auto missingConstraintRows = makeGeneratedContactBufferView(generatedManifoldMissingRows, generatedSimplexMissingRows);
        ok &= expectFloat("held grab CC buffers reject missing constraint rows", missingConstraintRows.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("held grab CC buffers missing constraint row reason", missingConstraintRows.reason, "missingConstraintEntries");

        alignas(void*) char generatedManifold[sizeof(void*) * 2]{};
        alignas(void*) char generatedEntries[0xC0]{};
        alignas(void*) char generatedSimplex[0x58]{};
        alignas(void*) char generatedConstraints[0xC0]{};
        *reinterpret_cast<char**>(generatedManifold) = generatedEntries;
        *reinterpret_cast<int*>(generatedManifold + sizeof(void*)) = 3;
        *reinterpret_cast<char**>(generatedSimplex + 0x48) = generatedConstraints;
        *reinterpret_cast<int*>(generatedSimplex + 0x50) = 3;
        *reinterpret_cast<std::uint32_t*>(generatedEntries + 0x00 * 0x40 + 0x28) = 10;
        *reinterpret_cast<std::uint32_t*>(generatedEntries + 0x01 * 0x40 + 0x28) = 20;
        *reinterpret_cast<std::uint32_t*>(generatedEntries + 0x02 * 0x40 + 0x28) = 30;
        *reinterpret_cast<std::uint32_t*>(generatedConstraints + 0x00 * 0x40 + 0x30) = 100;
        *reinterpret_cast<std::uint32_t*>(generatedConstraints + 0x01 * 0x40 + 0x30) = 200;
        *reinterpret_cast<std::uint32_t*>(generatedConstraints + 0x02 * 0x40 + 0x30) = 300;
        const auto generatedBuffers = makeGeneratedContactBufferView(generatedManifold, generatedSimplex);
        ok &= expectFloat("held grab CC buffers accept verified layout", generatedBuffers.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("held grab CC buffers clamp pair count", static_cast<float>(generatedBuffers.pairCount), 3.0f);
        ok &= expectString("held grab CC buffers valid reason", generatedBuffers.reason, "ok");
        const auto generatedFilter = filterGeneratedContactBuffers(generatedBuffers, [](std::uint32_t bodyId) {
            return bodyId == 20;
        });
        ok &= expectFloat("held grab CC filter removes held contact", static_cast<float>(generatedFilter.removedPairCount), 1.0f);
        ok &= expectFloat("held grab CC filter compacts manifold count", static_cast<float>(*reinterpret_cast<int*>(generatedManifold + sizeof(void*))), 2.0f);
        ok &= expectFloat("held grab CC filter compacts constraint count", static_cast<float>(*reinterpret_cast<int*>(generatedSimplex + 0x50)), 2.0f);
        ok &= expectFloat("held grab CC filter keeps first body", static_cast<float>(*reinterpret_cast<std::uint32_t*>(generatedEntries + 0x00 * 0x40 + 0x28)), 10.0f);
        ok &= expectFloat("held grab CC filter moves third body down", static_cast<float>(*reinterpret_cast<std::uint32_t*>(generatedEntries + 0x01 * 0x40 + 0x28)), 30.0f);
        ok &= expectFloat("held grab CC filter keeps matching first row", static_cast<float>(*reinterpret_cast<std::uint32_t*>(generatedConstraints + 0x00 * 0x40 + 0x30)), 100.0f);
        ok &= expectFloat("held grab CC filter moves third row down", static_cast<float>(*reinterpret_cast<std::uint32_t*>(generatedConstraints + 0x01 * 0x40 + 0x30)), 300.0f);
        ok &= expectFloat("held player-space does not carry previous velocity without registered bodies",
            rock::held_player_space_registry::shouldCarryPreviousPlayerVelocity(true, false, 0) ? 1.0f : 0.0f,
            0.0f);
        ok &= expectFloat("held player-space carries previous velocity with written bodies",
            rock::held_player_space_registry::shouldCarryPreviousPlayerVelocity(true, false, 2) ? 1.0f : 0.0f,
            1.0f);
    }

    {
        ok &= expectFloat("nearby damping runtime writes are not verified",
            rock::nearby_grab_damping::runtimeDampingWritesVerified() ? 1.0f : 0.0f,
            0.0f);
        ok &= expectFloat("nearby damping skips scan without verified writer",
            rock::nearby_grab_damping::shouldBeginRuntimeNearbyDamping(true, false, 12.0f, 0.25f) ? 1.0f : 0.0f,
            0.0f);
        ok &= expectFloat("nearby damping would scan with verified writer",
            rock::nearby_grab_damping::shouldBeginRuntimeNearbyDamping(true, true, 12.0f, 0.25f) ? 1.0f : 0.0f,
            1.0f);
    }

    {
        using rock::geometry_body_resolver::GeometryBodyResolutionInput;
        using rock::geometry_body_resolver::GeometryBodyResolutionSource;
        using rock::geometry_body_resolver::resolveGeometryBody;
        GeometryBodyResolutionInput bodyResolution{};
        bodyResolution.selectedBodyId = 10;
        bodyResolution.nearestBodyId = 11;
        bodyResolution.triangleOwnerBodyId = 12;
        bodyResolution.authoredBodyId = 13;
        bodyResolution.authoredUsable = true;
        bodyResolution.triangleOwnerUsable = true;
        const auto authoredResolution = resolveGeometryBody(bodyResolution);
        ok &= expectFloat("geometry resolver authored node wins",
            static_cast<float>(authoredResolution.source),
            static_cast<float>(GeometryBodyResolutionSource::AuthoredNode));
        ok &= expectFloat("geometry resolver authored body id", static_cast<float>(authoredResolution.bodyId), 13.0f);
        bodyResolution.authoredUsable = false;
        const auto triangleResolution = resolveGeometryBody(bodyResolution);
        ok &= expectFloat("geometry resolver triangle owner second",
            static_cast<float>(triangleResolution.source),
            static_cast<float>(GeometryBodyResolutionSource::TriangleOwner));
        bodyResolution.triangleOwnerUsable = false;
        const auto selectedResolution = resolveGeometryBody(bodyResolution);
        ok &= expectFloat("geometry resolver selected body fallback",
            static_cast<float>(selectedResolution.source),
            static_cast<float>(GeometryBodyResolutionSource::SelectedBody));
        ok &= expectString("geometry resolver dynamic skinned disabled reason",
            rock::geometry_body_resolver::dynamicSkinnedResolutionFallbackReason(false),
            "dynamicSkinnedWeightsUnverified");
    }

    {
        using rock::weapon_authority_lifecycle_policy::GeneratedCollisionRebuildInput;
        using rock::weapon_authority_lifecycle_policy::GeneratedCollisionRebuildAction;
        using rock::weapon_authority_lifecycle_policy::evaluateGeneratedCollisionRebuild;
        const auto keepExisting = evaluateGeneratedCollisionRebuild(GeneratedCollisionRebuildInput{
            .hasExistingBodies = true,
            .cachedKey = 100,
            .currentKey = 200,
            .settingsChanged = false,
            .generatedSourceCount = 0,
            .hasBuildableSource = false,
        });
        ok &= expectFloat("weapon rebuild keeps existing without replacement source", keepExisting.keepExistingBodies ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("weapon rebuild does not destroy before source", keepExisting.destroyExistingBeforeCreate ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("weapon rebuild remains pending", keepExisting.pending ? 1.0f : 0.0f, 1.0f);
        const auto replaceExisting = evaluateGeneratedCollisionRebuild(GeneratedCollisionRebuildInput{
            .hasExistingBodies = true,
            .cachedKey = 100,
            .currentKey = 200,
            .settingsChanged = false,
            .generatedSourceCount = 3,
            .hasBuildableSource = true,
        });
        ok &= expectFloat("weapon rebuild replaces only with buildable source",
            static_cast<float>(replaceExisting.action),
            static_cast<float>(GeneratedCollisionRebuildAction::ReplaceExisting));
        ok &= expectFloat("weapon rebuild does not destroy before replacement is built", replaceExisting.destroyExistingBeforeCreate ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("weapon rebuild builds replacement before active bank release", replaceExisting.createReplacementBeforeDestroy ? 1.0f : 0.0f, 1.0f);

        const auto createInitialAfterKeyCached = evaluateGeneratedCollisionRebuild(GeneratedCollisionRebuildInput{
            .hasExistingBodies = false,
            .cachedKey = 200,
            .currentKey = 200,
            .settingsChanged = false,
            .generatedSourceCount = 3,
            .hasBuildableSource = true,
        });
        ok &= expectFloat("weapon rebuild creates initial bodies even after key cache is primed",
            static_cast<float>(createInitialAfterKeyCached.action),
            static_cast<float>(GeneratedCollisionRebuildAction::CreateInitial));
        ok &= expectFloat("weapon rebuild initial create is not left pending", createInitialAfterKeyCached.pending ? 1.0f : 0.0f, 0.0f);
    }

    ok &= expectFloat("debug hand draw enabled",
        rock::debug_overlay_policy::shouldDrawHandBody(true, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug hand draw disabled",
        rock::debug_overlay_policy::shouldDrawHandBody(true, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug weapon index inside cap",
        rock::debug_overlay_policy::shouldDrawWeaponBody(true, true, 5, 6) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug weapon index outside cap",
        rock::debug_overlay_policy::shouldDrawWeaponBody(true, true, 6, 6) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug shape generation clamp low", static_cast<float>(rock::debug_overlay_policy::clampShapeGenerationsPerFrame(-2)), 0.0f);
    ok &= expectFloat("debug shape generation clamp high", static_cast<float>(rock::debug_overlay_policy::clampShapeGenerationsPerFrame(99)), 32.0f);
    ok &= expectFloat("debug max convex vertex clamp low", static_cast<float>(rock::debug_overlay_policy::clampMaxConvexSupportVertices(0)), 4.0f);
    ok &= expectFloat("debug max convex vertex clamp high", static_cast<float>(rock::debug_overlay_policy::clampMaxConvexSupportVertices(999)), 256.0f);
    ok &= expectFloat("debug heavy convex uses bounds",
        rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(192, 64, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("debug small convex keeps detail",
        rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(32, 64, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("debug heavy convex bounds disabled",
        rock::debug_overlay_policy::shouldUseBoundsForHeavyConvex(192, 64, false) ? 1.0f : 0.0f,
        0.0f);

    ok &= expectFloat("weapon collision body cap", static_cast<float>(rock::MAX_WEAPON_COLLISION_BODIES), 100.0f);
    {
        using namespace rock::weapon_visual_composition_policy;

        std::uint64_t keyA = kWeaponVisualCompositionOffset;
        mixVisualRecord(keyA,
            VisualRecord{
                .nodeAddress = 0x1000,
                .parentAddress = 0x2000,
                .name = "P-Barrel",
                .depth = 2,
                .childIndex = 1,
                .childCount = 0,
                .visible = true,
                .triShape = true,
                .rendererData = 0x3000,
                .skinInstance = 0,
                .vertexBlock = 0x4000,
                .triangleBlock = 0x5000,
                .vertexDesc = 0x1234,
                .numTriangles = 20,
                .numVertices = 40,
                .geometryType = 3,
            });

        std::uint64_t keyTransformOnly = kWeaponVisualCompositionOffset;
        mixVisualRecord(keyTransformOnly,
            VisualRecord{
                .nodeAddress = 0x1000,
                .parentAddress = 0x2000,
                .name = "P-Barrel",
                .depth = 2,
                .childIndex = 1,
                .childCount = 0,
                .visible = true,
                .triShape = true,
                .rendererData = 0x3000,
                .skinInstance = 0,
                .vertexBlock = 0x4000,
                .triangleBlock = 0x5000,
                .vertexDesc = 0x1234,
                .numTriangles = 20,
                .numVertices = 40,
                .geometryType = 3,
            });
        ok &= expectFloat("weapon visual key ignores transform-only changes", keyTransformOnly == keyA ? 1.0f : 0.0f, 1.0f);

        std::uint64_t keyHidden = kWeaponVisualCompositionOffset;
        mixVisualRecord(keyHidden,
            VisualRecord{
                .nodeAddress = 0x1000,
                .parentAddress = 0x2000,
                .name = "P-Barrel",
                .depth = 2,
                .childIndex = 1,
                .childCount = 0,
                .visible = false,
                .triShape = true,
                .rendererData = 0x3000,
                .skinInstance = 0,
                .vertexBlock = 0x4000,
                .triangleBlock = 0x5000,
                .vertexDesc = 0x1234,
                .numTriangles = 20,
                .numVertices = 40,
                .geometryType = 3,
            });
        ok &= expectFloat("weapon visual key changes when visibility changes", keyHidden != keyA ? 1.0f : 0.0f, 1.0f);

        std::uint64_t keyRendererChanged = kWeaponVisualCompositionOffset;
        mixVisualRecord(keyRendererChanged,
            VisualRecord{
                .nodeAddress = 0x1000,
                .parentAddress = 0x2000,
                .name = "P-Barrel",
                .depth = 2,
                .childIndex = 1,
                .childCount = 0,
                .visible = true,
                .triShape = true,
                .rendererData = 0x3333,
                .skinInstance = 0,
                .vertexBlock = 0x4444,
                .triangleBlock = 0x5555,
                .vertexDesc = 0x1234,
                .numTriangles = 20,
                .numVertices = 40,
                .geometryType = 3,
            });
        ok &= expectFloat("weapon visual key changes when renderer buffers change", keyRendererChanged != keyA ? 1.0f : 0.0f, 1.0f);

        std::uint64_t keyTriangleCountChanged = kWeaponVisualCompositionOffset;
        mixVisualRecord(keyTriangleCountChanged,
            VisualRecord{
                .nodeAddress = 0x1000,
                .parentAddress = 0x2000,
                .name = "P-Barrel",
                .depth = 2,
                .childIndex = 1,
                .childCount = 0,
                .visible = true,
                .triShape = true,
                .rendererData = 0x3000,
                .skinInstance = 0,
                .vertexBlock = 0x4000,
                .triangleBlock = 0x5000,
                .vertexDesc = 0x1234,
                .numTriangles = 21,
                .numVertices = 40,
                .geometryType = 3,
            });
        ok &= expectFloat("weapon visual key changes when triangle count changes", keyTriangleCountChanged != keyA ? 1.0f : 0.0f, 1.0f);

        VisualSettleState settle{};
        auto settle1 = advanceVisualSettle(settle, 0xAAAA);
        auto settle2 = advanceVisualSettle(settle, 0xAAAA);
        auto settle3 = advanceVisualSettle(settle, 0xAAAA);
        ok &= expectFloat("weapon visual settle waits first frame", settle1.settled ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("weapon visual settle waits second frame", settle2.settled ? 1.0f : 0.0f, 0.0f);
        ok &= expectFloat("weapon visual settle accepts third stable frame", settle3.settled ? 1.0f : 0.0f, 1.0f);

        auto settleReset = advanceVisualSettle(settle, 0xBBBB);
        ok &= expectFloat("weapon visual settle resets when key changes", static_cast<float>(settleReset.stableFrames), 1.0f);
        ok &= expectFloat("weapon visual settle changed key not settled", settleReset.settled ? 1.0f : 0.0f, 0.0f);
    }
    {
        std::vector<rock::WeaponSemanticHullBudgetInput> budgetInputs;
        budgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Barrel), .sourceIndex = 0, .sourceGroupId = 0xB001, .pointCount = 80 });
        budgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Barrel), .sourceIndex = 1, .sourceGroupId = 0xB001, .pointCount = 70 });
        budgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Stock), .sourceIndex = 2, .sourceGroupId = 0xA001, .pointCount = 60 });
        budgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Magazine), .sourceIndex = 3, .sourceGroupId = 0xC001, .pointCount = 50 });
        budgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Sight), .sourceIndex = 4, .sourceGroupId = 0xD001, .pointCount = 200 });
        const auto selected = rock::selectSemanticWeaponHullIndices(budgetInputs, 3);
        ok &= expectFloat("weapon budget keeps one hull per gameplay source group count", static_cast<float>(selected.size()), 3.0f);
        ok &= expectFloat("weapon budget keeps barrel group", std::find(selected.begin(), selected.end(), 0) != selected.end() ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("weapon budget keeps stock group", std::find(selected.begin(), selected.end(), 2) != selected.end() ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("weapon budget keeps magazine group", std::find(selected.begin(), selected.end(), 3) != selected.end() ? 1.0f : 0.0f, 1.0f);
    }
    ok &= expectFloat("weapon collision grouping default is semantic part node",
        static_cast<float>(rock::weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(99)),
        static_cast<float>(rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::SemanticPartNode));
    ok &= expectFloat("weapon collision grouping preserves legacy mode",
        static_cast<float>(rock::weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(0)),
        static_cast<float>(rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::LegacyTriShape));
    ok &= expectFloat("weapon collision grouping preserves semantic mode",
        static_cast<float>(rock::weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(1)),
        static_cast<float>(rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::SemanticPartNode));
    ok &= expectFloat("weapon collision grouping preserves compound mode",
        static_cast<float>(rock::weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(2)),
        static_cast<float>(rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::CompoundSemanticPart));
    ok &= expectString("weapon collision grouping semantic name",
        rock::weapon_collision_grouping_policy::weaponCollisionGroupingModeName(
            rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::SemanticPartNode),
        "SemanticPartNode");
    ok &= expectString("weapon collision grouping compound name",
        rock::weapon_collision_grouping_policy::weaponCollisionGroupingModeName(
            rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::CompoundSemanticPart),
        "CompoundSemanticPart");
    ok &= expectFloat("weapon collision grouping mode change rebuilds",
        rock::weapon_collision_grouping_policy::weaponCollisionGroupingModeChanged(0, 1) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision compound mode change rebuilds",
        rock::weapon_collision_grouping_policy::weaponCollisionGroupingModeChanged(1, 2) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision semantic parent accepts magazine",
        rock::weapon_collision_grouping_policy::startsSemanticWeaponPartGroup(rock::classifyWeaponPartName("WeaponMagazine")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision semantic mode excludes loose rounds",
        rock::weapon_collision_grouping_policy::excludeFromSemanticWeaponCollision(rock::classifyWeaponPartName("LoadedRounds")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision semantic child splits from parent",
        rock::weapon_collision_grouping_policy::semanticChildSplitsFromParent(
            rock::classifyWeaponPartName("WeaponMagazine"),
            rock::classifyWeaponPartName("Cylinder")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision semantic mode still uses package drive root",
        rock::weapon_collision_grouping_policy::usesSingleWeaponPackageDriveRoot(
            rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::SemanticPartNode) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision compound mode still uses package drive root",
        rock::weapon_collision_grouping_policy::usesSingleWeaponPackageDriveRoot(
            rock::weapon_collision_grouping_policy::WeaponCollisionGroupingMode::CompoundSemanticPart) ? 1.0f : 0.0f,
        1.0f);
    const auto compoundMag = rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("Mag:0");
    const auto compoundMagSecond = rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("MagSecond:0");
    const auto compoundExtra = rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("WeaponExtra3");
    ok &= expectFloat("weapon collision compound classifies mk18 mag alias",
        static_cast<float>(compoundMag.partKind),
        static_cast<float>(rock::WeaponPartKind::Magazine));
    ok &= expectFloat("weapon collision compound classifies mk18 second mag alias",
        static_cast<float>(compoundMagSecond.partKind),
        static_cast<float>(rock::WeaponPartKind::Magazine));
    ok &= expectFloat("weapon collision compound treats weapon extra as helper",
        static_cast<float>(compoundExtra.partKind),
        static_cast<float>(rock::WeaponPartKind::Other));
    ok &= expectFloat("weapon collision compound keeps mk18 weapon extra inside magazine",
        rock::weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(
            compoundMag,
            compoundExtra) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon collision compound keeps mk18 second mag inside magazine",
        rock::weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(
            compoundMag,
            compoundMagSecond) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon collision compound merges matching magazine bodies",
        rock::weapon_collision_grouping_policy::compoundSemanticPartsMayShareBody(
            compoundMag,
            compoundMagSecond) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision compound does not merge magazine with cylinder",
        rock::weapon_collision_grouping_policy::compoundSemanticPartsMayShareBody(
            compoundMag,
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("44MagnumCylinder:0")) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon collision compound keeps stock tube inside stock",
        rock::weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("Stock"),
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("StockTube:0")) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon collision compound keeps charging helper inside charging handle",
        rock::weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("ChargingHandle"),
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("WeaponExtra1")) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon collision compound splits revolver cylinder from magazine combo",
        rock::weapon_collision_grouping_policy::compoundSemanticChildSplitsFromParent(
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("WeaponMagazine"),
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("44MagnumCylinder:0")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision compound excludes revolver bullet display",
        rock::weapon_collision_grouping_policy::excludeFromCompoundSemanticWeaponCollision(
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("44MagnumBullets:0")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision compound excludes revolver moon clip display",
        rock::weapon_collision_grouping_policy::excludeFromCompoundSemanticWeaponCollision(
            rock::weapon_collision_grouping_policy::classifyCompoundSemanticWeaponPartName("44MagnumMoonClip:0")) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision candidate scoring penalizes fragmentation",
        rock::weapon_collision_grouping_policy::weaponCollisionCandidateScore(3, 100) >
                rock::weapon_collision_grouping_policy::weaponCollisionCandidateScore(8, 100)
            ? 1.0f
            : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision candidate scoring keeps triangle coverage dominant",
        rock::weapon_collision_grouping_policy::weaponCollisionCandidateScore(8, 200) >
                rock::weapon_collision_grouping_policy::weaponCollisionCandidateScore(1, 100)
            ? 1.0f
            : 0.0f,
        1.0f);
    ok &= expectFloat("weapon collision magazine shell classifies as ammo",
        static_cast<float>(rock::classifyWeaponPartName("MagazineShell01").partKind),
        static_cast<float>(rock::WeaponPartKind::Shell));
    ok &= expectFloat("weapon collision semantic excludes magazine shell ammo",
        rock::weapon_collision_grouping_policy::excludeFromSemanticWeaponCollision(rock::classifyWeaponPartName("MagazineShell01")) ? 1.0f : 0.0f,
        1.0f);

    const auto magazinePart = rock::classifyWeaponPartName("AssaultRifle:MagazineLarge");
    ok &= expectFloat("weapon classifier magazine kind", static_cast<float>(magazinePart.partKind), static_cast<float>(rock::WeaponPartKind::Magazine));
    ok &= expectFloat("weapon classifier magazine reload role", static_cast<float>(magazinePart.reloadRole), static_cast<float>(rock::WeaponReloadRole::MagazineBody));

    const auto magwellPart = rock::classifyWeaponPartName("CombatRifle:MagwellSocket");
    ok &= expectFloat("weapon classifier magwell kind", static_cast<float>(magwellPart.partKind), static_cast<float>(rock::WeaponPartKind::Magwell));
    ok &= expectFloat("weapon classifier magwell socket role", static_cast<float>(magwellPart.socketRole), static_cast<float>(rock::WeaponSocketRole::Magwell));

    const auto handguardPart = rock::classifyWeaponPartName("LaserRifle:Handguard");
    ok &= expectFloat("weapon classifier handguard support", static_cast<float>(handguardPart.supportGripRole), static_cast<float>(rock::WeaponSupportGripRole::SupportSurface));
    ok &= expectFloat("weapon classifier handguard pose", static_cast<float>(handguardPart.fallbackGripPose), static_cast<float>(rock::WeaponGripPoseId::HandguardClamp));

    std::vector<rock::WeaponSemanticHullBudgetInput> semanticBudgetInputs;
    semanticBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Other), .sourceIndex = 0, .pointCount = 300 });
    semanticBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::CosmeticAmmo), .sourceIndex = 1, .pointCount = 900 });
    semanticBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Magazine), .sourceIndex = 2, .pointCount = 20 });
    semanticBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Magwell), .sourceIndex = 3, .pointCount = 25 });
    semanticBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Bolt), .sourceIndex = 4, .pointCount = 15 });
    const auto semanticBudgetIndices = rock::selectSemanticWeaponHullIndices(semanticBudgetInputs, 3);
    ok &= expectFloat("weapon semantic budget count", static_cast<float>(semanticBudgetIndices.size()), 3.0f);
    ok &= expectFloat("weapon semantic budget keeps magwell first", static_cast<float>(semanticBudgetIndices[0]), 3.0f);
    ok &= expectFloat("weapon semantic budget keeps magazine second", static_cast<float>(semanticBudgetIndices[1]), 2.0f);
    ok &= expectFloat("weapon semantic budget keeps bolt third", static_cast<float>(semanticBudgetIndices[2]), 4.0f);

    std::vector<rock::WeaponSemanticHullBudgetInput> duplicateSupportBudgetInputs;
    duplicateSupportBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Handguard), .sourceIndex = 10, .pointCount = 900 });
    duplicateSupportBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Handguard), .sourceIndex = 11, .pointCount = 800 });
    duplicateSupportBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Handguard), .sourceIndex = 12, .pointCount = 700 });
    duplicateSupportBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Magazine), .sourceIndex = 13, .pointCount = 20 });
    duplicateSupportBudgetInputs.push_back({ .semantic = rock::classifyWeaponPartKind(rock::WeaponPartKind::Bolt), .sourceIndex = 14, .pointCount = 15 });
    const auto duplicateSupportBudgetIndices = rock::selectSemanticWeaponHullIndices(duplicateSupportBudgetInputs, 3);
    const auto hasBudgetIndex = [&duplicateSupportBudgetIndices](std::size_t index) {
        return std::find(duplicateSupportBudgetIndices.begin(), duplicateSupportBudgetIndices.end(), index) != duplicateSupportBudgetIndices.end();
    };
    ok &= expectFloat("weapon semantic budget keeps one handguard", (hasBudgetIndex(10) || hasBudgetIndex(11) || hasBudgetIndex(12)) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("weapon semantic budget keeps magazine despite lower priority duplicate", hasBudgetIndex(13) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("weapon semantic budget keeps bolt despite lower priority duplicate", hasBudgetIndex(14) ? 1.0f : 0.0f, 1.0f);

    rock::WeaponInteractionContact handguardContact{};
    handguardContact.valid = true;
    handguardContact.bodyId = 200;
    handguardContact.partKind = handguardPart.partKind;
    handguardContact.supportGripRole = handguardPart.supportGripRole;
    handguardContact.reloadRole = handguardPart.reloadRole;
    handguardContact.socketRole = handguardPart.socketRole;
    handguardContact.actionRole = handguardPart.actionRole;
    const auto supportRoute = rock::routeWeaponInteraction(handguardContact, rock::WeaponInteractionRuntimeState{});
    ok &= expectFloat("weapon router support grip", static_cast<float>(supportRoute.kind), static_cast<float>(rock::WeaponInteractionKind::SupportGrip));

    handguardContact.sourceRoot = reinterpret_cast<RE::NiAVObject*>(0x1000);
    handguardContact.interactionRoot = reinterpret_cast<RE::NiAVObject*>(0x2000);
    handguardContact.weaponGenerationKey = 0x1234;
    const auto rootedSupportRoute = rock::routeWeaponInteraction(handguardContact, rock::WeaponInteractionRuntimeState{});
    ok &= expectFloat(
        "weapon router preserves source root",
        static_cast<float>(reinterpret_cast<std::uintptr_t>(rootedSupportRoute.sourceRoot)),
        static_cast<float>(reinterpret_cast<std::uintptr_t>(handguardContact.sourceRoot)));
    ok &= expectFloat(
        "weapon router preserves interaction root",
        static_cast<float>(reinterpret_cast<std::uintptr_t>(rootedSupportRoute.interactionRoot)),
        static_cast<float>(reinterpret_cast<std::uintptr_t>(handguardContact.interactionRoot)));
    ok &= expectFloat(
        "weapon router preserves generation",
        static_cast<float>(rootedSupportRoute.weaponGenerationKey),
        static_cast<float>(handguardContact.weaponGenerationKey));
    ok &= expectFloat(
        "weapon contact current generation accepted",
        rock::weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handguardContact.weaponGenerationKey, 0x1234) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat(
        "weapon contact stale generation rejected",
        rock::weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handguardContact.weaponGenerationKey, 0x5678) ? 1.0f : 0.0f,
        0.0f);

    rock::WeaponInteractionContact accessoryContact{};
    accessoryContact.valid = true;
    accessoryContact.bodyId = 203;
    accessoryContact.partKind = rock::WeaponPartKind::Accessory;
    const auto accessorySupportRoute = rock::routeWeaponInteraction(accessoryContact, rock::WeaponInteractionRuntimeState{});
    ok &= expectFloat(
        "weapon router generic accessory support grip",
        static_cast<float>(accessorySupportRoute.kind),
        static_cast<float>(rock::WeaponInteractionKind::SupportGrip));
    ok &= expectFloat(
        "weapon router generic accessory support pose",
        static_cast<float>(accessorySupportRoute.gripPose),
        static_cast<float>(rock::WeaponGripPoseId::ReceiverSupport));

    rock::WeaponInteractionContact otherPartContact{};
    otherPartContact.valid = true;
    otherPartContact.bodyId = 204;
    otherPartContact.partKind = rock::WeaponPartKind::Other;
    const auto otherSupportRoute = rock::routeWeaponInteraction(otherPartContact, rock::WeaponInteractionRuntimeState{});
    ok &= expectFloat(
        "weapon router generic unknown support grip",
        static_cast<float>(otherSupportRoute.kind),
        static_cast<float>(rock::WeaponInteractionKind::SupportGrip));
    ok &= expectFloat(
        "weapon router generic unknown support pose",
        static_cast<float>(otherSupportRoute.gripPose),
        static_cast<float>(rock::WeaponGripPoseId::BarrelWrap));

    ok &= expectString("weapon debug part name", rock::weapon_debug_notification_policy::nameOf(rock::WeaponPartKind::Handguard), "Handguard");
    ok &= expectString(
        "weapon debug interaction name",
        rock::weapon_debug_notification_policy::nameOf(rock::WeaponInteractionKind::SupportGrip),
        "SupportGrip");
    rock::weapon_debug_notification_policy::WeaponNotificationState weaponNotificationState{};
    const auto firstWeaponNotificationKey = rock::weapon_debug_notification_policy::makeWeaponNotificationKey(
        handguardContact,
        supportRoute,
        rock::weapon_debug_notification_policy::WeaponContactSource::Probe);
    ok &= expectFloat(
        "weapon debug first contact notifies",
        rock::weapon_debug_notification_policy::shouldNotifyWeaponContact(weaponNotificationState, firstWeaponNotificationKey) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat(
        "weapon debug repeated contact suppressed",
        rock::weapon_debug_notification_policy::shouldNotifyWeaponContact(weaponNotificationState, firstWeaponNotificationKey) ? 1.0f : 0.0f,
        0.0f);

    rock::WeaponInteractionContact barrelContact = handguardContact;
    barrelContact.partKind = rock::WeaponPartKind::Barrel;
    barrelContact.fallbackGripPose = rock::WeaponGripPoseId::BarrelWrap;
    const auto barrelRoute = rock::routeWeaponInteraction(barrelContact, rock::WeaponInteractionRuntimeState{});
    const auto changedWeaponNotificationKey = rock::weapon_debug_notification_policy::makeWeaponNotificationKey(
        barrelContact,
        barrelRoute,
        rock::weapon_debug_notification_policy::WeaponContactSource::Probe);
    ok &= expectFloat(
        "weapon debug changed contact notifies",
        rock::weapon_debug_notification_policy::shouldNotifyWeaponContact(weaponNotificationState, changedWeaponNotificationKey) ? 1.0f : 0.0f,
        1.0f);
    rock::WeaponInteractionDebugInfo weaponDebugInfo{};
    weaponDebugInfo.weaponName = "Hunting Rifle";
    weaponDebugInfo.weaponFormId = 0x0004F46A;
    weaponDebugInfo.weaponNodeName = "Weapon";
    weaponDebugInfo.sourceRootName = "firstPersonSkeleton:Weapon";
    weaponDebugInfo.sourceName = "MagazineLarge";
    ok &= expectFloat(
        "weapon debug grip start",
        static_cast<float>(rock::weapon_debug_notification_policy::observeWeaponSupportGrip(weaponNotificationState, true)),
        static_cast<float>(rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::Started));
    ok &= expectFloat(
        "weapon debug repeated grip active",
        static_cast<float>(rock::weapon_debug_notification_policy::observeWeaponSupportGrip(weaponNotificationState, true)),
        static_cast<float>(rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::None));
    ok &= expectString(
        "weapon debug grip start message",
        rock::weapon_debug_notification_policy::formatWeaponGripNotification(
            rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::Started,
            changedWeaponNotificationKey,
            weaponDebugInfo),
        "[ROCK] WPN GRIP START weapon='Hunting Rifle' form=0004F46A node='Weapon' root='firstPersonSkeleton:Weapon' nif='MagazineLarge' part=Barrel route=SupportGrip pose=BarrelWrap body=200");
    ok &= expectString(
        "weapon debug grip start fallback message",
        rock::weapon_debug_notification_policy::formatWeaponGripNotification(
            rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::Started,
            changedWeaponNotificationKey,
            rock::WeaponInteractionDebugInfo{}),
        "[ROCK] WPN GRIP START weapon='(unknown)' form=00000000 node='(unknown)' root='(unknown)' nif='(unknown)' part=Barrel route=SupportGrip pose=BarrelWrap body=200");
    ok &= expectFloat(
        "weapon debug grip end",
        static_cast<float>(rock::weapon_debug_notification_policy::observeWeaponSupportGrip(weaponNotificationState, false)),
        static_cast<float>(rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::Ended));
    ok &= expectString(
        "weapon debug grip end message",
        rock::weapon_debug_notification_policy::formatWeaponGripNotification(
            rock::weapon_debug_notification_policy::WeaponGripNotificationEvent::Ended,
            changedWeaponNotificationKey),
        "[ROCK] WPN GRIP END");

    const float probeInside = rock::weapon_interaction_probe_math::pointAabbDistanceSquared(
        TestVector{ 1.0f, 2.0f, 3.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 2.0f, 4.0f, 6.0f });
    ok &= expectFloat("weapon interaction probe point inside aabb", probeInside, 0.0f);
    const float probeOutside = rock::weapon_interaction_probe_math::pointAabbDistanceSquared(
        TestVector{ 5.0f, 2.0f, 3.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 2.0f, 4.0f, 6.0f });
    ok &= expectFloat("weapon interaction probe point outside aabb", probeOutside, 9.0f);
    ok &= expectFloat("weapon interaction probe radius check",
        rock::weapon_interaction_probe_math::isWithinProbeRadiusSquared(probeOutside, 3.0f) ? 1.0f : 0.0f,
        1.0f);

    const TestMatrix identity = rock::transform_math::makeIdentityRotation<TestMatrix>();
    TestTransform twoHandWeapon{};
    twoHandWeapon.rotate = identity;
    twoHandWeapon.scale = 1.0f;
    twoHandWeapon.translate = TestVector{ 3.0f, 4.0f, 5.0f };
    rock::WeaponTwoHandedSolverInput<TestTransform, TestVector> solverInput{};
    solverInput.weaponWorldTransform = twoHandWeapon;
    solverInput.primaryGripLocal = TestVector{ 0.5f, 0.0f, 0.0f };
    solverInput.supportGripLocal = TestVector{ 0.5f, 2.0f, 0.0f };
    solverInput.primaryTargetWorld = TestVector{ 10.0f, 20.0f, 30.0f };
    solverInput.supportTargetWorld = TestVector{ 16.0f, 20.0f, 30.0f };

    const auto solvedTwoHand = rock::solveTwoHandedWeaponTransform(solverInput);
    ok &= expectFloat("two handed solver solved", solvedTwoHand.solved ? 1.0f : 0.0f, 1.0f);
    const auto solvedPrimaryWorld = rock::transform_math::localPointToWorld(solvedTwoHand.weaponWorldTransform, solverInput.primaryGripLocal);
    const auto solvedSupportWorld = rock::transform_math::localPointToWorld(solvedTwoHand.weaponWorldTransform, solverInput.supportGripLocal);
    ok &= expectFloat("two handed solver primary x", solvedPrimaryWorld.x, 10.0f);
    ok &= expectFloat("two handed solver primary y", solvedPrimaryWorld.y, 20.0f);
    ok &= expectFloat("two handed solver primary z", solvedPrimaryWorld.z, 30.0f);

    TestTransform twoHandTwistWeapon{};
    twoHandTwistWeapon.rotate = identity;
    twoHandTwistWeapon.scale = 1.0f;
    twoHandTwistWeapon.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    rock::WeaponTwoHandedSolverInput<TestTransform, TestVector> twistSolverInput{};
    twistSolverInput.weaponWorldTransform = twoHandTwistWeapon;
    twistSolverInput.primaryGripLocal = TestVector{ 0.0f, 0.0f, 0.0f };
    twistSolverInput.supportGripLocal = TestVector{ 2.0f, 0.0f, 0.0f };
    twistSolverInput.primaryTargetWorld = TestVector{ 0.0f, 0.0f, 0.0f };
    twistSolverInput.supportTargetWorld = TestVector{ 2.0f, 0.0f, 0.0f };
    twistSolverInput.supportNormalLocal = TestVector{ 0.0f, 0.0f, 1.0f };
    twistSolverInput.supportNormalTargetWorld = TestVector{ 0.0f, 1.0f, 0.0f };
    twistSolverInput.useSupportNormalTwist = true;
    twistSolverInput.supportNormalTwistFactor = 1.0f;
    const auto solvedTwist = rock::solveTwoHandedWeaponTransform(twistSolverInput);
    const auto solvedTwistNormal = rock::transform_math::localVectorToWorld(solvedTwist.weaponWorldTransform, twistSolverInput.supportNormalLocal);
    ok &= expectFloat("two handed solver twist solved", solvedTwist.solved ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("two handed solver twists support normal x", solvedTwistNormal.x, 0.0f);
    ok &= expectFloat("two handed solver twists support normal y", solvedTwistNormal.y, 1.0f);
    ok &= expectFloat("two handed solver twists support normal z", solvedTwistNormal.z, 0.0f);
    ok &= expectFloat("two handed solver support x", solvedSupportWorld.x, 12.0f);
    ok &= expectFloat("two handed solver support y", solvedSupportWorld.y, 20.0f);
    ok &= expectFloat("two handed solver support z", solvedSupportWorld.z, 30.0f);

    TestTransform frikStyleWeapon{};
    frikStyleWeapon.rotate = identity;
    frikStyleWeapon.scale = 1.0f;
    frikStyleWeapon.translate = TestVector{ 4.0f, 5.0f, 6.0f };
    rock::WeaponTwoHandedSolverInput<TestTransform, TestVector> frikStyleInput{};
    frikStyleInput.weaponWorldTransform = frikStyleWeapon;
    frikStyleInput.primaryGripLocal = TestVector{ 0.0f, 0.0f, 0.0f };
    frikStyleInput.supportGripLocal = TestVector{ 0.0f, 10.0f, 0.0f };
    frikStyleInput.primaryTargetWorld = TestVector{ 12.0f, 20.0f, 30.0f };
    frikStyleInput.supportTargetWorld = TestVector{ 22.0f, 20.0f, 30.0f };
    const auto frikStyleSolved = rock::solveTwoHandedWeaponTransformFrikPivot(frikStyleInput);
    const auto frikStylePrimaryWorld = rock::transform_math::localPointToWorld(frikStyleSolved.weaponWorldTransform, frikStyleInput.primaryGripLocal);
    const auto frikStyleSupportWorld = rock::transform_math::localPointToWorld(frikStyleSolved.weaponWorldTransform, frikStyleInput.supportGripLocal);
    const auto frikStyleRotatedForward = rock::transform_math::rotateLocalVectorToWorld(frikStyleSolved.rotationDelta, TestVector{ 0.0f, 1.0f, 0.0f });
    ok &= expectFloat("two handed frik-pivot solver solved", frikStyleSolved.solved ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("two handed frik-pivot primary x", frikStylePrimaryWorld.x, 12.0f);
    ok &= expectFloat("two handed frik-pivot primary y", frikStylePrimaryWorld.y, 20.0f);
    ok &= expectFloat("two handed frik-pivot primary z", frikStylePrimaryWorld.z, 30.0f);
    ok &= expectFloat("two handed frik-pivot support x", frikStyleSupportWorld.x, 22.0f);
    ok &= expectFloat("two handed frik-pivot support y", frikStyleSupportWorld.y, 20.0f);
    ok &= expectFloat("two handed frik-pivot support z", frikStyleSupportWorld.z, 30.0f);
    ok &= expectFloat("two handed frik-pivot delta forward x", frikStyleRotatedForward.x, 1.0f);
    ok &= expectFloat("two handed frik-pivot delta forward y", frikStyleRotatedForward.y, 0.0f);
    ok &= expectFloat("two handed frik-pivot delta forward z", frikStyleRotatedForward.z, 0.0f);

    const auto lockedSupportTarget = rock::makeLockedSupportGripTarget(
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 25.0f, 0.0f },
        TestVector{ 0.0f, 10.0f, 0.0f },
        10.0f,
        0.001f);
    ok &= expectFloat("two handed locked support target ignores axial slide x", lockedSupportTarget.x, 0.0f);
    ok &= expectFloat("two handed locked support target ignores axial slide y", lockedSupportTarget.y, 10.0f);
    ok &= expectFloat("two handed locked support target ignores axial slide z", lockedSupportTarget.z, 0.0f);

    const auto lockedSupportPivotTarget = rock::makeLockedSupportGripTarget(
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 10.0f, 10.0f, 0.0f },
        TestVector{ 0.0f, 10.0f, 0.0f },
        10.0f,
        0.001f);
    ok &= expectFloat("two handed locked support target preserves lateral pivot x", lockedSupportPivotTarget.x, 7.0711f);
    ok &= expectFloat("two handed locked support target preserves lateral pivot y", lockedSupportPivotTarget.y, 7.0711f);
    ok &= expectFloat("two handed locked support target preserves lateral pivot z", lockedSupportPivotTarget.z, 0.0f);

    ok &= expectFloat("frik api v9 required", static_cast<float>(frik::api::FRIK_API_VERSION), 9.0f);
    ok &= expectFloat("frik api hand target appended",
        offsetof(frik::api::FRIKApi, applyExternalHandWorldTransform) > offsetof(frik::api::FRIKApi, setHandPoseCustomJointPositionsWithPriority) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("frik api hand target clear appended",
        offsetof(frik::api::FRIKApi, clearExternalHandWorldTransform) > offsetof(frik::api::FRIKApi, applyExternalHandWorldTransform) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("frik api local finger transform appended",
        offsetof(frik::api::FRIKApi, setHandPoseCustomLocalTransformsWithPriority) > offsetof(frik::api::FRIKApi, clearExternalHandWorldTransform) ? 1.0f : 0.0f,
        1.0f);

    TestTransform supportHandFrame{};
    supportHandFrame.rotate = identity;
    supportHandFrame.scale = 1.0f;
    supportHandFrame.translate = TestVector{ 100.0f, 200.0f, 300.0f };
    const auto alignedSupportHandFrame = rock::weapon_two_handed_grip_math::alignHandFrameToGripPoint(
        supportHandFrame,
        TestVector{ 101.0f, 203.0f, 307.0f },
        TestVector{ 110.0f, 220.0f, 340.0f });
    ok &= expectFloat("two handed support hand frame aligned x", alignedSupportHandFrame.translate.x, 109.0f);
    ok &= expectFloat("two handed support hand frame aligned y", alignedSupportHandFrame.translate.y, 217.0f);
    ok &= expectFloat("two handed support hand frame aligned z", alignedSupportHandFrame.translate.z, 333.0f);
    ok &= expectFloat("two handed support can start free hand",
        rock::weapon_two_handed_grip_math::canStartSupportGrip(true, true, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("two handed support cannot start while holding",
        rock::weapon_two_handed_grip_math::canStartSupportGrip(true, true, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("two handed support cannot continue while holding",
        rock::weapon_two_handed_grip_math::shouldContinueSupportGrip(true, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("normal right grab allowed without weapon",
        rock::weapon_two_handed_grip_math::canProcessNormalGrabInput(false, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("normal right grab suppressed with right weapon",
        rock::weapon_two_handed_grip_math::canProcessNormalGrabInput(false, false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("normal left grab suppressed during support grip",
        rock::weapon_two_handed_grip_math::canProcessNormalGrabInput(true, true, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("normal left grab allowed with right weapon only",
        rock::weapon_two_handed_grip_math::canProcessNormalGrabInput(true, false, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("near selected grab not constrained by far range",
        rock::grab_interaction_policy::canAttemptSelectedObjectGrab(false, 900.0f, 350.0f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("far selected grab allowed within configured range",
        rock::grab_interaction_policy::canAttemptSelectedObjectGrab(true, 229.0f, 350.0f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("far selected grab rejected beyond configured range",
        rock::grab_interaction_policy::canAttemptSelectedObjectGrab(true, 351.0f, 350.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("far selected grab rejected with invalid configured range",
        rock::grab_interaction_policy::canAttemptSelectedObjectGrab(true, 20.0f, 0.0f) ? 1.0f : 0.0f,
        0.0f);

    TestTransform weaponParentWorld{};
    weaponParentWorld.rotate = identity;
    weaponParentWorld.scale = 2.0f;
    weaponParentWorld.translate = TestVector{ 10.0f, 20.0f, 30.0f };
    TestTransform targetWeaponWorld{};
    targetWeaponWorld.rotate = makeNonSymmetricRotation();
    targetWeaponWorld.scale = 2.0f;
    targetWeaponWorld.translate = TestVector{ 30.0f, 60.0f, 90.0f };
    const auto weaponParentLocal = rock::weapon_visual_authority_math::worldTargetToParentLocal(weaponParentWorld, targetWeaponWorld);
    const auto recomposedWeaponWorld = rock::transform_math::composeTransforms(weaponParentWorld, weaponParentLocal);
    ok &= expectFloat("weapon visual authority parent local x", recomposedWeaponWorld.translate.x, targetWeaponWorld.translate.x);
    ok &= expectFloat("weapon visual authority parent local y", recomposedWeaponWorld.translate.y, targetWeaponWorld.translate.y);
    ok &= expectFloat("weapon visual authority parent local z", recomposedWeaponWorld.translate.z, targetWeaponWorld.translate.z);
    ok &= expectFloat("weapon visual authority parent local scale", recomposedWeaponWorld.scale, targetWeaponWorld.scale);
    ok &= expectFloat("weapon visual authority parent local rot 00", recomposedWeaponWorld.rotate.entry[0][0], targetWeaponWorld.rotate.entry[0][0]);
    ok &= expectFloat("weapon visual authority parent local rot 01", recomposedWeaponWorld.rotate.entry[0][1], targetWeaponWorld.rotate.entry[0][1]);
    ok &= expectFloat("weapon visual authority parent local rot 10", recomposedWeaponWorld.rotate.entry[1][0], targetWeaponWorld.rotate.entry[1][0]);
    TestTransform lockedHandWeaponLocal{};
    lockedHandWeaponLocal.rotate = identity;
    lockedHandWeaponLocal.scale = 1.0f;
    lockedHandWeaponLocal.translate = TestVector{ 4.0f, -2.0f, 8.0f };
    const auto lockedHandWorld = rock::weapon_visual_authority_math::weaponLocalFrameToWorld(targetWeaponWorld, lockedHandWeaponLocal);
    const auto lockedHandLocalRoundTrip =
        rock::transform_math::composeTransforms(rock::transform_math::invertTransform(targetWeaponWorld), lockedHandWorld);
    ok &= expectFloat("weapon visual authority locked hand local x", lockedHandLocalRoundTrip.translate.x, lockedHandWeaponLocal.translate.x);
    ok &= expectFloat("weapon visual authority locked hand local y", lockedHandLocalRoundTrip.translate.y, lockedHandWeaponLocal.translate.y);
    ok &= expectFloat("weapon visual authority locked hand local z", lockedHandLocalRoundTrip.translate.z, lockedHandWeaponLocal.translate.z);
    ok &= expectFloat("weapon visual authority locked hand ignores controller x", lockedHandWorld.translate.x == 999.0f ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("weapon hand authority publishes pose before locked transform",
        rock::weapon_visual_authority_math::handPosePrecedesLockedHandAuthority() ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon hand authority applies weapon before locked transform",
        rock::weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority() ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon hand authority keeps primary native pose",
        rock::weapon_visual_authority_math::shouldPublishTwoHandedGripPose(rock::weapon_visual_authority_math::LockedHandRole::Primary) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon hand authority publishes support pose",
        rock::weapon_visual_authority_math::shouldPublishTwoHandedGripPose(rock::weapon_visual_authority_math::LockedHandRole::Support) ? 1.0f : 0.0f,
        1.0f);
    const auto sidearmVisualMode = rock::weapon_support_authority_policy::resolveSupportAuthorityMode(
        true,
        rock::weapon_support_authority_policy::WeaponSupportWeaponClass::Sidearm);
    const auto longGunSupportMode = rock::weapon_support_authority_policy::resolveSupportAuthorityMode(
        true,
        rock::weapon_support_authority_policy::WeaponSupportWeaponClass::LongGun);
    ok &= expectFloat("sidearm support visual-only mode selected",
        static_cast<float>(sidearmVisualMode),
        static_cast<float>(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport));
    ok &= expectFloat("long gun support keeps full solver mode",
        static_cast<float>(longGunSupportMode),
        static_cast<float>(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectFloat("visual-only support does not own weapon transform",
        rock::weapon_support_authority_policy::supportGripOwnsWeaponTransform(sidearmVisualMode) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("full support solver owns weapon transform",
        rock::weapon_support_authority_policy::supportGripOwnsWeaponTransform(longGunSupportMode) ? 1.0f : 0.0f,
        1.0f);
    TestTransform visualOnlyWeaponWorldA{};
    visualOnlyWeaponWorldA.rotate = identity;
    visualOnlyWeaponWorldA.scale = 1.0f;
    visualOnlyWeaponWorldA.translate = TestVector{ 10.0f, 20.0f, 30.0f };
    TestTransform visualOnlyWeaponWorldB = visualOnlyWeaponWorldA;
    visualOnlyWeaponWorldB.translate = TestVector{ 25.0f, 18.0f, 34.0f };
    TestTransform supportHandWeaponLocal{};
    supportHandWeaponLocal.rotate = identity;
    supportHandWeaponLocal.scale = 1.0f;
    supportHandWeaponLocal.translate = TestVector{ 1.0f, 2.0f, 3.0f };
    const auto visualSupportA =
        rock::weapon_support_authority_policy::buildVisualOnlySupportHandWorld(visualOnlyWeaponWorldA, supportHandWeaponLocal);
    const auto visualSupportB =
        rock::weapon_support_authority_policy::buildVisualOnlySupportHandWorld(visualOnlyWeaponWorldB, supportHandWeaponLocal);
    ok &= expectFloat("visual-only support follows weapon x", visualSupportA.translate.x, 11.0f);
    ok &= expectFloat("visual-only support follows weapon y", visualSupportA.translate.y, 22.0f);
    ok &= expectFloat("visual-only support follows recoil x", visualSupportB.translate.x, 26.0f);
    ok &= expectFloat("visual-only support follows recoil y", visualSupportB.translate.y, 20.0f);
    rock::weapon_support_authority_policy::EquippedWeaponIdentity pistolGripIdentity{};
    pistolGripIdentity.displayName = "Glock 19x";
    pistolGripIdentity.hasPistolGripKeyword = true;
    ok &= expectFloat("pistol grip keyword selects sidearm support",
        rock::weapon_support_authority_policy::classifyEquippedWeaponForSupportGrip(pistolGripIdentity) ==
                rock::weapon_support_authority_policy::WeaponSupportWeaponClass::Sidearm ?
            1.0f :
            0.0f,
        1.0f);

    rock::weapon_support_authority_policy::EquippedWeaponIdentity instancePistolIdentity{};
    instancePistolIdentity.displayName = "10mm Pistol";
    instancePistolIdentity.hasInstancePistolGripKeyword = true;
    ok &= expectFloat("instance pistol grip keyword selects modded sidearm support",
        rock::weapon_support_authority_policy::classifyEquippedWeaponForSupportGrip(instancePistolIdentity) ==
                rock::weapon_support_authority_policy::WeaponSupportWeaponClass::Sidearm ?
            1.0f :
            0.0f,
        1.0f);

    rock::weapon_support_authority_policy::EquippedWeaponIdentity longGunGripIdentity{};
    longGunGripIdentity.displayName = "Custom Pistol-Caliber Rifle";
    longGunGripIdentity.hasLongGunGripKeyword = true;
    ok &= expectFloat("long gun grip keyword keeps full two handed solver",
        rock::weapon_support_authority_policy::classifyEquippedWeaponForSupportGrip(longGunGripIdentity) ==
                rock::weapon_support_authority_policy::WeaponSupportWeaponClass::LongGun ?
            1.0f :
            0.0f,
        1.0f);

    rock::weapon_support_authority_policy::EquippedWeaponIdentity instanceLongGunGripIdentity{};
    instanceLongGunGripIdentity.displayName = "Converted Shoulder-Fired Pistol";
    instanceLongGunGripIdentity.hasPistolGripKeyword = true;
    instanceLongGunGripIdentity.hasInstanceLongGunGripKeyword = true;
    ok &= expectFloat("instance long gun grip guard wins over pistol grip",
        rock::weapon_support_authority_policy::classifyEquippedWeaponForSupportGrip(instanceLongGunGripIdentity) ==
                rock::weapon_support_authority_policy::WeaponSupportWeaponClass::LongGun ?
            1.0f :
            0.0f,
        1.0f);

    ok &= expectFloat("ambiguous pipe gun without semantic keyword stays unknown",
        rock::weapon_support_authority_policy::classifyEquippedWeaponForSupportGrip({ 0, "Pipe Gun", "" }) ==
                rock::weapon_support_authority_policy::WeaponSupportWeaponClass::Unknown ?
            1.0f :
            0.0f,
        1.0f);
    ok &= expectFloat("weapon hand authority preserves primary grab-start rotation",
        rock::weapon_visual_authority_math::shouldUseMeshGripFrameRotationAtGrabStart(rock::weapon_visual_authority_math::LockedHandRole::Primary) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon hand authority preserves support grab-start rotation",
        rock::weapon_visual_authority_math::shouldUseMeshGripFrameRotationAtGrabStart(rock::weapon_visual_authority_math::LockedHandRole::Support) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon hand authority keeps primary grip point current",
        rock::weapon_visual_authority_math::shouldSelectMeshGripPointAtGrabStart(rock::weapon_visual_authority_math::LockedHandRole::Primary) ? 1.0f : 0.0f,
        0.0f);

    TestTransform projectileWorld{};
    projectileWorld.rotate = makeNonSymmetricRotation();
    projectileWorld.scale = 1.0f;
    projectileWorld.translate = TestVector{ 42.0f, 84.0f, 126.0f };
    const auto muzzleFireLocal = rock::weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(projectileWorld);
    ok &= expectFloat("muzzle authority copies projectile x", muzzleFireLocal.translate.x, projectileWorld.translate.x);
    ok &= expectFloat("muzzle authority copies projectile y", muzzleFireLocal.translate.y, projectileWorld.translate.y);
    ok &= expectFloat("muzzle authority copies projectile z", muzzleFireLocal.translate.z, projectileWorld.translate.z);
    ok &= expectFloat("muzzle authority copies projectile rot 00", muzzleFireLocal.rotate.entry[0][0], projectileWorld.rotate.entry[0][0]);
    ok &= expectFloat("muzzle authority copies projectile rot 01", muzzleFireLocal.rotate.entry[0][1], projectileWorld.rotate.entry[0][1]);
    ok &= expectFloat("muzzle authority copies projectile rot 10", muzzleFireLocal.rotate.entry[1][0], projectileWorld.rotate.entry[1][0]);

    ok &= expectFloat("authority lifecycle clears on menu",
        rock::weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(true, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("authority lifecycle clears on disabled update",
        rock::weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(false, true, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("authority lifecycle keeps normal frame",
        rock::weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(false, false, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held velocity damping clamp low", rock::held_object_damping_math::clampVelocityDamping(-2.0f), 0.0f);
    ok &= expectFloat("held velocity damping clamp high", rock::held_object_damping_math::clampVelocityDamping(1.5f), 1.0f);
    ok &= expectFloat("held velocity damping keep factor", rock::held_object_damping_math::velocityKeepFactor(0.25f), 0.75f);

    const auto dampedVelocity = rock::held_object_damping_math::applyVelocityDamping(TestVector{ 10.0f, -20.0f, 5.0f }, 0.25f);
    ok &= expectFloat("held damped velocity x", dampedVelocity.x, 7.5f);
    ok &= expectFloat("held damped velocity y", dampedVelocity.y, -15.0f);
    ok &= expectFloat("held damped velocity z", dampedVelocity.z, 3.75f);

    const auto playerVelocity = rock::held_object_physics_math::gameUnitsDeltaToHavokVelocity(TestVector{ 70.0f, -140.0f, 35.0f }, 0.5f);
    ok &= expectFloat("held player velocity x", playerVelocity.x, 2.0f);
    ok &= expectFloat("held player velocity y", playerVelocity.y, -4.0f);
    ok &= expectFloat("held player velocity z", playerVelocity.z, 1.0f);

    const auto residualDampedVelocity =
        rock::held_object_physics_math::applyResidualVelocityDamping(TestVector{ 12.0f, 0.0f, 0.0f }, TestVector{ 8.0f, 0.0f, 0.0f }, 0.25f);
    ok &= expectFloat("held residual damping preserves player x", residualDampedVelocity.x, 11.0f);
    ok &= expectFloat("held residual damping preserves player y", residualDampedVelocity.y, 0.0f);
    ok &= expectFloat("held residual damping preserves player z", residualDampedVelocity.z, 0.0f);

    ok &= expectFloat("held player-space warp below threshold",
        rock::held_object_physics_math::shouldWarpPlayerSpaceDelta(TestVector{ 35.0f, 0.0f, 0.0f }, 35.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held player-space warp above threshold",
        rock::held_object_physics_math::shouldWarpPlayerSpaceDelta(TestVector{ 36.0f, 0.0f, 0.0f }, 35.0f) ? 1.0f : 0.0f,
        1.0f);

    TestTransform previousRoom = rock::transform_math::makeIdentityTransform<TestTransform>();
    previousRoom.translate = TestVector{ 100.0f, 0.0f, 0.0f };
    TestTransform currentRoom = rock::transform_math::makeIdentityTransform<TestTransform>();
    currentRoom.translate = TestVector{ 110.0f, 0.0f, 0.0f };
    TestTransform roomBody = rock::transform_math::makeIdentityTransform<TestTransform>();
    roomBody.translate = TestVector{ 105.0f, 2.0f, 3.0f };
    const auto translatedWarp = rock::held_player_space_math::warpBodyWorldThroughPlayerSpace(previousRoom, currentRoom, roomBody);
    ok &= expectFloat("held player-space translation warp x", translatedWarp.translate.x, 115.0f);
    ok &= expectFloat("held player-space translation warp y", translatedWarp.translate.y, 2.0f);
    ok &= expectFloat("held player-space translation warp z", translatedWarp.translate.z, 3.0f);

    previousRoom = rock::transform_math::makeIdentityTransform<TestTransform>();
    currentRoom = rock::transform_math::makeIdentityTransform<TestTransform>();
    currentRoom.rotate = makeNonSymmetricRotation();
    roomBody = rock::transform_math::makeIdentityTransform<TestTransform>();
    roomBody.translate = TestVector{ 5.0f, 0.0f, 0.0f };
    const auto rotatedWarp = rock::held_player_space_math::warpBodyWorldThroughPlayerSpace(previousRoom, currentRoom, roomBody);
    ok &= expectFloat("held player-space rotation warp x", rotatedWarp.translate.x, 0.0f);
    ok &= expectFloat("held player-space rotation warp y", rotatedWarp.translate.y, -5.0f);
    ok &= expectFloat("held player-space rotation angle", rock::held_player_space_math::rotationDeltaDegrees(previousRoom.rotate, currentRoom.rotate), 90.0f);
    ok &= expectFloat("held player-space rotation warp threshold",
        rock::held_player_space_math::shouldWarpPlayerSpaceRotation(previousRoom.rotate, currentRoom.rotate, 0.5f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("held player-space runtime warp requires explicit config",
        rock::held_player_space_math::shouldApplyRuntimeTransformWarp(false, true, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held player-space runtime warp requires diagnostic warp",
        rock::held_player_space_math::shouldApplyRuntimeTransformWarp(true, false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held player-space runtime warp requires transforms",
        rock::held_player_space_math::shouldApplyRuntimeTransformWarp(true, true, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("held player-space runtime warp accepts fully verified gate",
        rock::held_player_space_math::shouldApplyRuntimeTransformWarp(true, true, true) ? 1.0f : 0.0f,
        1.0f);

    const auto releaseVelocity =
        rock::held_object_physics_math::composeReleaseVelocity(TestVector{ 3.0f, 0.0f, 0.0f }, TestVector{ 8.0f, 1.0f, 0.0f }, 1.5f);
    ok &= expectFloat("held release velocity x", releaseVelocity.x, 12.5f);
    ok &= expectFloat("held release velocity y", releaseVelocity.y, 1.0f);
    ok &= expectFloat("held release velocity z", releaseVelocity.z, 0.0f);

    std::array<TestVector, 5> releaseHistory{};
    releaseHistory[0] = TestVector{ 1.0f, 0.0f, 0.0f };
    releaseHistory[1] = TestVector{ 4.0f, 0.0f, 0.0f };
    releaseHistory[2] = TestVector{ 7.0f, 0.0f, 0.0f };
    releaseHistory[3] = TestVector{ 4.0f, 0.0f, 0.0f };
    releaseHistory[4] = TestVector{ 1.0f, 0.0f, 0.0f };
    const auto maxReleaseVelocity = rock::held_object_physics_math::maxMagnitudeVelocity(releaseHistory, 5);
    ok &= expectFloat("held release max history x", maxReleaseVelocity.x, 5.0f);
    ok &= expectFloat("held release max history y", maxReleaseVelocity.y, 0.0f);
    ok &= expectFloat("held release max history z", maxReleaseVelocity.z, 0.0f);

    ok &= expectFloat("held lerp duration min",
        rock::held_object_physics_math::computeHandLerpDuration(7.0f, 0.10f, 0.20f, 7.0f, 14.0f),
        0.10f);
    ok &= expectFloat("held lerp duration midpoint",
        rock::held_object_physics_math::computeHandLerpDuration(10.5f, 0.10f, 0.20f, 7.0f, 14.0f),
        0.15f);
    ok &= expectFloat("held tau advance",
        rock::held_object_physics_math::advanceToward(0.03f, 0.01f, 0.5f, 0.02f),
        0.02f);
    ok &= expectFloat("held angular force ratio",
        rock::held_object_physics_math::angularForceFromRatio(2000.0f, 12.5f),
        160.0f);

    rock::grab_motion_controller::MotorInput quietMotor{};
    quietMotor.enabled = true;
    quietMotor.positionErrorGameUnits = 0.0f;
    quietMotor.rotationErrorDegrees = 0.0f;
    quietMotor.baseLinearTau = 0.03f;
    quietMotor.baseAngularTau = 0.03f;
    quietMotor.collisionTau = 0.01f;
    quietMotor.maxTau = 0.8f;
    quietMotor.currentLinearTau = 0.03f;
    quietMotor.currentAngularTau = 0.03f;
    quietMotor.tauLerpSpeed = 10.0f;
    quietMotor.deltaTime = 0.1f;
    quietMotor.baseMaxForce = 2000.0f;
    quietMotor.mass = 10.0f;
    quietMotor.forceToMassRatio = 500.0f;
    quietMotor.maxForceMultiplier = 4.0f;
    quietMotor.fullPositionErrorGameUnits = 20.0f;
    quietMotor.fullRotationErrorDegrees = 60.0f;
    quietMotor.fadeElapsed = 1.0f;
    quietMotor.fadeDuration = 0.1f;
    quietMotor.angularToLinearForceRatio = 12.5f;
    quietMotor.fadeStartAngularRatio = 100.0f;
    const auto quietMotorResult = rock::grab_motion_controller::solveMotorTargets(quietMotor);
    ok &= expectFloat("grab adaptive quiet force", quietMotorResult.linearMaxForce, 2000.0f);
    ok &= expectFloat("grab adaptive quiet tau", quietMotorResult.linearTau, 0.03f);

    auto noFadeMotor = quietMotor;
    noFadeMotor.fadeInEnabled = false;
    noFadeMotor.fadeElapsed = 0.0f;
    const auto noFadeMotorResult = rock::grab_motion_controller::solveMotorTargets(noFadeMotor);
    ok &= expectFloat("grab no-fade starts with full linear force", noFadeMotorResult.linearMaxForce, 2000.0f);
    ok &= expectFloat("grab no-fade starts with normal angular force", noFadeMotorResult.angularMaxForce, 160.0f);
    ok &= expectFloat("grab no-fade factor is authoritative", noFadeMotorResult.fadeFactor, 1.0f);

    auto fadeMotor = quietMotor;
    fadeMotor.fadeInEnabled = true;
    fadeMotor.fadeElapsed = 0.0f;
    const auto fadeMotorResult = rock::grab_motion_controller::solveMotorTargets(fadeMotor);
    ok &= expectFloat("grab fade starts with zero linear force", fadeMotorResult.linearMaxForce, 0.0f);
    ok &= expectFloat("grab fade starts with zero angular force", fadeMotorResult.angularMaxForce, 0.0f);
    ok &= expectFloat("grab fade starts at zero factor", fadeMotorResult.fadeFactor, 0.0f);

    auto laggingMotor = quietMotor;
    laggingMotor.positionErrorGameUnits = 20.0f;
    laggingMotor.rotationErrorDegrees = 60.0f;
    const auto laggingMotorResult = rock::grab_motion_controller::solveMotorTargets(laggingMotor);
    ok &= expectFloat("grab adaptive lag error factor", laggingMotorResult.errorFactor, 1.0f);
    ok &= expectFloat("grab adaptive lag force mass capped", laggingMotorResult.linearMaxForce, 5000.0f);
    ok &= expectFloat("grab adaptive lag angular force", laggingMotorResult.angularMaxForce, 400.0f);
    ok &= expectFloat("grab adaptive lag tau reaches max", laggingMotorResult.linearTau, 0.8f);

    auto collidingMotor = laggingMotor;
    collidingMotor.heldBodyColliding = true;
    const auto collidingMotorResult = rock::grab_motion_controller::solveMotorTargets(collidingMotor);
    ok &= expectFloat("grab adaptive collision tau min", collidingMotorResult.linearTau, 0.01f);

    TestTransform currentHandVisual{};
    currentHandVisual.rotate = identity;
    currentHandVisual.scale = 1.0f;
    currentHandVisual.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    TestTransform targetHandVisual = currentHandVisual;
    targetHandVisual.translate = TestVector{ 10.0f, 0.0f, 0.0f };
    const auto advancedHandVisual = rock::hand_visual_lerp_math::advanceTransform(currentHandVisual, targetHandVisual, 25.0f, 360.0f, 0.2f);
    ok &= expectFloat("hand visual lerp still advancing", advancedHandVisual.reachedTarget ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("hand visual lerp x", advancedHandVisual.transform.translate.x, 5.0f);

    const auto completedHandVisual =
        rock::hand_visual_lerp_math::advanceTransform(advancedHandVisual.transform, targetHandVisual, 25.0f, 360.0f, 0.2f);
    ok &= expectFloat("hand visual lerp reached", completedHandVisual.reachedTarget ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand visual lerp target x", completedHandVisual.transform.translate.x, 10.0f);

    ok &= expectFloat("grab visual authority disabled by default",
        rock::grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("grab visual authority only applies when explicitly enabled and target is valid",
        rock::grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(true, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("grab visual authority rejects unvalidated contact",
        rock::grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(true, true, false) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("grab visual authority accepts validated contact",
        rock::grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(true, true, true) ? 1.0f : 0.0f,
        1.0f);
    {
        rock::grab_visual_authority_policy::VisualAuthorityContactState authoredState{};
        rock::grab_visual_authority_policy::assignFallbackReasonIfInvalid(authoredState, "meshSurface");
        rock::grab_visual_authority_policy::markValidatedContact(authoredState, "authoredGrabNode");
        ok &= expectFloat("visual authority late authored node validation wins", authoredState.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("visual authority late authored node reason", authoredState.reason, "authoredGrabNode");

        rock::grab_visual_authority_policy::VisualAuthorityContactState contactPatchState{};
        rock::grab_visual_authority_policy::markValidatedContact(contactPatchState, "contactPatchMeshSnap");
        rock::grab_visual_authority_policy::assignFallbackReasonIfInvalid(contactPatchState, "meshSurface");
        ok &= expectFloat("visual authority validated contact patch remains valid", contactPatchState.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("visual authority validated contact patch reason is stable", contactPatchState.reason, "contactPatchMeshSnap");

        rock::grab_visual_authority_policy::VisualAuthorityContactState meshFallbackState{};
        rock::grab_visual_authority_policy::assignFallbackReasonIfInvalid(meshFallbackState, "meshSurface");
        ok &= expectFloat("visual authority mesh fallback remains invalid", meshFallbackState.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("visual authority mesh fallback reason", meshFallbackState.reason, "unvalidatedMeshSurface");
    }
    ok &= expectFloat("grab finger pose uses tracking hand when visual authority is disabled",
        rock::grab_visual_authority_policy::shouldUseObjectReverseAlignedHandForFingerPose(false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("grab finger pose can opt into reverse-aligned visual hand",
        rock::grab_visual_authority_policy::shouldUseObjectReverseAlignedHandForFingerPose(true, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("grab finger pose rejects unvalidated reverse target",
        rock::grab_visual_authority_policy::shouldUseObjectReverseAlignedHandForFingerPose(true, true, false) ? 1.0f : 0.0f,
        0.0f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> fingerTriangles;
    fingerTriangles.push_back({ TestVector{ 4.0f, -1.0f, -1.0f }, TestVector{ 4.0f, 1.0f, -1.0f }, TestVector{ 4.0f, 0.0f, 1.0f } });
    const auto fingerValue = rock::grab_finger_pose_math::solveFingerCurlValue(
        fingerTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        2.0f);
    ok &= expectFloat("finger curl triangle hit", fingerValue.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("finger curl half extended", fingerValue.value, 0.5f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> nearFingerTriangles;
    nearFingerTriangles.push_back({ TestVector{ 4.0f, 0.20f, -1.0f }, TestVector{ 4.0f, 0.20f, 1.0f }, TestVector{ 4.0f, 1.20f, 0.0f } });
    const auto nearFingerValue = rock::grab_finger_pose_math::solveFingerCurlValue(
        nearFingerTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        0.25f);
    ok &= expectFloat("finger curl probe-radius near hit", nearFingerValue.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("finger curl probe-radius distance", nearFingerValue.distance, 4.0f);

    const auto missedFingerValue = rock::grab_finger_pose_math::solveFingerCurlValue(
        fingerTriangles,
        TestVector{ 0.0f, 5.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        8.0f,
        0.2f,
        0.25f);
    ok &= expectFloat("finger curl miss closes", missedFingerValue.hit ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("finger curl miss min", missedFingerValue.value, 0.2f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> candidateTriangles;
    candidateTriangles.push_back({ TestVector{ 1.0f, 0.0f, 0.0f }, TestVector{ 2.0f, 0.0f, 0.0f }, TestVector{ 1.0f, 1.0f, 0.0f } });
    candidateTriangles.push_back({ TestVector{ 20.0f, 0.0f, 0.0f }, TestVector{ 21.0f, 0.0f, 0.0f }, TestVector{ 20.0f, 1.0f, 0.0f } });
    const auto nearbyFingerTriangles =
        rock::grab_finger_pose_math::filterTrianglesNearPoint(candidateTriangles, TestVector{ 0.0f, 0.0f, 0.0f }, 25.0f);
    ok &= expectFloat("finger candidate filtering uses squared distance", static_cast<float>(nearbyFingerTriangles.size()), 1.0f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> curveTriangles;
    curveTriangles.push_back({ TestVector{ 3.0f, 3.0f, -0.25f }, TestVector{ 3.0f, 3.0f, 0.25f }, TestVector{ 2.6f, 3.4f, 0.0f } });
    const auto curveFingerValue = rock::grab_finger_pose_math::solveFingerCurveCurlValue(
        curveTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f);
    ok &= expectFloat("finger curve plane hit", curveFingerValue.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("finger curve contact curls halfway", curveFingerValue.value, 0.5f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> alternateThumbTriangles;
    alternateThumbTriangles.push_back({ TestVector{ 0.0f, -1.0f, -3.0f }, TestVector{ 0.0f, 1.0f, -3.0f }, TestVector{ 1.0f, 0.0f, -3.0f } });
    const auto alternateThumbValue = rock::grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(
        alternateThumbTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f,
        true);
    ok &= expectFloat("thumb alternate curve hit", alternateThumbValue.value.hit ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("thumb alternate curve selected", alternateThumbValue.usedAlternateThumbCurve ? 1.0f : 0.0f, 1.0f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> behindPrimaryAlternateThumbTriangles;
    behindPrimaryAlternateThumbTriangles.push_back(
        { TestVector{ 3.0f, -0.5f, -0.25f }, TestVector{ 3.0f, -0.5f, 0.25f }, TestVector{ 1.0f, 0.5f, -3.0f } });
    const auto behindPrimaryAlternateThumbValue = rock::grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(
        behindPrimaryAlternateThumbTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f,
        true);
    ok &= expectFloat("thumb alternate curve selected when primary behind",
        behindPrimaryAlternateThumbValue.usedAlternateThumbCurve ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("thumb primary behind contact detected",
        behindPrimaryAlternateThumbValue.primary.openedByBehindContact ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("thumb alternate replaces primary open value",
        behindPrimaryAlternateThumbValue.value.value < 1.0f ? 1.0f : 0.0f,
        1.0f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> missedThumbTriangles;
    missedThumbTriangles.push_back({ TestVector{ 3.0f, 3.0f, 3.0f }, TestVector{ 4.0f, 3.0f, 3.0f }, TestVector{ 3.0f, 4.0f, 3.0f } });
    const auto missedThumbValue = rock::grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(
        missedThumbTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f,
        true);
    ok &= expectFloat("thumb alternate curve selected when both curves miss",
        missedThumbValue.usedAlternateThumbCurve ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("thumb both curves miss keeps min curl", missedThumbValue.value.value, 0.2f);

    const auto positivePrimaryThumbValue = rock::grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(
        curveTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f,
        true);
    ok &= expectFloat("thumb alternate curve not selected for positive primary",
        positivePrimaryThumbValue.usedAlternateThumbCurve ? 1.0f : 0.0f,
        0.0f);

    constexpr float nearClosedPrimaryAngle = 1.45f;
    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> nearClosedPrimaryThumbTriangles;
    nearClosedPrimaryThumbTriangles.push_back({
        TestVector{ std::cos(nearClosedPrimaryAngle), std::sin(nearClosedPrimaryAngle), -0.25f },
        TestVector{ std::cos(nearClosedPrimaryAngle), std::sin(nearClosedPrimaryAngle), 0.25f },
        TestVector{ 0.0f, -1.0f, -3.0f },
    });
    const auto nearClosedPrimaryThumbValue = rock::grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(
        nearClosedPrimaryThumbTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f,
        true);
    ok &= expectFloat("thumb positive primary below min keeps primary curve",
        nearClosedPrimaryThumbValue.usedAlternateThumbCurve ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("thumb alternate curve suppresses fallback ray",
        rock::grab_finger_pose_math::shouldRunFallbackRayAfterCurveSolve(0, false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("thumb primary curve miss can use fallback ray",
        rock::grab_finger_pose_math::shouldRunFallbackRayAfterCurveSolve(0, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("non-thumb curve miss can use fallback ray",
        rock::grab_finger_pose_math::shouldRunFallbackRayAfterCurveSolve(1, false, false) ? 1.0f : 0.0f,
        1.0f);

    ok &= expectFloat("alternate thumb local override publishes only solved alternate",
        rock::weapon_support_thumb_pose_policy::shouldPublishAlternateThumbLocalOverride(true, true, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("alternate thumb local override needs alternate curve",
        rock::weapon_support_thumb_pose_policy::shouldPublishAlternateThumbLocalOverride(true, false, true) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("alternate thumb local override needs frik api",
        rock::weapon_support_thumb_pose_policy::shouldPublishAlternateThumbLocalOverride(true, true, false) ? 1.0f : 0.0f,
        0.0f);
    const TestVector predictedThumbNode = rock::weapon_support_thumb_pose_policy::predictThumbNodeWorldForGripFrame(
        TestVector{ 10.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 100.0f, 0.0f, 0.0f });
    const TestVector predictedThumbToGrip = rock::weapon_support_thumb_pose_policy::vectorToGripFromPredictedThumbNode(
        TestVector{ 10.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 100.0f, 0.0f, 0.0f });
    ok &= expectFloat("alternate thumb local frame predicts node with support-hand grip offset", predictedThumbNode.x, 110.0f);
    ok &= expectFloat("alternate thumb local frame targets grip from predicted node", predictedThumbToGrip.x, -10.0f);

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> behindFingerTriangles;
    behindFingerTriangles.push_back({ TestVector{ 3.0f, -0.5f, -0.25f }, TestVector{ 3.0f, -0.5f, 0.25f }, TestVector{ 3.4f, -0.25f, 0.0f } });
    const auto behindFingerValue = rock::grab_finger_pose_math::solveFingerCurveCurlValue(
        behindFingerTriangles,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        1.57079632679f,
        6.0f,
        0.2f);
    ok &= expectFloat("finger curve behind opens hand", behindFingerValue.value, 1.0f);

    const std::array<float, 5> curlValues{ 0.40f, 0.55f, 0.70f, 0.85f, 1.0f };
    const auto jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(curlValues);
    ok &= expectFloat("finger joint thumb proximal keeps knuckle open", jointValues[0], 0.49f);
    ok &= expectFloat("finger joint thumb middle", jointValues[1], 0.40f);
    ok &= expectFloat("finger joint index proximal", jointValues[3], 0.6625f);
    ok &= expectFloat("finger joint index distal closes more", jointValues[5], 0.4825f);
    ok &= expectFloat("finger joint pinky open distal", jointValues[14], 1.0f);

    const std::array<float, 15> previousJointValues{
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
    };
    const auto smoothedJoints = rock::grab_finger_pose_math::advanceJointValues(previousJointValues, jointValues, 2.0f, 0.1f);
    ok &= expectFloat("finger joint smoothing steps toward target", smoothedJoints[1], 0.8f);

    ok &= expectFloat("held deviation below threshold resets",
        rock::held_object_physics_math::advanceDeviationSeconds(1.0f, 49.0f, 50.0f, 0.2f),
        0.0f);
    ok &= expectFloat("held deviation above threshold accumulates",
        rock::held_object_physics_math::advanceDeviationSeconds(1.0f, 60.0f, 50.0f, 0.2f),
        1.2f);
    ok &= expectFloat("held deviation disabled by zero threshold",
        rock::held_object_physics_math::advanceDeviationSeconds(1.0f, 60.0f, 0.0f, 0.2f),
        0.0f);

    std::uint64_t weaponLayerMatrix[48]{};
    for (std::uint32_t i = 0; i < 48; ++i) {
        weaponLayerMatrix[i] = ~0ULL;
    }
    rock::collision_layer_policy::applyWeaponProjectileBlockingPolicy(
        weaponLayerMatrix, rock::collision_layer_policy::ROCK_LAYER_WEAPON, false, false);
    ok &= expectFloat("weapon projectile layer disabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("projectile reciprocal disabled",
        (weaponLayerMatrix[rock::collision_layer_policy::FO4_LAYER_PROJECTILE] & (1ULL << rock::collision_layer_policy::ROCK_LAYER_WEAPON)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon cone projectile layer disabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_CONEPROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon spell layer disabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_SPELL)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon clutter/world layer unchanged",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << 1)) ? 1.0f : 0.0f,
        1.0f);

    rock::collision_layer_policy::applyWeaponProjectileBlockingPolicy(
        weaponLayerMatrix, rock::collision_layer_policy::ROCK_LAYER_WEAPON, true, true);
    ok &= expectFloat("weapon projectile layer enabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon cone projectile layer enabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_CONEPROJECTILE)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon spell layer enabled",
        (weaponLayerMatrix[rock::collision_layer_policy::ROCK_LAYER_WEAPON] & (1ULL << rock::collision_layer_policy::FO4_LAYER_SPELL)) ? 1.0f : 0.0f,
        1.0f);

    ok &= expectFloat("dynamic prop layer clutter accepted",
        rock::collision_layer_policy::isDynamicPropInteractionLayer(rock::collision_layer_policy::FO4_LAYER_CLUTTER) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("dynamic prop layer weapon accepted",
        rock::collision_layer_policy::isDynamicPropInteractionLayer(rock::collision_layer_policy::FO4_LAYER_WEAPON) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("dynamic prop layer shell casing accepted",
        rock::collision_layer_policy::isDynamicPropInteractionLayer(rock::collision_layer_policy::FO4_LAYER_SHELLCASING) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("dynamic prop layer char controller rejected",
        rock::collision_layer_policy::isDynamicPropInteractionLayer(rock::collision_layer_policy::FO4_LAYER_CHARCONTROLLER) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask includes clutter",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_CLUTTER)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand mask includes biped for broad HIGGS-style body collision",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_BIPED)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand mask includes dead biped for per-finger ragdoll collision",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_DEADBIP)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand mask includes biped no-cc for ragdoll body coverage",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand mask excludes static layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_STATIC)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask excludes item pick layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_ITEMPICK)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask excludes line of sight layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_LINEOFSIGHT)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask excludes path pick layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_PATHPICK)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask excludes projectile layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask excludes spell layer",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_SPELL)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("hand mask always excludes char controller",
        (rock::collision_layer_policy::buildRockHandExpectedMask(false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_CHARCONTROLLER)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon mask includes weapon props",
        (rock::collision_layer_policy::buildRockWeaponExpectedMask(false, false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_WEAPON)) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("weapon mask excludes projectile by default",
        (rock::collision_layer_policy::buildRockWeaponExpectedMask(false, false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_PROJECTILE)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon mask excludes unidentified layer",
        (rock::collision_layer_policy::buildRockWeaponExpectedMask(false, false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_UNIDENTIFIED)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon mask excludes static layer",
        (rock::collision_layer_policy::buildRockWeaponExpectedMask(false, false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_STATIC)) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("weapon mask excludes item pick layer",
        (rock::collision_layer_policy::buildRockWeaponExpectedMask(false, false) & (1ULL << rock::collision_layer_policy::FO4_LAYER_ITEMPICK)) ? 1.0f : 0.0f,
        0.0f);
    const auto handExpectedMask = rock::collision_layer_policy::buildRockHandExpectedMask(true);
    ok &= expectFloat("configured layer compare ignores unmanaged high bits",
        rock::collision_layer_policy::configuredLayerMaskMatches(handExpectedMask | (1ULL << 55), handExpectedMask) ? 1.0f : 0.0f,
        1.0f);

    using rock::physics_body_classifier::BodyClassificationInput;
    using rock::physics_body_classifier::BodyMotionType;
    using rock::physics_body_classifier::BodyRejectReason;
    using rock::physics_body_classifier::InteractionMode;
    using rock::physics_body_classifier::classifyBody;
    using rock::physics_body_classifier::motionTypeFromMotionPropertiesId;

    BodyClassificationInput dynamicClutter{};
    dynamicClutter.bodyId = 100;
    dynamicClutter.motionId = 10;
    dynamicClutter.layer = rock::collision_layer_policy::FO4_LAYER_CLUTTER;
    dynamicClutter.motionType = BodyMotionType::Dynamic;
    ok &= expectFloat("classifier accepts passive dynamic clutter",
        classifyBody(dynamicClutter, InteractionMode::PassivePush).accepted ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("motion properties id dynamic maps to dynamic",
        motionTypeFromMotionPropertiesId(1) == BodyMotionType::Dynamic ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("motion properties id default maps to other",
        motionTypeFromMotionPropertiesId(0xFF) == BodyMotionType::Other ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("body flags static maps to static",
        rock::physics_body_classifier::motionTypeFromBodyFlags(0x00000001) == BodyMotionType::Static ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("body flags dynamic maps to dynamic",
        rock::physics_body_classifier::motionTypeFromBodyFlags(0x0004008A) == BodyMotionType::Dynamic ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("body flags keyframed maps to keyframed",
        rock::physics_body_classifier::motionTypeFromBodyFlags(0x00000006) == BodyMotionType::Keyframed ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("failed prep restores static original motion",
        rock::active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(true, 0) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("failed prep restores keyframed original motion",
        rock::active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(true, 2) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("failed prep keeps already dynamic original motion",
        rock::active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(true, 1) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("failed prep without conversion has nothing to restore",
        rock::active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(false, 0) ? 1.0f : 0.0f,
        0.0f);

    auto dynamicMotionZero = dynamicClutter;
    dynamicMotionZero.motionId = 0;
    dynamicMotionZero.motionType = BodyMotionType::Dynamic;
    const auto dynamicMotionZeroResult = classifyBody(dynamicMotionZero, InteractionMode::PassivePush);
    ok &= expectFloat("classifier rejects motion id zero even when dynamic",
        dynamicMotionZeroResult.reason == BodyRejectReason::InvalidMotionId ? 1.0f : 0.0f,
        1.0f);

    auto keyframedPassive = dynamicClutter;
    keyframedPassive.motionType = BodyMotionType::Keyframed;
    const auto keyframedPassiveResult = classifyBody(keyframedPassive, InteractionMode::PassivePush);
    ok &= expectFloat("classifier rejects passive keyframed",
        keyframedPassiveResult.reason == BodyRejectReason::KeyframedPassive ? 1.0f : 0.0f,
        1.0f);

    auto staticActive = dynamicClutter;
    staticActive.motionType = BodyMotionType::Static;
    const auto staticActiveResult = classifyBody(staticActive, InteractionMode::ActiveGrab);
    ok &= expectFloat("classifier rejects active static after prep",
        staticActiveResult.reason == BodyRejectReason::StaticMotion ? 1.0f : 0.0f,
        1.0f);

    auto heldSelf = dynamicClutter;
    heldSelf.isHeldBySameHand = true;
    const auto heldSelfResult = classifyBody(heldSelf, InteractionMode::PassivePush);
    ok &= expectFloat("classifier rejects same-hand held body",
        heldSelfResult.reason == BodyRejectReason::HeldBySameHand ? 1.0f : 0.0f,
        1.0f);

    auto bipedBody = dynamicClutter;
    bipedBody.layer = rock::collision_layer_policy::FO4_LAYER_BIPED;
    const auto passiveBipedResult = classifyBody(bipedBody, InteractionMode::PassivePush);
    ok &= expectFloat("classifier accepts passive biped push",
        passiveBipedResult.accepted ? 1.0f : 0.0f,
        1.0f);
    const auto activeBipedResult = classifyBody(bipedBody, InteractionMode::ActiveGrab);
    ok &= expectFloat("classifier still rejects active biped grab",
        activeBipedResult.reason == BodyRejectReason::ActorLayer ? 1.0f : 0.0f,
        1.0f);

    auto deadBipedBody = dynamicClutter;
    deadBipedBody.layer = rock::collision_layer_policy::FO4_LAYER_DEADBIP;
    const auto passiveDeadBipedResult = classifyBody(deadBipedBody, InteractionMode::PassivePush);
    ok &= expectFloat("classifier accepts passive dead biped push",
        passiveDeadBipedResult.accepted ? 1.0f : 0.0f,
        1.0f);

    const auto setMotionArgs = rock::physics_recursive_wrappers::makeSetMotionCommand(
        rock::physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
    ok &= expectFloat("set motion command preset dynamic", static_cast<float>(setMotionArgs.presetValue), 1.0f);
    ok &= expectFloat("set motion command recursive", setMotionArgs.recursive ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("set motion command activate", setMotionArgs.activate ? 1.0f : 0.0f, 1.0f);
    const auto enableCollisionArgs = rock::physics_recursive_wrappers::makeEnableCollisionCommand(true, true, true);
    ok &= expectFloat("enable collision command recursive", enableCollisionArgs.recursive ? 1.0f : 0.0f, 1.0f);

    using rock::physics_shape_cast_math::RuntimeShapeCastQuery;
    using rock::physics_shape_cast_math::ShapeCastVec4;
    using rock::physics_shape_cast_math::buildRuntimeShapeCastQuery;
    static_assert(sizeof(RuntimeShapeCastQuery) == 0x80);
    static_assert(offsetof(RuntimeShapeCastQuery, filterRef) == 0x00);
    static_assert(offsetof(RuntimeShapeCastQuery, collisionFilterInfo) == 0x0C);
    static_assert(offsetof(RuntimeShapeCastQuery, shape) == 0x20);
    static_assert(offsetof(RuntimeShapeCastQuery, start) == 0x30);
    static_assert(offsetof(RuntimeShapeCastQuery, displacement) == 0x40);
    static_assert(offsetof(RuntimeShapeCastQuery, inverseDisplacementAndSign) == 0x50);
    static_assert(offsetof(RuntimeShapeCastQuery, tolerance) == 0x60);

    void* fakeFilter = reinterpret_cast<void*>(0x12345678);
    void* fakeShape = reinterpret_cast<void*>(0x87654321);
    const ShapeCastVec4 castStart{ 1.0f, 2.0f, 3.0f, 0.0f };
    const ShapeCastVec4 castDisplacement{ 2.0f, -4.0f, 0.0f, 1.0f };
    const auto castQuery = buildRuntimeShapeCastQuery(fakeFilter, fakeShape, 0x000B002D, castStart, castDisplacement);
    ok &= expectFloat("shape cast query material id", static_cast<float>(castQuery.materialId), 65535.0f);
    ok &= expectFloat("shape cast query filter info", static_cast<float>(castQuery.collisionFilterInfo), static_cast<float>(0x000B002D));
    ok &= expectFloat("shape cast query start x", castQuery.start.x, 1.0f);
    ok &= expectFloat("shape cast query displacement y", castQuery.displacement.y, -4.0f);
    ok &= expectFloat("shape cast query inverse x", castQuery.inverseDisplacementAndSign.x, 0.5f);
    ok &= expectFloat("shape cast query inverse y", castQuery.inverseDisplacementAndSign.y, -0.25f);
    ok &= expectFloat("shape cast query tolerance", castQuery.tolerance, 0.001f);
    const std::uint32_t signMaskBits = std::bit_cast<std::uint32_t>(castQuery.inverseDisplacementAndSign.w);
    ok &= expectFloat("shape cast query sign mask", static_cast<float>(signMaskBits), static_cast<float>(0x3F000005));

    ok &= expectFloat("selection near maps SelectedClose",
        rock::selection_state_policy::stateForSelection(false) == rock::HandState::SelectedClose ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection far maps SelectedFar",
        rock::selection_state_policy::stateForSelection(true) == rock::HandState::SelectedFar ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection process accepts close",
        rock::selection_state_policy::canProcessSelectedState(rock::HandState::SelectedClose) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection process accepts far",
        rock::selection_state_policy::canProcessSelectedState(rock::HandState::SelectedFar) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("plain close selection is not exclusive to other hand",
        rock::selection_state_policy::hasExclusiveObjectSelection(rock::HandState::SelectedClose) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("plain far selection is not exclusive to other hand",
        rock::selection_state_policy::hasExclusiveObjectSelection(rock::HandState::SelectedFar) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("locked far selection reserves object for other hand",
        rock::selection_state_policy::hasExclusiveObjectSelection(rock::HandState::SelectionLocked) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("dynamic pull reserves object for other hand",
        rock::selection_state_policy::hasExclusiveObjectSelection(rock::HandState::Pulled) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("held dynamic object reserves object for other hand",
        rock::selection_state_policy::hasExclusiveObjectSelection(rock::HandState::HeldBody) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection same ref refreshes far to near",
        rock::selection_query_policy::shouldReplaceSelectionForSameRef(true, false, 10, 10) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection same ref refreshes changed body",
        rock::selection_query_policy::shouldReplaceSelectionForSameRef(false, false, 10, 11) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selection same ref keeps unchanged body source",
        rock::selection_query_policy::shouldReplaceSelectionForSameRef(false, false, 10, 10) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("far shape selection tie prefers nearer hit",
        rock::selection_query_policy::isBetterShapeCastCandidate(true, 0.0f, 5.0f, 0.0f, 10.0f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("far shape selection tie rejects farther hit",
        rock::selection_query_policy::isBetterShapeCastCandidate(true, 0.0f, 15.0f, 0.0f, 10.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("close shape selection prefers nearer reach before lateral",
        rock::selection_query_policy::isBetterShapeCastCandidate(false, 2.0f, 5.0f, 0.0f, 15.0f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("close shape selection rejects farther reach despite centered ray",
        rock::selection_query_policy::isBetterShapeCastCandidate(false, 0.0f, 15.0f, 2.0f, 5.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("far hit inside near reach promotes to close",
        rock::selection_query_policy::shouldPromoteFarHitToClose(6.0f, 25.0f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("far hit beyond near reach stays far",
        rock::selection_query_policy::shouldPromoteFarHitToClose(30.0f, 25.0f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("selection shape filter fallback",
        static_cast<float>(rock::selection_query_policy::sanitizeFilterInfo(0, rock::selection_query_policy::kDefaultShapeCastFilterInfo)),
        static_cast<float>(rock::selection_query_policy::kDefaultShapeCastFilterInfo));
    ok &= expectFloat("selection shape filter configured",
        static_cast<float>(rock::selection_query_policy::sanitizeFilterInfo(0x0006002D, rock::selection_query_policy::kDefaultShapeCastFilterInfo)),
        static_cast<float>(0x0006002D));
    ok &= expectFloat("selected object policy blocks fixed activator",
        rock::grab_interaction_policy::shouldBlockSelectedObjectInteraction("ACTI", false, true, 2) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selected object policy allows dynamic activator",
        rock::grab_interaction_policy::shouldBlockSelectedObjectInteraction("ACTI", false, true, 1) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("selected object policy blocks live npc",
        rock::grab_interaction_policy::shouldBlockSelectedObjectInteraction("NPC_", true, false, 0) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selected object policy allows dead npc",
        rock::grab_interaction_policy::shouldBlockSelectedObjectInteraction("NPC_", false, false, 0) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("selected close finger curl applies when slow",
        rock::selected_close_finger_policy::shouldApplyPreCurl(true, true, true, 0.25f, 0.9f) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("selected close finger curl skips when fast",
        rock::selected_close_finger_policy::shouldApplyPreCurl(true, true, true, 1.25f, 0.9f) ? 1.0f : 0.0f,
        0.0f);
    ok &= expectFloat("selected close finger curl disabled by config",
        rock::selected_close_finger_policy::shouldApplyPreCurl(false, true, true, 0.25f, 0.9f) ? 1.0f : 0.0f,
        0.0f);

    const float higgsPullDuration = rock::pull_motion_math::computePullDurationSeconds(1.0f, 0.715619f, -0.415619f, 0.656256f);
    ok &= expectFloat("pull duration matches HIGGS curve at 1m", higgsPullDuration, 0.5f);

    const rock::pull_motion_math::PullMotionInput<TestVector> pullStart{
        .handHavok = TestVector{ 0.0f, 0.0f, 0.0f },
        .objectPointHavok = TestVector{ -0.25f, 0.0f, 0.0f },
        .elapsedSeconds = 0.0f,
        .durationSeconds = 0.5f,
        .applyVelocitySeconds = 0.2f,
        .trackHandSeconds = 0.1f,
        .destinationOffsetHavok = 0.01f,
        .maxVelocityHavok = 10.0f,
    };
    const auto pullStartMotion = rock::pull_motion_math::computePullMotion(pullStart);
    ok &= expectFloat("pull motion starts applying velocity", pullStartMotion.applyVelocity ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("pull motion tracks hand initially", pullStartMotion.refreshTarget ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("pull motion horizontal velocity", pullStartMotion.velocityHavok.x, 0.5f);
    ok &= expectFloat("pull motion vertical arc velocity", pullStartMotion.velocityHavok.z, 2.4725f);

    const rock::pull_motion_math::PullMotionInput<TestVector> pullAfterTrack{
        .handHavok = TestVector{ 5.0f, 0.0f, 0.0f },
        .objectPointHavok = TestVector{ 0.0f, 0.0f, 0.0f },
        .previousTargetHavok = TestVector{ 1.0f, 0.0f, 0.0f },
        .elapsedSeconds = 0.15f,
        .durationSeconds = 0.5f,
        .applyVelocitySeconds = 0.2f,
        .trackHandSeconds = 0.1f,
        .destinationOffsetHavok = 0.01f,
        .maxVelocityHavok = 10.0f,
        .hasPreviousTarget = true,
    };
    const auto pullAfterTrackMotion = rock::pull_motion_math::computePullMotion(pullAfterTrack);
    ok &= expectFloat("pull motion stops tracking after window", pullAfterTrackMotion.refreshTarget ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("pull motion keeps previous target", pullAfterTrackMotion.targetHavok.x, 1.0f);

    auto pullExpired = pullStart;
    pullExpired.elapsedSeconds = 0.25f;
    const auto pullExpiredMotion = rock::pull_motion_math::computePullMotion(pullExpired);
    ok &= expectFloat("pull motion expires after apply window", pullExpiredMotion.expired ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("pull motion expired skips velocity", pullExpiredMotion.applyVelocity ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("pull angular damping keep",
        rock::pull_motion_math::angularVelocityKeepForDamping(8.0f, 0.125f),
        0.5f);

    using rock::object_physics_body_set::BodySetBuilder;
    using rock::object_physics_body_set::PureBodyRecord;
    BodySetBuilder pureSetBuilder;
    pureSetBuilder.addForTest(PureBodyRecord{ .bodyId = 10, .motionId = 2, .accepted = true, .position = TestVector{ 10.0f, 0.0f, 0.0f } });
    pureSetBuilder.addForTest(PureBodyRecord{ .bodyId = 10, .motionId = 2, .accepted = true, .position = TestVector{ 11.0f, 0.0f, 0.0f } });
    pureSetBuilder.addForTest(PureBodyRecord{ .bodyId = 11, .motionId = 2, .accepted = true, .position = TestVector{ 2.0f, 0.0f, 0.0f } });
    pureSetBuilder.addForTest(PureBodyRecord{ .bodyId = 12, .motionId = 3, .accepted = true, .position = TestVector{ 4.0f, 0.0f, 0.0f } });
    pureSetBuilder.addForTest(PureBodyRecord{ .bodyId = 13, .motionId = 4, .accepted = false, .position = TestVector{ 0.0f, 0.0f, 0.0f } });
    const auto pureBodySet = pureSetBuilder.build();
    ok &= expectFloat("body set dedupes body IDs", static_cast<float>(pureBodySet.records.size()), 4.0f);
    ok &= expectFloat("body set unique accepted motions", static_cast<float>(pureBodySet.uniqueAcceptedMotionBodyIds().size()), 2.0f);
    ok &= expectFloat("body set prefers accepted original hit",
        static_cast<float>(pureBodySet.choosePrimaryBody(12, TestVector{ 0.0f, 0.0f, 0.0f }).bodyId),
        12.0f);
    ok &= expectFloat("body set falls back nearest accepted body",
        static_cast<float>(pureBodySet.choosePrimaryBody(99, TestVector{ 0.0f, 0.0f, 0.0f }).bodyId),
        11.0f);
    ok &= expectFloat("body set records duplicate motion skips", static_cast<float>(pureBodySet.duplicateAcceptedMotionSkips()), 1.0f);

    BodySetBuilder surfaceOwnerSetBuilder;
    surfaceOwnerSetBuilder.addForTest(PureBodyRecord{ .bodyId = 20, .motionId = 5, .accepted = true, .position = TestVector{ 100.0f, 0.0f, 0.0f }, .ownerKey = 0xA0 });
    surfaceOwnerSetBuilder.addForTest(PureBodyRecord{ .bodyId = 21, .motionId = 6, .accepted = true, .position = TestVector{ 0.5f, 0.0f, 0.0f }, .ownerKey = 0xB0 });
    surfaceOwnerSetBuilder.addForTest(PureBodyRecord{ .bodyId = 22, .motionId = 7, .accepted = false, .position = TestVector{ 0.25f, 0.0f, 0.0f }, .ownerKey = 0xC0 });
    const auto surfaceOwnerBodySet = surfaceOwnerSetBuilder.build();
    const auto surfaceOwnerChoice = surfaceOwnerBodySet.choosePrimaryBodyWithSurfaceOwner(99, 0xA0, TestVector{ 0.0f, 0.0f, 0.0f });
    ok &= expectFloat("body set surface owner beats nearest", static_cast<float>(surfaceOwnerChoice.bodyId), 20.0f);
    ok &= expectFloat("body set surface owner reason",
        static_cast<float>(surfaceOwnerChoice.reason),
        static_cast<float>(rock::object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted));
    const auto rejectedSurfaceOwnerChoice = surfaceOwnerBodySet.choosePrimaryBodyWithSurfaceOwner(99, 0xC0, TestVector{ 0.0f, 0.0f, 0.0f });
    ok &= expectFloat("body set rejected surface owner falls back nearest", static_cast<float>(rejectedSurfaceOwnerChoice.bodyId), 21.0f);
    const auto heldWakeIds = rock::held_object_body_set_policy::makePrimaryFirstUniqueBodyList(20, std::vector<std::uint32_t>{ 21, 20, 22, 21 });
    ok &= expectFloat("held wake ids include primary plus unique held bodies", static_cast<float>(heldWakeIds.size()), 3.0f);
    ok &= expectFloat("held wake ids keep primary first", static_cast<float>(heldWakeIds[0]), 20.0f);
    ok &= expectFloat("held body set detects any active held body",
        rock::held_object_body_set_policy::containsAnyBody(std::vector<std::uint32_t>{ 10, 11 }, std::vector<std::uint32_t>{ 21, 22 }, 22) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("held body set ignores non-held push body",
        rock::held_object_body_set_policy::containsAnyBody(std::vector<std::uint32_t>{ 10, 11 }, std::vector<std::uint32_t>{ 21, 22 }, 30) ? 1.0f : 0.0f,
        0.0f);

    rock::CanonicalGrabFrame canonicalFrame;
    canonicalFrame.rawHandSpace.translate = RE::NiPoint3{ 1.0f, 2.0f, 3.0f };
    canonicalFrame.constraintHandSpace.translate = RE::NiPoint3{ 4.0f, 5.0f, 6.0f };
    canonicalFrame.handBodyToRawHandAtGrab.translate = RE::NiPoint3{ 19.0f, 20.0f, 21.0f };
    canonicalFrame.bodyLocal.translate = RE::NiPoint3{ 7.0f, 8.0f, 9.0f };
    canonicalFrame.rootBodyLocal.translate = RE::NiPoint3{ 10.0f, 11.0f, 12.0f };
    canonicalFrame.ownerBodyLocal.translate = RE::NiPoint3{ 13.0f, 14.0f, 15.0f };
    canonicalFrame.surfacePointLocal = RE::NiPoint3{ 16.0f, 17.0f, 18.0f };
    canonicalFrame.handScaleAtGrab = 2.0f;
    canonicalFrame.localMeshTriangles.push_back(rock::GrabLocalTriangle{});
    canonicalFrame.heldNode = reinterpret_cast<RE::NiAVObject*>(0x1000);
    canonicalFrame.hasMeshPoseData = true;
    ok &= expectFloat("canonical grab frame reports valid before clear", canonicalFrame.isValid() ? 1.0f : 0.0f, 1.0f);
    canonicalFrame.clear();
    ok &= expectFloat("canonical grab frame clear invalidates", canonicalFrame.isValid() ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("canonical grab frame clear removes triangles", static_cast<float>(canonicalFrame.localMeshTriangles.size()), 0.0f);
    ok &= expectFloat("canonical grab frame clear resets mesh flag", canonicalFrame.hasMeshPoseData ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("canonical grab frame clear resets body raw correction", canonicalFrame.handBodyToRawHandAtGrab.translate.x, 0.0f);
    ok &= expectFloat("canonical grab frame clear resets surface point", canonicalFrame.surfacePointLocal.x, 0.0f);
    ok &= expectFloat("canonical grab frame clear resets hand scale", canonicalFrame.handScaleAtGrab, 1.0f);

    {
        TestTransform rawHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform objectNodeWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform bodyLocal = rock::transform_math::makeIdentityTransform<TestTransform>();

        handBodyWorld.translate = TestVector{ 10.0f, 0.0f, 0.0f };
        objectNodeWorld.translate = TestVector{ 100.0f, 0.0f, 0.0f };

        const TestVector grabPivotWorld{ 13.0f, 0.0f, 0.0f };
        const TestVector surfacePointWorld{ 102.0f, 0.0f, 0.0f };
        const auto splitFrame = rock::grab_frame_math::buildSplitGrabFrame(
            rawHandWorld,
            handBodyWorld,
            objectNodeWorld,
            bodyLocal,
            grabPivotWorld,
            surfacePointWorld);

        ok &= expectFloat("split grab shifted object aligns surface x", splitFrame.shiftedObjectWorld.translate.x, 11.0f);
        ok &= expectFloat("split grab raw visual handspace x", splitFrame.rawHandSpace.translate.x, 11.0f);
        ok &= expectFloat("split grab constraint handspace x", splitFrame.constraintHandSpace.translate.x, 1.0f);
        ok &= expectFloat("split grab pivot A hand-body local x", splitFrame.pivotAHandBodyLocal.x, 3.0f);
        ok &= expectFloat("split grab pivot B body local x", splitFrame.pivotBBodyLocal.x, 2.0f);

        const auto desiredBodyInHandBody = rock::grab_frame_math::desiredBodyInHandBodySpace(splitFrame.constraintHandSpace, bodyLocal);
        const auto pivotInHandBody = rock::transform_math::localPointToWorld(desiredBodyInHandBody, splitFrame.pivotBBodyLocal);
        ok &= expectFloat("split grab pivot B resolves to pivot A local x", pivotInHandBody.x, splitFrame.pivotAHandBodyLocal.x);
    }

    {
        TestTransform rawHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform objectNodeWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform bodyLocal = rock::transform_math::makeIdentityTransform<TestTransform>();

        handBodyWorld.translate = TestVector{ 10.0f, 0.0f, 0.0f };
        objectNodeWorld.translate = TestVector{ 100.0f, 0.0f, 0.0f };

        const auto splitFrame = rock::grab_frame_math::buildSplitGrabFrame(
            rawHandWorld,
            handBodyWorld,
            objectNodeWorld,
            bodyLocal,
            TestVector{ 13.0f, 0.0f, 0.0f },
            TestVector{ 102.0f, 0.0f, 0.0f });

        const auto adjustedVisualHand = rock::grab_frame_math::computeVisualHandFromHeldNode(splitFrame.shiftedObjectWorld, splitFrame.rawHandSpace);
        ok &= expectFloat("split grab visual reverse returns raw hand x", adjustedVisualHand.translate.x, 0.0f);

        const auto wrongConstraintReverse = rock::grab_frame_math::computeVisualHandFromHeldNode(splitFrame.shiftedObjectWorld, splitFrame.constraintHandSpace);
        ok &= expectFloat("split grab constraint reverse would return hand body x", wrongConstraintReverse.translate.x, 10.0f);
    }

    {
        TestTransform rawHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform objectNodeWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform bodyLocal = rock::transform_math::makeIdentityTransform<TestTransform>();

        rawHandWorld.rotate = makeNonSymmetricRotation();
        rawHandWorld.translate = TestVector{ 5.0f, 1.0f, -2.0f };
        rawHandWorld.scale = 1.0f;
        handBodyWorld.translate = TestVector{ 10.0f, 2.0f, 3.0f };
        handBodyWorld.scale = 1.2f;
        objectNodeWorld.rotate = makeNonSymmetricRotation();
        objectNodeWorld.translate = TestVector{ 100.0f, 20.0f, -10.0f };
        objectNodeWorld.scale = 1.25f;
        bodyLocal.translate = TestVector{ 1.0f, 2.0f, 3.0f };
        bodyLocal.scale = 0.8f;

        const auto splitFrame = rock::grab_frame_math::buildSplitGrabFrame(
            rawHandWorld,
            handBodyWorld,
            objectNodeWorld,
            bodyLocal,
            TestVector{ 12.0f, 4.0f, 5.0f },
            TestVector{ 101.0f, 23.0f, -8.0f });

        const auto desiredBodyInHandBody = rock::grab_frame_math::desiredBodyInHandBodySpace(splitFrame.constraintHandSpace, bodyLocal);
        const auto pivotInHandBody = rock::transform_math::localPointToWorld(desiredBodyInHandBody, splitFrame.pivotBBodyLocal);
        ok &= expectFloat("split rotated pivot B resolves to pivot A local x", pivotInHandBody.x, splitFrame.pivotAHandBodyLocal.x);
        ok &= expectFloat("split rotated pivot B resolves to pivot A local y", pivotInHandBody.y, splitFrame.pivotAHandBodyLocal.y);
        ok &= expectFloat("split rotated pivot B resolves to pivot A local z", pivotInHandBody.z, splitFrame.pivotAHandBodyLocal.z);

        const auto adjustedVisualHand = rock::grab_frame_math::computeVisualHandFromHeldNode(splitFrame.shiftedObjectWorld, splitFrame.rawHandSpace);
        ok &= expectFloat("split rotated visual reverse returns raw hand x", adjustedVisualHand.translate.x, rawHandWorld.translate.x);
        ok &= expectFloat("split rotated visual reverse returns raw hand y", adjustedVisualHand.translate.y, rawHandWorld.translate.y);
        ok &= expectFloat("split rotated visual reverse returns raw hand z", adjustedVisualHand.translate.z, rawHandWorld.translate.z);
        ok &= expectMatrix("split rotated visual reverse returns raw hand rotation", adjustedVisualHand.rotate, rawHandWorld.rotate);
    }

    {
        TestTransform rawHandWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform desiredObjectWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform bodyLocal = rock::transform_math::makeIdentityTransform<TestTransform>();

        rawHandWorldAtGrab.rotate = makeRotationY90();
        rawHandWorldAtGrab.translate = TestVector{ 2.0f, 3.0f, 4.0f };
        handBodyWorldAtGrab.rotate = makeRotationX90();
        handBodyWorldAtGrab.translate = TestVector{ 8.0f, -1.0f, 5.0f };
        desiredObjectWorldAtGrab.rotate = makeNonSymmetricRotation();
        desiredObjectWorldAtGrab.translate = TestVector{ 20.0f, 10.0f, -7.0f };

        const auto splitFrame = rock::grab_frame_math::buildSplitGrabFrameFromDesiredObject(
            rawHandWorldAtGrab,
            handBodyWorldAtGrab,
            desiredObjectWorldAtGrab,
            bodyLocal,
            TestVector{ 9.0f, -1.0f, 5.0f });

        const auto visualAtGrab = rock::hand_visual_authority_math::buildSplitFrameReverseAlignedHandWorld(
            desiredObjectWorldAtGrab,
            splitFrame.constraintHandSpace,
            splitFrame.handBodyToRawHandAtGrab);
        ok &= expectFloat("split visual authority returns raw hand at grab x", visualAtGrab.translate.x, rawHandWorldAtGrab.translate.x);
        ok &= expectFloat("split visual authority returns raw hand at grab y", visualAtGrab.translate.y, rawHandWorldAtGrab.translate.y);
        ok &= expectFloat("split visual authority returns raw hand at grab z", visualAtGrab.translate.z, rawHandWorldAtGrab.translate.z);
        ok &= expectMatrix("split visual authority returns raw hand rotation at grab", visualAtGrab.rotate, rawHandWorldAtGrab.rotate);

        TestTransform currentHandBodyWorld = handBodyWorldAtGrab;
        currentHandBodyWorld.rotate = makeNonSymmetricRotation();
        currentHandBodyWorld.translate = TestVector{ 30.0f, -5.0f, 11.0f };
        const auto currentHeldNodeWorld =
            rock::transform_math::composeTransforms(currentHandBodyWorld, splitFrame.constraintHandSpace);
        const auto expectedCurrentRawHand =
            rock::transform_math::composeTransforms(currentHandBodyWorld, splitFrame.handBodyToRawHandAtGrab);
        const auto correctedVisual = rock::hand_visual_authority_math::buildSplitFrameReverseAlignedHandWorld(
            currentHeldNodeWorld,
            splitFrame.constraintHandSpace,
            splitFrame.handBodyToRawHandAtGrab);

        ok &= expectFloat("split visual authority follows hand-body driven held object x", correctedVisual.translate.x, expectedCurrentRawHand.translate.x);
        ok &= expectFloat("split visual authority follows hand-body driven held object y", correctedVisual.translate.y, expectedCurrentRawHand.translate.y);
        ok &= expectFloat("split visual authority follows hand-body driven held object z", correctedVisual.translate.z, expectedCurrentRawHand.translate.z);
        ok &= expectMatrix("split visual authority follows same-direction rotation", correctedVisual.rotate, expectedCurrentRawHand.rotate);
    }

    {
        TestTransform rawHandWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform visualHeldNodeWorldAtGrab = rock::transform_math::makeIdentityTransform<TestTransform>();

        rawHandWorldAtGrab.rotate = makeRotationY90();
        rawHandWorldAtGrab.translate = TestVector{ 4.0f, 8.0f, 12.0f };
        handBodyWorldAtGrab.rotate = makeRotationX90();
        handBodyWorldAtGrab.translate = TestVector{ 9.0f, 8.0f, 12.0f };
        visualHeldNodeWorldAtGrab.rotate = makeNonSymmetricRotation();
        visualHeldNodeWorldAtGrab.translate = TestVector{ 30.0f, -2.0f, 5.0f };

        const auto splitFrame = rock::grab_frame_math::buildSplitGrabFrameFromDesiredObject(
            rawHandWorldAtGrab,
            handBodyWorldAtGrab,
            visualHeldNodeWorldAtGrab,
            rock::transform_math::makeIdentityTransform<TestTransform>(),
            TestVector{ 9.0f, 8.0f, 12.0f });

        TestTransform visualHeldNodeWorldNow =
            rock::transform_math::composeTransforms(rawHandWorldAtGrab, splitFrame.rawHandSpace);
        visualHeldNodeWorldNow.rotate = makeRotationX90();
        visualHeldNodeWorldNow.translate = TestVector{ 40.0f, -6.0f, 7.0f };

        TestTransform bodyDerivedHeldNodeWorldNow = visualHeldNodeWorldNow;
        bodyDerivedHeldNodeWorldNow.rotate = makeNonSymmetricRotation();
        bodyDerivedHeldNodeWorldNow.translate = TestVector{ 47.0f, -3.0f, 10.0f };

        const auto appliedVisualTarget = rock::hand_visual_authority_math::buildAppliedVisualAuthorityHandWorld(
            visualHeldNodeWorldNow,
            splitFrame.rawHandSpace,
            bodyDerivedHeldNodeWorldNow,
            splitFrame.constraintHandSpace,
            splitFrame.handBodyToRawHandAtGrab);
        const auto higgsVisualTarget = rock::hand_visual_authority_math::buildHiggsReverseAlignedHandWorld(
            visualHeldNodeWorldNow,
            splitFrame.rawHandSpace);
        const auto bodyDerivedSplitTarget = rock::hand_visual_authority_math::buildSplitFrameReverseAlignedHandWorld(
            bodyDerivedHeldNodeWorldNow,
            splitFrame.constraintHandSpace,
            splitFrame.handBodyToRawHandAtGrab);

        const TestVector splitDrift{
            appliedVisualTarget.translate.x - bodyDerivedSplitTarget.translate.x,
            appliedVisualTarget.translate.y - bodyDerivedSplitTarget.translate.y,
            appliedVisualTarget.translate.z - bodyDerivedSplitTarget.translate.z
        };
        const float splitDriftDistance = std::sqrt(
            splitDrift.x * splitDrift.x +
            splitDrift.y * splitDrift.y +
            splitDrift.z * splitDrift.z);

        ok &= expectFloat("visual authority applied target uses actual node x", appliedVisualTarget.translate.x, higgsVisualTarget.translate.x);
        ok &= expectFloat("visual authority applied target uses actual node y", appliedVisualTarget.translate.y, higgsVisualTarget.translate.y);
        ok &= expectFloat("visual authority applied target uses actual node z", appliedVisualTarget.translate.z, higgsVisualTarget.translate.z);
        ok &= expectMatrix("visual authority applied target uses actual node rotation", appliedVisualTarget.rotate, higgsVisualTarget.rotate);
        ok &= expectFloat("visual authority rejects body-derived split drift", splitDriftDistance > 1.0f ? 1.0f : 0.0f, 1.0f);
    }

    {
        TestTransform rawHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform heldNodeWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        heldNodeWorld.translate = TestVector{ 20.0f, -5.0f, 7.0f };
        rawHandWorld.translate = TestVector{ 3.0f, 4.0f, 5.0f };

        const auto rawHandSpace = rock::grab_frame_math::objectInFrameSpace(rawHandWorld, heldNodeWorld);
        const auto telemetryTarget = rock::grab_transform_telemetry::computeHiggsReverseTarget(heldNodeWorld, rawHandSpace);
        ok &= expectFloat("grab telemetry HIGGS reverse returns raw hand x", telemetryTarget.translate.x, rawHandWorld.translate.x);
        ok &= expectFloat("grab telemetry HIGGS reverse returns raw hand y", telemetryTarget.translate.y, rawHandWorld.translate.y);
        ok &= expectFloat("grab telemetry HIGGS reverse returns raw hand z", telemetryTarget.translate.z, rawHandWorld.translate.z);

        const auto sameAxisDots = rock::grab_transform_telemetry::axisAlignmentDots(rawHandWorld.rotate, telemetryTarget.rotate);
        ok &= expectFloat("grab telemetry same axis dot x", sameAxisDots.x, 1.0f);
        ok &= expectFloat("grab telemetry same axis dot y", sameAxisDots.y, 1.0f);
        ok &= expectFloat("grab telemetry same axis dot z", sameAxisDots.z, 1.0f);

        TestTransform flippedHandWorld = rawHandWorld;
        flippedHandWorld.rotate.entry[0][0] = -1.0f;
        const auto flippedAxisDots = rock::grab_transform_telemetry::axisAlignmentDots(rawHandWorld.rotate, flippedHandWorld.rotate);
        ok &= expectFloat("grab telemetry flipped axis dot x", flippedAxisDots.x, -1.0f);
    }

    {
        const TestVector pivotA{ 1.0f, 2.0f, 3.0f };
        const TestVector pivotB{ 4.0f, 6.0f, 3.0f };
        const auto delta = rock::grab_transform_telemetry::measurePointPair(pivotA, pivotB);
        ok &= expectFloat("grab telemetry point delta x", delta.delta.x, -3.0f);
        ok &= expectFloat("grab telemetry point delta y", delta.delta.y, -4.0f);
        ok &= expectFloat("grab telemetry point distance", delta.distance, 5.0f);
    }

    {
        rock::grab_transform_telemetry::FrameStamp stamp{};
        stamp.session = 42;
        stamp.frame = 777;
        stamp.tickMs = 123456;
        const auto prefix = rock::grab_transform_telemetry::formatStampPrefix(stamp, false, "held");
        ok &= expectString("grab telemetry stamp prefix", prefix, "GTEL session=42 frame=777 tickMs=123456 hand=R phase=held");
    }

    {
        TestTransform liveHandWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform handBodyWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        TestTransform desiredObjectWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        liveHandWorld.translate = TestVector{ 10.0f, 20.0f, 30.0f };
        handBodyWorld.translate = TestVector{ 15.0f, 20.0f, 30.0f };
        desiredObjectWorld.translate = TestVector{ 25.0f, 23.0f, 31.0f };

        const auto rawHandSpace = rock::grab_frame_math::objectInFrameSpace(liveHandWorld, desiredObjectWorld);
        const auto constraintHandSpace = rock::grab_frame_math::objectInFrameSpace(handBodyWorld, desiredObjectWorld);
        const auto reconstructedDesired = rock::grab_transform_telemetry::computeCapturedObjectFromLiveHand(liveHandWorld, rawHandSpace);
        ok &= expectFloat("grab telemetry captured object reconstructs desired x", reconstructedDesired.translate.x, desiredObjectWorld.translate.x);
        ok &= expectFloat("grab telemetry captured object reconstructs desired y", reconstructedDesired.translate.y, desiredObjectWorld.translate.y);
        ok &= expectFloat("grab telemetry captured object reconstructs desired z", reconstructedDesired.translate.z, desiredObjectWorld.translate.z);

        const auto rawReverse = rock::grab_transform_telemetry::computeReverseTargetFromCapturedSpace(desiredObjectWorld, rawHandSpace);
        ok &= expectFloat("grab telemetry raw reverse returns live hand x", rawReverse.translate.x, liveHandWorld.translate.x);

        const auto constraintReverse = rock::grab_transform_telemetry::computeConstraintReverseTarget(desiredObjectWorld, constraintHandSpace);
        ok &= expectFloat("grab telemetry constraint reverse returns hand body x", constraintReverse.translate.x, handBodyWorld.translate.x);

        TestTransform currentLiveHandWorld = liveHandWorld;
        TestTransform currentHandBodyWorld = handBodyWorld;
        currentLiveHandWorld.translate = TestVector{ 110.0f, 20.0f, 30.0f };
        currentHandBodyWorld.translate = TestVector{ 115.0f, 20.0f, 30.0f };
        const auto currentRawDesired =
            rock::grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(currentLiveHandWorld, rawHandSpace);
        const auto currentConstraintDesired =
            rock::grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(currentHandBodyWorld, constraintHandSpace);
        ok &= expectFloat("grab telemetry current raw desired follows current hand", currentRawDesired.translate.x, 125.0f);
        ok &= expectFloat("grab telemetry current constraint desired follows current hand body", currentConstraintDesired.translate.x, 125.0f);

        const auto higgsVisualTarget =
            rock::hand_visual_authority_math::buildHiggsReverseAlignedHandWorld(desiredObjectWorld, rawHandSpace);
        ok &= expectFloat("grab visual authority HIGGS reverse target returns live hand x", higgsVisualTarget.translate.x, liveHandWorld.translate.x);

        TestTransform driftedHeldNode = desiredObjectWorld;
        driftedHeldNode.translate.x += 10.0f;
        driftedHeldNode.rotate = makeNonSymmetricRotation();
        const auto heldVsDesired = rock::grab_transform_telemetry::measureHeldNodeVsDesiredObject(driftedHeldNode, desiredObjectWorld);
        ok &= expectFloat("grab telemetry held desired drift position", heldVsDesired.positionGameUnits, 10.0f);
        ok &= expectFloat("grab telemetry held desired drift rotation", heldVsDesired.rotationDegrees, 90.0f);
    }

    {
        RE::NiTransform rawHandWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        rawHandWorld.translate = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
        const auto basis = rock::grab_transform_telemetry_overlay::buildHandAttachedTextBasis(rawHandWorld, false);
        ok &= expectFloat("grab telemetry label anchor follows hand x", basis.anchor.x, 10.0f);
        ok &= expectFloat("grab telemetry label anchor follows hand y", basis.anchor.y, 32.0f);
        ok &= expectFloat("grab telemetry label anchor follows hand z", basis.anchor.z, 48.0f);
        ok &= expectFloat("grab telemetry label line advances downward in panel up", rock::grab_transform_telemetry_overlay::lineAnchor(basis, 2).z, 39.0f);
    }

    using rock::nearby_grab_damping::PureDampingCandidate;
    using rock::nearby_grab_damping::PureDampingCandidateSet;
    PureDampingCandidateSet dampingCandidates;
    dampingCandidates.add(PureDampingCandidate{ .bodyId = 30, .motionId = 300, .accepted = true });
    dampingCandidates.add(PureDampingCandidate{ .bodyId = 31, .motionId = 300, .accepted = true });
    dampingCandidates.add(PureDampingCandidate{ .bodyId = 32, .motionId = 301, .accepted = true });
    dampingCandidates.add(PureDampingCandidate{ .bodyId = 33, .motionId = 302, .accepted = true, .heldBySameHand = true });
    dampingCandidates.add(PureDampingCandidate{ .bodyId = 34, .motionId = 303, .accepted = false });
    const auto dampingBodyIds = dampingCandidates.uniqueAcceptedMotionBodyIds();
    ok &= expectFloat("nearby damping dedupes by motion id", static_cast<float>(dampingBodyIds.size()), 2.0f);
    ok &= expectFloat("nearby damping keeps first motion body", static_cast<float>(dampingBodyIds[0]), 30.0f);
    ok &= expectFloat("nearby damping rejects held body ids", dampingCandidates.containsBodyId(33) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("nearby damping duplicate motion skips", static_cast<float>(dampingCandidates.duplicateMotionSkips()), 1.0f);
    ok &= expectFloat("nearby damping clamp radius", rock::nearby_grab_damping::sanitizeRadius(-10.0f), 0.0f);
    ok &= expectFloat("nearby damping clamp duration", rock::nearby_grab_damping::sanitizeDuration(-1.0f), 0.0f);
    ok &= expectFloat("nearby damping clamp value high", rock::nearby_grab_damping::sanitizeDamping(2.0f), 1.0f);
    ok &= expectFloat("nearby damping velocity attenuation",
        rock::nearby_grab_damping::applyDampingToVelocity(TestVector{ 10.0f, -5.0f, 2.0f }, 0.25f).x,
        7.5f);
    rock::nearby_grab_damping::NearbyGrabDampingState pureDampingState;
    pureDampingState.active = true;
    pureDampingState.remainingSeconds = 0.1f;
    pureDampingState.motions.push_back(rock::nearby_grab_damping::SavedNearbyMotionDamping{ .representativeBodyId = 30, .motionId = 300, .active = true });
    const bool expired = rock::nearby_grab_damping::advanceTimer(pureDampingState, 0.2f);
    ok &= expectFloat("nearby damping timer expires once", expired ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("nearby damping timer deactivates", pureDampingState.active ? 1.0f : 0.0f, 0.0f);
    const bool expiredAgain = rock::nearby_grab_damping::advanceTimer(pureDampingState, 0.2f);
    ok &= expectFloat("nearby damping timer restore idempotent", expiredAgain ? 1.0f : 0.0f, 0.0f);

    const rock::push_assist::PushAssistInput pushInput{
        .enabled = true,
        .sourceVelocity = TestVector{ 3.0f, 4.0f, 0.0f },
        .minSpeed = 2.0f,
        .maxImpulse = 3.0f,
        .layerMultiplier = 1.0f,
        .cooldownRemainingSeconds = 0.0f,
    };
    const auto pushResult = rock::push_assist::computePushImpulse(pushInput);
    ok &= expectFloat("push assist accepted", pushResult.apply ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("push assist clamps magnitude", pushResult.impulseMagnitude, 3.0f);
    const auto halfMassVelocityDelta = rock::push_assist::computeVelocityDeltaFromImpulse(pushResult.impulse, 0.5f);
    ok &= expectFloat("push assist mass-aware velocity delta x", halfMassVelocityDelta.x, 0.9f);
    ok &= expectFloat("push assist mass-aware velocity delta y", halfMassVelocityDelta.y, 1.2f);
    const auto zeroMassVelocityDelta = rock::push_assist::computeVelocityDeltaFromImpulse(pushResult.impulse, 0.0f);
    ok &= expectFloat("push assist skips invalid inverse mass", zeroMassVelocityDelta.x, 0.0f);
    const rock::push_assist::PushAssistInput pushCooldownInput{ .enabled = true,
        .sourceVelocity = TestVector{ 3.0f, 4.0f, 0.0f },
        .minSpeed = 2.0f,
        .maxImpulse = 3.0f,
        .layerMultiplier = 1.0f,
        .cooldownRemainingSeconds = 0.1f };
    const auto pushCooldown = rock::push_assist::computePushImpulse(pushCooldownInput);
    ok &= expectFloat("push assist cooldown skips", pushCooldown.apply ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("hand reset cleanup required with active constraint",
        rock::hand_lifecycle_policy::requiresHavokCleanupBeforeReset(true, false, 0, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand reset cleanup required with collision suppression",
        rock::hand_lifecycle_policy::requiresHavokCleanupBeforeReset(false, true, 0, false, false) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand reset cleanup required with collision body",
        rock::hand_lifecycle_policy::requiresHavokCleanupBeforeReset(false, false, 0, false, true) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("hand reset cleanup clean state not required",
        rock::hand_lifecycle_policy::requiresHavokCleanupBeforeReset(false, false, 0, false, false) ? 1.0f : 0.0f,
        0.0f);

    using rock::native_melee_suppression::NativeMeleeEvent;
    using rock::native_melee_suppression::NativeMeleePolicyInput;
    using rock::native_melee_suppression::NativeMeleeSuppressionAction;
    using rock::native_melee_suppression::evaluateNativeMeleeSuppression;

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
        .fullSuppression = false,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = true };
    ok &= expectFloat("native melee player hitframe handled by physical swing",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, physicalSwingHitFrameInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::ReturnHandled));

    const NativeMeleePolicyInput fullSuppressionHitFrameInput{ .rockEnabled = true,
        .suppressionEnabled = true,
        .fullSuppression = true,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = true };
    ok &= expectFloat("native melee full suppression ignores physical swing hitframe",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::HitFrame, fullSuppressionHitFrameInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::ReturnUnhandled));

    const NativeMeleePolicyInput disabledSuppressionInput{ .rockEnabled = true,
        .suppressionEnabled = false,
        .suppressWeaponSwing = true,
        .suppressHitFrame = true,
        .actorIsPlayer = true,
        .physicalSwingActive = false };
    ok &= expectFloat("native melee disabled policy calls native",
        static_cast<float>(evaluateNativeMeleeSuppression(NativeMeleeEvent::WeaponSwing, disabledSuppressionInput).action),
        static_cast<float>(NativeMeleeSuppressionAction::CallNative));
    ok &= expectFloat("native melee swing lease inactive with no expiry", rock::native_melee_suppression::isPhysicalSwingLeaseActive(100, 0) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("native melee swing lease active before expiry", rock::native_melee_suppression::isPhysicalSwingLeaseActive(100, 110) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("native melee swing lease inactive after expiry", rock::native_melee_suppression::isPhysicalSwingLeaseActive(111, 110) ? 1.0f : 0.0f, 0.0f);

    ok &= expectFloat("logging clamps below trace", static_cast<float>(rock::logging_policy::clampLogLevel(-4)), 0.0f);
    ok &= expectFloat("logging clamps above off", static_cast<float>(rock::logging_policy::clampLogLevel(99)), 6.0f);
    ok &= expectFloat("logging info emits info", rock::logging_policy::shouldEmit(2, rock::logging_policy::LogLevel::Info) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("logging info suppresses debug", rock::logging_policy::shouldEmit(2, rock::logging_policy::LogLevel::Debug) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("logging warn emits error", rock::logging_policy::shouldEmit(3, rock::logging_policy::LogLevel::Error) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("logging off suppresses critical", rock::logging_policy::shouldEmit(6, rock::logging_policy::LogLevel::Critical) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("logging sample clamps low", static_cast<float>(rock::logging_policy::sanitizeSampleMilliseconds(1)), 250.0f);
    ok &= expectFloat("logging sample clamps high", static_cast<float>(rock::logging_policy::sanitizeSampleMilliseconds(90000)), 60000.0f);

    using rock::root_flattened_finger_skeleton_runtime::FingerChain;
    using rock::root_flattened_finger_skeleton_runtime::Snapshot;
    using rock::root_flattened_finger_skeleton_runtime::buildLandmarkSet;
    using rock::root_flattened_finger_skeleton_runtime::fingerBoneName;

    Snapshot liveFingerSnapshot{};
    liveFingerSnapshot.palmNormalWorld = RE::NiPoint3(0.0f, 0.0f, -1.0f);
    liveFingerSnapshot.palmNormalValid = true;
    for (std::size_t finger = 0; finger < liveFingerSnapshot.fingers.size(); ++finger) {
        FingerChain& chain = liveFingerSnapshot.fingers[finger];
        chain.valid = true;
        chain.points[0] = RE::NiPoint3(1.0f + static_cast<float>(finger), 0.0f, 0.0f);
        chain.points[1] = RE::NiPoint3(3.0f + static_cast<float>(finger), 0.0f, 0.0f);
        chain.points[2] = RE::NiPoint3(6.0f + static_cast<float>(finger), 0.0f, 0.0f);
    }
    liveFingerSnapshot.valid = true;
    const auto liveLandmarks = buildLandmarkSet(liveFingerSnapshot);
    ok &= expectFloat("live frik landmark set valid", liveLandmarks.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("live frik thumb base x", liveLandmarks.fingers[0].base.x, 1.0f);
    ok &= expectFloat("live frik thumb length", liveLandmarks.fingers[0].length, 5.0f);
    ok &= expectFloat("live frik thumb direction x", liveLandmarks.fingers[0].openDirection.x, 1.0f);
    ok &= expectFloat("live frik thumb direction y", liveLandmarks.fingers[0].openDirection.y, 0.0f);
    ok &= expectFloat("live frik thumb direction z", liveLandmarks.fingers[0].openDirection.z, 0.0f);
    ok &= expectFloat("live frik palm normal x", liveLandmarks.palmNormalWorld.x, 0.0f);
    ok &= expectFloat("live frik palm normal y", liveLandmarks.palmNormalWorld.y, 0.0f);
    ok &= expectFloat("live frik palm normal z", liveLandmarks.palmNormalWorld.z, -1.0f);

    liveFingerSnapshot.fingers[2].valid = false;
    const auto missingLandmarks = buildLandmarkSet(liveFingerSnapshot);
    ok &= expectFloat("live frik missing chain invalidates set", missingLandmarks.valid ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("live frik missing middle invalid", missingLandmarks.fingers[2].valid ? 1.0f : 0.0f, 0.0f);

    ok &= expectString("right index tip bone name", fingerBoneName(false, 1, 2), "RArm_Finger23");
    ok &= expectString("left pinky base bone name", fingerBoneName(true, 4, 0), "LArm_Finger51");

    using rock::skeleton_bone_debug_math::BoneColliderProfileVariant;
    using rock::skeleton_bone_debug_math::DebugSkeletonBoneMode;
    using rock::skeleton_bone_debug_math::DebugSkeletonBoneSource;
    using rock::skeleton_bone_debug_math::computeAxisEndpoints;
    using rock::skeleton_bone_debug_math::descriptorTableBonesAreNamed;
    using rock::skeleton_bone_debug_math::fingerSegmentDescriptorCount;
    using rock::skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource;
    using rock::skeleton_bone_debug_math::sanitizeMaxSkeletonAxesDrawn;
    using rock::skeleton_bone_debug_math::sanitizeMaxSkeletonBonesDrawn;
    using rock::skeleton_bone_debug_math::requiredFingerBoneNames;
    using rock::skeleton_bone_debug_math::resolveDrawableParentIndex;
    using rock::skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode;
    using rock::skeleton_bone_debug_math::shouldIncludeBone;
    using rock::skeleton_bone_debug_math::shouldDrawSkeletonAxis;
    using rock::skeleton_bone_debug_math::skeletonOverlayBudget;
    using rock::skeleton_bone_debug_math::tipExtrapolatedDescriptorCount;

    const auto& requiredFingerBones = requiredFingerBoneNames();
    ok &= expectFloat("skeleton required finger bone count", static_cast<float>(requiredFingerBones.size()), 30.0f);
    ok &= expectFloat("skeleton required contains right hand", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "RArm_Hand") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton required contains left hand", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "LArm_Hand") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton required contains right index tip", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "RArm_Finger23") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton required contains left pinky tip", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "LArm_Finger53") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton required contains chest", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "Chest") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton required excludes skin helper", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "RArm_UpperArm_skin") ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton required excludes weapon helper", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "Weapon") ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton required excludes camera helper", shouldIncludeBone(DebugSkeletonBoneMode::CoreBodyAndFingers, "Camera") ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton all mode includes helper", shouldIncludeBone(DebugSkeletonBoneMode::AllFlattenedBones, "Weapon") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton hands mode includes forearm", shouldIncludeBone(DebugSkeletonBoneMode::HandsAndForearmsOnly, "RArm_ForeArm1") ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton hands mode excludes torso", shouldIncludeBone(DebugSkeletonBoneMode::HandsAndForearmsOnly, "Chest") ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton invalid mode defaults to core", static_cast<float>(sanitizeDebugSkeletonBoneMode(99)), static_cast<float>(DebugSkeletonBoneMode::CoreBodyAndFingers));
    ok &= expectFloat("skeleton source default is game root flattened tree",
        static_cast<float>(sanitizeDebugSkeletonBoneSource(0)),
        static_cast<float>(DebugSkeletonBoneSource::GameRootFlattenedBoneTree));
    ok &= expectFloat("skeleton source explicit diagnostic is first-person",
        static_cast<float>(sanitizeDebugSkeletonBoneSource(2)),
        static_cast<float>(DebugSkeletonBoneSource::FirstPersonDiagnosticOnly));

    const std::vector<int> parentIndices{ -1, 0, 1, 2, 0 };
    const std::vector<bool> includedBones{ true, false, false, true, true };
    ok &= expectFloat("skeleton parent collapse skips filtered ancestors", static_cast<float>(resolveDrawableParentIndex(3, parentIndices, includedBones)), 0.0f);
    ok &= expectFloat("skeleton parent collapse keeps direct included parent", static_cast<float>(resolveDrawableParentIndex(4, parentIndices, includedBones)), 0.0f);
    ok &= expectFloat("skeleton parent collapse root has no parent", static_cast<float>(resolveDrawableParentIndex(0, parentIndices, includedBones)), -1.0f);

    TestTransform identityAxisTransform = rock::transform_math::makeIdentityTransform<TestTransform>();
    identityAxisTransform.translate = TestVector{ 2.0f, 3.0f, 4.0f };
    const auto identityAxisEndpoints = computeAxisEndpoints(identityAxisTransform, 5.0f);
    ok &= expectFloat("skeleton identity x axis end x", identityAxisEndpoints.xEnd.x, 7.0f);
    ok &= expectFloat("skeleton identity y axis end y", identityAxisEndpoints.yEnd.y, 8.0f);
    ok &= expectFloat("skeleton identity z axis end z", identityAxisEndpoints.zEnd.z, 9.0f);

    TestTransform rotatedAxisTransform = rock::transform_math::makeIdentityTransform<TestTransform>();
    rotatedAxisTransform.rotate = niRotation;
    rotatedAxisTransform.translate = TestVector{ 2.0f, 3.0f, 4.0f };
    const auto rotatedAxisEndpoints = computeAxisEndpoints(rotatedAxisTransform, 5.0f);
    ok &= expectFloat("skeleton rotated x axis end x", rotatedAxisEndpoints.xEnd.x, 2.0f);
    ok &= expectFloat("skeleton rotated x axis end y", rotatedAxisEndpoints.xEnd.y, -2.0f);
    ok &= expectFloat("skeleton rotated y axis end x", rotatedAxisEndpoints.yEnd.x, 7.0f);
    ok &= expectFloat("skeleton rotated y axis end y", rotatedAxisEndpoints.yEnd.y, 3.0f);

    ok &= expectFloat("skeleton overlay budget supports core and fingers",
        skeletonOverlayBudget() >= rock::skeleton_bone_debug_math::estimatedCoreBodyAndFingerBoneCount() ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("skeleton overlay budget supports max flattened tree",
        skeletonOverlayBudget() >= 768 ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("skeleton bone draw cap clamps negative", static_cast<float>(sanitizeMaxSkeletonBonesDrawn(-10)), 0.0f);
    ok &= expectFloat("skeleton bone draw cap clamps high", static_cast<float>(sanitizeMaxSkeletonBonesDrawn(2000)), static_cast<float>(skeletonOverlayBudget()));
    ok &= expectFloat("skeleton axis draw cap clamps high", static_cast<float>(sanitizeMaxSkeletonAxesDrawn(2000)), static_cast<float>(skeletonOverlayBudget()));
    ok &= expectFloat("skeleton axis filter accepts listed bone", shouldDrawSkeletonAxis("RArm_Hand,LArm_Hand", "RArm_Hand", 0, 80) ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("skeleton axis filter rejects unlisted bone", shouldDrawSkeletonAxis("RArm_Hand,LArm_Hand", "Chest", 0, 80) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton axis cap rejects after cap", shouldDrawSkeletonAxis("", "Chest", 80, 80) ? 1.0f : 0.0f, 0.0f);
    ok &= expectFloat("skeleton standard collider table is named",
        descriptorTableBonesAreNamed(BoneColliderProfileVariant::Standard) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("skeleton power armor collider table is named",
        descriptorTableBonesAreNamed(BoneColliderProfileVariant::PowerArmor) ? 1.0f : 0.0f,
        1.0f);
    ok &= expectFloat("skeleton power armor forearm radius is larger",
        rock::skeleton_bone_debug_math::representativeForearmRadius(BoneColliderProfileVariant::PowerArmor) >
                rock::skeleton_bone_debug_math::representativeForearmRadius(BoneColliderProfileVariant::Standard)
            ? 1.0f
            : 0.0f,
        1.0f);
    ok &= expectFloat("skeleton standard finger descriptors complete",
        static_cast<float>(fingerSegmentDescriptorCount(BoneColliderProfileVariant::Standard)),
        30.0f);
    ok &= expectFloat("skeleton power armor finger descriptors complete",
        static_cast<float>(fingerSegmentDescriptorCount(BoneColliderProfileVariant::PowerArmor)),
        30.0f);
    ok &= expectFloat("skeleton standard tip extrapolation descriptors complete",
        static_cast<float>(tipExtrapolatedDescriptorCount(BoneColliderProfileVariant::Standard)),
        10.0f);
    ok &= expectFloat("skeleton power armor tip extrapolation descriptors complete",
        static_cast<float>(tipExtrapolatedDescriptorCount(BoneColliderProfileVariant::PowerArmor)),
        10.0f);

    using rock::hand_bone_collider_geometry_math::BoneColliderFrameInput;
    using rock::hand_bone_collider_geometry_math::buildSegmentColliderFrame;
    using rock::hand_bone_collider_geometry_math::buildPalmAnchorFrame;
    using rock::hand_bone_collider_geometry_math::makeCapsuleLikeHullPoints;
    using rock::hand_bone_grab_pivot_math::BoneContactPoint;
    using rock::hand_bone_grab_pivot_math::chooseHandContactPivot;
    using rock::hand_collider_semantics::HandColliderRole;
    using rock::hand_collider_semantics::HandColliderRuntimeMode;
    using rock::hand_collider_semantics::HandFinger;
    using rock::hand_collider_semantics::HandFingerSegment;
    using rock::hand_collider_semantics::colliderBodyCountPerHand;
    using rock::hand_collider_semantics::fingerForRole;
    using rock::hand_collider_semantics::roleName;
    using rock::hand_collider_semantics::sanitizeHandColliderRuntimeMode;
    using rock::hand_collider_semantics::segmentForRole;

    ok &= expectFloat("hand collider default runtime mode is bone derived",
        static_cast<float>(sanitizeHandColliderRuntimeMode(99)),
        static_cast<float>(HandColliderRuntimeMode::BoneDerivedHands));
    ok &= expectFloat("hand collider body count includes palm anchor plus all finger segments",
        static_cast<float>(colliderBodyCountPerHand()),
        20.0f);
    ok &= expectString("hand collider index tip role name", roleName(HandColliderRole::IndexTip), "IndexTip");
    ok &= expectFloat("hand collider index tip finger",
        static_cast<float>(fingerForRole(HandColliderRole::IndexTip)),
        static_cast<float>(HandFinger::Index));
    ok &= expectFloat("hand collider thumb middle segment",
        static_cast<float>(segmentForRole(HandColliderRole::ThumbMiddle)),
        static_cast<float>(HandFingerSegment::Middle));

    BoneColliderFrameInput<TestTransform, TestVector> indexBaseInput{};
    indexBaseInput.start = rock::transform_math::makeIdentityTransform<TestTransform>();
    indexBaseInput.end = rock::transform_math::makeIdentityTransform<TestTransform>();
    indexBaseInput.start.translate = TestVector{ 1.0f, 2.0f, 3.0f };
    indexBaseInput.end.translate = TestVector{ 5.0f, 2.0f, 3.0f };
    indexBaseInput.radius = 0.5f;
    indexBaseInput.convexRadius = 0.1f;
    const auto indexBaseFrame = buildSegmentColliderFrame(indexBaseInput);
    ok &= expectFloat("hand bone segment frame valid", indexBaseFrame.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand bone segment center x", indexBaseFrame.transform.translate.x, 3.0f);
    ok &= expectFloat("hand bone segment length", indexBaseFrame.length, 4.0f);
    ok &= expectFloat("hand bone segment x axis follows child x", indexBaseFrame.xAxis.x, 1.0f);
    ok &= expectFloat("hand bone segment y axis uses bone roll y", indexBaseFrame.yAxis.y, 1.0f);
    ok &= expectFloat("hand bone segment hull has support points",
        static_cast<float>(makeCapsuleLikeHullPoints<TestVector>(indexBaseFrame.length, indexBaseFrame.radius).size()),
        16.0f);

    BoneColliderFrameInput<TestTransform, TestVector> tipInput = indexBaseInput;
    tipInput.start.translate = TestVector{ 5.0f, 2.0f, 3.0f };
    tipInput.end.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    tipInput.previous.translate = TestVector{ 1.0f, 2.0f, 3.0f };
    tipInput.extrapolateFromPrevious = true;
    tipInput.extrapolatedLengthScale = 0.65f;
    const auto tipFrame = buildSegmentColliderFrame(tipInput);
    ok &= expectFloat("hand bone tip extrapolated valid", tipFrame.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand bone tip extrapolated length", tipFrame.length, 2.6f);
    ok &= expectFloat("hand bone tip center x", tipFrame.transform.translate.x, 6.3f);

    TestTransform palmHand = rock::transform_math::makeIdentityTransform<TestTransform>();
    palmHand.translate = TestVector{ 0.0f, 0.0f, 0.0f };
    const std::array<TestVector, 5> fingerBases{
        TestVector{ 2.0f, -2.0f, -1.0f },
        TestVector{ 4.0f, -1.0f, -1.0f },
        TestVector{ 4.4f, 0.0f, -1.0f },
        TestVector{ 4.0f, 1.0f, -1.0f },
        TestVector{ 3.0f, 2.0f, -1.0f }
    };
    const auto palmAnchor = buildPalmAnchorFrame(palmHand, fingerBases, TestVector{ 0.0f, 0.0f, 1.0f }, 0.75f);
    ok &= expectFloat("hand palm anchor valid", palmAnchor.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand palm anchor center x", palmAnchor.transform.translate.x, 2.9f);
    ok &= expectFloat("hand palm anchor center y", palmAnchor.transform.translate.y, 0.0f);
    ok &= expectFloat("hand palm anchor center z uses palm face", palmAnchor.transform.translate.z, -0.25f);
    ok &= expectFloat("hand palm anchor normal z points out of back", palmAnchor.backAxis.z, 1.0f);

    std::array<BoneContactPoint<TestVector>, 8> contactPoints{};
    contactPoints[0] = BoneContactPoint<TestVector>{ true, HandColliderRole::PalmFace, TestVector{ 10.0f, 0.0f, 0.0f }, TestVector{ -1.0f, 0.0f, 0.0f }, 1.0f };
    const auto palmOnlyPivot = chooseHandContactPivot(contactPoints, 1, TestVector{ 0.0f, 0.0f, 0.0f });
    ok &= expectFloat("hand contact pivot palm-only valid", palmOnlyPivot.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand contact pivot palm-only x", palmOnlyPivot.point.x, 10.0f);

    contactPoints[0] = BoneContactPoint<TestVector>{ true, HandColliderRole::ThumbTip, TestVector{ 9.0f, -2.0f, 0.0f }, TestVector{ -1.0f, 0.0f, 0.0f }, 1.0f };
    contactPoints[1] = BoneContactPoint<TestVector>{ true, HandColliderRole::IndexTip, TestVector{ 11.0f, 2.0f, 0.0f }, TestVector{ -1.0f, 0.0f, 0.0f }, 1.0f };
    const auto pinchPivot = chooseHandContactPivot(contactPoints, 2, TestVector{ 0.0f, 0.0f, 0.0f });
    ok &= expectFloat("hand contact pivot pinch valid", pinchPivot.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectFloat("hand contact pivot pinch midpoint x", pinchPivot.point.x, 10.0f);
    ok &= expectFloat("hand contact pivot pinch midpoint y", pinchPivot.point.y, 0.0f);
    ok &= expectFloat("hand contact pivot pinch confidence", pinchPivot.confidence, 1.0f);

    rock::hand_semantic_contact_state::SemanticContactSet semanticSet{};
    rock::hand_semantic_contact_state::SemanticContactRecord thumbContact{};
    thumbContact.valid = true;
    thumbContact.otherBodyId = 700;
    thumbContact.handBodyId = 77;
    thumbContact.role = HandColliderRole::ThumbTip;
    thumbContact.finger = HandFinger::Thumb;
    thumbContact.segment = HandFingerSegment::Tip;
    thumbContact.framesSinceContact = 0;
    rock::hand_semantic_contact_state::SemanticContactRecord indexContact = thumbContact;
    indexContact.handBodyId = 78;
    indexContact.role = HandColliderRole::IndexTip;
    indexContact.finger = HandFinger::Index;
    semanticSet.record(thumbContact);
    semanticSet.record(indexContact);
    semanticSet.advanceFrames();
    const auto freshContacts = semanticSet.collectFreshForBody(700, 5);
    ok &= expectFloat("semantic contact set keeps thumb and finger", static_cast<float>(freshContacts.count), 2.0f);
    const auto selectedOppositionContacts = rock::hand_semantic_contact_state::selectThumbOppositionContacts(freshContacts);
    ok &= expectFloat("semantic contact set selects opposition pair", selectedOppositionContacts.valid ? 1.0f : 0.0f, 1.0f);
    ok &= expectUint32("semantic contact set selected thumb body", selectedOppositionContacts.thumb.handBodyId, 77);
    ok &= expectUint32("semantic contact set selected opposing body", selectedOppositionContacts.opposing.handBodyId, 78);
    semanticSet.advanceFrames();
    const auto staleContacts = semanticSet.collectFreshForBody(700, 1);
    ok &= expectFloat("semantic contact set expires stale contacts", static_cast<float>(staleContacts.count), 0.0f);

    {
        using rock::grab_opposition_frame_math::OppositionFrameInput;
        using rock::grab_opposition_frame_math::buildOppositionDesiredObjectWorld;

        OppositionFrameInput<TestTransform, TestVector> oppositionInput{};
        oppositionInput.objectWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        oppositionInput.objectWorld.translate = TestVector{ 100.0f, 50.0f, -20.0f };
        oppositionInput.thumbObjectLocal = TestVector{ -2.0f, 0.0f, 0.0f };
        oppositionInput.opposingObjectLocal = TestVector{ 2.0f, 0.0f, 0.0f };
        oppositionInput.thumbHandWorld = TestVector{ 10.0f, -2.0f, 4.0f };
        oppositionInput.opposingHandWorld = TestVector{ 10.0f, 2.0f, 4.0f };
        oppositionInput.objectRollAxisLocal = TestVector{ 0.0f, 0.0f, 1.0f };
        oppositionInput.handRollAxisWorld = TestVector{ 0.0f, 0.0f, 1.0f };
        oppositionInput.enabled = true;

        const auto oppositionFrame = buildOppositionDesiredObjectWorld(oppositionInput);
        ok &= expectFloat("opposition frame valid", oppositionFrame.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("opposition frame reason", oppositionFrame.reason, "thumbOpposition");
        ok &= expectFloat("opposition frame pivot x", oppositionFrame.pivotWorld.x, 10.0f);
        ok &= expectFloat("opposition frame pivot y", oppositionFrame.pivotWorld.y, 0.0f);
        ok &= expectFloat("opposition frame pivot z", oppositionFrame.pivotWorld.z, 4.0f);
        const auto mappedThumb = rock::transform_math::localPointToWorld(oppositionFrame.desiredObjectWorld, oppositionInput.thumbObjectLocal);
        const auto mappedFinger = rock::transform_math::localPointToWorld(oppositionFrame.desiredObjectWorld, oppositionInput.opposingObjectLocal);
        const auto mappedRoll = rock::transform_math::localVectorToWorld(oppositionFrame.desiredObjectWorld, oppositionInput.objectRollAxisLocal);
        ok &= expectFloat("opposition frame maps thumb x", mappedThumb.x, oppositionInput.thumbHandWorld.x);
        ok &= expectFloat("opposition frame maps thumb y", mappedThumb.y, oppositionInput.thumbHandWorld.y);
        ok &= expectFloat("opposition frame maps thumb z", mappedThumb.z, oppositionInput.thumbHandWorld.z);
        ok &= expectFloat("opposition frame maps opposing x", mappedFinger.x, oppositionInput.opposingHandWorld.x);
        ok &= expectFloat("opposition frame maps opposing y", mappedFinger.y, oppositionInput.opposingHandWorld.y);
        ok &= expectFloat("opposition frame maps opposing z", mappedFinger.z, oppositionInput.opposingHandWorld.z);
        ok &= expectFloat("opposition frame keeps roll z", mappedRoll.z, 1.0f);

        oppositionInput.opposingHandWorld = oppositionInput.thumbHandWorld;
        const auto degenerateOppositionFrame = buildOppositionDesiredObjectWorld(oppositionInput);
        ok &= expectFloat("opposition frame rejects degenerate hand span", degenerateOppositionFrame.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("opposition frame degenerate reason", degenerateOppositionFrame.reason, "degenerateHandSpan");
    }

    rock::hand_semantic_contact_state::SemanticContactRecord semanticContact{};
    semanticContact.valid = true;
    semanticContact.otherBodyId = 700;
    semanticContact.handBodyId = 77;
    semanticContact.role = HandColliderRole::IndexTip;
    semanticContact.framesSinceContact = 2;
    const auto semanticDecision = rock::hand_semantic_contact_state::evaluateSemanticPivotCandidate(
        true,
        semanticContact,
        700,
        5);
    ok &= expectFloat("semantic pivot accepts fresh same body", semanticDecision.accept ? 1.0f : 0.0f, 1.0f);
    ok &= expectString("semantic pivot accepted reason", semanticDecision.reason, "semanticContact");
    const auto staleSemanticDecision = rock::hand_semantic_contact_state::evaluateSemanticPivotCandidate(
        true,
        semanticContact,
        700,
        1);
    ok &= expectFloat("semantic pivot rejects stale contact", staleSemanticDecision.accept ? 1.0f : 0.0f, 0.0f);
    const auto wrongBodySemanticDecision = rock::hand_semantic_contact_state::evaluateSemanticPivotCandidate(
        true,
        semanticContact,
        701,
        5);
    ok &= expectFloat("semantic pivot rejects different body", wrongBodySemanticDecision.accept ? 1.0f : 0.0f, 0.0f);

    {
        using rock::grab_multi_finger_contact_math::FingerContactPatch;
        using rock::grab_multi_finger_contact_math::GripContactSetOptions;
        using rock::grab_multi_finger_contact_math::buildGripContactSet;
        using rock::grab_multi_finger_contact_math::captureObjectToGripFrame;
        using rock::grab_multi_finger_contact_math::recomposeObjectFromGripFrame;

        GripContactSetOptions options{};
        options.enabled = true;
        options.targetBodyId = 900;
        options.minimumFingerGroups = 3;
        options.maxContactAgeFrames = 5;
        options.minimumSpreadGameUnits = 1.0f;

        std::vector<FingerContactPatch<TestVector>> contacts{
            FingerContactPatch<TestVector>{
                .valid = true,
                .finger = HandFinger::Index,
                .segment = HandFingerSegment::Tip,
                .role = HandColliderRole::IndexTip,
                .handBodyId = 101,
                .objectBodyId = 900,
                .handPointWorld = TestVector{ -1.0f, 0.0f, 0.0f },
                .objectPointWorld = TestVector{ -1.0f, 0.0f, 0.0f },
                .normalWorld = TestVector{ 0.0f, -1.0f, 0.0f },
                .quality = 1.0f,
                .framesSinceContact = 0 },
            FingerContactPatch<TestVector>{
                .valid = true,
                .finger = HandFinger::Index,
                .segment = HandFingerSegment::Middle,
                .role = HandColliderRole::IndexMiddle,
                .handBodyId = 102,
                .objectBodyId = 900,
                .handPointWorld = TestVector{ -0.8f, 0.1f, 0.0f },
                .objectPointWorld = TestVector{ -0.8f, 0.1f, 0.0f },
                .normalWorld = TestVector{ 0.0f, -1.0f, 0.0f },
                .quality = 0.5f,
                .framesSinceContact = 0 },
            FingerContactPatch<TestVector>{
                .valid = true,
                .finger = HandFinger::Middle,
                .segment = HandFingerSegment::Tip,
                .role = HandColliderRole::MiddleTip,
                .handBodyId = 103,
                .objectBodyId = 900,
                .handPointWorld = TestVector{ 0.5f, 0.0f, 0.0f },
                .objectPointWorld = TestVector{ 0.5f, 0.0f, 0.0f },
                .normalWorld = TestVector{ 0.0f, -1.0f, 0.0f },
                .quality = 1.0f,
                .framesSinceContact = 0 },
            FingerContactPatch<TestVector>{
                .valid = true,
                .finger = HandFinger::Thumb,
                .segment = HandFingerSegment::Tip,
                .role = HandColliderRole::ThumbTip,
                .handBodyId = 104,
                .objectBodyId = 900,
                .handPointWorld = TestVector{ 0.0f, 2.0f, 0.0f },
                .objectPointWorld = TestVector{ 0.0f, 2.0f, 0.0f },
                .normalWorld = TestVector{ 0.0f, 1.0f, 0.0f },
                .quality = 1.0f,
                .framesSinceContact = 0 },
        };

        const auto gripSet = buildGripContactSet(contacts, options);
        ok &= expectFloat("multi-finger grip accepts three distinct fingers", gripSet.valid ? 1.0f : 0.0f, 1.0f);
        ok &= expectFloat("multi-finger grip counts distinct fingers", static_cast<float>(gripSet.groupCount), 3.0f);
        ok &= expectString("multi-finger grip accepted reason", gripSet.reason, "validGripContactSet");
        ok &= expectFloat("multi-finger grip center x", gripSet.contactCenterWorld.x, (-1.0f + 0.5f + 0.0f) / 3.0f);
        ok &= expectFloat("multi-finger grip center y", gripSet.contactCenterWorld.y, (0.0f + 0.0f + 2.0f) / 3.0f);

        auto insufficient = contacts;
        insufficient.resize(2);
        const auto insufficientSet = buildGripContactSet(insufficient, options);
        ok &= expectFloat("multi-finger grip rejects fewer than three distinct fingers", insufficientSet.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("multi-finger grip insufficient reason", insufficientSet.reason, "insufficientFingerGroups");

        auto mixedBodies = contacts;
        mixedBodies[2].objectBodyId = 901;
        const auto mixedSet = buildGripContactSet(mixedBodies, options);
        ok &= expectFloat("multi-finger grip rejects mixed object bodies", mixedSet.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("multi-finger grip mixed-body reason", mixedSet.reason, "mixedBodies");

        auto collapsed = contacts;
        for (auto& contact : collapsed) {
            contact.objectPointWorld = TestVector{ 2.0f, 2.0f, 2.0f };
            contact.handPointWorld = TestVector{ 2.0f, 2.0f, 2.0f };
        }
        const auto collapsedSet = buildGripContactSet(collapsed, options);
        ok &= expectFloat("multi-finger grip rejects collapsed patches", collapsedSet.valid ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("multi-finger grip collapsed reason", collapsedSet.reason, "degenerateContactSpread");

        TestTransform objectWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        objectWorld.translate = TestVector{ 10.0f, 20.0f, 30.0f };
        TestTransform gripWorld = rock::transform_math::makeIdentityTransform<TestTransform>();
        gripWorld.translate = gripSet.contactCenterWorld;
        const auto capturedObjectToGrip = captureObjectToGripFrame(objectWorld, gripWorld);
        gripWorld.translate = TestVector{ 50.0f, 60.0f, 70.0f };
        const auto recomposedObject = recomposeObjectFromGripFrame(gripWorld, capturedObjectToGrip);
        ok &= expectFloat("multi-finger grip preserves captured object offset x", recomposedObject.translate.x, 50.0f + 10.0f - gripSet.contactCenterWorld.x);
        ok &= expectFloat("multi-finger grip preserves captured object offset y", recomposedObject.translate.y, 60.0f + 20.0f - gripSet.contactCenterWorld.y);
        ok &= expectFloat("multi-finger grip preserves captured object offset z", recomposedObject.translate.z, 70.0f + 30.0f - gripSet.contactCenterWorld.z);
    }

    {
        using namespace rock::grab_contact_evidence_policy;

        GrabContactEvidenceInput hybridPatchOnly{};
        hybridPatchOnly.qualityMode = static_cast<int>(GrabContactQualityMode::HybridEvidence);
        hybridPatchOnly.multiFingerValidationEnabled = true;
        hybridPatchOnly.contactPatchAccepted = true;
        hybridPatchOnly.contactPatchMeshSnapped = true;
        hybridPatchOnly.contactPatchReliable = true;
        hybridPatchOnly.contactPatchConfidence = 1.0f;
        hybridPatchOnly.minimumFingerGroups = 3;
        const auto hybridPatchOnlyDecision = evaluateGrabContactEvidence(hybridPatchOnly);
        ok &= expectFloat("hybrid evidence accepts reliable contact patch without fingers", hybridPatchOnlyDecision.accept ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("hybrid evidence patch-only level", contactEvidenceLevelName(hybridPatchOnlyDecision.level), "baselinePatch");
        ok &= expectFloat("hybrid evidence patch-only does not use multi-finger pivot", hybridPatchOnlyDecision.useMultiFingerPivot ? 1.0f : 0.0f, 0.0f);

        auto enhanced = hybridPatchOnly;
        enhanced.semanticFingerGroups = 1;
        enhanced.probeFingerGroups = 1;
        const auto enhancedDecision = evaluateGrabContactEvidence(enhanced);
        ok &= expectFloat("hybrid evidence accepts one semantic plus one probe group", enhancedDecision.accept ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("hybrid evidence enhanced level", contactEvidenceLevelName(enhancedDecision.level), "enhancedFingerPatch");
        ok &= expectFloat("hybrid evidence enhanced still keeps patch pivot", enhancedDecision.useMultiFingerPivot ? 1.0f : 0.0f, 0.0f);

        auto highConfidence = hybridPatchOnly;
        highConfidence.multiFingerGripValid = true;
        highConfidence.semanticFingerGroups = 2;
        highConfidence.probeFingerGroups = 1;
        const auto highDecision = evaluateGrabContactEvidence(highConfidence);
        ok &= expectFloat("hybrid evidence accepts high confidence multi-finger grip", highDecision.accept ? 1.0f : 0.0f, 1.0f);
        ok &= expectString("hybrid evidence high-confidence level", contactEvidenceLevelName(highDecision.level), "highConfidenceFingerGrip");
        ok &= expectFloat("hybrid evidence high-confidence uses multi-finger pivot", highDecision.useMultiFingerPivot ? 1.0f : 0.0f, 1.0f);

        auto weakPatch = hybridPatchOnly;
        weakPatch.contactPatchReliable = false;
        weakPatch.contactPatchConfidence = 0.35f;
        const auto weakPatchDecision = evaluateGrabContactEvidence(weakPatch);
        ok &= expectFloat("hybrid evidence rejects weak patch without fingers", weakPatchDecision.accept ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("hybrid evidence weak patch reason", weakPatchDecision.reason, "weakPatchWithoutFingerEvidence");

        auto strictPatchOnly = hybridPatchOnly;
        strictPatchOnly.qualityMode = static_cast<int>(GrabContactQualityMode::StrictMultiFinger);
        const auto strictPatchOnlyDecision = evaluateGrabContactEvidence(strictPatchOnly);
        ok &= expectFloat("strict evidence rejects patch-only grab", strictPatchOnlyDecision.accept ? 1.0f : 0.0f, 0.0f);
        ok &= expectString("strict evidence patch-only reason", strictPatchOnlyDecision.reason, "strictMultiFingerRequired");
    }

    ok &= expectFloat("hand palm rounded hull has box corners and face supports",
        static_cast<float>(rock::hand_bone_collider_geometry_math::makeRoundedBoxHullPoints<TestVector>(4.0f, 2.0f, 1.0f).size()),
        14.0f);

    return ok ? 0 : 1;
}
