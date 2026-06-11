#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
    struct TestVector
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct TestTransform
    {
        struct
        {
            float entry[3][3]{};
        } rotate{};
        TestVector translate{};
        float scale = 1.0f;
    };

    bool expectBool(const char* name, bool actual, bool expected)
    {
        if (actual != expected) {
            std::printf("%s expected %s got %s\n", name, expected ? "true" : "false", actual ? "true" : "false");
            return false;
        }
        return true;
    }

    bool expectFloat(const char* name, float actual, float expected)
    {
        if (std::fabs(actual - expected) > 0.0001f) {
            std::printf("%s expected %.4f got %.4f\n", name, expected, actual);
            return false;
        }
        return true;
    }

    bool expectVectorClose(const char* name, const TestVector& actual, const TestVector& expected)
    {
        if (std::fabs(actual.x - expected.x) > 0.001f ||
            std::fabs(actual.y - expected.y) > 0.001f ||
            std::fabs(actual.z - expected.z) > 0.001f) {
            std::printf("%s expected (%.4f, %.4f, %.4f) got (%.4f, %.4f, %.4f)\n",
                name,
                expected.x,
                expected.y,
                expected.z,
                actual.x,
                actual.y,
                actual.z);
            return false;
        }
        return true;
    }

    bool expectPointClose(const char* name, const RE::NiPoint3& actual, const RE::NiPoint3& expected)
    {
        if (std::fabs(actual.x - expected.x) > 0.001f ||
            std::fabs(actual.y - expected.y) > 0.001f ||
            std::fabs(actual.z - expected.z) > 0.001f) {
            std::printf("%s expected (%.4f, %.4f, %.4f) got (%.4f, %.4f, %.4f)\n",
                name,
                expected.x,
                expected.y,
                expected.z,
                actual.x,
                actual.y,
                actual.z);
            return false;
        }
        return true;
    }
}

int main()
{
    bool ok = true;
    using namespace rock::grab_finger_local_transform_math;
    using namespace rock::grab_finger_pose_runtime;
    using rock::TriangleData;

    ok &= expectBool("full 15-bone mask is sanitized",
        sanitizeFingerLocalTransformMask(0xFFFF) == kFullFingerLocalTransformMask, true);
    ok &= expectBool("local transforms publish only with solved joint pose and full API",
        shouldPublishLocalTransformPose(true, true, true, true, true), true);
    ok &= expectBool("disabled config does not publish local transforms",
        shouldPublishLocalTransformPose(false, true, true, true, true), false);
    ok &= expectBool("unsolved pose does not publish local transforms",
        shouldPublishLocalTransformPose(true, false, true, true, true), false);
    ok &= expectBool("missing FRIK baseline API does not publish local transforms",
        shouldPublishLocalTransformPose(true, true, true, false, true), false);

    ok &= expectFloat("surface aim strength clamps to one",
        sanitizeUnitStrength(4.0f, 0.25f), 1.0f);
    ok &= expectFloat("non-finite strength uses fallback",
        sanitizeUnitStrength(std::nanf(""), 0.75f), 0.75f);
    ok &= expectFloat("negative max correction clamps to zero",
        sanitizeMaxCorrectionDegrees(-10.0f, 35.0f), 0.0f);
    ok &= expectFloat("non-finite max correction uses fallback",
        sanitizeMaxCorrectionDegrees(std::nanf(""), 35.0f), 35.0f);
    ok &= expectFloat("thumb correction uses opposition strength",
        correctionStrengthForFinger(0, 0.25f, 0.8f), 0.8f);
    ok &= expectFloat("finger correction uses surface aim strength",
        correctionStrengthForFinger(3, 0.25f, 0.8f), 0.25f);
    ok &= expectFloat("non-thumb proximal surface target correction is disabled",
        surfaceAimSegmentCorrectionWeight(2, 0), 0.0f);
    ok &= expectFloat("non-thumb distal surface target correction is disabled",
        surfaceAimSegmentCorrectionWeight(2, 2), 0.0f);
    ok &= expectFloat("thumb surface target correction keeps proximal limit",
        surfaceAimSegmentCorrectionWeight(0, 0), 0.08f);
    ok &= expectFloat("thumb surface aim correction hard-caps at five degrees",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 0, 2), kMaxSurfaceAimCorrectionRadians);
    ok &= expectFloat("non-thumb surface aim correction skips disabled distal segment",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 2, 2), 0.0f);
    ok &= expectFloat("bounded surface aim correction skips disabled proximal segment",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 2, 0), 0.0f);
    ok &= expectBool("alternate thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, true), false);
    ok &= expectBool("primary thumb keeps shared surface aim",
        shouldApplySurfaceAimCorrection(0, false), true);
    ok &= expectBool("curve-only thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, false, false), false);
    ok &= expectBool("non-thumb red target local correction is disabled",
        shouldApplySurfaceAimCorrection(2, true, false), false);
    ok &= expectBool("alternate thumb local correction needs a surface hit",
        shouldApplyAlternateThumbLocalCorrection(true, false), false);
    ok &= expectBool("alternate thumb local correction accepts real surface hit",
        shouldApplyAlternateThumbLocalCorrection(true, true), true);
    ok &= expectFloat("alternate thumb proximal correction is conservative",
        alternateThumbSegmentCorrectionStrength(0, 1.0f), 0.25f);
    ok &= expectFloat("alternate thumb distal correction is strongest",
        alternateThumbSegmentCorrectionStrength(2, 1.0f), 0.85f);
    ok &= expectFloat("alternate thumb segment strength clamps",
        alternateThumbSegmentCorrectionStrength(2, 4.0f), 0.85f);
    ok &= expectFloat("zero smoothing speed snaps to target",
        exponentialSmoothingAlpha(0.0f, 1.0f / 90.0f), 1.0f);
    const auto overOpenJoints = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(
        std::array<float, 5>{ 2.5f, 2.5f, 1.0f, 1.0f, 1.0f });
    ok &= expectFloat("thumb curl can over-open through FRIK flex range",
        overOpenJoints[1],
        rock::grab_finger_pose_math::kMaxThumbOverOpenValue);
    ok &= expectFloat("non-thumb curl remains capped at authored open",
        overOpenJoints[4],
        rock::grab_finger_pose_math::kMaxFingerOpenValue);
    std::array<float, 15> overOpenTarget{};
    overOpenTarget[1] = rock::grab_finger_pose_math::kMaxThumbOverOpenValue;
    overOpenTarget[4] = 2.5f;
    const auto snappedOverOpenJoints = rock::grab_finger_pose_math::advanceJointValues({}, overOpenTarget, 0.0f, 1.0f / 90.0f);
    ok &= expectFloat("thumb over-open survives snap sanitization",
        snappedOverOpenJoints[1],
        rock::grab_finger_pose_math::kMaxThumbOverOpenValue);
    ok &= expectFloat("non-thumb snap sanitization rejects over-open",
        snappedOverOpenJoints[4],
        rock::grab_finger_pose_math::kMaxFingerOpenValue);

    TestTransform hiddenScaleTransform{};
    hiddenScaleTransform.rotate.entry[0][0] = 1.0f;
    hiddenScaleTransform.rotate.entry[1][1] = 1.0f;
    hiddenScaleTransform.rotate.entry[2][2] = 1.0f;
    hiddenScaleTransform.scale = 0.00001f;
    ok &= expectBool("hidden FRIK node scale still has usable rotation",
        sceneTransformHasUsableBasis(hiddenScaleTransform), true);

    const TestVector openDirection{ 1.0f, 0.0f, 0.0f };
    const TestVector alternateNormal{ 0.0f, 0.0f, 1.0f };
    ok &= expectVectorClose("open alternate thumb stays on open direction",
        alternateThumbPlaneCurlDirection(openDirection, alternateNormal, 1.0f, 1.57079632679f),
        TestVector{ 1.0f, 0.0f, 0.0f });
    ok &= expectVectorClose("closed alternate thumb follows alternate plane",
        alternateThumbPlaneCurlDirection(openDirection, alternateNormal, 0.0f, 1.57079632679f),
        TestVector{ 0.0f, 1.0f, 0.0f });
    ok &= expectVectorClose("alternate thumb direction is independent of shared grab point",
        alternateThumbPlaneCurlDirection(openDirection, alternateNormal, 0.5f, 1.57079632679f),
        TestVector{ 0.7071f, 0.7071f, 0.0f });
    ok &= expectBool("alternate thumb surface guard allows tangent/on-surface correction",
        thumbAlternateSurfaceGuardAllows(
            TestVector{ 0.0f, 0.0f, 1.0f },
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            0.5f,
            true,
            true),
        true);
    ok &= expectBool("alternate thumb surface guard rejects already-inside tip",
        thumbAlternateSurfaceGuardAllows(
            TestVector{ 0.0f, 0.0f, -2.0f },
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 1.0f },
            0.5f,
            true,
            true),
        false);
    ok &= expectBool("alternate thumb surface guard rejects missing surface",
        thumbAlternateSurfaceGuardAllows(
            TestVector{ 0.0f, 0.0f, 1.0f },
            TestVector{ 1.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 0.0f },
            TestVector{ 0.0f, 0.0f, 0.0f },
            0.5f,
            true,
            false),
        false);

    ok &= expectFloat("missed generic mesh finger uses hardcoded close fallback",
        missedFingerCurlFallbackValue(false, rock::grab_finger_pose_math::FingerCurlValue::HitKind::Miss, 0.2f),
        0.3f);
    ok &= expectFloat("explicit target miss keeps configured minimum",
        missedFingerCurlFallbackValue(true, rock::grab_finger_pose_math::FingerCurlValue::HitKind::Miss, 0.2f),
        0.2f);
    ok &= expectFloat("back-surface miss opens to avoid penetration",
        missedFingerCurlFallbackValue(true, rock::grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface, 0.2f),
        1.0f);

    ok &= expectBool("pad probes require published TouchHeld-or-later pose",
        shouldRunFingerPadProbeRefinement(true, false, true, true, true),
        false);
    ok &= expectBool("pad probes require mesh finger pose enablement",
        shouldRunFingerPadProbeRefinement(false, true, true, true, true),
        false);
    ok &= expectBool("pad probes accept complete held-state inputs",
        shouldRunFingerPadProbeRefinement(true, true, true, true, true),
        true);

    SolvedGrabFingerPose pose{};
    pose.surfaceAimTarget[0] = RE::NiPoint3{ 11.0f, 2.0f, 3.0f };
    pose.surfaceAimNormal[0] = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
    pose.surfaceAimTargetValid[0] = 1;
    pose.surfaceAimNormalValid[0] = 1;
    RE::NiTransform objectAtGrab = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    objectAtGrab.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };
    captureSurfaceAimObjectLocal(pose, objectAtGrab);

    RE::NiTransform movedObject = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    movedObject.translate = RE::NiPoint3{ 20.0f, 0.0f, 0.0f };
    const auto resolvedPose = resolveSurfaceAimObjectLocal(pose, movedObject);
    ok &= expectBool("surface aim is captured object-local", pose.hasObjectLocalSurfaceAim, true);
    ok &= expectPointClose("surface aim follows moved object",
        resolvedPose.surfaceAimTarget[0],
        RE::NiPoint3{ 21.0f, 2.0f, 3.0f });
    ok &= expectPointClose("surface normal follows moved object",
        resolvedPose.surfaceAimNormal[0],
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f });

    ok &= expectFloat("surface-contact splay uses signed palm-plane angle",
        signedPalmPlaneSplayRadians(
            RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
            RE::NiPoint3{ 1.0f, 0.1f, 0.0f },
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
        std::atan2(0.1f, 1.0f));
    ok &= expectFloat("surface-contact splay preserves negative side",
        signedPalmPlaneSplayRadians(
            RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
            RE::NiPoint3{ 1.0f, -0.1f, 0.0f },
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
        std::atan2(-0.1f, 1.0f));
    ok &= expectFloat("surface-contact splay clamps large lateral targets to five degrees",
        clampSurfaceContactSplayRadians(1.0f, 0.28f),
        kMaxSurfaceContactSplayRadians);
    ok &= expectFloat("surface-contact splay default stays conservative",
        clampSurfaceContactSplayRadians(1.0f),
        kDefaultSurfaceContactSplayMaxRadians);

    rock::root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
    liveFingerSnapshot.valid = true;
    liveFingerSnapshot.palmNormalValid = true;
    liveFingerSnapshot.palmNormalWorld = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
    for (auto& chain : liveFingerSnapshot.fingers) {
        chain.points[0] = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        chain.points[1] = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        chain.points[2] = RE::NiPoint3{ 2.0f, 0.0f, 0.0f };
        chain.valid = true;
    }

    RE::NiPoint3 padCenter{};
    ok &= expectBool("pad center uses distal chain midpoint",
        computeFingerPadCenter(liveFingerSnapshot.fingers[1], padCenter),
        true);
    ok &= expectPointClose("pad center midpoint matches live chain",
        padCenter,
        RE::NiPoint3{ 1.5f, 0.0f, 0.0f });
    const auto padLandmarks = rock::root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
    SolvedGrabFingerPose directionPose{};
    directionPose.surfaceAimTarget[1] = RE::NiPoint3{ 1.5f, 2.0f, 0.0f };
    directionPose.surfaceAimTargetValid[1] = 1;
    auto padTargets = makeSharedGripPoseTarget(RE::NiPoint3{ 1.5f, 0.0f, 2.0f });
    ok &= expectPointClose("pad direction prefers current surface target",
        fingerPadProbeDirection(directionPose, padTargets, padLandmarks.fingers[1], 1, padCenter),
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
    padTargets.padProbeAimWorld = RE::NiPoint3{ 1.5f, -2.0f, 0.0f };
    padTargets.padProbeAimValid = true;
    ok &= expectPointClose("pinch-pocket pad aim overrides red surface target",
        fingerPadProbeDirection(directionPose, padTargets, padLandmarks.fingers[1], 1, padCenter),
        RE::NiPoint3{ 0.0f, -1.0f, 0.0f });
    padTargets.padProbeAimValid = false;
    directionPose.surfaceAimTargetValid[1] = 0;
    ok &= expectPointClose("pad direction falls back to grip seat",
        fingerPadProbeDirection(directionPose, padTargets, padLandmarks.fingers[1], 1, padCenter),
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

    std::array<float, 5> splayRadians{};
    SolvedGrabFingerPose splayPose{};
    ok &= expectBool("surface-contact splay rejects poses without targets",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayRadians),
        false);
    splayPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 0.2f, 0.0f };
    splayPose.surfaceAimTargetValid[1] = 1;
    ok &= expectBool("surface-contact splay builds from current target",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayRadians),
        true);
    ok &= expectFloat("surface-contact splay stores positive index offset",
        splayRadians[1],
        kMaxSurfaceContactSplayRadians);
    splayPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, -0.2f, 0.0f };
    ok &= expectBool("surface-contact splay rebuilds when target moves",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayRadians),
        true);
    ok &= expectFloat("surface-contact splay stores negative index offset",
        splayRadians[1],
        -kMaxSurfaceContactSplayRadians);

    SolvedGrabFingerPose thumbIndexCurveOnly{};
    thumbIndexCurveOnly.usedAlternateThumbSurfaceHit = true;
    thumbIndexCurveOnly.surfaceAimTargetValid[0] = 1;
    thumbIndexCurveOnly.surfaceAimNormalValid[0] = 1;
    thumbIndexCurveOnly.surfaceAimTargetValid[1] = 1;
    thumbIndexCurveOnly.surfaceAimNormalValid[1] = 1;
    thumbIndexCurveOnly.surfaceAimTargetObjectLocalValid[0] = 1;
    thumbIndexCurveOnly.surfaceAimNormalObjectLocalValid[0] = 1;
    thumbIndexCurveOnly.surfaceAimTargetObjectLocalValid[1] = 1;
    thumbIndexCurveOnly.surfaceAimNormalObjectLocalValid[1] = 1;
    thumbIndexCurveOnly.surfaceAimTargetObjectLocalValid[2] = 1;
    thumbIndexCurveOnly.hasObjectLocalSurfaceAim = true;
    useThumbIndexCurveOnlyPose(thumbIndexCurveOnly);
    ok &= expectBool("curve-only thumb-index disables thumb surface follow",
        thumbIndexCurveOnly.thumbSurfaceFollowAllowed, false);
    ok &= expectBool("curve-only thumb-index clears thumb raw surface target",
        thumbIndexCurveOnly.surfaceAimTargetValid[0] == 0, true);
    ok &= expectBool("curve-only thumb-index clears index raw surface target",
        thumbIndexCurveOnly.surfaceAimTargetValid[1] == 0, true);
    ok &= expectBool("curve-only thumb-index clears thumb object-local target",
        thumbIndexCurveOnly.surfaceAimTargetObjectLocalValid[0] == 0, true);
    ok &= expectBool("curve-only thumb-index clears index object-local target",
        thumbIndexCurveOnly.surfaceAimTargetObjectLocalValid[1] == 0, true);
    ok &= expectBool("curve-only thumb-index preserves other object-local aim state",
        thumbIndexCurveOnly.hasObjectLocalSurfaceAim, true);

    rock::root_flattened_finger_skeleton_runtime::Snapshot padProbeSnapshot = liveFingerSnapshot;
    for (auto& chain : padProbeSnapshot.fingers) {
        chain.points[0] = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        chain.points[1] = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        chain.points[2] = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };
        chain.valid = true;
    }

    std::vector<TriangleData> closePadTriangles{
        TriangleData{
            RE::NiPoint3{ 1.0f, -1.0f, 0.5f },
            RE::NiPoint3{ 3.0f, -1.0f, 0.5f },
            RE::NiPoint3{ 2.0f, 1.0f, 0.5f },
        },
    };
    RE::NiTransform padObjectWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    auto closePadTargets = makeSharedGripPoseTarget(RE::NiPoint3{ 2.0f, 0.0f, 0.5f });
    closePadTargets.useSeatPointForMissingTargets = false;
    closePadTargets.useWholeMeshForMissingTargets = true;

    SolvedGrabFingerPose padPose{};
    padPose.solved = true;
    padPose.hasJointValues = true;
    padPose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    padPose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(padPose.values);
    const float previousIndexOpenValue = padPose.values[1];
    const float previousIndexMiddleJoint = padPose.jointValues[4];
    std::array<FingerPadSurfaceEvidence, 5> closePadEvidence{};
    ok &= expectBool("pad refinement runs with complete held inputs",
        refineGrabFingerPoseWithPadProbes(
            padPose,
            closePadTriangles,
            closePadTargets,
            padProbeSnapshot,
            padObjectWorld,
            true,
            true,
            closePadEvidence),
        true);
    ok &= expectBool("pad evidence records index hit",
        closePadEvidence[1].hit,
        true);
    ok &= expectBool("pad evidence opens but never closes finger value",
        padPose.values[1] >= previousIndexOpenValue && padPose.values[1] <= 1.0f,
        true);
    ok &= expectBool("pad open bias feeds joint pose by max only",
        padPose.jointValues[4] >= previousIndexMiddleJoint && padPose.jointValues[4] <= 1.0f,
        true);
    ok &= expectBool("pad evidence can over-open thumb only",
        padPose.values[0] > 1.0f && padPose.values[0] <= rock::grab_finger_pose_math::kMaxThumbOverOpenValue,
        true);
    ok &= expectFloat("thumb over-open feeds thumb joint pose",
        padPose.jointValues[1],
        padPose.values[0]);
    ok &= expectBool("pad evidence can create surface target",
        padPose.surfaceAimTargetValid[1] != 0,
        true);
    ok &= expectPointClose("pad-created surface target uses hit point",
        padPose.surfaceAimTarget[1],
        RE::NiPoint3{ 2.0f, 0.0f, 0.5f });

    SolvedGrabFingerPose targetCaptureOnlyPose{};
    targetCaptureOnlyPose.solved = true;
    targetCaptureOnlyPose.hasJointValues = true;
    targetCaptureOnlyPose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    targetCaptureOnlyPose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(targetCaptureOnlyPose.values);
    const float captureOnlyOpenValue = targetCaptureOnlyPose.values[1];
    std::array<FingerPadSurfaceEvidence, 5> targetCaptureOnlyEvidence{};
    (void)refineGrabFingerPoseWithPadProbes(
        targetCaptureOnlyPose,
        closePadTriangles,
        closePadTargets,
        padProbeSnapshot,
        padObjectWorld,
        true,
        true,
        targetCaptureOnlyEvidence,
        true,
        false);
    ok &= expectPointClose("pad target capture can create stable target without opening",
        targetCaptureOnlyPose.surfaceAimTarget[1],
        RE::NiPoint3{ 2.0f, 0.0f, 0.5f });
    ok &= expectFloat("pad target capture leaves curl unchanged when open bias is disabled",
        targetCaptureOnlyPose.values[1],
        captureOnlyOpenValue);

    FingerPadSurfaceEvidence directBiasEvidence{};
    directBiasEvidence.hit = true;
    directBiasEvidence.distanceGameUnits = 0.25f;
    directBiasEvidence.quality = 1.0f;
    directBiasEvidence.padMayBeInsideSurface = true;
    ok &= expectBool("direct pad open bias cannot reduce openness",
        fingerPadOpenBiasValue(0.8f, directBiasEvidence) >= 0.8f,
        true);
    ok &= expectFloat("direct thumb pad over-open reaches full hFRIK flex cap",
        fingerPadThumbOverOpenValue(1.0f, directBiasEvidence),
        rock::grab_finger_pose_math::kMaxThumbOverOpenValue);
    ok &= expectFloat("direct thumb pad over-open waits for open thumb",
        fingerPadThumbOverOpenValue(0.5f, directBiasEvidence),
        0.5f);

    SolvedGrabFingerPose invalidPadPose{};
    invalidPadPose.solved = true;
    invalidPadPose.surfaceAimTarget[2] = RE::NiPoint3{ 9.0f, 0.0f, 0.0f };
    invalidPadPose.surfaceAimTargetValid[2] = 1;
    std::array<FingerPadSurfaceEvidence, 5> invalidPadEvidence{};
    ok &= expectBool("missing triangles fail pad refinement closed",
        refineGrabFingerPoseWithPadProbes(
            invalidPadPose,
            std::vector<TriangleData>{},
            closePadTargets,
            padProbeSnapshot,
            padObjectWorld,
            true,
            true,
            invalidPadEvidence),
        false);
    ok &= expectPointClose("invalid pad evidence preserves existing target",
        invalidPadPose.surfaceAimTarget[2],
        RE::NiPoint3{ 9.0f, 0.0f, 0.0f });

    SolvedGrabFingerPose replacePadPose{};
    replacePadPose.solved = true;
    replacePadPose.hasJointValues = true;
    replacePadPose.values = { 1.0f, 0.5f, 1.0f, 1.0f, 1.0f };
    replacePadPose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(replacePadPose.values);
    replacePadPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 0.0f, 5.0f };
    replacePadPose.surfaceAimTargetValid[1] = 1;
    std::array<FingerPadSurfaceEvidence, 5> replacePadEvidence{};
    (void)refineGrabFingerPoseWithPadProbes(
        replacePadPose,
        closePadTriangles,
        closePadTargets,
        padProbeSnapshot,
        padObjectWorld,
        true,
        true,
        replacePadEvidence);
    ok &= expectPointClose("better pad evidence replaces broad target",
        replacePadPose.surfaceAimTarget[1],
        RE::NiPoint3{ 2.0f, 0.0f, 0.5f });

    SolvedGrabFingerPose heldUpdatePose{};
    heldUpdatePose.solved = true;
    heldUpdatePose.hasJointValues = true;
    heldUpdatePose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    heldUpdatePose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(heldUpdatePose.values);
    heldUpdatePose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 0.0f, 5.0f };
    heldUpdatePose.surfaceAimTargetValid[1] = 1;
    const float heldUpdateOpenValue = heldUpdatePose.values[1];
    std::array<FingerPadSurfaceEvidence, 5> heldUpdateEvidence{};
    (void)refineGrabFingerPoseWithPadProbes(
        heldUpdatePose,
        closePadTriangles,
        closePadTargets,
        padProbeSnapshot,
        padObjectWorld,
        true,
        true,
        heldUpdateEvidence,
        false,
        true);
    ok &= expectPointClose("held pad update preserves captured surface target",
        heldUpdatePose.surfaceAimTarget[1],
        RE::NiPoint3{ 2.0f, 0.0f, 5.0f });
    ok &= expectBool("held pad update can still open finger",
        heldUpdatePose.values[1] >= heldUpdateOpenValue,
        true);

    std::vector<TriangleData> farPadTriangles{
        TriangleData{
            RE::NiPoint3{ 1.0f, -1.0f, 2.0f },
            RE::NiPoint3{ 3.0f, -1.0f, 2.0f },
            RE::NiPoint3{ 2.0f, 1.0f, 2.0f },
        },
    };
    auto explicitPadTargets = closePadTargets;
    explicitPadTargets.targets[1] = RE::NiPoint3{ 2.0f, 0.0f, 5.0f };
    explicitPadTargets.targetValid[1] = 1;
    explicitPadTargets.targetCount = 1;
    SolvedGrabFingerPose explicitPadPose{};
    explicitPadPose.solved = true;
    explicitPadPose.values = { 1.0f, 0.5f, 1.0f, 1.0f, 1.0f };
    explicitPadPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 0.0f, 5.0f };
    explicitPadPose.surfaceAimTargetValid[1] = 1;
    std::array<FingerPadSurfaceEvidence, 5> explicitPadEvidence{};
    (void)refineGrabFingerPoseWithPadProbes(
        explicitPadPose,
        farPadTriangles,
        explicitPadTargets,
        padProbeSnapshot,
        padObjectWorld,
        true,
        true,
        explicitPadEvidence);
    ok &= expectPointClose("non-penetrating pad evidence preserves explicit target",
        explicitPadPose.surfaceAimTarget[1],
        RE::NiPoint3{ 2.0f, 0.0f, 5.0f });

    return ok ? 0 : 1;
}
