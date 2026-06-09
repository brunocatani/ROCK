#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"

#include <array>
#include <cmath>
#include <cstdio>

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
    ok &= expectBool("alternate thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, true), false);
    ok &= expectBool("primary thumb keeps shared surface aim",
        shouldApplySurfaceAimCorrection(0, false), true);
    ok &= expectBool("curve-only thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, false, false), false);
    ok &= expectBool("alternate thumb does not disable non-thumb aim",
        shouldApplySurfaceAimCorrection(2, true, false), true);
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
    ok &= expectFloat("surface-contact splay clamps large lateral targets",
        clampSurfaceContactSplayRadians(1.0f, 0.28f),
        0.28f);

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
        std::atan2(0.2f, 2.0f));
    splayPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, -0.2f, 0.0f };
    ok &= expectBool("surface-contact splay rebuilds when target moves",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayRadians),
        true);
    ok &= expectFloat("surface-contact splay stores negative index offset",
        splayRadians[1],
        std::atan2(-0.2f, 2.0f));

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

    return ok ? 0 : 1;
}
