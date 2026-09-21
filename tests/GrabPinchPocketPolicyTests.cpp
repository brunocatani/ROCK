#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <array>
#include <cstdio>
#include <cstring>
#include <limits>

namespace
{
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

    bool expectReason(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }
        std::printf("%s expected %s got %s\n", label, expected, actual ? actual : "(null)");
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon = 0.0001f)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    rock::grab_pinch_pocket_policy::MeshExtentMetrics bounds(float x, float y, float z)
    {
        return rock::grab_pinch_pocket_policy::computeMeshExtentsFromBounds(
            RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
            RE::NiPoint3{ x, y, z },
            1.0f);
    }

    rock::grab_pinch_pocket_policy::ObjectDecisionInput validInput(
        rock::grab_pinch_pocket_policy::MeshExtentMetrics mesh)
    {
        return rock::grab_pinch_pocket_policy::ObjectDecisionInput{
            .config = rock::grab_pinch_pocket_policy::Config{},
            .mesh = mesh,
            .closeGrab = true,
            .handPocketOnlyGrab = false,
            .looseWeaponGrab = false,
            .ownerMatchesResolvedBody = true,
            .hasFingerSnapshot = true,
            .hasPinchSurface = true,
            .multipleAcceptedBodies = false,
            .thumbIndexGapGameUnits = 6.0f,
            .pocketToSurfaceDistanceGameUnits = 2.0f,
        };
    }
}

int main()
{
    using namespace rock::grab_pinch_pocket_policy;

    bool ok = true;

    {
        using namespace rock::hand_bone_collider_geometry_math;
        BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
        input.previous = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        input.start = input.previous;
        input.previous.translate = { 1.0f, 0.0f, 0.0f };
        input.start.translate = { 3.0f, 0.0f, 0.0f };
        input.extrapolateFromPrevious = true;
        input.extrapolateAlongStartBoneAxis = true;
        // The distal joint bends independently of the middle-to-distal bone.
        input.start.rotate.entry[0][0] = 0.0f;
        input.start.rotate.entry[0][1] = 1.0f;
        input.start.rotate.entry[1][0] = -1.0f;
        input.start.rotate.entry[1][1] = 0.0f;
        const auto segment = buildSegmentColliderFrame(input);
        RE::NiPoint3 tip{};
        ok &= expectTrue("distal collider endpoint is available", colliderTipEndpoint(
            segment.transform.translate, segment.xAxis, segment.length, 0.1f, tip));
        ok &= expectNear("tip follows distal bend instead of middle bone", tip.x, 3.0f);
        ok &= expectNear("tip reaches the collider end including convex skin", tip.y, 1.4f);
        const auto frame = makeFingerFrame(RE::NiPoint3{ 0.0f, 2.0f, 0.0f }, tip);
        ok &= expectNear("pocket centers the terminal endpoints", frame.center.x, 1.5f);
        ok &= expectNear("pocket moves with distal bend", frame.center.y, 1.7f);
        const auto ray = detectionDirection(frame, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, 1.0f);
        ok &= expectNear("ray follows current fingertip axis", ray.x, frame.axis.x);
        ok &= expectNear("ray follows current fingertip closure", ray.y, frame.axis.y);
        ok &= expectFalse("missing collider dimensions cannot supply a tip", colliderTipEndpoint(
            segment.transform.translate, segment.xAxis, 0.0f, 0.1f, tip));
        ok &= expectFalse("coincident endpoints have no pinch axis", makeFingerFrame({}, {}).valid);
    }

    {
        auto evaluate = [](float thickness, float opening) {
            ClosureSample sample{};
            sample.fingers = makeFingerFrame(
                { 0.0f, opening * 2.0f, 0.0f }, { opening * 4.0f, opening * 2.0f, 0.0f });
            sample.thicknessGameUnits = thickness;
            Config config{};
            config.thumbIndexMaxOpenValue = opening;
            sample.pose = buildStablePinchFingerPose(config, 0.05f);
            return sample;
        };
        const auto smallObjectFit = solveClosure(0.05f, 1.0f, [&](float opening) { return evaluate(1.0f, opening); });
        const auto largeObjectFit = solveClosure(0.05f, 1.0f, [&](float opening) { return evaluate(3.0f, opening); });
        ok &= expectTrue("small and large objects find closure poses", smallObjectFit.valid && smallObjectFit.bracketed && largeObjectFit.valid && largeObjectFit.bracketed);
        ok &= expectNear("small object closes farther", smallObjectFit.sample.opening, 0.25f, 0.001f);
        ok &= expectNear("large object retains wider closure", largeObjectFit.sample.opening, 0.75f, 0.001f);
        ok &= expectNear("small-object seat follows solved fingertips", smallObjectFit.sample.fingers.center.y, 0.5f, 0.002f);
        ok &= expectNear("large-object seat follows solved fingertips", largeObjectFit.sample.fingers.center.y, 1.5f, 0.002f);
        ok &= expectNear("stored finger command matches seat solve", smallObjectFit.sample.pose.values[0], smallObjectFit.sample.opening);
        const auto limited = solveClosure(0.05f, 0.45f, [&](float opening) { return evaluate(3.0f, opening); });
        ok &= expectTrue("curl limit is reported without a new grab veto", limited.valid && !limited.bracketed);
        ok &= expectNear("unreachable thickness uses closest permitted pose", limited.sample.opening, 0.45f);
        const auto missing = solveClosure(0.05f, 1.0f, [&](float opening) {
            return opening < 0.9f ? ClosureSample{} : evaluate(1.0f, opening);
        });
        ok &= expectFalse("provider loss cannot publish a partial closure", missing.valid);
    }

    const auto longRodBounds = bounds(30.0f, 1.5f, 2.0f);
    ok &= expectNear("total bounding volume includes long dimension", longRodBounds.boundsVolumeCubicGameUnits, 90.0f);
    ok &= expectTrue("long thin object fits volume budget", evaluateObject(validInput(longRodBounds)).accept);
    ok &= expectTrue("compact object of equal volume also fits", evaluateObject(validInput(bounds(6.0f, 5.0f, 3.0f))).accept);
    ok &= expectTrue("flat wide object fits volume budget", evaluateObject(validInput(bounds(30.0f, 20.0f, 0.1f))).accept);
    ok &= expectTrue("coin still fits", evaluateObject(validInput(bounds(2.0f, 2.0f, 0.3f))).accept);
    ok &= expectTrue("small thick object has no separate thickness veto", evaluateObject(validInput(bounds(4.5f, 4.5f, 4.5f))).accept);
    ok &= expectTrue("volume boundary is inclusive", evaluateObject(validInput(bounds(10.0f, 5.0f, 2.0f))).accept);
    ok &= expectFalse("volume above boundary is rejected", evaluateObject(validInput(bounds(10.0f, 5.0f, 2.01f))).accept);
    ok &= expectFalse("large total volume is rejected", evaluateObject(validInput(bounds(16.0f, 12.0f, 8.0f))).accept);

    const auto scaledBounds = computeMeshExtentsFromBounds(
        RE::NiPoint3{}, RE::NiPoint3{ 10.0f, 5.0f, 2.0f }, 2.0f);
    ok &= expectNear("doubling scale multiplies volume by eight", scaledBounds.boundsVolumeCubicGameUnits, 800.0f);
    ok &= expectFalse("scaled-up object exceeds volume budget", evaluateObject(validInput(scaledBounds)).accept);
    auto tunedVolume = validInput(scaledBounds);
    tunedVolume.config.maxVolumeCubicGameUnits = 800.0f;
    ok &= expectTrue("configured volume limit controls acceptance", evaluateObject(tunedVolume).accept);
    ok &= expectFalse("invalid scale cannot understate volume", computeMeshExtentsFromBounds(
        RE::NiPoint3{}, RE::NiPoint3{ 10.0f, 5.0f, 2.0f }, std::numeric_limits<float>::quiet_NaN()).valid);
    ok &= expectFalse("overflowing volume fails closed", computeMeshExtentsFromBounds(
        RE::NiPoint3{}, RE::NiPoint3{ 1.0e20f, 1.0e20f, 1.0e20f }, 1.0f).valid);

    struct Triangle { RE::NiPoint3 v0, v1, v2; };
    std::array<Triangle, 2> mesh{{
        { { 0.0f, 0.0f, 0.0f }, { 1.0f, 3.0f, 0.0f }, { 1.0f, 0.0f, 2.0f } },
        { { 29.0f, 0.0f, 0.0f }, { 30.0f, 3.0f, 0.0f }, { 30.0f, 0.0f, 2.0f } },
    }};
    ok &= expectNear("volume includes separated mesh parts", computeMeshExtents(mesh, 1.0f).boundsVolumeCubicGameUnits, 180.0f);
    mesh[1].v0.x = std::numeric_limits<float>::quiet_NaN();
    ok &= expectFalse("invalid mesh part cannot silently shrink total volume", computeMeshExtents(mesh, 1.0f).valid);
    ok &= expectFalse("empty mesh has no volume evidence", computeMeshExtents(std::array<Triangle, 0>{}, 1.0f).valid);

    auto farGrab = validInput(bounds(5.0f, 4.0f, 2.0f));
    farGrab.closeGrab = false;
    auto decision = evaluateObject(farGrab);
    ok &= expectFalse("far grab rejected", decision.accept);
    ok &= expectReason("far grab reason", decision.reason, "notCloseGrab");

    auto handPocketOnly = validInput(bounds(5.0f, 4.0f, 2.0f));
    handPocketOnly.handPocketOnlyGrab = true;
    decision = evaluateObject(handPocketOnly);
    ok &= expectTrue("hand-pocket-only can use pinch geometry", decision.accept);
    ok &= expectReason("hand-pocket-only pinch reason", decision.reason, "pinchObjectVolume");

    auto looseWeapon = validInput(bounds(5.0f, 4.0f, 2.0f));
    looseWeapon.looseWeaponGrab = true;
    decision = evaluateObject(looseWeapon);
    ok &= expectTrue("loose weapon can use pinch geometry", decision.accept);
    ok &= expectReason("loose weapon pinch reason", decision.reason, "pinchObjectVolume");

    auto multiBody = validInput(bounds(5.0f, 4.0f, 2.0f));
    multiBody.multipleAcceptedBodies = true;
    decision = evaluateObject(multiBody);
    ok &= expectFalse("multi-body rejected", decision.accept);
    ok &= expectReason("multi-body reason", decision.reason, "multiBody");

    auto ownerMismatch = validInput(bounds(5.0f, 4.0f, 2.0f));
    ownerMismatch.ownerMatchesResolvedBody = false;
    decision = evaluateObject(ownerMismatch);
    ok &= expectFalse("owner mismatch rejected", decision.accept);
    ok &= expectFalse("owner mismatch is not retried", decision.retryable);
    ok &= expectReason("owner mismatch reason", decision.reason, "ownerMismatch");

    auto missingSnapshot = validInput(bounds(5.0f, 4.0f, 2.0f));
    missingSnapshot.hasFingerSnapshot = false;
    decision = evaluateObject(missingSnapshot);
    ok &= expectFalse("missing finger snapshot rejected", decision.accept);
    ok &= expectTrue("missing finger snapshot can recover", decision.retryable);
    ok &= expectReason("missing finger snapshot reason", decision.reason, "missingFingerSnapshot");

    auto gapTooWide = validInput(bounds(5.0f, 4.0f, 2.0f));
    gapTooWide.thumbIndexGapGameUnits = 20.0f;
    decision = evaluateObject(gapTooWide);
    ok &= expectFalse("wide thumb-index gap rejected", decision.accept);
    ok &= expectTrue("finger gap can recover as hand moves", decision.retryable);
    ok &= expectReason("wide thumb-index gap reason", decision.reason, "fingerGapRejected");

    auto surfaceTooFar = validInput(bounds(5.0f, 4.0f, 2.0f));
    surfaceTooFar.pocketToSurfaceDistanceGameUnits = 12.0f;
    decision = evaluateObject(surfaceTooFar);
    ok &= expectFalse("far surface rejected", decision.accept);
    ok &= expectTrue("surface distance can recover as hand moves", decision.retryable);
    ok &= expectReason("far surface reason", decision.reason, "surfaceTooFarFromPocket");

    auto noMesh = validInput(MeshExtentMetrics{});
    decision = evaluateObject(noMesh);
    ok &= expectFalse("missing mesh rejected", decision.accept);
    ok &= expectReason("missing mesh reason", decision.reason, "noMeshExtents");

    Config detectionConfig{};
    detectionConfig.detectionDirectionHandspace = RE::NiPoint3{ 0.0f, 3.0f, 4.0f };
    detectionConfig.detectionAxisBlend = 2.0f;
    auto sanitized = sanitizeConfig(detectionConfig);
    ok &= expectNear("pinch detection direction x", sanitized.detectionDirectionHandspace.x, 0.0f);
    ok &= expectNear("pinch detection direction y", sanitized.detectionDirectionHandspace.y, 0.6f);
    ok &= expectNear("pinch detection direction z", sanitized.detectionDirectionHandspace.z, 0.8f);
    ok &= expectNear("pinch detection axis blend clamped", sanitized.detectionAxisBlend, 1.0f);

    detectionConfig = Config{};
    detectionConfig.detectionDirectionHandspace = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    sanitized = sanitizeConfig(detectionConfig);
    ok &= expectNear("pinch detection fallback x", sanitized.detectionDirectionHandspace.x, kDefaultDetectionDirectionHandspaceX);
    ok &= expectNear("pinch detection fallback y", sanitized.detectionDirectionHandspace.y, kDefaultDetectionDirectionHandspaceY);
    ok &= expectNear("pinch detection fallback z", sanitized.detectionDirectionHandspace.z, kDefaultDetectionDirectionHandspaceZ);

    Config poseConfig{};
    poseConfig.thumbIndexMaxOpenValue = 0.45f;
    poseConfig.otherFingerCurlValue = 0.20f;
    const auto pinchPose = buildStablePinchFingerPose(poseConfig, 0.20f);
    ok &= expectNear("pinch thumb scalar uses configured stable value", pinchPose.values[0], 0.45f);
    ok &= expectNear("pinch index scalar uses configured stable value", pinchPose.values[1], 0.45f);
    ok &= expectNear("pinch other fingers close", pinchPose.values[2], 0.20f);
    ok &= expectTrue("pinch thumb proximal is engaged", pinchPose.jointValues[0] < 0.70f);
    ok &= expectTrue("pinch thumb distal does not over-chase", pinchPose.jointValues[2] >= 0.45f);
    ok &= expectTrue("pinch thumb curls as a chain", pinchPose.jointValues[0] > pinchPose.jointValues[1] && pinchPose.jointValues[1] > pinchPose.jointValues[2]);

    const auto pinkyOppositionPose =
        buildStableOppositionFingerPose(
            poseConfig,
            0.20f,
            4);
    ok &= expectNear(
        "pinky opposition keeps thumb at stable endpoint value",
        pinkyOppositionPose.values[0],
        0.45f);
    ok &= expectNear(
        "pinky opposition keeps pinky at stable endpoint value",
        pinkyOppositionPose.values[4],
        0.45f);
    ok &= expectNear(
        "pinky opposition closes inner fingers coherently",
        pinkyOppositionPose.values[1],
        0.20f);
    ok &= expectTrue(
        "pinky opposition articulates the pinky endpoint chain",
        pinkyOppositionPose.jointValues[12] >
                pinkyOppositionPose.jointValues[13] &&
            pinkyOppositionPose.jointValues[13] >
                pinkyOppositionPose.jointValues[14]);

    poseConfig.thumbIndexMaxOpenValue = 0.05f;
    const auto clampedPinchPose = buildStablePinchFingerPose(poseConfig, 0.30f);
    ok &= expectNear("pinch pose respects minimum thumb/index value", clampedPinchPose.values[0], 0.30f);
    ok &= expectNear("pinch thumb distal respects minimum", clampedPinchPose.jointValues[2], 0.30f);

    return ok ? 0 : 1;
}
