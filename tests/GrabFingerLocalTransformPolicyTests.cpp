#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <random>
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
        if (!std::isfinite(actual) || std::fabs(actual - expected) > 0.0001f) {
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
        if (!std::isfinite(actual.x) || !std::isfinite(actual.y) || !std::isfinite(actual.z) ||
            std::fabs(actual.x - expected.x) > 0.001f ||
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

    // A tiny sliver triangle containing the point works with any solver that
    // measures closest-point distance; orientation is irrelevant.
    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> makeSliverTriangleAtPoint(const TestVector& point)
    {
        return {
            rock::grab_finger_pose_math::Triangle<TestVector>{
                TestVector{ point.x - 0.05f, point.y, point.z },
                TestVector{ point.x + 0.05f, point.y, point.z },
                TestVector{ point.x + 0.01f, point.y + 0.01f, point.z + 0.02f },
            },
        };
    }

    std::vector<rock::grab_finger_pose_math::Triangle<TestVector>> makeClosedBox(
        const TestVector& boundsMin,
        const TestVector& boundsMax)
    {
        using Triangle = rock::grab_finger_pose_math::Triangle<TestVector>;
        const TestVector v000{ boundsMin.x, boundsMin.y, boundsMin.z };
        const TestVector v001{ boundsMin.x, boundsMin.y, boundsMax.z };
        const TestVector v010{ boundsMin.x, boundsMax.y, boundsMin.z };
        const TestVector v011{ boundsMin.x, boundsMax.y, boundsMax.z };
        const TestVector v100{ boundsMax.x, boundsMin.y, boundsMin.z };
        const TestVector v101{ boundsMax.x, boundsMin.y, boundsMax.z };
        const TestVector v110{ boundsMax.x, boundsMax.y, boundsMin.z };
        const TestVector v111{ boundsMax.x, boundsMax.y, boundsMax.z };
        return {
            Triangle{ v000, v011, v010 }, Triangle{ v000, v001, v011 },  // -X
            Triangle{ v100, v110, v111 }, Triangle{ v100, v111, v101 },  // +X
            Triangle{ v000, v100, v101 }, Triangle{ v000, v101, v001 },  // -Y
            Triangle{ v010, v011, v111 }, Triangle{ v010, v111, v110 },  // +Y
            Triangle{ v000, v010, v110 }, Triangle{ v000, v110, v100 },  // -Z
            Triangle{ v001, v101, v111 }, Triangle{ v001, v111, v011 },  // +Z
        };
    }
}

static bool testPublicationSafety()
{
    using namespace rock::grab_finger_local_transform_math;
    bool ok = true;
    TestTransform hiddenScaleTransform{};
    hiddenScaleTransform.rotate.entry[0][0] = 1.0f;
    hiddenScaleTransform.rotate.entry[1][1] = 1.0f;
    hiddenScaleTransform.rotate.entry[2][2] = 1.0f;
    hiddenScaleTransform.scale = 0.00001f;
    ok &= expectBool("hidden FRIK node scale still has usable rotation",
        sceneTransformHasUsableBasis(hiddenScaleTransform), true);

    TestTransform safePublishedFingerTransform = hiddenScaleTransform;
    safePublishedFingerTransform.scale = 1.0f;
    ok &= expectBool("rigid local finger transform is safe to publish",
        inspectFingerLocalTransformForPublication(safePublishedFingerTransform) == FingerLocalTransformSafetyFailure::None, true);

    TestTransform hiddenPublishedFingerTransform = safePublishedFingerTransform;
    hiddenPublishedFingerTransform.scale = 0.00001f;
    ok &= expectBool("near-zero local finger scale is rejected before publication",
        inspectFingerLocalTransformForPublication(hiddenPublishedFingerTransform) == FingerLocalTransformSafetyFailure::ScaleMagnitude, true);

    TestTransform giantPublishedFingerTransform = safePublishedFingerTransform;
    giantPublishedFingerTransform.scale = 1000.0f;
    ok &= expectBool("giant local finger scale is rejected before publication",
        inspectFingerLocalTransformForPublication(giantPublishedFingerTransform) == FingerLocalTransformSafetyFailure::ScaleMagnitude, true);

    TestTransform distantPublishedFingerTransform = safePublishedFingerTransform;
    distantPublishedFingerTransform.translate.x = kMaxPublishedFingerLocalTranslationGameUnits + 1.0f;
    ok &= expectBool("world-sized local finger translation is rejected before publication",
        inspectFingerLocalTransformForPublication(distantPublishedFingerTransform) == FingerLocalTransformSafetyFailure::TranslationMagnitude, true);

    TestTransform shearedPublishedFingerTransform = safePublishedFingerTransform;
    shearedPublishedFingerTransform.rotate.entry[0][1] = 0.5f;
    ok &= expectBool("sheared local finger basis is rejected before publication",
        inspectFingerLocalTransformForPublication(shearedPublishedFingerTransform) == FingerLocalTransformSafetyFailure::RotationNorm, true);

    TestTransform reflectedPublishedFingerTransform = safePublishedFingerTransform;
    reflectedPublishedFingerTransform.rotate.entry[2][2] = -1.0f;
    ok &= expectBool("reflected local finger basis is rejected before publication",
        inspectFingerLocalTransformForPublication(reflectedPublishedFingerTransform) == FingerLocalTransformSafetyFailure::RotationDeterminant, true);

    {
        rock::frik_visual_authority::FingerLocalTransformOverride target{};
        target.enabledMask = kFullFingerLocalTransformMask;
        for (auto& local : target.localTransforms) {
            local = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        }

        rock::grab_finger_local_transform_runtime::State poisonedState{};
        poisonedState.currentMask = kFullFingerLocalTransformMask;
        poisonedState.hasCurrentTransforms = true;
        for (std::size_t index = 0; index < poisonedState.currentTransforms.size(); ++index) {
            poisonedState.currentTransforms[index] = target.localTransforms[index];
        }
        poisonedState.currentTransforms[0].rotate.entry[0][0] = 1000.0f;
        const auto recovered = rock::grab_finger_local_transform_runtime::smoothLocalTransforms(
            target,
            poisonedState,
            14.0f,
            1.0f / 90.0f,
            false);
        ok &= expectBool("poisoned dynamic-grab smoothing state resets to the safe target",
            rock::grab_finger_local_transform_runtime::fingerLocalTransformOverrideIsSafeForPublication(recovered), true);
        ok &= expectBool("poisoned dynamic-grab smoothing state remains recovered",
            inspectFingerLocalTransformForPublication(poisonedState.currentTransforms[0]) == FingerLocalTransformSafetyFailure::None, true);
    }

    return ok;
}

static bool testCommandedFingerCorrectionFrame()
{
    using namespace rock;
    using namespace grab_finger_local_transform_runtime;
    bool ok = true;
    for (const bool isLeft : { false, true }) {
        DirectSkeletonBoneSnapshot snapshot{};
        snapshot.valid = true;
        auto hand = transform_math::makeIdentityTransform<RE::NiTransform>();
        hand.translate = { -79200.0f, 90300.0f, 7920.0f };
        hand.rotate = axisAngleStored({ 0.0f, 0.0f, 1.0f }, isLeft ? -0.6f : 0.6f);
        snapshot.bones.push_back({ .name = isLeft ? "LArm_Hand" : "RArm_Hand", .treeIndex = 0, .world = hand });
        frik_visual_authority::FingerLocalTransformOverride baseline{};
        baseline.enabledMask = grab_finger_local_transform_math::kFullFingerLocalTransformMask;
        for (std::size_t index = 0; index < 15; ++index) {
            baseline.localTransforms[index] = transform_math::makeIdentityTransform<RE::NiTransform>();
            baseline.localTransforms[index].translate = { 2.0f, 0.0f, 0.0f };
            baseline.localTransforms[index].rotate = axisAngleStored({ 0.0f, 0.0f, 1.0f }, 0.4f);
            snapshot.bones.push_back({
                .name = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, index / 3, index % 3),
                .treeIndex = static_cast<int>(index + 1),
                .parentTreeIndex = index % 3 == 0 ? 0 : static_cast<int>(index),
                .world = hand,
            });
        }

        std::array<LiveFingerTransform, 15> nodes{};
        ok &= expectBool("requested finger frame resolves", resolveFingerTransforms(snapshot, isLeft, baseline, nodes), true);
        const auto first = transform_math::composeTransforms(hand, baseline.localTransforms[0]);
        const auto second = transform_math::composeTransforms(first, baseline.localTransforms[1]);
        ok &= expectPointClose("proximal uses requested rest offset", nodes[0].world.translate, first.translate);
        ok &= expectPointClose("middle inherits requested proximal pose", nodes[1].world.translate, second.translate);
        ok &= expectFloat("requested curl replaces displayed pose as correction source", nodes[0].world.rotate.entry[0][0], first.rotate.entry[0][0]);

        // Displayed poses can differ during interpolation or after a prior
        // grab. Even a poisoned displayed thumb must not change this solve.
        snapshot.bones[1].world.rotate.entry[0][0] = std::numeric_limits<float>::quiet_NaN();
        snapshot.bones[2].world.translate.x = 1.0e20f;
        snapshot.bones[3].world.scale = 0.00001f;
        for (int frame = 0; frame < 2000; ++frame) {
            ok &= expectBool("displayed finger state is excluded", resolveFingerTransforms(snapshot, isLeft, baseline, nodes), true);
            ok &= expectFloat("repeated capture preserves requested curl", nodes[0].world.rotate.entry[0][0], first.rotate.entry[0][0]);
            ok &= expectBool("correction frame stays rigid", transform_math::storedRotationOrthonormalityError(nodes[2].world.rotate) < 0.000001, true);
        }

        auto corrected = baseline;
        corrected.localTransforms[0].rotate = axisAngleStored({ 0.0f, 1.0f, 0.0f }, 0.7f);
        updatePoseFingerTransform(0, corrected, nodes);
        updatePoseFingerTransform(1, corrected, nodes);
        const auto correctedFirst = transform_math::composeTransforms(hand, corrected.localTransforms[0]);
        const auto correctedSecond = transform_math::composeTransforms(correctedFirst, baseline.localTransforms[1]);
        ok &= expectPointClose("child correction follows corrected parent", nodes[1].world.translate, correctedSecond.translate);
        ok &= expectFloat("correction preserves authored scale", corrected.localTransforms[0].scale, baseline.localTransforms[0].scale);

        snapshot.bones[2].parentTreeIndex = 0;
        const char* reason = nullptr;
        std::size_t failureIndex = 15;
        ok &= expectBool("unexpected finger parent fails closed", resolveFingerTransforms(snapshot, isLeft, baseline, nodes, &reason, &failureIndex), false);
        ok &= expectBool("chain failure identifies joint", failureIndex == 1, true);
        snapshot.bones[2].parentTreeIndex = 1;
        snapshot.bones[0].world.rotate.entry[0][0] = 100.0f;
        ok &= expectBool("invalid hand root fails closed", resolveFingerTransforms(snapshot, isLeft, baseline, nodes), false);
    }

    // Reaching an alternate thumb lane exactly must retain the full pose,
    // rather than report a correction failure and discard other fingers.
    frik_visual_authority::FingerLocalTransformOverride baseline{};
    baseline.enabledMask = grab_finger_local_transform_math::kFullFingerLocalTransformMask;
    std::array<LiveFingerTransform, 15> nodes{};
    for (std::size_t index = 0; index < 15; ++index) {
        baseline.localTransforms[index] = transform_math::makeIdentityTransform<RE::NiTransform>();
        nodes[index] = { .world = baseline.localTransforms[index], .parentWorld = baseline.localTransforms[index], .valid = true };
    }
    grab_finger_pose_runtime::SolvedGrabFingerPose pose{};
    pose.usedAlternateThumbCurve = true;
    pose.usedAlternateThumbSurfaceHit = true;
    pose.hasThumbAlternateCurveFrame = true;
    pose.thumbAlternateCurveOpenDirectionWorld = { 1.0f, 0.0f, 0.0f };
    pose.thumbAlternateCurveNormalWorld = { 0.0f, 0.0f, 1.0f };
    pose.values[0] = 1.0f;
    ok &= expectBool("aligned alternate thumb lane succeeds", applyAlternateThumbPlaneCorrection(false, pose, nodes, 0.3f, 1.0f, false, 1.0f, baseline), true);
    ok &= expectBool("aligned alternate thumb pose stays safe", fingerLocalTransformOverrideIsSafeForPublication(baseline), true);
    return ok;
}

bool testIndexedFingerPadProbes()
{
    using namespace rock;
    using namespace grab_finger_pose_runtime;
    const auto equalPoint = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    };
    const auto equalEvidence = [&](const FingerPadSurfaceEvidence& a, const FingerPadSurfaceEvidence& b) {
        return equalPoint(a.startWorld, b.startWorld) && equalPoint(a.endWorld, b.endWorld) &&
            equalPoint(a.hitPointWorld, b.hitPointWorld) && equalPoint(a.hitNormalWorld, b.hitNormalWorld) &&
            a.distanceGameUnits == b.distanceGameUnits && a.quality == b.quality && a.hit == b.hit &&
            a.fromClosestSurface == b.fromClosestSurface && a.padMayBeInsideSurface == b.padMayBeInsideSurface;
    };
    std::mt19937 random(81239);
    std::uniform_real_distribution<float> coordinate(-100.0f, 100.0f);
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
    std::vector<TriangleData> local;
    for (int i = 0; i < 2048; ++i) {
        const RE::NiPoint3 p{ coordinate(random), coordinate(random), coordinate(random) };
        local.push_back({ p, p + RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, p + RE::NiPoint3{ 0.0f, 2.0f, 0.5f } });
    }
    // Duplicate and reversed faces exercise source-order ties. Degenerate and
    // non-finite inputs exercise the same finite-input admission as the caller.
    local[14] = local[13];
    local[15] = { local[13].v0, local[13].v2, local[13].v1 };
    local[32] = {};
    local[33].v0.x = std::numeric_limits<float>::quiet_NaN();
    FingerPoseTriangleSpatialIndex index;
    if (!index.buildFromLocalTriangles(local)) return false;
    std::uint64_t linearTests = 0, indexedTests = 0;
    int compared = 0;
    for (const float scale : { 1.0f, 0.1f, 3.0f, -2.0f }) {
        auto world = transform_math::makeIdentityTransform<RE::NiTransform>();
        world.scale = scale;
        world.translate = { 12000.0f, -23000.0f, 9000.0f };
        world.rotate.entry[0][0] = 0.8f;
        world.rotate.entry[0][1] = 0.6f;
        world.rotate.entry[1][0] = -0.6f;
        world.rotate.entry[1][1] = 0.8f;
        world.rotate.entry[2][0] = 0.0001f; // finite authored basis drift
        std::vector<TriangleData> triangles;
        rebuildBoundedWorldTriangles(local, world, triangles);
        if (triangles.size() != index.triangleCount()) return false;
        for (int query = 0; query < 160; ++query) {
            const auto& triangle = triangles[(query * 13) % triangles.size()];
            const auto point = query < 120 ? triangle.v0 + RE::NiPoint3{ unit(random), unit(random), unit(random) } :
                transform_math::localPointToWorld(world, RE::NiPoint3{ coordinate(random), coordinate(random), coordinate(random) });
            const RE::NiPoint3 direction{ unit(random), unit(random), unit(random) };
            const RE::NiPoint3 preferred{ unit(random), unit(random), unit(random) };
            FingerPadProbeOptions options;
            options.probeRadiusGameUnits = 0.1f + std::abs(unit(random));
            options.probeDistanceGameUnits = 0.2f + 12.0f * std::abs(unit(random));
            options.closestSurfaceMaxDistanceGameUnits = 0.2f + 5.0f * std::abs(unit(random));
            FingerPoseSpatialQueryStats linearStats{}, indexedStats{};
            const auto baseline = solveFingerPadSurfaceEvidenceFromTriangles(triangles, point, direction, preferred, query % 2 != 0,
                options, nullptr, nullptr, &linearStats);
            const auto accelerated = solveFingerPadSurfaceEvidenceFromTriangles(triangles, point, direction, preferred, query % 2 != 0,
                options, &index, &world, &indexedStats);
            if (!equalEvidence(baseline, accelerated)) {
                std::printf("Indexed finger pad mismatch scale=%f query=%d\n", scale, query);
                return false;
            }
            linearTests += linearStats.triangleTests;
            indexedTests += indexedStats.triangleTests;
            ++compared;
        }
    }
    // Exact tangent, coincident surfaces, capsule-only contact and clear misses.
    local = { { {0, 0, 0}, {4, 0, 0}, {0, 4, 0} },
        { {0, 0, 0}, {0, 4, 0}, {4, 0, 0} } };
    index.buildFromLocalTriangles(local);
    const auto identity = transform_math::makeIdentityTransform<RE::NiTransform>();
    for (const auto point : { RE::NiPoint3{1, 1, 1}, RE::NiPoint3{1, 1, 0}, RE::NiPoint3{4.5f, 0, 1}, RE::NiPoint3{100, 100, 100} }) {
        const auto baseline = solveFingerPadSurfaceEvidenceFromTriangles(local, point, {0, 0, -1}, {}, false);
        const auto accelerated = solveFingerPadSurfaceEvidenceFromTriangles(local, point, {0, 0, -1}, {}, false, {}, &index, &identity);
        if (!equalEvidence(baseline, accelerated)) return false;
    }
    index.clear();
    if (index.buildFromLocalTriangles(std::vector<TriangleData>{})) return false;
    std::printf("Finger pad parity: %d randomized queries, %llu linear vs %llu indexed triangle tests\n",
        compared, static_cast<unsigned long long>(linearTests), static_cast<unsigned long long>(indexedTests));
    return indexedTests < linearTests / 3;
}

int main()
{
    bool ok = testIndexedFingerPadProbes();
    {
        rock::AuthoredWeaponGripPose pose{};
        ok &= expectBool("empty authored transfer pose fails closed", pose.valid(), false);
        pose.role = rock::loose_weapon_authored_grab_policy::Role::Support;
        pose.weaponFormId = 0x1234;
        pose.isLeft = true;
        pose.fingerMask = 0x7FFF;
        pose.handWeaponLocal = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        pose.placementHandWeaponLocal = pose.handWeaponLocal;
        pose.placementHandWeaponLocal.translate = { 2.0f, 3.0f, -1.0f };
        pose.fingerLocals.fill(pose.handWeaponLocal);
        ok &= expectBool("complete authored support transfer pose is valid", pose.valid(), true);
        const auto snapshot = pose;
        pose.fingerMask = 0x3FFF;
        ok &= expectBool("partial authored fingers reject transfer", pose.valid(), false);
        ok &= expectBool("transfer snapshot owns an independent value copy", snapshot.valid(), true);
        pose = snapshot;
        pose.handWeaponLocal.rotate.entry[1] = {};
        ok &= expectBool("singular authored hand relation rejects transfer", pose.valid(), false);
        pose = snapshot;
        pose.placementHandWeaponLocal.scale = 0.0f;
        ok &= expectBool("invalid physical placement rejects transfer", pose.valid(), false);
        pose = snapshot;
        pose.fingerLocals[14].translate.x = std::numeric_limits<float>::quiet_NaN();
        ok &= expectBool("nonfinite authored finger rejects transfer", pose.valid(), false);
    }
    {
        namespace transfer = rock::weapon_grip_transfer;
        const auto identity = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        transfer::Pair pair{};
        ok &= expectBool("empty paired grip cannot transfer", pair.valid(), false);
        pair.weaponFormID = 0x1234;
        pair.primary.handWeaponLocal = identity;
        pair.support.handWeaponLocal = identity;
        pair.primary.hasFingerPose = true;
        pair.support.hasFingerPose = true;
        pair.support.gripWeaponLocal = { 0.0f, 20.0f, 0.0f };
        ok &= expectBool("dynamic curls can transfer without authored bone locals", pair.valid(), true);
        const auto captured = pair;
        pair.support.fingerMask = 1;
        pair.support.fingerLocals[0] = identity;
        ok &= expectBool("partial dynamic finger locals remain supported", pair.valid(), true);
        pair.support.fingerLocals[0].scale = 0.0f;
        ok &= expectBool("invalid peer fingers reject the whole paired transfer", pair.valid(), false);
        ok &= expectBool("captured pair survives source cleanup independently", captured.valid(), true);
        pair = captured;
        pair.primary.handWeaponLocal.rotate.entry[1] = {};
        ok &= expectBool("singular primary capture rejects paired equip", pair.valid(), false);

        auto bodyInWeapon = identity;
        bodyInWeapon.rotate.entry[0] = { 0.0f, 1.0f, 0.0f, 0.0f };
        bodyInWeapon.rotate.entry[1] = { -1.0f, 0.0f, 0.0f, 0.0f };
        bodyInWeapon.translate = { 8.0f, 2.0f, -1.0f };
        auto bodyInObject = identity;
        bodyInObject.translate = { 3.0f, 1.0f, 0.0f };
        auto objectInHand = identity;
        objectInHand.translate = { 0.0f, -4.0f, 0.0f };
        const auto local = transfer::handInWeapon(bodyInWeapon, bodyInObject, objectInHand);
        ok &= expectFloat("nested held body transfers wrist X", local.translate.x, 5.0f);
        ok &= expectFloat("nested held body transfers wrist Y", local.translate.y, -1.0f);
        ok &= expectFloat("nested held body transfers wrist Z", local.translate.z, -1.0f);
        ok &= expectFloat("nested held body transfers wrist rotation", local.rotate.entry[0].y, 1.0f);
        static_assert(transfer::requesterIsPrimary(true, false, 9, 2));
        static_assert(!transfer::requesterIsPrimary(false, true, 2, 9));
        static_assert(transfer::requesterIsPrimary(false, false, 2, 9));
        static_assert(!transfer::requesterIsPrimary(true, true, 9, 2));
    }
    {
        const auto identity = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        auto handInWeapon = identity;
        handInWeapon.rotate.entry[0] = { 0.0f, 1.0f, 0.0f, 0.0f };
        handInWeapon.rotate.entry[1] = { -1.0f, 0.0f, 0.0f, 0.0f };
        handInWeapon.translate = { 4.0f, -6.0f, 2.0f };
        handInWeapon.scale = 2.0f;
        const RE::NiPoint3 grip{ 1.0f, 3.0f, -2.0f };
        const RE::NiPoint3 tuning{ 1.0f, 2.0f, 3.0f };
        const auto delta = rock::weapon_grip_calibration::offsetInWeapon(handInWeapon, tuning, true);
        ok &= expectFloat("grip tuning adds to calibrated hand Y through rotated weapon", delta.x, -5.2f);
        ok &= expectFloat("grip tuning follows fingers through rotated weapon", delta.y, 2.0f);
        ok &= expectFloat("grip tuning uses authored cross-palm sign and scale", delta.z, -6.0f);
        const auto shifted = rock::weapon_grip_calibration::shiftedHand(handInWeapon, tuning, true);
        const auto shiftedGrip = grip + delta;
        const auto oldGripInHand = rock::transform_math::worldPointToLocal(handInWeapon, grip);
        const auto newGripInHand = rock::transform_math::worldPointToLocal(shifted, shiftedGrip);
        ok &= expectFloat("calibrated firing point keeps physical hand X anchor", newGripInHand.x, oldGripInHand.x);
        ok &= expectFloat("calibrated firing point keeps physical hand Y anchor", newGripInHand.y, oldGripInHand.y);
        ok &= expectFloat("calibrated firing point keeps physical hand Z anchor", newGripInHand.z, oldGripInHand.z);
        auto weapon = identity;
        weapon.translate = { 20.0f, 30.0f, 40.0f };
        weapon.rotate = handInWeapon.rotate;
        weapon.scale = 1.5f;
        const auto oldHandWorld = rock::transform_math::composeTransforms(weapon, handInWeapon);
        const auto targetPalm = rock::transform_math::localPointToWorld(oldHandWorld, newGripInHand);
        const auto movedWeapon = rock::left_firing_position_only_math::resolveWeaponWorldPositionOnly(
            identity, weapon, weapon, shiftedGrip, targetPalm);
        const auto newHandWorld = rock::transform_math::composeTransforms(movedWeapon, shifted);
        ok &= expectFloat("relative grip tuning leaves firing hand world X fixed", newHandWorld.translate.x, oldHandWorld.translate.x);
        ok &= expectFloat("relative grip tuning leaves firing hand world Y fixed", newHandWorld.translate.y, oldHandWorld.translate.y);
        ok &= expectFloat("relative grip tuning leaves firing hand world Z fixed", newHandWorld.translate.z, oldHandWorld.translate.z);
        ok &= expectBool("relative grip tuning moves the weapon", movedWeapon.translate != weapon.translate, true);
        ok &= expectFloat("relative grip tuning preserves weapon orientation", movedWeapon.rotate.entry[0].y, weapon.rotate.entry[0].y);
        const auto zeroFiring = rock::weapon_grip_calibration::shiftedHand(handInWeapon, RE::NiPoint3{}, true);
        ok &= expectFloat("zero firing tuning retains calibrated palm-depth correction", zeroFiring.translate.x, handInWeapon.translate.x - 1.2f);
        ok &= expectFloat("firing calibration does not shift the finger axis", zeroFiring.translate.y, handInWeapon.translate.y);
        const auto zeroSupport = rock::weapon_grip_calibration::shiftedHand(handInWeapon, RE::NiPoint3{}, false);
        ok &= expectFloat("zero support tuning retains calibrated finger-axis correction", zeroSupport.translate.y, handInWeapon.translate.y - 1.8f);
        ok &= expectFloat("support calibration does not shift palm depth", zeroSupport.translate.x, handInWeapon.translate.x);
        const auto cancelledFiring = rock::weapon_grip_calibration::shiftedHand(handInWeapon, RE::NiPoint3{ 0.0f, -0.6f, 0.0f }, true);
        const auto cancelledSupport = rock::weapon_grip_calibration::shiftedHand(handInWeapon, RE::NiPoint3{ 0.9f, 0.0f, 0.0f }, false);
        ok &= expectBool("firing tuning can cancel its baseline once", cancelledFiring.translate == handInWeapon.translate, true);
        ok &= expectBool("support tuning can cancel its baseline once", cancelledSupport.translate == handInWeapon.translate, true);
        const auto support = rock::weapon_grip_calibration::shiftedHand(handInWeapon, tuning, false);
        ok &= expectFloat("right support tuning adds to its own hand-local calibration", support.translate.y, handInWeapon.translate.y + 0.2f);
    }
    ok &= testCommandedFingerCorrectionFrame();
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
    ok &= expectFloat("middle proximal surface target correction is bounded",
        surfaceAimSegmentCorrectionWeight(2, 0), 0.05f);
    ok &= expectFloat("middle distal surface target correction is bounded",
        surfaceAimSegmentCorrectionWeight(2, 2), 0.20f);
    ok &= expectFloat("thumb surface target correction keeps proximal limit",
        surfaceAimSegmentCorrectionWeight(0, 0), 0.08f);
    ok &= expectFloat("thumb surface aim correction hard-caps at five degrees",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 0, 2), kMaxSurfaceAimCorrectionRadians);
    ok &= expectFloat("middle distal surface aim correction keeps global cap",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 2, 2), kMaxSurfaceAimCorrectionRadians);
    ok &= expectFloat("middle proximal surface aim correction keeps segment bound",
        boundedSurfaceAimCorrectionRadians(1.0f, 1.0f, 0.5f, 2, 0), 0.025f);
    ok &= expectBool("alternate thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, true), false);
    ok &= expectBool("primary thumb keeps shared surface aim",
        shouldApplySurfaceAimCorrection(0, false), true);
    ok &= expectBool("curve-only thumb skips shared surface aim",
        shouldApplySurfaceAimCorrection(0, false, false), false);
    ok &= expectBool("alternate-plane transaction skips middle surface aim",
        shouldApplySurfaceAimCorrection(2, true, false), false);
    ok &= expectBool("middle surface target local correction is enabled",
        shouldApplySurfaceAimCorrection(2, false, false), true);
    ok &= expectBool("index remains curve-only",
        shouldApplySurfaceAimCorrection(1, false, true), false);
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
        rock::grab_finger_pose_math::kMaxOverOpenValue);
    ok &= expectFloat("non-thumb curl can over-open through FRIK flex range",
        overOpenJoints[4],
        rock::grab_finger_pose_math::kMaxOverOpenValue);

    const RE::NiPoint3 liveProximalBase{ 1.0f, 2.0f, 3.0f };
    const RE::NiPoint3 seatPoint{ 7.0f, 8.0f, 9.0f };
    const RE::NiPoint3 grabAnchor{ 4.0f, 4.0f, 4.0f };
    ok &= expectPointClose("already-seated mesh keeps sweep pivot on live proximal bone",
        resolveFingerSweepBaseWorld(
            liveProximalBase, seatPoint, grabAnchor, true, 0, FingerPoseMeshRelation::AlreadyAtCommandedSeat),
        liveProximalBase);
    ok &= expectPointClose("current mesh receives one explicit virtual-seat shift",
        resolveFingerSweepBaseWorld(
            liveProximalBase, seatPoint, grabAnchor, true, 0, FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat),
        RE::NiPoint3{ 4.0f, 6.0f, 8.0f });
    ok &= expectPointClose("explicit pinch targets never shift the proximal pivot",
        resolveFingerSweepBaseWorld(
            liveProximalBase, seatPoint, grabAnchor, true, 2, FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat),
        liveProximalBase);
    const auto overOpenUniformJoints = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(
        std::array<float, 5>{ 1.0f, 1.5f, 1.0f, 1.0f, 1.0f });
    ok &= expectFloat("over-open joints hyper-extend uniformly (no proximal bias)",
        overOpenUniformJoints[3],
        1.5f);
    ok &= expectFloat("over-open joints hyper-extend uniformly (no distal bias)",
        overOpenUniformJoints[5],
        1.5f);
    std::array<float, 15> overOpenTarget{};
    overOpenTarget[1] = rock::grab_finger_pose_math::kMaxOverOpenValue;
    overOpenTarget[4] = 2.5f;
    const auto snappedOverOpenJoints = rock::grab_finger_pose_math::advanceJointValues({}, overOpenTarget, 0.0f, 1.0f / 90.0f);
    ok &= expectFloat("thumb over-open survives snap sanitization",
        snappedOverOpenJoints[1],
        rock::grab_finger_pose_math::kMaxOverOpenValue);
    ok &= expectFloat("non-thumb over-open snaps to the FRIK flex ceiling",
        snappedOverOpenJoints[4],
        rock::grab_finger_pose_math::kMaxOverOpenValue);

    ok &= testPublicationSafety();

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

    const std::vector<rock::TriangleData> oppositionMesh{
        rock::TriangleData{
            RE::NiPoint3{ 0.0f, -2.0f, -2.0f },
            RE::NiPoint3{ 0.0f, 2.0f, -2.0f },
            RE::NiPoint3{ 0.0f, 0.0f, 2.0f },
        },
    };
    auto thumbIndexPocketSnapshot = liveFingerSnapshot;
    thumbIndexPocketSnapshot.fingers[0].points[2] =
        RE::NiPoint3{ -3.0f, 0.0f, 0.0f };
    thumbIndexPocketSnapshot.fingers[1].points[2] =
        RE::NiPoint3{ 3.0f, 0.0f, 0.0f };
    const auto thumbIndexPocket = findLocalOppositionPocketEvidence(
        oppositionMesh,
        thumbIndexPocketSnapshot,
        RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
        0x01,
        1.0f,
        12.0f,
        4.0f);
    ok &= expectBool(
        "local thumb-index pocket accepts a crossed nearby weapon surface",
        thumbIndexPocket.valid &&
            thumbIndexPocket.kind == OppositionPocketKind::ThumbIndex,
        true);
    ok &= expectBool(
        "local opposition pocket records only direct endpoint evidence",
        thumbIndexPocket.directEndpointMask == 0x01 &&
            thumbIndexPocket.endpointMask == 0x03,
        true);
    ok &= expectBool(
        "local opposition pocket rejects a crossed surface without direct endpoint evidence",
        findLocalOppositionPocketEvidence(
            oppositionMesh,
            thumbIndexPocketSnapshot,
            RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
            0x00,
            1.0f,
            12.0f,
            4.0f)
            .valid,
        false);
    ok &= expectBool(
        "local opposition pocket rejects a surface outside the captured grip neighborhood",
        findLocalOppositionPocketEvidence(
            oppositionMesh,
            thumbIndexPocketSnapshot,
            RE::NiPoint3{ 0.0f, 20.0f, 0.0f },
            0x01,
            1.0f,
            12.0f,
            4.0f)
            .valid,
        false);

    auto thumbPinkyPocketSnapshot = liveFingerSnapshot;
    thumbPinkyPocketSnapshot.fingers[0].points[2] =
        RE::NiPoint3{ -5.0f, 0.0f, 0.0f };
    thumbPinkyPocketSnapshot.fingers[1].points[2] =
        RE::NiPoint3{ -5.0f, 5.0f, 0.0f };
    thumbPinkyPocketSnapshot.fingers[4].points[2] =
        RE::NiPoint3{ 5.0f, 0.0f, 0.0f };
    const auto thumbPinkyPocket = findLocalOppositionPocketEvidence(
        oppositionMesh,
        thumbPinkyPocketSnapshot,
        RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
        0x10,
        1.0f,
        24.0f,
        4.0f);
    ok &= expectBool(
        "local thumb-pinky pocket accepts a crossed nearby weapon surface",
        thumbPinkyPocket.valid &&
            thumbPinkyPocket.kind == OppositionPocketKind::ThumbPinky &&
            thumbPinkyPocket.opposedFingerIndex == 4,
        true);

    RE::NiPoint3 padCenter{};
    ok &= expectBool("pad center uses distal chain point",
        computeFingerPadCenter(liveFingerSnapshot.fingers[1], padCenter),
        true);
    ok &= expectPointClose("pad center matches distal live chain point",
        padCenter,
        RE::NiPoint3{ 2.0f, 0.0f, 0.0f });
    const auto padLandmarks = rock::root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
    auto opposedProbeSnapshot = liveFingerSnapshot;
    opposedProbeSnapshot.fingers[0].points[2] = RE::NiPoint3{ 0.0f, -2.0f, 0.0f };
    opposedProbeSnapshot.fingers[1].points[2] = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    opposedProbeSnapshot.fingers[2].points[2] = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
    const auto opposedPadLandmarks = rock::root_flattened_finger_skeleton_runtime::buildLandmarkSet(opposedProbeSnapshot);
    RE::NiPoint3 opposedProbeDirection{};
    ok &= expectBool("thumb pad probe can aim at distal index point",
        computeOpposedFingerPadProbeDirection(
            opposedProbeSnapshot,
            opposedPadLandmarks.fingers[0],
            0,
            opposedProbeSnapshot.fingers[0].points[2],
            opposedProbeDirection),
        true);
    ok &= expectPointClose("thumb pad probe direction points to index",
        opposedProbeDirection,
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
    ok &= expectBool("index pad probe can aim at distal thumb point",
        computeOpposedFingerPadProbeDirection(
            opposedProbeSnapshot,
            opposedPadLandmarks.fingers[1],
            1,
            opposedProbeSnapshot.fingers[1].points[2],
            opposedProbeDirection),
        true);
    ok &= expectPointClose("index pad probe direction points to thumb",
        opposedProbeDirection,
        RE::NiPoint3{ 0.0f, -1.0f, 0.0f });
    ok &= expectBool("middle pad probe also aims at distal thumb point",
        computeOpposedFingerPadProbeDirection(
            opposedProbeSnapshot,
            opposedPadLandmarks.fingers[2],
            2,
            opposedProbeSnapshot.fingers[2].points[2],
            opposedProbeDirection),
        true);
    ok &= expectPointClose("middle pad probe direction points to thumb",
        opposedProbeDirection,
        RE::NiPoint3{ -0.4472f, -0.8944f, 0.0f });

    SolvedGrabFingerPose directionPose{};
    directionPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 2.0f, 0.0f };
    directionPose.surfaceAimTargetValid[1] = 1;
    auto padTargets = makeSharedGripPoseTarget(RE::NiPoint3{ 2.0f, 0.0f, 2.0f });
    ok &= expectPointClose("pad direction prefers current surface target",
        fingerPadProbeDirection(directionPose, padTargets, padLandmarks.fingers[1], 1, padCenter),
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
    directionPose.surfaceAimTargetValid[1] = 0;
    ok &= expectPointClose("pad direction falls back to grip seat",
        fingerPadProbeDirection(directionPose, padTargets, padLandmarks.fingers[1], 1, padCenter),
        RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

    std::array<float, 5> splayRadians{};
    std::array<RE::NiPoint3, 5> splayOpenDirections{};
    const auto splayLandmarks = rock::root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
    for (std::size_t finger = 0; finger < splayOpenDirections.size(); ++finger) {
        splayOpenDirections[finger] = splayLandmarks.fingers[finger].openDirection;
    }
    SolvedGrabFingerPose splayPose{};
    ok &= expectBool("surface-contact splay rejects poses without targets",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayOpenDirections, splayRadians),
        false);
    splayPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, 0.2f, 0.0f };
    splayPose.surfaceAimTargetValid[1] = 1;
    ok &= expectBool("surface-contact splay builds from current target",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayOpenDirections, splayRadians),
        true);
    ok &= expectFloat("surface-contact splay stores positive index offset",
        splayRadians[1],
        kMaxSurfaceContactSplayRadians);
    splayPose.surfaceAimTarget[1] = RE::NiPoint3{ 2.0f, -0.2f, 0.0f };
    ok &= expectBool("surface-contact splay rebuilds when target moves",
        buildSurfaceContactSplayValues(splayPose, liveFingerSnapshot, splayOpenDirections, splayRadians),
        true);
    ok &= expectFloat("surface-contact splay stores negative index offset",
        splayRadians[1],
        -kMaxSurfaceContactSplayRadians);

    auto displayedSplayedSnapshot = liveFingerSnapshot;
    displayedSplayedSnapshot.fingers[1].points[1].y -= 0.8f;
    displayedSplayedSnapshot.fingers[1].points[2].y -= 1.6f;
    ok &= expectBool("surface splay ignores previously displayed splay",
        buildSurfaceContactSplayValues(splayPose, displayedSplayedSnapshot, splayOpenDirections, splayRadians), true);
    ok &= expectFloat("absolute splay stays fixed across displayed poses", splayRadians[1], -kMaxSurfaceContactSplayRadians);

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

    SolvedGrabFingerPose completeEvidence{};
    completeEvidence.solved = true;
    completeEvidence.contactValidMask = kCompleteFingerContactMask;
    ok &= expectBool("all five contact lanes complete the mesh hand transaction",
        hasCompleteFingerContactEvidence(completeEvidence), true);
    completeEvidence.contactValidMask &=
        static_cast<std::uint8_t>(~(1u << 3));
    ok &= expectBool("one missing contact lane invalidates the mesh hand transaction",
        hasCompleteFingerContactEvidence(completeEvidence), false);

    rock::root_flattened_finger_skeleton_runtime::Snapshot padProbeSnapshot = liveFingerSnapshot;
    for (auto& chain : padProbeSnapshot.fingers) {
        chain.points[0] = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        chain.points[1] = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        chain.points[2] = RE::NiPoint3{ 3.0f, 0.0f, 0.0f };
        chain.valid = true;
    }

    std::vector<TriangleData> closePadTriangles{
        TriangleData{
            RE::NiPoint3{ 2.0f, -1.0f, 0.5f },
            RE::NiPoint3{ 4.0f, -1.0f, 0.5f },
            RE::NiPoint3{ 3.0f, 1.0f, 0.5f },
        },
    };
    RE::NiTransform padObjectWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    auto closePadTargets = makeSharedGripPoseTarget(RE::NiPoint3{ 3.0f, 0.0f, 0.5f });
    closePadTargets.useSeatPointForMissingTargets = false;
    closePadTargets.useWholeMeshForMissingTargets = true;

    /*
     * The pad refinement must never mutate finger VALUES: the proximity-scaled
     * open bias (and thumb over-open) formed a publish->pad-moves->bias-changes
     * feedback loop that bypassed the held-re-solve deadband (the in-game
     * finger twitch). Values are the swept-arc solver's alone; pads only
     * refine surface-aim targets and report evidence.
     */
    SolvedGrabFingerPose padPose{};
    padPose.solved = true;
    padPose.hasJointValues = true;
    padPose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    padPose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(padPose.values);
    const auto padPoseValuesBefore = padPose.values;
    const auto padPoseJointsBefore = padPose.jointValues;
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
    ok &= expectBool("pad refinement never mutates finger values",
        padPose.values == padPoseValuesBefore,
        true);
    ok &= expectBool("pad refinement never mutates joint values",
        padPose.jointValues == padPoseJointsBefore,
        true);
    ok &= expectBool("pad evidence can create surface target",
        padPose.surfaceAimTargetValid[1] != 0,
        true);
    ok &= expectPointClose("pad-created surface target uses hit point",
        padPose.surfaceAimTarget[1],
        RE::NiPoint3{ 3.0f, 0.0f, 0.5f });

    SolvedGrabFingerPose evidenceOnlyPose{};
    evidenceOnlyPose.solved = true;
    evidenceOnlyPose.hasJointValues = true;
    evidenceOnlyPose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    evidenceOnlyPose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(evidenceOnlyPose.values);
    std::array<FingerPadSurfaceEvidence, 5> evidenceOnlyEvidence{};
    (void)refineGrabFingerPoseWithPadProbes(
        evidenceOnlyPose,
        closePadTriangles,
        closePadTargets,
        padProbeSnapshot,
        padObjectWorld,
        true,
        true,
        evidenceOnlyEvidence,
        false);
    ok &= expectBool("evidence-only pad refinement still reports evidence",
        evidenceOnlyEvidence[1].hit,
        true);
    ok &= expectBool("evidence-only pad refinement leaves surface targets alone",
        evidenceOnlyPose.surfaceAimTargetValid[1] == 0,
        true);

    {
        /*
         * Commanded anchor reconstruction: the held-re-solve arc zero comes
         * from the authored open pose's chain bone ORIGINS in hand space
         * (normalize(bone3 origin - bone1 origin)), the bake's own zero
         * definition. It must never consult rendered geometry.
         */
        std::array<RE::NiTransform, 15> openLocals{};
        for (auto& local : openLocals) {
            local = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        }
        // Index finger chain: base at (1,0,0), each child 2gu then 1gu along +Y.
        openLocals[3].translate = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        openLocals[4].translate = RE::NiPoint3{ 0.0f, 2.0f, 0.0f };
        openLocals[5].translate = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
        // Middle finger chain: straight up.
        openLocals[6].translate = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        openLocals[7].translate = RE::NiPoint3{ 0.0f, 0.0f, 2.0f };
        openLocals[8].translate = RE::NiPoint3{ 0.0f, 0.0f, 2.0f };
        const auto commandedDirections = computeCommandedOpenDirectionsHandLocal(openLocals.data());
        ok &= expectPointClose("commanded open direction follows the chain bone origins",
            commandedDirections[1],
            RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
        ok &= expectPointClose("commanded open direction normalizes the chain span",
            commandedDirections[2],
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        ok &= expectBool("degenerate chain yields a zero (invalid) direction",
            distanceSquared(commandedDirections[4], RE::NiPoint3{}) < 0.000001f,
            true);
        ok &= expectBool("null locals yield all-invalid directions",
            distanceSquared(
                computeCommandedOpenDirectionsHandLocal(nullptr)[0], RE::NiPoint3{}) < 0.000001f,
            true);
    }

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
    replacePadPose.surfaceAimTarget[1] = RE::NiPoint3{ 3.0f, 0.0f, 5.0f };
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
        RE::NiPoint3{ 3.0f, 0.0f, 0.5f });

    SolvedGrabFingerPose heldUpdatePose{};
    heldUpdatePose.solved = true;
    heldUpdatePose.hasJointValues = true;
    heldUpdatePose.values = { 1.0f, 0.2f, 1.0f, 1.0f, 1.0f };
    heldUpdatePose.jointValues = rock::grab_finger_pose_math::expandFingerCurlsToJointValues(heldUpdatePose.values);
    heldUpdatePose.surfaceAimTarget[1] = RE::NiPoint3{ 3.0f, 0.0f, 5.0f };
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
        false);
    ok &= expectPointClose("held pad update preserves captured surface target",
        heldUpdatePose.surfaceAimTarget[1],
        RE::NiPoint3{ 3.0f, 0.0f, 5.0f });
    ok &= expectFloat("held pad update leaves finger values alone",
        heldUpdatePose.values[1],
        heldUpdateOpenValue);

    std::vector<TriangleData> farPadTriangles{
        TriangleData{
            RE::NiPoint3{ 2.0f, -1.0f, 2.0f },
            RE::NiPoint3{ 4.0f, -1.0f, 2.0f },
            RE::NiPoint3{ 3.0f, 1.0f, 2.0f },
        },
    };
    auto explicitPadTargets = closePadTargets;
    explicitPadTargets.targets[1] = RE::NiPoint3{ 3.0f, 0.0f, 5.0f };
    explicitPadTargets.targetValid[1] = 1;
    explicitPadTargets.targetCount = 1;
    SolvedGrabFingerPose explicitPadPose{};
    explicitPadPose.solved = true;
    explicitPadPose.values = { 1.0f, 0.5f, 1.0f, 1.0f, 1.0f };
    explicitPadPose.surfaceAimTarget[1] = RE::NiPoint3{ 3.0f, 0.0f, 5.0f };
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
        RE::NiPoint3{ 3.0f, 0.0f, 5.0f });

    std::vector<TriangleData> highPolyFallbackTriangles;
    highPolyFallbackTriangles.reserve(kMaxFingerPoseCandidateTriangles + 16);
    highPolyFallbackTriangles.push_back(TriangleData{
        RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
        RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f },
    });
    for (std::size_t i = 0; i < kMaxFingerPoseCandidateTriangles + 15; ++i) {
        const float offset = 1000.0f + static_cast<float>(i * 3);
        highPolyFallbackTriangles.push_back(TriangleData{
            RE::NiPoint3{ offset, 0.0f, 0.0f },
            RE::NiPoint3{ offset + 1.0f, 0.0f, 0.0f },
            RE::NiPoint3{ offset, 1.0f, 0.0f },
        });
    }
    auto highPolyTargets = makeSharedGripPoseTarget(RE::NiPoint3{ 0.0f, 0.0f, 0.0f });
    highPolyTargets.useSeatPointForMissingTargets = false;
    highPolyTargets.useWholeMeshForMissingTargets = true;
    std::vector<rock::grab_finger_pose_math::Triangle<RE::NiPoint3>> highPolyCandidates;
    appendCandidateTriangles(highPolyFallbackTriangles, highPolyTargets, 100.0f, highPolyCandidates);
    ok &= expectBool("whole-mesh finger fallback is bounded", highPolyCandidates.size() == kMaxFingerPoseCandidateTriangles, true);
    ok &= expectPointClose("whole-mesh finger fallback keeps nearest seat triangle first", highPolyCandidates.empty() ? RE::NiPoint3{} : highPolyCandidates.front().v0,
        RE::NiPoint3{ 0.0f, 0.0f, 0.0f });

    std::vector<TriangleData> spatialLocalTriangles;
    spatialLocalTriangles.reserve(kMaxFingerPoseCandidateTriangles + 32);
    for (std::size_t i = 0; i < kMaxFingerPoseCandidateTriangles + 32; ++i) {
        const float x = static_cast<float>(i) * 4.0f;
        spatialLocalTriangles.push_back(TriangleData{
            RE::NiPoint3{ x, 0.0f, 0.0f },
            RE::NiPoint3{ x + 1.0f, 0.0f, 0.0f },
            RE::NiPoint3{ x, 1.0f, 0.0f },
        });
    }
    FingerPoseTriangleSpatialIndex spatialIndex{};
    ok &= expectBool("finger spatial index builds from bounded local triangles", spatialIndex.buildFromLocalTriangles(spatialLocalTriangles), true);
    ok &= expectBool("finger spatial index retains the 2048 triangle cap", spatialIndex.triangleCount() == kMaxFingerPoseCandidateTriangles, true);

    constexpr std::size_t kSpatialHitTriangle = 1377;
    const float spatialHitX = static_cast<float>(kSpatialHitTriangle) * 4.0f;
    RE::NiTransform spatialObjectWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    spatialObjectWorld.scale = 2.0f;
    spatialObjectWorld.translate = RE::NiPoint3{ 5.0f, -3.0f, 4.0f };
    const RE::NiPoint3 spatialExpectedLocal{ spatialHitX + 0.25f, 0.25f, 0.0f };
    const RE::NiPoint3 spatialExpectedWorld = rock::transform_math::localPointToWorld(spatialObjectWorld, spatialExpectedLocal);
    const RE::NiPoint3 spatialProbeWorld = rock::transform_math::localPointToWorld(spatialObjectWorld, RE::NiPoint3{ spatialHitX + 0.25f, 0.25f, 0.2f });
    RE::NiPoint3 spatialHitPoint{};
    RE::NiPoint3 spatialHitNormal{};
    FingerPoseSpatialQueryStats spatialStats{};
    ok &= expectBool("finger spatial index finds exact transformed sphere contact",
        spatialIndex.querySphereWorld(spatialObjectWorld, spatialProbeWorld, 0.6f, &spatialHitPoint, &spatialHitNormal, &spatialStats), true);
    ok &= expectPointClose("finger spatial index returns exact transformed closest point", spatialHitPoint, spatialExpectedWorld);
    ok &= expectBool("finger spatial index orients the hit normal toward the probe", spatialHitNormal.z > 0.99f, true);
    ok &= expectBool("finger spatial index culls dense unrelated geometry", spatialStats.triangleTests <= 32 && spatialStats.nodeVisits < 128, true);

    using namespace rock::grab_finger_pose_math;
    namespace grab_data = rock::grab_finger_calibration_data;
    constexpr float kHalfPi = 1.57079632679489661923f;

    const auto bakedIndexCurve =
        makeBakedCalibratedFingerCurve<TestVector>(1, false, false, TestVector{ 0.0f, 0.0f, 0.0f }, TestVector{ 0.0f, 0.0f, 1.0f }, TestVector{ 1.0f, 0.0f, 0.0f }, 10.0f);
    ok &= expectBool("baked calibration creates three probe curves",
        bakedIndexCurve.probeCount == 3 &&
            bakedIndexCurve.probes[0].sampleCount == kCalibratedFingerCurveSampleCount &&
            bakedIndexCurve.probes[1].sampleCount == kCalibratedFingerCurveSampleCount &&
            bakedIndexCurve.probes[2].sampleCount == kCalibratedFingerCurveSampleCount,
        true);
    {
        // samples[0] is now an over-open row (hyper-extension folds the tip
        // back, shortening reach); the authored-open scaling contract lives
        // at the first sub-open row.
        std::size_t authoredOpenRow = 0;
        while (authoredOpenRow < bakedIndexCurve.probes[0].sampleCount &&
               bakedIndexCurve.probes[0].samples[authoredOpenRow].openValue > 1.0f + 0.0001f) {
            ++authoredOpenRow;
        }
        ok &= expectBool("baked tip curve carries over-open rows ahead of the authored open row",
            authoredOpenRow > 0 && authoredOpenRow < bakedIndexCurve.probes[0].sampleCount,
            true);
        ok &= expectBool("baked tip reach is scaled from runtime landmark length to authored fingertip reach",
            bakedIndexCurve.probes[0].samples[authoredOpenRow].reachLength > 12.0f,
            true);
        ok &= expectBool("baked over-open rows carry negative arc angles",
            bakedIndexCurve.probes[0].samples[0].angleRadians < 0.0f,
            true);
    }
    ok &= expectBool("baked standard index has usable max curl angle",
        bakedCalibratedFingerMaxAngleRadians(1, false, false) > 1.0f,
        true);
    ok &= expectBool("baked power armor profile has distinct index curve",
        bakedCalibratedFingerMaxAngleRadians(1, false, true) > bakedCalibratedFingerMaxAngleRadians(1, false, false),
        true);
    const auto& bakedThumbProfile = grab_data::bakedGrabThumbProfile(false, false);
    ok &= expectBool("baked thumb profile creates wrap/opposition/side-pad lanes",
        bakedThumbProfile.lanes[0].lane == grab_data::BakedGrabThumbLane::Wrap &&
            bakedThumbProfile.lanes[1].lane == grab_data::BakedGrabThumbLane::Opposition &&
            bakedThumbProfile.lanes[2].lane == grab_data::BakedGrabThumbLane::SidePad,
        true);
    ok &= expectFloat("baked side-pad lane uses intermediate thumb normal",
        bakedThumbProfile.lanes[2].normalBlend,
        0.48f);
    ok &= expectFloat("baked side-pad lane has bounded local correction strength",
        bakedThumbProfile.lanes[2].localCorrectionStrength,
        0.82f);
    ok &= expectBool("baked side-pad lane owns the custom thumb curve",
        bakedThumbProfile.lanes[2].curveSource == grab_data::BakedGrabThumbCurveSource::SidePad,
        true);
    const auto bakedThumbPrimaryCurve = makeBakedCalibratedFingerCurve<TestVector>(
        0,
        false,
        false,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        10.0f);
    const auto bakedThumbAlternateCurve = makeBakedCalibratedFingerCurve<TestVector>(
        0,
        false,
        false,
        TestVector{ 0.0f, 0.0f, 0.0f },
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 1.0f, 0.0f, 0.0f },
        10.0f,
        false);
    ok &= expectFloat("baked thumb primary applies authored normal sign",
        bakedThumbPrimaryCurve.normal.z,
        -1.0f);
    ok &= expectFloat("baked thumb alternate preserves explicit plane normal",
        bakedThumbAlternateCurve.normal.z,
        1.0f);
    const TestVector sidePadNormal = blendedThumbLaneNormal(
        TestVector{ 0.0f, 0.0f, 1.0f },
        TestVector{ 0.0f, 1.0f, 0.0f },
        bakedThumbProfile.lanes[2].normalBlend);
    const auto bakedThumbSidePadCurve = makeBakedCalibratedThumbLaneCurve<TestVector>(
        bakedThumbProfile.lanes[2],
        bakedThumbProfile.sidePadCurve,
        TestVector{ 0.0f, 0.0f, 0.0f },
        sidePadNormal,
        TestVector{ 1.0f, 0.0f, 0.0f },
        10.0f);
    ok &= expectBool("baked side-pad curve preserves blended explicit plane",
        bakedThumbSidePadCurve.normal.y > 0.60f && bakedThumbSidePadCurve.normal.z > 0.70f,
        true);

    // ---- swept-arc solver (replaces the retired plane-slice solver tests) ----

    const auto makeDenseProbeCurve = [](CalibratedFingerProbe probe, float reach) {
        CalibratedFingerProbeCurve<TestVector> curve{};
        curve.probe = probe;
        curve.sampleCount = kCalibratedFingerCurveSampleCount;
        for (std::size_t i = 0; i < kCalibratedFingerCurveSampleCount; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(kCalibratedFingerCurveSampleCount - 1);
            curve.samples[i] = {
                .openValue = 1.0f - t,
                .angleRadians = t * kHalfPi,
                .reachLength = reach,
            };
        }
        return curve;
    };
    const auto makeDenseCurve = [&](float tipReach, float outerReach, float innerReach, std::size_t probeCount) {
        CalibratedFingerCurve<TestVector> curve{};
        curve.center = TestVector{ 0.0f, 0.0f, 0.0f };
        curve.normal = TestVector{ 0.0f, 0.0f, 1.0f };
        curve.zeroAngleVector = TestVector{ 1.0f, 0.0f, 0.0f };
        curve.surfaceThickness = 0.0f;
        curve.probeCount = probeCount;
        curve.probes[0] = makeDenseProbeCurve(CalibratedFingerProbe::Tip, tipReach);
        curve.probes[1] = makeDenseProbeCurve(CalibratedFingerProbe::Outer, outerReach);
        curve.probes[2] = makeDenseProbeCurve(CalibratedFingerProbe::Inner, innerReach);
        return curve;
    };
    const auto arcPoint = [](float reach, float angleRadians) {
        // Same rotation convention as the sweep: zeroAngle (+X) rotated around
        // the curl normal (+Z) by a positive angle lands in +Y.
        return TestVector{ reach * std::cos(angleRadians), reach * std::sin(angleRadians), 0.0f };
    };

    // First contact along the closing arc stops the finger near the surface's
    // arc angle (a pad-radius of early contact is physical, never late).
    const auto midArcSolved = sweepCalibratedFingerCurveCurlValue(
        makeSliverTriangleAtPoint(arcPoint(2.0f, kHalfPi * 0.5f)),
        makeDenseCurve(2.0f, 2.0f, 2.0f, 1),
        0.2f,
        0.15f);
    ok &= expectBool("sweep hits a surface on the closing arc", midArcSolved.hit, true);
    ok &= expectBool("sweep stops at first contact, never past the surface",
        midArcSolved.value >= 0.49f && midArcSolved.value <= 0.58f,
        true);
    ok &= expectBool("sweep exposes the selected contact sphere", midArcSolved.hasContactCenter, true);
    ok &= expectFloat("sweep exposes the exact contact radius", midArcSolved.contactRadius, 0.15f);
    ok &= expectBool("single-probe sweep identifies the tip probe", midArcSolved.selectedProbeIndex == 0, true);
    ok &= expectBool("sweep records the actual probe start row", midArcSolved.sweptProbeStartRowValid[0] != 0, true);

    // The most-open first contact across probes wins: the outer probe touches
    // its surface at 30 degrees before the tip reaches its own at 60.
    {
        auto twoContactTriangles = makeSliverTriangleAtPoint(arcPoint(1.2f, kHalfPi / 3.0f));
        const auto tipTriangle = makeSliverTriangleAtPoint(arcPoint(2.0f, kHalfPi * 2.0f / 3.0f));
        twoContactTriangles.insert(twoContactTriangles.end(), tipTriangle.begin(), tipTriangle.end());
        const auto mostOpenSolved = sweepCalibratedFingerCurveCurlValue(
            twoContactTriangles,
            makeDenseCurve(2.0f, 1.2f, 0.6f, 3),
            0.2f,
            0.15f);
        ok &= expectBool("sweep most-open probe contact wins",
            mostOpenSolved.hit && mostOpenSolved.value >= 0.64f && mostOpenSolved.value <= 0.78f,
            true);
        ok &= expectBool("sweep identifies the winning outer probe", mostOpenSolved.selectedProbeIndex == 1, true);
    }

    // A surface already touching the fully open pose keeps the finger open.
    const auto openContactSolved = sweepCalibratedFingerCurveCurlValue(
        makeSliverTriangleAtPoint(arcPoint(2.0f, 0.0f)),
        makeDenseCurve(2.0f, 2.0f, 2.0f, 1),
        0.2f,
        0.15f);
    ok &= expectBool("sweep contact at fully open stays open",
        openContactSolved.hit && openContactSolved.value >= 0.97f,
        true);

    // Nothing within the whole arc: out of reach (callers hold anticipation).
    const auto outOfReachSolved = sweepCalibratedFingerCurveCurlValue(
        makeSliverTriangleAtPoint(TestVector{ 10.0f, 10.0f, 0.0f }),
        makeDenseCurve(2.0f, 2.0f, 2.0f, 3),
        0.2f,
        0.15f);
    ok &= expectBool("sweep out-of-reach reports no hit", outOfReachSolved.hit, false);
    ok &= expectBool("sweep out-of-reach flags anticipation", outOfReachSolved.outOfReach, true);
    ok &= expectFloat("sweep out-of-reach keeps min value", outOfReachSolved.value, 0.2f);
    ok &= expectBool("debug state distinguishes out-of-reach from fallback", classifyFingerSweepDebugState(outOfReachSolved, 0.2f) == FingerSweepDebugState::OutOfReach, true);

    // Debug capture decimates the already-solved path into object-local fixed
    // storage. It must preserve the selected sphere without issuing geometry
    // queries of its own.
    {
        CalibratedFingerCurve<RE::NiPoint3> debugCurve{};
        debugCurve.center = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
        debugCurve.normal = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        debugCurve.zeroAngleVector = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        debugCurve.probeCount = 1;
        debugCurve.probes[0].probe = CalibratedFingerProbe::Tip;
        debugCurve.probes[0].sampleCount = 3;
        debugCurve.probes[0].samples[0] = { .openValue = 2.0f, .angleRadians = -kHalfPi, .reachLength = 2.0f };
        debugCurve.probes[0].samples[1] = { .openValue = 1.0f, .angleRadians = 0.0f, .reachLength = 2.0f };
        debugCurve.probes[0].samples[2] = { .openValue = 0.0f, .angleRadians = kHalfPi, .reachLength = 2.0f };

        auto debugSolved = rock::grab_finger_pose_math::FingerCurlValue{};
        debugSolved.hit = true;
        debugSolved.value = 1.0f;
        debugSolved.rawCurveValue = 1.0f;
        debugSolved.hasContactCenter = true;
        debugSolved.contactCenterX = 2.0f;
        debugSolved.contactCenterY = 0.0f;
        debugSolved.contactRadius = 0.25f;
        debugSolved.selectedProbeIndex = 0;
        debugSolved.sweptProbeStartRowValid[0] = 1;
        debugSolved.sweptProbeStartRow[0] = 0;

        const auto identity = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        FingerSweepDebugFingerCapture capture{};
        ok &= expectBool("sweep debug capture accepts a solved fixed path",
            captureFingerSweepDebugCurve(capture, debugCurve, debugSolved, 0.2f, grab_data::BakedGrabThumbLane::Wrap, identity), true);
        ok &= expectBool("sweep debug capture keeps all short-path rows", capture.probePointCount[0] == 3, true);
        ok &= expectBool("sweep debug capture exposes the proximal pivot", capture.hasPivot, true);
        ok &= expectPointClose("sweep debug capture stores proximal pivot object-local", capture.pivotObjectLocal, RE::NiPoint3{});
        ok &= expectFloat("sweep debug capture starts at full flex ceiling", capture.probeStartOpenValue[0], 2.0f);
        ok &= expectFloat("sweep debug capture ends fully closed", capture.probeEndOpenValue[0], 0.0f);
        ok &= expectBool("sweep debug capture marks authored open independently", capture.authoredOpenPointValid[0] != 0, true);
        ok &= expectPointClose("sweep debug capture stores authored-open point", capture.authoredOpenPointObjectLocal[0], RE::NiPoint3{ 2.0f, 0.0f, 0.0f });
        ok &= expectPointClose("sweep debug capture stores contact center object-local", capture.contactCenterObjectLocal, RE::NiPoint3{ 2.0f, 0.0f, 0.0f });
        ok &= expectFloat("sweep debug capture stores contact radius object-local", capture.contactRadiusObjectLocal, 0.25f);
        ok &= expectBool("sweep debug capture classifies a normal hit", capture.state == FingerSweepDebugState::Hit, true);
    }

    // ---- over-open sweep rows (values past the authored open pose) ----
    {
        /*
         * Synthetic curve spanning openValue [0, 2]: value 1.0 sits at angle
         * 0 (the zero reference), over-open rows carry negative angles, the
         * closed end reaches +kHalfPi - the same shape the generator bakes.
         */
        const auto makeOverOpenProbeCurve = [](CalibratedFingerProbe probe, float reach) {
            CalibratedFingerProbeCurve<TestVector> curve{};
            curve.probe = probe;
            curve.sampleCount = kCalibratedFingerCurveSampleCount;
            for (std::size_t i = 0; i < kCalibratedFingerCurveSampleCount; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(kCalibratedFingerCurveSampleCount - 1);
                curve.samples[i] = {
                    .openValue = 2.0f * (1.0f - t),
                    .angleRadians = (t - 0.5f) * 2.0f * kHalfPi,
                    .reachLength = reach,
                };
            }
            return curve;
        };
        CalibratedFingerCurve<TestVector> overOpenCurve{};
        overOpenCurve.center = TestVector{ 0.0f, 0.0f, 0.0f };
        overOpenCurve.normal = TestVector{ 0.0f, 0.0f, 1.0f };
        overOpenCurve.zeroAngleVector = TestVector{ 1.0f, 0.0f, 0.0f };
        overOpenCurve.surfaceThickness = 0.0f;
        overOpenCurve.probeCount = 1;
        overOpenCurve.probes[0] = makeOverOpenProbeCurve(CalibratedFingerProbe::Tip, 2.0f);

        // A surface only reachable over-open (negative arc angle).
        const auto overOpenOnlyTriangles = makeSliverTriangleAtPoint(arcPoint(2.0f, -kHalfPi * 0.5f));

        // The default walk starts at 2.0, but an isolated dorsal surface does
        // not prove that the authored-open pose is obstructed. It therefore
        // remains ineligible as a published over-open stop.
        const auto cappedSolved = sweepCalibratedFingerCurveCurlValue(
            overOpenOnlyTriangles, overOpenCurve, 0.2f, 0.15f);
        ok &= expectBool("default full sweep ignores unsupported dorsal contact", cappedSolved.hit, false);
        ok &= expectBool("default full sweep records row zero as its actual start",
            cappedSolved.sweptProbeStartRowValid[0] != 0 && cappedSolved.sweptProbeStartRow[0] == 0,
            true);
        ok &= expectBool("isolated surface does not classify authored open as enclosed",
            pointInsideClosedTriangleMesh(overOpenOnlyTriangles, arcPoint(2.0f, 0.0f)),
            false);

        /*
         * Over-open engages ONLY when the authored-open row is blocked. A
         * surface that merely grazes the dorsal (over-open) side of the arc
         * while the finger can rest freely at 1.0 must be ignored - the
         * skull case: fingers must wrap down the closing side, never rest
         * hyper-extended on top.
         */
        const auto dorsalGrazeSolved = sweepCalibratedFingerCurveCurlValue(
            overOpenOnlyTriangles, overOpenCurve, 0.2f, 0.15f, 2.0f);
        ok &= expectBool("free authored-open row ignores dorsal-only over-open contact",
            dorsalGrazeSolved.hit, false);

        // With the authored-open row BLOCKED (mesh interpenetrating the open
        // finger), the walk starts hyper-open and rests at the first
        // over-open contact.
        auto blockedOpenTriangles = overOpenOnlyTriangles;
        const auto openRowBlocker = makeSliverTriangleAtPoint(arcPoint(2.0f, 0.0f));
        blockedOpenTriangles.insert(blockedOpenTriangles.end(), openRowBlocker.begin(), openRowBlocker.end());
        const auto overOpenSolved = sweepCalibratedFingerCurveCurlValue(
            blockedOpenTriangles, overOpenCurve, 0.2f, 0.15f, 2.0f);
        ok &= expectBool("blocked authored-open row engages the over-open walk",
            overOpenSolved.hit && overOpenSolved.value > 1.0f,
            true);
        ok &= expectBool("over-open sweep lands near the contact row",
            overOpenSolved.value >= 1.42f && overOpenSolved.value <= 1.58f,
            true);
        ok &= expectBool("over-open contact row reports its negative arc angle",
            overOpenSolved.distance < 0.0f,
            true);

        // A thick closed object can place the authored-open probe well inside
        // the mesh, farther than the contact-sphere radius from every face.
        // The old single sphere-at-1.0 gate called that pose free and skipped
        // the entire 2 -> 1 clearance interval. Closed-shell occupancy must
        // recognize it and select the first entry contact from the full walk.
        const auto enclosingBox = makeClosedBox(
            TestVector{ 1.7f, -0.6f, -0.3f },
            TestVector{ 2.3f, 0.6f, 0.3f });
        ok &= expectBool("closed shell recognizes a deep-inside authored-open point",
            pointInsideClosedTriangleMesh(enclosingBox, arcPoint(2.0f, 0.0f)),
            true);
        const auto deepInsideSolved = sweepCalibratedFingerCurveCurlValue(
            enclosingBox, overOpenCurve, 0.2f, 0.15f);
        ok &= expectBool("deep-inside authored open engages full over-open clearance",
            deepInsideSolved.hit && deepInsideSolved.value > 1.0f,
            true);
        ok &= expectBool("deep-inside clearance lands on the box entry surface",
            deepInsideSolved.value >= 1.10f && deepInsideSolved.value <= 1.40f,
            true);

        // A cap below the contact row hides it even when the open row is
        // blocked (row skipping honors the cap, not just the result clamp);
        // the walk then starts at the cap and first contacts the blocker at
        // the authored-open row.
        const auto partialCapSolved = sweepCalibratedFingerCurveCurlValue(
            blockedOpenTriangles, overOpenCurve, 0.2f, 0.15f, 1.25f);
        ok &= expectBool("partial cap skips rows above it",
            partialCapSolved.hit && partialCapSolved.value >= 1.0f && partialCapSolved.value <= 1.1f,
            true);

        // The classic sub-open region is unchanged under a raised cap.
        const auto subOpenSolved = sweepCalibratedFingerCurveCurlValue(
            makeSliverTriangleAtPoint(arcPoint(2.0f, kHalfPi * 0.5f)),
            overOpenCurve, 0.2f, 0.15f, 2.0f);
        ok &= expectBool("raised cap leaves sub-open contacts where they were",
            subOpenSolved.hit && subOpenSolved.value >= 0.44f && subOpenSolved.value <= 0.56f,
            true);
    }

    // Thumb lanes: a contact reachable only in the opposition plane must
    // select the opposition lane with its baked correction strength.
    {
        const auto& thumbProfile = grab_data::bakedGrabThumbProfile(false, false);
        const auto& thumbHandProfile = grab_data::bakedGrabFingerHandProfile(false, false);
        const TestVector primaryNormal{ 0.0f, 0.0f, 1.0f };
        const TestVector alternateNormal{ 0.0f, 1.0f, 0.0f };
        const TestVector zeroAngle{ 1.0f, 0.0f, 0.0f };
        constexpr float kThumbLength = 2.0f;

        const auto reconstructLaneTipRow = [&](const grab_data::BakedGrabThumbLaneCurve& lane,
                                               const grab_data::BakedGrabFingerCurve& bakedCurve,
                                               std::size_t sampleIndex) {
            const TestVector laneNormal = blendedThumbLaneNormal(primaryNormal, alternateNormal, lane.normalBlend);
            const auto laneCurve = makeBakedCalibratedThumbLaneCurve<TestVector>(
                lane, bakedCurve, TestVector{}, laneNormal, zeroAngle, kThumbLength);
            const auto& sample = laneCurve.probes[0].samples[sampleIndex];
            const TestVector arm = rotateAroundUnitAxis(
                normalize(laneCurve.zeroAngleVector), normalize(laneCurve.normal), sample.angleRadians);
            return TestVector{ arm.x * sample.reachLength, arm.y * sample.reachLength, arm.z * sample.reachLength };
        };

        const auto oppositionSolved = sweepThumbAwareCalibratedFingerCurveCurlValue(
            makeSliverTriangleAtPoint(reconstructLaneTipRow(thumbProfile.lanes[1], thumbHandProfile.fingers[0], 120)),
            0,
            false,
            false,
            TestVector{ 0.0f, 0.0f, 0.0f },
            primaryNormal,
            alternateNormal,
            zeroAngle,
            kThumbLength,
            0.2f,
            true,
            0.2f);
        ok &= expectBool("sweep thumb retries when primary lane misses",
            oppositionSolved.usedAlternateThumbCurve,
            true);
        ok &= expectBool("sweep thumb records opposition lane",
            oppositionSolved.selectedThumbLane == grab_data::BakedGrabThumbLane::Opposition,
            true);
        ok &= expectFloat("sweep thumb carries opposition correction strength",
            oppositionSolved.selectedThumbLaneLocalCorrectionStrength,
            1.0f);
        ok &= expectBool("sweep opposition thumb returns a valid hit",
            oppositionSolved.value.hit,
            true);

        const auto sidePadSolved = sweepThumbAwareCalibratedFingerCurveCurlValue(
            makeSliverTriangleAtPoint(reconstructLaneTipRow(thumbProfile.lanes[2], thumbProfile.sidePadCurve, 120)),
            0,
            false,
            false,
            TestVector{ 0.0f, 0.0f, 0.0f },
            primaryNormal,
            alternateNormal,
            zeroAngle,
            kThumbLength,
            0.2f,
            true,
            0.2f);
        ok &= expectBool("sweep thumb can select baked side-pad lane",
            sidePadSolved.selectedThumbLane == grab_data::BakedGrabThumbLane::SidePad,
            true);
        ok &= expectBool("sweep side-pad thumb returns selected hit",
            sidePadSolved.selectedThumbCurve.hit && sidePadSolved.value.hit,
            true);
        ok &= expectFloat("sweep side-pad thumb carries lane correction strength",
            sidePadSolved.selectedThumbLaneLocalCorrectionStrength,
            0.82f);
    }

    {
        /*
         * Chord curl estimation must be self-consistent with the baked Tip
         * reach table: feeding back a baked sample's own reach must recover
         * that sample's arc angle. This is the contract the runtime
         * de-rotation relies on (the live chord IS the Tip probe point
         * relative to the base).
         */
        using rock::grab_finger_pose_math::estimateCalibratedChainCurlFromChord;
        const auto& indexBaked = grab_data::bakedGrabFingerHandProfile(false, false).fingers[1];
        const grab_data::BakedGrabFingerProbeCurve* indexTip = nullptr;
        for (const auto& probe : indexBaked.probes) {
            if (probe.probe == grab_data::BakedGrabFingerProbe::Tip) {
                indexTip = &probe;
                break;
            }
        }
        ok &= expectBool("baked index finger exposes a tip probe", indexTip != nullptr, true);
        if (indexTip) {
            constexpr float kFingerLength = 7.5f;
            /*
             * The inversion is restricted to the openValue <= 1 region: past
             * the authored open pose the chord shortens again, so reach is
             * ambiguous across the over-open rows. Self-consistency is only
             * a contract inside the restricted region.
             */
            std::size_t scanStart = 0;
            while (scanStart < indexTip->samples.size() && indexTip->samples[scanStart].openValue > 1.0f + 0.0001f) {
                ++scanStart;
            }
            ok &= expectBool("baked index finger has over-open rows before the scan region", scanStart > 0, true);
            ok &= expectBool("baked index finger keeps a usable sub-open scan region",
                scanStart + 60 < indexTip->samples.size(),
                true);
            const std::size_t lastSample = indexTip->samples.size() - 11;
            for (const std::size_t sampleIndex : { scanStart, scanStart + 30, scanStart + 60, lastSample }) {
                const auto& sample = indexTip->samples[sampleIndex];
                const auto estimate = estimateCalibratedChainCurlFromChord(
                    1, false, false, kFingerLength, sample.reachScale * kFingerLength);
                ok &= expectBool("chord curl estimate valid at baked sample", estimate.valid, true);
                ok &= expectBool("chord curl estimate recovers the baked arc angle",
                    std::fabs(estimate.chordAngleRadians - sample.angleRadians) <= 0.02f,
                    true);
            }
            const auto openEstimate = estimateCalibratedChainCurlFromChord(1, false, false, kFingerLength, kFingerLength * 2.0f);
            ok &= expectBool("over-long chord clamps to the sub-open scan start", openEstimate.valid, true);
            ok &= expectBool("over-long chord reports the scan-start angle",
                std::fabs(openEstimate.chordAngleRadians - indexTip->samples[scanStart].angleRadians) <= 0.0001f,
                true);
            ok &= expectBool("chord estimate never reports an over-open value",
                openEstimate.openValue <= 1.0f + 0.001f,
                true);
            const auto closedEstimate = estimateCalibratedChainCurlFromChord(1, false, false, kFingerLength, 0.01f);
            ok &= expectBool("under-reach chord clamps to the closed end", closedEstimate.valid, true);
            const auto invalidEstimate = estimateCalibratedChainCurlFromChord(1, false, false, 0.0f, 3.0f);
            ok &= expectBool("degenerate finger length rejects the estimate", invalidEstimate.valid, false);
        }
    }

    return ok ? 0 : 1;
}
