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

namespace rock::root_flattened_finger_skeleton_runtime
{
    // A capture with a supplied snapshot must never query the game scene again.
    bool queriedLiveScene = false;
    bool resolveLiveFingerSkeletonSnapshot(bool, Snapshot&, std::string*, SkeletonBoneCaptureSpace)
    {
        queriedLiveScene = true;
        return false;
    }
}

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
    bool ok = true;
    ok &= testPublicationSafety();
    ok &= testIndexedFingerPadProbes();
    return ok ? 0 : 1;
}
