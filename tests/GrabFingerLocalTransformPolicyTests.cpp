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

/*
 * The capsule finger sweep asks the mesh index "is any triangle within r of
 * this point, and which is nearest". The BVH answer must match a brute-force
 * scan over the same triangles, including scaled and rotated object frames,
 * duplicate and reversed faces, and degenerate or non-finite input.
 */
bool testIndexedSphereQueries()
{
    using namespace rock;
    using namespace grab_finger_pose_runtime;
    std::mt19937 random(81239);
    std::uniform_real_distribution<float> coordinate(-100.0f, 100.0f);
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
    std::vector<TriangleData> local;
    for (int i = 0; i < 2048; ++i) {
        const RE::NiPoint3 p{ coordinate(random), coordinate(random), coordinate(random) };
        local.push_back({ p, p + RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, p + RE::NiPoint3{ 0.0f, 2.0f, 0.5f } });
    }
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
        for (int query = 0; query < 160; ++query) {
            const auto& seed = local[(query * 13) % local.size()];
            const RE::NiPoint3 pointLocal = query < 120 && grab_finger_pose_runtime::isFinitePoint(seed.v0) ?
                seed.v0 + RE::NiPoint3{ unit(random), unit(random), unit(random) } :
                RE::NiPoint3{ coordinate(random), coordinate(random), coordinate(random) };
            const RE::NiPoint3 pointWorld = transform_math::localPointToWorld(world, pointLocal);
            const float radiusWorld = (0.2f + 3.0f * std::abs(unit(random))) * std::abs(scale);

            // Brute force in object space over the triangles the index admits.
            float bestLocal = std::numeric_limits<float>::max();
            for (const auto& triangle : local) {
                if (!isFinitePoint(triangle.v0) || !isFinitePoint(triangle.v1) || !isFinitePoint(triangle.v2)) continue;
                const RE::NiPoint3 closest = grab_finger_pose_math::closestPointOnTriangle(pointLocal, grab_finger_pose_math::Triangle<RE::NiPoint3>{ triangle.v0, triangle.v1, triangle.v2 });
                bestLocal = (std::min)(bestLocal, grab_finger_pose_math::length(grab_finger_pose_math::sub(closest, pointLocal)));
                ++linearTests;
            }
            const bool expectedHit = bestLocal * std::abs(scale) <= radiusWorld;

            RE::NiPoint3 hitWorld{};
            FingerPoseSpatialQueryStats stats{};
            const bool hit = index.querySphereWorld(world, pointWorld, radiusWorld, &hitWorld, nullptr, &stats);
            indexedTests += stats.triangleTests;
            ++compared;
            if (hit != expectedHit) {
                std::printf("Indexed sphere hit mismatch scale=%f query=%d expected=%d\n", scale, query, expectedHit ? 1 : 0);
                return false;
            }
            if (hit) {
                const RE::NiPoint3 offset = hitWorld - pointWorld;
                const float distanceWorld = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
                // World coordinates near 25000 gu carry ~0.002 gu of float error.
                if (std::abs(distanceWorld - bestLocal * std::abs(scale)) > 0.01f) {
                    std::printf("Indexed sphere nearest mismatch scale=%f query=%d %f vs %f\n", scale, query, distanceWorld, bestLocal * std::abs(scale));
                    return false;
                }
            }
        }
    }
    index.clear();
    if (index.buildFromLocalTriangles(std::vector<TriangleData>{})) return false;
    std::printf("Indexed sphere parity: %d queries, %llu linear vs %llu indexed triangle tests\n", compared,
        static_cast<unsigned long long>(linearTests), static_cast<unsigned long long>(indexedTests));
    return indexedTests < linearTests / 3;
}

int main()
{
    bool ok = true;
    ok &= testPublicationSafety();
    ok &= testIndexedSphereQueries();
    return ok ? 0 : 1;
}
