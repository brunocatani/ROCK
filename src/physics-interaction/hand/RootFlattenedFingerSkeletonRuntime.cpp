#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/hand/SkeletonBoneNameIndex.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

/*
 * Runtime lookup is intentionally kept outside the header so pure math tests
 * can validate finger-chain conventions without depending on live FO4VR scene
 * objects. Production code resolves the same root flattened bone tree used by
 * generated hand colliders and returns a compact world-space snapshot to the
 * pose solver and debug overlay.
 */

#include "physics-interaction/debug/DebugMath.h"

#include <string_view>

namespace rock::root_flattened_finger_skeleton_runtime
{
    namespace
    {
        struct FingerCaptureScratch
        {
            DirectSkeletonBoneReader reader;
            DirectSkeletonBoneSnapshot bones;
            SkeletonBoneNameIndex names;
        };

        const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }
    }

    bool buildFingerSkeletonSnapshot(
        const DirectSkeletonBoneSnapshot& boneSnapshot,
        bool isLeft,
        Snapshot& outSnapshot,
        std::string* outMissingBoneName,
        SkeletonBoneNameIndex* nameIndex)
    {
        outSnapshot = Snapshot{};
        if (outMissingBoneName) {
            outMissingBoneName->clear();
        }
        if (!boneSnapshot.valid) {
            if (outMissingBoneName) {
                *outMissingBoneName = "rootFlattenedBoneTree";
            }
            return false;
        }

        const auto indexed = nameIndex ? nameIndex->bind(boneSnapshot) : SkeletonBoneNameIndex::View{ boneSnapshot, {} };
        const auto findBone = [&](std::string_view name) {
            return nameIndex ? indexed.find(name) : findSnapshotBone(boneSnapshot, name);
        };
        const auto* handNode = findBone(isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!handNode) {
            if (outMissingBoneName) {
                *outMissingBoneName = isLeft ? "LArm_Hand" : "RArm_Hand";
            }
            return false;
        }

        outSnapshot.inPowerArmor = boneSnapshot.inPowerArmor;
        outSnapshot.palmNormalWorld = normalizedOrFallback(
            debug_axis_math::rotateNiLocalToWorld(handNode->world.rotate, RE::NiPoint3(0.0f, 0.0f, -1.0f)),
            RE::NiPoint3(0.0f, 0.0f, -1.0f));
        outSnapshot.palmNormalValid = true;

        for (std::size_t finger = 0; finger < outSnapshot.fingers.size(); ++finger) {
            auto& chain = outSnapshot.fingers[finger];
            hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> tipInput{};
            tipInput.extrapolateFromPrevious = true;
            tipInput.extrapolateAlongStartBoneAxis = true;
            for (std::size_t segment = 0; segment < chain.points.size(); ++segment) {
                const char* name = fingerBoneName(isLeft, finger, segment);
                const auto* node = name ?
                    findBone(name) :
                    nullptr;
                if (!node) {
                    if (outMissingBoneName) {
                        *outMissingBoneName = name ? name : "invalidFingerBone";
                    }
                    outSnapshot = Snapshot{};
                    return false;
                }
                chain.points[segment] = node->world.translate;
                if (segment == 1) tipInput.previous = node->world;
                if (segment == 2) tipInput.start = node->world;
            }
            const auto tipFrame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(tipInput);
            chain.tipSegmentCenterWorld = tipFrame.transform.translate;
            chain.tipDirectionWorld = tipFrame.xAxis;
            chain.tipGeometryValid = tipFrame.valid;
            chain.valid = true;
        }

        outSnapshot.valid = true;
        return true;
    }

    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName, SkeletonBoneCaptureSpace space)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::FingerBoneCapture);
        // No engine calls or callbacks occur between capture and copying the compact result.
        // Each calling thread owns its scratch; transforms are freshly captured on every call.
        thread_local FingerCaptureScratch scratch;
        auto& boneSnapshot = scratch.bones;
        if (!scratch.reader.capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                space,
                boneSnapshot)) {
            outSnapshot = Snapshot{};
            if (outMissingBoneName) {
                *outMissingBoneName = "rootFlattenedBoneTree";
            }
            return false;
        }
        return buildFingerSkeletonSnapshot(
            boneSnapshot,
            isLeft,
            outSnapshot,
            outMissingBoneName, &scratch.names);
    }
}
