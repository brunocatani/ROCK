#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandColliderTypes.h"
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
        DirectSkeletonBoneReader& rootFlattenedFingerReader()
        {
            static DirectSkeletonBoneReader reader;
            return reader;
        }

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
        std::string* outMissingBoneName)
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

        const auto* handNode = findSnapshotBone(
            boneSnapshot,
            isLeft ? "LArm_Hand" : "RArm_Hand");
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
                    findSnapshotBone(boneSnapshot, name) :
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

    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::FingerBoneCapture);
        DirectSkeletonBoneSnapshot boneSnapshot{};
        if (!rootFlattenedFingerReader().capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Controller,
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
            outMissingBoneName);
    }
}
