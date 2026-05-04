#include "RootFlattenedFingerSkeletonRuntime.h"

/*
 * Runtime lookup is intentionally kept outside the header so pure math tests
 * can validate finger-chain conventions without depending on live FO4VR scene
 * objects. Production code resolves the same root flattened bone tree used by
 * generated hand colliders and returns a compact world-space snapshot to the
 * pose solver and debug overlay.
 */

#include "DebugAxisMath.h"
#include "DirectSkeletonBoneReader.h"

#include <string_view>

namespace frik::rock::root_flattened_finger_skeleton_runtime
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

    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName)
    {
        outSnapshot = Snapshot{};
        if (outMissingBoneName) {
            outMissingBoneName->clear();
        }

        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedFingerReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            if (outMissingBoneName) {
                *outMissingBoneName = "rootFlattenedBoneTree";
            }
            return false;
        }

        const auto* handNode = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!handNode) {
            if (outMissingBoneName) {
                *outMissingBoneName = isLeft ? "LArm_Hand" : "RArm_Hand";
            }
            return false;
        }

        outSnapshot.palmNormalWorld = normalizedOrFallback(
            debug_axis_math::rotateNiLocalToWorld(handNode->world.rotate, RE::NiPoint3(0.0f, 0.0f, -1.0f)),
            RE::NiPoint3(0.0f, 0.0f, -1.0f));
        outSnapshot.palmNormalValid = true;

        for (std::size_t finger = 0; finger < outSnapshot.fingers.size(); ++finger) {
            auto& chain = outSnapshot.fingers[finger];
            for (std::size_t segment = 0; segment < chain.points.size(); ++segment) {
                const char* name = fingerBoneName(isLeft, finger, segment);
                const auto* node = name ? findSnapshotBone(snapshot, name) : nullptr;
                if (!node) {
                    if (outMissingBoneName) {
                        *outMissingBoneName = name ? name : "invalidFingerBone";
                    }
                    outSnapshot = Snapshot{};
                    return false;
                }
                chain.points[segment] = node->world.translate;
            }
            chain.valid = true;
        }

        outSnapshot.valid = true;
        return true;
    }
}
