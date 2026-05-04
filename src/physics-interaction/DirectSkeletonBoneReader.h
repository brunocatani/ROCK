#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "SkeletonBoneDebugMath.h"

#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock
{
    struct DirectSkeletonBoneEntry
    {
        std::string name;
        int treeIndex = -1;
        int parentTreeIndex = -1;
        int drawableParentSnapshotIndex = -1;
        RE::NiTransform world{};
        bool included = false;
    };

    struct DirectSkeletonBoneSnapshot
    {
        bool valid = false;
        bool inPowerArmor = false;
        skeleton_bone_debug_math::DebugSkeletonBoneMode mode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource source = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        const void* skeleton = nullptr;
        const void* boneTree = nullptr;
        int totalBoneCount = 0;
        int requiredResolvedCount = 0;
        std::vector<DirectSkeletonBoneEntry> bones;
        std::vector<std::string> missingRequiredBones;
    };

    class DirectSkeletonBoneReader
    {
    public:
        bool capture(
            skeleton_bone_debug_math::DebugSkeletonBoneMode mode,
            skeleton_bone_debug_math::DebugSkeletonBoneSource source,
            DirectSkeletonBoneSnapshot& outSnapshot);
        void resetCache();

    private:
        struct CachedBone
        {
            std::string name;
            int treeIndex = -1;
            int parentTreeIndex = -1;
            int drawableParentSnapshotIndex = -1;
            bool included = false;
        };

        bool rebuildTreeCache(
            void* skeleton,
            void* boneTree,
            skeleton_bone_debug_math::SkeletonBoneSnapshotSource source,
            skeleton_bone_debug_math::DebugSkeletonBoneMode mode,
            bool inPowerArmor);

        bool captureFromCachedTree(DirectSkeletonBoneSnapshot& outSnapshot);

        const void* _cachedSkeleton = nullptr;
        void* _cachedBoneTree = nullptr;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource _cachedSource = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        skeleton_bone_debug_math::DebugSkeletonBoneMode _cachedMode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        int _cachedTotalBoneCount = 0;
        bool _cachedInPowerArmor = false;
        bool _missingSourceLogged = false;
        int _cachedRequiredResolvedCount = 0;
        std::vector<CachedBone> _cachedBones;
        std::vector<std::string> _cachedMissingRequiredBones;
    };
}
