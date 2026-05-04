#include "DirectSkeletonBoneReader.h"

#include <algorithm>
#include <array>
#include <string_view>
#include <unordered_set>

#include "PhysicsLog.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace frik::rock
{
    namespace
    {
        using f4cf::f4vr::BSFlattenedBoneTree;
        using skeleton_bone_debug_math::DebugSkeletonBoneMode;
        using skeleton_bone_debug_math::DebugSkeletonBoneSource;
        using skeleton_bone_debug_math::SkeletonBoneSnapshotSource;

        constexpr int kMaxFlattenedBoneTransforms = 768;

        struct ResolvedTreeSource
        {
            BSFlattenedBoneTree* tree = nullptr;
            RE::NiNode* skeleton = nullptr;
            SkeletonBoneSnapshotSource source = SkeletonBoneSnapshotSource::None;
        };

        std::vector<std::string_view> requiredNamesForMode(DebugSkeletonBoneMode mode)
        {
            std::vector<std::string_view> names;
            if (mode == DebugSkeletonBoneMode::Off || mode == DebugSkeletonBoneMode::AllFlattenedBones) {
                return names;
            }

            if (mode == DebugSkeletonBoneMode::CoreBodyAndFingers) {
                const auto& core = skeleton_bone_debug_math::requiredCoreBoneNames();
                names.insert(names.end(), core.begin(), core.end());
            } else if (mode == DebugSkeletonBoneMode::HandsAndForearmsOnly) {
                for (std::string_view name : std::array<std::string_view, 16>{
                         "LArm_ForeArm1", "LArm_ForeArm2", "LArm_ForeArm3", "LArm_Hand",
                         "RArm_ForeArm1", "RArm_ForeArm2", "RArm_ForeArm3", "RArm_Hand",
                         "LArm_UpperArm", "RArm_UpperArm", "LArm_Collarbone", "RArm_Collarbone",
                         "LArm_UpperTwist1", "LArm_UpperTwist2", "RArm_UpperTwist1", "RArm_UpperTwist2" }) {
                    names.push_back(name);
                }
            }

            const auto& fingers = skeleton_bone_debug_math::requiredFingerBoneNames();
            names.insert(names.end(), fingers.begin(), fingers.end());
            return names;
        }

        bool validTree(const BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= kMaxFlattenedBoneTransforms;
        }

        std::string_view transformName(const BSFlattenedBoneTree::BoneTransforms& transform)
        {
            const char* name = transform.name.c_str();
            return name ? std::string_view(name) : std::string_view{};
        }

        ResolvedTreeSource resolveTreeSource(DebugSkeletonBoneSource source)
        {
            if (source == DebugSkeletonBoneSource::FirstPersonDiagnosticOnly) {
                return ResolvedTreeSource{
                    .tree = f4vr::getFirstPersonBoneTree(),
                    .skeleton = f4vr::getFirstPersonSkeleton(),
                    .source = SkeletonBoneSnapshotSource::FirstPersonDiagnosticOnly,
                };
            }

            return ResolvedTreeSource{
                .tree = f4vr::getFlattenedBoneTree(),
                .skeleton = f4vr::getRootNode(),
                .source = SkeletonBoneSnapshotSource::GameRootFlattenedBoneTree,
            };
        }
    }

    void DirectSkeletonBoneReader::resetCache()
    {
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedSource = SkeletonBoneSnapshotSource::None;
        _cachedMode = DebugSkeletonBoneMode::Off;
        _cachedTotalBoneCount = 0;
        _cachedInPowerArmor = false;
        _missingSourceLogged = false;
        _cachedRequiredResolvedCount = 0;
        _cachedBones.clear();
        _cachedMissingRequiredBones.clear();
    }

    bool DirectSkeletonBoneReader::capture(
        DebugSkeletonBoneMode mode,
        DebugSkeletonBoneSource source,
        DirectSkeletonBoneSnapshot& outSnapshot)
    {
        outSnapshot = DirectSkeletonBoneSnapshot{};
        mode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(static_cast<int>(mode));
        source = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(static_cast<int>(source));
        if (mode == DebugSkeletonBoneMode::Off) {
            resetCache();
            return false;
        }

        const bool inPowerArmor = f4vr::isInPowerArmor();
        const ResolvedTreeSource resolved = resolveTreeSource(source);
        if (!validTree(resolved.tree)) {
            if (!_missingSourceLogged || _cachedSource != resolved.source) {
                ROCK_LOG_WARN(Hand,
                    "Direct skeleton bone reader source unavailable: source={} skeleton={} tree={} mode={}",
                    skeleton_bone_debug_math::snapshotSourceName(resolved.source),
                    reinterpret_cast<std::uintptr_t>(resolved.skeleton),
                    reinterpret_cast<std::uintptr_t>(resolved.tree),
                    skeleton_bone_debug_math::modeName(mode));
                _missingSourceLogged = true;
            }
            _cachedSkeleton = resolved.skeleton;
            _cachedBoneTree = resolved.tree;
            _cachedSource = resolved.source;
            _cachedMode = mode;
            _cachedTotalBoneCount = 0;
            _cachedRequiredResolvedCount = 0;
            _cachedBones.clear();
            _cachedMissingRequiredBones.clear();
            return false;
        }

        _missingSourceLogged = false;
        if (_cachedSkeleton != resolved.skeleton ||
            _cachedBoneTree != resolved.tree ||
            _cachedSource != resolved.source ||
            _cachedMode != mode ||
            _cachedTotalBoneCount != resolved.tree->numTransforms ||
            _cachedInPowerArmor != inPowerArmor) {
            if (!rebuildTreeCache(resolved.skeleton, resolved.tree, resolved.source, mode, inPowerArmor)) {
                resetCache();
                return false;
            }
        }
        return captureFromCachedTree(outSnapshot);
    }

    bool DirectSkeletonBoneReader::rebuildTreeCache(void* skeleton, void* boneTree, SkeletonBoneSnapshotSource source, DebugSkeletonBoneMode mode, bool inPowerArmor)
    {
        auto* tree = static_cast<BSFlattenedBoneTree*>(boneTree);
        if (!validTree(tree)) {
            return false;
        }

        _cachedBones.clear();
        _cachedMissingRequiredBones.clear();
        _cachedSkeleton = skeleton;
        _cachedBoneTree = boneTree;
        _cachedSource = source;
        _cachedMode = mode;
        _cachedTotalBoneCount = tree->numTransforms;
        _cachedInPowerArmor = inPowerArmor;
        _cachedRequiredResolvedCount = 0;

        std::vector<int> parentIndices(static_cast<std::size_t>(tree->numTransforms), -1);
        std::vector<bool> included(static_cast<std::size_t>(tree->numTransforms), false);
        std::vector<int> treeToSnapshot(static_cast<std::size_t>(tree->numTransforms), -1);
        std::unordered_set<std::string_view> resolvedRequired;
        const auto requiredNames = requiredNamesForMode(mode);

        for (int i = 0; i < tree->numTransforms; ++i) {
            const auto& transform = tree->transforms[i];
            const std::string_view name = transformName(transform);
            parentIndices[static_cast<std::size_t>(i)] = transform.parPos;
            included[static_cast<std::size_t>(i)] = skeleton_bone_debug_math::shouldIncludeBone(mode, name);
            if (std::find(requiredNames.begin(), requiredNames.end(), name) != requiredNames.end()) {
                resolvedRequired.insert(name);
            }
        }

        _cachedRequiredResolvedCount = static_cast<int>(resolvedRequired.size());
        for (const std::string_view required : requiredNames) {
            if (resolvedRequired.find(required) == resolvedRequired.end()) {
                _cachedMissingRequiredBones.emplace_back(required);
            }
        }

        for (int i = 0; i < tree->numTransforms; ++i) {
            if (!included[static_cast<std::size_t>(i)]) {
                continue;
            }

            const std::string_view name = transformName(tree->transforms[i]);
            treeToSnapshot[static_cast<std::size_t>(i)] = static_cast<int>(_cachedBones.size());
            _cachedBones.push_back(CachedBone{
                .name = std::string(name),
                .treeIndex = i,
                .parentTreeIndex = parentIndices[static_cast<std::size_t>(i)],
                .drawableParentSnapshotIndex = -1,
                .included = true,
            });
        }

        for (auto& bone : _cachedBones) {
            const int drawableParentTreeIndex =
                skeleton_bone_debug_math::resolveDrawableParentIndex(static_cast<std::size_t>(bone.treeIndex), parentIndices, included);
            if (drawableParentTreeIndex >= 0 && static_cast<std::size_t>(drawableParentTreeIndex) < treeToSnapshot.size()) {
                bone.drawableParentSnapshotIndex = treeToSnapshot[static_cast<std::size_t>(drawableParentTreeIndex)];
            }
        }

        ROCK_LOG_INFO(Hand,
            "Direct skeleton bone cache rebuilt: source={} skeleton={} tree={} total={} included={} required={}/{} mode={} powerArmor={}",
            skeleton_bone_debug_math::snapshotSourceName(source),
            reinterpret_cast<std::uintptr_t>(skeleton),
            reinterpret_cast<std::uintptr_t>(boneTree),
            _cachedTotalBoneCount,
            _cachedBones.size(),
            _cachedRequiredResolvedCount,
            requiredNames.size(),
            skeleton_bone_debug_math::modeName(mode),
            _cachedInPowerArmor);

        if (!_cachedMissingRequiredBones.empty()) {
            std::string missing;
            for (const auto& name : _cachedMissingRequiredBones) {
                if (!missing.empty()) {
                    missing += ",";
                }
                missing += name;
            }
            ROCK_LOG_WARN(Hand, "Direct skeleton bone cache missing required bones: {}", missing);
        }

        return !_cachedBones.empty();
    }

    bool DirectSkeletonBoneReader::captureFromCachedTree(DirectSkeletonBoneSnapshot& outSnapshot)
    {
        auto* tree = static_cast<BSFlattenedBoneTree*>(_cachedBoneTree);
        if (!validTree(tree)) {
            resetCache();
            return false;
        }

        outSnapshot.valid = true;
        outSnapshot.inPowerArmor = _cachedInPowerArmor;
        outSnapshot.mode = _cachedMode;
        outSnapshot.source = _cachedSource;
        outSnapshot.skeleton = _cachedSkeleton;
        outSnapshot.boneTree = _cachedBoneTree;
        outSnapshot.totalBoneCount = _cachedTotalBoneCount;
        outSnapshot.requiredResolvedCount = _cachedRequiredResolvedCount;
        outSnapshot.missingRequiredBones = _cachedMissingRequiredBones;
        outSnapshot.bones.reserve(_cachedBones.size());

        for (const auto& cached : _cachedBones) {
            if (cached.treeIndex < 0 || cached.treeIndex >= tree->numTransforms) {
                continue;
            }

            outSnapshot.bones.push_back(DirectSkeletonBoneEntry{
                .name = cached.name,
                .treeIndex = cached.treeIndex,
                .parentTreeIndex = cached.parentTreeIndex,
                .drawableParentSnapshotIndex = cached.drawableParentSnapshotIndex,
                .world = tree->transforms[cached.treeIndex].world,
                .included = cached.included,
            });
        }

        outSnapshot.valid = !outSnapshot.bones.empty();
        return outSnapshot.valid;
    }
}
