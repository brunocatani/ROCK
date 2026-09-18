#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <algorithm>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        using f4vr::BSFlattenedBoneTree;
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
                const auto& handsAndForearms = skeleton_bone_debug_math::handsAndForearmsBoneNames();
                names.insert(names.end(), handsAndForearms.begin(), handsAndForearms.end());
            }

            const auto& fingers = skeleton_bone_debug_math::requiredFingerBoneNames();
            names.insert(names.end(), fingers.begin(), fingers.end());
            return names;
        }

        bool validTree(const BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= kMaxFlattenedBoneTransforms;
        }

        RE::NiNode* safeWorldRootNode()
        {
            return f4vr::getWorldRootNode();
        }

        RE::NiNode* safeRootNode(RE::NiNode* worldRoot)
        {
            if (!worldRoot || worldRoot->children.empty() || !worldRoot->children[0]) {
                return nullptr;
            }

            return worldRoot->children[0]->IsNode();
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

            auto* worldRoot = safeWorldRootNode();
            auto* rootNode = safeRootNode(worldRoot);
            return ResolvedTreeSource{
                .tree = rootNode ? reinterpret_cast<BSFlattenedBoneTree*>(rootNode) : nullptr,
                .skeleton = rootNode,
                .source = SkeletonBoneSnapshotSource::GameRootFlattenedBoneTree,
            };
        }
    }

    void logHandBoneCacheResolved(const void* skeleton, const void* boneTree, bool inPowerArmor)
    {
        ROCK_LOG_DEBUG(Hand,
            "HandBoneCache resolved rootFlattenedTree skeleton={:p} tree={:p} powerArmor={}",
            skeleton,
            boneTree,
            inPowerArmor ? "yes" : "no");
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
        const SkeletonBoneCaptureSpace space,
        DirectSkeletonBoneSnapshot& outSnapshot)
    {
        // Reuse copied storage, but replace every transform and validity bit.
        // Controller transport below always starts from this capture's array.
        outSnapshot.valid = false;
        mode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(static_cast<int>(mode));
        source = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(static_cast<int>(source));
        if (mode == DebugSkeletonBoneMode::Off) {
            resetCache();
            outSnapshot = {};
            return false;
        }

        const bool inPowerArmor = f4vr::isInPowerArmor();
        const ResolvedTreeSource resolved = resolveTreeSource(source);
        if (!validTree(resolved.tree)) {
            outSnapshot = {};
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
                outSnapshot = {};
                return false;
            }
        }
        const bool captured = captureFromCachedTree(outSnapshot, space);
        if (captured) {
            performance_profiler::observeValue(space == SkeletonBoneCaptureSpace::Rendered ?
                performance_profiler::ValueMetric::RenderedSkeletonBones : performance_profiler::ValueMetric::ControllerSkeletonBones,
                outSnapshot.bones.size());
        } else {
            outSnapshot = {};
        }
        return captured;
    }

    bool DirectSkeletonBoneReader::rebuildTreeCache(void* skeleton, void* boneTree, SkeletonBoneSnapshotSource source, DebugSkeletonBoneMode mode, bool inPowerArmor)
    {
        auto* tree = static_cast<BSFlattenedBoneTree*>(boneTree);
        if (!validTree(tree)) {
            return false;
        }

        ++_topologyRevision;
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
                missing += "'";
                if (name.empty()) {
                    missing += "<empty>";
                } else {
                    missing += name;
                }
                missing += "'";
            }
            ROCK_LOG_WARN(Hand,
                "Direct skeleton bone cache missing required bones count={}: {}",
                _cachedMissingRequiredBones.size(),
                missing);
        }

        return !_cachedBones.empty();
    }

    bool DirectSkeletonBoneReader::captureFromCachedTree(DirectSkeletonBoneSnapshot& outSnapshot, const SkeletonBoneCaptureSpace space)
    {
        auto* tree = static_cast<BSFlattenedBoneTree*>(_cachedBoneTree);
        if (!validTree(tree)) {
            resetCache();
            return false;
        }

        outSnapshot.valid = true;
        outSnapshot.topologyOwner = this;
        outSnapshot.topologyRevision = _topologyRevision;
        outSnapshot.inPowerArmor = _cachedInPowerArmor;
        outSnapshot.mode = _cachedMode;
        outSnapshot.source = _cachedSource;
        outSnapshot.space = space;
        outSnapshot.skeleton = _cachedSkeleton;
        outSnapshot.boneTree = _cachedBoneTree;
        outSnapshot.totalBoneCount = _cachedTotalBoneCount;
        outSnapshot.requiredResolvedCount = _cachedRequiredResolvedCount;
        outSnapshot.missingRequiredBones = _cachedMissingRequiredBones;
        outSnapshot.bones.resize(_cachedBones.size());
        std::size_t capturedCount = 0;

        for (const auto& cached : _cachedBones) {
            if (cached.treeIndex < 0 || cached.treeIndex >= tree->numTransforms) {
                continue;
            }

            auto& entry = outSnapshot.bones[capturedCount++];
            entry.name = cached.name;
            entry.treeIndex = cached.treeIndex;
            entry.parentTreeIndex = cached.parentTreeIndex;
            entry.drawableParentSnapshotIndex = cached.drawableParentSnapshotIndex;
            entry.world = tree->transforms[cached.treeIndex].world;
            entry.included = cached.included;
            entry.nodeWorld = {};
            entry.nodeWorldValid = false;
            // refNode is an engine scene pointer the tree owns; a guarded copy
            // keeps a torn tree from faulting the frame.
            if (const RE::NiNode* refNode = tree->transforms[cached.treeIndex].refNode) {
                entry.nodeWorldValid = native_memory::tryReadValue(&refNode->world, entry.nodeWorld);
            }
        }

        outSnapshot.bones.resize(capturedCount);
        if (space == SkeletonBoneCaptureSpace::Controller) {
            namespace transport = rendered_bone_transport_policy;
            for (const bool isLeft : { false, true }) {
                RE::NiTransform controllerRoot{};
                if (!frik_hand_world_authority::tryGetRawHandWorld(isLeft, controllerRoot) ||
                    !transport::transportSnapshotHand(outSnapshot.bones,
                        isLeft ? transport::HandChainSide::Left : transport::HandChainSide::Right,
                        controllerRoot)) {
                    ROCK_LOG_SAMPLE_WARN(Hand, 5000,
                        "Controller skeleton capture rejected: {} hand input or sampled bone chain unavailable", isLeft ? "left" : "right");
                    outSnapshot = {};
                    return false;
                }
            }
        }
        outSnapshot.valid = !outSnapshot.bones.empty();
        return outSnapshot.valid;
    }
}
