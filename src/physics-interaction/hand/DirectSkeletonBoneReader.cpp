#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <algorithm>
#include <array>
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
        // Keep caller-owned storage on successful captures; every transform and
        // validity bit is replaced below, including when a refNode disappears.
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
                .chainSide = rendered_bone_transport_policy::chainSideForBone(name),
                .armSide = arm_presentation_policy::armSideForBone(name),
                .armSegment = arm_presentation_policy::armSegmentForBone(name),
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

        /*
         * Controller space carries each hand chain by the hand world
         * authority's per-frame transport (isolated controller root versus
         * rendered root). On claim-free frames it is inactive; without an
         * isolation result the bones stay rendered, which the authority
         * counts as a claimed frame without transport.
         */
        namespace transport_policy = rendered_bone_transport_policy;
        std::array<transport_policy::HandTransport, 2> transports{};
        if (space == SkeletonBoneCaptureSpace::Controller) {
            for (std::size_t hand = 0; hand < transports.size(); ++hand) {
                (void)frik_hand_world_authority::tryGetHandChainTransport(hand == 1, transports[hand]);
            }
        }
        const auto transportFor = [&transports](const transport_policy::HandChainSide side) -> const transport_policy::HandTransport* {
            switch (side) {
            case transport_policy::HandChainSide::Right:
                return &transports[0];
            case transport_policy::HandChainSide::Left:
                return &transports[1];
            default:
                return nullptr;
            }
        };

        outSnapshot.valid = true;
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
            if (const transport_policy::HandTransport* transport = transportFor(cached.chainSide); transport && transport->active) {
                entry.world = transport_policy::transportWorld(*transport, entry.world);
                if (entry.nodeWorldValid) {
                    entry.nodeWorld = transport_policy::transportWorld(*transport, entry.nodeWorld);
                }
            }
        }

        outSnapshot.bones.resize(capturedCount);
        outSnapshot.valid = !outSnapshot.bones.empty();
        return outSnapshot.valid;
    }

    bool DirectSkeletonBoneReader::presentCachedArm(
        const rendered_bone_transport_policy::HandChainSide side,
        const RE::NiTransform& handDelta,
        float& outElbowMoveGameUnits,
        float& outReachDeficitGameUnits)
    {
        namespace arm_policy = arm_presentation_policy;
        namespace transport_policy = rendered_bone_transport_policy;
        outElbowMoveGameUnits = 0.0f;
        outReachDeficitGameUnits = 0.0f;
        auto* tree = static_cast<BSFlattenedBoneTree*>(_cachedBoneTree);
        if (side == transport_policy::HandChainSide::None || !validTree(tree) ||
            !tracked_hand_isolation_policy::isFiniteTransform(handDelta)) {
            return false;
        }

        struct BoneWrite
        {
            int treeIndex = -1;
            int parentTreeIndex = -1;
            int parentWrite = -1;
            arm_policy::ArmSegment segment = arm_policy::ArmSegment::None;
            bool nodeValid = false;
            bool refParentValid = false;
            RE::NiNode* refNode = nullptr;
            RE::NiNode* refParent = nullptr;
            RE::NiTransform world{};
            RE::NiTransform nodeWorld{};
            RE::NiTransform refParentWorld{};
            RE::NiTransform newWorld{};
            RE::NiTransform newNodeWorld{};
        };
        // Upper arm and two twists, three forearm bones, the hand, fifteen fingers.
        constexpr std::size_t kMaxArmBones = 32;
        std::array<BoneWrite, kMaxArmBones> writes{};
        std::size_t count = 0;
        int shoulder = -1;
        int elbow = -1;
        int wrist = -1;

        // Validate the whole arm before the first write so a torn tree cannot
        // leave a half-carried arm.
        for (const auto& cached : _cachedBones) {
            if (cached.armSide != side || cached.armSegment == arm_policy::ArmSegment::None) {
                continue;
            }
            if (count >= kMaxArmBones || cached.treeIndex < 0 || cached.treeIndex >= tree->numTransforms) {
                return false;
            }
            BoneWrite& write = writes[count];
            write.treeIndex = cached.treeIndex;
            write.parentTreeIndex = cached.parentTreeIndex;
            write.segment = cached.armSegment;
            const auto& entry = tree->transforms[cached.treeIndex];
            write.world = entry.world;
            if (!tracked_hand_isolation_policy::isFiniteTransform(write.world)) {
                return false;
            }
            if (RE::NiNode* refNode = entry.refNode) {
                if (!native_memory::tryReadValue(&refNode->world, write.nodeWorld) ||
                    !native_memory::tryReadValue(&refNode->parent, write.refParent) ||
                    !tracked_hand_isolation_policy::isFiniteTransform(write.nodeWorld)) {
                    return false;
                }
                write.refNode = refNode;
                write.nodeValid = true;
                if (write.refParent) {
                    write.refParentValid = native_memory::tryReadValue(&write.refParent->world, write.refParentWorld);
                }
            }
            const std::string_view tail = std::string_view(cached.name).substr(5);
            if (tail == "UpperArm") {
                shoulder = static_cast<int>(count);
            } else if (tail == "ForeArm1") {
                elbow = static_cast<int>(count);
            } else if (tail == "Hand") {
                wrist = static_cast<int>(count);
            }
            ++count;
        }
        if (shoulder < 0 || elbow < 0 || wrist < 0) {
            return false;
        }
        const arm_policy::ArmCarry carry = arm_policy::planArmCarry(
            writes[shoulder].world.translate,
            writes[elbow].world.translate,
            writes[wrist].world.translate,
            handDelta);
        if (!carry.valid) {
            return false;
        }

        for (std::size_t i = 0; i < count; ++i) {
            BoneWrite& write = writes[i];
            const transport_policy::HandTransport transport{ .delta = arm_policy::carryForSegment(carry, write.segment), .active = true };
            write.newWorld = transport_policy::transportWorld(transport, write.world);
            if (write.nodeValid) {
                write.newNodeWorld = transport_policy::transportWorld(transport, write.nodeWorld);
            }
            for (std::size_t j = 0; j < count; ++j) {
                if (writes[j].treeIndex == write.parentTreeIndex) {
                    write.parentWrite = static_cast<int>(j);
                    break;
                }
            }
        }

        for (std::size_t i = 0; i < count; ++i) {
            const BoneWrite& write = writes[i];
            auto& entry = tree->transforms[write.treeIndex];
            entry.world = write.newWorld;
            // A bone whose parent moved with it keeps its local; the others are
            // re-expressed under their parent's new, or unmoved, world.
            const bool parentCarriedAlong = write.parentWrite >= 0 && writes[write.parentWrite].segment == write.segment;
            if (!parentCarriedAlong) {
                if (write.parentWrite >= 0) {
                    entry.local = transport_policy::localUnderParent(writes[write.parentWrite].newWorld, write.newWorld);
                } else if (write.parentTreeIndex >= 0 && write.parentTreeIndex < tree->numTransforms) {
                    entry.local = transport_policy::localUnderParent(tree->transforms[write.parentTreeIndex].world, write.newWorld);
                }
            }
            if (!write.nodeValid) {
                continue;
            }
            (void)native_memory::tryWriteValue(&write.refNode->world, write.newNodeWorld);
            if (parentCarriedAlong) {
                continue;
            }
            if (write.parentWrite >= 0 && writes[write.parentWrite].nodeValid) {
                (void)native_memory::tryWriteValue(&write.refNode->local, transport_policy::localUnderParent(writes[write.parentWrite].newNodeWorld, write.newNodeWorld));
            } else if (write.refParentValid) {
                (void)native_memory::tryWriteValue(&write.refNode->local, transport_policy::localUnderParent(write.refParentWorld, write.newNodeWorld));
            }
        }
        outElbowMoveGameUnits = carry.elbowMoveGameUnits;
        outReachDeficitGameUnits = carry.reachDeficitGameUnits;
        return true;
    }
}
