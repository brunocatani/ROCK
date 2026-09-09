#pragma once

#include "physics-interaction/weapon/WeaponCollision.h"

#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeNiNodeFactory.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "RockConfig.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponSceneChildren.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/TransformMath.h"

#include <intrin.h>

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/MagicItems.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpShape.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <string_view>
#include <vector>

namespace RE
{
    /*
     * Fallout4VR 1.2.72 hknpCompoundShape::getShapeKeys at 0x1416E2430
     * and hknpCompressedMeshShape::getShapeKeys at 0x1416026D0 agree on
     * this 0x18-byte input layout. ROCK uses only the native all-zero
     * configuration: no parent prefix and no shape-key mask.
     */
    struct hknpShape::GetShapeKeysConfig
    {
        std::uint32_t parentShapeKey{ 0 };
        std::uint32_t parentShapeKeyBits{ 0 };
        hknpShapeKeyMask* shapeKeyMask{ nullptr };
        std::uint64_t reserved{ 0 };
    };
    static_assert(sizeof(hknpShape::GetShapeKeysConfig) == 0x18);
    static_assert(offsetof(hknpShape::GetShapeKeysConfig, shapeKeyMask) == 0x08);
}

/*
 * Shared internals of the WeaponCollision implementation, split across the
 * collision/ translation units. These helpers lived in the anonymous
 * namespace of the former single WeaponCollision.cpp; the using-directive
 * below preserves unqualified name lookup for the moved method bodies.
 * Include only from WeaponCollision implementation files.
 */

namespace rock
{
    namespace weapon_collision_internal
    {
        inline constexpr std::size_t MAX_CONVEX_HULL_POINTS = 0xFC;
        inline constexpr float MIN_HULL_DIAGONAL_GAME_UNITS = 0.5f;
        // Generated weapon collision has one fixed production geometry policy.
        inline constexpr float WEAPON_COLLISION_CONVEX_RADIUS_HAVOK = 0.01f;
        inline constexpr float WEAPON_COLLISION_POINT_DEDUP_GRID_HAVOK = 0.002f;
        inline constexpr std::size_t WEAPON_COLLISION_SUPPORT_FIT_TARGET_POINTS = 96;
        inline constexpr float WEAPON_COLLISION_SUPPORT_FIT_MAX_ERROR_GAME_UNITS = 0.5f;
        inline constexpr std::size_t MAX_GENERATED_CHILD_CONVEXES_PER_SOURCE = 16;
        inline constexpr std::size_t GENERATED_WEAPON_BODY_CREATION_BATCH = 8;
        inline constexpr float GENERATED_RECAPTURE_WEAPON_CENTER_DRIFT_GAME = 0.25f;
        inline constexpr float GENERATED_RECAPTURE_SOURCE_CENTER_DRIFT_GAME = 0.10f;
        inline constexpr std::size_t MAX_GENERATED_RECAPTURE_DETAIL_ROWS = 8;
        inline constexpr std::size_t MAX_COLLISION_SOUND_SHAPE_KEYS = 4096;
        inline constexpr std::size_t COLLISION_SOUND_SHAPE_KEY_BATCH = 64;
        inline constexpr std::size_t MAX_COLLISION_SOUND_HISTOGRAM_ROWS = 8;
        inline constexpr std::uint32_t INVALID_COLLISION_SOUND_SHAPE_KEY = 0xFFFF'FFFF;
        inline constexpr float GENERATED_SOURCE_COMPONENT_JOIN_TOLERANCE_GAME = 2.0f;
        inline constexpr float GENERATED_SOURCE_DETACHED_COMPONENT_MIN_GAP_GAME = 24.0f;
        inline constexpr std::size_t MAX_CACHED_DETACHED_SOURCE_GROUPS =
            weapon_collision_geometry_math::kMaxDetachedComponentAnalysisSources;

        struct QuantizedPointKey
        {
            std::int64_t x = 0;
            std::int64_t y = 0;
            std::int64_t z = 0;

            bool operator==(const QuantizedPointKey& rhs) const noexcept { return x == rhs.x && y == rhs.y && z == rhs.z; }
        };

        struct QuantizedPointKeyHash
        {
            std::size_t operator()(const QuantizedPointKey& key) const noexcept
            {
                const auto hx = std::hash<std::int64_t>{}(key.x);
                const auto hy = std::hash<std::int64_t>{}(key.y);
                const auto hz = std::hash<std::int64_t>{}(key.z);
                return hx ^ (hy + 0x9e3779b97f4a7c15ull + (hx << 6) + (hx >> 2)) ^ (hz + 0x9e3779b97f4a7c15ull + (hy << 6) + (hy >> 2));
            }
        };

        struct WeaponMeshRootCandidate
        {
            RE::NiAVObject* root = nullptr;
            const char* label = "";
        };

        struct PointCloudBounds
        {
            RE::NiPoint3 min{};
            RE::NiPoint3 max{};
        };

        enum GeneratedHullCoverageClass : int
        {
            HullCoverageStock = 0,
            HullCoverageReceiver = 1,
            HullCoverageBarrel = 2,
            HullCoverageMagazine = 3,
            HullCoverageTopAccessory = 4,
            HullCoverageAction = 5,
            HullCoverageOther = 6,
            HullCoverageCosmeticAmmo = 7
        };

        struct GeneratedHullCoverageInfo
        {
            int coverageClass = HullCoverageOther;
            int priority = 50;
            bool cosmetic = false;
            const char* label = "other";
        };

        struct GeneratedPointCloudClusterSet
        {
            std::vector<std::vector<RE::NiPoint3>> clusters;
            bool supportFitAttempted{ false };
            bool supportFitAccepted{ false };
            bool supportFitFallbackSplit{ false };
            float supportFitMaxError{ 0.0f };
            std::size_t supportFitInputPoints{ 0 };
            std::size_t supportFitOutputPoints{ 0 };
            std::size_t supportFitRepairPoints{ 0 };
            std::size_t supportFitValidationDirections{ 0 };
        };

        struct WeaponAnimNodeMatch
        {
            RE::NiAVObject* node{ nullptr };
            std::string path;
            std::uint32_t depth{ 0 };
        };

        struct WeaponAnimNodeSubtreeStats
        {
            std::uint32_t nodeCount{ 0 };
            std::uint32_t niNodeCount{ 0 };
            std::uint32_t triShapeCount{ 0 };
            std::uint32_t visibleTriShapeCount{ 0 };
            std::uint32_t hiddenFlagCount{ 0 };
            std::uint32_t appCulledCount{ 0 };
            std::uint32_t maxDepth{ 0 };
        };

        struct WeaponAnimNodeDumpRoot
        {
            const char* label{ "" };
            RE::NiAVObject* root{ nullptr };
        };

        struct WeaponAnimFlattenedRoot
        {
            const char* label{ "" };
            f4vr::BSFlattenedBoneTree* tree{ nullptr };
        };

        struct WeaponAnimFlattenedBoneMatch
        {
            int index{ -1 };
            int parentIndex{ -1 };
            short childPosition{ -1 };
            RE::NiNode* refNode{ nullptr };
            std::string name;
        };

        inline constexpr std::array<const char*, 11> WEAPON_ANIM_NODE_DUMP_TARGETS{
            "Weapon",
            "WeaponLeft",
            "ProjectileNode",
            "AnimObjectR1",
            "AnimObjectR2",
            "AnimObjectR3",
            "AnimObjectL1",
            "AnimObjectL2",
            "AnimObjectL3",
            "AnimObjectA",
            "AnimObjectB",
        };

        inline constexpr int WEAPON_ANIM_NODE_DUMP_MAX_DEPTH = 32;
        inline constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_MATCHES_PER_NAME = 32;
        inline constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES = 4096;
        inline constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_CHILD_NAMES = 16;
        inline constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES = 4096;
        inline constexpr int WEAPON_ANIM_NODE_DUMP_MAX_FLATTENED_TRANSFORMS = 768;

        bool weaponVisualNodeVisible(const RE::NiAVObject* node);
        const char* safeNodeName(const RE::NiAVObject* node);

        inline const char* generatedWeaponPartKindName(WeaponPartKind kind)
        {
            switch (kind) {
            case WeaponPartKind::Receiver:
                return "Receiver";
            case WeaponPartKind::Barrel:
                return "Barrel";
            case WeaponPartKind::Handguard:
                return "Handguard";
            case WeaponPartKind::Foregrip:
                return "Foregrip";
            case WeaponPartKind::Pump:
                return "Pump";
            case WeaponPartKind::Stock:
                return "Stock";
            case WeaponPartKind::Grip:
                return "Grip";
            case WeaponPartKind::Magazine:
                return "Magazine";
            case WeaponPartKind::Magwell:
                return "Magwell";
            case WeaponPartKind::Bolt:
                return "Bolt";
            case WeaponPartKind::Slide:
                return "Slide";
            case WeaponPartKind::ChargingHandle:
                return "ChargingHandle";
            case WeaponPartKind::BreakAction:
                return "BreakAction";
            case WeaponPartKind::Cylinder:
                return "Cylinder";
            case WeaponPartKind::Chamber:
                return "Chamber";
            case WeaponPartKind::Shell:
                return "Shell";
            case WeaponPartKind::Round:
                return "Round";
            case WeaponPartKind::LaserCell:
                return "LaserCell";
            case WeaponPartKind::Lever:
                return "Lever";
            case WeaponPartKind::Sight:
                return "Sight";
            case WeaponPartKind::Accessory:
                return "Accessory";
            case WeaponPartKind::CosmeticAmmo:
                return "CosmeticAmmo";
            case WeaponPartKind::Other:
                return "Other";
            case WeaponPartKind::LaserSight:
                return "LaserSight";
            case WeaponPartKind::Flashlight:
                return "Flashlight";
            case WeaponPartKind::LaserFlashlightCombo:
                return "LaserFlashlightCombo";
            case WeaponPartKind::Scope:
                return "Scope";
            case WeaponPartKind::MuzzleDevice:
                return "MuzzleDevice";
            case WeaponPartKind::Bipod:
                return "Bipod";
            case WeaponPartKind::Count:
            default:
                return "Invalid";
            }
        }

        inline bool weaponAnimNodeNameMatches(const RE::NiAVObject* node, const char* targetName)
        {
            if (!node || !targetName) {
                return false;
            }
            return _stricmp(targetName, node->name.c_str()) == 0;
        }

        inline void collectWeaponAnimNodeMatchesRecursive(
            RE::NiAVObject* node,
            const char* targetName,
            std::string path,
            std::uint32_t depth,
            std::size_t& visited,
            std::vector<WeaponAnimNodeMatch>& outMatches)
        {
            if (!node || visited >= WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES || depth > WEAPON_ANIM_NODE_DUMP_MAX_DEPTH ||
                outMatches.size() >= WEAPON_ANIM_NODE_DUMP_MAX_MATCHES_PER_NAME) {
                return;
            }

            ++visited;
            const char* nodeName = node->name.c_str();
            if (!nodeName || nodeName[0] == '\0') {
                nodeName = "(unnamed)";
            }
            if (path.empty()) {
                path = nodeName;
            } else {
                path += "/";
                path += nodeName;
            }

            if (weaponAnimNodeNameMatches(node, targetName)) {
                outMatches.push_back(WeaponAnimNodeMatch{
                    .node = node,
                    .path = path,
                    .depth = depth,
                });
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.capacity(); ++i) {
                if (auto* child = children[i].get()) {
                    collectWeaponAnimNodeMatchesRecursive(child, targetName, path, depth + 1, visited, outMatches);
                }
            }
        }

        inline std::vector<WeaponAnimNodeMatch> collectWeaponAnimNodeMatches(RE::NiAVObject* root, const char* targetName)
        {
            std::vector<WeaponAnimNodeMatch> matches;
            std::size_t visited = 0;
            collectWeaponAnimNodeMatchesRecursive(root, targetName, {}, 0, visited, matches);
            return matches;
        }

        inline void accumulateWeaponAnimNodeSubtreeStats(
            RE::NiAVObject* node,
            WeaponAnimNodeSubtreeStats& stats,
            std::uint32_t depth,
            std::size_t& visited)
        {
            if (!node || visited >= WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES) {
                return;
            }

            ++visited;
            ++stats.nodeCount;
            stats.maxDepth = (std::max)(stats.maxDepth, depth);
            if ((node->flags.flags & 1) != 0) {
                ++stats.hiddenFlagCount;
            }
            if (node->GetAppCulled()) {
                ++stats.appCulledCount;
            }

            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                if (weaponVisualNodeVisible(node)) {
                    ++stats.visibleTriShapeCount;
                }
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            ++stats.niNodeCount;
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.capacity(); ++i) {
                if (auto* child = children[i].get()) {
                    accumulateWeaponAnimNodeSubtreeStats(child, stats, depth + 1, visited);
                }
            }
        }

        inline WeaponAnimNodeSubtreeStats summarizeWeaponAnimNodeSubtree(RE::NiAVObject* node)
        {
            WeaponAnimNodeSubtreeStats stats{};
            std::size_t visited = 0;
            accumulateWeaponAnimNodeSubtreeStats(node, stats, 0, visited);
            return stats;
        }

        inline std::string weaponAnimNodeImmediateChildNames(RE::NiAVObject* node)
        {
            auto* niNode = node ? node->IsNode() : nullptr;
            if (!niNode) {
                return "";
            }

            std::string result;
            const auto& children = niNode->children;
            std::size_t appended = 0;
            for (auto i = decltype(children.size()){ 0 }; i < children.capacity() && appended < WEAPON_ANIM_NODE_DUMP_MAX_CHILD_NAMES; ++i) {
                const auto* child = children[i].get();
                if (!child) {
                    continue;
                }
                if (!result.empty()) {
                    result += "|";
                }
                const char* childName = child->name.c_str();
                result += childName && childName[0] != '\0' ? childName : "(unnamed)";
                ++appended;
            }
            if (children.size() > appended) {
                result += "|+";
                result += std::to_string(children.size() - appended);
                result += " more";
            }
            return result;
        }

        inline bool weaponAnimFlattenedTreeValid(const f4vr::BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= WEAPON_ANIM_NODE_DUMP_MAX_FLATTENED_TRANSFORMS;
        }

        inline const char* weaponAnimFlattenedTransformName(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform)
        {
            const char* name = transform.name.c_str();
            return name && name[0] != '\0' ? name : "(unnamed)";
        }

        inline bool weaponAnimFlattenedTransformNameMatches(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform, const char* targetName)
        {
            if (!targetName) {
                return false;
            }

            const char* name = transform.name.c_str();
            return name && _stricmp(targetName, name) == 0;
        }

        inline std::vector<WeaponAnimFlattenedBoneMatch> collectWeaponAnimFlattenedBoneMatches(
            f4vr::BSFlattenedBoneTree* tree,
            const char* targetName)
        {
            std::vector<WeaponAnimFlattenedBoneMatch> matches;
            if (!weaponAnimFlattenedTreeValid(tree)) {
                return matches;
            }

            for (int index = 0; index < tree->numTransforms &&
                                matches.size() < WEAPON_ANIM_NODE_DUMP_MAX_MATCHES_PER_NAME;
                 ++index) {
                const auto& transform = tree->transforms[index];
                if (!weaponAnimFlattenedTransformNameMatches(transform, targetName)) {
                    continue;
                }

                matches.push_back(WeaponAnimFlattenedBoneMatch{
                    .index = index,
                    .parentIndex = transform.parPos,
                    .childPosition = transform.childPos,
                    .refNode = transform.refNode,
                    .name = weaponAnimFlattenedTransformName(transform),
                });
            }

            return matches;
        }

        inline const char* weaponAnimFlattenedParentName(const f4vr::BSFlattenedBoneTree* tree, int parentIndex)
        {
            if (!weaponAnimFlattenedTreeValid(tree) || parentIndex < 0 || parentIndex >= tree->numTransforms) {
                return "(none)";
            }

            return weaponAnimFlattenedTransformName(tree->transforms[parentIndex]);
        }

        inline void logWeaponAnimNodeMapRoot(const WeaponAnimNodeDumpRoot& dumpRoot)
        {
            auto* root = dumpRoot.root;
            auto* niNode = root ? root->IsNode() : nullptr;
            const auto childCount = niNode ? niNode->children.size() : 0;
            const auto childNames = weaponAnimNodeImmediateChildNames(root);
            const auto rootStats = root ? summarizeWeaponAnimNodeSubtree(root) : WeaponAnimNodeSubtreeStats{};

            ROCK_LOG_INFO(Weapon,
                "WeaponAnimMap root='{}' kind=node addr=0x{:X} name='{}' children={} childNames='{}' flags=0x{:X} appCulled={} visible={} subtreeNodes={} niNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} subtreeMaxDepth={}",
                dumpRoot.label,
                reinterpret_cast<std::uintptr_t>(root),
                safeNodeName(root),
                static_cast<std::size_t>(childCount),
                childNames,
                static_cast<std::uint32_t>(root ? root->flags.flags : 0),
                root && root->GetAppCulled() ? "yes" : "no",
                root && weaponVisualNodeVisible(root) ? "yes" : "no",
                rootStats.nodeCount,
                rootStats.niNodeCount,
                rootStats.triShapeCount,
                rootStats.visibleTriShapeCount,
                rootStats.hiddenFlagCount,
                rootStats.appCulledCount,
                rootStats.maxDepth);

            for (const char* targetName : WEAPON_ANIM_NODE_DUMP_TARGETS) {
                auto matches = collectWeaponAnimNodeMatches(root, targetName);
                ROCK_LOG_INFO(Weapon,
                    "WeaponAnimMap root='{}' kind=node target='{}' matches={}",
                    dumpRoot.label,
                    targetName,
                    matches.size());

                for (std::size_t matchIndex = 0; matchIndex < matches.size(); ++matchIndex) {
                    auto* node = matches[matchIndex].node;
                    if (!node) {
                        continue;
                    }

                    auto* matchNode = node->IsNode();
                    const auto matchChildCount = matchNode ? matchNode->children.size() : 0;
                    const auto stats = summarizeWeaponAnimNodeSubtree(node);
                    const auto matchChildNames = weaponAnimNodeImmediateChildNames(node);
                    auto* parent = node->parent;

                    ROCK_LOG_INFO(Weapon,
                        "WeaponAnimMap node root='{}' target='{}' match={} path='{}' depth={} addr=0x{:X} name='{}' parent='{}'/0x{:X} children={} childNames='{}' flags=0x{:X} appCulled={} visible={} subtreeNodes={} niNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} subtreeMaxDepth={}",
                        dumpRoot.label,
                        targetName,
                        matchIndex,
                        matches[matchIndex].path,
                        matches[matchIndex].depth,
                        reinterpret_cast<std::uintptr_t>(node),
                        safeNodeName(node),
                        safeNodeName(parent),
                        reinterpret_cast<std::uintptr_t>(parent),
                        static_cast<std::size_t>(matchChildCount),
                        matchChildNames,
                        static_cast<std::uint32_t>(node->flags.flags),
                        node->GetAppCulled() ? "yes" : "no",
                        weaponVisualNodeVisible(node) ? "yes" : "no",
                        stats.nodeCount,
                        stats.niNodeCount,
                        stats.triShapeCount,
                        stats.visibleTriShapeCount,
                        stats.hiddenFlagCount,
                        stats.appCulledCount,
                        stats.maxDepth);

                    ROCK_LOG_INFO(Weapon,
                        "WeaponAnimMap transform root='{}' target='{}' match={} localT=({:.3f},{:.3f},{:.3f}) localScale={:.3f} worldT=({:.3f},{:.3f},{:.3f}) worldScale={:.3f}",
                        dumpRoot.label,
                        targetName,
                        matchIndex,
                        node->local.translate.x,
                        node->local.translate.y,
                        node->local.translate.z,
                        node->local.scale,
                        node->world.translate.x,
                        node->world.translate.y,
                        node->world.translate.z,
                        node->world.scale);
                }
            }
        }

        inline void logWeaponAnimFlattenedMapRoot(const WeaponAnimFlattenedRoot& flatRoot)
        {
            auto* tree = flatRoot.tree;
            auto* treeNode = static_cast<RE::NiAVObject*>(tree);
            auto* niNode = treeNode ? treeNode->IsNode() : nullptr;
            const bool valid = weaponAnimFlattenedTreeValid(tree);
            const auto childCount = niNode ? niNode->children.size() : 0;
            const auto childNames = weaponAnimNodeImmediateChildNames(treeNode);

            ROCK_LOG_INFO(Weapon,
                "WeaponAnimMap root='{}' kind=flattened tree=0x{:X} valid={} name='{}' numTransforms={} transforms=0x{:X} bonePositions=0x{:X} children={} childNames='{}'",
                flatRoot.label,
                reinterpret_cast<std::uintptr_t>(tree),
                valid ? "yes" : "no",
                safeNodeName(treeNode),
                tree ? tree->numTransforms : 0,
                reinterpret_cast<std::uintptr_t>(tree ? tree->transforms : nullptr),
                reinterpret_cast<std::uintptr_t>(tree ? tree->bonePositions : nullptr),
                static_cast<std::size_t>(childCount),
                childNames);

            if (!valid) {
                return;
            }

            for (const char* targetName : WEAPON_ANIM_NODE_DUMP_TARGETS) {
                auto matches = collectWeaponAnimFlattenedBoneMatches(tree, targetName);
                ROCK_LOG_INFO(Weapon,
                    "WeaponAnimMap root='{}' kind=flattened target='{}' matches={}",
                    flatRoot.label,
                    targetName,
                    matches.size());

                for (std::size_t matchIndex = 0; matchIndex < matches.size(); ++matchIndex) {
                    const auto& match = matches[matchIndex];
                    if (match.index < 0 || match.index >= tree->numTransforms) {
                        continue;
                    }

                    const auto& transform = tree->transforms[match.index];
                    auto* refNode = match.refNode;
                    auto* refParent = refNode ? refNode->parent : nullptr;

                    ROCK_LOG_INFO(Weapon,
                        "WeaponAnimMap flatBone root='{}' target='{}' match={} index={} name='{}' parentIndex={} parentName='{}' childPos={} refNode='{}'/0x{:X} refParent='{}'/0x{:X} refFlags=0x{:X} refAppCulled={} refVisible={} localT=({:.3f},{:.3f},{:.3f}) localScale={:.3f} worldT=({:.3f},{:.3f},{:.3f}) worldScale={:.3f} unk8c=0x{:X} unk98=0x{:X}",
                        flatRoot.label,
                        targetName,
                        matchIndex,
                        match.index,
                        match.name,
                        match.parentIndex,
                        weaponAnimFlattenedParentName(tree, match.parentIndex),
                        match.childPosition,
                        safeNodeName(refNode),
                        reinterpret_cast<std::uintptr_t>(refNode),
                        safeNodeName(refParent),
                        reinterpret_cast<std::uintptr_t>(refParent),
                        static_cast<std::uint32_t>(refNode ? refNode->flags.flags : 0),
                        refNode && refNode->GetAppCulled() ? "yes" : "no",
                        refNode && weaponVisualNodeVisible(refNode) ? "yes" : "no",
                        transform.local.translate.x,
                        transform.local.translate.y,
                        transform.local.translate.z,
                        transform.local.scale,
                        transform.world.translate.x,
                        transform.world.translate.y,
                        transform.world.translate.z,
                        transform.world.scale,
                        transform.unk8c,
                        transform.unk98);
                }
            }
        }

        inline std::string generatedWeaponSemanticMaskNames(std::uint32_t mask)
        {
            std::string result;
            for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(WeaponPartKind::Count); ++i) {
                const auto bit = std::uint32_t{ 1 } << i;
                if ((mask & bit) == 0) {
                    continue;
                }
                if (!result.empty()) {
                    result += ",";
                }
                result += generatedWeaponPartKindName(static_cast<WeaponPartKind>(i));
            }
            return result.empty() ? std::string("none") : result;
        }

        inline float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        inline float bodyBasisDeterminant(const float* bodyFloats)
        {
            const float x0 = bodyFloats[0];
            const float x1 = bodyFloats[1];
            const float x2 = bodyFloats[2];
            const float y0 = bodyFloats[4];
            const float y1 = bodyFloats[5];
            const float y2 = bodyFloats[6];
            const float z0 = bodyFloats[8];
            const float z1 = bodyFloats[9];
            const float z2 = bodyFloats[10];

            return x0 * (y1 * z2 - y2 * z1) - y0 * (x1 * z2 - x2 * z1) + z0 * (x1 * y2 - x2 * y1);
        }

        inline RE::NiTransform makeIdentityTransform()
        {
            RE::NiTransform result{};
            result.rotate.entry[0][0] = 1.0f;
            result.rotate.entry[1][1] = 1.0f;
            result.rotate.entry[2][2] = 1.0f;
            result.scale = 1.0f;
            return result;
        }

        inline QuantizedPointKey quantizePoint(const RE::NiPoint3& point, float grid)
        {
            const float safeGrid = (std::max)(grid, 0.0001f);
            return QuantizedPointKey{ static_cast<std::int64_t>(std::llround(point.x / safeGrid)), static_cast<std::int64_t>(std::llround(point.y / safeGrid)),
                static_cast<std::int64_t>(std::llround(point.z / safeGrid)) };
        }

        inline void mixWeaponVisualKey(std::uint64_t& key, std::uint64_t value)
        {
            weapon_visual_composition_policy::mixValue(key, value);
        }

        inline void mixWeaponVisualString(std::uint64_t& key, const char* value)
        {
            if (!value) {
                return;
            }
            weapon_visual_composition_policy::mixString(key, value);
        }

        inline std::vector<RE::NiPoint3> dedupePointCloud(const std::vector<RE::NiPoint3>& points, float grid)
        {
            std::vector<RE::NiPoint3> unique;
            unique.reserve(points.size());
            std::unordered_set<QuantizedPointKey, QuantizedPointKeyHash> seen;
            seen.reserve(points.size());

            for (const auto& point : points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                    continue;
                }

                const auto key = quantizePoint(point, grid);
                if (seen.insert(key).second) {
                    unique.push_back(point);
                }
            }

            return unique;
        }

        inline float pointCloudDiagonalSquared(const std::vector<RE::NiPoint3>& points)
        {
            if (points.empty()) {
                return 0.0f;
            }

            RE::NiPoint3 minPoint = points.front();
            RE::NiPoint3 maxPoint = points.front();
            for (const auto& point : points) {
                minPoint = weapon_collision_geometry_math::pointMin(minPoint, point);
                maxPoint = weapon_collision_geometry_math::pointMax(maxPoint, point);
            }

            const float dx = maxPoint.x - minPoint.x;
            const float dy = maxPoint.y - minPoint.y;
            const float dz = maxPoint.z - minPoint.z;
            return dx * dx + dy * dy + dz * dz;
        }

        inline bool pointCloudCanBuildHull(const std::vector<RE::NiPoint3>& points, float sourceScale = 1.0f)
        {
            return points.size() >= 4 && weapon_collision_geometry_math::scaledHullDiagonalCanBuild(
                                             pointCloudDiagonalSquared(points), sourceScale, MIN_HULL_DIAGONAL_GAME_UNITS);
        }

        inline bool compressGeneratedChildClustersForBudget(
            std::vector<std::vector<RE::NiPoint3>>& clusters,
            const std::vector<RE::NiPoint3>& sourcePoints,
            std::string_view sourceName,
            int depth)
        {
            /*
             * Dense modded meshes should not explode into unbounded compound
             * children. The source itself is still kept and generated; only its
             * internal child representation is reduced geometrically when it
             * exceeds the per-source budget.
             */
            if (clusters.size() <= MAX_GENERATED_CHILD_CONVEXES_PER_SOURCE || !pointCloudCanBuildHull(sourcePoints)) {
                return false;
            }

            constexpr auto targetPoints = WEAPON_COLLISION_SUPPORT_FIT_TARGET_POINTS;
            const auto fit = weapon_collision_geometry_math::fitConvexSupportPointCloud(
                sourcePoints,
                targetPoints,
                MAX_CONVEX_HULL_POINTS,
                WEAPON_COLLISION_SUPPORT_FIT_MAX_ERROR_GAME_UNITS);
            if (fit.accepted && pointCloudCanBuildHull(fit.points)) {
                const std::size_t previousCount = clusters.size();
                clusters.clear();
                clusters.push_back(fit.points);
                ROCK_LOG_DEBUG(Weapon,
                    "{}generated source '{}' compressed dense child hulls with support fit children={}->{} rawPoints={} fittedPoints={} maxError={:.3f}",
                    std::string(depth * 2, ' '),
                    sourceName,
                    previousCount,
                    clusters.size(),
                    fit.inputPointCount,
                    fit.selectedPointCount,
                    fit.maxSupportError);
                return true;
            }

            auto fallback = weapon_collision_geometry_math::limitPointCloud(sourcePoints, MAX_CONVEX_HULL_POINTS);
            if (pointCloudCanBuildHull(fallback)) {
                const std::size_t previousCount = clusters.size();
                clusters.clear();
                clusters.push_back(std::move(fallback));
                ROCK_LOG_WARN(Weapon,
                    "{}generated source '{}' exceeded child hull budget and used limited single-hull fallback children={}->{} rawPoints={} targetPoints={} maxError={:.3f}",
                    std::string(depth * 2, ' '),
                    sourceName,
                    previousCount,
                    clusters.size(),
                    sourcePoints.size(),
                    targetPoints,
                    fit.maxSupportError);
                return true;
            }

            ROCK_LOG_WARN(Weapon,
                "{}generated source '{}' exceeded child hull budget but could not build compressed fallback children={} rawPoints={}",
                std::string(depth * 2, ' '),
                sourceName,
                clusters.size(),
                sourcePoints.size());
            return false;
        }

        inline PointCloudBounds pointCloudBounds(const std::vector<RE::NiPoint3>& points)
        {
            PointCloudBounds bounds{};
            if (points.empty()) {
                return bounds;
            }

            bounds.min = points.front();
            bounds.max = points.front();
            for (const auto& point : points) {
                bounds.min = weapon_collision_geometry_math::pointMin(bounds.min, point);
                bounds.max = weapon_collision_geometry_math::pointMax(bounds.max, point);
            }
            return bounds;
        }

        inline std::array<float, 3> pointToArray(const RE::NiPoint3& point)
        {
            return { point.x, point.y, point.z };
        }

        inline void mixFormPointer(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, reinterpret_cast<std::uintptr_t>(form));
            if (form) {
                weapon_visual_composition_policy::mixValue(key, form->formID);
            }
        }

        inline void mixFormStableContent(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, form ? form->formID : 0);
        }

        inline void mixFloatBits(std::uint64_t& key, float value)
        {
            weapon_visual_composition_policy::mixValue(key, std::bit_cast<std::uint32_t>(value));
        }

        inline void mixKeywordFormContent(std::uint64_t& key, const RE::BGSKeywordForm* keywords)
        {
            if (!keywords) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, keywords->GetNumKeywords());
            keywords->ForEachKeyword([&](RE::BGSKeyword* keyword) {
                mixFormStableContent(key, keyword);
                return RE::BSContainer::ForEachResult::kContinue;
            });
        }

        inline void mixBlockBashDataContent(std::uint64_t& key, const RE::BGSBlockBashData* blockBashData)
        {
            if (!blockBashData) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, 1u);
            mixFormStableContent(key, blockBashData->blockBashImpactDataSet);
            mixFormStableContent(key, blockBashData->altBlockMaterialType);
        }

        template <class Form>
        void mixFormPointerArray(std::uint64_t& key, const RE::BSTArray<Form*>* forms)
        {
            if (!forms) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, forms->size());
            for (std::uint32_t index = 0; index < forms->size(); ++index) {
                mixFormStableContent(key, (*forms)[index]);
            }
        }

        inline void mixObjectInstanceExtraContent(std::uint64_t& key, const RE::BGSObjectInstanceExtra* extra)
        {
            if (!extra || !extra->values) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            const auto indexData = extra->GetIndexData();
            weapon_visual_composition_policy::mixValue(key, 1u);
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);
            }
        }

        struct ObjectInstanceExtraWitness
        {
            std::uint64_t signature{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t activeCount{ 0 };
            std::uint32_t disabledCount{ 0 };
        };

        inline ObjectInstanceExtraWitness makeObjectInstanceExtraWitness(const RE::BGSObjectInstanceExtra* extra)
        {
            ObjectInstanceExtraWitness witness{};
            if (!extra || !extra->values) {
                return witness;
            }

            const auto indexData = extra->GetIndexData();
            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKObjectInstanceExtraIndexWitnessV1");
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            witness.count = static_cast<std::uint32_t>(
                (std::min)(indexData.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);
                if (modIndex.disabled) {
                    ++witness.disabledCount;
                } else {
                    ++witness.activeCount;
                }
            }
            witness.signature = key;
            return witness;
        }

        inline const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
            const RE::PlayerCharacter* player,
            const RE::TESForm* weaponForm,
            const RE::TBO_InstanceData* instanceData)
        {
            if (!player || !weaponForm) {
                return nullptr;
            }

            auto scanBiped = [&](const RE::BipedAnim* biped) -> const RE::BGSObjectInstanceExtra* {
                if (!biped) {
                    return nullptr;
                }
                for (std::uint32_t slotIndex = 0;
                     slotIndex < static_cast<std::uint32_t>(std::to_underlying(RE::BIPED_OBJECT::kTotal));
                     ++slotIndex) {
                    const auto& slot = biped->object[slotIndex];
                    if (slot.parent.object != weaponForm) {
                        continue;
                    }
                    if (instanceData && slot.parent.instanceData && slot.parent.instanceData.get() != instanceData) {
                        continue;
                    }
                    if (slot.modExtra) {
                        return slot.modExtra;
                    }
                }
                return nullptr;
            };

            if (const auto* firstPersonExtra = scanBiped(player->firstPersonBipedAnim.get())) {
                return firstPersonExtra;
            }
            return scanBiped(player->biped.get());
        }

        [[nodiscard]] inline bool startsWithAsciiInsensitive(std::string_view value, std::string_view prefix)
        {
            if (value.size() < prefix.size()) {
                return false;
            }
            for (std::size_t i = 0; i < prefix.size(); ++i) {
                const auto lowerAscii = [](const char c) {
                    return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
                };
                if (lowerAscii(value[i]) != lowerAscii(prefix[i])) {
                    return false;
                }
            }
            return true;
        }

        inline RE::NiPointer<RE::NiNode> loadOmodModelTemplate(
            const std::string& modelPath,
            const std::uint8_t modelDemandFlags)
        {
            if (modelPath.empty()) {
                return nullptr;
            }

            std::string resourcePath;
            if (startsWithAsciiInsensitive(modelPath, "Data\\") || startsWithAsciiInsensitive(modelPath, "Data/")) {
                resourcePath = modelPath;
            } else if (startsWithAsciiInsensitive(modelPath, "Meshes\\") || startsWithAsciiInsensitive(modelPath, "Meshes/")) {
                resourcePath = "Data/" + modelPath;
            } else {
                resourcePath = "Data/Meshes/" + modelPath;
            }

            std::uint64_t loadFlags[2]{ 0, modelDemandFlags };
            std::uint64_t loadedRoot = 0;
            const int result = f4vr::loadNif(
                reinterpret_cast<std::uint64_t>(resourcePath.c_str()),
                reinterpret_cast<std::uint64_t>(&loadedRoot),
                reinterpret_cast<std::uint64_t>(&loadFlags));
            if (result != 0 || loadedRoot == 0) {
                return nullptr;
            }

            RE::NiPointer<RE::NiNode> root;
            root.reset(reinterpret_cast<RE::NiNode*>(loadedRoot));
            return root;
        }

        /*
         * BSModelDB's ordinary OMOD demand uses flag 0x2D and may return only
         * the currently selected controller branch. Loading through the same
         * native entry with 0xED preserves the complete model hierarchy. This
         * lets scope metadata inspection find structural markers outside the
         * active branch without attaching model geometry to the weapon.
         */
        inline RE::NiPointer<RE::NiNode> loadCompleteOmodModelTemplate(const std::string& modelPath)
        {
            return loadOmodModelTemplate(modelPath, 0xED);
        }

        /*
         * Fallout4VR.exe 1.2.72 uses BSModelDB flag 0x20 in the geometry-query
         * path at 0x1402824B0. Unlike an ordinary attachment demand, this path
         * does not run the 0x08 model postprocessor which can consume display
         * geometry owned by a bhkNPCollisionObject. Collision-sound material
         * inspection reads this template without attaching it to the weapon.
         */
        inline RE::NiPointer<RE::NiNode> loadGeometryInspectionOmodModelTemplate(const std::string& modelPath)
        {
            return loadOmodModelTemplate(modelPath, 0x20);
        }

        struct CollisionSoundMaterialEvidence
        {
            std::uint32_t materialId{ 0 };
            const RE::BGSMaterialType* material{ nullptr };
            bool blocksFallback{ false };

            [[nodiscard]] bool valid() const noexcept
            {
                return materialId != 0 && material != nullptr;
            }
        };

        struct CollisionSoundMaterialDiagnostics
        {
            std::size_t inspectedShapes{ 0 };
            std::size_t simpleShapes{ 0 };
            std::size_t compositeShapes{ 0 };
            std::size_t enumeratedShapeKeys{ 0 };
            std::size_t materialQueries{ 0 };
            std::size_t zeroMaterialShapeKeys{ 0 };
            std::size_t unregisteredMaterialShapeKeys{ 0 };
            std::size_t ambiguousShapes{ 0 };
            std::size_t incompleteShapes{ 0 };
            std::size_t truncatedShapes{ 0 };
            std::size_t stalledShapes{ 0 };
            std::size_t invalidEnumerationShapes{ 0 };
            std::unordered_map<std::uint32_t, std::size_t> materialKeyCounts;
        };

        using CollisionSoundMaterialCache =
            std::unordered_map<RE::hknpShape*, CollisionSoundMaterialEvidence>;

        [[nodiscard]] inline const RE::BGSMaterialType* findRegisteredCollisionMaterial(
            const std::uint32_t materialId)
        {
            if (materialId == 0) {
                return nullptr;
            }

            auto* dataHandler = RE::TESDataHandler::GetSingleton();
            if (!dataHandler) {
                return nullptr;
            }

            for (const auto* material :
                dataHandler->GetFormArray<RE::BGSMaterialType>()) {
                if (material && material->materialID == materialId) {
                    return material;
                }
            }
            return nullptr;
        }

        [[nodiscard]] inline std::uint32_t nativeCollisionSoundMaterialForShape(
            RE::hknpShape* shape,
            const std::uint32_t shapeKey)
        {
            if (!shape) {
                return 0;
            }

            using GetMaterialForShape =
                std::uint32_t (*)(RE::hknpShape*, std::uint32_t);
            static REL::Relocation<GetMaterialForShape> getMaterialForShape{
                REL::ID(1349330)
            };
            return getMaterialForShape(shape, shapeKey);
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        resolveCollisionSoundMaterial(
            RE::hknpShape* shape,
            CollisionSoundMaterialDiagnostics& diagnostics)
        {
            CollisionSoundMaterialEvidence evidence{};
            if (!shape) {
                return evidence;
            }

            ++diagnostics.inspectedShapes;

            std::unordered_map<std::uint32_t, std::size_t> materialKeyCounts;
            materialKeyCounts.reserve(4);
            std::size_t zeroMaterialShapeKeys = 0;
            const auto recordShapeKey = [&](const std::uint32_t shapeKey) {
                ++diagnostics.materialQueries;
                const auto materialId =
                    nativeCollisionSoundMaterialForShape(shape, shapeKey);
                if (materialId == 0) {
                    ++zeroMaterialShapeKeys;
                    return;
                }
                ++materialKeyCounts[materialId];
            };

            bool scanComplete = true;
            const bool isComposite = shape->flags.all(
                RE::hknpShape::FlagsEnum::kIsCompositeShape);
            if (!isComposite) {
                ++diagnostics.simpleShapes;
                recordShapeKey(INVALID_COLLISION_SOUND_SHAPE_KEY);
            } else {
                ++diagnostics.compositeShapes;
                scanComplete = false;
                RE::hknpShape::GetShapeKeysConfig keyConfig{};
                std::array<std::uint32_t, COLLISION_SOUND_SHAPE_KEY_BATCH>
                    shapeKeys{};
                std::uint32_t previousShapeKey =
                    INVALID_COLLISION_SOUND_SHAPE_KEY;
                std::size_t enumeratedShapeKeys = 0;
                bool enumerationInvalid = false;

                while (enumeratedShapeKeys <
                       MAX_COLLISION_SOUND_SHAPE_KEYS) {
                    const auto remaining =
                        MAX_COLLISION_SOUND_SHAPE_KEYS -
                        enumeratedShapeKeys;
                    const auto capacity = static_cast<std::int32_t>(
                        std::min(remaining, shapeKeys.size()));
                    const auto shapeKeyCount = shape->GetShapeKeys(
                        shapeKeys.data(),
                        capacity,
                        previousShapeKey,
                        keyConfig);
                    if (shapeKeyCount < 0 || shapeKeyCount > capacity) {
                        ++diagnostics.invalidEnumerationShapes;
                        enumerationInvalid = true;
                        break;
                    }
                    if (shapeKeyCount == 0) {
                        scanComplete = true;
                        break;
                    }

                    const auto lastShapeKey =
                        shapeKeys[static_cast<std::size_t>(shapeKeyCount - 1)];
                    if (lastShapeKey == previousShapeKey) {
                        ++diagnostics.stalledShapes;
                        enumerationInvalid = true;
                        break;
                    }

                    for (std::int32_t index = 0;
                         index < shapeKeyCount;
                         ++index) {
                        recordShapeKey(
                            shapeKeys[static_cast<std::size_t>(index)]);
                    }
                    enumeratedShapeKeys +=
                        static_cast<std::size_t>(shapeKeyCount);
                    previousShapeKey = lastShapeKey;

                    if (shapeKeyCount < capacity) {
                        scanComplete = true;
                        break;
                    }
                }

                diagnostics.enumeratedShapeKeys += enumeratedShapeKeys;
                if (!scanComplete && !enumerationInvalid &&
                    enumeratedShapeKeys ==
                        MAX_COLLISION_SOUND_SHAPE_KEYS) {
                    std::uint32_t nextShapeKey = 0;
                    const auto remainingShapeKeyCount = shape->GetShapeKeys(
                        &nextShapeKey,
                        1,
                        previousShapeKey,
                        keyConfig);
                    if (remainingShapeKeyCount == 0) {
                        scanComplete = true;
                    } else if (remainingShapeKeyCount == 1 &&
                               nextShapeKey != previousShapeKey) {
                        ++diagnostics.truncatedShapes;
                    } else if (remainingShapeKeyCount == 1) {
                        ++diagnostics.stalledShapes;
                    } else {
                        ++diagnostics.invalidEnumerationShapes;
                    }
                }

                if (enumeratedShapeKeys == 0 && scanComplete) {
                    recordShapeKey(INVALID_COLLISION_SOUND_SHAPE_KEY);
                }
            }

            diagnostics.zeroMaterialShapeKeys += zeroMaterialShapeKeys;
            for (const auto& [materialId, shapeKeyCount] :
                 materialKeyCounts) {
                diagnostics.materialKeyCounts[materialId] += shapeKeyCount;
            }

            if (!scanComplete) {
                ++diagnostics.incompleteShapes;
                evidence.blocksFallback = true;
                return evidence;
            }
            if (zeroMaterialShapeKeys != 0) {
                if (!materialKeyCounts.empty()) {
                    ++diagnostics.ambiguousShapes;
                    evidence.blocksFallback = true;
                }
                return evidence;
            }
            if (materialKeyCounts.size() != 1) {
                if (materialKeyCounts.size() > 1) {
                    ++diagnostics.ambiguousShapes;
                    evidence.blocksFallback = true;
                }
                return evidence;
            }

            const auto [materialId, shapeKeyCount] =
                *materialKeyCounts.begin();
            const auto* material =
                findRegisteredCollisionMaterial(materialId);
            if (!material) {
                diagnostics.unregisteredMaterialShapeKeys += shapeKeyCount;
                evidence.blocksFallback = true;
                return evidence;
            }

            evidence.materialId = materialId;
            evidence.material = material;
            return evidence;
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        collisionSoundMaterialFromNode(
            RE::NiAVObject* node,
            CollisionSoundMaterialCache& cache,
            CollisionSoundMaterialDiagnostics& diagnostics)
        {
            CollisionSoundMaterialEvidence evidence{};
            if (!node) {
                return evidence;
            }

            auto* collisionObject = node->collisionObject.get();
            auto* nativeCollision = collisionObject ?
                collisionObject->IsbhkNPCollisionObject() :
                nullptr;
            if (!nativeCollision || !nativeCollision->spSystem) {
                return evidence;
            }

            auto* shape = nativeCollision->GetShape();
            if (!shape) {
                return evidence;
            }

            /*
             * bhkNPCollisionObject::GetShape returns the serialized body-cinfo
             * shape when a model template is not instantiated in a world.
             * Fallout4VR's own GetMaterialForShape handles simple userData,
             * compound shape tags, nested leaves, and hknpBSMaterialProperties
             * without adding a diagnostic body to the live world.
             */
            const auto cached = cache.find(shape);
            if (cached != cache.end()) {
                return cached->second;
            }

            evidence = resolveCollisionSoundMaterial(shape, diagnostics);
            cache.emplace(shape, evidence);
            return evidence;
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        nearestCollisionSoundMaterial(
            RE::NiAVObject* sourceNode,
            const RE::NiAVObject* weaponBoundary,
            CollisionSoundMaterialCache& cache,
            CollisionSoundMaterialDiagnostics& diagnostics)
        {
            constexpr int kMaximumAncestorSteps = 32;
            auto* node = sourceNode;
            for (int step = 0; node && step < kMaximumAncestorSteps;
                 ++step, node = node->parent) {
                auto evidence = collisionSoundMaterialFromNode(
                    node,
                    cache,
                    diagnostics);
                if (evidence.valid() || evidence.blocksFallback) {
                    return evidence;
                }
                if (node == weaponBoundary) {
                    break;
                }
            }
            return {};
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        firstCollisionSoundMaterialRecursive(
            RE::NiAVObject* node,
            std::unordered_set<std::uintptr_t>& visited,
            std::size_t& visitedCount,
            CollisionSoundMaterialCache& cache,
            CollisionSoundMaterialDiagnostics& diagnostics,
            const int depth = 0)
        {
            constexpr int kMaximumDepth = 24;
            constexpr std::size_t kMaximumVisitedNodes = 1024;
            if (!node || depth > kMaximumDepth ||
                visitedCount >= kMaximumVisitedNodes ||
                !visited.insert(reinterpret_cast<std::uintptr_t>(node)).second) {
                return {};
            }

            ++visitedCount;
            auto evidence = collisionSoundMaterialFromNode(
                node,
                cache,
                diagnostics);
            if (evidence.valid() || evidence.blocksFallback) {
                return evidence;
            }

            auto* treeNode = node->IsNode();
            if (!treeNode) {
                return {};
            }
            const auto& children = treeNode->GetRuntimeData().children;
            for (std::uint16_t index = 0; index < children.capacity(); ++index) {
                evidence = firstCollisionSoundMaterialRecursive(
                    children[index].get(),
                    visited,
                    visitedCount,
                    cache,
                    diagnostics,
                    depth + 1);
                if (evidence.valid() || evidence.blocksFallback) {
                    return evidence;
                }
            }
            return {};
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        firstCollisionSoundMaterial(
            RE::NiAVObject* root,
            CollisionSoundMaterialCache& cache,
            CollisionSoundMaterialDiagnostics& diagnostics)
        {
            std::unordered_set<std::uintptr_t> visited;
            visited.reserve(128);
            std::size_t visitedCount = 0;
            return firstCollisionSoundMaterialRecursive(
                root,
                visited,
                visitedCount,
                cache,
                diagnostics);
        }

        [[nodiscard]] inline CollisionSoundMaterialEvidence
        equippedWeaponWorldModelCollisionSoundMaterial(
            std::string& outModelPath,
            CollisionSoundMaterialDiagnostics& diagnostics)
        {
            outModelPath.clear();
            auto* equippedItem = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equippedItem ? equippedItem->item.object : nullptr;
            auto* weapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon) {
                return {};
            }

            const auto* model =
                static_cast<const RE::BGSModelMaterialSwap*>(weapon);
            const char* modelPath = model->GetModel();
            if (!modelPath || modelPath[0] == '\0') {
                return {};
            }

            outModelPath = modelPath;
            auto modelRoot =
                loadGeometryInspectionOmodModelTemplate(outModelPath);
            CollisionSoundMaterialCache modelMaterialCache;
            modelMaterialCache.reserve(16);
            return firstCollisionSoundMaterial(
                modelRoot.get(),
                modelMaterialCache,
                diagnostics);
        }

        [[nodiscard]] inline std::string collisionSoundMaterialHistogram(
            const CollisionSoundMaterialDiagnostics& diagnostics)
        {
            std::vector<std::pair<std::uint32_t, std::size_t>> rows(
                diagnostics.materialKeyCounts.begin(),
                diagnostics.materialKeyCounts.end());
            std::sort(
                rows.begin(),
                rows.end(),
                [](const auto& lhs, const auto& rhs) {
                    return lhs.first < rhs.first;
                });

            std::string histogram;
            const auto rowCount = std::min(
                rows.size(),
                MAX_COLLISION_SOUND_HISTOGRAM_ROWS);
            for (std::size_t index = 0; index < rowCount; ++index) {
                std::array<char, 48> row{};
                const auto written = std::snprintf(
                    row.data(),
                    row.size(),
                    "%s0x%08X:%zu",
                    histogram.empty() ? "" : ",",
                    rows[index].first,
                    rows[index].second);
                if (written > 0) {
                    histogram.append(
                        row.data(),
                        std::min(
                            static_cast<std::size_t>(written),
                            row.size() - 1));
                }
            }
            if (rows.size() > rowCount) {
                histogram.append(",...");
            }
            if (histogram.empty()) {
                histogram = "none";
            }
            return histogram;
        }

        template <class SourceRange>
        void assignCollisionSoundMaterials(
            RE::NiAVObject* assembledWeaponRoot,
            SourceRange& sources)
        {
            CollisionSoundMaterialCache materialCache;
            materialCache.reserve(64);
            CollisionSoundMaterialDiagnostics diagnostics{};
            diagnostics.materialKeyCounts.reserve(8);
            std::unordered_set<std::uintptr_t> blockedFallbackSources;
            blockedFallbackSources.reserve(16);

            std::size_t localSourceCount = 0;
            for (auto& source : sources) {
                const auto localEvidence =
                    nearestCollisionSoundMaterial(
                        source.sourceRoot,
                        assembledWeaponRoot,
                        materialCache,
                        diagnostics);
                if (localEvidence.valid()) {
                    source.collisionSoundMaterialId =
                        localEvidence.materialId;
                    ++localSourceCount;
                } else if (localEvidence.blocksFallback) {
                    blockedFallbackSources.insert(
                        reinterpret_cast<std::uintptr_t>(&source));
                }
            }

            const bool needsFallback = std::any_of(
                sources.begin(),
                sources.end(),
                [](const auto& source) {
                    return source.collisionSoundMaterialId == 0;
                });

            CollisionSoundMaterialEvidence fallback{};
            std::string fallbackSource = "none";
            std::string worldModelPath;
            if (needsFallback) {
                fallback = equippedWeaponWorldModelCollisionSoundMaterial(
                    worldModelPath,
                    diagnostics);
                if (fallback.valid()) {
                    fallbackSource = "world-model";
                } else if (!fallback.blocksFallback) {
                    fallback = firstCollisionSoundMaterial(
                        assembledWeaponRoot,
                        materialCache,
                        diagnostics);
                    if (fallback.valid()) {
                        fallbackSource = "assembled-root";
                    }
                }
            }

            std::size_t fallbackSourceCount = 0;
            std::size_t unresolvedSourceCount = 0;
            std::unordered_map<std::uint32_t, std::size_t> materialCounts;
            for (auto& source : sources) {
                if (source.collisionSoundMaterialId == 0 &&
                    fallback.valid() &&
                    !blockedFallbackSources.contains(
                        reinterpret_cast<std::uintptr_t>(&source))) {
                    source.collisionSoundMaterialId = fallback.materialId;
                    ++fallbackSourceCount;
                }
                if (source.collisionSoundMaterialId == 0) {
                    ++unresolvedSourceCount;
                    continue;
                }
                ++materialCounts[source.collisionSoundMaterialId];
            }

            for (const auto& [materialId, sourceCount] : materialCounts) {
                const auto* material =
                    findRegisteredCollisionMaterial(materialId);
                ROCK_LOG_INFO(
                    Weapon,
                    "Equipped weapon collision audio material: id=0x{:08X} form={:08X} name='{}' sources={} localSources={} fallbackSources={} fallback={} model='{}'",
                    materialId,
                    material ? material->formID : 0,
                    material && material->materialName.c_str() ?
                        material->materialName.c_str() :
                        "(unnamed)",
                    sourceCount,
                    localSourceCount,
                    fallbackSourceCount,
                    fallbackSource,
                    worldModelPath);
            }

            const auto nativeMaterialHistogram =
                collisionSoundMaterialHistogram(diagnostics);
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon collision audio native scan: shapes={} simple={} composite={} keys={} queries={} zeroKeys={} unregisteredKeys={} ambiguousShapes={} incompleteShapes={} truncatedShapes={} stalledShapes={} invalidEnumerations={} blockedFallbackSources={} candidates='{}' model='{}'",
                diagnostics.inspectedShapes,
                diagnostics.simpleShapes,
                diagnostics.compositeShapes,
                diagnostics.enumeratedShapeKeys,
                diagnostics.materialQueries,
                diagnostics.zeroMaterialShapeKeys,
                diagnostics.unregisteredMaterialShapeKeys,
                diagnostics.ambiguousShapes,
                diagnostics.incompleteShapes,
                diagnostics.truncatedShapes,
                diagnostics.stalledShapes,
                diagnostics.invalidEnumerationShapes,
                blockedFallbackSources.size(),
                nativeMaterialHistogram,
                worldModelPath);

            if (unresolvedSourceCount != 0) {
                ROCK_LOG_SAMPLE_WARN(
                    Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Equipped weapon collision audio material unresolved: sources={}/{} policy=fail-closed shapes={} composite={} keys={} zeroKeys={} ambiguousShapes={} incompleteShapes={} candidates='{}' model='{}'",
                    unresolvedSourceCount,
                    sources.size(),
                    diagnostics.inspectedShapes,
                    diagnostics.compositeShapes,
                    diagnostics.enumeratedShapeKeys,
                    diagnostics.zeroMaterialShapeKeys,
                    diagnostics.ambiguousShapes,
                    diagnostics.incompleteShapes,
                    nativeMaterialHistogram,
                    worldModelPath);
            }
        }

        inline void collectManualScopeStructuralMarkers(
            RE::NiAVObject* node,
            manual_scope_target_policy::StructuralMarkerEvidence& evidence,
            std::size_t& visited,
            const int depth = 0)
        {
            if (!node || depth > 16 || visited >= 512 ||
                manual_scope_target_policy::hasMagnifiedScopeStructure(evidence)) {
                return;
            }

            ++visited;
            const char* name = node->name.c_str();
            manual_scope_target_policy::observeStructuralNodeName(evidence, name ? name : "");
            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.capacity(); ++i) {
                collectManualScopeStructuralMarkers(children[i].get(), evidence, visited, depth + 1);
            }
        }

        struct EquippedManualScopeTarget
        {
            std::uintptr_t weaponIdentity{ 0 };
            std::uintptr_t instanceIdentity{ 0 };
            bool scopeEligible{ false };
            bool directTransitionRequired{ false };
            bool overlayValid{ false };
            std::uint32_t overlayIndex{ 0 };
        };

        inline std::unordered_map<std::uint32_t, std::uint32_t> readEquippedOmodsByAttachPointFormId(
            WeaponCollision::WeaponCompositionSnapshot* outComposition = nullptr)
        {
            std::unordered_map<std::uint32_t, std::uint32_t> result;
            if (outComposition) {
                *outComposition = {};
            }
            auto* player = f4vr::getPlayer();
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            auto* instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            const RE::BGSObjectInstanceExtra* objectInstanceExtra =
                weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, instanceData) : nullptr;
            if (!objectInstanceExtra || !objectInstanceExtra->values) {
                return result;
            }

            const auto indexData = objectInstanceExtra->GetIndexData();
            result.reserve(indexData.size());
            std::uint32_t stableIndex = 0;
            for (const auto& modIndex : indexData) {
                auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                const RE::BGSKeyword* attachPointKeyword =
                    omod ? RE::BGSKeyword::GetTypedKeywordByIndex(
                               RE::KeywordType::kAttachPoint,
                               omod->attachPoint.keywordIndex) :
                           nullptr;
                if (!modIndex.disabled && attachPointKeyword && omod) {
                    result.emplace(attachPointKeyword->formID, omod->formID);
                }
                if (outComposition &&
                    outComposition->entryCount <
                        outComposition->entries.size()) {
                    auto& entry = outComposition->entries[
                        outComposition->entryCount++];
                    entry.omodFormId = omod ? omod->formID : modIndex.objectID;
                    entry.attachPointFormId =
                        attachPointKeyword ? attachPointKeyword->formID : 0;
                    entry.stableIndex = stableIndex;
                    entry.flags = modIndex.disabled ? (1u << 1) : (1u << 0);
                    if (attachPointKeyword) {
                        entry.flags |= 1u << 2;
                    }
                }
                ++stableIndex;
            }
            return result;
        }

        [[nodiscard]] inline bool attachmentModHasScopeFlagProperty(std::uint32_t omodFormId)
        {
            constexpr std::uint8_t kBgsModPropertyBlockId = 1;
            constexpr std::uint32_t kHasScopeTarget = 48;
            using PropertyMod = RE::BGSMod::Property::Mod;

            auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(omodFormId);
            if (!omod) {
                return false;
            }
            for (const auto& property : omod->GetBuffer<PropertyMod>(kBgsModPropertyBlockId)) {
                // Declaration evidence for physical part classification only.
                // Activation must use the final equipped instance's flags.
                if (property.target == kHasScopeTarget) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] inline EquippedManualScopeTarget resolveEquippedManualScopeTarget(RE::NiAVObject* assembledWeaponRoot)
        {
            EquippedManualScopeTarget target{};
            auto* player = f4vr::getPlayer();
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            const RE::BGSObjectInstanceExtra* objectInstanceExtra =
                weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equippedInstanceData) : nullptr;
            auto* weapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon) {
                return target;
            }
            target.weaponIdentity = reinterpret_cast<std::uintptr_t>(weapon);
            target.instanceIdentity = reinterpret_cast<std::uintptr_t>(equippedInstanceData);
            auto* instanceData = weapon && equippedInstanceData ?
                static_cast<RE::TESObjectWEAP::InstanceData*>(equippedInstanceData) :
                nullptr;
            RE::BGSZoomData* zoomData = instanceData ? instanceData->zoomData : nullptr;
            if (!zoomData && weapon) {
                zoomData = weapon->weaponData.zoomData;
            }
            if (zoomData) {
                target.overlayIndex = zoomData->zoomData.overlay;
            }
            // An instance can explicitly clear a base weapon's HasScope bit.
            // Neither a base flag nor an OMOD property declaration may undo it.
            const bool nativeScopeMetadataAuthored = manual_scope_target_policy::nativeHasScope(
                instanceData != nullptr,
                instanceData && instanceData->flags.all(RE::WEAPON_FLAGS::kHasScope),
                weapon->weaponData.flags.all(RE::WEAPON_FLAGS::kHasScope));
            bool explicitScopeModelInstalled = false;
            manual_scope_target_policy::StructuralMarkerEvidence structuralEvidence{};
            std::size_t structuralVisited = 0;
            collectManualScopeStructuralMarkers(assembledWeaponRoot, structuralEvidence, structuralVisited);

            if (objectInstanceExtra && objectInstanceExtra->values) {
                for (const auto& modIndex : objectInstanceExtra->GetIndexData()) {
                    if (modIndex.disabled) {
                        continue;
                    }
                    auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                    if (!omod) {
                        continue;
                    }
                    explicitScopeModelInstalled = explicitScopeModelInstalled ||
                        manual_scope_target_policy::hasExplicitScopeIdentity(
                            omod->fullName.c_str() ? omod->fullName.c_str() : "",
                            omod->model.c_str() ? omod->model.c_str() : "");

                    if (!manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence)) {
                        const RE::BGSKeyword* attachPointKeyword =
                            RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, omod->attachPoint.keywordIndex);
                        const std::string_view recordName = omod->fullName.c_str() ? omod->fullName.c_str() : "";
                        const std::string modelPath = omod->model.c_str() ? omod->model.c_str() : "";
                        const bool opticalCandidate =
                            (attachPointKeyword && attachPointKeyword->formID == weapon_part_record_identity_policy::kAttachPointSight) ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "optic") ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "sight") ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "scope") ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "optic") ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "sight") ||
                            weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "scope");
                        if (opticalCandidate) {
                            auto templateRoot = loadCompleteOmodModelTemplate(modelPath);
                            std::size_t templateVisited = 0;
                            collectManualScopeStructuralMarkers(templateRoot.get(), structuralEvidence, templateVisited);
                        }
                    }
                }
            }
            target.scopeEligible = manual_scope_target_policy::isScopeEligible(
                nativeScopeMetadataAuthored, explicitScopeModelInstalled,
                manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence));
            target.overlayValid = target.scopeEligible;
            if (target.scopeEligible) {
                target.overlayIndex = manual_scope_target_policy::resolveOverlay(target.overlayIndex);
            }
            target.directTransitionRequired = manual_scope_target_policy::requiresDirectNativeTransition(
                nativeScopeMetadataAuthored,
                explicitScopeModelInstalled,
                manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence),
                target.overlayValid);
            if (target.scopeEligible) {
                ROCK_LOG_DEBUG(Weapon,
                    "Native scope target weapon={:08X} nativeHasScope={} named={} structure={} authoredOverlay={} selectedOverlay={} authoredZoom={} direct={}",
                    weapon->formID, nativeScopeMetadataAuthored, explicitScopeModelInstalled,
                    manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence),
                    zoomData ? zoomData->zoomData.overlay : 0, target.overlayIndex,
                    zoomData ? zoomData->zoomData.fovMult : 0.0f, target.directTransitionRequired);
            }
            return target;
        }

        inline const RE::TESObjectWEAP* asEquippedWeaponForm(const RE::TESForm* form)
        {
            if (!form || form->formType != RE::ENUM_FORM_ID::kWEAP) {
                return nullptr;
            }

            return form->As<RE::TESObjectWEAP>();
        }

        /*
         * Fallout4.esm's WeaponType* keyword records, verified directly against
         * the ESM (2026-07-03) rather than assumed from general modding
         * knowledge. Stored directly on every sampled vanilla WEAP record's own
         * keyword array - no OMOD/template indirection - so a direct
         * HasKeyword() check against the equipped form is sufficient. FormIDs are
         * master-relative (Fallout4.esm is always load-order index 0), matching
         * the existing hardcoded-keyword-lookup precedent in
         * hFRIK/src/FRIK.cpp (RE::TESForm::GetFormByID<RE::BGSKeyword>(0xB34A6)).
         */
        struct WeaponKeywordFormEntry
        {
            std::uint32_t formId;
            WeaponKeywordFlag flag;
        };

        inline constexpr WeaponKeywordFormEntry kWeaponKeywordForms[] = {
            { 0x0004A0A0, WeaponKeywordFlag::Pistol },
            { 0x0004A0A1, WeaponKeywordFlag::Rifle },
            { 0x00226454, WeaponKeywordFlag::Shotgun },
            { 0x00226455, WeaponKeywordFlag::AssaultRifle },
            { 0x001E325D, WeaponKeywordFlag::Sniper },
            { 0x00226456, WeaponKeywordFlag::GaussRifle },
            { 0x00226452, WeaponKeywordFlag::LaserMusket },
            { 0x0004A0A3, WeaponKeywordFlag::HeavyGun },
            { 0x00226453, WeaponKeywordFlag::HandToHand },
            { 0x0004A0A4, WeaponKeywordFlag::Melee1H },
            { 0x0004A0A5, WeaponKeywordFlag::Melee2H },
            { 0x0005240E, WeaponKeywordFlag::Unarmed },
            { 0x0022575D, WeaponKeywordFlag::Minigun },
            { 0x0022575C, WeaponKeywordFlag::Fatman },
            { 0x0022575B, WeaponKeywordFlag::MissileLauncher },
            { 0x0022575E, WeaponKeywordFlag::GatlingLaser },
            { 0x00225760, WeaponKeywordFlag::Flamer },
            { 0x0022575F, WeaponKeywordFlag::Cryolater },
            { 0x00225763, WeaponKeywordFlag::JunkJet },
            { 0x00225764, WeaponKeywordFlag::RailwayRifle },
            { 0x00225766, WeaponKeywordFlag::Broadsider },
            { 0x00225765, WeaponKeywordFlag::Syringer },
            { 0x00225761, WeaponKeywordFlag::FlareGun },
            { 0x00225762, WeaponKeywordFlag::GammaGun },
            { 0x0016968B, WeaponKeywordFlag::AlienBlaster },
            { 0x00225767, WeaponKeywordFlag::Ripper },
            { 0x00225768, WeaponKeywordFlag::Shishkebab },
            { 0x00092A84, WeaponKeywordFlag::Laser },
            { 0x00092A85, WeaponKeywordFlag::Plasma },
            { 0x00092A86, WeaponKeywordFlag::Ballistic },
            { 0x0004A0A6, WeaponKeywordFlag::Thrown },
            { 0x0010C415, WeaponKeywordFlag::Grenade },
            { 0x0010C414, WeaponKeywordFlag::Mine },
            { 0x0004C922, WeaponKeywordFlag::Explosive },
            { 0x0004A0A2, WeaponKeywordFlag::Automatic },
        };

        struct ResolvedWeaponKeywordEntry
        {
            const RE::BGSKeyword* keyword{ nullptr };
            WeaponKeywordFlag flag{ WeaponKeywordFlag::None };
        };

        inline const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)>& resolvedWeaponKeywordForms()
        {
            /*
             * Lazily resolved on first use (function-local static, thread-safe
             * magic-static init) because RE::TESForm::GetFormByID requires the
             * game's form table to be populated, which is not guaranteed at
             * static-initialization time. No static initialization-order
             * dependency: this runs on first equipped-weapon identity read,
             * well after data load.
             */
            static const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> resolved = [] {
                std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> table{};
                for (std::size_t i = 0; i < std::size(kWeaponKeywordForms); ++i) {
                    table[i].keyword = RE::TESForm::GetFormByID<RE::BGSKeyword>(kWeaponKeywordForms[i].formId);
                    table[i].flag = kWeaponKeywordForms[i].flag;
                }
                return table;
            }();
            return resolved;
        }

        inline std::uint64_t computeWeaponKeywordFlags(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData)
        {
            std::uint64_t flags = 0;
            if (!weapon) {
                return flags;
            }
            for (const auto& entry : resolvedWeaponKeywordForms()) {
                if (entry.keyword && weapon->HasKeyword(entry.keyword, instanceData)) {
                    flags |= static_cast<std::uint64_t>(entry.flag);
                }
            }
            return flags;
        }

        struct WeaponClassificationResult
        {
            WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
            WeaponClassificationSource source{ WeaponClassificationSource::None };
            std::uint64_t keywordFlags{ 0 };
            bool usedEffectiveInstanceKeywordData{ false };
            bool resolved{ false };
        };

        /*
         * FO4VR 1.2.72 BGSKeywordForm::HasKeyword at 0x140147F50 selects
         * TBO_InstanceData::GetKeywordData() when available. This reads the
         * engine-assembled keyword set after installed OMOD property changes.
         * Native instance weapon data resolves melee weapons. The effective
         * equip slot only disambiguates conflicting pistol and rifle tags.
         * Inconclusive weapons remain unclassified.
         */
        inline WeaponClassificationResult classifyEquippedWeapon(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData,
            const std::uint32_t effectiveEquipSlotFormID)
        {
            WeaponClassificationResult result{};
            if (!weapon) {
                return result;
            }

            const auto* effectiveData = instanceData ?
                static_cast<const RE::TESObjectWEAP::InstanceData*>(instanceData) :
                static_cast<const RE::TESObjectWEAP::InstanceData*>(&weapon->weaponData);
            result.keywordFlags = computeWeaponKeywordFlags(weapon, instanceData);
            const auto policyResult = weapon_classification_policy::classify({
                .keywordFlags = result.keywordFlags,
                .effectiveEquipSlotFormID = effectiveEquipSlotFormID,
                .nativeMeleeType = effectiveData &&
                    (effectiveData->type == RE::WEAPON_TYPE::kHandToHand ||
                     weapon_type_policy::isMelee(effectiveData->type.get())),
            });
            result.sizeClass = policyResult.sizeClass;
            result.source = policyResult.source;
            result.resolved = policyResult.resolved;
            result.usedEffectiveInstanceKeywordData =
                result.keywordFlags != 0 && instanceData &&
                instanceData->GetKeywordData();
            return result;
        }

        inline std::uint64_t makeEquippedWeaponInstanceContentKey(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra)
        {
            if (!instanceData && !objectInstanceExtra) {
                return 0;
            }

            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKEquippedInstanceContentV3");
            mixFormStableContent(key, weapon);
            if (instanceData) {
                mixKeywordFormContent(key, instanceData->GetKeywordData());
                mixBlockBashDataContent(key, instanceData->GetBlockBashData());
                mixFormPointerArray(key, instanceData->GetEnchantmentArray());
                mixFormPointerArray(key, instanceData->GetMaterialSwapArray());
            }
            mixObjectInstanceExtraContent(key, objectInstanceExtra);
            if (instanceData) {
                mixFloatBits(key, instanceData->GetWeight());
                weapon_visual_composition_policy::mixValue(key, static_cast<std::uint32_t>(instanceData->GetValue()));
                weapon_visual_composition_policy::mixValue(key, instanceData->GetHealth());
                mixFloatBits(key, instanceData->GetColorRemappingIndex());
            }
            return key;
        }

        inline weapon_generation_identity_policy::EquippedWeaponGenerationIdentity readEquippedWeaponGenerationIdentity()
        {
            weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};

            auto* player = f4vr::getPlayer();
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            auto* instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            if (!weaponForm || weaponForm->formType != RE::ENUM_FORM_ID::kWEAP) {
                return identity;
            }

            identity.hasEquippedWeapon = true;
            identity.formID = weaponForm->formID;
            identity.formAddress = reinterpret_cast<std::uintptr_t>(weaponForm);
            identity.instanceDataAddress = reinterpret_cast<std::uintptr_t>(instanceData);
            identity.instanceKeywordDataAddress = reinterpret_cast<std::uintptr_t>(
                instanceData ? instanceData->GetKeywordData() : nullptr);
            auto* equippedWeaponData = equipData->data ? static_cast<RE::EquippedWeaponData*>(equipData->data.get()) : nullptr;
            identity.equippedDataAddress = reinterpret_cast<std::uintptr_t>(equippedWeaponData);
            identity.equippedObjectAddress = reinterpret_cast<std::uintptr_t>(
                equippedWeaponData ? equippedWeaponData->fireNode : nullptr);
            const auto* objectInstanceExtra = findEquippedWeaponObjectInstanceExtra(player, weaponForm, instanceData);
            const auto objectInstanceWitness = makeObjectInstanceExtraWitness(objectInstanceExtra);
            identity.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(objectInstanceExtra);
            identity.objectIndexDataSignature = objectInstanceWitness.signature;
            identity.objectIndexDataCount = objectInstanceWitness.count;
            identity.activeModCount = objectInstanceWitness.activeCount;
            identity.disabledModCount = objectInstanceWitness.disabledCount;
            if (const auto* weapon = asEquippedWeaponForm(weaponForm)) {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(weapon, instanceData, objectInstanceExtra);
                /*
                 * FO4VR binary verification (2026-08-05): the TESObjectWEAP
                 * BGSEquipType-subobject override at 0x14033FD70 returns
                 * InstanceData::equipSlot (+0x70) whenever instanceData is
                 * non-null, otherwise BGSEquipType::equipSlot (+0x08). Keep
                 * both values for diagnostic provenance and classify authored
                 * grip behavior from the effective runtime value.
                 */
                const auto* baseEquipSlot = weapon->GetEquipSlot(nullptr);
                const auto* effectiveEquipSlot = weapon->GetEquipSlot(instanceData);
                identity.baseEquipSlotFormID =
                    baseEquipSlot ? baseEquipSlot->formID : 0;
                identity.effectiveEquipSlotFormID =
                    effectiveEquipSlot ? effectiveEquipSlot->formID : 0;
                identity.effectiveEquipSlotUsesInstanceData =
                    instanceData != nullptr;
                float weightGame = instanceData ? instanceData->GetWeight() : -1.0f;
                if (weightGame < 0.0f) {
                    weightGame = weapon->weaponData.weight;
                }
                identity.weightGame = std::isfinite(weightGame) && weightGame > 0.0f ? weightGame : 0.0f;
                const auto classification = classifyEquippedWeapon(
                    weapon,
                    instanceData,
                    identity.effectiveEquipSlotFormID);
                identity.sizeClass = classification.sizeClass;
                identity.classificationSource = classification.source;
                identity.keywordFlags = classification.keywordFlags;
                identity.usedEffectiveInstanceKeywordData =
                    classification.usedEffectiveInstanceKeywordData;
                identity.classificationResolved = classification.resolved;
            } else {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(nullptr, instanceData, objectInstanceExtra);
            }
            const auto fullName = RE::TESFullName::GetFullName(*weaponForm);
            if (!fullName.empty()) {
                identity.displayName = fullName;
            }
            return identity;
        }

        inline weapon_geometry_work::Task splitGeneratedWeaponPointCloudForCollisionDeferred(const std::vector<RE::NiPoint3>& localPoints, GeneratedPointCloudClusterSet& result)
        {
            result = {};
            constexpr auto targetPoints = WEAPON_COLLISION_SUPPORT_FIT_TARGET_POINTS;
            weapon_collision_geometry_math::ConvexSupportFitResult<RE::NiPoint3> fit;
            auto fitting = weapon_collision_geometry_math::fitConvexSupportPointCloudDeferred(
                localPoints,
                targetPoints,
                MAX_CONVEX_HULL_POINTS,
                WEAPON_COLLISION_SUPPORT_FIT_MAX_ERROR_GAME_UNITS, false, fit);
            while (fitting.step()) { co_yield 0; }
            result.supportFitAttempted = fit.attempted;
            result.supportFitAccepted = fit.accepted;
            result.supportFitMaxError = fit.maxSupportError;
            result.supportFitInputPoints = fit.inputPointCount;
            result.supportFitOutputPoints = fit.selectedPointCount;
            result.supportFitRepairPoints = fit.repairPointCount;
            result.supportFitValidationDirections = fit.validationDirectionCount;

            if (fit.accepted && !fit.points.empty()) {
                result.clusters.push_back(fit.points);
                co_return;
            }

            result.supportFitFallbackSplit = true;
            std::vector<std::vector<RE::NiPoint3>> splitClusters;
            auto splitting = weapon_collision_geometry_math::splitOversizedClusterDeferred(localPoints, MAX_CONVEX_HULL_POINTS, splitClusters);
            while (splitting.step()) { co_yield 0; }
            result.clusters.reserve(splitClusters.size());
            for (const auto& splitCluster : splitClusters) {
                weapon_collision_geometry_math::ConvexSupportFitResult<RE::NiPoint3> childFit;
                auto childFitting = weapon_collision_geometry_math::fitConvexSupportPointCloudDeferred(
                    splitCluster,
                    targetPoints,
                    MAX_CONVEX_HULL_POINTS,
                    WEAPON_COLLISION_SUPPORT_FIT_MAX_ERROR_GAME_UNITS, false, childFit);
                while (childFitting.step()) { co_yield 0; }
                if (childFit.accepted && !childFit.points.empty()) {
                    result.clusters.push_back(childFit.points);
                } else {
                    result.clusters.push_back(splitCluster);
                }
            }
            co_return;
        }

        inline GeneratedPointCloudClusterSet splitGeneratedWeaponPointCloudForCollision(const std::vector<RE::NiPoint3>& localPoints)
        {
            GeneratedPointCloudClusterSet result;
            auto task = splitGeneratedWeaponPointCloudForCollisionDeferred(localPoints, result);
            while (task.step()) {}
            return result;
        }

        inline GeneratedHullCoverageInfo classifyGeneratedHullSemantic(const WeaponPartClassification& semantic)
        {
            /*
             * Generated firearm collision must spend its limited body budget on
             * coverage, not triangle density. The log showed dense barrel chunks
             * crowding out stock, magazine, action, and top geometry, then a single
             * overflow hull combined unrelated leftovers. ROCK uses the generated
             * weapon visual tree as one coherent source; for FO4VR firearms this
             * selector keeps that package-level intent while capping bodies.
             */
            switch (semantic.partKind) {
            case WeaponPartKind::Stock:
            case WeaponPartKind::Grip:
                return { HullCoverageStock, semantic.priority, semantic.cosmetic, "stock/grip" };
            case WeaponPartKind::Receiver:
                return { HullCoverageReceiver, semantic.priority, semantic.cosmetic, "receiver/body" };
            case WeaponPartKind::Barrel:
            case WeaponPartKind::MuzzleDevice:
            case WeaponPartKind::Bipod:
            case WeaponPartKind::Handguard:
            case WeaponPartKind::Foregrip:
            case WeaponPartKind::Pump:
                return { HullCoverageBarrel, semantic.priority, semantic.cosmetic, "barrel/support" };
            case WeaponPartKind::Magazine:
            case WeaponPartKind::Magwell:
                return { HullCoverageMagazine, semantic.priority, semantic.cosmetic, "magazine/socket" };
            case WeaponPartKind::Sight:
            case WeaponPartKind::Accessory:
            case WeaponPartKind::LaserSight:
            case WeaponPartKind::Flashlight:
            case WeaponPartKind::LaserFlashlightCombo:
            case WeaponPartKind::Scope:
                return { HullCoverageTopAccessory, semantic.priority, semantic.cosmetic, "top/accessory" };
            case WeaponPartKind::Bolt:
            case WeaponPartKind::Slide:
            case WeaponPartKind::ChargingHandle:
            case WeaponPartKind::BreakAction:
            case WeaponPartKind::Cylinder:
            case WeaponPartKind::Chamber:
            case WeaponPartKind::LaserCell:
            case WeaponPartKind::Lever:
                return { HullCoverageAction, semantic.priority, semantic.cosmetic, "action/reload" };
            case WeaponPartKind::Shell:
            case WeaponPartKind::Round:
            case WeaponPartKind::CosmeticAmmo:
                return { HullCoverageCosmeticAmmo, semantic.priority, true, "cosmetic-ammo" };
            case WeaponPartKind::Other:
            case WeaponPartKind::Count:
            default:
                return { HullCoverageOther, semantic.priority, semantic.cosmetic, "other" };
            }
        }

        inline weapon_collision_geometry_math::HullSelectionInput makeHullSelectionInput(const RE::NiPoint3& localCenterGame, const RE::NiPoint3& localMinGame,
            const RE::NiPoint3& localMaxGame, std::size_t pointCount, const WeaponPartClassification& semantic)
        {
            const auto coverage = classifyGeneratedHullSemantic(semantic);
            return weapon_collision_geometry_math::HullSelectionInput{
                pointToArray(localCenterGame),
                pointToArray(localMinGame),
                pointToArray(localMaxGame),
                pointCount,
                coverage.coverageClass,
                coverage.priority,
                coverage.cosmetic
            };
        }

        inline bool isAssembledWeaponComponentAnchor(WeaponPartKind partKind)
        {
            switch (partKind) {
            case WeaponPartKind::Magazine:
            case WeaponPartKind::Shell:
            case WeaponPartKind::Round:
            case WeaponPartKind::LaserCell:
            case WeaponPartKind::CosmeticAmmo:
            case WeaponPartKind::Other:
            case WeaponPartKind::Count:
                return false;
            default:
                return true;
            }
        }

        inline void addUniqueWeaponMeshRootCandidate(std::vector<WeaponMeshRootCandidate>& candidates, RE::NiAVObject* root, const char* label)
        {
            if (!root) {
                return;
            }
            for (const auto& candidate : candidates) {
                if (candidate.root == root) {
                    return;
                }
            }
            candidates.push_back(WeaponMeshRootCandidate{ root, label });
        }

        inline std::vector<WeaponMeshRootCandidate> makeGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode)
        {
            /*
             * Weapon mesh collision has to be rooted on the visual weapon tree, not
             * the native collision attachment tree. ROCK scans several possible
             * visual roots, but every generated candidate must prove itself by
             * producing visible triangles before it is used for Havok body creation.
             */
            std::vector<WeaponMeshRootCandidate> candidates;
            candidates.reserve(6);

            addUniqueWeaponMeshRootCandidate(candidates, f4vr::getWeaponNode(), "firstPersonSkeleton:Weapon");

            if (auto* playerNodes = f4vr::getPlayerNodes()) {
                addUniqueWeaponMeshRootCandidate(candidates, playerNodes->primaryWeapontoWeaponNode, "PlayerNodes.primaryWeapontoWeaponNode");
                addUniqueWeaponMeshRootCandidate(candidates, playerNodes->primaryWeaponOffsetNOde, "PlayerNodes.primaryWeaponOffsetNode");
            }

            addUniqueWeaponMeshRootCandidate(candidates, updateWeaponNode, "updateWeaponNode");
            return candidates;
        }

        template <class Visitor>
        void visitGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode, Visitor&& visitor)
        {
            std::array<WeaponMeshRootCandidate, 4> candidates{};
            std::size_t count = 0;
            const auto addUnique = [&](RE::NiAVObject* root, const char* label) {
                if (!root) {
                    return;
                }
                for (std::size_t i = 0; i < count; ++i) {
                    if (candidates[i].root == root) {
                        return;
                    }
                }
                if (count < candidates.size()) {
                    candidates[count++] = WeaponMeshRootCandidate{ root, label };
                }
            };

            addUnique(f4vr::getWeaponNode(), "firstPersonSkeleton:Weapon");
            if (auto* playerNodes = f4vr::getPlayerNodes()) {
                addUnique(playerNodes->primaryWeapontoWeaponNode, "PlayerNodes.primaryWeapontoWeaponNode");
                addUnique(playerNodes->primaryWeaponOffsetNOde, "PlayerNodes.primaryWeaponOffsetNode");
            }
            addUnique(updateWeaponNode, "updateWeaponNode");

            for (std::size_t i = 0; i < count; ++i) {
                visitor(candidates[i]);
            }
        }

        inline std::uint64_t makeWeaponEmitterRootSetKey(RE::NiAVObject* updateWeaponNode)
        {
            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            visitGeneratedWeaponMeshRootCandidates(updateWeaponNode, [&](const WeaponMeshRootCandidate& candidate) {
                mixWeaponVisualKey(key, reinterpret_cast<std::uintptr_t>(candidate.root));
            });
            return key;
        }

        /*
         * sourceScale re-bakes a source NiNode's own NiTransform::scale into
         * the point cloud before Havok conversion. It must be 1.0 for points
         * already expressed in a frame with no scale divided out (e.g.
         * weapon-root-local localPointsGame under a scale=1.0 weapon root);
         * pass the captured GeneratedHullSource::sourceNodeScale for points
         * expressed in a source node's own local space
         * (sourceLocalPointsGame), since Havok never re-applies NiNode scale
         * to a built shape at runtime.
         */
        inline std::vector<RE::NiPoint3> makeCenteredHavokPointCloud(const std::vector<RE::NiPoint3>& localPointsGame, const RE::NiPoint3& localCenterGame, float sourceScale = 1.0f)
        {
            std::vector<RE::NiPoint3> result;
            result.reserve(localPointsGame.size());
            const float scaledHavokScale = sourceScale * gameToHavokScale();
            for (const auto& point : localPointsGame) {
                result.emplace_back((point.x - localCenterGame.x) * scaledHavokScale, (point.y - localCenterGame.y) * scaledHavokScale,
                    (point.z - localCenterGame.z) * scaledHavokScale);
            }
            return result;
        }

        inline const char* safeNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }
            const char* name = node->name.c_str();
            return name ? name : "(null)";
        }

        [[nodiscard]] inline bool niObjectRttiChainContains(const RE::NiObject* object, const char* typeName)
        {
            const RE::NiRTTI* rtti = object && typeName ? object->GetRTTI() : nullptr;
            for (int depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
                const char* rttiName = rtti->GetName();
                if (rttiName && std::strcmp(rttiName, typeName) == 0) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] inline bool generatedWeaponShapeHasEffectShaderProperty(const RE::BSTriShape* triShape)
        {
            if (!triShape) {
                return false;
            }
            for (const auto& property : triShape->GetRuntimeData().properties) {
                if (niObjectRttiChainContains(property.get(), "BSEffectShaderProperty")) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] inline bool generatedWeaponShapeHasBillboardAncestor(const RE::NiAVObject* node)
        {
            // Parent links are frame-scoped engine references; nothing from this walk is retained.
            for (auto* ancestor = node ? node->parent : nullptr; ancestor; ancestor = ancestor->parent) {
                if (niObjectRttiChainContains(ancestor, "NiBillboardNode")) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] inline weapon_effect_geometry_policy::ExclusionReason classifyGeneratedWeaponEffectGeometry(
            const RE::BSTriShape* triShape)
        {
            return weapon_effect_geometry_policy::classify({
                .hasEffectShaderProperty = generatedWeaponShapeHasEffectShaderProperty(triShape),
                .hasBillboardAncestor = generatedWeaponShapeHasBillboardAncestor(triShape),
                .geometryName = safeNodeName(triShape),
            });
        }

        inline bool weaponVisualNodeVisible(const RE::NiAVObject* node)
        {
            if (!node) {
                return false;
            }
            return (node->flags.flags & 1) == 0 && !node->GetAppCulled() && node->local.scale != 0.0f;
        }

        inline std::uintptr_t readRendererChildPointer(void* rendererData, std::ptrdiff_t rendererChildOffset)
        {
            if (!rendererData) {
                return 0;
            }
            auto* child = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + rendererChildOffset);
            if (!child) {
                return 0;
            }
            return reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(reinterpret_cast<char*>(child) + 0x08));
        }

        inline weapon_visual_composition_policy::VisualRecord makeWeaponVisualRecord(
            RE::NiAVObject* node,
            RE::NiAVObject* parent,
            std::uint32_t childIndex,
            std::uint32_t childCount,
            std::uint32_t depth,
            bool visible)
        {
            weapon_visual_composition_policy::VisualRecord record{
                .nodeAddress = reinterpret_cast<std::uintptr_t>(node),
                .parentAddress = reinterpret_cast<std::uintptr_t>(parent ? parent : node ? node->parent : nullptr),
                .name = safeNodeName(node),
                .depth = depth,
                .childIndex = childIndex,
                .childCount = childCount,
                .visible = visible,
                .triShape = node && node->IsTriShape(),
            };

            if (auto* triShape = node ? node->IsTriShape() : nullptr) {
                auto* base = reinterpret_cast<char*>(triShape);
                auto* rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
                record.rendererData = reinterpret_cast<std::uintptr_t>(rendererData);
                record.skinInstance = reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(base + VROffset::skinInstance));
                record.vertexDesc = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
                record.numTriangles = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
                record.numVertices = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
                record.geometryType = *reinterpret_cast<std::uint8_t*>(base + VROffset::geometryType);
                record.vertexBlock = readRendererChildPointer(rendererData, 0x08);
                record.triangleBlock = readRendererChildPointer(rendererData, 0x10);
            }

            return record;
        }

        inline void accumulateWeaponVisualKey(RE::NiAVObject* node, RE::NiAVObject* parent, std::uint32_t childIndex, int depth, std::uint64_t& key, WeaponVisualKeyStats& stats)
        {
            if (!node || depth > 15 || stats.nodeCount > 512) {
                return;
            }

            /*
             * Effect-only geometry cannot create collision and therefore must
             * not make flashlight/laser/reticle visibility toggle the weapon's
             * collision generation key. A billboard subtree is effect-only by
             * construction; other effect shapes are filtered by shader/name.
             */
            if (niObjectRttiChainContains(node, "NiBillboardNode")) {
                return;
            }
            if (auto* triShape = node->IsTriShape();
                triShape && classifyGeneratedWeaponEffectGeometry(triShape) != weapon_effect_geometry_policy::ExclusionReason::None) {
                return;
            }

            const bool visible = weaponVisualNodeVisible(node);
            ++stats.nodeCount;
            if (!visible) {
                ++stats.invisibleNodeCount;
            }

            std::uint32_t childCount = 0;
            if (auto* niNode = node->IsNode()) {
                childCount = static_cast<std::uint32_t>(niNode->GetRuntimeData().children.size());
            }
            const auto record = makeWeaponVisualRecord(node, parent, childIndex, childCount, static_cast<std::uint32_t>(depth), visible);
            weapon_visual_composition_policy::mixVisualRecord(key, record);

            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                if (visible) {
                    if (record.rendererData == 0 || record.vertexBlock == 0 || record.triangleBlock == 0) {
                        ++stats.missingRendererCount;
                    } else if (record.numTriangles == 0 || record.numVertices == 0) {
                        ++stats.emptyGeometryCount;
                    } else {
                        ++stats.visibleTriShapeCount;
                    }
                }
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            auto& kids = niNode->GetRuntimeData().children;
            visitWeaponChildSlots(kids, [&](auto* kid, auto i) {
                accumulateWeaponVisualKey(kid, node, i, depth + 1, key, stats);
                return stats.nodeCount <= 512;
            });
        }

        [[nodiscard]] inline bool weaponEmitterNodeEffectivelyVisible(const RE::NiAVObject* node)
        {
            int step = 0;
            for (auto* current = node; current && step < 32; current = current->parent, ++step) {
                if (!weaponVisualNodeVisible(current)) {
                    return false;
                }
            }
            return node != nullptr && step < 32;
        }

        template <class Predicate>
        [[nodiscard]] bool weaponEmitterAncestorMatches(const RE::NiAVObject* node, Predicate&& predicate)
        {
            int step = 0;
            for (auto* ancestor = node ? node->parent : nullptr; ancestor && step < 32; ancestor = ancestor->parent, ++step) {
                if (predicate(std::string_view{ safeNodeName(ancestor) })) {
                    return true;
                }
            }
            return false;
        }

        template <class Predicate>
        [[nodiscard]] bool weaponEmitterImmediateSiblingMatches(const RE::NiAVObject* node, Predicate&& predicate)
        {
            auto* parent = node && node->parent ? node->parent->IsNode() : nullptr;
            if (!parent) {
                return false;
            }
            const auto& children = parent->GetRuntimeData().children;
            const std::uint16_t count = (std::min)(children.capacity(), static_cast<std::uint16_t>(64));
            for (std::uint16_t i = 0; i < count; ++i) {
                const auto* sibling = children[i].get();
                if (sibling && sibling != node && predicate(std::string_view{ safeNodeName(sibling) })) {
                    return true;
                }
            }
            return false;
        }

        struct WeaponEmitterStructuralContext
        {
            weapon_part_record_identity_policy::StructureAnchor anchor{ weapon_part_record_identity_policy::StructureAnchor::None };
            RE::NiAVObject* ownerRoot{ nullptr };
            bool ownerRootStructural{ false };
        };

        [[nodiscard]] inline bool isWeaponAttachmentPointNode(std::string_view name)
        {
            return name.starts_with("P-");
        }

        [[nodiscard]] inline WeaponEmitterStructuralContext resolveWeaponEmitterStructuralContext(
            RE::NiAVObject* node,
            RE::NiAVObject* candidateRoot)
        {
            WeaponEmitterStructuralContext result{};
            RE::NiAVObject* childBelowAncestor = node;
            int step = 0;
            for (auto* ancestor = node ? node->parent : nullptr; ancestor && step < 32; ancestor = ancestor->parent, ++step) {
                const std::string_view ancestorName{ safeNodeName(ancestor) };
                const auto anchor = weapon_part_record_identity_policy::resolveStructureAnchor(ancestorName);
                if (anchor != weapon_part_record_identity_policy::StructureAnchor::None) {
                    result.anchor = anchor;
                    result.ownerRoot = childBelowAncestor;
                    result.ownerRootStructural = true;
                    return result;
                }
                /*
                 * Mod-added attachment slots do not have a vanilla attach-point
                 * FormID mapping, but their P-* boundary still gives us a safe
                 * physical owner subtree. Stop here instead of walking upward
                 * and incorrectly assigning the emitter to P-Receiver/barrel.
                 */
                if (isWeaponAttachmentPointNode(ancestorName)) {
                    result.ownerRoot = childBelowAncestor;
                    result.ownerRootStructural = true;
                    return result;
                }
                childBelowAncestor = ancestor;
            }
            result.ownerRoot = candidateRoot;
            return result;
        }

        [[nodiscard]] inline bool weaponTransformFinite(const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        inline constexpr std::size_t kMaximumWeaponLocalHierarchyDepth = 64;

        [[nodiscard]] inline bool weaponSceneNodePointerPlausible(const RE::NiAVObject* node) noexcept
        {
            const auto address = reinterpret_cast<std::uintptr_t>(node);
            return address >= 0x10000 && address <= 0x0000'7FFF'FFFF'FFFFull;
        }

        [[nodiscard]] inline bool tryResolveDescendantLocalTransform(
            const RE::NiAVObject* ancestor,
            const RE::NiAVObject* descendant,
            RE::NiTransform& outDescendantLocal)
        {
            /*
             * Descendant world transforms can belong to a different scene-graph
             * propagation epoch than the current weapon root. Compose the
             * bounded parent path so callers receive one coherent local frame.
             * Shoulder draw can also leave that coherent frame in presentation
             * space; post-undraw callers remove that separately with a validated
             * source-frame correction.
             */
            outDescendantLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
            if (!ancestor || !descendant) {
                return false;
            }

            std::array<const RE::NiAVObject*, kMaximumWeaponLocalHierarchyDepth> reversePath{};
            std::size_t pathLength = 0;
            auto* cursor = descendant;
            while (cursor && cursor != ancestor) {
                if (!weaponSceneNodePointerPlausible(cursor) ||
                    pathLength >= reversePath.size() ||
                    !weaponTransformFinite(cursor->local)) {
                    outDescendantLocal = {};
                    return false;
                }
                reversePath[pathLength++] = cursor;
                cursor = cursor->parent;
            }
            if (cursor != ancestor) {
                outDescendantLocal = {};
                return false;
            }

            while (pathLength != 0) {
                outDescendantLocal = transform_math::composeTransforms(
                    outDescendantLocal,
                    reversePath[--pathLength]->local);
            }
            if (!weaponTransformFinite(outDescendantLocal)) {
                outDescendantLocal = {};
                return false;
            }
            return true;
        }

        [[nodiscard]] inline bool tryResolveDescendantWorldTransform(
            const RE::NiAVObject* ancestor,
            const RE::NiTransform& ancestorWorld,
            const RE::NiAVObject* descendant,
            RE::NiTransform& outDescendantWorld)
        {
            RE::NiTransform descendantLocal{};
            if (!weaponTransformFinite(ancestorWorld) ||
                !tryResolveDescendantLocalTransform(ancestor, descendant, descendantLocal)) {
                outDescendantWorld = {};
                return false;
            }

            outDescendantWorld = transform_math::composeTransforms(ancestorWorld, descendantLocal);
            if (!weaponTransformFinite(outDescendantWorld)) {
                outDescendantWorld = {};
                return false;
            }
            return true;
        }

        inline bool updateWeaponEmitterTransform(
            WeaponEmitterDescriptor& descriptor,
            const RE::NiAVObject* transformNode,
            const RE::NiAVObject* weaponRoot)
        {
            if (!transformNode || !weaponRoot || !weaponTransformFinite(transformNode->world) ||
                !weaponTransformFinite(weaponRoot->world) || std::abs(weaponRoot->world.scale) <= 0.000001f) {
                return false;
            }

            const RE::NiTransform weaponLocal = transform_math::composeTransforms(
                transform_math::invertTransform(weaponRoot->world),
                transformNode->world);
            if (!weaponTransformFinite(weaponLocal)) {
                return false;
            }

            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    descriptor.rotate[static_cast<std::size_t>(row * 3 + column)] = weaponLocal.rotate.entry[row][column];
                }
            }
            descriptor.translate = { weaponLocal.translate.x, weaponLocal.translate.y, weaponLocal.translate.z };
            descriptor.scale = weaponLocal.scale;

            // Weapon attachment effects in FO4 NIFs emit along their local +Y axis.
            RE::NiPoint3 forward{
                weaponLocal.rotate.entry[1][0],
                weaponLocal.rotate.entry[1][1],
                weaponLocal.rotate.entry[1][2],
            };
            const float length = forward.Length();
            if (!std::isfinite(length) || length <= 0.000001f) {
                return false;
            }
            forward /= length;
            descriptor.forwardWeaponLocal = { forward.x, forward.y, forward.z };
            return true;
        }

        inline void copyWeaponEmitterSourceName(WeaponEmitterDescriptor& descriptor, std::string_view name)
        {
            descriptor.sourceName.fill('\0');
            const std::size_t copyCount = (std::min)(name.size(), descriptor.sourceName.size() - 1);
            if (copyCount != 0) {
                std::memcpy(descriptor.sourceName.data(), name.data(), copyCount);
            }
        }

        [[nodiscard]] inline bool weaponEmitterDescriptorsShareOwner(
            const WeaponEmitterDescriptor& lhs,
            const WeaponEmitterDescriptor& rhs)
        {
            if (lhs.kind != rhs.kind) {
                return false;
            }
            if (lhs.transformNodeAddress != 0 && lhs.transformNodeAddress == rhs.transformNodeAddress) {
                return true;
            }
            if (lhs.omodFormId != 0 && rhs.omodFormId != 0) {
                return lhs.omodFormId == rhs.omodFormId;
            }
            if (lhs.attachPointFormId != 0 && rhs.attachPointFormId != 0) {
                return lhs.attachPointFormId == rhs.attachPointFormId;
            }
            return lhs.ownerRootAddress != 0 && lhs.ownerRootAddress == rhs.ownerRootAddress;
        }

        inline void mergeWeaponEmitterDescriptor(WeaponEmitterSnapshot& snapshot, const WeaponEmitterDescriptor& candidate)
        {
            WeaponEmitterDescriptor* destination = nullptr;
            for (std::size_t i = 0; i < snapshot.count; ++i) {
                if (weaponEmitterDescriptorsShareOwner(snapshot.emitters[i], candidate)) {
                    destination = &snapshot.emitters[i];
                    break;
                }
            }
            if (!destination) {
                if (snapshot.count >= snapshot.emitters.size()) {
                    return;
                }
                destination = &snapshot.emitters[snapshot.count++];
                *destination = candidate;
                return;
            }

            destination->active = destination->active || candidate.active;
            destination->effectStateKnown = destination->effectStateKnown || candidate.effectStateKnown;
            if (candidate.effectStateKnown && destination->effectNodeAddress == 0) {
                destination->effectNodeAddress = candidate.effectNodeAddress;
            }
            if (candidate.hasAddOnNodeValue) {
                destination->hasAddOnNodeValue = true;
                destination->addOnNodeValue = candidate.addOnNodeValue;
            }
            if (destination->omodFormId == 0) {
                destination->omodFormId = candidate.omodFormId;
            }
            if (destination->attachPointFormId == 0) {
                destination->attachPointFormId = candidate.attachPointFormId;
            }
            if (candidate.transformPriority > destination->transformPriority) {
                const bool active = destination->active;
                const bool effectStateKnown = destination->effectStateKnown;
                const std::uintptr_t effectNodeAddress = destination->effectNodeAddress;
                const bool hasAddOnNodeValue = destination->hasAddOnNodeValue;
                const std::uint32_t addOnNodeValue = destination->addOnNodeValue;
                const std::uint32_t omodFormId = destination->omodFormId;
                const std::uint32_t attachPointFormId = destination->attachPointFormId;
                *destination = candidate;
                destination->active = active;
                destination->effectStateKnown = effectStateKnown;
                destination->effectNodeAddress = effectNodeAddress;
                destination->hasAddOnNodeValue = hasAddOnNodeValue;
                destination->addOnNodeValue = addOnNodeValue;
                destination->omodFormId = omodFormId;
                destination->attachPointFormId = attachPointFormId;
            }
        }

        inline void collectWeaponEmittersRecursive(
            RE::NiAVObject* node,
            RE::NiAVObject* candidateRoot,
            RE::NiAVObject* weaponRoot,
            const std::unordered_map<std::uint32_t, std::uint32_t>& omodByAttachPointFormId,
            std::uint64_t weaponGenerationKey,
            int depth,
            std::uint32_t& visitedNodes,
            WeaponEmitterSnapshot& snapshot)
        {
            if (!node || depth > 15 || visitedNodes >= 512) {
                return;
            }
            ++visitedNodes;

            const bool valueNode = niObjectRttiChainContains(node, "BSValueNode");
            auto* triShape = node->IsTriShape();
            const bool effectGeometry = triShape &&
                classifyGeneratedWeaponEffectGeometry(triShape) != weapon_effect_geometry_policy::ExclusionReason::None;
            if (valueNode || effectGeometry) {
                const auto structural = resolveWeaponEmitterStructuralContext(node, candidateRoot);
                const auto hasLaserName = [](std::string_view name) { return weapon_emitter_policy::hasLaserRoleName(name); };
                const auto hasFlashlightName = [](std::string_view name) { return weapon_emitter_policy::hasFlashlightRoleName(name); };
                const bool laserContext = weaponEmitterAncestorMatches(node, hasLaserName) ||
                    (valueNode && weaponEmitterImmediateSiblingMatches(node, hasLaserName));
                const bool flashlightContext = weaponEmitterAncestorMatches(node, hasFlashlightName) ||
                    (valueNode && weaponEmitterImmediateSiblingMatches(node, hasFlashlightName));
                const bool sightContext = structural.anchor == weapon_part_record_identity_policy::StructureAnchor::SlotSight ||
                    weaponEmitterAncestorMatches(node, [](std::string_view name) {
                        return weapon_effect_geometry_policy::containsAsciiInsensitive(name, "scope") ||
                               weapon_effect_geometry_policy::containsAsciiInsensitive(name, "sight") ||
                               weapon_effect_geometry_policy::containsAsciiInsensitive(name, "optic");
                    });
                const auto kind = weapon_emitter_policy::classify({
                    .nodeName = safeNodeName(node),
                    .effectGeometry = effectGeometry,
                    .valueNode = valueNode,
                    .laserContext = laserContext,
                    .flashlightContext = flashlightContext,
                    .sightContext = sightContext,
                });
                if (kind != weapon_emitter_policy::Kind::Unknown) {
                    WeaponEmitterDescriptor descriptor{};
                    descriptor.valid = true;
                    descriptor.active = effectGeometry && weaponEmitterNodeEffectivelyVisible(node);
                    descriptor.visible = weaponEmitterNodeEffectivelyVisible(node);
                    descriptor.effectStateKnown = effectGeometry;
                    descriptor.kind = static_cast<std::uint32_t>(kind);
                    const auto source = valueNode ? weapon_emitter_policy::Source::AddOnNode : weapon_emitter_policy::Source::EffectGeometry;
                    descriptor.source = static_cast<std::uint32_t>(source);
                    descriptor.transformPriority = weapon_emitter_policy::transformPriority(kind, source, safeNodeName(node));
                    descriptor.weaponGenerationKey = weaponGenerationKey;
                    descriptor.transformNodeAddress = reinterpret_cast<std::uintptr_t>(node);
                    descriptor.effectNodeAddress = effectGeometry ? reinterpret_cast<std::uintptr_t>(node) : 0;
                    descriptor.ownerRootAddress = reinterpret_cast<std::uintptr_t>(structural.ownerRoot);
                    descriptor.ownerRootStructural = structural.ownerRootStructural;
                    descriptor.attachPointFormId = weapon_part_record_identity_policy::attachPointFormIdForAnchor(structural.anchor);
                    if (descriptor.attachPointFormId != 0) {
                        const auto omod = omodByAttachPointFormId.find(descriptor.attachPointFormId);
                        if (omod != omodByAttachPointFormId.end()) {
                            descriptor.omodFormId = omod->second;
                        }
                    }
                    if (valueNode) {
                        const auto value = weapon_emitter_policy::parseAddOnNodeValue(safeNodeName(node));
                        descriptor.hasAddOnNodeValue = value.valid;
                        descriptor.addOnNodeValue = value.value;
                    }
                    copyWeaponEmitterSourceName(descriptor, safeNodeName(node));
                    if (updateWeaponEmitterTransform(descriptor, node, weaponRoot)) {
                        mergeWeaponEmitterDescriptor(snapshot, descriptor);
                    }
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < children.capacity(); ++i) {
                collectWeaponEmittersRecursive(
                    children[i].get(),
                    candidateRoot,
                    weaponRoot,
                    omodByAttachPointFormId,
                    weaponGenerationKey,
                    depth + 1,
                    visitedNodes,
                    snapshot);
            }
        }

        [[nodiscard]] inline WeaponEmitterSnapshot collectWeaponEmitterSnapshot(
            RE::NiAVObject* weaponNode,
            const std::unordered_map<std::uint32_t, std::uint32_t>& omodByAttachPointFormId,
            std::uint64_t equippedWeaponKey,
            std::uint64_t weaponGenerationKey,
            std::uint64_t rootSetKey)
        {
            WeaponEmitterSnapshot snapshot{};
            if (!weaponNode || equippedWeaponKey == 0 || weaponGenerationKey == 0) {
                return snapshot;
            }

            snapshot.weaponGenerationKey = weaponGenerationKey;
            snapshot.equippedWeaponKey = equippedWeaponKey;
            snapshot.rootSetKey = rootSetKey;
            snapshot.weaponRootAddress = reinterpret_cast<std::uintptr_t>(weaponNode);
            visitGeneratedWeaponMeshRootCandidates(weaponNode, [&](const WeaponMeshRootCandidate& candidate) {
                std::uint32_t visitedNodes = 0;
                collectWeaponEmittersRecursive(
                    candidate.root,
                    candidate.root,
                    weaponNode,
                    omodByAttachPointFormId,
                    weaponGenerationKey,
                    0,
                    visitedNodes,
                    snapshot);
            });
            return snapshot;
        }

        inline void refreshWeaponEmittersRecursive(
            RE::NiAVObject* node,
            RE::NiAVObject* weaponRoot,
            int depth,
            std::uint32_t& visitedNodes,
            WeaponEmitterSnapshot& snapshot)
        {
            if (!node || depth > 15 || visitedNodes >= 512) {
                return;
            }
            ++visitedNodes;

            const auto address = reinterpret_cast<std::uintptr_t>(node);
            for (std::size_t i = 0; i < snapshot.count; ++i) {
                auto& descriptor = snapshot.emitters[i];
                if (descriptor.transformNodeAddress == address) {
                    descriptor.visible = weaponEmitterNodeEffectivelyVisible(node);
                    (void)updateWeaponEmitterTransform(descriptor, node, weaponRoot);
                }
                if (descriptor.effectNodeAddress == address) {
                    descriptor.active = weaponEmitterNodeEffectivelyVisible(node);
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < children.capacity(); ++i) {
                refreshWeaponEmittersRecursive(children[i].get(), weaponRoot, depth + 1, visitedNodes, snapshot);
            }
        }

        inline void shapeRemoveRef(const RE::hknpShape* shape)
        {
            if (!shape)
                return;
            auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(shape)) + 0x08);
            for (;;) {
                long oldVal = *refCountDword;
                std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
                if (rc == 0xFFFF || rc == 0)
                    return;
                long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc - 1));
                if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal)
                    return;
            }
        }

        inline std::uint32_t generatedWeaponCollisionFilterInfo(bool collisionEnabled)
        {
            const std::uint32_t baseFilterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
            return collisionEnabled ? baseFilterInfo : (baseFilterInfo | collision_suppression_registry::kSuppressionNoCollideBit);
        }

        inline bool isFiniteOrderedEvidenceBounds(const WeaponEvidenceBounds3& bounds)
        {
            return bounds.valid && std::isfinite(bounds.min.x) && std::isfinite(bounds.min.y) && std::isfinite(bounds.min.z) && std::isfinite(bounds.max.x) &&
                std::isfinite(bounds.max.y) && std::isfinite(bounds.max.z) && bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y && bounds.min.z <= bounds.max.z;
        }

        inline WeaponCollision::NativeScopeSightAnchorSnapshot buildNativeScopeSightAnchorSnapshot(
            std::uint64_t weaponGenerationKey,
            std::uint64_t equippedWeaponOwnershipKey,
            std::uint32_t weaponFormID,
            const std::vector<WeaponCollisionProfileEvidenceDescriptor>& descriptors)
        {
            WeaponCollision::NativeScopeSightAnchorSnapshot snapshot{};
            snapshot.weaponGenerationKey = weaponGenerationKey;
            snapshot.equippedWeaponOwnershipKey = equippedWeaponOwnershipKey;
            snapshot.weaponFormID = weaponFormID;

            bool hasSightBounds = false;
            RE::NiPoint3 sightBoundsMin{};
            RE::NiPoint3 sightBoundsMax{};
            const auto accumulatePartKind = [&](WeaponPartKind partKind) {
                for (const auto& descriptor : descriptors) {
                    if (!descriptor.valid || descriptor.weaponGenerationKey != weaponGenerationKey || descriptor.semantic.partKind != partKind ||
                        !isFiniteOrderedEvidenceBounds(descriptor.localBoundsGame)) {
                        continue;
                    }

                    const RE::NiPoint3 candidateMin{
                        descriptor.localBoundsGame.min.x,
                        descriptor.localBoundsGame.min.y,
                        descriptor.localBoundsGame.min.z,
                    };
                    const RE::NiPoint3 candidateMax{
                        descriptor.localBoundsGame.max.x,
                        descriptor.localBoundsGame.max.y,
                        descriptor.localBoundsGame.max.z,
                    };
                    if (!hasSightBounds) {
                        sightBoundsMin = candidateMin;
                        sightBoundsMax = candidateMax;
                        hasSightBounds = true;
                    } else {
                        sightBoundsMin.x = (std::min)(sightBoundsMin.x, candidateMin.x);
                        sightBoundsMin.y = (std::min)(sightBoundsMin.y, candidateMin.y);
                        sightBoundsMin.z = (std::min)(sightBoundsMin.z, candidateMin.z);
                        sightBoundsMax.x = (std::max)(sightBoundsMax.x, candidateMax.x);
                        sightBoundsMax.y = (std::max)(sightBoundsMax.y, candidateMax.y);
                        sightBoundsMax.z = (std::max)(sightBoundsMax.z, candidateMax.z);
                    }
                    ++snapshot.sightBodyCount;
                }
            };

            // Native-overlay OMOD evidence isolates the actual scope. Retain
            // Sight as a compatibility fallback when record pairing is absent.
            accumulatePartKind(WeaponPartKind::Scope);
            if (!hasSightBounds) {
                accumulatePartKind(WeaponPartKind::Sight);
            }

            if (!hasSightBounds) {
                return snapshot;
            }

            const RE::NiPoint3 anchor = native_scope_camera_follow_math::rearPlaneCenterFromSightBounds(sightBoundsMin, sightBoundsMax);
            if (!std::isfinite(anchor.x) || !std::isfinite(anchor.y) || !std::isfinite(anchor.z)) {
                snapshot.sightBodyCount = 0;
                return snapshot;
            }

            snapshot.anchorWeaponLocal = anchor;
            snapshot.sightBoundsMinWeaponLocal = sightBoundsMin;
            snapshot.sightBoundsMaxWeaponLocal = sightBoundsMax;
            snapshot.valid = true;
            return snapshot;
        }
    }

    // Implementation TUs resolve the helper names unqualified, exactly as
    // they did inside the original anonymous namespace.
    using namespace weapon_collision_internal;
}
