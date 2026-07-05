#include "physics-interaction/weapon/WeaponCollision.h"

#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "RockConfig.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

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

#include "f4vr/PlayerNodes.h"
#include "f4vr/F4VRUtils.h"
#include "f4sevr/Forms.h"

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

namespace rock
{
    namespace
    {
        constexpr std::size_t MAX_CONVEX_HULL_POINTS = 0xFC;
        constexpr float MIN_HULL_DIAGONAL_GAME_UNITS = 0.5f;
        constexpr std::size_t MAX_GENERATED_CHILD_CONVEXES_PER_SOURCE = 16;
        constexpr std::size_t GENERATED_WEAPON_BODY_CREATION_BATCH = 8;

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

        constexpr std::array<const char*, 11> WEAPON_ANIM_NODE_DUMP_TARGETS{
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

        constexpr int WEAPON_ANIM_NODE_DUMP_MAX_DEPTH = 32;
        constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_MATCHES_PER_NAME = 32;
        constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES = 4096;
        constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_CHILD_NAMES = 16;
        constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES = 4096;
        constexpr int WEAPON_ANIM_NODE_DUMP_MAX_FLATTENED_TRANSFORMS = 768;

        bool weaponVisualNodeVisible(const RE::NiAVObject* node);
        const char* safeNodeName(RE::NiAVObject* node);

        const char* generatedWeaponPartKindName(WeaponPartKind kind)
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
            default:
                return "Other";
            }
        }

        bool weaponAnimNodeNameMatches(const RE::NiAVObject* node, const char* targetName)
        {
            if (!node || !targetName) {
                return false;
            }
            return _stricmp(targetName, node->name.c_str()) == 0;
        }

        void collectWeaponAnimNodeMatchesRecursive(
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
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    collectWeaponAnimNodeMatchesRecursive(child, targetName, path, depth + 1, visited, outMatches);
                }
            }
        }

        std::vector<WeaponAnimNodeMatch> collectWeaponAnimNodeMatches(RE::NiAVObject* root, const char* targetName)
        {
            std::vector<WeaponAnimNodeMatch> matches;
            std::size_t visited = 0;
            collectWeaponAnimNodeMatchesRecursive(root, targetName, {}, 0, visited, matches);
            return matches;
        }

        void accumulateWeaponAnimNodeSubtreeStats(
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
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    accumulateWeaponAnimNodeSubtreeStats(child, stats, depth + 1, visited);
                }
            }
        }

        WeaponAnimNodeSubtreeStats summarizeWeaponAnimNodeSubtree(RE::NiAVObject* node)
        {
            WeaponAnimNodeSubtreeStats stats{};
            std::size_t visited = 0;
            accumulateWeaponAnimNodeSubtreeStats(node, stats, 0, visited);
            return stats;
        }

        std::string weaponAnimNodeImmediateChildNames(RE::NiAVObject* node)
        {
            auto* niNode = node ? node->IsNode() : nullptr;
            if (!niNode) {
                return "";
            }

            std::string result;
            const auto& children = niNode->children;
            std::size_t appended = 0;
            for (auto i = decltype(children.size()){ 0 }; i < children.size() && appended < WEAPON_ANIM_NODE_DUMP_MAX_CHILD_NAMES; ++i) {
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

        bool weaponAnimFlattenedTreeValid(const f4vr::BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= WEAPON_ANIM_NODE_DUMP_MAX_FLATTENED_TRANSFORMS;
        }

        const char* weaponAnimFlattenedTransformName(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform)
        {
            const char* name = transform.name.c_str();
            return name && name[0] != '\0' ? name : "(unnamed)";
        }

        bool weaponAnimFlattenedTransformNameMatches(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform, const char* targetName)
        {
            if (!targetName) {
                return false;
            }

            const char* name = transform.name.c_str();
            return name && _stricmp(targetName, name) == 0;
        }

        std::vector<WeaponAnimFlattenedBoneMatch> collectWeaponAnimFlattenedBoneMatches(
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

        const char* weaponAnimFlattenedParentName(const f4vr::BSFlattenedBoneTree* tree, int parentIndex)
        {
            if (!weaponAnimFlattenedTreeValid(tree) || parentIndex < 0 || parentIndex >= tree->numTransforms) {
                return "(none)";
            }

            return weaponAnimFlattenedTransformName(tree->transforms[parentIndex]);
        }

        void logWeaponAnimNodeMapRoot(const WeaponAnimNodeDumpRoot& dumpRoot)
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

        void logWeaponAnimFlattenedMapRoot(const WeaponAnimFlattenedRoot& flatRoot)
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

        std::string generatedWeaponSemanticMaskNames(std::uint32_t mask)
        {
            std::string result;
            for (std::uint32_t i = 0; i <= static_cast<std::uint32_t>(WeaponPartKind::Other); ++i) {
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

        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        float bodyBasisDeterminant(const float* bodyFloats)
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

        RE::NiTransform makeIdentityTransform()
        {
            RE::NiTransform result{};
            result.rotate.entry[0][0] = 1.0f;
            result.rotate.entry[1][1] = 1.0f;
            result.rotate.entry[2][2] = 1.0f;
            result.scale = 1.0f;
            return result;
        }

        QuantizedPointKey quantizePoint(const RE::NiPoint3& point, float grid)
        {
            const float safeGrid = (std::max)(grid, 0.0001f);
            return QuantizedPointKey{ static_cast<std::int64_t>(std::llround(point.x / safeGrid)), static_cast<std::int64_t>(std::llround(point.y / safeGrid)),
                static_cast<std::int64_t>(std::llround(point.z / safeGrid)) };
        }

        void mixWeaponVisualKey(std::uint64_t& key, std::uint64_t value)
        {
            weapon_visual_composition_policy::mixValue(key, value);
        }

        void mixWeaponVisualString(std::uint64_t& key, const char* value)
        {
            if (!value) {
                return;
            }
            weapon_visual_composition_policy::mixString(key, value);
        }

        std::vector<RE::NiPoint3> dedupePointCloud(const std::vector<RE::NiPoint3>& points, float grid)
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

        float pointCloudDiagonalSquared(const std::vector<RE::NiPoint3>& points)
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

        bool pointCloudCanBuildHull(const std::vector<RE::NiPoint3>& points)
        {
            return points.size() >= 4 && pointCloudDiagonalSquared(points) >= (MIN_HULL_DIAGONAL_GAME_UNITS * MIN_HULL_DIAGONAL_GAME_UNITS);
        }

        bool compressGeneratedChildClustersForBudget(
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

            const auto targetPoints = static_cast<std::size_t>((std::max)(4, g_rockConfig.rockWeaponCollisionSupportFitTargetPoints));
            const auto fit = weapon_collision_geometry_math::fitConvexSupportPointCloud(
                sourcePoints,
                targetPoints,
                MAX_CONVEX_HULL_POINTS,
                g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits);
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

        PointCloudBounds pointCloudBounds(const std::vector<RE::NiPoint3>& points)
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

        std::array<float, 3> pointToArray(const RE::NiPoint3& point)
        {
            return { point.x, point.y, point.z };
        }

        void mixFormPointer(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, reinterpret_cast<std::uintptr_t>(form));
            if (form) {
                weapon_visual_composition_policy::mixValue(key, form->formID);
            }
        }

        void mixFormStableContent(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, form ? form->formID : 0);
        }

        void mixFloatBits(std::uint64_t& key, float value)
        {
            weapon_visual_composition_policy::mixValue(key, std::bit_cast<std::uint32_t>(value));
        }

        void mixKeywordFormContent(std::uint64_t& key, const RE::BGSKeywordForm* keywords)
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

        void mixBlockBashDataContent(std::uint64_t& key, const RE::BGSBlockBashData* blockBashData)
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

        void mixObjectInstanceExtraContent(std::uint64_t& key, const RE::BGSObjectInstanceExtra* extra)
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

        ObjectInstanceExtraWitness makeObjectInstanceExtraWitness(const RE::BGSObjectInstanceExtra* extra)
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

        const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
            const F4SEVR::PlayerCharacter* player,
            const F4SEVR::TESForm* weaponForm,
            const RE::TBO_InstanceData* instanceData)
        {
            if (!player || !weaponForm) {
                return nullptr;
            }

            const auto* reWeaponForm = reinterpret_cast<const RE::TESForm*>(weaponForm);
            auto scanEquipData = [&](const F4SEVR::ActorEquipData* equipData) -> const RE::BGSObjectInstanceExtra* {
                if (!equipData) {
                    return nullptr;
                }
                for (std::uint32_t slotIndex = 0; slotIndex < F4SEVR::ActorEquipData::kMaxSlots; ++slotIndex) {
                    const auto& slot = equipData->slots[slotIndex];
                    if (slot.item != reWeaponForm) {
                        continue;
                    }
                    if (instanceData && slot.instanceData && slot.instanceData != instanceData) {
                        continue;
                    }
                    if (slot.extraData) {
                        return slot.extraData;
                    }
                }
                return nullptr;
            };

            if (const auto* firstPersonExtra = scanEquipData(player->playerEquipData)) {
                return firstPersonExtra;
            }
            return scanEquipData(player->equipData);
        }

        const RE::TESObjectWEAP* asEquippedWeaponForm(const F4SEVR::TESForm* form)
        {
            if (!form || form->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return nullptr;
            }

            const auto* reForm = reinterpret_cast<const RE::TESForm*>(form);
            return reForm->As<RE::TESObjectWEAP>();
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

        constexpr WeaponKeywordFormEntry kWeaponKeywordForms[] = {
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

        const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)>& resolvedWeaponKeywordForms()
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

        std::uint64_t computeWeaponKeywordFlags(const RE::TESObjectWEAP* weapon)
        {
            std::uint64_t flags = 0;
            if (!weapon) {
                return flags;
            }
            for (const auto& entry : resolvedWeaponKeywordForms()) {
                if (entry.keyword && weapon->HasKeyword(entry.keyword)) {
                    flags |= static_cast<std::uint64_t>(entry.flag);
                }
            }
            return flags;
        }

        struct WeaponClassificationResult
        {
            WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
            WeaponClassificationSource source{ WeaponClassificationSource::Default };
            std::uint64_t keywordFlags{ 0 };
        };

        /*
         * Keyword-primary, weight-fallback: vanilla Fallout4.esm tags every
         * sampled weapon with exactly one (occasionally two, e.g. CombatShotgun
         * carries both Rifle and Shotgun) bucket keyword, but tagging on
         * player-installed weapon mods is author-discretion and unreliable
         * (verified directly: of two installed Glock pistol mods, one tags every
         * weapon with WeaponTypePistol, the other tags none). So a bucket
         * keyword is trusted when present; when absent, this falls back to the
         * existing weight heuristic rather than defaulting blindly.
         */
        WeaponClassificationResult classifyEquippedWeapon(const RE::TESObjectWEAP* weapon, float weightGame)
        {
            WeaponClassificationResult result{};
            if (!weapon) {
                return result;
            }

            result.keywordFlags = computeWeaponKeywordFlags(weapon);
            const auto has = [&](WeaponKeywordFlag flag) { return hasWeaponKeywordFlag(result.keywordFlags, flag); };

            if (has(WeaponKeywordFlag::Melee1H) || has(WeaponKeywordFlag::Melee2H) ||
                has(WeaponKeywordFlag::Unarmed) || has(WeaponKeywordFlag::HandToHand)) {
                result.sizeClass = WeaponSizeClass::Melee;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::HeavyGun)) {
                result.sizeClass = WeaponSizeClass::Heavy;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::Pistol)) {
                result.sizeClass = WeaponSizeClass::Pistol;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::Rifle) || has(WeaponKeywordFlag::Shotgun) ||
                has(WeaponKeywordFlag::AssaultRifle) || has(WeaponKeywordFlag::Sniper) ||
                has(WeaponKeywordFlag::GaussRifle) || has(WeaponKeywordFlag::LaserMusket)) {
                result.sizeClass = WeaponSizeClass::Rifle;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }

            if (weapon->IsMeleeWeapon()) {
                result.sizeClass = WeaponSizeClass::Melee;
                result.source = WeaponClassificationSource::WeightFallback;
                return result;
            }
            result.source = WeaponClassificationSource::WeightFallback;
            if (weightGame <= g_rockConfig.rockWeaponSizeClassPistolMaxWeight) {
                result.sizeClass = WeaponSizeClass::Pistol;
            } else if (weightGame <= g_rockConfig.rockWeaponSizeClassRifleMaxWeight) {
                result.sizeClass = WeaponSizeClass::Rifle;
            } else {
                result.sizeClass = WeaponSizeClass::Heavy;
            }
            return result;
        }

        float resolveMaxGeneratedSourceDistanceGame(WeaponSizeClass sizeClass)
        {
            if (!g_rockConfig.rockWeaponCollisionMaxSourceDistanceEnabled) {
                return 0.0f;
            }
            switch (sizeClass) {
            case WeaponSizeClass::Melee:
                return g_rockConfig.rockWeaponCollisionMaxSourceDistanceMelee;
            case WeaponSizeClass::Pistol:
                return g_rockConfig.rockWeaponCollisionMaxSourceDistancePistol;
            case WeaponSizeClass::Heavy:
                return g_rockConfig.rockWeaponCollisionMaxSourceDistanceHeavy;
            case WeaponSizeClass::Rifle:
            default:
                return g_rockConfig.rockWeaponCollisionMaxSourceDistanceRifle;
            }
        }

        std::uint64_t makeEquippedWeaponInstanceContentKey(
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

        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity readEquippedWeaponGenerationIdentity()
        {
            weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};

            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (!weaponForm || weaponForm->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return identity;
            }

            identity.hasEquippedWeapon = true;
            identity.formID = weaponForm->formID;
            identity.formAddress = reinterpret_cast<std::uintptr_t>(weaponForm);
            identity.instanceDataAddress = reinterpret_cast<std::uintptr_t>(equipData->instanceData);
            identity.instanceKeywordDataAddress = reinterpret_cast<std::uintptr_t>(
                equipData->instanceData ? equipData->instanceData->GetKeywordData() : nullptr);
            identity.equippedDataAddress = reinterpret_cast<std::uintptr_t>(equipData->equippedData);
            identity.equippedObjectAddress = reinterpret_cast<std::uintptr_t>(equipData->equippedData ? equipData->equippedData->object : nullptr);
            const auto* objectInstanceExtra = findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData);
            const auto objectInstanceWitness = makeObjectInstanceExtraWitness(objectInstanceExtra);
            identity.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(objectInstanceExtra);
            identity.objectIndexDataSignature = objectInstanceWitness.signature;
            identity.objectIndexDataCount = objectInstanceWitness.count;
            identity.activeModCount = objectInstanceWitness.activeCount;
            identity.disabledModCount = objectInstanceWitness.disabledCount;
            if (const auto* weapon = asEquippedWeaponForm(weaponForm)) {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(weapon, equipData->instanceData, objectInstanceExtra);
                float weightGame = equipData->instanceData ? equipData->instanceData->GetWeight() : -1.0f;
                if (weightGame < 0.0f) {
                    weightGame = weapon->weaponData.weight;
                }
                const auto classification = classifyEquippedWeapon(weapon, weightGame);
                identity.sizeClass = classification.sizeClass;
                identity.classificationSource = classification.source;
                identity.keywordFlags = classification.keywordFlags;
            } else {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(nullptr, equipData->instanceData, objectInstanceExtra);
            }
            if (const char* fullName = weaponForm->GetFullName()) {
                identity.displayName = fullName;
            }
            return identity;
        }

        GeneratedPointCloudClusterSet splitGeneratedWeaponPointCloudForCollision(const std::vector<RE::NiPoint3>& localPoints)
        {
            GeneratedPointCloudClusterSet result{};
            const auto targetPoints = static_cast<std::size_t>((std::max)(4, g_rockConfig.rockWeaponCollisionSupportFitTargetPoints));
            const auto fit = weapon_collision_geometry_math::fitConvexSupportPointCloud(
                localPoints,
                targetPoints,
                MAX_CONVEX_HULL_POINTS,
                g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits);
            result.supportFitAttempted = fit.attempted;
            result.supportFitAccepted = fit.accepted;
            result.supportFitMaxError = fit.maxSupportError;
            result.supportFitInputPoints = fit.inputPointCount;
            result.supportFitOutputPoints = fit.selectedPointCount;
            result.supportFitRepairPoints = fit.repairPointCount;
            result.supportFitValidationDirections = fit.validationDirectionCount;

            if (fit.accepted && !fit.points.empty()) {
                result.clusters.push_back(fit.points);
                return result;
            }

            result.supportFitFallbackSplit = true;
            std::vector<std::vector<RE::NiPoint3>> splitClusters;
            weapon_collision_geometry_math::splitOversizedCluster(localPoints, MAX_CONVEX_HULL_POINTS, splitClusters);
            result.clusters.reserve(splitClusters.size());
            for (const auto& splitCluster : splitClusters) {
                const auto childFit = weapon_collision_geometry_math::fitConvexSupportPointCloud(
                    splitCluster,
                    targetPoints,
                    MAX_CONVEX_HULL_POINTS,
                    g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits);
                if (childFit.accepted && !childFit.points.empty()) {
                    result.clusters.push_back(childFit.points);
                } else {
                    result.clusters.push_back(splitCluster);
                }
            }
            return result;
        }

        GeneratedHullCoverageInfo classifyGeneratedHullSemantic(const WeaponPartClassification& semantic)
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
            case WeaponPartKind::Handguard:
            case WeaponPartKind::Foregrip:
            case WeaponPartKind::Pump:
                return { HullCoverageBarrel, semantic.priority, semantic.cosmetic, "barrel/support" };
            case WeaponPartKind::Magazine:
            case WeaponPartKind::Magwell:
                return { HullCoverageMagazine, semantic.priority, semantic.cosmetic, "magazine/socket" };
            case WeaponPartKind::Sight:
            case WeaponPartKind::Accessory:
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
            default:
                return { HullCoverageOther, semantic.priority, semantic.cosmetic, "other" };
            }
        }

        GeneratedHullCoverageInfo classifyGeneratedHull(std::string_view sourceName)
        {
            return classifyGeneratedHullSemantic(classifyWeaponPartName(sourceName));
        }

        weapon_collision_geometry_math::HullSelectionInput makeHullSelectionInput(const RE::NiPoint3& localCenterGame, const RE::NiPoint3& localMinGame,
            const RE::NiPoint3& localMaxGame, std::size_t pointCount, std::string_view sourceName)
        {
            const auto coverage = classifyGeneratedHull(sourceName);
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

        void addUniqueWeaponMeshRootCandidate(std::vector<WeaponMeshRootCandidate>& candidates, RE::NiAVObject* root, const char* label)
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

        std::vector<WeaponMeshRootCandidate> makeGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode)
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
        std::vector<RE::NiPoint3> makeCenteredHavokPointCloud(const std::vector<RE::NiPoint3>& localPointsGame, const RE::NiPoint3& localCenterGame, float sourceScale = 1.0f)
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

        const char* safeNodeName(RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }
            const char* name = node->name.c_str();
            return name ? name : "(null)";
        }

        bool weaponVisualNodeVisible(const RE::NiAVObject* node)
        {
            if (!node) {
                return false;
            }
            return (node->flags.flags & 1) == 0 && !node->GetAppCulled() && node->local.scale != 0.0f;
        }

        std::uintptr_t readRendererChildPointer(void* rendererData, std::ptrdiff_t rendererChildOffset)
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

        weapon_visual_composition_policy::VisualRecord makeWeaponVisualRecord(
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

        void accumulateWeaponVisualKey(RE::NiAVObject* node, RE::NiAVObject* parent, std::uint32_t childIndex, int depth, std::uint64_t& key, WeaponVisualKeyStats& stats)
        {
            if (!node || depth > 15 || stats.nodeCount > 512) {
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
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                auto* kid = kids[i].get();
                accumulateWeaponVisualKey(kid, node, i, depth + 1, key, stats);
            }
        }

    }

    static void shapeRemoveRef(const RE::hknpShape* shape)
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

    std::uint32_t generatedWeaponCollisionFilterInfo(bool collisionEnabled)
    {
        const std::uint32_t baseFilterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
        return collisionEnabled ? baseFilterInfo : (baseFilterInfo | collision_suppression_registry::kSuppressionNoCollideBit);
    }

    WeaponCollision::WeaponCollision() { clearAtomicBodyIds(); }

    WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies()
    {
        return _usingReplacementWeaponBodies ? _weaponReplacementBodies : _weaponBodies;
    }

    const WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies() const
    {
        return _usingReplacementWeaponBodies ? _weaponReplacementBodies : _weaponBodies;
    }

    WeaponCollision::WeaponBodyBank& WeaponCollision::inactiveWeaponBodies()
    {
        return _usingReplacementWeaponBodies ? _weaponBodies : _weaponReplacementBodies;
    }

    bool WeaponCollision::bankHasWeaponBody(const WeaponBodyBank& bank)
    {
        return std::any_of(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        });
    }

    std::uint32_t WeaponCollision::bankWeaponBodyCount(const WeaponBodyBank& bank)
    {
        return static_cast<std::uint32_t>(std::count_if(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        }));
    }

    RE::NiAVObject* WeaponCollision::resolvePackageDriveNode(const WeaponBodyBank& bank, RE::NiAVObject* fallbackWeaponNode)
    {
        if (fallbackWeaponNode) {
            return fallbackWeaponNode;
        }

        for (const auto& instance : bank) {
            if (instance.body.isValid() && instance.driveNode) {
                return instance.driveNode;
            }
        }
        return nullptr;
    }

    weapon_generated_source_completeness_policy::GeneratedSourceCompleteness WeaponCollision::summarizeGeneratedSources(const std::vector<GeneratedHullSource>& sources)
    {
        using namespace weapon_generated_source_completeness_policy;

        GeneratedSourceCompleteness summary{};
        if (sources.empty()) {
            return summary;
        }

        std::uint64_t signature = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        std::uint64_t geometryHash = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        std::uint64_t durableGeometryHash = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(signature, "ROCKGeneratedWeaponSourcesV1");
        weapon_visual_composition_policy::mixString(geometryHash, "ROCKGeneratedWeaponGeometryV1");
        weapon_visual_composition_policy::mixString(durableGeometryHash, "ROCKGeneratedWeaponDurableGeometryV1");
        weapon_visual_composition_policy::mixValue(signature, sources.size());
        weapon_visual_composition_policy::mixValue(geometryHash, sources.size());
        bool hasDurableGeometry = false;

        auto quantizedCoordinate = [](float value, float scale) {
            if (!std::isfinite(value)) {
                return std::int64_t{ 0 };
            }
            return static_cast<std::int64_t>(std::llround(value * scale));
        };
        auto mixQuantizedPoint = [&](std::uint64_t& key, const RE::NiPoint3& point, float scale) {
            weapon_visual_composition_policy::mixValue(key, static_cast<std::uint64_t>(quantizedCoordinate(point.x, scale)));
            weapon_visual_composition_policy::mixValue(key, static_cast<std::uint64_t>(quantizedCoordinate(point.y, scale)));
            weapon_visual_composition_policy::mixValue(key, static_cast<std::uint64_t>(quantizedCoordinate(point.z, scale)));
        };
        auto extentScoreForBounds = [&](const RE::NiPoint3& minPoint, const RE::NiPoint3& maxPoint) {
            const float dx = (std::max)(0.0f, maxPoint.x - minPoint.x);
            const float dy = (std::max)(0.0f, maxPoint.y - minPoint.y);
            const float dz = (std::max)(0.0f, maxPoint.z - minPoint.z);
            return static_cast<std::uint64_t>(std::llround((dx + dy + dz) * 100.0f));
        };
        /*
         * The source-set signature is intentionally structural. Runtime logs
         * showed skinned weapon extraction changes point counts and local bounds
         * from frame to frame even when the authored part set is the same; using
         * that volatile geometry as the pending-create settle key forces ROCK to
         * rescan expensive firearm meshes every frame. Geometry is still tracked
         * separately for body-set evidence and late enrichment decisions, but it
         * must not be the identity boundary that gates creation.
         */
        constexpr float kGeometryHashQuantizationScale = 10.0f;
        constexpr std::size_t kGeometryPointSampleStride = 16;

        summary.sourceCount = sources.size();
        for (const auto& source : sources) {
            weapon_visual_composition_policy::mixString(signature, source.sourceName);
            weapon_visual_composition_policy::mixValue(signature, reinterpret_cast<std::uintptr_t>(source.driveRoot));
            weapon_visual_composition_policy::mixValue(signature, reinterpret_cast<std::uintptr_t>(source.sourceRoot));
            weapon_visual_composition_policy::mixValue(signature, source.sourceGroupId);
            weapon_visual_composition_policy::mixValue(signature, static_cast<std::uint32_t>(source.semantic.partKind));
            weapon_visual_composition_policy::mixValue(signature, static_cast<std::uint32_t>(source.semantic.reloadRole));
            weapon_visual_composition_policy::mixValue(signature, static_cast<std::uint32_t>(source.semantic.supportGripRole));
            weapon_visual_composition_policy::mixValue(signature, static_cast<std::uint32_t>(source.semantic.socketRole));
            weapon_visual_composition_policy::mixValue(signature, static_cast<std::uint32_t>(source.semantic.actionRole));
            weapon_visual_composition_policy::mixValue(signature, source.childLocalPointCloudsGame.size());
            mixQuantizedPoint(geometryHash, source.localCenterGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMinGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMaxGame, kGeometryHashQuantizationScale);
            summary.boundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);

            summary.pointCount += source.localPointsGame.size();
            summary.childClusterCount += source.childLocalPointCloudsGame.size();
            summary.semanticPartMask |= partMask(source.semantic.partKind);
            const bool transientReloadSource = isTransientReloadPart(source.semantic.partKind);
            if (transientReloadSource) {
                ++summary.transientReloadSourceCount;
            } else {
                hasDurableGeometry = true;
                ++summary.durableSourceCount;
                summary.durableChildClusterCount += source.childLocalPointCloudsGame.size();
                summary.durablePointCount += source.localPointsGame.size();
                summary.durableBoundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);
                weapon_visual_composition_policy::mixString(durableGeometryHash, source.sourceName);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.driveRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.sourceRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, source.sourceGroupId);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, static_cast<std::uint32_t>(source.semantic.partKind));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, source.childLocalPointCloudsGame.size());
                mixQuantizedPoint(durableGeometryHash, source.localCenterGame, kGeometryHashQuantizationScale);
                mixQuantizedPoint(durableGeometryHash, source.localMinGame, kGeometryHashQuantizationScale);
                mixQuantizedPoint(durableGeometryHash, source.localMaxGame, kGeometryHashQuantizationScale);
            }
            if (source.semantic.gameplayCritical &&
                (partMask(source.semantic.partKind) & permanentGameplayCriticalPartMask()) != 0) {
                ++summary.gameplayCriticalCount;
            }

            for (std::size_t i = 0; i < source.localPointsGame.size(); i += kGeometryPointSampleStride) {
                mixQuantizedPoint(geometryHash, source.localPointsGame[i], kGeometryHashQuantizationScale);
                if (!transientReloadSource) {
                    mixQuantizedPoint(durableGeometryHash, source.localPointsGame[i], kGeometryHashQuantizationScale);
                }
            }
            if (!source.localPointsGame.empty()) {
                mixQuantizedPoint(geometryHash, source.localPointsGame.back(), kGeometryHashQuantizationScale);
                if (!transientReloadSource) {
                    mixQuantizedPoint(durableGeometryHash, source.localPointsGame.back(), kGeometryHashQuantizationScale);
                }
            }
            for (const auto& child : source.childLocalPointCloudsGame) {
                weapon_visual_composition_policy::mixValue(geometryHash, child.size());
                const auto childBounds = pointCloudBounds(child);
                mixQuantizedPoint(geometryHash, childBounds.min, kGeometryHashQuantizationScale);
                mixQuantizedPoint(geometryHash, childBounds.max, kGeometryHashQuantizationScale);
                summary.boundsExtentScore += extentScoreForBounds(childBounds.min, childBounds.max);
                if (!transientReloadSource) {
                    weapon_visual_composition_policy::mixValue(durableGeometryHash, child.size());
                    mixQuantizedPoint(durableGeometryHash, childBounds.min, kGeometryHashQuantizationScale);
                    mixQuantizedPoint(durableGeometryHash, childBounds.max, kGeometryHashQuantizationScale);
                    summary.durableBoundsExtentScore += extentScoreForBounds(childBounds.min, childBounds.max);
                }
                for (std::size_t i = 0; i < child.size(); i += kGeometryPointSampleStride) {
                    mixQuantizedPoint(geometryHash, child[i], kGeometryHashQuantizationScale);
                    if (!transientReloadSource) {
                        mixQuantizedPoint(durableGeometryHash, child[i], kGeometryHashQuantizationScale);
                    }
                }
                if (!child.empty()) {
                    mixQuantizedPoint(geometryHash, child.back(), kGeometryHashQuantizationScale);
                    if (!transientReloadSource) {
                        mixQuantizedPoint(durableGeometryHash, child.back(), kGeometryHashQuantizationScale);
                    }
                }
            }
        }

        summary.signature = signature;
        summary.geometryHash = geometryHash;
        summary.durableGeometryHash = hasDurableGeometry ? durableGeometryHash : 0;
        return withDerivedPackageCoverage(summary);
    }

    void WeaponCollision::clearGeneratedSourceCompletenessTracking()
    {
        _cachedGeneratedSourceCompleteness = {};
    }

    void WeaponCollision::clearPendingWeaponVisualRebuild()
    {
        _pendingWeaponVisualRebuildKey = 0;
        _pendingWeaponVisualWitnessKey = 0;
        _pendingWeaponVisualVisibleTriShapeCount = 0;
        _pendingWeaponVisualStableFrames = 0;
    }

    void WeaponCollision::clearGeneratedSourceCache()
    {
        _generatedSourceCache = {};
    }

    void WeaponCollision::resetVisualSourceUnavailableRetention()
    {
        _visualSourceUnavailableRetainIdentityKey = 0;
        _visualSourceUnavailableRetainRoot = 0;
        _visualSourceUnavailableRetainFrames = 0;
    }

    bool WeaponCollision::canRetainCurrentWeaponBodiesForVisualSourceMiss(
        std::uint64_t observedIdentityKey,
        RE::NiAVObject* currentWeaponRoot,
        int retainFrameLimit)
    {
        if (observedIdentityKey == 0 || !currentWeaponRoot) {
            resetVisualSourceUnavailableRetention();
            return false;
        }

        retainFrameLimit = (std::max)(1, retainFrameLimit);
        const auto currentRoot = reinterpret_cast<std::uintptr_t>(currentWeaponRoot);
        if (_visualSourceUnavailableRetainIdentityKey != observedIdentityKey ||
            _visualSourceUnavailableRetainRoot != currentRoot) {
            _visualSourceUnavailableRetainIdentityKey = observedIdentityKey;
            _visualSourceUnavailableRetainRoot = currentRoot;
            _visualSourceUnavailableRetainFrames = 0;
        }

        if (_visualSourceUnavailableRetainFrames >= retainFrameLimit) {
            return false;
        }

        ++_visualSourceUnavailableRetainFrames;
        return true;
    }

    bool WeaponCollision::generatedSourceCacheMatches(std::uint64_t equippedKey, std::uint64_t visualKey) const
    {
        return _generatedSourceCache.valid &&
               _generatedSourceCache.equippedKey == equippedKey &&
               _generatedSourceCache.visualKey == visualKey &&
               std::abs(_generatedSourceCache.convexRadius - g_rockConfig.rockWeaponCollisionConvexRadius) <= 0.00001f &&
               std::abs(_generatedSourceCache.pointDedupGrid - g_rockConfig.rockWeaponCollisionPointDedupGrid) <= 0.00001f &&
               _generatedSourceCache.supportFitTargetPoints == g_rockConfig.rockWeaponCollisionSupportFitTargetPoints &&
               std::abs(_generatedSourceCache.supportFitMaxErrorGameUnits - g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits) <= 0.00001f &&
               !_generatedSourceCache.sources.empty() &&
               _generatedSourceCache.summary.signature != 0;
    }

    void WeaponCollision::storeGeneratedSourceCache(std::uint64_t equippedKey,
        std::uint64_t visualKey,
        std::vector<GeneratedHullSource> sources,
        const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary)
    {
        if (equippedKey == 0 || visualKey == 0 || sources.empty() || summary.signature == 0) {
            clearGeneratedSourceCache();
            return;
        }

        _generatedSourceCache.valid = true;
        _generatedSourceCache.equippedKey = equippedKey;
        _generatedSourceCache.visualKey = visualKey;
        _generatedSourceCache.convexRadius = g_rockConfig.rockWeaponCollisionConvexRadius;
        _generatedSourceCache.pointDedupGrid = g_rockConfig.rockWeaponCollisionPointDedupGrid;
        _generatedSourceCache.supportFitTargetPoints = g_rockConfig.rockWeaponCollisionSupportFitTargetPoints;
        _generatedSourceCache.supportFitMaxErrorGameUnits = g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits;
        _generatedSourceCache.sources = std::move(sources);
        _generatedSourceCache.summary = summary;
    }

    void WeaponCollision::clearPendingGeneratedWeaponBuild(RE::hknpWorld* world, bool destroyTargetBank)
    {
        if (_pendingGeneratedWeaponBuild.active && destroyTargetBank) {
            destroyWeaponBodyBank(_pendingGeneratedWeaponBuild.replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies(), true);
        }
        _pendingGeneratedWeaponBuild = {};
        (void)world;
    }

    bool WeaponCollision::beginPendingGeneratedWeaponBuild(std::uint64_t equippedKey,
        std::uint64_t visualKey,
        std::uint64_t identityKey,
        const WeaponVisualKeyStats& visualKeyStats,
        bool replacingExisting,
        bool settingsChanged,
        bool driveRequestedRebuild,
        std::vector<GeneratedHullSource> sources,
        const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary)
    {
        if (equippedKey == 0 || sources.empty() || summary.signature == 0) {
            return false;
        }

        _pendingGeneratedWeaponBuild = {};
        _pendingGeneratedWeaponBuild.active = true;
        _pendingGeneratedWeaponBuild.replacingExisting = replacingExisting;
        _pendingGeneratedWeaponBuild.settingsChanged = settingsChanged;
        _pendingGeneratedWeaponBuild.driveRequestedRebuild = driveRequestedRebuild;
        _pendingGeneratedWeaponBuild.equippedKey = equippedKey;
        _pendingGeneratedWeaponBuild.visualKey = visualKey;
        _pendingGeneratedWeaponBuild.identityKey = identityKey;
        _pendingGeneratedWeaponBuild.visualRootCount = visualKeyStats.rootCount;
        _pendingGeneratedWeaponBuild.visibleTriShapeCount = visualKeyStats.visibleTriShapeCount;
        _pendingGeneratedWeaponBuild.convexRadius = g_rockConfig.rockWeaponCollisionConvexRadius;
        _pendingGeneratedWeaponBuild.pointDedupGrid = g_rockConfig.rockWeaponCollisionPointDedupGrid;
        _pendingGeneratedWeaponBuild.supportFitTargetPoints = g_rockConfig.rockWeaponCollisionSupportFitTargetPoints;
        _pendingGeneratedWeaponBuild.supportFitMaxErrorGameUnits = g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits;
        _pendingGeneratedWeaponBuild.sources = std::move(sources);
        _pendingGeneratedWeaponBuild.summary = summary;
        return true;
    }

    bool WeaponCollision::pendingGeneratedWeaponBuildMatches(std::uint64_t equippedKey) const
    {
        return _pendingGeneratedWeaponBuild.active &&
               _pendingGeneratedWeaponBuild.equippedKey == equippedKey &&
               std::abs(_pendingGeneratedWeaponBuild.convexRadius - g_rockConfig.rockWeaponCollisionConvexRadius) <= 0.00001f &&
               std::abs(_pendingGeneratedWeaponBuild.pointDedupGrid - g_rockConfig.rockWeaponCollisionPointDedupGrid) <= 0.00001f &&
               _pendingGeneratedWeaponBuild.supportFitTargetPoints == g_rockConfig.rockWeaponCollisionSupportFitTargetPoints &&
               std::abs(_pendingGeneratedWeaponBuild.supportFitMaxErrorGameUnits - g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits) <= 0.00001f;
    }

    bool WeaponCollision::advancePendingGeneratedWeaponBuild(RE::hknpWorld* world)
    {
        if (!_pendingGeneratedWeaponBuild.active) {
            return false;
        }
        if (!world || !_cachedBhkWorld) {
            clearPendingGeneratedWeaponBuild(world, true);
            return false;
        }

        auto& pending = _pendingGeneratedWeaponBuild;
        auto& targetBank = pending.replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies();
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponColliderCreate);
            pending.createdCount += createGeneratedWeaponBodiesInBankSlice(
                world,
                pending.sources,
                targetBank,
                GeneratedWeaponBodyCreateOptions{ .collisionEnabledOnCreate = false },
                pending.nextSourceIndex,
                GENERATED_WEAPON_BODY_CREATION_BATCH);
        }

        if (pending.nextSourceIndex < pending.sources.size()) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon collision staged create pending key={:016X} created={} nextSource={}/{} batch={}",
                pending.equippedKey,
                pending.createdCount,
                pending.nextSourceIndex,
                pending.sources.size(),
                GENERATED_WEAPON_BODY_CREATION_BATCH);
            return false;
        }

        if (pending.createdCount == 0) {
            ROCK_LOG_WARN(Weapon,
                "Generated weapon staged creation failed - no bodies created key={:016X} sources={}",
                pending.equippedKey,
                pending.sources.size());
            const bool replacingExisting = pending.replacingExisting;
            clearPendingGeneratedWeaponBuild(world, true);
            if (!replacingExisting) {
                _cachedWeaponKey = 0;
                _cachedWeaponVisualKey = 0;
                _cachedWeaponIdentityKey = 0;
                clearGeneratedSourceCompletenessTracking();
                clearPendingWeaponVisualRebuild();
                clearAtomicBodyIds();
                resetWeaponBodySetGeneration();
            }
            return false;
        }

        const auto equippedKey = pending.equippedKey;
        const auto sourceCount = pending.sources.size();
        const auto createdCount = pending.createdCount;
        const auto visualRootCount = pending.visualRootCount;
        const auto visibleTriShapeCount = pending.visibleTriShapeCount;
        const bool replacingExisting = pending.replacingExisting;
        const bool settingsChanged = pending.settingsChanged;
        const bool driveRequestedRebuild = pending.driveRequestedRebuild;
        const auto summary = pending.summary;

        if (replacingExisting) {
            ROCK_LOG_INFO(Weapon,
                "Replacing generated weapon collision bodies cachedKey={:016X} observedKey={:016X} sources={} replacementBodies={} settingsChanged={} driveRebuild={} staged=yes",
                _cachedWeaponKey,
                equippedKey,
                sourceCount,
                createdCount,
                settingsChanged,
                driveRequestedRebuild);
            clearAtomicBodyIds();
            destroyWeaponBodyBank(activeWeaponBodies(), true);
            _usingReplacementWeaponBodies = !_usingReplacementWeaponBodies;
        } else {
            ROCK_LOG_INFO(Weapon,
                "Created generated weapon collision bodies key={:016X} sources={} bodies={} visualRoots={} visibleTriShapes={} staged=yes",
                equippedKey,
                sourceCount,
                createdCount,
                visualRootCount,
                visibleTriShapeCount);
        }

        const auto finalBodyCount = static_cast<std::uint64_t>(bankWeaponBodyCount(activeWeaponBodies()));

        _cachedWeaponKey = equippedKey;
        _cachedWeaponVisualKey = pending.visualKey;
        _cachedWeaponIdentityKey = pending.identityKey;
        _cachedGeneratedSourceCompleteness = summary;
        clearPendingWeaponVisualRebuild();
        publishWeaponBodySetGeneration(summary);
        publishAtomicBodyIds(activeWeaponBodies());
        setWeaponBodyBankCollisionEnabled(world, activeWeaponBodies(), true);
        _cachedConvexRadius = g_rockConfig.rockWeaponCollisionConvexRadius;
        _cachedPointDedupGrid = g_rockConfig.rockWeaponCollisionPointDedupGrid;
        _cachedSupportFitTargetPoints = g_rockConfig.rockWeaponCollisionSupportFitTargetPoints;
        _cachedSupportFitMaxErrorGameUnits = g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildCompleted);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildVisibleTriShapes, visibleTriShapeCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildGeneratedSources, sourceCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildBodiesCreated, createdCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildTransientReloadSources, summary.transientReloadSourceCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildBodyCount, finalBodyCount);
        _pendingGeneratedWeaponBuild = {};
        return true;
    }

    void WeaponCollision::resetWeaponCollisionSettingsCache()
    {
        _cachedConvexRadius = -1.0f;
        _cachedPointDedupGrid = -1.0f;
        _cachedSupportFitTargetPoints = -1;
        _cachedSupportFitMaxErrorGameUnits = -1.0f;
    }

    void WeaponCollision::resetWeaponBodySetGeneration()
    {
        _cachedWeaponBodySetKey = 0;
        _weaponBodySetKeyAtomic.store(0, std::memory_order_release);
    }

    void WeaponCollision::publishWeaponBodySetGeneration(const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& sourceCompleteness)
    {
        if (_weaponBodySetEpoch == (std::numeric_limits<std::uint64_t>::max)()) {
            _weaponBodySetEpoch = 1;
        } else {
            ++_weaponBodySetEpoch;
        }
        _cachedWeaponBodySetKey = weapon_generated_source_completeness_policy::makeGeneratedWeaponBodySetKey(
            _cachedWeaponKey,
            sourceCompleteness,
            _weaponBodySetEpoch);
    }

    bool WeaponCollision::hasWeaponBody() const
    {
        return bankHasWeaponBody(activeWeaponBodies());
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        return _weaponBodyCountAtomic.load(std::memory_order_acquire);
    }

    RE::hknpBodyId WeaponCollision::getWeaponBodyId() const
    {
        for (const auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body.getBodyId();
            }
        }
        return RE::hknpBodyId{ INVALID_BODY_ID };
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic() const
    {
        return getWeaponBodyIdAtomic(0);
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic(std::size_t index) const
    {
        const auto snapshot = getWeaponBodySnapshotAtomic();
        if (index >= snapshot.count || index >= MAX_WEAPON_BODIES) {
            return INVALID_BODY_ID;
        }
        return snapshot.bodyIds[index];
    }

    WeaponCollision::WeaponBodySnapshot WeaponCollision::getWeaponBodySnapshotAtomic() const
    {
        WeaponBodySnapshot snapshot{};
        snapshot.bodyIds.fill(INVALID_BODY_ID);

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            WeaponBodySnapshot candidate{};
            candidate.bodyIds.fill(INVALID_BODY_ID);
            candidate.generationKey = _weaponBodySetKeyAtomic.load(std::memory_order_acquire);
            candidate.count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < candidate.count; ++i) {
                candidate.bodyIds[i] = _weaponBodyIdsAtomic[i].load(std::memory_order_acquire);
            }

            const std::uint64_t endVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return candidate;
            }
        }

        return snapshot;
    }

    bool WeaponCollision::isWeaponBodyIdAtomic(std::uint32_t bodyId) const
    {
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const auto snapshot = getWeaponBodySnapshotAtomic();
        for (std::uint32_t i = 0; i < snapshot.count && i < MAX_WEAPON_BODIES; ++i) {
            if (snapshot.bodyIds[i] == bodyId) {
                return true;
            }
        }
        return false;
    }

    bool WeaponCollision::tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            WeaponInteractionContact candidate{};
            const std::uint32_t count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            bool found = false;
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
                    continue;
                }

                candidate.valid = true;
                candidate.bodyId = bodyId;
                candidate.partKind = static_cast<WeaponPartKind>(_weaponBodyPartKindsAtomic[i].load(std::memory_order_acquire));
                candidate.reloadRole = static_cast<WeaponReloadRole>(_weaponBodyReloadRolesAtomic[i].load(std::memory_order_acquire));
                candidate.supportGripRole = static_cast<WeaponSupportGripRole>(_weaponBodySupportRolesAtomic[i].load(std::memory_order_acquire));
                candidate.socketRole = static_cast<WeaponSocketRole>(_weaponBodySocketRolesAtomic[i].load(std::memory_order_acquire));
                candidate.actionRole = static_cast<WeaponActionRole>(_weaponBodyActionRolesAtomic[i].load(std::memory_order_acquire));
                candidate.fallbackGripPose = static_cast<WeaponGripPoseId>(_weaponBodyGripPosesAtomic[i].load(std::memory_order_acquire));
                candidate.interactionRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodyInteractionRootsAtomic[i].load(std::memory_order_acquire));
                candidate.sourceRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodySourceRootsAtomic[i].load(std::memory_order_acquire));
                candidate.weaponGenerationKey = _weaponBodyGenerationKeysAtomic[i].load(std::memory_order_acquire);
                found = true;
                break;
            }

            const std::uint64_t endVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                if (found) {
                    outContact = candidate;
                }
                return found;
            }
        }
        return false;
    }

    bool WeaponCollision::tryGetWeaponBodySampledVelocityAtomic(std::uint32_t bodyId, float* outVelocityHavok) const
    {
        if (!outVelocityHavok) {
            return false;
        }
        outVelocityHavok[0] = 0.0f;
        outVelocityHavok[1] = 0.0f;
        outVelocityHavok[2] = 0.0f;
        outVelocityHavok[3] = 0.0f;
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            float vx = 0.0f;
            float vy = 0.0f;
            float vz = 0.0f;
            bool found = false;
            const std::uint32_t count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId ||
                    _weaponBodySampledVelocityValidAtomic[i].load(std::memory_order_acquire) == 0) {
                    continue;
                }

                vx = _weaponBodySampledVelocityHavokXAtomic[i].load(std::memory_order_acquire);
                vy = _weaponBodySampledVelocityHavokYAtomic[i].load(std::memory_order_acquire);
                vz = _weaponBodySampledVelocityHavokZAtomic[i].load(std::memory_order_acquire);
                found = true;
                break;
            }

            const std::uint64_t endVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if (startVersion != endVersion || (endVersion & 1u) != 0) {
                continue;
            }
            if (!found || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
                return false;
            }

            outVelocityHavok[0] = vx;
            outVelocityHavok[1] = vy;
            outVelocityHavok[2] = vz;
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryGetWeaponContactDebugInfo(std::uint32_t bodyId, WeaponInteractionDebugInfo& outInfo) const
    {
        outInfo = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }
        if (!isWeaponBodyIdAtomic(bodyId)) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || instance.body.getBodyId().value != bodyId) {
                continue;
            }

            RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(activeWeaponBodies(), nullptr);
            outInfo.sourceName = instance.sourceName;
            outInfo.interactionRootName = packageDriveRoot ? safeNodeName(packageDriveRoot) : "";
            outInfo.sourceRootName = instance.sourceRootName;
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryBuildSupportGripEvidenceTriangles(
        std::uint32_t bodyId,
        const RE::NiAVObject* currentWeaponRoot,
        std::vector<TriangleData>& outTriangles) const
    {
        outTriangles.clear();
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || instance.body.getBodyId().value != bodyId || instance.generatedLocalTrianglesGame.empty()) {
                continue;
            }
            const bool sourceNodeCurrent = instance.sourceNode && currentWeaponRoot &&
                actor_equipment_grab::nodeContainsNode(const_cast<RE::NiAVObject*>(currentWeaponRoot), instance.sourceNode, 64);
            const RE::NiAVObject* driveRoot = sourceNodeCurrent ? instance.sourceNode : (currentWeaponRoot ? currentWeaponRoot : instance.driveNode);
            if (!driveRoot) {
                continue;
            }

            const auto& localTriangles = sourceNodeCurrent && !instance.generatedSourceLocalTrianglesGame.empty() ?
                instance.generatedSourceLocalTrianglesGame :
                instance.generatedLocalTrianglesGame;
            outTriangles.reserve(localTriangles.size());
            const RE::NiTransform driveWorld = driveRoot->world;
            for (const auto& localTriangle : localTriangles) {
                TriangleData worldTriangle = localTriangle;
                worldTriangle.applyTransform(driveWorld);
                outTriangles.push_back(worldTriangle);
            }
            return !outTriangles.empty();
        }

        return false;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::buildProfileEvidenceSnapshot(const WeaponBodyBank& bank) const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        descriptors.reserve(bankWeaponBodyCount(bank));

        /*
         * Pair slot-classified parts with the installed OMOD occupying that
         * slot: resolve the equipped instance's active mods once and index
         * them by attach-point keyword FormID. Runs once per publication on
         * the main thread; ~a dozen form lookups.
         */
        std::unordered_map<std::uint32_t, std::uint32_t> omodByAttachPointFormId;
        {
            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            const RE::BGSObjectInstanceExtra* objectInstanceExtra =
                weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData) : nullptr;
            if (objectInstanceExtra && objectInstanceExtra->values) {
                for (const auto& modIndex : objectInstanceExtra->GetIndexData()) {
                    if (modIndex.disabled) {
                        continue;
                    }
                    auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                    if (!omod) {
                        continue;
                    }
                    const RE::BGSKeyword* attachPointKeyword =
                        RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, omod->attachPoint.keywordIndex);
                    if (attachPointKeyword) {
                        omodByAttachPointFormId.emplace(attachPointKeyword->formID, omod->formID);
                    }
                }
            }
        }

        auto copyLocalPoints = [](const std::vector<RE::NiPoint3>& points) {
            std::vector<WeaponEvidencePoint3> result;
            result.reserve(points.size());
            for (const auto& point : points) {
                result.push_back(makeWeaponEvidencePoint(point.x, point.y, point.z));
            }
            return result;
        };

        RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(bank, nullptr);
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }

            RE::NiAVObject* interactionRoot = packageDriveRoot ? packageDriveRoot : instance.driveNode;
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            descriptor.valid = true;
            descriptor.bodyId = instance.body.getBodyId().value;
            descriptor.weaponGenerationKey = _cachedWeaponBodySetKey;
            descriptor.sourceRootAddress = reinterpret_cast<std::uintptr_t>(instance.sourceNode);
            descriptor.geometryRootAddress = reinterpret_cast<std::uintptr_t>(interactionRoot);
            descriptor.sourceRootName = instance.sourceRootName;
            descriptor.geometryRootName = interactionRoot ? safeNodeName(interactionRoot) : "";
            descriptor.sourceName = instance.sourceName;
            descriptor.semantic = instance.semantic;
            descriptor.localBoundsGame = WeaponEvidenceBounds3{
                .min = makeWeaponEvidencePoint(instance.generatedLocalMinGame.x, instance.generatedLocalMinGame.y, instance.generatedLocalMinGame.z),
                .max = makeWeaponEvidencePoint(instance.generatedLocalMaxGame.x, instance.generatedLocalMaxGame.y, instance.generatedLocalMaxGame.z),
                .valid = true,
            };
            descriptor.localMeshPointsGame = copyLocalPoints(instance.generatedLocalPointsGame);
            descriptor.pointCount = instance.generatedPointCount;
            if (instance.semantic.attachPointFormId != 0) {
                const auto omodIt = omodByAttachPointFormId.find(instance.semantic.attachPointFormId);
                if (omodIt != omodByAttachPointFormId.end()) {
                    descriptor.omodFormId = omodIt->second;
                }
            }
            descriptors.push_back(std::move(descriptor));
        }

        return descriptors;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::getProfileEvidenceDescriptors() const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
            {
                std::scoped_lock lock(_profileEvidenceSnapshotMutex);
                descriptors = _profileEvidenceSnapshot;
            }

            const std::uint64_t endVersion = _weaponBodyPublicationVersion.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return descriptors;
            }
        }

        return {};
    }

    bool WeaponCollision::tryGetProfileEvidenceDescriptorForBodyId(
        std::uint32_t bodyId,
        WeaponCollisionProfileEvidenceDescriptor& outDescriptor,
        RE::NiAVObject*& outSourceNode) const
    {
        outDescriptor = {};
        outSourceNode = nullptr;
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const auto descriptors = getProfileEvidenceDescriptors();
        for (const auto& descriptor : descriptors) {
            if (!descriptor.valid || descriptor.bodyId != bodyId) {
                continue;
            }

            outDescriptor = descriptor;
            outSourceNode = reinterpret_cast<RE::NiAVObject*>(descriptor.sourceRootAddress);
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryFindInteractionContactNearPoint(
        const RE::NiAVObject* weaponNode,
        const RE::NiPoint3& probeWorldPoint,
        float probeRadiusGame,
        WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (!weaponNode || getCurrentWeaponGenerationKey() == 0 || probeRadiusGame <= 0.0f) {
            return false;
        }

        weapon_interaction_probe_math::ProbeCandidateRank bestRank{};
        const WeaponBodyInstance* bestInstance = nullptr;
        int candidateCount = 0;
        const RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(activeWeaponBodies(), const_cast<RE::NiAVObject*>(weaponNode));
        if (!packageDriveRoot) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid()) {
                continue;
            }

            const bool sourceNodeCurrent = instance.sourceNode &&
                actor_equipment_grab::nodeContainsNode(const_cast<RE::NiAVObject*>(packageDriveRoot), instance.sourceNode, 64);
            const RE::NiAVObject* probeRoot = sourceNodeCurrent ? instance.sourceNode : packageDriveRoot;
            const RE::NiPoint3 probeLocal = weapon_collision_geometry_math::worldPointToLocal(
                probeRoot->world.rotate,
                probeRoot->world.translate,
                probeRoot->world.scale,
                probeWorldPoint);
            const RE::NiPoint3& boundsMin = sourceNodeCurrent ? instance.generatedSourceLocalMinGame : instance.generatedLocalMinGame;
            const RE::NiPoint3& boundsMax = sourceNodeCurrent ? instance.generatedSourceLocalMaxGame : instance.generatedLocalMaxGame;

            const float distanceSquared = weapon_interaction_probe_math::pointAabbDistanceSquared(
                probeLocal,
                boundsMin,
                boundsMax);
            if (!weapon_interaction_probe_math::isWithinProbeRadiusSquared(distanceSquared, probeRadiusGame)) {
                continue;
            }

            ++candidateCount;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                .distanceSquaredGame = distanceSquared,
                .aabbDiagonalSquaredGame = weapon_interaction_probe_math::aabbDiagonalSquared(boundsMin, boundsMax),
                .semanticPriority = instance.semantic.priority,
            };
            if (bestInstance && !weapon_interaction_probe_math::isBetterProbeCandidate(rank, bestRank)) {
                continue;
            }

            bestRank = rank;
            bestInstance = &instance;
        }

        if (!bestInstance) {
            return false;
        }

        ROCK_LOG_SAMPLE_DEBUG(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "WeaponInteractionProbe ranked: part={} bodyId={} dist={:.2f} diag={:.1f} priority={} candidates={}",
            static_cast<int>(bestInstance->semantic.partKind),
            bestInstance->body.getBodyId().value,
            std::sqrt(bestRank.distanceSquaredGame),
            std::sqrt(bestRank.aabbDiagonalSquaredGame),
            bestInstance->semantic.priority,
            candidateCount);

        outContact.valid = true;
        outContact.bodyId = bestInstance->body.getBodyId().value;
        outContact.partKind = bestInstance->semantic.partKind;
        outContact.reloadRole = bestInstance->semantic.reloadRole;
        outContact.supportGripRole = bestInstance->semantic.supportGripRole;
        outContact.socketRole = bestInstance->semantic.socketRole;
        outContact.actionRole = bestInstance->semantic.actionRole;
        outContact.fallbackGripPose = bestInstance->semantic.fallbackGripPose;
        outContact.interactionRoot = const_cast<RE::NiAVObject*>(packageDriveRoot);
        outContact.sourceRoot = bestInstance->sourceNode;
        outContact.weaponGenerationKey = getCurrentWeaponGenerationKey();
        outContact.probeDistanceGame = std::sqrt(bestRank.distanceSquaredGame);
        return true;
    }

    BethesdaPhysicsBody& WeaponCollision::getWeaponBody()
    {
        for (auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body;
            }
        }
        return activeWeaponBodies()[0].body;
    }

    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        // Cache the Havok context even while the feature is disabled so the INI
        // watcher can hot-enable weapon collision without requiring a physics
        // module restart.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedWeaponKey = 0;
        _cachedWeaponVisualKey = 0;
        _cachedWeaponIdentityKey = 0;
        resetWeaponBodySetGeneration();
        _weaponBodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(world, false);
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        resetWeaponCollisionSettingsCache();
        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = 0;
        clearAtomicBodyIds();

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via config — context cached for hot reload");
            return;
        }

        ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
    }

    void WeaponCollision::shutdown()
    {
        if (hasWeaponBody()) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown destroying generated bodies from cached context");
            destroyWeaponBody(_cachedWorld);
        }

        _cachedWeaponKey = 0;
        _cachedWeaponVisualKey = 0;
        _cachedWeaponIdentityKey = 0;
        resetWeaponBodySetGeneration();
        _weaponBodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(_cachedWorld, true);
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        resetWeaponCollisionSettingsCache();
        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = 0;

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::requestWorkbenchExitRebuild()
    {
        /*
         * Workbench close is observed from the UI event source while weapon
         * collision is updated from the physics runtime. Keep the cross-surface
         * handoff to one atomic bit; the update path consumes it only when the
         * drawn weapon visual is available, so reload-null visuals cannot turn
         * this permission into a destroy/recreate cycle.
         */
        _workbenchExitRebuildRequested.store(true, std::memory_order_release);
    }


    void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, float dt, bool weaponDrawn)
    {
        (void)dt;

        auto clearCurrentWeaponState = [&]() {
            _cachedWeaponKey = 0;
            _cachedWeaponVisualKey = 0;
            _cachedWeaponIdentityKey = 0;
            clearGeneratedSourceCompletenessTracking();
            clearPendingWeaponVisualRebuild();
            clearGeneratedSourceCache();
            clearPendingGeneratedWeaponBuild(world, true);
            resetVisualSourceUnavailableRetention();
            resetWeaponBodySetGeneration();
            resetWeaponCollisionSettingsCache();
            _driveRebuildRequested.store(false, std::memory_order_release);
            _workbenchExitRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
        };

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            if (hasWeaponBody() && world) {
                ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via hot reload - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }

        if (!world) {
            return;
        }

        if (world != _cachedWorld) {
            ROCK_LOG_INFO(Weapon, "hknpWorld changed - resetting weapon collision state");
            if (hasWeaponBody()) {
                destroyWeaponBody(_cachedWorld ? _cachedWorld : world);
            } else {
                clearAtomicBodyIds();
            }
            _cachedWorld = world;
            clearCurrentWeaponState();
        }

        if (!weaponDrawn) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon no longer drawn - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }

        std::uint64_t observedIdentityKey = 0;
        WeaponSizeClass observedSizeClass{ WeaponSizeClass::Rifle };
        const std::uint64_t observedKey = getEquippedWeaponIdentityKey(&observedIdentityKey, &observedSizeClass);
        if (observedKey == 0) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon identity unavailable - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }

        const bool settingsChanged = weaponCollisionSettingsChanged();
        const bool driveRequestedRebuild = _driveRebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool workbenchExitRequested =
            weaponNode != nullptr && _workbenchExitRebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool keyChanged = observedKey != 0 && observedKey != _cachedWeaponKey;
        const bool missingBodies = observedKey != 0 && !hasWeaponBody();
        const bool identityKeyChanged = observedIdentityKey != 0 && observedIdentityKey != _cachedWeaponIdentityKey;
        bool rebuildRequired = driveRequestedRebuild || workbenchExitRequested || settingsChanged || keyChanged || missingBodies;
        bool rebuildDiagnosticsRecorded = false;

        const auto recordRebuildDiagnostics = [&]() {
            if (rebuildDiagnosticsRecorded) {
                return;
            }

            if (settingsChanged) {
                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildReasonSettingsChanged);
            }
            if (driveRequestedRebuild) {
                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildReasonDriveRequested);
            }
            if (missingBodies) {
                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildReasonMissingBodies);
            }
            if (keyChanged && _cachedWeaponKey != 0) {
                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildReasonKeyChanged);
                if (identityKeyChanged) {
                    performance_profiler::addCounter(performance_profiler::Counter::WeaponKeyChangeIdentityOnly);
                }
            }

            rebuildDiagnosticsRecorded = true;
        };

        maybeDumpWeaponAnimNodeDiagnostics(weaponNode, observedKey);

        if (driveRequestedRebuild) {
            ROCK_LOG_WARN(Weapon,
                "Generated weapon collision drive failure requested rebuild cachedKey={:016X} observedKey={:016X}",
                _cachedWeaponKey,
                observedKey);
        }
        if (workbenchExitRequested) {
            ROCK_LOG_INFO(Weapon,
                "Workbench exit requested generated weapon collision rebuild cachedKey={:016X} observedKey={:016X}",
                _cachedWeaponKey,
                observedKey);
        }

        if (_pendingGeneratedWeaponBuild.active) {
            const bool pendingInvalidated = driveRequestedRebuild || workbenchExitRequested || !pendingGeneratedWeaponBuildMatches(observedKey);
            if (pendingInvalidated) {
                ROCK_LOG_INFO(Weapon,
                    "Generated weapon staged create cancelled: pendingKey={:016X} observedKey={:016X} pendingVisual={:016X} driveRebuild={} workbenchExit={}",
                    _pendingGeneratedWeaponBuild.equippedKey,
                    observedKey,
                    _pendingGeneratedWeaponBuild.visualKey,
                    driveRequestedRebuild ? "yes" : "no",
                    workbenchExitRequested ? "yes" : "no");
                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildCanceled);
                clearPendingGeneratedWeaponBuild(world, true);
                rebuildRequired = true;
            } else {
                advancePendingGeneratedWeaponBuild(world);
                return;
            }
        }

        if (!weaponNode) {
            if (hasWeaponBody() && !keyChanged && !missingBodies && !settingsChanged && !driveRequestedRebuild) {
                /*
                 * Reload animation can briefly hide or detach the first-person
                 * weapon visual while the equipped identity is unchanged. Keep
                 * the existing collider set instead of turning that visual gap
                 * into a destroy/recreate cycle.
                 */
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Weapon visual node absent for unchanged equipped identity - retaining generated weapon bodies key={:016X} bodies={}",
                    _cachedWeaponKey,
                    getWeaponBodyCount());
                clearPendingWeaponVisualRebuild();
                resetVisualSourceUnavailableRetention();
                return;
            }

            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon,
                    "Weapon visual node absent while rebuild required - destroying generated weapon bodies cachedKey={:016X} observedKey={:016X} missingBodies={} settingsChanged={} driveRebuild={} identityChanged={}",
                    _cachedWeaponKey,
                    observedKey,
                    missingBodies ? "yes" : "no",
                    settingsChanged ? "yes" : "no",
                    driveRequestedRebuild ? "yes" : "no",
                    identityKeyChanged ? "yes" : "no");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }

        if (rebuildRequired) {
            WeaponVisualKeyStats visualKeyStats{};
            const std::uint64_t observedVisualKey = getWeaponVisualCompositionKey(weaponNode, visualKeyStats);
            const bool visualKeyChanged = observedVisualKey != 0 && observedVisualKey != _cachedWeaponVisualKey;
            const bool generationDrivenRebuild = keyChanged || missingBodies;
            const int requiredStableFrames = (std::max)(0, g_rockConfig.rockWeaponCollisionVisualStabilizationFrames);
            const bool stabilizeVisualRebuild = generationDrivenRebuild && requiredStableFrames > 0;

            if (stabilizeVisualRebuild && !weaponVisualNodeVisible(weaponNode)) {
                const bool newInvisibleDeferred =
                    _pendingWeaponVisualRebuildKey != observedKey ||
                    _pendingWeaponVisualWitnessKey != observedVisualKey ||
                    _pendingWeaponVisualVisibleTriShapeCount != 0 ||
                    _pendingWeaponVisualStableFrames != 0;
                /*
                 * Weapon mod swaps can expose a transient app-culled Weapon root
                 * while child TriShapes still look locally visible. Replacing the
                 * active body set from that frame can lock in an incomplete hull
                 * inventory, so keep the current bodies until the visual tree has
                 * presented a stable, visible witness.
                 */
                _pendingWeaponVisualRebuildKey = observedKey;
                _pendingWeaponVisualWitnessKey = observedVisualKey;
                _pendingWeaponVisualVisibleTriShapeCount = 0;
                _pendingWeaponVisualStableFrames = 0;
                if (newInvisibleDeferred) {
                    performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualRootDeferred);
                }
                ROCK_LOG_SAMPLE_INFO(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon collision rebuild deferred: visual root not ready cachedKey={:016X} observedKey={:016X} root='{}' flags=0x{:X} appCulled={} visibleTriShapes={} visualNodes={} invisibleNodes={} requiredStableFrames={}",
                    _cachedWeaponKey,
                    observedKey,
                    safeNodeName(weaponNode),
                    static_cast<std::uint32_t>(weaponNode->flags.flags),
                    weaponNode->GetAppCulled() ? "yes" : "no",
                    visualKeyStats.visibleTriShapeCount,
                    visualKeyStats.nodeCount,
                    visualKeyStats.invisibleNodeCount,
                    requiredStableFrames);
            } else {
                if (stabilizeVisualRebuild) {
                    /*
                     * Stabilization is a cheap visual-witness wait. Full mesh
                     * extraction and Havok shape creation happen once after the
                     * visible tree has stayed stable for the configured frames.
                     */
                    const bool samePendingVisual =
                        _pendingWeaponVisualRebuildKey == observedKey &&
                        _pendingWeaponVisualWitnessKey == observedVisualKey &&
                        _pendingWeaponVisualVisibleTriShapeCount == visualKeyStats.visibleTriShapeCount;

                    _pendingWeaponVisualRebuildKey = observedKey;
                    _pendingWeaponVisualWitnessKey = observedVisualKey;
                    _pendingWeaponVisualVisibleTriShapeCount = visualKeyStats.visibleTriShapeCount;
                    _pendingWeaponVisualStableFrames = samePendingVisual ? _pendingWeaponVisualStableFrames + 1 : 1;

                    if (_pendingWeaponVisualStableFrames < requiredStableFrames) {
                        if (!samePendingVisual) {
                            performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualStableWait);
                        }
                        ROCK_LOG_SAMPLE_INFO(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon collision rebuild waiting for stable visual witness cachedKey={:016X} observedKey={:016X} stableFrames={}/{} visualKey={:016X} visualRoots={} visibleTriShapes={} visualNodes={} invisibleNodes={}",
                            _cachedWeaponKey,
                            observedKey,
                            _pendingWeaponVisualStableFrames,
                            requiredStableFrames,
                            observedVisualKey,
                            visualKeyStats.rootCount,
                            visualKeyStats.visibleTriShapeCount,
                            visualKeyStats.nodeCount,
                            visualKeyStats.invisibleNodeCount);
                        return;
                        }
                }

                std::vector<GeneratedHullSource> generatedSources;
                weapon_generated_source_completeness_policy::GeneratedSourceCompleteness generatedSummary{};
                std::size_t generatedCount = 0;
                bool usedCachedSources = false;

                if (generatedSourceCacheMatches(observedKey, observedVisualKey)) {
                    generatedSources = _generatedSourceCache.sources;
                    generatedSummary = _generatedSourceCache.summary;
                    generatedCount = generatedSources.size();
                    usedCachedSources = true;
                    ROCK_LOG_DEBUG(Weapon,
                        "Generated weapon mesh source cache hit key={:016X} visualKey={:016X} sources={}",
                        observedKey,
                        observedVisualKey,
                        generatedCount);
                } else {
                    performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponColliderBuild);
                    const float maxGeneratedSourceDistanceGame = resolveMaxGeneratedSourceDistanceGame(observedSizeClass);
                    generatedCount = findGeneratedWeaponShapeSources(weaponNode, generatedSources, maxGeneratedSourceDistanceGame);
                    generatedSummary = summarizeGeneratedSources(generatedSources);
                }

                const bool hasBuildableSource = std::any_of(generatedSources.begin(), generatedSources.end(), [](const GeneratedHullSource& source) {
                    return pointCloudCanBuildHull(source.localPointsGame);
                });

                if (!hasBuildableSource || generatedCount == 0 || generatedSummary.signature == 0) {
                    recordRebuildDiagnostics();
                    ROCK_LOG_SAMPLE_WARN(Weapon,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Generated weapon mesh collision unavailable from current visible geometry cachedKey={:016X} observedKey={:016X} visualRoots={} visualNodes={} visibleTriShapes={} sources={} missingGeometry={} invisibleNodes={}",
                        _cachedWeaponKey,
                        observedKey,
                        visualKeyStats.rootCount,
                        visualKeyStats.nodeCount,
                        visualKeyStats.visibleTriShapeCount,
                        generatedCount,
                        visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount,
                        visualKeyStats.invisibleNodeCount);

                    const bool sameEquippedIdentity =
                        observedIdentityKey != 0 &&
                        _cachedWeaponIdentityKey != 0 &&
                        observedIdentityKey == _cachedWeaponIdentityKey &&
                        !identityKeyChanged;
                    RE::NiAVObject* retainedPackageRoot = resolvePackageDriveNode(activeWeaponBodies(), nullptr);
                    const bool retainedPackageRootStillCurrent = retainedPackageRoot && retainedPackageRoot == weaponNode;
                    const bool retainCandidate =
                        hasWeaponBody() &&
                        sameEquippedIdentity &&
                        visualKeyChanged &&
                        retainedPackageRootStillCurrent &&
                        !settingsChanged &&
                        !driveRequestedRebuild;
                    const int visualSourceMissRetainFrameLimit = (std::max)(1, requiredStableFrames);
                    if (retainCandidate &&
                        canRetainCurrentWeaponBodiesForVisualSourceMiss(observedIdentityKey, weaponNode, visualSourceMissRetainFrameLimit)) {
                        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualSourceUnavailableRetained);
                        /*
                         * The visible tree can briefly report no extractable
                         * TriShapes while the same equipped weapon identity and
                         * package root are still live. Keep the current body set
                         * only for a bounded window; actual identity/root,
                         * settings, or drive changes still fall through and
                         * destroy stale collision.
                         */
                        ROCK_LOG_SAMPLE_INFO(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon mesh collision unavailable for same equipped identity - retaining current bodies cachedKey={:016X} observedKey={:016X} visualKey={:016X} bodies={} retainFrame={}/{}",
                            _cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            getWeaponBodyCount(),
                            _visualSourceUnavailableRetainFrames,
                            visualSourceMissRetainFrameLimit);
                        clearPendingGeneratedWeaponBuild(world, true);
                        clearPendingWeaponVisualRebuild();
                        return;
                    }
                    if (retainCandidate) {
                        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualSourceUnavailableRetainExpired);
                        ROCK_LOG_SAMPLE_WARN(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon mesh collision same-identity retain window expired cachedKey={:016X} observedKey={:016X} visualKey={:016X} retainFrames={} limit={} - destroying stale bodies",
                            _cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            _visualSourceUnavailableRetainFrames,
                            visualSourceMissRetainFrameLimit);
                    } else {
                        resetVisualSourceUnavailableRetention();
                    }

                    if (hasWeaponBody()) {
                        destroyWeaponBody(world);
                    } else {
                        clearAtomicBodyIds();
                        resetWeaponBodySetGeneration();
                    }
                    _cachedWeaponKey = 0;
                    _cachedWeaponVisualKey = 0;
                    _cachedWeaponIdentityKey = 0;
                    clearGeneratedSourceCompletenessTracking();
                    clearPendingWeaponVisualRebuild();
                    clearGeneratedSourceCache();
                    resetVisualSourceUnavailableRetention();
                    clearPendingGeneratedWeaponBuild(world, true);
                    return;
                }

                resetVisualSourceUnavailableRetention();

                const bool replacingExisting = hasWeaponBody();
                auto& targetBank = replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies();
                destroyWeaponBodyBank(targetBank, true);

                if (!usedCachedSources) {
                    storeGeneratedSourceCache(observedKey, observedVisualKey, generatedSources, generatedSummary);
                }

                recordRebuildDiagnostics();

                if (!beginPendingGeneratedWeaponBuild(
                        observedKey,
                        observedVisualKey,
                        observedIdentityKey,
                        visualKeyStats,
                        replacingExisting,
                        settingsChanged,
                        driveRequestedRebuild,
                        std::move(generatedSources),
                        generatedSummary)) {
                    ROCK_LOG_WARN(Weapon,
                        "Generated weapon staged creation could not be queued cachedKey={:016X} observedKey={:016X} sources={}",
                        _cachedWeaponKey,
                        observedKey,
                        generatedCount);
                    if (!replacingExisting) {
                        clearAtomicBodyIds();
                        resetWeaponBodySetGeneration();
                        _cachedWeaponKey = 0;
                        _cachedWeaponVisualKey = 0;
                        _cachedWeaponIdentityKey = 0;
                        clearGeneratedSourceCompletenessTracking();
                    }
                    clearPendingWeaponVisualRebuild();
                    return;
                }

                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildQueued);

                ROCK_LOG_INFO(Weapon,
                    "Generated weapon collision staged create queued cachedKey={:016X} observedKey={:016X} sources={} replacingExisting={} settingsChanged={} driveRebuild={} workbenchExit={} cachedSources={} batch={}",
                    _cachedWeaponKey,
                    observedKey,
                    generatedCount,
                    replacingExisting ? "yes" : "no",
                    settingsChanged ? "yes" : "no",
                    driveRequestedRebuild ? "yes" : "no",
                    workbenchExitRequested ? "yes" : "no",
                    usedCachedSources ? "yes" : "no",
                    GENERATED_WEAPON_BODY_CREATION_BATCH);
                return;
            }
        }

        maybeRunWeaponOmodCoverageAudit(weaponNode);
    }


    void WeaponCollision::maybeDumpWeaponAnimNodeDiagnostics(RE::NiAVObject* updateWeaponNode, std::uint64_t observedKey)
    {
        if (!g_rockConfig.rockDebugDumpWeaponAnimNodes) {
            _weaponAnimNodeDumpFrameCounter = 0;
            _lastWeaponAnimNodeDumpKey = 0;
            return;
        }

        const bool generationChanged = observedKey != _lastWeaponAnimNodeDumpKey;
        const int intervalFrames = (std::max)(1, g_rockConfig.rockDebugWeaponAnimNodeDumpIntervalFrames);
        const bool intervalDue = ++_weaponAnimNodeDumpFrameCounter >= intervalFrames;
        if (!generationChanged && !intervalDue) {
            return;
        }

        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = observedKey;

        auto* firstPersonBoneTree = f4vr::getFirstPersonBoneTree();
        auto* gameFlattenedBoneTree = f4vr::getFlattenedBoneTree();
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        auto* gameRootNode = f4vr::getRootNode();
        auto* weaponNode = f4vr::getWeaponNode();
        auto* player = f4vr::getPlayer();
        auto* playerNodes = player ? f4vr::getPlayerNodes() : nullptr;

        ROCK_LOG_INFO(Weapon,
            "WeaponAnimDump begin key={:016X} reason={} firstPersonSkeleton=0x{:X} firstPersonBoneTree=0x{:X} gameRootNode='{}'/0x{:X} gameFlattenedBoneTree=0x{:X} updateWeaponNode='{}'/0x{:X} getWeaponNode='{}'/0x{:X} WeaponLeftNode='{}'/0x{:X} primaryWeapontoWeaponNode='{}'/0x{:X} primaryWeaponOffsetNode='{}'/0x{:X}",
            observedKey,
            generationChanged ? "generation-change" : "interval",
            reinterpret_cast<std::uintptr_t>(firstPersonSkeleton),
            reinterpret_cast<std::uintptr_t>(firstPersonBoneTree),
            safeNodeName(gameRootNode),
            reinterpret_cast<std::uintptr_t>(gameRootNode),
            reinterpret_cast<std::uintptr_t>(gameFlattenedBoneTree),
            safeNodeName(updateWeaponNode),
            reinterpret_cast<std::uintptr_t>(updateWeaponNode),
            safeNodeName(weaponNode),
            reinterpret_cast<std::uintptr_t>(weaponNode),
            playerNodes ? safeNodeName(playerNodes->WeaponLeftNode) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->WeaponLeftNode : nullptr),
            playerNodes ? safeNodeName(playerNodes->primaryWeapontoWeaponNode) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr),
            playerNodes ? safeNodeName(playerNodes->primaryWeaponOffsetNOde) : "(null)",
            reinterpret_cast<std::uintptr_t>(playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr));

        for (const char* targetName : WEAPON_ANIM_NODE_DUMP_TARGETS) {
            auto matches = collectWeaponAnimNodeMatches(firstPersonSkeleton, targetName);
            ROCK_LOG_INFO(Weapon, "WeaponAnimDump target='{}' matches={}", targetName, matches.size());

            for (std::size_t matchIndex = 0; matchIndex < matches.size(); ++matchIndex) {
                auto* node = matches[matchIndex].node;
                if (!node) {
                    continue;
                }

                auto* niNode = node->IsNode();
                const auto childCount = niNode ? niNode->children.size() : 0;
                const auto stats = summarizeWeaponAnimNodeSubtree(node);
                const auto childNames = weaponAnimNodeImmediateChildNames(node);
                auto* parent = node->parent;

                ROCK_LOG_INFO(Weapon,
                    "WeaponAnimDump node target='{}' match={} path='{}' depth={} addr=0x{:X} name='{}' parent='{}'/0x{:X} children={} childNames='{}' flags=0x{:X} appCulled={} visible={} subtreeNodes={} niNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} subtreeMaxDepth={}",
                    targetName,
                    matchIndex,
                    matches[matchIndex].path,
                    matches[matchIndex].depth,
                    reinterpret_cast<std::uintptr_t>(node),
                    safeNodeName(node),
                    safeNodeName(parent),
                    reinterpret_cast<std::uintptr_t>(parent),
                    static_cast<std::size_t>(childCount),
                    childNames,
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
                    "WeaponAnimDump transform target='{}' match={} localT=({:.3f},{:.3f},{:.3f}) localScale={:.3f} localR=[{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f}] worldT=({:.3f},{:.3f},{:.3f}) worldScale={:.3f} worldR=[{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f};{:.3f},{:.3f},{:.3f}]",
                    targetName,
                    matchIndex,
                    node->local.translate.x,
                    node->local.translate.y,
                    node->local.translate.z,
                    node->local.scale,
                    node->local.rotate.entry[0][0],
                    node->local.rotate.entry[0][1],
                    node->local.rotate.entry[0][2],
                    node->local.rotate.entry[1][0],
                    node->local.rotate.entry[1][1],
                    node->local.rotate.entry[1][2],
                    node->local.rotate.entry[2][0],
                    node->local.rotate.entry[2][1],
                    node->local.rotate.entry[2][2],
                    node->world.translate.x,
                    node->world.translate.y,
                    node->world.translate.z,
                    node->world.scale,
                    node->world.rotate.entry[0][0],
                    node->world.rotate.entry[0][1],
                    node->world.rotate.entry[0][2],
                    node->world.rotate.entry[1][0],
                    node->world.rotate.entry[1][1],
                    node->world.rotate.entry[1][2],
                    node->world.rotate.entry[2][0],
                    node->world.rotate.entry[2][1],
                    node->world.rotate.entry[2][2]);
            }
        }

        // Debug-only authority map: collider generation still follows updateWeaponNode,
        // while these rows show whether the flat-root data has names the visual tree lost.
        const std::array<WeaponAnimNodeDumpRoot, 7> nodeRoots{ {
            { "firstPersonSkeleton", firstPersonSkeleton },
            { "firstPersonBoneTree.nodeChildren", static_cast<RE::NiAVObject*>(firstPersonBoneTree) },
            { "gameRootNode", gameRootNode },
            { "gameFlattenedBoneTree.nodeChildren", static_cast<RE::NiAVObject*>(gameFlattenedBoneTree) },
            { "PlayerNodes.primaryWeapontoWeaponNode", playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr },
            { "PlayerNodes.primaryWeaponOffsetNode", playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr },
            { "PlayerNodes.WeaponLeftNode", playerNodes ? playerNodes->WeaponLeftNode : nullptr },
        } };

        for (const auto& nodeRoot : nodeRoots) {
            logWeaponAnimNodeMapRoot(nodeRoot);
        }

        const std::array<WeaponAnimFlattenedRoot, 2> flatRoots{ {
            { "firstPersonBoneTree.transforms", firstPersonBoneTree },
            { "gameFlattenedBoneTree.transforms", gameFlattenedBoneTree },
        } };

        for (const auto& flatRoot : flatRoots) {
            logWeaponAnimFlattenedMapRoot(flatRoot);
        }

        ROCK_LOG_INFO(Weapon, "WeaponAnimDump end key={:016X}", observedKey);
    }


    std::uint64_t WeaponCollision::getEquippedWeaponIdentityKey(std::uint64_t* outIdentityKey, WeaponSizeClass* outSizeClass) const
    {
        const auto identity = readEquippedWeaponGenerationIdentity();
        const auto identityKey = weapon_generation_identity_policy::makeEquippedWeaponIdentityKey(identity);
        if (outIdentityKey) {
            *outIdentityKey = identityKey;
        }
        if (outSizeClass) {
            *outSizeClass = identity.sizeClass;
        }

        return identityKey;
    }

    weapon_generation_identity_policy::EquippedWeaponGenerationIdentity WeaponCollision::getEquippedWeaponClassification() const
    {
        return readEquippedWeaponGenerationIdentity();
    }

    std::uint64_t WeaponCollision::getWeaponVisualCompositionKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const
    {
        std::uint64_t visualKey = 0;
        if (weaponNode) {
            visualKey = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
            for (const auto& candidate : candidates) {
                if (!candidate.root) {
                    continue;
                }

                ++stats.rootCount;
                mixWeaponVisualString(visualKey, candidate.label);
                mixWeaponVisualKey(visualKey, reinterpret_cast<std::uintptr_t>(candidate.root));
                accumulateWeaponVisualKey(candidate.root, nullptr, 0, 0, visualKey, stats);
            }

            if (visualKey == weapon_visual_composition_policy::kWeaponVisualCompositionOffset) {
                visualKey = reinterpret_cast<std::uint64_t>(weaponNode);
            }
        }
        return visualKey;
    }

    std::size_t WeaponCollision::findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode, std::vector<GeneratedHullSource>& outSources, float maxSourceDistanceGame)
    {
        outSources.clear();
        if (!weaponNode) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon drive root");
            return 0;
        }

        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        if (candidates.empty()) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon root candidates");
            return 0;
        }

        /*
         * Generated weapon collision is now geometry-first: candidate roots are
         * discovery witnesses for the same equipped package, not a competition
         * where one root can hide valid geometry from the others. Sources from
         * every candidate are converted into the update weapon root's local
         * frame, and duplicate TriShapes are accepted once by source pointer.
         */
        RE::NiAVObject* packageDriveRoot = weaponNode;
        const RE::NiTransform packageDriveRootTransform = packageDriveRoot->world;
        std::unordered_set<std::uintptr_t> claimedSourceGroups;
        claimedSourceGroups.reserve(256);
        std::size_t acceptedCandidateCount = 0;
        std::uint32_t totalVisitedShapes = 0;
        std::uint32_t totalExtractedTriangles = 0;
        std::uint32_t totalCulledForDistance = 0;
        const auto groupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(g_rockConfig.rockWeaponCollisionGroupingMode);
        for (const auto& candidate : candidates) {
            std::vector<GeneratedHullSource> candidateSources;
            std::unordered_set<std::uintptr_t> candidateExtractedSourceGroups;
            candidateExtractedSourceGroups.reserve(64);
            std::uint32_t visitedShapes = 0;
            std::uint32_t extractedTriangles = 0;
            std::uint32_t culledForDistance = 0;
            findGeneratedWeaponShapeSourcesRecursive(
                candidate.root,
                packageDriveRoot,
                packageDriveRootTransform,
                0,
                candidateSources,
                visitedShapes,
                extractedTriangles,
                claimedSourceGroups,
                candidateExtractedSourceGroups,
                maxSourceDistanceGame,
                culledForDistance);
            totalCulledForDistance += culledForDistance;

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh candidate: label='{}' root='{}' addr={:x} packageRoot='{}' grouping={} acceptedShapes={} visitedShapes={} triangles={} hulls={}",
                candidate.label,
                safeNodeName(candidate.root),
                reinterpret_cast<std::uintptr_t>(candidate.root),
                safeNodeName(packageDriveRoot),
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(groupingMode),
                candidateExtractedSourceGroups.size(),
                visitedShapes,
                extractedTriangles,
                candidateSources.size());
            totalVisitedShapes += visitedShapes;
            totalExtractedTriangles += extractedTriangles;

            if (!candidateSources.empty()) {
                const auto before = outSources.size();
                outSources.reserve(outSources.size() + candidateSources.size());
                for (auto& source : candidateSources) {
                    outSources.push_back(std::move(source));
                }
                for (const auto sourceGroupId : candidateExtractedSourceGroups) {
                    claimedSourceGroups.insert(sourceGroupId);
                }
                ++acceptedCandidateCount;
                ROCK_LOG_DEBUG(Weapon,
                    "Generated weapon mesh candidate merged: label='{}' root='{}' addedHulls={} totalHulls={} claimedShapes={}",
                    candidate.label,
                    safeNodeName(candidate.root),
                    outSources.size() - before,
                    outSources.size(),
                    claimedSourceGroups.size());
            }
        }

        if (outSources.empty()) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: all {} candidates produced zero hulls", candidates.size());
            return 0;
        }

        auto generatedSourceConvexCount = [](const GeneratedHullSource& source) {
            return source.childLocalPointCloudsGame.empty() ? std::size_t{ 1 } : source.childLocalPointCloudsGame.size();
        };

        auto generatedSourceSemanticMask = [](const std::vector<GeneratedHullSource>& sources) {
            std::uint32_t mask = 0;
            for (const auto& source : sources) {
                mask |= weapon_generated_source_completeness_policy::partMask(source.semantic.partKind);
            }
            return mask;
        };

        auto totalGeneratedConvexCount = [&](const std::vector<GeneratedHullSource>& sources) {
            std::size_t count = 0;
            for (const auto& source : sources) {
                count += generatedSourceConvexCount(source);
            }
            return count;
        };

        auto logGeneratedSourceInventory = [&](const char* reason, const std::vector<GeneratedHullSource>& sources) {
            const auto semanticMask = generatedSourceSemanticMask(sources);
            const auto convexCount = totalGeneratedConvexCount(sources);
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon mesh source inventory: reason={} label='mergedCandidates' root='{}' candidates={} acceptedCandidates={} sources={} convexes={} maxConvexes={} semanticMask=0x{:08X} parts='{}'",
                reason,
                safeNodeName(packageDriveRoot),
                candidates.size(),
                acceptedCandidateCount,
                sources.size(),
                convexCount,
                MAX_WEAPON_BODIES,
                semanticMask,
                generatedWeaponSemanticMaskNames(semanticMask));

            for (std::size_t i = 0; i < sources.size(); ++i) {
                const auto& source = sources[i];
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon mesh source inventory[{}]: source='{}' driveRoot='{}' sourceRoot='{}' part={} partKind={} points={} children={} convexes={} group={:x} boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f})",
                    i,
                    source.sourceName,
                    safeNodeName(source.driveRoot),
                    safeNodeName(source.sourceRoot),
                    generatedWeaponPartKindName(source.semantic.partKind),
                    static_cast<int>(source.semantic.partKind),
                    source.localPointsGame.size(),
                    source.childLocalPointCloudsGame.size(),
                    generatedSourceConvexCount(source),
                    source.sourceGroupId,
                    source.localMinGame.x,
                    source.localMinGame.y,
                    source.localMinGame.z,
                    source.localMaxGame.x,
                    source.localMaxGame.y,
                    source.localMaxGame.z);
            }
        };

        if (outSources.size() > MAX_WEAPON_BODIES) {
            logGeneratedSourceInventory("body-capacity-overflow", outSources);
            const std::size_t droppedCount = outSources.size() - MAX_WEAPON_BODIES;
            ROCK_LOG_WARN(Weapon,
                "Generated weapon mesh body cap reached: extracted={} kept={} dropped={} policy=visible-traversal-order",
                outSources.size(),
                MAX_WEAPON_BODIES,
                droppedCount);
            outSources.resize(MAX_WEAPON_BODIES);
        } else if (outSources.size() == MAX_WEAPON_BODIES) {
            logGeneratedSourceInventory("body-capacity-exact", outSources);
        }

        for (std::size_t i = 0; i < outSources.size(); ++i) {
            const auto& source = outSources[i];
            const auto coverage = classifyGeneratedHull(source.sourceName);
            /*
             * The actual Havok hull is built from sourceLocalPoints* (the
             * source NiNode's own local space), not localPoints* (weapon-root
             * local space) - see buildSourceShape() in
             * createGeneratedWeaponBodiesInBankSlice. If a source node's own
             * NiTransform::scale differs from the weapon root's, the two
             * bounds below will diverge even though position (center) stays
             * correct, since Havok's keyframed placement only drives
             * rotation+translation and never re-applies node scale to an
             * already-baked shape. Logged here to make that divergence
             * directly visible instead of inferred.
             */
            const float sourceNodeScale = source.sourceRoot ? source.sourceRoot->world.scale : 1.0f;
            ROCK_LOG_TRACE(Weapon,
                "Generated weapon mesh selected[{}]: category={} source='{}' driveRoot='{}' sourceRoot='{}' points={} center=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f}) sourceLocalCenter=({:.2f},{:.2f},{:.2f}) sourceLocalBoundsMin=({:.2f},{:.2f},{:.2f}) sourceLocalBoundsMax=({:.2f},{:.2f},{:.2f}) sourceNodeScale={:.4f} weaponRootScale={:.4f}",
                i,
                coverage.label,
                source.sourceName,
                safeNodeName(source.driveRoot),
                safeNodeName(source.sourceRoot),
                source.localPointsGame.size(),
                source.localCenterGame.x,
                source.localCenterGame.y,
                source.localCenterGame.z,
                source.localMinGame.x,
                source.localMinGame.y,
                source.localMinGame.z,
                source.localMaxGame.x,
                source.localMaxGame.y,
                source.localMaxGame.z,
                source.sourceLocalCenterGame.x,
                source.sourceLocalCenterGame.y,
                source.sourceLocalCenterGame.z,
                source.sourceLocalMinGame.x,
                source.sourceLocalMinGame.y,
                source.sourceLocalMinGame.z,
                source.sourceLocalMaxGame.x,
                source.sourceLocalMaxGame.y,
                source.sourceLocalMaxGame.z,
                sourceNodeScale,
                packageDriveRootTransform.scale);
        }

        ROCK_LOG_DEBUG(Weapon,
            "Generated weapon mesh source merged: root='{}' candidates={} acceptedCandidates={} claimedShapes={} visitedShapes={} triangles={} hulls={}",
            safeNodeName(packageDriveRoot),
            candidates.size(),
            acceptedCandidateCount,
            claimedSourceGroups.size(),
            totalVisitedShapes,
            totalExtractedTriangles,
            outSources.size());

        if (totalCulledForDistance > 0) {
            ROCK_LOG_WARN(Weapon,
                "Generated weapon mesh distance filter: culled {} source(s) beyond {:.2f} game units from weapon origin root='{}' (likely misplaced/detached attachment geometry, e.g. laser/holosight nodes authored off-mesh)",
                totalCulledForDistance,
                maxSourceDistanceGame,
                safeNodeName(packageDriveRoot));
        }

        return outSources.size();
    }

    void WeaponCollision::findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node,
        RE::NiAVObject* sourceRoot,
        const RE::NiTransform& weaponRootTransform,
        int depth,
        std::vector<GeneratedHullSource>& outSources,
        std::uint32_t& visitedShapes,
        std::uint32_t& extractedTriangles,
        const std::unordered_set<std::uintptr_t>& claimedSourceGroups,
        std::unordered_set<std::uintptr_t>& candidateExtractedSourceGroups,
        float maxSourceDistanceGame,
        std::uint32_t& culledForDistance)
    {
        if (!node || depth > 15) {
            return;
        }
        auto* triShape = node->IsTriShape();
        if (triShape) {
            const auto sourceGroupId = reinterpret_cast<std::uintptr_t>(triShape);
            if (claimedSourceGroups.find(sourceGroupId) != claimedSourceGroups.end()) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': duplicate TriShape already claimed by earlier candidate", std::string(depth * 2, ' '), safeNodeName(node));
                return;
            }
            if (!weaponVisualNodeVisible(node)) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': TriShape is hidden or locally zero-scale", std::string(depth * 2, ' '), safeNodeName(node));
                return;
            }
            ++visitedShapes;

            std::vector<TriangleData> triangles;
            const bool skinned = isSkinned(triShape);
            const int added = skinned ? extractTrianglesFromSkinnedTriShape(triShape, triangles) : extractTrianglesFromTriShape(triShape, triangles);
            if (added <= 0) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), safeNodeName(node));
                return;
            }
            extractedTriangles += static_cast<std::uint32_t>(added);
            candidateExtractedSourceGroups.insert(sourceGroupId);

            std::vector<RE::NiPoint3> localPoints;
            localPoints.reserve(triangles.size() * 3);
            std::vector<TriangleData> localTriangles;
            localTriangles.reserve(triangles.size());
            std::vector<TriangleData> sourceLocalTriangles;
            sourceLocalTriangles.reserve(triangles.size());
            for (const auto& triangle : triangles) {
                TriangleData localTriangle{};
                localTriangle.v0 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v0);
                localTriangle.v1 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v1);
                localTriangle.v2 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v2);
                TriangleData sourceLocalTriangle{};
                sourceLocalTriangle.v0 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v0);
                sourceLocalTriangle.v1 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v1);
                sourceLocalTriangle.v2 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v2);
                localPoints.push_back(localTriangle.v0);
                localPoints.push_back(localTriangle.v1);
                localPoints.push_back(localTriangle.v2);
                localTriangles.push_back(localTriangle);
                sourceLocalTriangles.push_back(sourceLocalTriangle);
            }

            const float dedupGridGame = (std::max)(g_rockConfig.rockWeaponCollisionPointDedupGrid * havokToGameScale(), 0.01f);
            localPoints = dedupePointCloud(localPoints, dedupGridGame);
            if (!pointCloudCanBuildHull(localPoints)) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': degenerate point cloud points={}", std::string(depth * 2, ' '), safeNodeName(node),
                    localPoints.size());
                return;
            }

            if (maxSourceDistanceGame > 0.0f) {
                /*
                 * Distance is measured from the weapon-root origin (0,0,0 in this
                 * already-converted local space), not from the individual node's
                 * own transform, so it catches geometry whose NiNode was authored
                 * detached/displaced from the weapon mesh (common for laser and
                 * holosight attachments) before any hull/Havok work is spent on it.
                 */
                const float centerDistanceGame = weapon_collision_geometry_math::pointCenter(localPoints).Length();
                if (centerDistanceGame > maxSourceDistanceGame) {
                    ++culledForDistance;
                    ROCK_LOG_TRACE(Weapon,
                        "{}generated mesh source skipped '{}': centerDistance={:.2f} exceeds maxSourceDistance={:.2f} game units from weapon origin",
                        std::string(depth * 2, ' '),
                        safeNodeName(node),
                        centerDistanceGame,
                        maxSourceDistanceGame);
                    return;
                }
            }

            /*
             * Structure anchors outrank NIF name tokens: the nearest ancestor
             * that is a connect point (P-*) or an engine rig node decides the
             * part's slot/function per the record-identity policy. The walk is
             * bounded and purely upward, so it needs no recursion-state
             * threading and stays valid for cached sources (the OMOD set is
             * part of the weapon generation identity).
             */
            auto sourceSemantic = classifyWeaponPartName(safeNodeName(node));
            {
                auto structureAnchor = weapon_part_record_identity_policy::StructureAnchor::None;
                RE::NiAVObject* ancestor = node->parent;
                for (int step = 0; ancestor && step < 24; ++step, ancestor = ancestor->parent) {
                    structureAnchor = weapon_part_record_identity_policy::resolveStructureAnchor(safeNodeName(ancestor));
                    if (structureAnchor != weapon_part_record_identity_policy::StructureAnchor::None) {
                        break;
                    }
                }
                sourceSemantic = weapon_part_record_identity_policy::applyStructureAnchor(sourceSemantic, structureAnchor);
                if (sourceSemantic.classificationSource != WeaponPartClassificationSource::NameToken) {
                    ROCK_LOG_DEBUG(Weapon,
                        "{}generated mesh source '{}' classified by structure anchor: partKind={} attachPoint={:08X}",
                        std::string(depth * 2, ' '),
                        safeNodeName(node),
                        static_cast<int>(sourceSemantic.partKind),
                        sourceSemantic.attachPointFormId);
                }
            }
            auto clusterSet = splitGeneratedWeaponPointCloudForCollision(localPoints);
            auto& clusters = clusterSet.clusters;
            if (clusterSet.supportFitAttempted) {
                ROCK_LOG_DEBUG(Weapon,
                    "{}generated support-fit source '{}': accepted={} fallbackSplit={} rawPoints={} fittedPoints={} clusters={} maxError={:.3f} targetPoints={} repairPoints={} validationDirections={}",
                    std::string(depth * 2, ' '),
                    safeNodeName(node),
                    clusterSet.supportFitAccepted,
                    clusterSet.supportFitFallbackSplit,
                    clusterSet.supportFitInputPoints,
                    clusterSet.supportFitOutputPoints,
                    clusterSet.clusters.size(),
                    clusterSet.supportFitMaxError,
                    g_rockConfig.rockWeaponCollisionSupportFitTargetPoints,
                    clusterSet.supportFitRepairPoints,
                    clusterSet.supportFitValidationDirections);
            }
            for (std::size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {
                auto cluster = weapon_collision_geometry_math::limitPointCloud(std::move(clusters[clusterIndex]), MAX_CONVEX_HULL_POINTS);
                if (!pointCloudCanBuildHull(cluster)) {
                    continue;
                }

                GeneratedHullSource source;
                source.localCenterGame = weapon_collision_geometry_math::pointCenter(cluster);
                source.sourceLocalPointsGame.reserve(cluster.size());
                for (const auto& point : cluster) {
                    const RE::NiPoint3 pointWorld = weapon_collision_geometry_math::localPointToWorld(
                        weaponRootTransform.rotate,
                        weaponRootTransform.translate,
                        weaponRootTransform.scale,
                        point);
                    source.sourceLocalPointsGame.push_back(weapon_collision_geometry_math::worldPointToLocal(
                        node->world.rotate,
                        node->world.translate,
                        node->world.scale,
                        pointWorld));
                }
                source.sourceLocalCenterGame = weapon_collision_geometry_math::pointCenter(source.sourceLocalPointsGame);
                const auto bounds = pointCloudBounds(cluster);
                const auto sourceBounds = pointCloudBounds(source.sourceLocalPointsGame);
                source.localMinGame = bounds.min;
                source.localMaxGame = bounds.max;
                source.sourceLocalMinGame = sourceBounds.min;
                source.sourceLocalMaxGame = sourceBounds.max;
                source.localPointsGame = std::move(cluster);
                source.localTrianglesGame = localTriangles;
                source.sourceLocalTrianglesGame = sourceLocalTriangles;
                source.driveRoot = sourceRoot;
                source.sourceRoot = node;
                source.sourceNodeScale = node->world.scale;
                source.sourceGroupId = sourceGroupId;
                source.sourceName = safeNodeName(node);
                if (clusters.size() > 1) {
                    source.sourceName += "#";
                    source.sourceName += std::to_string(clusterIndex);
                }
                source.semantic = sourceSemantic;
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source '{}': points={} center=({:.2f},{:.2f},{:.2f})", std::string(depth * 2, ' '), source.sourceName,
                    source.localPointsGame.size(), source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
                outSources.push_back(std::move(source));
            }
            return;
        }

        auto* niNode = node->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                if (auto* kid = kids[i].get()) {
                    findGeneratedWeaponShapeSourcesRecursive(
                        kid,
                        sourceRoot,
                        weaponRootTransform,
                        depth + 1,
                        outSources,
                        visitedShapes,
                        extractedTriangles,
                        claimedSourceGroups,
                        candidateExtractedSourceGroups,
                        maxSourceDistanceGame,
                        culledForDistance);
                }
            }
        }
    }

    RE::NiTransform WeaponCollision::makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const
    {
        RE::NiTransform result = weaponRootTransform;
        /*
         * Generated weapon hull points are extracted in weapon-root local space,
         * but the body target still passes through the shared generated-body
         * Ni-to-Havok conversion. Use the inverse stored basis here so Havok
         * receives the same effective package orientation that the center math
         * uses. Without this, the package center tracks correctly while each
         * hull spins around its creation center with all axes reversed.
         */
        result.rotate = weapon_collision_geometry_math::transposeRotation(weaponRootTransform.rotate);
        result.translate = weapon_collision_geometry_math::localPointToWorld(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, localCenterGame);
        return result;
    }

    bool WeaponCollision::weaponCollisionSettingsChanged() const
    {
        if (_cachedConvexRadius < 0.0f || _cachedPointDedupGrid < 0.0f || _cachedSupportFitTargetPoints < 0 ||
            _cachedSupportFitMaxErrorGameUnits < 0.0f) {
            return false;
        }
        return std::abs(g_rockConfig.rockWeaponCollisionConvexRadius - _cachedConvexRadius) > 0.00001f ||
               std::abs(g_rockConfig.rockWeaponCollisionPointDedupGrid - _cachedPointDedupGrid) > 0.00001f ||
               g_rockConfig.rockWeaponCollisionSupportFitTargetPoints != _cachedSupportFitTargetPoints ||
               std::abs(g_rockConfig.rockWeaponCollisionSupportFitMaxErrorGameUnits - _cachedSupportFitMaxErrorGameUnits) > 0.00001f;
    }

    std::size_t WeaponCollision::createGeneratedWeaponBodiesInBank(RE::hknpWorld* world,
        const std::vector<GeneratedHullSource>& sources,
        WeaponBodyBank& bank,
        const GeneratedWeaponBodyCreateOptions& options)
    {
        std::size_t nextSourceIndex = 0;
        const auto createdCount = createGeneratedWeaponBodiesInBankSlice(world, sources, bank, options, nextSourceIndex, MAX_WEAPON_BODIES);
        if (createdCount > 0) {
            ROCK_LOG_INFO(Weapon, "Generated weapon mesh collision created {}/{} hull bodies", createdCount, sources.size());
        }
        return createdCount;
    }

    std::size_t WeaponCollision::createGeneratedWeaponBodiesInBankSlice(RE::hknpWorld* world,
        const std::vector<GeneratedHullSource>& sources,
        WeaponBodyBank& bank,
        const GeneratedWeaponBodyCreateOptions& options,
        std::size_t& nextSourceIndex,
        std::size_t maxSourceAttemptsThisFrame)
    {
        if (nextSourceIndex == 0 && bankHasWeaponBody(bank)) {
            ROCK_LOG_WARN(Weapon, "createGeneratedWeaponBodiesInBankSlice called with a non-empty target bank at source start - skipping");
            return 0;
        }
        if (!world || !_cachedBhkWorld || sources.empty()) {
            return 0;
        }
        if (maxSourceAttemptsThisFrame == 0 || nextSourceIndex >= sources.size()) {
            return 0;
        }

        std::size_t createdCount = bankWeaponBodyCount(bank);
        std::size_t createdThisFrame = 0;
        std::size_t attemptedThisFrame = 0;
        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(options.collisionEnabledOnCreate);
        auto buildSourceShape = [&](const GeneratedHullSource& source) -> RE::hknpShape* {
            if (source.childLocalPointCloudsGame.size() <= 1) {
                const bool useSourceLocal = !source.sourceLocalPointsGame.empty();
                const auto& sourcePoints = useSourceLocal ? source.sourceLocalPointsGame : source.localPointsGame;
                const auto& sourceCenter = useSourceLocal ? source.sourceLocalCenterGame : source.localCenterGame;
                const float sourceScale = useSourceLocal ? source.sourceNodeScale : 1.0f;
                auto centeredHavokPoints = makeCenteredHavokPointCloud(sourcePoints, sourceCenter, sourceScale);
                return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
            }

            std::vector<RE::hknpShape*> childShapes;
            std::vector<havok_compound_shape_builder::CompoundChild> children;
            childShapes.reserve(source.childLocalPointCloudsGame.size());
            children.reserve(source.childLocalPointCloudsGame.size());

            for (const auto& childLocalPointsGame : source.childLocalPointCloudsGame) {
                if (!pointCloudCanBuildHull(childLocalPointsGame)) {
                    continue;
                }

                const auto childCenterGame = weapon_collision_geometry_math::pointCenter(childLocalPointsGame);
                auto centeredChildHavokPoints = makeCenteredHavokPointCloud(childLocalPointsGame, childCenterGame);
                auto* childShape =
                    havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredChildHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
                if (!childShape) {
                    ROCK_LOG_WARN(Weapon, "Generated weapon compound source '{}' failed child convex build", source.sourceName);
                    continue;
                }

                childShapes.push_back(childShape);

                havok_compound_shape_builder::CompoundChild child{};
                child.shape = childShape;
                child.transform.translation.x = (childCenterGame.x - source.localCenterGame.x) * gameToHavokScale();
                child.transform.translation.y = (childCenterGame.y - source.localCenterGame.y) * gameToHavokScale();
                child.transform.translation.z = (childCenterGame.z - source.localCenterGame.z) * gameToHavokScale();
                child.transform.translation.w = 1.0f;
                children.push_back(child);
            }

            RE::hknpShape* compoundShape = nullptr;
            if (children.size() > 1) {
                compoundShape = havok_compound_shape_builder::buildStaticCompoundShape(children);
            } else if (children.size() == 1) {
                compoundShape = const_cast<RE::hknpShape*>(children.front().shape);
                childShapes.clear();
            }

            for (auto* childShape : childShapes) {
                shapeRemoveRef(childShape);
            }

            return compoundShape;
        };

        while (nextSourceIndex < sources.size() && createdCount < MAX_WEAPON_BODIES && attemptedThisFrame < maxSourceAttemptsThisFrame) {
            const std::size_t sourceIndex = nextSourceIndex++;
            ++attemptedThisFrame;
            const auto& source = sources[sourceIndex];
            const auto& shapePoints = source.sourceLocalPointsGame.empty() ? source.localPointsGame : source.sourceLocalPointsGame;
            if (!pointCloudCanBuildHull(shapePoints)) {
                continue;
            }

            auto* shape = buildSourceShape(source);
            if (!shape) {
                ROCK_LOG_WARN(Weapon, "Generated weapon mesh hull '{}' failed native shape build", source.sourceName);
                continue;
            }

            auto& instance = bank[createdCount];
            instance.shape = shape;
            instance.driveNode = source.driveRoot ? source.driveRoot : source.sourceRoot;
            instance.sourceNode = source.sourceRoot;
            instance.sourceName = source.sourceName;
            instance.sourceRootName = source.sourceRoot ? safeNodeName(source.sourceRoot) : "";
            instance.generatedLocalCenterGame = source.localCenterGame;
            instance.generatedSourceLocalCenterGame = source.sourceLocalCenterGame;
            instance.generatedLocalMinGame = source.localMinGame;
            instance.generatedLocalMaxGame = source.localMaxGame;
            instance.generatedSourceLocalMinGame = source.sourceLocalMinGame;
            instance.generatedSourceLocalMaxGame = source.sourceLocalMaxGame;
            instance.generatedLocalPointsGame = source.localPointsGame;
            instance.generatedLocalTrianglesGame = source.localTrianglesGame;
            instance.generatedSourceLocalPointsGame = source.sourceLocalPointsGame;
            instance.generatedSourceLocalTrianglesGame = source.sourceLocalTrianglesGame;
            instance.generatedPointCount = static_cast<std::uint32_t>(
                (std::min)(source.localPointsGame.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            instance.semantic = source.semantic;
            instance.ownsShapeRef = true;
            clearGeneratedKeyframedBodyDriveState(instance.driveState);

            const bool ok =
                instance.body.create(world, _cachedBhkWorld, shape, filterInfo, { 0 }, BethesdaMotionType::Keyframed, "ROCK_WeaponMeshCollision");

            if (!ok) {
                ROCK_LOG_ERROR(Weapon, "BethesdaPhysicsBody::create failed for generated weapon mesh hull '{}'", source.sourceName);
                shapeRemoveRef(shape);
                clearWeaponBodyInstance(instance, false);
                continue;
            }

            instance.body.createNiNode("ROCK_WeaponMeshCollision");
            const RE::NiTransform driveRootTransform = instance.sourceNode ? instance.sourceNode->world : (instance.driveNode ? instance.driveNode->world : makeIdentityTransform());
            const RE::NiPoint3 initialCenterGame = instance.sourceNode ? source.sourceLocalCenterGame : source.localCenterGame;
            const RE::NiTransform initialTransform = makeGeneratedBodyWorldTransform(driveRootTransform, initialCenterGame);
            if (!placeGeneratedKeyframedBodyImmediately(instance.body, initialTransform)) {
                ROCK_LOG_ERROR(Weapon,
                    "Generated weapon mesh collision initial placement failed meshIndex={} bodyId={} source='{}' driveRoot='{}' sourceRoot='{}'",
                    createdCount,
                    instance.body.getBodyId().value,
                    source.sourceName,
                    safeNodeName(source.driveRoot),
                    safeNodeName(source.sourceRoot));
                retireWeaponBodyInstance(instance, false);
                shapeRemoveRef(shape);
                continue;
            }
            initializeGeneratedKeyframedBodyDriveState(instance.driveState, initialTransform);

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh collision body created: meshIndex={} bodyId={} source='{}' driveRoot='{}' sourceRoot='{}' partKind={} supportRole={} reloadRole={} points={} children={} center=({:.2f},{:.2f},{:.2f}) layer=44",
                createdCount, instance.body.getBodyId().value, source.sourceName, safeNodeName(source.driveRoot), safeNodeName(source.sourceRoot), static_cast<int>(source.semantic.partKind),
                static_cast<int>(source.semantic.supportGripRole), static_cast<int>(source.semantic.reloadRole), source.localPointsGame.size(),
                source.childLocalPointCloudsGame.size(), source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
            ++createdCount;
            ++createdThisFrame;
        }

        if (createdThisFrame > 0) {
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
        }
        return createdThisFrame;
    }

    void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
    {
        if (!bankHasWeaponBody(_weaponBodies) && !bankHasWeaponBody(_weaponReplacementBodies)) {
            clearGeneratedSourceCompletenessTracking();
            clearPendingWeaponVisualRebuild();
            clearGeneratedSourceCache();
            clearPendingGeneratedWeaponBuild(world, false);
            _driveRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
            return;
        }

        clearAtomicBodyIds();
        resetWeaponBodySetGeneration();

        const auto activeDestroyed = bankWeaponBodyCount(activeWeaponBodies());
        const auto inactiveDestroyed = bankWeaponBodyCount(inactiveWeaponBodies());
        destroyWeaponBodyBank(activeWeaponBodies(), true);
        destroyWeaponBodyBank(inactiveWeaponBodies(), true);
        _usingReplacementWeaponBodies = false;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(world, false);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);

        ROCK_LOG_INFO(Weapon, "Weapon collision bodies destroyed count={}", activeDestroyed + inactiveDestroyed);
    }

    void WeaponCollision::invalidateForScaleChange(RE::hknpWorld* world)
    {
        const bool hadWeaponBody = hasWeaponBody();
        if (hadWeaponBody) {
            ROCK_LOG_INFO(Weapon, "Generated weapon collision invalidated by physics scale change");
            destroyWeaponBody(world);
        } else {
            clearAtomicBodyIds();
            resetWeaponBodySetGeneration();
            ROCK_LOG_DEBUG(Weapon, "Generated weapon collision scale invalidation had no active bodies");
        }

        _cachedWeaponKey = 0;
        _cachedWeaponVisualKey = 0;
        _cachedWeaponIdentityKey = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(world, true);
        resetVisualSourceUnavailableRetention();
        resetWeaponCollisionSettingsCache();
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
    }

    void WeaponCollision::destroyWeaponBodyBank(WeaponBodyBank& bank, bool releaseShapeRef)
    {
        for (auto& instance : bank) {
            retireWeaponBodyInstance(instance, releaseShapeRef);
        }
    }

    void WeaponCollision::retireWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (instance.body.isValid()) {
            RetiredBethesdaPhysicsBodyPayload payload{};
            if (instance.body.retireFromWorld(_cachedBhkWorld, payload) && payload.occupied()) {
                retireWeaponBodyPayload(payload);
            } else {
                /*
                 * Releasing a generated wrapper immediately after a rebuild was
                 * observed to leave native readers with stale collision-object
                 * pointers. If retirement cannot produce a payload, clear ROCK's
                 * ownership without calling the immediate destructor path.
                 */
                ROCK_LOG_ERROR(Weapon,
                    "Generated weapon body {} could not be retired; wrapper ownership cleared without immediate native release",
                    instance.body.getBodyId().value);
            }
        }
        clearWeaponBodyInstance(instance, releaseShapeRef);
    }

    void WeaponCollision::retireWeaponBodyPayload(RetiredBethesdaPhysicsBodyPayload& payload)
    {
        if (!payload.occupied()) {
            return;
        }

        std::scoped_lock lock(_retiredWeaponBodyPayloadMutex);
        for (auto& retired : _retiredWeaponBodyPayloads) {
            if (!retired.occupied()) {
                retired.bodyPayload = payload;
                retired.remainingPhysicsSteps = RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS;
                ++_retiredWeaponBodyPayloadCount;
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    1000,
                    "Generated weapon body {} payload retired for {} physics steps activeRetired={}",
                    payload.bodyId,
                    RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS,
                    _retiredWeaponBodyPayloadCount);
                payload = {};
                return;
            }
        }

        /*
         * A leak is safer than freeing a collision object that native pathing or
         * collision readers may still touch after a generated weapon rebuild.
         */
        ROCK_LOG_ERROR(Weapon,
            "Retired generated weapon body queue full; intentionally leaking body {} payload to avoid native use-after-free",
            payload.bodyId);
        payload = {};
    }

    void WeaponCollision::setWeaponBodyBankCollisionEnabled(RE::hknpWorld* world, WeaponBodyBank& bank, bool enabled)
    {
        if (!world) {
            return;
        }

        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(enabled);
        for (auto& instance : bank) {
            if (instance.body.isValid()) {
                body_collision::setFilterInfo(world, instance.body.getBodyId(), filterInfo);
            }
        }
    }

    void WeaponCollision::clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.driveNode = nullptr;
        instance.sourceNode = nullptr;
        instance.sourceName.clear();
        instance.sourceRootName.clear();
        instance.generatedLocalCenterGame = {};
        instance.generatedSourceLocalCenterGame = {};
        instance.generatedLocalMinGame = {};
        instance.generatedLocalMaxGame = {};
        instance.generatedSourceLocalMinGame = {};
        instance.generatedSourceLocalMaxGame = {};
        instance.generatedLocalPointsGame.clear();
        instance.generatedLocalTrianglesGame.clear();
        instance.generatedSourceLocalPointsGame.clear();
        instance.generatedSourceLocalTrianglesGame.clear();
        instance.generatedPointCount = 0;
        instance.semantic = {};
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
        instance.publicationIndex = INVALID_BODY_ID;
    }

    void WeaponCollision::beginWeaponBodyPublication()
    {
        const std::uint64_t version = _weaponBodyPublicationVersion.load(std::memory_order_relaxed);
        _weaponBodyPublicationVersion.store((version & ~1ull) + 1ull, std::memory_order_release);
    }

    void WeaponCollision::endWeaponBodyPublication()
    {
        const std::uint64_t version = _weaponBodyPublicationVersion.load(std::memory_order_relaxed);
        _weaponBodyPublicationVersion.store((version | 1ull) + 1ull, std::memory_order_release);
    }

    void WeaponCollision::clearAtomicBodyIds()
    {
        beginWeaponBodyPublication();
        _weaponBodyCountAtomic.store(0, std::memory_order_release);
        _weaponBodySetKeyAtomic.store(0, std::memory_order_release);
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (auto& value : _weaponBodyPartKindsAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        }
        for (auto& value : _weaponBodyReloadRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodySupportRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodySocketRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyActionRolesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyGripPosesAtomic) {
            value.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        }
        for (auto& value : _weaponBodyInteractionRootsAtomic) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _weaponBodySourceRootsAtomic) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _weaponBodyGenerationKeysAtomic) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _weaponBodySampledVelocityHavokXAtomic) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _weaponBodySampledVelocityHavokYAtomic) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _weaponBodySampledVelocityHavokZAtomic) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _weaponBodySampledVelocityValidAtomic) {
            value.store(0, std::memory_order_release);
        }
        {
            std::scoped_lock lock(_profileEvidenceSnapshotMutex);
            _profileEvidenceSnapshot.clear();
        }
        endWeaponBodyPublication();
    }

    void WeaponCollision::unpublishAtomicBodyIds()
    {
        clearAtomicBodyIds();
    }

    void WeaponCollision::publishAtomicBodyIds(WeaponBodyBank& bank)
    {
        auto evidenceSnapshot = buildProfileEvidenceSnapshot(bank);
        std::uint32_t count = 0;
        beginWeaponBodyPublication();
        _weaponBodyCountAtomic.store(0, std::memory_order_release);
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (auto& value : _weaponBodySampledVelocityValidAtomic) {
            value.store(0, std::memory_order_release);
        }
        _weaponBodySetKeyAtomic.store(_cachedWeaponBodySetKey, std::memory_order_release);
        {
            std::scoped_lock lock(_profileEvidenceSnapshotMutex);
            _profileEvidenceSnapshot = std::move(evidenceSnapshot);
        }
        RE::NiAVObject* packageDriveNode = resolvePackageDriveNode(bank, nullptr);
        for (auto& instance : bank) {
            if (instance.body.isValid() && count < MAX_WEAPON_BODIES) {
                instance.publicationIndex = count;
                _weaponBodyPartKindsAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.partKind), std::memory_order_release);
                _weaponBodyReloadRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.reloadRole), std::memory_order_release);
                _weaponBodySupportRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.supportGripRole), std::memory_order_release);
                _weaponBodySocketRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.socketRole), std::memory_order_release);
                _weaponBodyActionRolesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.actionRole), std::memory_order_release);
                _weaponBodyGripPosesAtomic[count].store(static_cast<std::uint32_t>(instance.semantic.fallbackGripPose), std::memory_order_release);
                _weaponBodyInteractionRootsAtomic[count].store(reinterpret_cast<std::uintptr_t>(packageDriveNode), std::memory_order_release);
                _weaponBodySourceRootsAtomic[count].store(reinterpret_cast<std::uintptr_t>(instance.sourceNode), std::memory_order_release);
                _weaponBodyGenerationKeysAtomic[count].store(_cachedWeaponBodySetKey, std::memory_order_release);
                _weaponBodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
                ++count;
            } else {
                instance.publicationIndex = INVALID_BODY_ID;
            }
        }
        _weaponBodyCountAtomic.store(count, std::memory_order_release);
        endWeaponBodyPublication();
        dumpEquippedWeaponOmodEvidence(bank, packageDriveNode);
    }

    namespace
    {
        void dumpOmodWeaponTreeRecursive(
            RE::NiAVObject* node,
            int depth,
            std::size_t& visited,
            const std::unordered_map<std::uintptr_t, std::string>& evidenceMarkers)
        {
            constexpr int kMaxDumpDepth = 16;
            constexpr std::size_t kMaxDumpNodes = 512;
            if (!node || depth > kMaxDumpDepth || visited >= kMaxDumpNodes) {
                return;
            }
            ++visited;

            const auto markerIt = evidenceMarkers.find(reinterpret_cast<std::uintptr_t>(node));
            auto* niNode = node->IsNode();
            ROCK_LOG_INFO(Weapon,
                "OMOD-DUMP tree {}{} addr={:x} children={} localT=({:.2f},{:.2f},{:.2f}){}",
                std::string(static_cast<std::size_t>(depth) * 2, ' '),
                safeNodeName(node),
                reinterpret_cast<std::uintptr_t>(node),
                niNode ? niNode->children.size() : 0,
                node->local.translate.x,
                node->local.translate.y,
                node->local.translate.z,
                markerIt != evidenceMarkers.end() ? markerIt->second : "");

            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    dumpOmodWeaponTreeRecursive(child, depth + 1, visited, evidenceMarkers);
                }
            }
        }
    }

    /*
     * One-shot research dump (gated on bDebugWeaponOmodDump) that pairs the
     * equipped instance's installed-OMOD records with the assembled scene tree
     * and the generated evidence bindings. This exists to establish the
     * record-to-node anchoring mechanism for record-authored part identity;
     * every field read below that is not already exercised elsewhere in ROCK
     * (attachPoint index, OMOD model path) is deliberately printed raw so a
     * VR layout mismatch shows up as garbage in the log instead of a crash.
     */
    void WeaponCollision::dumpEquippedWeaponOmodEvidence(const WeaponBodyBank& bank, RE::NiAVObject* packageDriveNode)
    {
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
            return;
        }
        if (_cachedWeaponBodySetKey == 0 || _cachedWeaponBodySetKey == _lastOmodDumpGenerationKey) {
            return;
        }
        _lastOmodDumpGenerationKey = _cachedWeaponBodySetKey;

        auto* player = f4vr::getPlayer();
        auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
        auto* equipData = processData ? processData->equipData : nullptr;
        auto* weaponForm = equipData ? equipData->item : nullptr;
        ROCK_LOG_INFO(Weapon,
            "OMOD-DUMP begin generation={:016X} weapon={:08X} '{}'",
            _cachedWeaponBodySetKey,
            weaponForm ? weaponForm->formID : 0u,
            weaponForm && weaponForm->GetFullName() ? weaponForm->GetFullName() : "");

        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData) : nullptr;
        if (objectInstanceExtra && objectInstanceExtra->values) {
            const auto indexData = objectInstanceExtra->GetIndexData();
            ROCK_LOG_INFO(Weapon, "OMOD-DUMP installed mods count={}", indexData.size());
            for (const auto& modIndex : indexData) {
                auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                if (!omod) {
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-DUMP mod objectID={:08X} index={} rank={} disabled={} UNRESOLVED",
                        modIndex.objectID,
                        modIndex.index,
                        modIndex.rank,
                        modIndex.disabled);
                    continue;
                }

                const std::uint16_t attachPointIndex = omod->attachPoint.keywordIndex;
                const RE::BGSKeyword* attachPointKeyword =
                    RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, attachPointIndex);
                ROCK_LOG_INFO(Weapon,
                    "OMOD-DUMP mod formID={:08X} formType={:02X} name='{}' index={} rank={} disabled={} "
                    "attachPointIndex={} attachPoint={:08X} '{}' model='{}'",
                    omod->formID,
                    static_cast<std::uint32_t>(omod->formType.underlying()),
                    omod->fullName.c_str() ? omod->fullName.c_str() : "",
                    modIndex.index,
                    modIndex.rank,
                    modIndex.disabled,
                    attachPointIndex,
                    attachPointKeyword ? attachPointKeyword->formID : 0u,
                    attachPointKeyword && attachPointKeyword->formEditorID.c_str() ? attachPointKeyword->formEditorID.c_str() : "",
                    omod->model.c_str() ? omod->model.c_str() : "");
            }
        } else {
            ROCK_LOG_INFO(Weapon, "OMOD-DUMP no object instance extra available");
        }

        std::unordered_map<std::uintptr_t, std::string> evidenceMarkers;
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            ROCK_LOG_INFO(Weapon,
                "OMOD-DUMP evidence bodyId={} source='{}' sourceRoot='{}' sourceNode={:x} partKind={} reload={} support={} socket={} action={} points={}",
                instance.body.getBodyId().value,
                instance.sourceName,
                instance.sourceRootName,
                reinterpret_cast<std::uintptr_t>(instance.sourceNode),
                static_cast<int>(instance.semantic.partKind),
                static_cast<int>(instance.semantic.reloadRole),
                static_cast<int>(instance.semantic.supportGripRole),
                static_cast<int>(instance.semantic.socketRole),
                static_cast<int>(instance.semantic.actionRole),
                instance.generatedPointCount);
            if (instance.sourceNode) {
                auto& marker = evidenceMarkers[reinterpret_cast<std::uintptr_t>(instance.sourceNode)];
                marker += fmt::format(" <== bodyId={} '{}'", instance.body.getBodyId().value, instance.sourceName);
            }
        }

        std::size_t visited = 0;
        dumpOmodWeaponTreeRecursive(packageDriveNode, 0, visited, evidenceMarkers);
        ROCK_LOG_INFO(Weapon, "OMOD-DUMP end nodesLogged={}", visited);
    }

    namespace
    {
        constexpr std::size_t OMOD_AUDIT_MAX_MATCHES_PER_TOKEN = 8;
        constexpr std::size_t OMOD_AUDIT_MAX_CONNECT_POINT_MATCHES = 64;
        constexpr std::size_t OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD = 3;

        struct OmodAuditNodeMatch
        {
            RE::NiAVObject* node{ nullptr };
            const char* rootLabel{ "" };
        };

        struct OmodAuditTokenSlot
        {
            std::string lowerToken;
            /*
             * Word-set fallback: mesh authors reorder basename words
             * ('AK74_HG_Lower.nif' vs node 'AK74_Lower_HG'), which made exact
             * substring matching report false NODE_NOT_FOUND. A node matches
             * when every basename word appears somewhere in its name. False
             * NODE_NOT_FOUND must stay rare because the self-heal uses that
             * verdict as its trigger.
             */
            std::vector<std::string> lowerWords;
            std::vector<OmodAuditNodeMatch> matches;
        };

        struct OmodAuditRecord
        {
            std::uint32_t formId{ 0 };
            std::uint32_t attachPointFormId{ 0 };
            std::uint16_t attachPointIndex{ 0 };
            std::uint32_t modIndex{ 0 };
            std::uint32_t rank{ 0 };
            bool disabled{ false };
            bool resolved{ false };
            std::string name;
            std::string modelPath;
        };

        char omodAuditToLowerAscii(char c)
        {
            return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + ('a' - 'A')) : c;
        }

        /*
         * Search token = OMOD model NIF basename without extension, lowered.
         * Node names authored from the model file usually contain this token;
         * mesh-internal names may not, which is why the connect-point census
         * below exists as the structural fallback.
         */
        std::string makeOmodAuditModelToken(const char* modelPath)
        {
            if (!modelPath || modelPath[0] == '\0') {
                return {};
            }
            const char* base = modelPath;
            for (const char* cursor = modelPath; *cursor; ++cursor) {
                if (*cursor == '\\' || *cursor == '/') {
                    base = cursor + 1;
                }
            }
            std::string token(base);
            const auto dot = token.find_last_of('.');
            if (dot != std::string::npos) {
                token.resize(dot);
            }
            for (auto& c : token) {
                c = omodAuditToLowerAscii(c);
            }
            return token;
        }

        // Allocation-free case-insensitive substring test against a pre-lowered token.
        bool omodAuditNameContainsToken(const char* name, const std::string& lowerToken)
        {
            if (!name || lowerToken.empty()) {
                return false;
            }
            const std::size_t tokenLength = lowerToken.size();
            for (const char* cursor = name; *cursor; ++cursor) {
                std::size_t i = 0;
                while (i < tokenLength) {
                    const char c = cursor[i];
                    if (c == '\0' || omodAuditToLowerAscii(c) != lowerToken[i]) {
                        break;
                    }
                    ++i;
                }
                if (i == tokenLength) {
                    return true;
                }
            }
            return false;
        }

        bool omodAuditNameIsConnectPoint(const char* name)
        {
            return name && (name[0] == 'P' || name[0] == 'p') && name[1] == '-';
        }

        std::vector<std::string> makeOmodAuditTokenWords(const std::string& lowerToken)
        {
            std::vector<std::string> words;
            std::string current;
            for (const char c : lowerToken) {
                if (c == '_' || c == '-' || c == ' ') {
                    if (current.size() >= 2) {
                        words.push_back(current);
                    }
                    current.clear();
                } else {
                    current += c;
                }
            }
            if (current.size() >= 2) {
                words.push_back(current);
            }
            // A single word degenerates to the substring test; two or more
            // words are required for the reordered-words fallback to add
            // signal instead of noise.
            if (words.size() < 2) {
                words.clear();
            }
            return words;
        }

        bool omodAuditNameMatchesTokenSlot(const char* name, const OmodAuditTokenSlot& slot)
        {
            if (omodAuditNameContainsToken(name, slot.lowerToken)) {
                return true;
            }
            if (slot.lowerWords.empty()) {
                return false;
            }
            for (const auto& word : slot.lowerWords) {
                if (!omodAuditNameContainsToken(name, word)) {
                    return false;
                }
            }
            return true;
        }

        bool omodAuditMatchesContainNode(const std::vector<OmodAuditNodeMatch>& matches, const RE::NiAVObject* node)
        {
            for (const auto& match : matches) {
                if (match.node == node) {
                    return true;
                }
            }
            return false;
        }

        /*
         * One walk per root evaluates every OMOD token plus the P-* connect
         * point predicate, instead of one walk per (root, token) pair. Matches
         * deduplicate across roots by node address because the weapon subtree
         * is reachable from several of the audited roots.
         */
        void scanOmodAuditTreeRecursive(
            RE::NiAVObject* node,
            std::uint32_t depth,
            std::size_t& visited,
            std::size_t maxVisited,
            const char* rootLabel,
            std::vector<OmodAuditTokenSlot>& tokenSlots,
            std::vector<OmodAuditNodeMatch>& connectPointMatches)
        {
            if (!node || visited >= maxVisited || depth > WEAPON_ANIM_NODE_DUMP_MAX_DEPTH) {
                return;
            }
            ++visited;

            const char* name = node->name.c_str();
            if (name && name[0] != '\0') {
                if (omodAuditNameIsConnectPoint(name) &&
                    connectPointMatches.size() < OMOD_AUDIT_MAX_CONNECT_POINT_MATCHES &&
                    !omodAuditMatchesContainNode(connectPointMatches, node)) {
                    connectPointMatches.push_back(OmodAuditNodeMatch{ node, rootLabel });
                }
                for (auto& slot : tokenSlots) {
                    if (slot.lowerToken.empty() || slot.matches.size() >= OMOD_AUDIT_MAX_MATCHES_PER_TOKEN) {
                        continue;
                    }
                    if (!omodAuditNameMatchesTokenSlot(name, slot)) {
                        continue;
                    }
                    if (!omodAuditMatchesContainNode(slot.matches, node)) {
                        slot.matches.push_back(OmodAuditNodeMatch{ node, rootLabel });
                    }
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    scanOmodAuditTreeRecursive(child, depth + 1, visited, maxVisited, rootLabel, tokenSlots, connectPointMatches);
                }
            }
        }

        // Path is rebuilt from the parent chain only for matched nodes, so the
        // scan itself stays allocation-free per visited node.
        std::string buildOmodAuditNodePath(const RE::NiAVObject* node)
        {
            std::array<const char*, WEAPON_ANIM_NODE_DUMP_MAX_DEPTH + 1> names{};
            std::size_t count = 0;
            for (const RE::NiAVObject* cursor = node; cursor && count < names.size(); cursor = cursor->parent) {
                const char* name = cursor->name.c_str();
                names[count++] = name && name[0] != '\0' ? name : "(unnamed)";
            }
            std::string path;
            for (std::size_t i = count; i > 0; --i) {
                if (!path.empty()) {
                    path += "/";
                }
                path += names[i - 1];
            }
            return path;
        }

        std::size_t countOmodAuditEvidenceSourcesInSubtree(
            RE::NiAVObject* node,
            const std::unordered_set<std::uintptr_t>& evidenceSourceAddresses,
            std::size_t& visited)
        {
            if (!node || visited >= WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES) {
                return 0;
            }
            ++visited;
            std::size_t count = evidenceSourceAddresses.count(reinterpret_cast<std::uintptr_t>(node)) != 0 ? 1 : 0;
            auto* niNode = node->IsNode();
            if (!niNode) {
                return count;
            }
            const auto& children = niNode->children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    count += countOmodAuditEvidenceSourcesInSubtree(child, evidenceSourceAddresses, visited);
                }
            }
            return count;
        }
    }

    /*
     * Periodic research audit (gated on bDebugWeaponOmodCoverageAudit) that
     * re-diffs record truth against the live scene graphs while a generated
     * weapon body set is active. The build-time OMOD-DUMP can only show what
     * the tree looked like when the body set was published; this audit exists
     * to catch the missing-part failure where the engine attaches an OMOD's
     * model subtree after ROCK's build window closed and nothing ever looks
     * again. Per installed OMOD it reports whether a node matching the model
     * NIF exists under any audited root, whether it is visible, and whether
     * any generated collider evidence source lives beneath it; the P-* census
     * covers parts whose mesh names do not contain the model basename. The
     * visual-key drift line on each audit is the decisive signal: drift=YES
     * with an unchanged body set means geometry arrived or changed after the
     * build and current triggers never rescanned it.
     */
    void WeaponCollision::maybeRunWeaponOmodCoverageAudit(RE::NiAVObject* weaponNode)
    {
        if (!g_rockConfig.rockDebugWeaponOmodCoverageAudit) {
            return;
        }
        if (!weaponNode || !hasWeaponBody() || _cachedWeaponBodySetKey == 0) {
            return;
        }

        if (_omodCoverageAuditBodySetKey != _cachedWeaponBodySetKey) {
            _omodCoverageAuditBodySetKey = _cachedWeaponBodySetKey;
            _omodCoverageAuditFrameCounter = 0;
            _omodCoverageAuditRunIndex = 0;
        }

        const int intervalFrames = (std::max)(30, g_rockConfig.rockDebugWeaponOmodCoverageAuditIntervalFrames);
        // First audit fires ~1s after publication so late model streaming is
        // observed quickly; later audits repeat at the configured interval.
        const int dueFrames = _omodCoverageAuditRunIndex == 0 ? (std::min)(90, intervalFrames) : intervalFrames;
        if (++_omodCoverageAuditFrameCounter < dueFrames) {
            return;
        }
        _omodCoverageAuditFrameCounter = 0;
        const std::uint32_t runIndex = _omodCoverageAuditRunIndex++;

        auto* player = f4vr::getPlayer();
        auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
        auto* equipData = processData ? processData->equipData : nullptr;
        auto* weaponForm = equipData ? equipData->item : nullptr;

        WeaponVisualKeyStats visualStatsNow{};
        const std::uint64_t visualKeyNow = getWeaponVisualCompositionKey(weaponNode, visualStatsNow);
        const bool visualDrift = visualKeyNow != 0 && _cachedWeaponVisualKey != 0 && visualKeyNow != _cachedWeaponVisualKey;

        ROCK_LOG_INFO(Weapon,
            "OMOD-AUDIT begin run={} bodySetKey={:016X} weapon={:08X} '{}' bodies={} visualKeyNow={:016X} visualKeyAtBuild={:016X} drift={} visibleTriShapes={} nodes={} invisibleNodes={}",
            runIndex,
            _cachedWeaponBodySetKey,
            weaponForm ? weaponForm->formID : 0u,
            weaponForm && weaponForm->GetFullName() ? weaponForm->GetFullName() : "",
            getWeaponBodyCount(),
            visualKeyNow,
            _cachedWeaponVisualKey,
            visualDrift ? "YES" : "no",
            visualStatsNow.visibleTriShapeCount,
            visualStatsNow.nodeCount,
            visualStatsNow.invisibleNodeCount);

        /*
         * Stored sourceNode pointers are compared by address during tree walks
         * only, never dereferenced directly, matching the discipline of the
         * build-time dump.
         */
        std::unordered_set<std::uintptr_t> evidenceSourceAddresses;
        std::unordered_map<std::uint32_t, std::uint32_t> bodiesByAttachPointFormId;
        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid()) {
                continue;
            }
            if (instance.sourceNode) {
                evidenceSourceAddresses.insert(reinterpret_cast<std::uintptr_t>(instance.sourceNode));
            }
            if (instance.semantic.attachPointFormId != 0) {
                ++bodiesByAttachPointFormId[instance.semantic.attachPointFormId];
            }
        }

        std::vector<OmodAuditRecord> records;
        const RE::BGSObjectInstanceExtra* objectInstanceExtra =
            weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData) : nullptr;
        if (objectInstanceExtra && objectInstanceExtra->values) {
            const auto indexData = objectInstanceExtra->GetIndexData();
            records.reserve(indexData.size());
            for (const auto& modIndex : indexData) {
                OmodAuditRecord record{};
                record.modIndex = modIndex.index;
                record.rank = modIndex.rank;
                record.disabled = modIndex.disabled;
                record.formId = modIndex.objectID;
                if (auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID)) {
                    record.resolved = true;
                    record.formId = omod->formID;
                    record.attachPointIndex = omod->attachPoint.keywordIndex;
                    const RE::BGSKeyword* attachPointKeyword =
                        RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, record.attachPointIndex);
                    record.attachPointFormId = attachPointKeyword ? attachPointKeyword->formID : 0u;
                    record.name = omod->fullName.c_str() ? omod->fullName.c_str() : "";
                    record.modelPath = omod->model.c_str() ? omod->model.c_str() : "";
                }
                records.push_back(std::move(record));
            }
        } else {
            ROCK_LOG_INFO(Weapon, "OMOD-AUDIT run={} no object instance extra available", runIndex);
        }

        std::vector<OmodAuditTokenSlot> tokenSlots(records.size());
        for (std::size_t i = 0; i < records.size(); ++i) {
            tokenSlots[i].lowerToken = makeOmodAuditModelToken(records[i].modelPath.c_str());
            tokenSlots[i].lowerWords = makeOmodAuditTokenWords(tokenSlots[i].lowerToken);
        }
        /*
         * Extra census slot: assembled weapon roots are named
         * 'Weapon  (<formID>)'. The 2026-07-04 session proved the game keeps
         * several parallel assembled instances (different addresses, identical
         * paths) that disagree about which OMOD subtrees exist, while the
         * renderer displays parts absent from the instances ROCK harvests.
         * Counting every instance across roots, with per-instance subtree
         * stats, identifies which copy is complete.
         */
        if (weaponForm) {
            tokenSlots.push_back(OmodAuditTokenSlot{ fmt::format("({:08x})", weaponForm->formID), {} });
        }
        std::vector<OmodAuditNodeMatch> connectPointMatches;

        struct OmodAuditRoot
        {
            const char* label;
            RE::NiAVObject* root;
            std::size_t maxVisited;
        };
        auto* playerNodes = f4vr::getPlayerNodes();
        std::vector<OmodAuditRoot> roots;
        roots.reserve(6);
        const auto addRoot = [&roots](const char* label, RE::NiAVObject* root, std::size_t maxVisited) {
            if (!root) {
                return;
            }
            for (const auto& existing : roots) {
                if (existing.root == root) {
                    return;
                }
            }
            roots.push_back(OmodAuditRoot{ label, root, maxVisited });
        };
        // Weapon-local roots stay on the shared dump budget; the skeleton and
        // full scene roots get a deep budget because the rendered weapon
        // instance may sit beyond 4096 nodes (cap saturation is logged below).
        constexpr std::size_t kOmodAuditDeepRootMaxVisited = 32768;
        // 2026-07-04 session 2 proved the renderer draws a weapon copy that is
        // in NEITHER instance reachable from the roots below (parts render
        // while absent, and the whole weapon can vanish while both instances
        // stay visible-flagged). The absolute scene root — reached by climbing
        // parents from the update weapon node to the top 'WorldRoot Node' —
        // covers everything parented into the loaded scene and gets a very
        // deep budget to find that copy.
        constexpr std::size_t kOmodAuditSceneRootMaxVisited = 262144;
        addRoot("updateWeaponNode", weaponNode, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("firstPersonSkeleton:Weapon", f4vr::getWeaponNode(), WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.primaryWeapontoWeaponNode", playerNodes ? playerNodes->primaryWeapontoWeaponNode : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.primaryWeaponOffsetNode", playerNodes ? playerNodes->primaryWeaponOffsetNOde : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("PlayerNodes.playerworldnode", playerNodes ? playerNodes->playerworldnode : nullptr, kOmodAuditDeepRootMaxVisited);
        addRoot("PlayerNodes.roomnode", playerNodes ? playerNodes->roomnode : nullptr, WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES);
        addRoot("firstPersonSkeleton", f4vr::getFirstPersonSkeleton(), kOmodAuditDeepRootMaxVisited);
        addRoot("playerFadeRootNode", f4vr::getWorldRootNode(), kOmodAuditDeepRootMaxVisited);
        addRoot("gameRootNode", f4vr::getRootNode(), kOmodAuditDeepRootMaxVisited);
        RE::NiAVObject* absoluteSceneRoot = weaponNode;
        for (int hop = 0; hop < 64 && absoluteSceneRoot->parent; ++hop) {
            absoluteSceneRoot = absoluteSceneRoot->parent;
        }
        addRoot("absoluteSceneRoot", absoluteSceneRoot, kOmodAuditSceneRootMaxVisited);

        for (const auto& root : roots) {
            std::size_t visited = 0;
            scanOmodAuditTreeRecursive(root.root, 0, visited, root.maxVisited, root.label, tokenSlots, connectPointMatches);
            ROCK_LOG_INFO(Weapon,
                "OMOD-AUDIT scan root='{}' addr={:x} visitedNodes={} capHit={}",
                root.label,
                reinterpret_cast<std::uintptr_t>(root.root),
                visited,
                visited >= root.maxVisited ? "YES" : "no");
        }

        std::vector<std::size_t> selfHealCandidates;
        for (std::size_t i = 0; i < records.size(); ++i) {
            const auto& record = records[i];
            const auto& slot = tokenSlots[i];

            std::uint32_t pairedBodies = 0;
            if (record.attachPointFormId != 0) {
                const auto pairedIt = bodiesByAttachPointFormId.find(record.attachPointFormId);
                if (pairedIt != bodiesByAttachPointFormId.end()) {
                    pairedBodies = pairedIt->second;
                }
            }

            std::size_t evidenceUnderMatches = 0;
            bool anyMatchVisible = false;
            std::size_t loggedMatches = 0;
            for (const auto& match : slot.matches) {
                const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                std::size_t evidenceVisited = 0;
                const std::size_t evidenceSources =
                    countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                evidenceUnderMatches += evidenceSources;
                const bool matchVisible = weaponVisualNodeVisible(match.node);
                anyMatchVisible = anyMatchVisible || matchVisible || stats.visibleTriShapeCount > 0;
                if (loggedMatches < OMOD_AUDIT_MAX_LOGGED_MATCHES_PER_OMOD) {
                    ++loggedMatches;
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-AUDIT match omod={:08X} root='{}' path='{}' addr={:x} visible={} flags=0x{:X} appCulled={} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={}",
                        record.formId,
                        match.rootLabel,
                        buildOmodAuditNodePath(match.node),
                        reinterpret_cast<std::uintptr_t>(match.node),
                        matchVisible ? "yes" : "no",
                        static_cast<std::uint32_t>(match.node->flags.flags),
                        match.node->GetAppCulled() ? "yes" : "no",
                        stats.nodeCount,
                        stats.triShapeCount,
                        stats.visibleTriShapeCount,
                        stats.hiddenFlagCount,
                        stats.appCulledCount,
                        evidenceSources);
                }
            }

            const char* verdict = nullptr;
            if (!record.resolved) {
                verdict = "UNRESOLVED_FORM";
            } else if (slot.lowerToken.empty()) {
                verdict = pairedBodies > 0 ? "OK_RECORD_PAIRED" : "NO_MODEL";
            } else if (evidenceUnderMatches > 0) {
                verdict = "OK";
            } else if (pairedBodies > 0) {
                verdict = "OK_RECORD_PAIRED";
            } else if (!slot.matches.empty()) {
                verdict = anyMatchVisible ? "NODE_PRESENT_NO_COLLIDER" : "NODE_HIDDEN_NO_COLLIDER";
            } else {
                verdict = "NODE_NOT_FOUND";
                selfHealCandidates.push_back(i);
            }

            ROCK_LOG_INFO(Weapon,
                "OMOD-AUDIT omod={:08X} '{}' index={} rank={} disabled={} attachPoint={:08X} attachPointIndex={} model='{}' token='{}' pairedBodies={} nodeMatches={} evidenceUnderMatches={} verdict={}",
                record.formId,
                record.name,
                record.modIndex,
                record.rank,
                record.disabled,
                record.attachPointFormId,
                record.attachPointIndex,
                record.modelPath,
                slot.lowerToken,
                pairedBodies,
                slot.matches.size(),
                evidenceUnderMatches,
                verdict);
        }

        for (const auto& match : connectPointMatches) {
            const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
            std::size_t evidenceVisited = 0;
            const std::size_t evidenceSources =
                countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
            ROCK_LOG_INFO(Weapon,
                "OMOD-AUDIT pnode name='{}' root='{}' path='{}' addr={:x} visible={} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={} childNames='{}'",
                safeNodeName(match.node),
                match.rootLabel,
                buildOmodAuditNodePath(match.node),
                reinterpret_cast<std::uintptr_t>(match.node),
                weaponVisualNodeVisible(match.node) ? "yes" : "no",
                stats.nodeCount,
                stats.triShapeCount,
                stats.visibleTriShapeCount,
                stats.hiddenFlagCount,
                stats.appCulledCount,
                evidenceSources,
                weaponAnimNodeImmediateChildNames(match.node));
        }

        std::size_t weaponInstanceCount = 0;
        if (weaponForm && tokenSlots.size() > records.size()) {
            const auto& instanceSlot = tokenSlots.back();
            weaponInstanceCount = instanceSlot.matches.size();
            for (const auto& match : instanceSlot.matches) {
                const auto stats = summarizeWeaponAnimNodeSubtree(match.node);
                std::size_t evidenceVisited = 0;
                const std::size_t evidenceSources =
                    countOmodAuditEvidenceSourcesInSubtree(match.node, evidenceSourceAddresses, evidenceVisited);
                ROCK_LOG_INFO(Weapon,
                    "OMOD-AUDIT instance name='{}' root='{}' path='{}' addr={:x} visible={} flags=0x{:X} appCulled={} subtreeNodes={} triShapes={} visibleTriShapes={} hiddenFlags={} appCulledNodes={} evidenceSources={} childNames='{}'",
                    safeNodeName(match.node),
                    match.rootLabel,
                    buildOmodAuditNodePath(match.node),
                    reinterpret_cast<std::uintptr_t>(match.node),
                    weaponVisualNodeVisible(match.node) ? "yes" : "no",
                    static_cast<std::uint32_t>(match.node->flags.flags),
                    match.node->GetAppCulled() ? "yes" : "no",
                    stats.nodeCount,
                    stats.triShapeCount,
                    stats.visibleTriShapeCount,
                    stats.hiddenFlagCount,
                    stats.appCulledCount,
                    evidenceSources,
                    weaponAnimNodeImmediateChildNames(match.node));
            }
        }

        /*
         * Flattened-bone-tree pass. Actor skeleton roots are BSFlattenedBoneTree
         * objects whose bones live in a flat transforms array, not as NiNode
         * children — a part that exists only as a flattened entry (plus skinned
         * render geometry) renders on screen while every child walk above
         * misses it. Session 3 (2026-07-04) proved the full scene graph holds
         * only the two incomplete instances, so this array is the remaining
         * candidate for where the rendered copy of a missing part lives.
         */
        std::size_t flatMatchCount = 0;
        struct OmodAuditFlatRoot
        {
            const char* label;
            f4vr::BSFlattenedBoneTree* tree;
        };
        const std::array<OmodAuditFlatRoot, 2> flatRoots{ {
            { "gameFlattenedBoneTree", f4vr::getFlattenedBoneTree() },
            { "firstPersonBoneTree", f4vr::getFirstPersonBoneTree() },
        } };
        constexpr std::size_t OMOD_AUDIT_MAX_FLAT_MATCHES = 48;
        for (const auto& flatRoot : flatRoots) {
            if (!weaponAnimFlattenedTreeValid(flatRoot.tree)) {
                continue;
            }
            for (int index = 0; index < flatRoot.tree->numTransforms && flatMatchCount < OMOD_AUDIT_MAX_FLAT_MATCHES; ++index) {
                const auto& transform = flatRoot.tree->transforms[index];
                const char* boneName = transform.name.c_str();
                auto* refNode = transform.refNode;
                const char* refNodeName = safeNodeName(refNode);
                bool matched = omodAuditNameIsConnectPoint(boneName) || omodAuditNameIsConnectPoint(refNodeName);
                if (!matched) {
                    for (const auto& slot : tokenSlots) {
                        if (slot.lowerToken.empty()) {
                            continue;
                        }
                        if (omodAuditNameContainsToken(boneName, slot.lowerToken) ||
                            omodAuditNameContainsToken(refNodeName, slot.lowerToken)) {
                            matched = true;
                            break;
                        }
                    }
                }
                if (!matched) {
                    continue;
                }
                ++flatMatchCount;
                ROCK_LOG_INFO(Weapon,
                    "OMOD-AUDIT flatbone root='{}' index={} name='{}' parentIndex={} parentName='{}' refNode={:x} refNodeName='{}' refParent='{}' refVisible={} worldT=({:.2f},{:.2f},{:.2f})",
                    flatRoot.label,
                    index,
                    boneName && boneName[0] != '\0' ? boneName : "(unnamed)",
                    transform.parPos,
                    weaponAnimFlattenedParentName(flatRoot.tree, transform.parPos),
                    reinterpret_cast<std::uintptr_t>(refNode),
                    refNodeName,
                    refNode ? safeNodeName(refNode->parent) : "(null)",
                    refNode && weaponVisualNodeVisible(refNode) ? "yes" : "no",
                    transform.world.translate.x,
                    transform.world.translate.y,
                    transform.world.translate.z);
            }
        }

        /*
         * Self-heal (bDebugWeaponOmodSelfHeal): reattach missing OMOD models
         * with the engine's own primitive. Ghidra-verified (raw disasm +
         * decompiler + address database, 2026-07-04):
         *   bool BGSMod::Attachment::Mod::TryAttach3DRecurse(
         *       NiNode* root, char* rankSuffix, TBO_InstanceData* instData)
         * at VR offset 0x2D9140. It demands the mod's model, deep-clones it
         * (scale from the root's REFR), applies material swaps with the
         * instance data, and attaches at the mod NIF's declared connect point
         * (BSConnectPoint::Parents) or the root as fallback. Fail-closed at
         * every hop; success requests a collider rebuild so the healed
         * geometry gets bodies. NODE_NOT_FOUND is the trigger, so the word-set
         * matcher above must keep false negatives rare - a heal on a part that
         * exists under an unmatchable name would duplicate its geometry, which
         * the once-per-instance-address guard bounds to a single attempt.
         */
        std::size_t selfHealAttemptCount = 0;
        std::size_t selfHealSuccessCount = 0;
        if (g_rockConfig.rockDebugWeaponOmodSelfHeal && !selfHealCandidates.empty()) {
            RE::NiNode* healTargetNode = nullptr;
            const char* healTargetRootLabel = "";
            if (weaponForm && tokenSlots.size() > records.size()) {
                for (const auto& match : tokenSlots.back().matches) {
                    auto* candidateNode = match.node ? match.node->IsNode() : nullptr;
                    if (!candidateNode) {
                        continue;
                    }
                    // Prefer the instance ROCK harvests colliders from.
                    if (!healTargetNode || std::strcmp(match.rootLabel, "updateWeaponNode") == 0) {
                        healTargetNode = candidateNode;
                        healTargetRootLabel = match.rootLabel;
                    }
                    if (std::strcmp(match.rootLabel, "updateWeaponNode") == 0) {
                        break;
                    }
                }
            }

            if (!healTargetNode) {
                ROCK_LOG_WARN(Weapon,
                    "OMOD-HEAL run={} skipped: no weapon instance node found for {} candidate(s)",
                    runIndex,
                    selfHealCandidates.size());
            } else {
                using TryAttach3DRecurseFn = bool (*)(RE::BGSMod::Attachment::Mod*, RE::NiNode*, const char*, RE::TBO_InstanceData*);
                static REL::Relocation<TryAttach3DRecurseFn> tryAttach3DRecurse{ REL::Offset(0x2D9140) };
                constexpr std::size_t OMOD_SELF_HEAL_MAX_PER_AUDIT = 4;

                if (_omodSelfHealAttempted.size() > 256) {
                    _omodSelfHealAttempted.clear();
                }
                for (const std::size_t candidateIndex : selfHealCandidates) {
                    if (selfHealAttemptCount >= OMOD_SELF_HEAL_MAX_PER_AUDIT) {
                        break;
                    }
                    const auto& record = records[candidateIndex];
                    const std::uint64_t attemptKey =
                        reinterpret_cast<std::uintptr_t>(healTargetNode) ^ (static_cast<std::uint64_t>(record.formId) << 20);
                    if (_omodSelfHealAttempted.contains(attemptKey)) {
                        continue;
                    }
                    _omodSelfHealAttempted.insert(attemptKey);

                    auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(record.formId);
                    if (!omod) {
                        ROCK_LOG_WARN(Weapon, "OMOD-HEAL run={} omod={:08X} skipped: form no longer resolves", runIndex, record.formId);
                        continue;
                    }

                    char rankSuffixBuffer[8] = {};
                    const char* rankSuffix = nullptr;
                    if (record.modIndex != 0) {
                        // Same suffix rule as the engine's own attach loop:
                        // non-zero index entries get a "%u" node-name suffix.
                        std::snprintf(rankSuffixBuffer, sizeof(rankSuffixBuffer), "%u", record.modIndex);
                        rankSuffix = rankSuffixBuffer;
                    }

                    const auto beforeStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                    ++selfHealAttemptCount;
                    const bool attached = tryAttach3DRecurse(omod, healTargetNode, rankSuffix, equipData ? equipData->instanceData : nullptr);
                    const auto afterStats = summarizeWeaponAnimNodeSubtree(healTargetNode);
                    selfHealSuccessCount += attached ? 1 : 0;

                    ROCK_LOG_INFO(Weapon,
                        "OMOD-HEAL run={} omod={:08X} '{}' model='{}' suffix='{}' target='{}'/{:x} attached={} subtreeNodes {}->{} triShapes {}->{} visibleTriShapes {}->{}",
                        runIndex,
                        record.formId,
                        record.name,
                        record.modelPath,
                        rankSuffix ? rankSuffix : "",
                        healTargetRootLabel,
                        reinterpret_cast<std::uintptr_t>(healTargetNode),
                        attached ? "YES" : "no",
                        beforeStats.nodeCount,
                        afterStats.nodeCount,
                        beforeStats.triShapeCount,
                        afterStats.triShapeCount,
                        beforeStats.visibleTriShapeCount,
                        afterStats.visibleTriShapeCount);
                }

                if (selfHealSuccessCount > 0) {
                    ROCK_LOG_INFO(Weapon,
                        "OMOD-HEAL run={} healed={} of {} attempted - requesting collider rebuild",
                        runIndex,
                        selfHealSuccessCount,
                        selfHealAttemptCount);
                    requestWorkbenchExitRebuild();
                }
            }
        }

        ROCK_LOG_INFO(Weapon,
            "OMOD-AUDIT end run={} bodySetKey={:016X} installedMods={} connectPoints={} weaponInstances={} flatMatches={} healCandidates={} healAttempted={} healed={}",
            runIndex,
            _cachedWeaponBodySetKey,
            records.size(),
            connectPointMatches.size(),
            weaponInstanceCount,
            flatMatchCount,
            selfHealCandidates.size(),
            selfHealAttemptCount,
            selfHealSuccessCount);
    }

    void WeaponCollision::publishSampledVelocityAtomic(std::uint32_t publicationIndex, const GeneratedKeyframedBodyDriveQueueResult& queueResult)
    {
        if (publicationIndex >= MAX_WEAPON_BODIES || publicationIndex >= _weaponBodyCountAtomic.load(std::memory_order_acquire)) {
            return;
        }

        if (!queueResult.sampledVelocityValid) {
            _weaponBodySampledVelocityValidAtomic[publicationIndex].store(0, std::memory_order_release);
            _weaponBodySampledVelocityHavokXAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            _weaponBodySampledVelocityHavokYAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            _weaponBodySampledVelocityHavokZAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            return;
        }

        _weaponBodySampledVelocityHavokXAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.x, std::memory_order_release);
        _weaponBodySampledVelocityHavokYAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.y, std::memory_order_release);
        _weaponBodySampledVelocityHavokZAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.z, std::memory_order_release);
        _weaponBodySampledVelocityValidAtomic[publicationIndex].store(1, std::memory_order_release);
    }

    void WeaponCollision::updateBodiesFromCurrentSourceTransforms(
        RE::hknpWorld* world,
        RE::NiAVObject* fallbackWeaponNode,
        float sourceDeltaSeconds,
        const RE::NiAVObject* const* drivenSourceNodes,
        std::size_t drivenSourceNodeCount)
    {
        if (!world || !hasWeaponBody() || getCurrentWeaponGenerationKey() == 0) {
            return;
        }

        auto& bank = activeWeaponBodies();
        RE::NiAVObject* cachedPackageDriveNode = resolvePackageDriveNode(bank, nullptr);
        RE::NiAVObject* packageDriveNode = fallbackWeaponNode ? fallbackWeaponNode : cachedPackageDriveNode;
        if (!packageDriveNode) {
            return;
        }
        const RE::NiTransform packageWorld = packageDriveNode->world;
        const bool packageRootDiffersFromCached = cachedPackageDriveNode && cachedPackageDriveNode != packageDriveNode;
        bool updatedPublishedRoots = false;

        (void)drivenSourceNodes;
        (void)drivenSourceNodeCount;

        for (std::size_t i = 0; i < bank.size(); ++i) {
            auto& instance = bank[i];
            if (!instance.body.isValid()) {
                continue;
            }

            if (instance.driveNode && instance.driveNode != packageDriveNode) {
                ROCK_LOG_SAMPLE_WARN(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon package drive root mismatch bodyId={} bodyRoot=0x{:X} packageRoot='{}' packageRootAddr=0x{:X} sourceRoot='{}' - using current package root for motion",
                    instance.body.getBodyId().value,
                    static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(instance.driveNode)),
                    safeNodeName(packageDriveNode),
                    static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(packageDriveNode)),
                    instance.sourceRootName);
            }

            if (packageRootDiffersFromCached && instance.publicationIndex < MAX_WEAPON_BODIES) {
                if (!updatedPublishedRoots) {
                    beginWeaponBodyPublication();
                    updatedPublishedRoots = true;
                }
                _weaponBodyInteractionRootsAtomic[instance.publicationIndex].store(reinterpret_cast<std::uintptr_t>(packageDriveNode), std::memory_order_release);
                if (instance.driveNode && instance.driveNode != packageDriveNode) {
                    _weaponBodySourceRootsAtomic[instance.publicationIndex].store(0, std::memory_order_release);
                }
            }

            const bool useSourceNode = instance.sourceNode && actor_equipment_grab::nodeContainsNode(packageDriveNode, instance.sourceNode, 64);
            const RE::NiTransform& driveWorld = useSourceNode ? instance.sourceNode->world : packageWorld;
            const RE::NiPoint3& centerGame = useSourceNode ? instance.generatedSourceLocalCenterGame : instance.generatedLocalCenterGame;
            const RE::NiTransform generatedTransform = makeGeneratedBodyWorldTransform(driveWorld, centerGame);
            queueBodyTarget(instance, generatedTransform, sourceDeltaSeconds);
        }

        if (updatedPublishedRoots) {
            endWeaponBodyPublication();
        }

    }

    void WeaponCollision::queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float sourceDeltaSeconds)
    {
        if (!instance.body.isValid()) {
            return;
        }

        const auto queueResult = queueGeneratedKeyframedBodyTarget(instance.driveState, weaponTransform, sourceDeltaSeconds, 1000.0f);
        publishSampledVelocityAtomic(instance.publicationIndex, queueResult);
    }

    void WeaponCollision::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !hasWeaponBody() || getCurrentWeaponGenerationKey() == 0) {
            return;
        }

        auto& bank = activeWeaponBodies();

        for (std::size_t i = 0; i < bank.size(); ++i) {
            auto& instance = bank[i];
            if (!instance.body.isValid()) {
                continue;
            }
            const auto bodyIndex = static_cast<std::uint32_t>(i);
            handleGeneratedBodyDriveResult(
                driveGeneratedKeyframedBody(world,
                    instance.body,
                    instance.driveState,
                    timing,
                    "weapon-collision",
                    bodyIndex,
                    g_rockConfig.rockWeaponCollisionMaxLinearVelocity,
                    g_rockConfig.rockWeaponCollisionMaxAngularVelocity),
                "weapon-collision",
                bodyIndex);
        }
    }

    void WeaponCollision::serviceRetiredWeaponBodies(std::uint32_t completedPhysicsSteps)
    {
        if (completedPhysicsSteps == 0) {
            return;
        }

        std::scoped_lock lock(_retiredWeaponBodyPayloadMutex);
        for (auto& retired : _retiredWeaponBodyPayloads) {
            if (!retired.occupied()) {
                continue;
            }

            retired.remainingPhysicsSteps =
                retired.remainingPhysicsSteps > completedPhysicsSteps ? retired.remainingPhysicsSteps - completedPhysicsSteps : 0;
            if (retired.remainingPhysicsSteps != 0) {
                continue;
            }

            const auto bodyId = retired.bodyPayload.bodyId;
            BethesdaPhysicsBody::releaseRetiredPayload(retired.bodyPayload);
            retired = {};
            if (_retiredWeaponBodyPayloadCount > 0) {
                --_retiredWeaponBodyPayloadCount;
            }
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                1000,
                "Retired generated weapon body {} payload reclaimed activeRetired={}",
                bodyId,
                _retiredWeaponBodyPayloadCount);
        }
    }

    void WeaponCollision::handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex)
    {
        if (!result.attempted || result.skippedStale) {
            return;
        }

        if (result.driven) {
            _driveFailureCount.store(0, std::memory_order_release);
            return;
        }

        if (!result.shouldRequestRebuild()) {
            return;
        }

        const auto failures = _driveFailureCount.fetch_add(1, std::memory_order_acq_rel) + 1;
        _driveRebuildRequested.store(true, std::memory_order_release);
        ROCK_LOG_SAMPLE_WARN(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "Weapon generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} ownerMismatch={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f} bodyRotErr={:.2f}",
            ownerName ? ownerName : "unknown",
            bodyIndex,
            failures,
            result.missingBody ? "yes" : "no",
            result.bodyCollisionObjectMismatch ? "yes" : "no",
            result.placementFailed ? "yes" : "no",
            result.nativeDriveFailed ? "yes" : "no",
            result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f,
            result.hasLiveBodyTransform ? result.targetToBodyRotationDegrees : -1.0f);
    }
}
