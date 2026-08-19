#include "physics-interaction/weapon/collision/WeaponSceneGraphWalk.h"

#include "RE/NetImmerse/NiNode.h"

#include <algorithm>
#include <cstring>

namespace rock::weapon_collision_detail
{
    bool weaponAnimNodeNameMatches(const RE::NiAVObject* node, const char* targetName)
    {
        if (!node || !targetName) {
            return false;
        }
        return _stricmp(targetName, node->name.c_str()) == 0;
    }

    // Recursive worker; internal because callers only ever want the whole match
    // list, never a partial one.
    namespace
    {
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
    }

    std::vector<WeaponAnimNodeMatch> collectWeaponAnimNodeMatches(RE::NiAVObject* root, const char* targetName)
    {
        std::vector<WeaponAnimNodeMatch> matches;
        std::size_t visited = 0;
        collectWeaponAnimNodeMatchesRecursive(root, targetName, {}, 0, visited, matches);
        return matches;
    }

    // Recursive worker for the census below.
    namespace
    {
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
}
