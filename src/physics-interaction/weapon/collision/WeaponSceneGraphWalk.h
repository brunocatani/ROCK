#pragma once

/*
 * Named-node lookup and subtree summarization over the live weapon scene graph
 * and over BSFlattenedBoneTree.
 *
 * These read like debug-dump helpers, and the anim-node diagnostics do use them,
 * but the OMOD self-heal PRODUCTION path depends on them for node lookup at 14+
 * call sites. They were extracted before the diagnostics were separated from the
 * audit machinery precisely so that separation could not sever a production
 * dependency by accident.
 *
 * Every walk here is bounded by the WEAPON_ANIM_NODE_DUMP_* budgets in
 * WeaponCollisionInternal.h, and nothing retains an engine pointer past the
 * caller's frame.
 *
 * INTERNAL. Never reachable from any public include tree.
 */

#include "physics-interaction/weapon/collision/WeaponCollisionInternal.h"

#include "rock_support/Fo4VrRuntime.h"

#include <cstdint>
#include <string>
#include <vector>

namespace rock::weapon_collision_detail
{
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

    struct WeaponAnimFlattenedBoneMatch
    {
        int index{ -1 };
        int parentIndex{ -1 };
        short childPosition{ -1 };
        RE::NiNode* refNode{ nullptr };
        std::string name;
    };

    // Case-insensitive name comparison - the one test every lookup below uses.
    bool weaponAnimNodeNameMatches(const RE::NiAVObject* node, const char* targetName);

    // Every node under `root` whose name matches, each with its path from the
    // root. Bounded by the visited-node, depth and match-count budgets.
    std::vector<WeaponAnimNodeMatch> collectWeaponAnimNodeMatches(RE::NiAVObject* root, const char* targetName);

    // Shape and visibility census of one subtree - what the OMOD audit compares
    // before and after a heal.
    WeaponAnimNodeSubtreeStats summarizeWeaponAnimNodeSubtree(RE::NiAVObject* node);

    // "childA|childB|+3 more", for one log line.
    std::string weaponAnimNodeImmediateChildNames(RE::NiAVObject* node);

    /*
     * BSFlattenedBoneTree reads. The tree is a native array behind a raw pointer,
     * so validity is re-checked before every access and the transform count is
     * capped: a corrupt tree has to produce "(none)", never a fault.
     */
    bool weaponAnimFlattenedTreeValid(const f4vr::BSFlattenedBoneTree* tree);
    const char* weaponAnimFlattenedTransformName(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform);
    bool weaponAnimFlattenedTransformNameMatches(const f4vr::BSFlattenedBoneTree::BoneTransforms& transform, const char* targetName);
    std::vector<WeaponAnimFlattenedBoneMatch> collectWeaponAnimFlattenedBoneMatches(
        f4vr::BSFlattenedBoneTree* tree,
        const char* targetName);
    const char* weaponAnimFlattenedParentName(const f4vr::BSFlattenedBoneTree* tree, int parentIndex);
}
