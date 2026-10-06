#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// Collision diagnostics: the weapon animation-node dump.

namespace rock
{
    void WeaponCollision::maybeDumpWeaponAnimNodeDiagnostics(RE::NiAVObject* updateWeaponNode, std::uint64_t observedKey)
    {
        if (!g_rockConfig.rockDebugDumpWeaponAnimNodes) {
            _diagnostics.animNodeDumpFrameCounter = 0;
            _diagnostics.lastAnimNodeDumpKey = 0;
            return;
        }

        const bool generationChanged = observedKey != _diagnostics.lastAnimNodeDumpKey;
        const int intervalFrames = (std::max)(1, g_rockConfig.rockDebugWeaponAnimNodeDumpIntervalFrames);
        const bool intervalDue = ++_diagnostics.animNodeDumpFrameCounter >= intervalFrames;
        if (!generationChanged && !intervalDue) {
            return;
        }

        _diagnostics.animNodeDumpFrameCounter = 0;
        _diagnostics.lastAnimNodeDumpKey = observedKey;

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
}
