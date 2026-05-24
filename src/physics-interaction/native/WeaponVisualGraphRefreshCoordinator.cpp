#include "physics-interaction/native/WeaponVisualGraphRefreshCoordinator.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/WeaponEquippedModSignature.h"
#include "physics-interaction/native/WeaponWorkbenchGraphRefreshHook.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace rock
{
    namespace
    {
        struct WeaponNodeStats
        {
            std::uint32_t childCount{ 0 };
            std::uint32_t nodeCount{ 0 };
            std::uint32_t triShapeCount{ 0 };
        };

        void accumulateWeaponNodeStats(RE::NiAVObject* node, WeaponNodeStats& stats, std::uint32_t depth = 0)
        {
            if (!node || depth > 32 || stats.nodeCount >= 4096) {
                return;
            }

            ++stats.nodeCount;
            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            const auto& children = niNode->children;
            if (depth == 0) {
                stats.childCount = static_cast<std::uint32_t>(
                    (std::min)(children.size(), static_cast<decltype(children.size())>((std::numeric_limits<std::uint32_t>::max)())));
            }
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    accumulateWeaponNodeStats(child, stats, depth + 1);
                }
            }
        }

        [[nodiscard]] WeaponNodeStats summarizeWeaponNode(RE::NiAVObject* node)
        {
            WeaponNodeStats stats{};
            accumulateWeaponNodeStats(node, stats);
            return stats;
        }

        [[nodiscard]] const char* safeNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }
            const char* name = node->name.c_str();
            return name ? name : "";
        }
    }

    void WeaponVisualGraphRefreshCoordinator::reset()
    {
        _lastSignatureKey = 0;
        _lastConsumedWorkbenchRefreshEpoch = weapon_workbench_graph_refresh::currentRefreshEpoch();
        _pendingWorkbenchRefreshEpoch = 0;
        _hasLastSignature = false;
        _pendingApplyFrames = 0;
        _postRefreshCollisionHoldFrames = 0;
    }

    WeaponVisualGraphRefreshCoordinator::UpdateResult WeaponVisualGraphRefreshCoordinator::update(const UpdateInput& input)
    {
        UpdateResult result{};

        if (!input.enabled || !input.visualAuthorityAvailable || !input.skeletonReady) {
            reset();
            return result;
        }

        if (_postRefreshCollisionHoldFrames > 0) {
            result.deferWeaponCollision = true;
            --_postRefreshCollisionHoldFrames;
        }

        const auto signature = readEquippedWeaponModSignature();
        result.signatureKey = signature.key;
        if (!input.menuBlocking) {
            weapon_workbench_graph_refresh::publishObservedEquippedWeaponSignature(signature.key);
        }

        if (signature.hasEquippedWeapon) {
            if (!_hasLastSignature) {
                _lastSignatureKey = signature.key;
                _hasLastSignature = true;
            } else if (signature.key != _lastSignatureKey) {
                _lastSignatureKey = signature.key;
                result.signatureChanged = true;
            }
        } else if (_hasLastSignature) {
            ROCK_LOG_DEBUG(Weapon, "Workbench equipped weapon graph refresh state cleared: no equipped weapon");
            _lastSignatureKey = 0;
            _hasLastSignature = false;
        }

        const auto refreshEpoch = weapon_workbench_graph_refresh::currentRefreshEpoch();
        if (refreshEpoch != 0 && refreshEpoch != _lastConsumedWorkbenchRefreshEpoch) {
            _pendingWorkbenchRefreshEpoch = refreshEpoch;
        }

        if (_pendingWorkbenchRefreshEpoch == 0) {
            return result;
        }

        ++_pendingApplyFrames;
        if (input.menuBlocking || !input.weaponDrawn || !input.weaponNode) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                1000,
                "Workbench equipped weapon graph refresh pending epoch={} menuBlocking={} weaponDrawn={} weaponNode={} pendingFrames={}",
                _pendingWorkbenchRefreshEpoch,
                input.menuBlocking,
                input.weaponDrawn,
                input.weaponNode != nullptr,
                _pendingApplyFrames);
            return result;
        }

        const auto beforeStats = summarizeWeaponNode(input.weaponNode);
        _lastConsumedWorkbenchRefreshEpoch = _pendingWorkbenchRefreshEpoch;
        _pendingWorkbenchRefreshEpoch = 0;
        _pendingApplyFrames = 0;
        _postRefreshCollisionHoldFrames = 1;
        result.refreshApplied = true;
        result.deferWeaponCollision = true;

        ROCK_LOG_INFO(Weapon,
            "Workbench equipped weapon graph refresh consumed epoch={} formID={:08X} signature={:016X} node='{}' nodePtr=0x{:016X} children={} nodes={} triShapes={}",
            _lastConsumedWorkbenchRefreshEpoch,
            signature.formID,
            signature.key,
            safeNodeName(input.weaponNode),
            reinterpret_cast<std::uintptr_t>(input.weaponNode),
            beforeStats.childCount,
            beforeStats.nodeCount,
            beforeStats.triShapeCount);
        return result;
    }
}
