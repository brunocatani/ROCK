#include "physics-interaction/weapon/collision/WeaponCollision.h"

/*
 * Weapon emitter discovery: the lights, lasers and reticles bolted onto a weapon.
 *
 * An emitter is not collision. ROCK tracks these because consumers need to know
 * where a laser or a flashlight points and whether it is currently on, and because
 * effect geometry must be kept OUT of collider generation - the same node that
 * makes a good emitter descriptor makes a terrible hull.
 *
 * The scan runs over the assembled weapon and produces one bounded snapshot:
 *   discover  - walk the weapon, recognize BSValueNode and effect geometry
 *   attribute - resolve each emitter's structural context and its installing OMOD
 *   merge     - fold duplicates that describe the same physical emitter
 *   refresh   - re-read visibility and transforms for an existing snapshot,
 *               without rediscovering anything
 *
 * The snapshot is identity-stamped with the equipped key, the body-set generation
 * and the scanned root set, so a consumer can tell a stale snapshot from a current
 * one without walking the tree again.
 *
 * collectWeaponEmitterSnapshot is the ONLY thing exported (WeaponEmitterScan.h).
 * The predicates, the merge rules and the descriptor lifetime stay private on
 * purpose, so the generated-source scan cannot start making emitter policy.
 */

#include "physics-interaction/weapon/collision/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/collision/WeaponEmitterScan.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/collision/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/collision/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/parts/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>

namespace rock
{
    using namespace weapon_collision_detail;

    namespace
    {
        // Bool view of the visibility walk: visible only if nothing on the chain is
        // hidden AND the whole chain was actually reached. Fails closed on both.
        [[nodiscard]] bool weaponEmitterNodeEffectivelyVisible(const RE::NiAVObject* node)
        {
            if (!node) {
                return false;
            }
            const auto walk = walkWeaponVisibilityChain(node, kWeaponEmitterVisibilityAncestorSteps);
            return !walk.firstHidden && !walk.boundExhausted;
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
            const std::uint16_t count = (std::min)(children.size(), static_cast<std::uint16_t>(64));
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

        [[nodiscard]] bool isWeaponAttachmentPointNode(std::string_view name)
        {
            return name.starts_with("P-");
        }

        [[nodiscard]] WeaponEmitterStructuralContext resolveWeaponEmitterStructuralContext(
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
        bool updateWeaponEmitterTransform(
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

        void copyWeaponEmitterSourceName(WeaponEmitterDescriptor& descriptor, std::string_view name)
        {
            descriptor.sourceName.fill('\0');
            const std::size_t copyCount = (std::min)(name.size(), descriptor.sourceName.size() - 1);
            if (copyCount != 0) {
                std::memcpy(descriptor.sourceName.data(), name.data(), copyCount);
            }
        }

        [[nodiscard]] bool weaponEmitterDescriptorsShareOwner(
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

        void mergeWeaponEmitterDescriptor(WeaponEmitterSnapshot& snapshot, const WeaponEmitterDescriptor& candidate)
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

        void collectWeaponEmittersRecursive(
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
            for (std::uint16_t i = 0; i < children.size(); ++i) {
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

    }

    // The emitter scan's only cross-file entry point (WeaponEmitterScan.h), so it
    // needs external linkage; everything else about emitters stays file-local.
    namespace weapon_collision_detail
    {
        [[nodiscard]] WeaponEmitterSnapshot collectWeaponEmitterSnapshot(
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
    }

    namespace
    {
        void refreshWeaponEmittersRecursive(
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
            for (std::uint16_t i = 0; i < children.size(); ++i) {
                refreshWeaponEmittersRecursive(children[i].get(), weaponRoot, depth + 1, visitedNodes, snapshot);
            }
        }
    }

    WeaponEmitterSnapshot WeaponCollision::buildWeaponEmitterSnapshot(
        RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey,
        std::uint64_t weaponGenerationKey,
        std::uint64_t rootSetKey) const
    {
        const auto omodByAttachPointFormId = readEquippedOmodsByAttachPointFormId();
        auto snapshot = collectWeaponEmitterSnapshot(
            weaponNode,
            omodByAttachPointFormId,
            equippedWeaponKey,
            weaponGenerationKey,
            rootSetKey);

        ROCK_LOG_DEBUG(Weapon,
            "Weapon emitter snapshot discovered generation={:016X} emitters={} roots={:016X}",
            weaponGenerationKey,
            snapshot.count,
            rootSetKey);
        return snapshot;
    }

    void WeaponCollision::updateWeaponEmitterSnapshot(RE::NiAVObject* weaponNode, std::uint64_t equippedWeaponKey)
    {
        const std::uint64_t weaponGenerationKey = getCurrentWeaponGenerationKey();
        if (!weaponNode || equippedWeaponKey == 0 || weaponGenerationKey == 0 || _cachedWeaponKey != equippedWeaponKey) {
            clearWeaponEmitterSnapshot();
            return;
        }

        const std::uint64_t rootSetKey = makeWeaponEmitterRootSetKey(weaponNode);
        WeaponEmitterSnapshot snapshot{};
        {
            std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
            snapshot = _weaponEmitterSnapshot;
        }

        const bool discoveryRequired = snapshot.weaponGenerationKey != weaponGenerationKey ||
            snapshot.equippedWeaponKey != equippedWeaponKey ||
            snapshot.rootSetKey != rootSetKey ||
            snapshot.weaponRootAddress != reinterpret_cast<std::uintptr_t>(weaponNode);
        if (discoveryRequired) {
            snapshot = buildWeaponEmitterSnapshot(weaponNode, equippedWeaponKey, weaponGenerationKey, rootSetKey);
        } else {
            for (std::size_t i = 0; i < snapshot.count; ++i) {
                snapshot.emitters[i].active = false;
                snapshot.emitters[i].visible = false;
            }
            visitGeneratedWeaponMeshRootCandidates(weaponNode, [&](const WeaponMeshRootCandidate& candidate) {
                std::uint32_t visitedNodes = 0;
                refreshWeaponEmittersRecursive(candidate.root, weaponNode, 0, visitedNodes, snapshot);
            });
        }

        std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
        _weaponEmitterSnapshot = snapshot;
    }

    void WeaponCollision::clearWeaponEmitterSnapshot()
    {
        std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
        _weaponEmitterSnapshot = {};
    }

    WeaponEmitterSnapshot WeaponCollision::getWeaponEmitterSnapshot() const
    {
        std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
        return _weaponEmitterSnapshot;
    }
}
