#include "physics-interaction/weapon/WeaponCollision.h"

#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
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
#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/WeaponOmodAuditPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/WeaponEmitterScan.h"
#include "physics-interaction/weapon/WeaponSceneGraphWalk.h"
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

namespace rock
{
    /*
     * The shared WeaponCollision helpers live in a named detail namespace so they
     * cannot collide with the same-named local helpers other physics-interaction
     * TUs define. They are used unqualified throughout this file, exactly as they
     * were when they sat in the anonymous namespace below.
     */
    using namespace weapon_collision_detail;

    namespace
    {
        constexpr std::size_t GENERATED_WEAPON_BODY_CREATION_BATCH = 8;
        constexpr float GENERATED_SOURCE_COMPONENT_JOIN_TOLERANCE_GAME = 2.0f;
        constexpr float GENERATED_SOURCE_DETACHED_COMPONENT_MIN_GAP_GAME = 24.0f;
        constexpr std::size_t MAX_CACHED_DETACHED_SOURCE_GROUPS =
            weapon_collision_geometry_math::kMaxDetachedComponentAnalysisSources;

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

        std::string generatedWeaponSemanticMaskNames(std::uint32_t mask)
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

        RE::NiTransform makeIdentityTransform()
        {
            RE::NiTransform result{};
            result.rotate.entry[0][0] = 1.0f;
            result.rotate.entry[1][1] = 1.0f;
            result.rotate.entry[2][2] = 1.0f;
            result.scale = 1.0f;
            return result;
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

        weapon_collision_geometry_math::HullSelectionInput makeHullSelectionInput(const RE::NiPoint3& localCenterGame, const RE::NiPoint3& localMinGame,
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

        bool isAssembledWeaponComponentAnchor(WeaponPartKind partKind)
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

        // Min/max corner pair -> the (center, half-extent) form the snapshot API
        // publishes. Kept in one place so the two snapshot builders cannot drift.
        struct AabbCenterExtents
        {
            RE::NiPoint3 center{};
            RE::NiPoint3 halfExtents{};
        };

        [[nodiscard]] AabbCenterExtents aabbCenterExtents(const RE::NiPoint3& minPoint, const RE::NiPoint3& maxPoint) noexcept
        {
            return AabbCenterExtents{
                RE::NiPoint3{
                    (minPoint.x + maxPoint.x) * 0.5f,
                    (minPoint.y + maxPoint.y) * 0.5f,
                    (minPoint.z + maxPoint.z) * 0.5f,
                },
                RE::NiPoint3{
                    (maxPoint.x - minPoint.x) * 0.5f,
                    (maxPoint.y - minPoint.y) * 0.5f,
                    (maxPoint.z - minPoint.z) * 0.5f,
                },
            };
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
            mixQuantizedPoint(geometryHash, source.localCenterGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMinGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMaxGame, kGeometryHashQuantizationScale);
            summary.boundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);

            summary.pointCount += source.localPointsGame.size();
            summary.semanticPartMask |= partMask(source.semantic.partKind);
            const bool transientReloadSource = isTransientReloadPart(source.semantic.partKind);
            if (transientReloadSource) {
                ++summary.transientReloadSourceCount;
            } else {
                hasDurableGeometry = true;
                ++summary.durableSourceCount;
                summary.durablePointCount += source.localPointsGame.size();
                summary.durableBoundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);
                weapon_visual_composition_policy::mixString(durableGeometryHash, source.sourceName);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.driveRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.sourceRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, source.sourceGroupId);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, static_cast<std::uint32_t>(source.semantic.partKind));
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
        auto structuralMutation = destroyTargetBank && _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_pendingGeneratedWeaponBuild.active && destroyTargetBank) {
            destroyWeaponBodyBank(_pendingGeneratedWeaponBuild.replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies(), true);
        }
        _pendingGeneratedWeaponBuild = {};
        (void)world;
    }

    bool WeaponCollision::beginPendingGeneratedWeaponBuild(std::uint64_t equippedKey,
        std::uint64_t visualKey,
        std::uint64_t identityKey,
        std::uint64_t ownershipKey,
        std::uint32_t weaponFormID,
        const WeaponVisualKeyStats& visualKeyStats,
        bool replacingExisting,
        bool settingsChanged,
        bool driveRequestedRebuild,
        std::vector<GeneratedHullSource> sources,
        const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary)
    {
        if (equippedKey == 0 || ownershipKey == 0 || weaponFormID == 0 || sources.empty() || summary.signature == 0) {
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
        _pendingGeneratedWeaponBuild.ownershipKey = ownershipKey;
        _pendingGeneratedWeaponBuild.weaponFormID = weaponFormID;
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

    bool WeaponCollision::pendingGeneratedWeaponBuildMatches(
        std::uint64_t equippedKey,
        std::uint64_t ownershipKey,
        std::uint32_t weaponFormID) const
    {
        return _pendingGeneratedWeaponBuild.active &&
               _pendingGeneratedWeaponBuild.equippedKey == equippedKey &&
               _pendingGeneratedWeaponBuild.ownershipKey == ownershipKey &&
               _pendingGeneratedWeaponBuild.weaponFormID == weaponFormID &&
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
                // Nothing was published, so drop the identity and the empty body set.
                clearEquippedWeaponIdentityState(ClearScope::StagedBuildFailure, world);
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
        const auto ownershipKey = pending.ownershipKey;
        const auto weaponFormID = pending.weaponFormID;

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

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
        _cachedWeaponOwnershipKey = ownershipKey;
        _cachedWeaponFormID = weaponFormID;
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

    bool WeaponCollision::getApproximateBoundsSnapshot(ApproximateBoundsSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        outSnapshot.generationKey = getCurrentWeaponGenerationKey();
        if (outSnapshot.generationKey == 0) {
            return false;
        }

        bool sampled = false;
        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || !pointFinite(instance.generatedLocalMinGame) || !pointFinite(instance.generatedLocalMaxGame)) {
                continue;
            }
            if (instance.generatedLocalMaxGame.x < instance.generatedLocalMinGame.x ||
                instance.generatedLocalMaxGame.y < instance.generatedLocalMinGame.y ||
                instance.generatedLocalMaxGame.z < instance.generatedLocalMinGame.z) {
                continue;
            }

            if (!sampled) {
                outSnapshot.minWeaponLocal = instance.generatedLocalMinGame;
                outSnapshot.maxWeaponLocal = instance.generatedLocalMaxGame;
                sampled = true;
            } else {
                outSnapshot.minWeaponLocal.x = (std::min)(outSnapshot.minWeaponLocal.x, instance.generatedLocalMinGame.x);
                outSnapshot.minWeaponLocal.y = (std::min)(outSnapshot.minWeaponLocal.y, instance.generatedLocalMinGame.y);
                outSnapshot.minWeaponLocal.z = (std::min)(outSnapshot.minWeaponLocal.z, instance.generatedLocalMinGame.z);
                outSnapshot.maxWeaponLocal.x = (std::max)(outSnapshot.maxWeaponLocal.x, instance.generatedLocalMaxGame.x);
                outSnapshot.maxWeaponLocal.y = (std::max)(outSnapshot.maxWeaponLocal.y, instance.generatedLocalMaxGame.y);
                outSnapshot.maxWeaponLocal.z = (std::max)(outSnapshot.maxWeaponLocal.z, instance.generatedLocalMaxGame.z);
            }
            ++outSnapshot.sourceBodyCount;
        }

        if (!sampled || outSnapshot.sourceBodyCount == 0) {
            outSnapshot = {};
            return false;
        }
        const auto bounds = aabbCenterExtents(outSnapshot.minWeaponLocal, outSnapshot.maxWeaponLocal);
        outSnapshot.centerWeaponLocal = bounds.center;
        outSnapshot.halfExtentsWeaponLocal = bounds.halfExtents;
        outSnapshot.valid = pointFinite(outSnapshot.centerWeaponLocal) && pointFinite(outSnapshot.halfExtentsWeaponLocal);
        return outSnapshot.valid;
    }

    bool WeaponCollision::getCompoundGeometrySnapshot(CompoundGeometrySnapshot& outSnapshot) const
    {
        outSnapshot = {};
        outSnapshot.generationKey = getCurrentWeaponGenerationKey();
        if (outSnapshot.generationKey == 0) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::NoGeneration;
            return false;
        }

        const std::uint32_t expectedBodyCount = getWeaponBodyCount();
        if (expectedBodyCount == 0) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::NoActiveBodies;
            return false;
        }

        const auto& bank = activeWeaponBodies();
        const RE::NiAVObject* packageDriveNode = resolvePackageDriveNode(bank, nullptr);
        if (!packageDriveNode) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::SourceTransformUnavailable;
            return false;
        }

        outSnapshot.children.reserve(expectedBodyCount);
        auto fail = [&](const CompoundGeometrySnapshotFailure failure,
                        const std::uint32_t sourceIndex,
                        const std::uint32_t bodyId) {
            outSnapshot.failure = failure;
            outSnapshot.failedSourceIndex = sourceIndex;
            outSnapshot.failedBodyId = bodyId;
            return false;
        };

        bool sampledPoint = false;
        std::uint32_t sourceIndex = 0;
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }

            const std::uint32_t bodyId = instance.body.getBodyId().value;
            if (!instance.shape) {
                return fail(CompoundGeometrySnapshotFailure::MissingShape, sourceIndex, bodyId);
            }
            const auto& points = instance.generatedLocalPointsGame;
            if (points.empty()) {
                return fail(CompoundGeometrySnapshotFailure::MissingPointCloud, sourceIndex, bodyId);
            }
            for (const auto& point : points) {
                if (!pointFinite(point)) {
                    return fail(CompoundGeometrySnapshotFailure::NonFinitePoint, sourceIndex, bodyId);
                }
            }
            if (!pointCloudCanBuildHull(points)) {
                return fail(CompoundGeometrySnapshotFailure::DegeneratePointCloud, sourceIndex, bodyId);
            }

            CompoundGeometryChildSnapshot child{};
            child.shape = instance.shape;
            CompoundChildPoseSnapshot pose{};
            if (!resolveCompoundChildPose(instance, packageDriveNode, pose)) {
                return fail(CompoundGeometrySnapshotFailure::SourceTransformUnavailable, sourceIndex, bodyId);
            }
            child.shapeInWeapon = pose.shapeInWeapon;
            outSnapshot.sourcePointCount += points.size();

            for (const auto& point : points) {
                if (!sampledPoint) {
                    outSnapshot.minWeaponLocal = point;
                    outSnapshot.maxWeaponLocal = point;
                    sampledPoint = true;
                } else {
                    outSnapshot.minWeaponLocal = weapon_collision_geometry_math::pointMin(outSnapshot.minWeaponLocal, point);
                    outSnapshot.maxWeaponLocal = weapon_collision_geometry_math::pointMax(outSnapshot.maxWeaponLocal, point);
                }
            }

            outSnapshot.children.push_back(std::move(child));
            ++outSnapshot.sourceBodyCount;
            ++sourceIndex;
        }

        if (!sampledPoint || outSnapshot.sourceBodyCount == 0) {
            return fail(CompoundGeometrySnapshotFailure::NoActiveBodies, sourceIndex, INVALID_BODY_ID);
        }
        if (outSnapshot.sourceBodyCount != expectedBodyCount) {
            return fail(CompoundGeometrySnapshotFailure::BodyCountChanged, sourceIndex, INVALID_BODY_ID);
        }
        if (getCurrentWeaponGenerationKey() != outSnapshot.generationKey) {
            return fail(CompoundGeometrySnapshotFailure::GenerationChanged, sourceIndex, INVALID_BODY_ID);
        }

        const auto bounds = aabbCenterExtents(outSnapshot.minWeaponLocal, outSnapshot.maxWeaponLocal);
        outSnapshot.centerWeaponLocal = bounds.center;
        outSnapshot.halfExtentsWeaponLocal = bounds.halfExtents;
        if (!pointFinite(outSnapshot.centerWeaponLocal) || !pointFinite(outSnapshot.halfExtentsWeaponLocal) ||
            outSnapshot.halfExtentsWeaponLocal.x < 0.0f ||
            outSnapshot.halfExtentsWeaponLocal.y < 0.0f ||
            outSnapshot.halfExtentsWeaponLocal.z < 0.0f) {
            return fail(CompoundGeometrySnapshotFailure::InvalidBounds, sourceIndex, INVALID_BODY_ID);
        }

        outSnapshot.failure = CompoundGeometrySnapshotFailure::None;
        outSnapshot.valid = true;
        return true;
    }

    bool WeaponCollision::resolveCompoundChildPose(
        const WeaponBodyInstance& instance,
        const RE::NiAVObject* packageDriveNode,
        CompoundChildPoseSnapshot& outPose)
    {
        outPose = {};
        if (!packageDriveNode || !instance.shape) {
            return false;
        }

        RE::NiTransform sourceInWeapon = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiPoint3 shapeCenterWeaponLocal = instance.generatedLocalCenterGame;
        if (instance.sourceNode) {
            if (!tryResolveDescendantLocalTransform(packageDriveNode, instance.sourceNode, sourceInWeapon)) {
                return false;
            }
            if (!weaponTransformFinite(sourceInWeapon)) {
                return false;
            }
            shapeCenterWeaponLocal = transform_math::localPointToWorld(
                sourceInWeapon,
                instance.generatedSourceLocalCenterGame);
        }

        outPose.shapeInWeapon = sourceInWeapon;
        outPose.shapeInWeapon.translate = shapeCenterWeaponLocal;
        // Generated layer-44 shapes already bake their absolute source scale.
        // The dynamic compound supplies only the live relative pose, exactly
        // matching the per-part keyframed body convention.
        outPose.shapeInWeapon.scale = 1.0f;
        return weaponTransformFinite(outPose.shapeInWeapon);
    }

    bool WeaponCollision::getCompoundChildPoseSnapshot(
        const RE::NiAVObject* currentWeaponRoot,
        const std::uint64_t expectedGenerationKey,
        std::span<CompoundChildPoseSnapshot> outChildren,
        std::size_t& outChildCount) const
    {
        outChildCount = 0;
        if (!currentWeaponRoot || expectedGenerationKey == 0 ||
            getCurrentWeaponGenerationKey() != expectedGenerationKey) {
            return false;
        }

        const auto& bank = activeWeaponBodies();
        const std::uint32_t expectedBodyCount = getWeaponBodyCount();
        if (expectedBodyCount == 0 || outChildren.size() < expectedBodyCount) {
            return false;
        }

        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            if (outChildCount >= outChildren.size() ||
                !resolveCompoundChildPose(instance, currentWeaponRoot, outChildren[outChildCount])) {
                outChildCount = 0;
                return false;
            }
            ++outChildCount;
        }

        if (outChildCount != expectedBodyCount ||
            getCurrentWeaponGenerationKey() != expectedGenerationKey) {
            outChildCount = 0;
            return false;
        }
        return true;
    }

    WeaponCollision::ReleaseGeometrySnapshot WeaponCollision::getCurrentWeaponReleaseGeometry(
        const RE::NiPoint3& gripWorldPoint,
        const RE::NiTransform& capturedWeaponWorld) const
    {
        ReleaseGeometrySnapshot result{};
        // The captured pose is inverted below to bring the grip point into weapon
        // space, so it has to pass the same gate as any other weapon transform.
        if (!weaponTransformFinite(capturedWeaponWorld)) {
            return result;
        }
        result.hasCapturedWeaponWorld = true;
        result.capturedWeaponWorld = capturedWeaponWorld;
        if (!std::isfinite(gripWorldPoint.x) || !std::isfinite(gripWorldPoint.y) || !std::isfinite(gripWorldPoint.z)) {
            return result;
        }

        const auto& bank = activeWeaponBodies();
        float maxDistanceSquared = 0.0f;
        bool sampledPoint = false;
        auto sampleLocalPoint = [&](const RE::NiPoint3& localPoint) {
            if (!std::isfinite(localPoint.x) || !std::isfinite(localPoint.y) || !std::isfinite(localPoint.z)) {
                return;
            }
            const auto worldPoint = weapon_collision_geometry_math::localPointToWorld(
                capturedWeaponWorld.rotate,
                capturedWeaponWorld.translate,
                capturedWeaponWorld.scale,
                localPoint);
            const float dx = worldPoint.x - gripWorldPoint.x;
            const float dy = worldPoint.y - gripWorldPoint.y;
            const float dz = worldPoint.z - gripWorldPoint.z;
            const float distanceSquared = dx * dx + dy * dy + dz * dz;
            if (!std::isfinite(distanceSquared) || distanceSquared < 0.0f) {
                return;
            }
            maxDistanceSquared = (std::max)(maxDistanceSquared, distanceSquared);
            sampledPoint = true;
        };

        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            if (!instance.generatedLocalPointsGame.empty()) {
                for (const auto& point : instance.generatedLocalPointsGame) {
                    sampleLocalPoint(point);
                }
                continue;
            }

            // Bounds remain a safe release-only fallback for a generated body
            // whose reduced hull point cache is unexpectedly empty.
            for (int x = 0; x < 2; ++x) {
                for (int y = 0; y < 2; ++y) {
                    for (int z = 0; z < 2; ++z) {
                        sampleLocalPoint(RE::NiPoint3{
                            x == 0 ? instance.generatedLocalMinGame.x : instance.generatedLocalMaxGame.x,
                            y == 0 ? instance.generatedLocalMinGame.y : instance.generatedLocalMaxGame.y,
                            z == 0 ? instance.generatedLocalMinGame.z : instance.generatedLocalMaxGame.z,
                        });
                    }
                }
            }
        }

        result.leverGameUnits = sampledPoint ? std::sqrt(maxDistanceSquared) : 0.0f;
        return result;
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
        WeaponBodySnapshot candidate{};
        const bool stable = readUnderSeqlock([&] {
            candidate = {};
            candidate.bodyIds.fill(INVALID_BODY_ID);
            candidate.generationKey = _weaponBodySetKeyAtomic.load(std::memory_order_acquire);
            candidate.count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < candidate.count; ++i) {
                candidate.bodyIds[i] = _weaponBodyIdsAtomic[i].load(std::memory_order_acquire);
            }
        });
        if (stable) {
            return candidate;
        }

        // Fail closed: an unreadable publication looks like "no bodies", never like
        // a stale body set.
        WeaponBodySnapshot empty{};
        empty.bodyIds.fill(INVALID_BODY_ID);
        return empty;
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

        WeaponInteractionContact candidate{};
        bool found = false;
        const bool stable = readUnderSeqlock([&] {
            candidate = {};
            found = false;
            const std::uint32_t count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
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
        });
        if (!stable) {
            return false;
        }
        if (found) {
            outContact = candidate;
        }
        return found;
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

        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        bool found = false;
        const bool stable = readUnderSeqlock([&] {
            vx = 0.0f;
            vy = 0.0f;
            vz = 0.0f;
            found = false;
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
        });
        // A consistent read that found nothing, or found a non-finite sample, is a
        // final answer - only a torn read is worth another attempt, and
        // readUnderSeqlock has already spent those.
        if (!stable || !found || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
            return false;
        }

        outVelocityHavok[0] = vx;
        outVelocityHavok[1] = vy;
        outVelocityHavok[2] = vz;
        return true;
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

    bool WeaponCollision::tryBuildSupportGripEvidenceView(
        const WeaponBodyInstance& instance,
        const RE::NiAVObject* currentWeaponRoot,
        SupportGripEvidenceView& outView) const
    {
        outView = {};
        if (!instance.body.isValid() ||
            instance.generatedLocalTrianglesGame.empty()) {
            return false;
        }

        RE::NiTransform localToWorld{};
        const bool sourceNodeCurrent = instance.sourceNode && currentWeaponRoot &&
            tryResolveDescendantWorldTransform(
                currentWeaponRoot,
                currentWeaponRoot->world,
                instance.sourceNode,
                localToWorld);
        if (!sourceNodeCurrent) {
            const RE::NiAVObject* fallbackRoot = currentWeaponRoot ?
                currentWeaponRoot :
                instance.driveNode;
            if (!fallbackRoot) {
                return false;
            }
            localToWorld = fallbackRoot->world;
        }
        if (!weaponTransformFinite(localToWorld)) {
            return false;
        }

        const auto& localTriangles =
            sourceNodeCurrent &&
                !instance.generatedSourceLocalTrianglesGame.empty() ?
            instance.generatedSourceLocalTrianglesGame :
            instance.generatedLocalTrianglesGame;
        if (localTriangles.empty() ||
            !std::isfinite(localToWorld.scale) ||
            std::abs(localToWorld.scale) <= 0.000001f) {
            return false;
        }

        outView.localTriangles = std::span<const TriangleData>(
            localTriangles.data(),
            localTriangles.size());
        outView.localToWorld = localToWorld;
        outView.sourceGroupId = instance.generatedSourceGroupId != 0 ?
            instance.generatedSourceGroupId :
            reinterpret_cast<std::uintptr_t>(instance.sourceNode);
        outView.bodyId = instance.body.getBodyId().value;
        outView.weaponGenerationKey = getCurrentWeaponGenerationKey();
        outView.sourceNodeCurrent = sourceNodeCurrent;
        return outView.weaponGenerationKey != 0;
    }

    bool WeaponCollision::tryGetSupportGripEvidenceView(
        std::uint32_t bodyId,
        const RE::NiAVObject* currentWeaponRoot,
        SupportGripEvidenceView& outView) const
    {
        outView = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() ||
                instance.body.getBodyId().value != bodyId) {
                continue;
            }
            return tryBuildSupportGripEvidenceView(
                instance,
                currentWeaponRoot,
                outView);
        }

        return false;
    }

    std::size_t WeaponCollision::findSupportGripEvidenceViews(
        const RE::NiAVObject* currentWeaponRoot,
        std::span<SupportGripEvidenceView> outViews) const
    {
        for (auto& view : outViews) {
            view = {};
        }
        if (outViews.empty()) {
            return 0;
        }

        std::array<std::uintptr_t, MAX_WEAPON_BODIES> seenSourceGroups{};
        std::size_t seenSourceCount = 0;
        std::size_t viewCount = 0;
        for (const auto& instance : activeWeaponBodies()) {
            SupportGripEvidenceView view{};
            if (!tryBuildSupportGripEvidenceView(
                    instance,
                    currentWeaponRoot,
                    view)) {
                continue;
            }

            const std::uintptr_t sourceGroup = view.sourceGroupId;
            const bool duplicate = sourceGroup != 0 &&
                std::find(
                    seenSourceGroups.begin(),
                    seenSourceGroups.begin() + seenSourceCount,
                    sourceGroup) !=
                    seenSourceGroups.begin() + seenSourceCount;
            if (duplicate) {
                continue;
            }
            if (sourceGroup != 0 &&
                seenSourceCount < seenSourceGroups.size()) {
                seenSourceGroups[seenSourceCount++] = sourceGroup;
            }

            outViews[viewCount++] = view;
            if (viewCount >= outViews.size()) {
                break;
            }
        }
        return viewCount;
    }

    bool WeaponCollision::tryFindCurrentWeaponSurfaceNearPoint(
        const RE::NiAVObject* currentWeaponRoot,
        const RE::NiPoint3& pointWorld,
        float maxDistanceGameUnits,
        WeaponSurfaceProximityWitness& outWitness) const
    {
        const std::array<RE::NiPoint3, 1> points{ pointWorld };
        std::array<WeaponSurfaceProximityWitness, 1> witnesses{};
        const std::size_t witnessCount = findCurrentWeaponSurfaceNearPoints(
            currentWeaponRoot,
            points,
            maxDistanceGameUnits,
            witnesses);
        outWitness = witnesses[0];
        return witnessCount == 1 && outWitness.valid;
    }

    /*
     * Resolve the frame one generated body is proximity-tested in, and reject the
     * bodies that cannot be tested at all.
     *
     * Shared by both triangle-exact entry points below:
     * findCurrentWeaponSurfaceNearPoints (batch, nearest witness per point) and
     * tryFindInteractionContactNearPoint (one point, best-ranked body). The two
     * rank differently on purpose, but they must agree on WHICH frame a body lives
     * in and WHICH bodies are testable - otherwise the same hand position yields a
     * surface witness and no interaction contact.
     */
    bool WeaponCollision::resolveWeaponSurfaceScanFrame(
        const RE::NiAVObject* scanRoot,
        const WeaponBodyInstance& instance,
        WeaponSurfaceScanFrame& outFrame)
    {
        outFrame = {};
        if (!scanRoot || !instance.body.isValid()) {
            return false;
        }

        // A body extracted with its own source-node triangles is tested in that
        // node's frame: pumps, bolts and magazines move independently of the weapon
        // root, so root-space triangles would put the surface in the wrong place
        // mid-animation.
        RE::NiTransform world = scanRoot->world;
        const bool sourceNodeCurrent = instance.sourceNode &&
            tryResolveDescendantWorldTransform(
                scanRoot,
                scanRoot->world,
                instance.sourceNode,
                world);
        outFrame.useSourceFrame = sourceNodeCurrent && !instance.generatedSourceLocalTrianglesGame.empty();
        if (!outFrame.useSourceFrame) {
            // Covers the failed-resolve case too: the resolver may have written a
            // partial transform, so the root frame is restored rather than trusted.
            world = scanRoot->world;
        }

        outFrame.world = world;
        outFrame.localTriangles = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalTrianglesGame :
            &instance.generatedLocalTrianglesGame;
        outFrame.boundsMin = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalMinGame :
            &instance.generatedLocalMinGame;
        outFrame.boundsMax = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalMaxGame :
            &instance.generatedLocalMaxGame;

        // Fail closed: no triangles, an unusable frame, or an inverted/NaN AABB all
        // mean this body cannot answer a proximity question this frame.
        // weaponTransformFinite already rejects |scale| <= 0.0001, so the frame
        // scale needs no separate floor here.
        if (outFrame.localTriangles->empty() ||
            !weaponTransformFinite(outFrame.world) ||
            !pointFinite(*outFrame.boundsMin) ||
            !pointFinite(*outFrame.boundsMax) ||
            outFrame.boundsMin->x > outFrame.boundsMax->x ||
            outFrame.boundsMin->y > outFrame.boundsMax->y ||
            outFrame.boundsMin->z > outFrame.boundsMax->z) {
            return false;
        }

        outFrame.absoluteScale = std::abs(outFrame.world.scale);
        return true;
    }

    std::size_t WeaponCollision::findCurrentWeaponSurfaceNearPoints(
        const RE::NiAVObject* currentWeaponRoot,
        const std::span<const RE::NiPoint3> pointsWorld,
        const float maxDistanceGameUnits,
        const std::span<WeaponSurfaceProximityWitness> outWitnesses) const
    {
        constexpr std::size_t kMaximumPointCount = 16;
        for (auto& witness : outWitnesses) {
            witness = {};
        }
        const std::uint64_t currentGeneration = getCurrentWeaponGenerationKey();
        if (!currentWeaponRoot ||
            currentGeneration == 0 ||
            pointsWorld.empty() ||
            pointsWorld.size() > kMaximumPointCount ||
            outWitnesses.size() != pointsWorld.size() ||
            !std::isfinite(maxDistanceGameUnits) ||
            maxDistanceGameUnits < 0.0f) {
            return 0;
        }
        for (const auto& pointWorld : pointsWorld) {
            if (!pointFinite(pointWorld)) {
                return 0;
            }
        }

        /*
         * Authored-pose validity is a surface claim, not a part-AABB claim:
         * animation-zero/default hands can land inside a broad receiver box
         * while remaining nowhere near rendered weapon geometry. AABBs only
         * reject unrelated parts before the exact cached-triangle test.
         *
         * The bounded batch form is also used by the opt-in authored-grip
         * visualizer. It allocates nothing, scans each current body/triangle
         * bank once, and retains the true nearest witness per supplied point.
         */
        for (const auto& instance : activeWeaponBodies()) {
            WeaponSurfaceScanFrame frame{};
            if (!resolveWeaponSurfaceScanFrame(currentWeaponRoot, instance, frame)) {
                continue;
            }
            const RE::NiTransform& surfaceWorld = frame.world;
            const auto& localTriangles = *frame.localTriangles;
            const RE::NiPoint3& boundsMin = *frame.boundsMin;
            const RE::NiPoint3& boundsMax = *frame.boundsMax;

            // The search radius is in game units; the cached triangles are in the
            // frame's own local space, so the radius converts once per body.
            const float absoluteScale = frame.absoluteScale;
            const float localRadius = maxDistanceGameUnits / absoluteScale;
            std::array<RE::NiPoint3, kMaximumPointCount> localPoints{};
            std::array<bool, kMaximumPointCount> pointMayReachBody{};
            bool anyPointMayReachBody = false;
            for (std::size_t pointIndex = 0;
                 pointIndex < pointsWorld.size();
                 ++pointIndex) {
                localPoints[pointIndex] =
                    weapon_collision_geometry_math::worldPointToLocal(
                        surfaceWorld.rotate,
                        surfaceWorld.translate,
                        surfaceWorld.scale,
                        pointsWorld[pointIndex]);
                if (!pointFinite(localPoints[pointIndex])) {
                    continue;
                }
                const float boundsDistanceSquared =
                    weapon_interaction_probe_math::pointAabbDistanceSquared(
                        localPoints[pointIndex],
                        boundsMin,
                        boundsMax);
                pointMayReachBody[pointIndex] =
                    std::isfinite(boundsDistanceSquared) &&
                    weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                        boundsDistanceSquared,
                        localRadius);
                anyPointMayReachBody =
                    anyPointMayReachBody || pointMayReachBody[pointIndex];
            }
            if (!anyPointMayReachBody) {
                continue;
            }

            for (const auto& triangle : localTriangles) {
                if (!pointFinite(triangle.v0) ||
                    !pointFinite(triangle.v1) ||
                    !pointFinite(triangle.v2)) {
                    continue;
                }

                for (std::size_t pointIndex = 0;
                     pointIndex < pointsWorld.size();
                     ++pointIndex) {
                    if (!pointMayReachBody[pointIndex]) {
                        continue;
                    }
                    float surfaceDistanceSquared =
                        (std::numeric_limits<float>::infinity)();
                    const RE::NiPoint3 closestPointLocal =
                        closestPointOnTriangleToPoint(
                            localPoints[pointIndex],
                            triangle,
                            surfaceDistanceSquared);
                    if (!std::isfinite(surfaceDistanceSquared) ||
                        surfaceDistanceSquared < 0.0f ||
                        !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                            surfaceDistanceSquared,
                            localRadius)) {
                        continue;
                    }

                    const float distanceGameUnits =
                        std::sqrt(surfaceDistanceSquared) * absoluteScale;
                    auto& witness = outWitnesses[pointIndex];
                    if (!std::isfinite(distanceGameUnits) ||
                        (witness.valid &&
                            distanceGameUnits >= witness.distanceGameUnits)) {
                        continue;
                    }
                    witness.closestPointWorld =
                        weapon_collision_geometry_math::localPointToWorld(
                            surfaceWorld.rotate,
                            surfaceWorld.translate,
                            surfaceWorld.scale,
                            closestPointLocal);
                    witness.distanceGameUnits = distanceGameUnits;
                    witness.bodyId = instance.body.getBodyId().value;
                    witness.weaponGenerationKey = currentGeneration;
                    witness.sourceNodeCurrent = frame.useSourceFrame;
                    witness.valid = pointFinite(witness.closestPointWorld);
                }
            }
        }

        if (getCurrentWeaponGenerationKey() != currentGeneration) {
            for (auto& witness : outWitnesses) {
                witness = {};
            }
            return 0;
        }

        std::size_t witnessCount = 0;
        for (const auto& witness : outWitnesses) {
            if (witness.valid &&
                witness.weaponGenerationKey == currentGeneration) {
                ++witnessCount;
            }
        }
        return witnessCount;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::buildProfileEvidenceSnapshot(
        const WeaponBodyBank& bank,
        WeaponCompositionSnapshot& outComposition) const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        descriptors.reserve(bankWeaponBodyCount(bank));

        /*
         * Pair slot-classified parts with the installed OMOD occupying that
         * slot: resolve the equipped instance's active mods once and index
         * them by attach-point keyword FormID. Runs once per publication on
         * the main thread; ~a dozen form lookups.
         */
        const auto omodByAttachPointFormId =
            readEquippedOmodsByAttachPointFormId(&outComposition);
        outComposition.weaponGenerationKey = _cachedWeaponBodySetKey;
        outComposition.weaponFormId = _cachedWeaponFormID;

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
            const auto partValue = static_cast<std::uint32_t>(
                instance.semantic.partKind);
            if (partValue < 64) {
                const auto coverageBit = 1ull << partValue;
                outComposition.semanticCoverageMask |= coverageBit;
                for (std::uint32_t compositionIndex = 0;
                     compositionIndex < outComposition.entryCount;
                     ++compositionIndex) {
                    auto& entry = outComposition.entries[compositionIndex];
                    if ((descriptor.omodFormId != 0 &&
                            entry.omodFormId == descriptor.omodFormId) ||
                        (instance.semantic.attachPointFormId != 0 &&
                            entry.attachPointFormId ==
                                instance.semantic.attachPointFormId)) {
                        entry.semanticCoverageMask |= coverageBit;
                        entry.flags |= 1u << 3;
                    }
                }
            }
            descriptors.push_back(std::move(descriptor));
        }

        constexpr std::uint64_t kFnvOffset = 1469598103934665603ull;
        constexpr std::uint64_t kFnvPrime = 1099511628211ull;
        auto signature = kFnvOffset;
        for (std::uint32_t i = 0; i < outComposition.entryCount; ++i) {
            const auto& entry = outComposition.entries[i];
            for (const auto value : {
                     entry.omodFormId,
                     entry.attachPointFormId,
                     entry.stableIndex,
                     entry.flags }) {
                signature ^= value;
                signature *= kFnvPrime;
            }
            if ((entry.flags & (1u << 0)) != 0 &&
                entry.semanticCoverageMask == 0 && i < 64) {
                outComposition.missingCoverageMask |= 1ull << i;
            }
        }
        outComposition.compositionSignature =
            outComposition.entryCount != 0 ? signature : 0;

        return descriptors;
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

    namespace
    {
        bool isFiniteOrderedEvidenceBounds(const WeaponEvidenceBounds3& bounds)
        {
            return bounds.valid && std::isfinite(bounds.min.x) && std::isfinite(bounds.min.y) && std::isfinite(bounds.min.z) && std::isfinite(bounds.max.x) &&
                std::isfinite(bounds.max.y) && std::isfinite(bounds.max.z) && bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y && bounds.min.z <= bounds.max.z;
        }

        WeaponCollision::NativeScopeSightAnchorSnapshot buildNativeScopeSightAnchorSnapshot(
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

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::getProfileEvidenceDescriptors() const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                descriptors = _profileEvidenceSnapshot;
            })) {
            return descriptors;
        }

        return {};
    }

    WeaponEmitterSnapshot WeaponCollision::getWeaponEmitterSnapshot() const
    {
        std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
        return _weaponEmitterSnapshot;
    }

    WeaponCollision::NativeScopeSightAnchorSnapshot WeaponCollision::getNativeScopeSightAnchorSnapshot() const
    {
        NativeScopeSightAnchorSnapshot snapshot{};
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                snapshot = _nativeScopeSightAnchorSnapshot;
            })) {
            return snapshot;
        }

        return {};
    }

    WeaponCollision::WeaponCompositionSnapshot
    WeaponCollision::getWeaponCompositionSnapshot() const
    {
        WeaponCompositionSnapshot snapshot{};
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                snapshot = _weaponCompositionSnapshot;
            })) {
            return snapshot;
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
        const std::uint64_t currentGeneration = getCurrentWeaponGenerationKey();
        if (!weaponNode || currentGeneration == 0 ||
            !pointFinite(probeWorldPoint) ||
            !std::isfinite(probeRadiusGame) || probeRadiusGame <= 0.0f) {
            return false;
        }

        weapon_interaction_probe_math::ProbeCandidateRank bestRank{};
        const WeaponBodyInstance* bestInstance = nullptr;
        int boundsCandidateCount = 0;
        int surfaceCandidateCount = 0;
        const RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(activeWeaponBodies(), const_cast<RE::NiAVObject*>(weaponNode));
        if (!packageDriveRoot) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            WeaponSurfaceScanFrame frame{};
            if (!resolveWeaponSurfaceScanFrame(packageDriveRoot, instance, frame)) {
                continue;
            }
            const RE::NiTransform& probeWorld = frame.world;
            const auto& localTriangles = *frame.localTriangles;
            const RE::NiPoint3& boundsMin = *frame.boundsMin;
            const RE::NiPoint3& boundsMax = *frame.boundsMax;

            const float absoluteScale = frame.absoluteScale;
            const float localRadius = probeRadiusGame / absoluteScale;
            const RE::NiPoint3 probeLocal = weapon_collision_geometry_math::worldPointToLocal(
                probeWorld.rotate,
                probeWorld.translate,
                probeWorld.scale,
                probeWorldPoint);
            if (!pointFinite(probeLocal)) {
                continue;
            }

            const float boundsDistanceSquared = weapon_interaction_probe_math::pointAabbDistanceSquared(
                probeLocal,
                boundsMin,
                boundsMax);
            if (!std::isfinite(boundsDistanceSquared) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                    boundsDistanceSquared,
                    localRadius)) {
                continue;
            }

            ++boundsCandidateCount;
            float minimumSurfaceDistanceSquaredLocal =
                (std::numeric_limits<float>::infinity)();
            for (const auto& triangle : localTriangles) {
                if (!pointFinite(triangle.v0) ||
                    !pointFinite(triangle.v1) ||
                    !pointFinite(triangle.v2)) {
                    continue;
                }
                float surfaceDistanceSquaredLocal =
                    (std::numeric_limits<float>::infinity)();
                (void)closestPointOnTriangleToPoint(
                    probeLocal,
                    triangle,
                    surfaceDistanceSquaredLocal);
                if (std::isfinite(surfaceDistanceSquaredLocal) &&
                    surfaceDistanceSquaredLocal >= 0.0f) {
                    minimumSurfaceDistanceSquaredLocal = (std::min)(
                        minimumSurfaceDistanceSquaredLocal,
                        surfaceDistanceSquaredLocal);
                }
            }
            if (!std::isfinite(minimumSurfaceDistanceSquaredLocal) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                    minimumSurfaceDistanceSquaredLocal,
                    localRadius)) {
                continue;
            }

            ++surfaceCandidateCount;
            const float scaleSquared = absoluteScale * absoluteScale;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                .distanceSquaredGame =
                    minimumSurfaceDistanceSquaredLocal * scaleSquared,
                .aabbDiagonalSquaredGame =
                    weapon_interaction_probe_math::aabbDiagonalSquared(
                        boundsMin,
                        boundsMax) * scaleSquared,
                .semanticPriority = instance.semantic.priority,
            };
            if (bestInstance && !weapon_interaction_probe_math::isBetterProbeCandidate(rank, bestRank)) {
                continue;
            }

            bestRank = rank;
            bestInstance = &instance;
        }

        if (!bestInstance || getCurrentWeaponGenerationKey() != currentGeneration) {
            return false;
        }

        ROCK_LOG_SAMPLE_DEBUG(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "WeaponInteractionProbe exact surface ranked: part={} bodyId={} dist={:.2f} diag={:.1f} priority={} boundsCandidates={} surfaceCandidates={}",
            static_cast<int>(bestInstance->semantic.partKind),
            bestInstance->body.getBodyId().value,
            std::sqrt(bestRank.distanceSquaredGame),
            std::sqrt(bestRank.aabbDiagonalSquaredGame),
            bestInstance->semantic.priority,
            boundsCandidateCount,
            surfaceCandidateCount);

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
        outContact.weaponGenerationKey = currentGeneration;
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

    /*
     * One clear for every "forget the weapon we are built for" path. Seven sites
     * used to open-code this with subtly different member subsets; the scope enum
     * in the header now names each deliberate difference, and the ordering below
     * is smallest-reset-first so a reader can see exactly what each scope adds.
     *
     * Body-bank work stays with the callers on purpose: destroying bodies,
     * clearing the published atomic ids and resetting the body-set generation are
     * conditional on whether bodies actually exist, which only the caller knows.
     */
    void WeaponCollision::clearEquippedWeaponIdentityState(ClearScope scope, RE::hknpWorld* world)
    {
        // Always: the identity of the weapon the current body set was built for.
        // Until these are zero the update path believes its bodies still match.
        _cachedWeaponKey = 0;
        _cachedWeaponVisualKey = 0;
        _cachedWeaponIdentityKey = 0;
        _cachedWeaponOwnershipKey = 0;
        _cachedWeaponFormID = 0;
        // Completeness tracking and the visual-settle counter are derived from
        // that identity, so they can never outlive it.
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();

        if (scope == ClearScope::StagedBuildFailure) {
            return;
        }

        // The cached sources and any half-finished staged build describe geometry
        // that is now known not to match, so both go.
        clearGeneratedSourceCache();
        // LifecycleInit is the one scope that abandons the pending slot instead of
        // destroying its target bank: at subsystem start there is no live Havok
        // bank behind it, so taking the physics-gate mutation lease would buy
        // nothing.
        clearPendingGeneratedWeaponBuild(world, scope != ClearScope::LifecycleInit);
        // The visual-miss retention window is a per-weapon counter. The two
        // lifecycle scopes never reset it. That reads as an oversight rather than
        // a decision, but it is preserved here rather than normalized - see the
        // commit note.
        if (scope != ClearScope::LifecycleInit && scope != ClearScope::LifecycleShutdown) {
            resetVisualSourceUnavailableRetention();
        }

        if (scope == ClearScope::VisualSourceMiss) {
            return;
        }

        // From here the weapon itself is considered gone or rescaled. The observed
        // keys are the last thing read off the equipped form, so they must not
        // survive into the next weapon's first frame.
        _observedEquippedWeaponIdentityKey = 0;
        _observedEquippedWeaponOwnershipKey = 0;
        _observedEquippedWeaponFormID = 0;
        _observedEquippedWeaponInstanceContentKey = 0;
        // The OMOD pre-build audit marker is a (key, root) pair: leaving it set
        // would make the next weapon look already audited.
        _omodPrebuildAuditEquippedKey = 0;
        _omodPrebuildAuditRoot = nullptr;
        // Settings are re-read on the next build, and any pending drive rebuild
        // request refers to bodies that are about to stop existing.
        resetWeaponCollisionSettingsCache();
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);

        if (scope == ClearScope::ScaleInvalidation) {
            // A rescale does not change which weapon is held: the emitter snapshot
            // stays valid and a queued workbench rebuild is still wanted.
            return;
        }

        // No weapon: a queued workbench-exit rebuild refers to a weapon that is no
        // longer there, and nothing is published any more.
        _workbenchExitRebuildRequested.store(false, std::memory_order_release);
        resetWeaponBodySetGeneration();
        // LifecycleInit does not drop the emitter snapshot. Like the retention
        // counter above this looks accidental, and is preserved rather than
        // normalized - see the commit note.
        if (scope != ClearScope::LifecycleInit) {
            clearWeaponEmitterSnapshot();
        }

        if (scope == ClearScope::CurrentWeapon) {
            return;
        }

        // Subsystem start/stop only. These deliberately outlive a weapon swap, so
        // no per-weapon scope may touch them.
        _detachedSourceExclusionEquippedKey = 0;
        _detachedSourceExclusionGroups.clear();
        _weaponBodySetEpoch = 0;
        _generatedRecaptureDiagnostic = {};
        _usingReplacementWeaponBodies = false;
        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = 0;
    }

    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        // Cache the Havok context even while the feature is disabled so the INI
        // watcher can hot-enable weapon collision without requiring a physics
        // module restart.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        clearEquippedWeaponIdentityState(ClearScope::LifecycleInit, world);
        // No bodies exist yet at init, so publish an empty id set explicitly.
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

        // Clear before dropping the world pointer: the pending staged build still
        // has to destroy its target bank in that world.
        clearEquippedWeaponIdentityState(ClearScope::LifecycleShutdown, _cachedWorld);
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::abandonHavokStateAfterWorldLoss()
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        clearAtomicBodyIds();
        resetWeaponBodySetGeneration();
        for (auto& instance : _weaponBodies) {
            clearWeaponBodyInstance(instance, true);
        }
        for (auto& instance : _weaponReplacementBodies) {
            clearWeaponBodyInstance(instance, true);
        }
        _pendingGeneratedWeaponBuild = {};
        _generatedRecaptureDiagnostic = {};
        _usingReplacementWeaponBodies = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        ROCK_LOG_INFO(Weapon, "Weapon collision wrappers abandoned after Havok world loss");
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
            clearEquippedWeaponIdentityState(ClearScope::CurrentWeapon, world);
        };

        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            if (hasWeaponBody() && world) {
                ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via hot reload - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            _generatedRecaptureDiagnostic = {};
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
            _detachedSourceExclusionEquippedKey = 0;
            _detachedSourceExclusionGroups.clear();
            _generatedRecaptureDiagnostic = {};
            clearCurrentWeaponState();
        }

        if (!weaponDrawn) {
            noteUndrawnIntervalForRecaptureDiagnostic();
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon no longer drawn - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }

        std::uint64_t observedIdentityKey = 0;
        std::uint64_t observedOwnershipKey = 0;
        std::uint64_t observedInstanceContentKey = 0;
        std::uint32_t observedFormID = 0;
        const std::uint64_t observedKey =
            getEquippedWeaponIdentityKey(&observedIdentityKey, &observedOwnershipKey, nullptr, &observedFormID, &observedInstanceContentKey);
        if (observedKey == 0) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon identity unavailable - destroying generated weapon bodies");
                destroyWeaponBody(world);
            }
            clearCurrentWeaponState();
            return;
        }
        _observedEquippedWeaponIdentityKey = observedIdentityKey;
        _observedEquippedWeaponOwnershipKey = observedOwnershipKey;
        _observedEquippedWeaponFormID = observedFormID;
        _observedEquippedWeaponInstanceContentKey = observedInstanceContentKey;
        updateWeaponEmitterSnapshot(weaponNode, observedKey);

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
            const bool pendingInvalidated = driveRequestedRebuild || workbenchExitRequested ||
                !pendingGeneratedWeaponBuildMatches(observedKey, observedOwnershipKey, observedFormID);
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
            const bool omodPrebuildAuditCurrent =
                _omodPrebuildAuditEquippedKey == observedKey && _omodPrebuildAuditRoot == weaponNode;
            if (generationDrivenRebuild && !omodPrebuildAuditCurrent &&
                g_rockConfig.rockDebugWeaponOmodCoverageAudit && g_rockConfig.rockDebugWeaponOmodSelfHeal) {
                const auto auditResult = maybeRunWeaponOmodCoverageAudit(weaponNode, observedKey, true);
                if (auditResult.sceneEnriched) {
                    /*
                     * TryAttach3DRecurse mutates the assembled tree. Let the
                     * engine settle transforms once, then run the unchanged
                     * full visual witness and collider builder.
                     */
                    clearPendingWeaponVisualRebuild();
                    ROCK_LOG_INFO(Weapon,
                        "Generated weapon collision pre-build OMOD enrichment completed key={:016X}; deferring source capture one frame",
                        observedKey);
                    return;
                }
                if (auditResult.ran) {
                    // Cache only a non-mutating pass. A successful attachment
                    // must be followed by another pre-build pass so batches
                    // larger than the per-audit cap fully converge.
                    _omodPrebuildAuditEquippedKey = observedKey;
                    _omodPrebuildAuditRoot = weaponNode;
                }
            }
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
                    generatedCount = findGeneratedWeaponShapeSources(weaponNode, observedKey, generatedSources);
                    recordGeneratedRecaptureDiagnostic(
                        observedKey,
                        observedIdentityKey,
                        observedOwnershipKey,
                        observedFormID,
                        generatedSources);
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
                    clearEquippedWeaponIdentityState(ClearScope::VisualSourceMiss, world);
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
                        observedOwnershipKey,
                        observedFormID,
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
                        clearEquippedWeaponIdentityState(ClearScope::StagedBuildFailure, world);
                    }
                    // The visual-settle counter restarts either way: a replacement
                    // build that failed to queue must not look already settled.
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

        maybeRunWeaponOmodCoverageAudit(weaponNode, observedKey);
    }


    std::size_t WeaponCollision::findGeneratedWeaponShapeSources(
        RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey,
        std::vector<GeneratedHullSource>& outSources)
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
        if (_detachedSourceExclusionEquippedKey != equippedWeaponKey) {
            _detachedSourceExclusionEquippedKey = equippedWeaponKey;
            _detachedSourceExclusionGroups.clear();
            _detachedSourceExclusionGroups.reserve(64);
        }
        std::unordered_set<std::uintptr_t> claimedSourceGroups;
        claimedSourceGroups.reserve(256);
        std::size_t acceptedCandidateCount = 0;
        std::uint32_t totalVisitedShapes = 0;
        std::uint32_t totalExtractedTriangles = 0;
        std::uint32_t totalCulledForEffectGeometry = 0;
        const auto groupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(g_rockConfig.rockWeaponCollisionGroupingMode);
        for (const auto& candidate : candidates) {
            std::vector<GeneratedHullSource> candidateSources;
            std::unordered_set<std::uintptr_t> candidateExtractedSourceGroups;
            candidateExtractedSourceGroups.reserve(64);
            std::uint32_t visitedShapes = 0;
            std::uint32_t extractedTriangles = 0;
            std::uint32_t culledForEffectGeometry = 0;
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
                culledForEffectGeometry);
            totalCulledForEffectGeometry += culledForEffectGeometry;

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh candidate: label='{}' root='{}' addr={:x} packageRoot='{}' grouping={} acceptedShapes={} visitedShapes={} triangles={} hulls={} effectShapesCulled={}",
                candidate.label,
                safeNodeName(candidate.root),
                reinterpret_cast<std::uintptr_t>(candidate.root),
                safeNodeName(packageDriveRoot),
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(groupingMode),
                candidateExtractedSourceGroups.size(),
                visitedShapes,
                extractedTriangles,
                candidateSources.size(),
                culledForEffectGeometry);
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

        /*
         * Refine physical module kinds from the same installed-OMOD and live
         * emitter evidence that ROCK publishes through the provider API.
         * Exact OMOD identity is authoritative for standard slots. Mod-added
         * P-* slots have no vanilla attach-point FormID, so they use only the
         * bounded owner subtree captured during this same traversal; candidate
         * root fallbacks are explicitly rejected by ownerRootStructural.
         */
        const auto omodByAttachPointFormId = readEquippedOmodsByAttachPointFormId();
        std::unordered_set<std::uint32_t> nativeScopeOverlayOmods;
        nativeScopeOverlayOmods.reserve(omodByAttachPointFormId.size());
        for (const auto& [attachPointFormId, omodFormId] : omodByAttachPointFormId) {
            (void)attachPointFormId;
            if (attachmentModHasNativeScopeOverlayTarget(omodFormId)) {
                nativeScopeOverlayOmods.insert(omodFormId);
            }
        }

        const auto emitterSnapshot = collectWeaponEmitterSnapshot(
            weaponNode,
            omodByAttachPointFormId,
            equippedWeaponKey,
            equippedWeaponKey,
            makeWeaponEmitterRootSetKey(weaponNode));
        for (auto& source : outSources) {
            std::uint32_t sourceOmodFormId = 0;
            if (source.semantic.attachPointFormId != 0) {
                const auto omod = omodByAttachPointFormId.find(source.semantic.attachPointFormId);
                if (omod != omodByAttachPointFormId.end()) {
                    sourceOmodFormId = omod->second;
                }
            }

            weapon_accessory_part_kind_policy::Evidence evidence{};
            evidence.nativeScopeOverlay =
                (sourceOmodFormId != 0 && nativeScopeOverlayOmods.contains(sourceOmodFormId)) ||
                (sourceOmodFormId == 0 && source.semantic.partKind == WeaponPartKind::Sight && nativeScopeOverlayOmods.size() == 1);

            for (std::size_t emitterIndex = 0; emitterIndex < emitterSnapshot.count; ++emitterIndex) {
                const auto& emitter = emitterSnapshot.emitters[emitterIndex];
                if (!emitter.valid) {
                    continue;
                }

                const bool sameOmod = sourceOmodFormId != 0 && emitter.omodFormId == sourceOmodFormId;
                bool sameStructuralOwner = false;
                if (!sameOmod && emitter.ownerRootStructural && emitter.ownerRootAddress != 0 && source.sourceRoot) {
                    auto* ownerRoot = reinterpret_cast<RE::NiAVObject*>(emitter.ownerRootAddress);
                    sameStructuralOwner = actor_equipment_grab::nodeContainsNode(ownerRoot, source.sourceRoot, 32);
                }
                if (!sameOmod && !sameStructuralOwner) {
                    continue;
                }

                switch (static_cast<weapon_emitter_policy::Kind>(emitter.kind)) {
                case weapon_emitter_policy::Kind::Laser:
                    evidence.laserEmitter = true;
                    break;
                case weapon_emitter_policy::Kind::Flashlight:
                    evidence.flashlightEmitter = true;
                    break;
                case weapon_emitter_policy::Kind::Reticle:
                case weapon_emitter_policy::Kind::Unknown:
                default:
                    break;
                }
            }

            const auto baseKind = source.semantic.partKind;
            source.semantic = weapon_accessory_part_kind_policy::applyAttachmentEvidence(source.semantic, evidence);
            if (source.semantic.partKind != baseKind) {
                ROCK_LOG_DEBUG(Weapon,
                    "Generated weapon part refined by attachment evidence: source='{}' base={} resolved={} omod={:08X} nativeScope={} laser={} flashlight={}",
                    source.sourceName,
                    generatedWeaponPartKindName(baseKind),
                    generatedWeaponPartKindName(source.semantic.partKind),
                    sourceOmodFormId,
                    evidence.nativeScopeOverlay,
                    evidence.laserEmitter,
                    evidence.flashlightEmitter);
            }
        }

        const std::size_t cachedDetachedSourceCount = std::erase_if(outSources, [&](const GeneratedHullSource& source) {
            return source.sourceGroupId != 0 && _detachedSourceExclusionGroups.contains(source.sourceGroupId);
        });
        if (cachedDetachedSourceCount != 0) {
            ROCK_LOG_SAMPLE_INFO(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon detached component cache: excluded {} source(s) for equippedKey={:016X} cachedGroups={}",
                cachedDetachedSourceCount,
                equippedWeaponKey,
                _detachedSourceExclusionGroups.size());
        }

        if (!outSources.empty()) {
            std::vector<weapon_collision_geometry_math::DetachedComponentInput> componentInputs;
            componentInputs.reserve(outSources.size());
            for (const auto& source : outSources) {
                componentInputs.push_back(weapon_collision_geometry_math::DetachedComponentInput{
                    .min = pointToArray(source.localMinGame),
                    .max = pointToArray(source.localMaxGame),
                    .coherenceGroup = source.sourceGroupId,
                    .assembledAnchor = isAssembledWeaponComponentAnchor(source.semantic.partKind),
                });
            }

            const auto componentFilter = weapon_collision_geometry_math::findDetachedSourceComponentIndices(
                componentInputs,
                GENERATED_SOURCE_COMPONENT_JOIN_TOLERANCE_GAME,
                GENERATED_SOURCE_DETACHED_COMPONENT_MIN_GAP_GAME);
            if (componentFilter.verdict == weapon_collision_geometry_math::DetachedComponentVerdict::Filtered) {
                std::vector<std::uint8_t> excluded(outSources.size(), 0);
                std::size_t newlyCachedGroups = 0;
                const std::string representativeName = outSources[componentFilter.excludedIndices.front()].sourceName;
                for (const auto sourceIndex : componentFilter.excludedIndices) {
                    if (sourceIndex >= outSources.size()) {
                        continue;
                    }
                    excluded[sourceIndex] = 1;
                    const auto sourceGroupId = outSources[sourceIndex].sourceGroupId;
                    if (sourceGroupId != 0 &&
                        _detachedSourceExclusionGroups.size() < MAX_CACHED_DETACHED_SOURCE_GROUPS) {
                        newlyCachedGroups += _detachedSourceExclusionGroups.insert(sourceGroupId).second ? 1u : 0u;
                    }
                }

                std::vector<GeneratedHullSource> retainedSources;
                retainedSources.reserve(outSources.size() - componentFilter.excludedIndices.size());
                for (std::size_t sourceIndex = 0; sourceIndex < outSources.size(); ++sourceIndex) {
                    if (!excluded[sourceIndex]) {
                        retainedSources.push_back(std::move(outSources[sourceIndex]));
                    }
                }
                const auto excludedSourceCount = outSources.size() - retainedSources.size();
                outSources = std::move(retainedSources);
                ROCK_LOG_INFO(Weapon,
                    "Generated weapon detached component filter: excludedComponents={} excludedSources={} retainedSources={} components={} anchorComponents={} nearestGap={:.2f} joinTolerance={:.2f} minimumGap={:.2f} newlyCachedGroups={} cachedGroups={} representative='{}' equippedKey={:016X}",
                    componentFilter.excludedComponentCount,
                    excludedSourceCount,
                    outSources.size(),
                    componentFilter.componentCount,
                    componentFilter.assembledAnchorComponentCount,
                    componentFilter.minimumExcludedGap,
                    GENERATED_SOURCE_COMPONENT_JOIN_TOLERANCE_GAME,
                    GENERATED_SOURCE_DETACHED_COMPONENT_MIN_GAP_GAME,
                    newlyCachedGroups,
                    _detachedSourceExclusionGroups.size(),
                    representativeName,
                    equippedWeaponKey);
            } else if (componentFilter.verdict == weapon_collision_geometry_math::DetachedComponentVerdict::FailOpenInvalidInput ||
                       componentFilter.verdict == weapon_collision_geometry_math::DetachedComponentVerdict::FailOpenSourceLimit ||
                       componentFilter.verdict == weapon_collision_geometry_math::DetachedComponentVerdict::FailOpenNoAssembledAnchor) {
                ROCK_LOG_SAMPLE_WARN(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon detached component filter failed open: verdict={} sources={} components={} anchorComponents={} equippedKey={:016X}",
                    static_cast<int>(componentFilter.verdict),
                    outSources.size(),
                    componentFilter.componentCount,
                    componentFilter.assembledAnchorComponentCount,
                    equippedWeaponKey);
            }
        }

        if (outSources.empty()) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon mesh source scan: no collider sources remain after detached-component exclusions equippedKey={:016X}",
                equippedWeaponKey);
            return 0;
        }

        auto generatedSourceSemanticMask = [](const std::vector<GeneratedHullSource>& sources) {
            std::uint32_t mask = 0;
            for (const auto& source : sources) {
                mask |= weapon_generated_source_completeness_policy::partMask(source.semantic.partKind);
            }
            return mask;
        };

        auto logGeneratedSourceInventory = [&](const char* reason, const std::vector<GeneratedHullSource>& sources) {
            const auto semanticMask = generatedSourceSemanticMask(sources);
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon mesh source inventory: reason={} label='mergedCandidates' root='{}' candidates={} acceptedCandidates={} sources={} maxConvexes={} semanticMask=0x{:08X} parts='{}'",
                reason,
                safeNodeName(packageDriveRoot),
                candidates.size(),
                acceptedCandidateCount,
                sources.size(),
                MAX_WEAPON_BODIES,
                semanticMask,
                generatedWeaponSemanticMaskNames(semanticMask));

            for (std::size_t i = 0; i < sources.size(); ++i) {
                const auto& source = sources[i];
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon mesh source inventory[{}]: source='{}' driveRoot='{}' sourceRoot='{}' part={} partKind={} points={} group={:x} boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f})",
                    i,
                    source.sourceName,
                    safeNodeName(source.driveRoot),
                    safeNodeName(source.sourceRoot),
                    generatedWeaponPartKindName(source.semantic.partKind),
                    static_cast<int>(source.semantic.partKind),
                    source.localPointsGame.size(),
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
            const std::size_t extractedCount = outSources.size();
            const std::size_t droppedCount = extractedCount - MAX_WEAPON_BODIES;
            std::vector<weapon_collision_geometry_math::HullSelectionInput> selectionInputs;
            selectionInputs.reserve(outSources.size());
            for (const auto& source : outSources) {
                selectionInputs.push_back(makeHullSelectionInput(
                    source.localCenterGame,
                    source.localMinGame,
                    source.localMaxGame,
                    source.localPointsGame.size(),
                    source.semantic));
            }

            const auto selectedIndices =
                weapon_collision_geometry_math::selectBalancedHullIndices(selectionInputs, MAX_WEAPON_BODIES);
            std::vector<GeneratedHullSource> selectedSources;
            selectedSources.reserve(selectedIndices.size());
            for (const auto selectedIndex : selectedIndices) {
                selectedSources.push_back(std::move(outSources[selectedIndex]));
            }
            outSources = std::move(selectedSources);
            ROCK_LOG_WARN(Weapon,
                "Generated weapon mesh body cap reached: extracted={} kept={} dropped={} policy=balanced-semantic-coverage",
                extractedCount,
                outSources.size(),
                droppedCount);
        } else if (outSources.size() == MAX_WEAPON_BODIES) {
            logGeneratedSourceInventory("body-capacity-exact", outSources);
        }

        for (std::size_t i = 0; i < outSources.size(); ++i) {
            const auto& source = outSources[i];
            const auto coverage = classifyGeneratedHullSemantic(source.semantic);
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
            const float sourceNodeScale = source.sourceNodeScale;
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

        if (totalCulledForEffectGeometry > 0) {
            ROCK_LOG_INFO(Weapon,
                "Generated weapon effect geometry filter: excluded {} visual-only shape(s) from collision root='{}' policy=effect-shader+billboard+role-name",
                totalCulledForEffectGeometry,
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
        std::uint32_t& culledForEffectGeometry)
    {
        if (!node || depth > 15) {
            return;
        }
        if (node->GetAppCulled()) {
            ROCK_LOG_TRACE(Weapon,
                "{}generated mesh source branch skipped '{}': ancestor branch is app-culled",
                std::string(depth * 2, ' '),
                safeNodeName(node));
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

            const auto effectExclusionReason = classifyGeneratedWeaponEffectGeometry(triShape);
            if (effectExclusionReason != weapon_effect_geometry_policy::ExclusionReason::None) {
                ++culledForEffectGeometry;
                ROCK_LOG_TRACE(Weapon,
                    "{}generated mesh source skipped '{}': visual effect geometry reason={}",
                    std::string(depth * 2, ' '),
                    safeNodeName(node),
                    weapon_effect_geometry_policy::exclusionReasonName(effectExclusionReason));
                return;
            }
            ++visitedShapes;

            std::vector<TriangleData> triangles;
            std::vector<TriangleData> directSourceLocalTriangles;
            const bool skinned = isSkinned(triShape);
            const int added = skinned ?
                                  extractTrianglesFromSkinnedTriShape(
                                      triShape,
                                      triangles,
                                      nullptr,
                                      false,
                                      &directSourceLocalTriangles) :
                                  extractTrianglesFromTriShape(
                                      triShape,
                                      triangles,
                                      nullptr,
                                      &directSourceLocalTriangles);
            if (added <= 0) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), safeNodeName(node));
                return;
            }
            extractedTriangles += static_cast<std::uint32_t>(added);
            candidateExtractedSourceGroups.insert(sourceGroupId);

            RE::NiTransform sourceInWeapon{};
            const bool sourceInWeaponAvailable =
                tryResolveDescendantLocalTransform(sourceRoot, node, sourceInWeapon);
            RE::NiTransform sourceWorldForDrive = node->world;
            if (sourceInWeaponAvailable) {
                sourceWorldForDrive = transform_math::composeTransforms(
                    weaponRootTransform,
                    sourceInWeapon);
            }

            std::vector<RE::NiPoint3> localPoints;
            localPoints.reserve(triangles.size() * 3);
            std::vector<TriangleData> localTriangles;
            localTriangles.reserve(triangles.size());
            std::vector<TriangleData> sourceLocalTriangles;
            sourceLocalTriangles.reserve(triangles.size());
            const bool hasDirectSourceLocalTriangles =
                directSourceLocalTriangles.size() == triangles.size();
            for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
                const auto& triangle = triangles[triangleIndex];
                TriangleData sourceLocalTriangle{};
                if (hasDirectSourceLocalTriangles) {
                    /*
                     * Preserve the extractor's native local vertices. Reversing
                     * already-transformed world vertices loses enough float
                     * precision in distant cells to look like mesh mutation and
                     * incorrectly reject an otherwise valid shoulder restore.
                     */
                    sourceLocalTriangle = directSourceLocalTriangles[triangleIndex];
                } else {
                    sourceLocalTriangle.v0 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v0);
                    sourceLocalTriangle.v1 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v1);
                    sourceLocalTriangle.v2 = weapon_collision_geometry_math::worldPointToLocal(node->world.rotate, node->world.translate, node->world.scale, triangle.v2);
                }
                TriangleData localTriangle{};
                if (sourceInWeaponAvailable) {
                    localTriangle.v0 = transform_math::localPointToWorld(sourceInWeapon, sourceLocalTriangle.v0);
                    localTriangle.v1 = transform_math::localPointToWorld(sourceInWeapon, sourceLocalTriangle.v1);
                    localTriangle.v2 = transform_math::localPointToWorld(sourceInWeapon, sourceLocalTriangle.v2);
                } else {
                    localTriangle.v0 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v0);
                    localTriangle.v1 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v1);
                    localTriangle.v2 = weapon_collision_geometry_math::worldPointToLocal(weaponRootTransform.rotate, weaponRootTransform.translate, weaponRootTransform.scale, triangle.v2);
                }
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

            /*
             * Structure anchors outrank NIF name tokens. A P-* slot is the
             * attachment owner and therefore outranks a nearer animation rig
             * node such as WeaponMagazine; explicit ammunition mesh names are
             * preserved by the record-identity policy. The walk is bounded and
             * purely upward, so it needs no recursion-state threading and stays
             * valid for cached sources (the OMOD set is part of the weapon
             * generation identity).
             */
            auto sourceSemantic = classifyWeaponPartName(safeNodeName(node));
            {
                auto slotAnchor = weapon_part_record_identity_policy::StructureAnchor::None;
                auto rigAnchor = weapon_part_record_identity_policy::StructureAnchor::None;
                RE::NiAVObject* ancestor = node->parent;
                for (int step = 0; ancestor && step < 24; ++step, ancestor = ancestor->parent) {
                    const auto candidateAnchor = weapon_part_record_identity_policy::resolveStructureAnchor(safeNodeName(ancestor));
                    if (candidateAnchor == weapon_part_record_identity_policy::StructureAnchor::RigBolt ||
                        candidateAnchor == weapon_part_record_identity_policy::StructureAnchor::RigMagazineDisplay) {
                        if (rigAnchor == weapon_part_record_identity_policy::StructureAnchor::None) {
                            rigAnchor = candidateAnchor;
                        }
                        continue;
                    }
                    if (candidateAnchor != weapon_part_record_identity_policy::StructureAnchor::None) {
                        slotAnchor = candidateAnchor;
                        break;
                    }
                }
                const auto structureAnchor = weapon_part_record_identity_policy::chooseStructureAnchor(slotAnchor, rigAnchor);
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
                    if (sourceInWeaponAvailable) {
                        source.sourceLocalPointsGame.push_back(
                            transform_math::worldPointToLocal(sourceInWeapon, point));
                    } else {
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
                source.sourceInWeapon = sourceInWeapon;
                source.sourceInWeaponAvailable = sourceInWeaponAvailable;
                source.sourceNodeScale = sourceWorldForDrive.scale;
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
                        culledForEffectGeometry);
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
        // One source, one convex hull. There is no compound path: nothing ever
        // populates a per-source child cluster list, so every generated source bakes
        // into exactly one hull built from its own point cloud.
        auto buildSourceShape = [&](const GeneratedHullSource& source) -> RE::hknpShape* {
            // Prefer the source-node-local cloud when extraction produced one - it is
            // centered on the source node itself, so the body can drive that node
            // directly instead of being rebased through the weapon root every frame.
            const bool useSourceLocal = !source.sourceLocalPointsGame.empty();
            const auto& sourcePoints = useSourceLocal ? source.sourceLocalPointsGame : source.localPointsGame;
            const auto& sourceCenter = useSourceLocal ? source.sourceLocalCenterGame : source.localCenterGame;
            // Bake NiNode scale in now: Havok never re-applies node scale to a built shape.
            const float sourceScale = useSourceLocal ? source.sourceNodeScale : 1.0f;
            auto centeredHavokPoints = makeCenteredHavokPointCloud(sourcePoints, sourceCenter, sourceScale);
            return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredHavokPoints, g_rockConfig.rockWeaponCollisionConvexRadius);
        };

        while (nextSourceIndex < sources.size() && createdCount < MAX_WEAPON_BODIES && attemptedThisFrame < maxSourceAttemptsThisFrame) {
            const std::size_t sourceIndex = nextSourceIndex++;
            ++attemptedThisFrame;
            const auto& source = sources[sourceIndex];
            const bool useSourceLocal = !source.sourceLocalPointsGame.empty();
            const auto& shapePoints = useSourceLocal ? source.sourceLocalPointsGame : source.localPointsGame;
            const float shapePointScale = useSourceLocal ? source.sourceNodeScale : 1.0f;
            if (!pointCloudCanBuildHull(shapePoints, shapePointScale)) {
                ROCK_LOG_DEBUG(Weapon,
                    "Generated weapon mesh hull '{}' rejected before native shape build: points={} sourceLocal={} sourceScale={:.4f} effective hull diagonal below {:.2f} game units",
                    source.sourceName,
                    shapePoints.size(),
                    useSourceLocal,
                    shapePointScale,
                    MIN_HULL_DIAGONAL_GAME_UNITS);
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
            instance.generatedSourceGroupId = source.sourceGroupId;
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
            RE::NiTransform driveRootTransform =
                instance.sourceNode ?
                    instance.sourceNode->world :
                    (instance.driveNode ? instance.driveNode->world : makeIdentityTransform());
            if (instance.sourceNode && instance.driveNode) {
                RE::NiTransform hierarchyWorld{};
                if (tryResolveDescendantWorldTransform(
                        instance.driveNode,
                        instance.driveNode->world,
                        instance.sourceNode,
                        hierarchyWorld)) {
                    driveRootTransform = hierarchyWorld;
                }
            }
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
                "Generated weapon mesh collision body created: meshIndex={} bodyId={} source='{}' driveRoot='{}' sourceRoot='{}' partKind={} supportRole={} reloadRole={} points={} center=({:.2f},{:.2f},{:.2f}) layer=44",
                createdCount, instance.body.getBodyId().value, source.sourceName, safeNodeName(source.driveRoot), safeNodeName(source.sourceRoot), static_cast<int>(source.semantic.partKind),
                static_cast<int>(source.semantic.supportGripRole), static_cast<int>(source.semantic.reloadRole), source.localPointsGame.size(),
                source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
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
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
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

        clearEquippedWeaponIdentityState(ClearScope::ScaleInvalidation, world);
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
        instance.generatedSourceGroupId = 0;
        instance.semantic = {};
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
        instance.publicationIndex = INVALID_BODY_ID;
    }

    // Seqlock write edge: force the version odd so every readUnderSeqlock() in
    // flight discards what it read.
    void WeaponCollision::beginWeaponBodyPublication()
    {
        const std::uint64_t version = _weaponBodyPublicationVersion.load(std::memory_order_relaxed);
        _weaponBodyPublicationVersion.store((version & ~1ull) + 1ull, std::memory_order_release);
    }

    // Seqlock write edge: back to even, at a new value, so readers can see that
    // the set they copied is whole and current.
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
            std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
            _profileEvidenceSnapshot.clear();
            _weaponEmitterSnapshot = {};
            _nativeScopeSightAnchorSnapshot = {};
            _weaponCompositionSnapshot = {};
        }
        endWeaponBodyPublication();
    }

    void WeaponCollision::publishAtomicBodyIds(WeaponBodyBank& bank)
    {
        WeaponCompositionSnapshot weaponCompositionSnapshot{};
        auto evidenceSnapshot = buildProfileEvidenceSnapshot(
            bank,
            weaponCompositionSnapshot);
        weaponCompositionSnapshot.publicationSequence =
            ++_weaponCompositionPublicationSequence;
        RE::NiAVObject* packageDriveNode = resolvePackageDriveNode(bank, nullptr);
        NativeScopeSightAnchorSnapshot nativeScopeSightAnchorSnapshot = buildNativeScopeSightAnchorSnapshot(
            _cachedWeaponBodySetKey,
            _cachedWeaponOwnershipKey,
            _cachedWeaponFormID,
            evidenceSnapshot);
        applyEquippedManualScopeTarget(packageDriveNode, nativeScopeSightAnchorSnapshot);
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
            std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
            _profileEvidenceSnapshot = std::move(evidenceSnapshot);
            _nativeScopeSightAnchorSnapshot = nativeScopeSightAnchorSnapshot;
            _weaponCompositionSnapshot = weaponCompositionSnapshot;
        }
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

            RE::NiTransform sourceWorld{};
            const bool useSourceNode = instance.sourceNode &&
                tryResolveDescendantWorldTransform(
                    packageDriveNode,
                    packageWorld,
                    instance.sourceNode,
                    sourceWorld);
            const RE::NiTransform& driveWorld = useSourceNode ? sourceWorld : packageWorld;
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
        const auto publishedGeneration = getCurrentWeaponGenerationKey();
        if (!world || publishedGeneration == 0) {
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
                    bodyIndex),
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
