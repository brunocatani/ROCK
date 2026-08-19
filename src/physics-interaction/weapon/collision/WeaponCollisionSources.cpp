#include "physics-interaction/weapon/collision/WeaponCollision.h"

/*
 * Where colliders come from: turning the assembled weapon's rendered geometry into
 * the hull sources ROCK builds Havok bodies from.
 *
 * The scan runs in this order, and the order matters:
 *   1. walk the weapon tree and extract triangles per candidate source node
 *      (findGeneratedWeaponShapeSourcesRecursive), skipping effect-only geometry -
 *      a muzzle flash must never become a collider
 *   2. merge the candidate roots into one source set, since several scanned roots
 *      can reach the same physical weapon
 *   3. refine each source's semantic part kind from installed-OMOD and live emitter
 *      evidence, so a magazine is known to be a magazine
 *   4. drop detached components: geometry that sits far from the weapon body and
 *      belongs to something else the scan happened to reach
 *   5. select within the body-count budget, keeping gameplay-critical parts
 *
 * Sources are extracted in the SOURCE NODE's own local frame wherever possible, not
 * the weapon root's, so a part that animates independently (pump, bolt, magazine)
 * carries a collider that follows it. The captured source-node scale travels with
 * the source, because Havok never re-applies NiNode scale to a shape it has built.
 *
 * GeneratedHullSource is a private nested type, so everything here that touches one
 * is a member function rather than a free helper.
 */

#include "physics-interaction/weapon/collision/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/collision/WeaponEmitterScan.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/native/hooks/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/weapon/parts/WeaponAccessoryPartKindPolicy.h"
#include "physics-interaction/weapon/collision/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/collision/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/parts/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rock
{
    using namespace weapon_collision_detail;

    namespace
    {
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
    }

    /*
     * Phase 1 - merge. Walk every candidate root and fold what each one reaches
     * into one source set.
     *
     * Candidate roots are discovery witnesses for the same equipped package, not
     * competitors: several of them legitimately reach the same weapon, so a
     * TriShape is accepted once by source pointer and the rest are skipped rather
     * than one root being allowed to hide geometry from the others. Every source is
     * converted into the update weapon root's local frame.
     *
     * Returns false when nothing was found, in which case outStats is not filled.
     */
    bool WeaponCollision::mergeGeneratedWeaponSourceCandidates(
        RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey,
        std::vector<GeneratedHullSource>& outSources,
        GeneratedSourceScanStats& outStats)
    {
        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
        if (candidates.empty()) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon root candidates");
            return false;
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
            return false;
        }

        outStats.packageDriveRoot = packageDriveRoot;
        outStats.weaponRootScale = packageDriveRootTransform.scale;
        outStats.candidateCount = candidates.size();
        outStats.acceptedCandidateCount = acceptedCandidateCount;
        outStats.claimedShapeCount = claimedSourceGroups.size();
        outStats.visitedShapes = totalVisitedShapes;
        outStats.extractedTriangles = totalExtractedTriangles;
        outStats.culledForEffectGeometry = totalCulledForEffectGeometry;
        return true;

    }

    /*
     * Phase 2 - refine. A source's part kind starts as a name-based guess; this
     * confirms or corrects it from the same installed-OMOD and live-emitter
     * evidence ROCK publishes through the provider API.
     *
     * Exact OMOD identity is authoritative for standard attach points. Mod-added
     * P-* slots have no vanilla attach-point FormID, so they fall back to the
     * bounded owner subtree captured during the same traversal; a candidate-root
     * fallback is explicitly rejected by ownerRootStructural, because "somewhere
     * under the weapon" is not evidence of ownership.
     */
    void WeaponCollision::refineGeneratedWeaponSourceSemantics(
        RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey,
        std::vector<GeneratedHullSource>& sources) const
    {
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
        for (auto& source : sources) {
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

    }

    /*
     * Phase 3 - exclude detached components. The scan reaches everything under the
     * weapon roots, which can include geometry that is not part of this weapon at
     * all. A component that sits beyond the minimum gap from the assembled anchor
     * is dropped.
     *
     * Exclusions are cached by source group for the current equipped key, because
     * the same stray component reappears every frame and re-deriving it is the
     * expensive part. The filter FAILS OPEN: on invalid input, too many sources, or
     * no identifiable anchor it keeps everything, since dropping real weapon
     * geometry is far worse than keeping a stray hull.
     */
    void WeaponCollision::excludeDetachedGeneratedWeaponSources(
        std::uint64_t equippedWeaponKey,
        std::vector<GeneratedHullSource>& sources)
    {
        const std::size_t cachedDetachedSourceCount = std::erase_if(sources, [&](const GeneratedHullSource& source) {
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

        if (!sources.empty()) {
            std::vector<weapon_collision_geometry_math::DetachedComponentInput> componentInputs;
            componentInputs.reserve(sources.size());
            for (const auto& source : sources) {
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
                std::vector<std::uint8_t> excluded(sources.size(), 0);
                std::size_t newlyCachedGroups = 0;
                const std::string representativeName = sources[componentFilter.excludedIndices.front()].sourceName;
                for (const auto sourceIndex : componentFilter.excludedIndices) {
                    if (sourceIndex >= sources.size()) {
                        continue;
                    }
                    excluded[sourceIndex] = 1;
                    const auto sourceGroupId = sources[sourceIndex].sourceGroupId;
                    if (sourceGroupId != 0 &&
                        _detachedSourceExclusionGroups.size() < MAX_CACHED_DETACHED_SOURCE_GROUPS) {
                        newlyCachedGroups += _detachedSourceExclusionGroups.insert(sourceGroupId).second ? 1u : 0u;
                    }
                }

                std::vector<GeneratedHullSource> retainedSources;
                retainedSources.reserve(sources.size() - componentFilter.excludedIndices.size());
                for (std::size_t sourceIndex = 0; sourceIndex < sources.size(); ++sourceIndex) {
                    if (!excluded[sourceIndex]) {
                        retainedSources.push_back(std::move(sources[sourceIndex]));
                    }
                }
                const auto excludedSourceCount = sources.size() - retainedSources.size();
                sources = std::move(retainedSources);
                ROCK_LOG_INFO(Weapon,
                    "Generated weapon detached component filter: excludedComponents={} excludedSources={} retainedSources={} components={} anchorComponents={} nearestGap={:.2f} joinTolerance={:.2f} minimumGap={:.2f} newlyCachedGroups={} cachedGroups={} representative='{}' equippedKey={:016X}",
                    componentFilter.excludedComponentCount,
                    excludedSourceCount,
                    sources.size(),
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
                    sources.size(),
                    componentFilter.componentCount,
                    componentFilter.assembledAnchorComponentCount,
                    equippedWeaponKey);
            }
        }

    }

    /*
     * Phase 4 - select within capacity. MAX_WEAPON_BODIES is a hard Havok budget,
     * so when extraction produces more sources than that, the balanced-coverage
     * policy decides which survive: it keeps the semantic spread rather than the
     * biggest hulls, so a weapon does not lose its magazine to keep three pieces of
     * receiver.
     *
     * Also the final inventory logging, including the source-node-scale line that
     * makes weapon-root vs source-node bounds divergence visible instead of
     * inferred.
     */
    void WeaponCollision::selectGeneratedWeaponSourcesWithinCapacity(
        const GeneratedSourceScanStats& stats,
        std::vector<GeneratedHullSource>& sources)
    {
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
                safeNodeName(stats.packageDriveRoot),
                stats.candidateCount,
                stats.acceptedCandidateCount,
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

        if (sources.size() > MAX_WEAPON_BODIES) {
            logGeneratedSourceInventory("body-capacity-overflow", sources);
            const std::size_t extractedCount = sources.size();
            const std::size_t droppedCount = extractedCount - MAX_WEAPON_BODIES;
            std::vector<weapon_collision_geometry_math::HullSelectionInput> selectionInputs;
            selectionInputs.reserve(sources.size());
            for (const auto& source : sources) {
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
                selectedSources.push_back(std::move(sources[selectedIndex]));
            }
            sources = std::move(selectedSources);
            ROCK_LOG_WARN(Weapon,
                "Generated weapon mesh body cap reached: extracted={} kept={} dropped={} policy=balanced-semantic-coverage",
                extractedCount,
                sources.size(),
                droppedCount);
        } else if (sources.size() == MAX_WEAPON_BODIES) {
            logGeneratedSourceInventory("body-capacity-exact", sources);
        }

        for (std::size_t i = 0; i < sources.size(); ++i) {
            const auto& source = sources[i];
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
                stats.weaponRootScale);
        }

        ROCK_LOG_DEBUG(Weapon,
            "Generated weapon mesh source merged: root='{}' candidates={} acceptedCandidates={} claimedShapes={} visitedShapes={} triangles={} hulls={}",
            safeNodeName(stats.packageDriveRoot),
            stats.candidateCount,
            stats.acceptedCandidateCount,
            stats.claimedShapeCount,
            stats.visitedShapes,
            stats.extractedTriangles,
            sources.size());

        if (stats.culledForEffectGeometry > 0) {
            ROCK_LOG_INFO(Weapon,
                "Generated weapon effect geometry filter: excluded {} visual-only shape(s) from collision root='{}' policy=effect-shader+billboard+role-name",
                stats.culledForEffectGeometry,
                safeNodeName(stats.packageDriveRoot));
        }

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

        // 1. Discover and merge. Several scanned roots reach the same weapon.
        GeneratedSourceScanStats stats{};
        if (!mergeGeneratedWeaponSourceCandidates(weaponNode, equippedWeaponKey, outSources, stats)) {
            return 0;
        }

        // 2. Refine. Name-based part kinds are a guess until OMOD and emitter
        //    evidence confirms them, and the later phases rank on part kind.
        refineGeneratedWeaponSourceSemantics(weaponNode, equippedWeaponKey, outSources);

        // 3. Drop geometry that belongs to something else the scan happened to
        //    reach. This runs after refinement because it needs the part kinds to
        //    know which components are assembled anchors.
        excludeDetachedGeneratedWeaponSources(equippedWeaponKey, outSources);
        if (outSources.empty()) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon mesh source scan: no collider sources remain after detached-component exclusions equippedKey={:016X}",
                equippedWeaponKey);
            return 0;
        }

        // 4. Fit the result into the Havok body budget.
        selectGeneratedWeaponSourcesWithinCapacity(stats, outSources);
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
}
