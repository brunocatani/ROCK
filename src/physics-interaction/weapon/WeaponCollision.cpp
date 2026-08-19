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
    // The shared WeaponCollision helpers live in a named detail namespace so they
    // cannot collide with the same-named local helpers other physics-interaction TUs
    // define; they are used unqualified here, as in every WeaponCollision TU.
    using namespace weapon_collision_detail;

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
}
