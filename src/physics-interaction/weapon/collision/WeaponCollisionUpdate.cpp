#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// Frame update and Havok drive: init/shutdown, world-loss handling, the per-frame update, source-transform body updates, and drive flushing.

namespace rock
{
    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        // Cache the Havok context for the generated weapon-collision lifetime.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedWeaponKey = 0;
        _cachedWeaponVisualKey = 0;
        _cachedWeaponIdentityKey = 0;
        _cachedWeaponOwnershipKey = 0;
        _cachedWeaponFormID = 0;
        _observedEquippedWeaponIdentityKey = 0;
        _observedEquippedWeaponOwnershipKey = 0;
        _observedEquippedWeaponFormID = 0;
        _observedEquippedWeaponInstanceContentKey = 0;
        _omodPrebuildReconciliationEquippedKey = 0;
        _omodPrebuildReconciliationRoot = nullptr;
        resetWeaponBodySetGeneration();
        _weaponBodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        _detachedSourceExclusionEquippedKey = 0;
        _detachedSourceExclusionGroups.clear();
        _generatedRecaptureDiagnostic = {};
        clearPendingGeneratedWeaponBuild(world, false);
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = 0;
        clearAtomicBodyIds();

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
        _cachedWeaponOwnershipKey = 0;
        _cachedWeaponFormID = 0;
        _observedEquippedWeaponIdentityKey = 0;
        _observedEquippedWeaponOwnershipKey = 0;
        _observedEquippedWeaponFormID = 0;
        _observedEquippedWeaponInstanceContentKey = 0;
        _omodPrebuildReconciliationEquippedKey = 0;
        _omodPrebuildReconciliationRoot = nullptr;
        resetWeaponBodySetGeneration();
        _weaponBodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        _detachedSourceExclusionEquippedKey = 0;
        _detachedSourceExclusionGroups.clear();
        _generatedRecaptureDiagnostic = {};
        clearPendingGeneratedWeaponBuild(_cachedWorld, true);
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _usingReplacementWeaponBodies = false;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _weaponAnimNodeDumpFrameCounter = 0;
        _lastWeaponAnimNodeDumpKey = 0;
        clearWeaponEmitterSnapshot();

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
            _cachedWeaponKey = 0;
            _cachedWeaponVisualKey = 0;
            _cachedWeaponIdentityKey = 0;
            _cachedWeaponOwnershipKey = 0;
            _cachedWeaponFormID = 0;
            _observedEquippedWeaponIdentityKey = 0;
            _observedEquippedWeaponOwnershipKey = 0;
            _observedEquippedWeaponFormID = 0;
            _observedEquippedWeaponInstanceContentKey = 0;
            clearGeneratedSourceCompletenessTracking();
            clearPendingWeaponVisualRebuild();
            clearGeneratedSourceCache();
            clearPendingGeneratedWeaponBuild(world, true);
            resetVisualSourceUnavailableRetention();
            resetWeaponBodySetGeneration();
            _driveRebuildRequested.store(false, std::memory_order_release);
            _workbenchExitRebuildRequested.store(false, std::memory_order_release);
            _driveFailureCount.store(0, std::memory_order_release);
            _omodPrebuildReconciliationEquippedKey = 0;
            _omodPrebuildReconciliationRoot = nullptr;
            clearWeaponEmitterSnapshot();
        };

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
            if (_generatedRecaptureDiagnostic.valid) {
                _generatedRecaptureDiagnostic.sawUndrawnInterval = true;
            }
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

        const bool driveRequestedRebuild = _driveRebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool workbenchExitRequested =
            weaponNode != nullptr && _workbenchExitRebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool keyChanged = observedKey != 0 && observedKey != _cachedWeaponKey;
        const bool identityKeyChanged = observedIdentityKey != 0 && observedIdentityKey != _cachedWeaponIdentityKey;
        const bool ownershipKeyChanged =
            _cachedWeaponOwnershipKey != 0 && observedOwnershipKey != _cachedWeaponOwnershipKey;
        const bool hadPublishedWeaponBodies =
            getCurrentWeaponGenerationKey() != 0 &&
            getWeaponBodyCount() != 0 &&
            hasWeaponBody();
        const bool activeRootChanged =
            hadPublishedWeaponBodies && weaponNode && !activeWeaponBodyRootMatches(weaponNode);
        if (hadPublishedWeaponBodies &&
            (identityKeyChanged || ownershipKeyChanged || activeRootChanged || workbenchExitRequested)) {
            const char* transitionReason = identityKeyChanged ?
                "equipped-identity-changed" :
                (ownershipKeyChanged ?
                        "equipped-ownership-changed" :
                        (activeRootChanged ? "weapon-root-changed" : "scene-rebuild-requested"));
            retireActiveWeaponBodiesForSceneTransition(world, transitionReason);
        }
        updateWeaponEmitterSnapshot(weaponNode, observedKey);

        const bool missingBodies = observedKey != 0 && !hasWeaponBody();
        bool rebuildRequired = driveRequestedRebuild || workbenchExitRequested || keyChanged ||
            ownershipKeyChanged || activeRootChanged || missingBodies;
        bool rebuildDiagnosticsRecorded = false;

        const auto recordRebuildDiagnostics = [&]() {
            if (rebuildDiagnosticsRecorded) {
                return;
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
            const bool pendingInvalidated = driveRequestedRebuild || workbenchExitRequested || activeRootChanged ||
                !pendingGeneratedWeaponBuildMatches(
                    observedKey,
                    observedOwnershipKey,
                    weaponNode,
                    observedFormID);
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
            if (hasWeaponBody() && !keyChanged && !missingBodies && !driveRequestedRebuild) {
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
                    "Weapon visual node absent while rebuild required - destroying generated weapon bodies cachedKey={:016X} observedKey={:016X} missingBodies={} driveRebuild={} identityChanged={}",
                    _cachedWeaponKey,
                    observedKey,
                    missingBodies ? "yes" : "no",
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
            const bool omodPrebuildReconciliationCurrent =
                _omodPrebuildReconciliationEquippedKey == observedKey &&
                _omodPrebuildReconciliationRoot == weaponNode;
            if (generationDrivenRebuild &&
                !omodPrebuildReconciliationCurrent) {
                const auto reconciliation = maybeRunWeaponOmodReconciliation(weaponNode, observedKey, true);
                if (reconciliation.sceneEnriched) {
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
                if (reconciliation.ran) {
                    // Cache only a non-mutating pass. A successful attachment
                    // must be followed by another pre-build pass so batches
                    // larger than the per-pass cap fully converge.
                    _omodPrebuildReconciliationEquippedKey = observedKey;
                    _omodPrebuildReconciliationRoot = weaponNode;
                }
            }
            const float requiredStableSeconds = (std::max)(0.0f, g_rockConfig.rockWeaponCollisionVisualStabilizationSeconds);
            const float measuredStabilizationDelta =
                std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
            const bool stabilizeVisualRebuild = generationDrivenRebuild && requiredStableSeconds > 0.0f;

            if (stabilizeVisualRebuild && !weaponVisualNodeVisible(weaponNode)) {
                const bool newInvisibleDeferred =
                    _pendingWeaponVisualRebuildKey != observedKey ||
                    _pendingWeaponVisualWitnessKey != observedVisualKey ||
                    _pendingWeaponVisualVisibleTriShapeCount != 0 ||
                    _pendingWeaponVisualStableSeconds != 0.0f;
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
                _pendingWeaponVisualStableSeconds = 0.0f;
                if (newInvisibleDeferred) {
                    performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualRootDeferred);
                }
                ROCK_LOG_SAMPLE_INFO(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon collision rebuild deferred: visual root not ready cachedKey={:016X} observedKey={:016X} root='{}' flags=0x{:X} appCulled={} visibleTriShapes={} visualNodes={} invisibleNodes={} requiredStableSeconds={:.3f}",
                    _cachedWeaponKey,
                    observedKey,
                    safeNodeName(weaponNode),
                    static_cast<std::uint32_t>(weaponNode->flags.flags),
                    weaponNode->GetAppCulled() ? "yes" : "no",
                    visualKeyStats.visibleTriShapeCount,
                    visualKeyStats.nodeCount,
                    visualKeyStats.invisibleNodeCount,
                    requiredStableSeconds);
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
                    // Measured elapsed stability only: an unmeasurable frame
                    // holds the wait instead of advancing it.
                    _pendingWeaponVisualStableSeconds = samePendingVisual ?
                        _pendingWeaponVisualStableSeconds + measuredStabilizationDelta :
                        measuredStabilizationDelta;

                    if (_pendingWeaponVisualStableSeconds < requiredStableSeconds) {
                        if (!samePendingVisual) {
                            performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualStableWait);
                        }
                        ROCK_LOG_SAMPLE_INFO(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon collision rebuild waiting for stable visual witness cachedKey={:016X} observedKey={:016X} stableSeconds={:.3f}/{:.3f} visualKey={:016X} visualRoots={} visibleTriShapes={} visualNodes={} invisibleNodes={}",
                            _cachedWeaponKey,
                            observedKey,
                            _pendingWeaponVisualStableSeconds,
                            requiredStableSeconds,
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

                if (generatedSourceCacheMatches(
                        observedKey,
                        observedOwnershipKey,
                        observedVisualKey,
                        weaponNode)) {
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
                        !driveRequestedRebuild;
                    const float visualSourceMissRetainSecondsLimit = (std::max)(0.011f, requiredStableSeconds);
                    if (retainCandidate &&
                        canRetainCurrentWeaponBodiesForVisualSourceMiss(observedIdentityKey, weaponNode, visualSourceMissRetainSecondsLimit, measuredStabilizationDelta)) {
                        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualSourceUnavailableRetained);
                        /*
                         * The visible tree can briefly report no extractable
                         * TriShapes while the same equipped weapon identity and
                         * package root are still live. Keep the current body set
                         * only for a bounded window; actual identity, root, or
                         * drive changes still fall through and destroy stale
                         * collision.
                         */
                        ROCK_LOG_SAMPLE_INFO(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon mesh collision unavailable for same equipped identity - retaining current bodies cachedKey={:016X} observedKey={:016X} visualKey={:016X} bodies={} retainSeconds={:.3f}/{:.3f}",
                            _cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            getWeaponBodyCount(),
                            _visualSourceUnavailableRetainSeconds,
                            visualSourceMissRetainSecondsLimit);
                        clearPendingGeneratedWeaponBuild(world, true);
                        clearPendingWeaponVisualRebuild();
                        return;
                    }
                    if (retainCandidate) {
                        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualSourceUnavailableRetainExpired);
                        ROCK_LOG_SAMPLE_WARN(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon mesh collision same-identity retain window expired cachedKey={:016X} observedKey={:016X} visualKey={:016X} retainSeconds={:.3f} limit={:.3f} - destroying stale bodies",
                            _cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            _visualSourceUnavailableRetainSeconds,
                            visualSourceMissRetainSecondsLimit);
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
                    _cachedWeaponOwnershipKey = 0;
                    _cachedWeaponFormID = 0;
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
                    storeGeneratedSourceCache(
                        observedKey,
                        observedOwnershipKey,
                        observedVisualKey,
                        weaponNode,
                        generatedSources,
                        generatedSummary);
                }

                recordRebuildDiagnostics();

                if (!beginPendingGeneratedWeaponBuild(
                        observedKey,
                        observedVisualKey,
                        observedIdentityKey,
                        observedOwnershipKey,
                        weaponNode,
                        observedFormID,
                        visualKeyStats,
                        replacingExisting,
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
                        _cachedWeaponOwnershipKey = 0;
                        _cachedWeaponFormID = 0;
                        clearGeneratedSourceCompletenessTracking();
                    }
                    clearPendingWeaponVisualRebuild();
                    return;
                }

                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildQueued);

                ROCK_LOG_INFO(Weapon,
                    "Generated weapon collision staged create queued cachedKey={:016X} observedKey={:016X} sources={} replacingExisting={} driveRebuild={} workbenchExit={} cachedSources={} batch={}",
                    _cachedWeaponKey,
                    observedKey,
                    generatedCount,
                    replacingExisting ? "yes" : "no",
                    driveRequestedRebuild ? "yes" : "no",
                    workbenchExitRequested ? "yes" : "no",
                    usedCachedSources ? "yes" : "no",
                    GENERATED_WEAPON_BODY_CREATION_BATCH);
                return;
            }
        }

        maybeRunWeaponOmodReconciliation(weaponNode, observedKey);
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
        if (!fallbackWeaponNode || !activeWeaponBodyRootMatches(fallbackWeaponNode)) {
            _driveRebuildRequested.store(true, std::memory_order_release);
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon source update rejected: cached root 0x{:X} does not own current root 0x{:X}; requesting rebuild",
                static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(cachedPackageDriveNode)),
                static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(fallbackWeaponNode)));
            return;
        }
        RE::NiAVObject* packageDriveNode = fallbackWeaponNode;
        const RE::NiTransform packageWorld = packageDriveNode->world;

        (void)drivenSourceNodes;
        (void)drivenSourceNodeCount;

        for (std::size_t i = 0; i < bank.size(); ++i) {
            auto& instance = bank[i];
            if (!instance.body.isValid()) {
                continue;
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
                    bodyIndex,
                    g_rockConfig.rockWeaponCollisionMaxLinearVelocity,
                    g_rockConfig.rockWeaponCollisionMaxAngularVelocity),
                "weapon-collision",
                bodyIndex);
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
