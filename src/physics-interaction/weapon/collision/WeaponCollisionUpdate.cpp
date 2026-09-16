#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// Frame update and Havok drive: init/shutdown, world-loss handling, the per-frame update, source-transform body updates, and drive flushing.

namespace rock
{
    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        _identity.classificationValid = false;
        // Cache the Havok context for the generated weapon-collision lifetime.
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _identity.cachedWeaponKey = 0;
        _identity.cachedVisualKey = 0;
        _identity.cachedIdentityKey = 0;
        _identity.cachedOwnershipKey = 0;
        _identity.cachedFormID = 0;
        _identity.observedIdentityKey = 0;
        _identity.observedOwnershipKey = 0;
        _identity.observedFormID = 0;
        _identity.observedInstanceContentKey = 0;
        resetWeaponBodySetGeneration();
        _identity.bodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        _sources.detachedExclusionEquippedKey = 0;
        _sources.detachedExclusionGroups.clear();
        _diagnostics.generatedRecapture = {};
        clearPendingGeneratedWeaponBuild(world, false);
        _bodies.usingReplacementBank = false;
        _drive.rebuildRequested.store(false, std::memory_order_release);
        _drive.workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _drive.failureCount.store(0, std::memory_order_release);
        _diagnostics.animNodeDumpFrameCounter = 0;
        _diagnostics.lastAnimNodeDumpKey = 0;
        clearAtomicBodyIds();

        ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
    }

    void WeaponCollision::shutdown()
    {
        _identity.classificationValid = false;
        if (hasWeaponBody()) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown destroying generated bodies from cached context");
            destroyWeaponBody(_cachedWorld);
        }

        _identity.cachedWeaponKey = 0;
        _identity.cachedVisualKey = 0;
        _identity.cachedIdentityKey = 0;
        _identity.cachedOwnershipKey = 0;
        _identity.cachedFormID = 0;
        _identity.observedIdentityKey = 0;
        _identity.observedOwnershipKey = 0;
        _identity.observedFormID = 0;
        _identity.observedInstanceContentKey = 0;
        resetWeaponBodySetGeneration();
        _identity.bodySetEpoch = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        _sources.detachedExclusionEquippedKey = 0;
        _sources.detachedExclusionGroups.clear();
        _diagnostics.generatedRecapture = {};
        clearPendingGeneratedWeaponBuild(_cachedWorld, true);
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _bodies.usingReplacementBank = false;
        _drive.rebuildRequested.store(false, std::memory_order_release);
        _drive.workbenchExitRebuildRequested.store(false, std::memory_order_release);
        _drive.failureCount.store(0, std::memory_order_release);
        _diagnostics.animNodeDumpFrameCounter = 0;
        _diagnostics.lastAnimNodeDumpKey = 0;
        clearWeaponEmitterSnapshot();

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::abandonHavokStateAfterWorldLoss()
    {
        _identity.classificationValid = false;
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        clearAtomicBodyIds();
        resetWeaponBodySetGeneration();
        for (auto& instance : _bodies.bank) {
            clearWeaponBodyInstance(instance, true);
        }
        for (auto& instance : _bodies.replacementBank) {
            clearWeaponBodyInstance(instance, true);
        }
        _sources.pendingBuild = {};
        _diagnostics.generatedRecapture = {};
        _bodies.usingReplacementBank = false;
        _cachedWorld = nullptr;
        clearGeneratedSourceCache();
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
        _drive.workbenchExitRebuildRequested.store(true, std::memory_order_release);
    }

    void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, float dt, bool weaponDrawn)
    {
        (void)dt;

        auto clearCurrentWeaponState = [&]() {
            _identity.cachedWeaponKey = 0;
            _identity.cachedVisualKey = 0;
            _identity.cachedIdentityKey = 0;
            _identity.cachedOwnershipKey = 0;
            _identity.cachedFormID = 0;
            _identity.observedIdentityKey = 0;
            _identity.observedOwnershipKey = 0;
            _identity.observedFormID = 0;
            _identity.observedInstanceContentKey = 0;
            clearGeneratedSourceCompletenessTracking();
            clearPendingWeaponVisualRebuild();
            clearGeneratedSourceCache();
            clearPendingGeneratedWeaponBuild(world, true);
            resetVisualSourceUnavailableRetention();
            resetWeaponBodySetGeneration();
            _drive.rebuildRequested.store(false, std::memory_order_release);
            _drive.workbenchExitRebuildRequested.store(false, std::memory_order_release);
            _drive.failureCount.store(0, std::memory_order_release);
            clearWeaponEmitterSnapshot();
        };

        if (!world) {
            _sources.preparation.reset();
            return;
        }

        if (world != _cachedWorld) {
            _identity.classificationValid = false;
            ROCK_LOG_INFO(Weapon, "hknpWorld changed - resetting weapon collision state");
            if (hasWeaponBody()) {
                destroyWeaponBody(_cachedWorld ? _cachedWorld : world);
            } else {
                clearAtomicBodyIds();
            }
            _cachedWorld = world;
            _sources.detachedExclusionEquippedKey = 0;
            _sources.detachedExclusionGroups.clear();
            _diagnostics.generatedRecapture = {};
            clearCurrentWeaponState();
        }

        if (!weaponDrawn) {
            if (_diagnostics.generatedRecapture.valid) {
                _diagnostics.generatedRecapture.sawUndrawnInterval = true;
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
        _identity.observedIdentityKey = observedIdentityKey;
        _identity.observedOwnershipKey = observedOwnershipKey;
        _identity.observedFormID = observedFormID;
        _identity.observedInstanceContentKey = observedInstanceContentKey;

        const bool geometryModeChanged = _sources.preserveGaps != g_rockConfig.rockWeaponCollisionPreserveGaps;
        if (geometryModeChanged) {
            _sources.preserveGaps = g_rockConfig.rockWeaponCollisionPreserveGaps;
            clearGeneratedSourceCache();
            clearPendingWeaponVisualRebuild();
            ROCK_LOG_INFO(Weapon, "Weapon collider generation mode changed: preserveGaps={} rebuilding geometry", _sources.preserveGaps);
        }
        const bool geometryModeRebuildRequired = hasWeaponBody() && _sources.activePreserveGaps != _sources.preserveGaps;
        const bool driveRequestedRebuild = _drive.rebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool workbenchExitRequested =
            weaponNode != nullptr && _drive.workbenchExitRebuildRequested.exchange(false, std::memory_order_acq_rel);
        const bool keyChanged = observedKey != 0 && observedKey != _identity.cachedWeaponKey;
        const bool identityKeyChanged = observedIdentityKey != 0 && observedIdentityKey != _identity.cachedIdentityKey;
        const bool ownershipKeyChanged =
            _identity.cachedOwnershipKey != 0 && observedOwnershipKey != _identity.cachedOwnershipKey;
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
            ownershipKeyChanged || activeRootChanged || missingBodies || geometryModeChanged || geometryModeRebuildRequired;
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
            if (keyChanged && _identity.cachedWeaponKey != 0) {
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
                _identity.cachedWeaponKey,
                observedKey);
        }
        if (workbenchExitRequested) {
            ROCK_LOG_INFO(Weapon,
                "Workbench exit requested generated weapon collision rebuild cachedKey={:016X} observedKey={:016X}",
                _identity.cachedWeaponKey,
                observedKey);
        }

        if (_sources.preparation) {
            WeaponVisualKeyStats currentStats{};
            const auto currentVisual = weaponNode ? getWeaponVisualCompositionKey(weaponNode, currentStats) : 0;
            const auto& preparation = *_sources.preparation;
            const bool invalidated = driveRequestedRebuild || workbenchExitRequested || geometryModeChanged ||
                preparation.equippedKey != observedKey || preparation.ownershipKey != observedOwnershipKey ||
                preparation.root.get() != weaponNode || preparation.visualKey != currentVisual;
            if (invalidated) {
                ROCK_LOG_INFO(Weapon, "Weapon geometry preparation cancelled: key={:016X} current={:016X} visual={:016X}->{:016X}",
                    preparation.equippedKey, observedKey, preparation.visualKey, currentVisual);
                _sources.preparation.reset();
                clearPendingWeaponVisualRebuild();
                rebuildRequired = true;
            } else if (!preparation.ready) {
                (void)advanceSourcePreparation();
                return;
            } else {
                rebuildRequired = true;
            }
        }

        if (_sources.pendingBuild.active) {
            const bool pendingInvalidated = driveRequestedRebuild || workbenchExitRequested || activeRootChanged || geometryModeChanged ||
                !pendingGeneratedWeaponBuildMatches(
                    observedKey,
                    observedOwnershipKey,
                    weaponNode,
                    observedFormID);
            if (pendingInvalidated) {
                ROCK_LOG_INFO(Weapon,
                    "Generated weapon staged create cancelled: pendingKey={:016X} observedKey={:016X} pendingVisual={:016X} driveRebuild={} workbenchExit={}",
                    _sources.pendingBuild.equippedKey,
                    observedKey,
                    _sources.pendingBuild.visualKey,
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
                    _identity.cachedWeaponKey,
                    getWeaponBodyCount());
                clearPendingWeaponVisualRebuild();
                resetVisualSourceUnavailableRetention();
                return;
            }

            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon,
                    "Weapon visual node absent while rebuild required - destroying generated weapon bodies cachedKey={:016X} observedKey={:016X} missingBodies={} driveRebuild={} identityChanged={}",
                    _identity.cachedWeaponKey,
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
            const bool visualKeyChanged = observedVisualKey != 0 && observedVisualKey != _identity.cachedVisualKey;
            const bool generationDrivenRebuild = keyChanged || missingBodies || geometryModeRebuildRequired;
            const float requiredStableSeconds = (std::max)(0.0f, g_rockConfig.rockWeaponCollisionVisualStabilizationSeconds);
            const float measuredStabilizationDelta =
                std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
            const bool stabilizeVisualRebuild = generationDrivenRebuild && requiredStableSeconds > 0.0f;

            if (stabilizeVisualRebuild && !weaponVisualNodeVisible(weaponNode)) {
                const bool newInvisibleDeferred =
                    _sources.pendingVisualRebuildKey != observedKey ||
                    _sources.pendingVisualWitnessKey != observedVisualKey ||
                    _sources.pendingVisualVisibleTriShapeCount != 0 ||
                    _sources.pendingVisualStableSeconds != 0.0f;
                /*
                 * Weapon mod swaps can expose a transient app-culled Weapon root
                 * while child TriShapes still look locally visible. Replacing the
                 * active body set from that frame can lock in an incomplete hull
                 * inventory, so keep the current bodies until the visual tree has
                 * presented a stable, visible witness.
                 */
                _sources.pendingVisualRebuildKey = observedKey;
                _sources.pendingVisualWitnessKey = observedVisualKey;
                _sources.pendingVisualVisibleTriShapeCount = 0;
                _sources.pendingVisualStableSeconds = 0.0f;
                if (newInvisibleDeferred) {
                    performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualRootDeferred);
                }
                ROCK_LOG_SAMPLE_INFO(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon collision rebuild deferred: visual root not ready cachedKey={:016X} observedKey={:016X} root='{}' flags=0x{:X} appCulled={} visibleTriShapes={} visualNodes={} invisibleNodes={} requiredStableSeconds={:.3f}",
                    _identity.cachedWeaponKey,
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
                        _sources.pendingVisualRebuildKey == observedKey &&
                        _sources.pendingVisualWitnessKey == observedVisualKey &&
                        _sources.pendingVisualVisibleTriShapeCount == visualKeyStats.visibleTriShapeCount;

                    _sources.pendingVisualRebuildKey = observedKey;
                    _sources.pendingVisualWitnessKey = observedVisualKey;
                    _sources.pendingVisualVisibleTriShapeCount = visualKeyStats.visibleTriShapeCount;
                    // Measured elapsed stability only: an unmeasurable frame
                    // holds the wait instead of advancing it.
                    _sources.pendingVisualStableSeconds = samePendingVisual ?
                        _sources.pendingVisualStableSeconds + measuredStabilizationDelta :
                        measuredStabilizationDelta;

                    if (_sources.pendingVisualStableSeconds < requiredStableSeconds) {
                        if (!samePendingVisual) {
                            performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildVisualStableWait);
                        }
                        ROCK_LOG_SAMPLE_INFO(Weapon,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Generated weapon collision rebuild waiting for stable visual witness cachedKey={:016X} observedKey={:016X} stableSeconds={:.3f}/{:.3f} visualKey={:016X} visualRoots={} visibleTriShapes={} visualNodes={} invisibleNodes={}",
                            _identity.cachedWeaponKey,
                            observedKey,
                            _sources.pendingVisualStableSeconds,
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
                    generatedSources = _sources.cache.sources;
                    generatedSummary = _sources.cache.summary;
                    generatedCount = generatedSources.size();
                    usedCachedSources = true;
                    ROCK_LOG_DEBUG(Weapon,
                        "Generated weapon mesh source cache hit key={:016X} visualKey={:016X} sources={}",
                        observedKey,
                        observedVisualKey,
                        generatedCount);
                } else {
                    if (_sources.preserveGaps) {
                        if (!_sources.preparation) {
                            try {
                                auto preparation = std::make_unique<PendingSourcePreparation>();
                                preparation->root.reset(weaponNode);
                                preparation->equippedKey = observedKey;
                                preparation->ownershipKey = observedOwnershipKey;
                                preparation->visualKey = observedVisualKey;
                                preparation->task = prepareGeneratedWeaponShapeSources(preparation->root,
                                    observedKey, preparation->sources, true);
                                _sources.preparation = std::move(preparation);
                                ROCK_LOG_INFO(Weapon, "Weapon geometry preparation queued: key={:016X} visual={:016X}", observedKey, observedVisualKey);
                            } catch (...) {
                                ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation allocation failed: key={:016X}", observedKey);
                            }
                            return;
                        }
                        generatedSources = std::move(_sources.preparation->sources);
                        _sources.preparation.reset();
                        generatedCount = generatedSources.size();
                    } else {
                        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponColliderBuild);
                        generatedCount = findGeneratedWeaponShapeSources(weaponNode, observedKey, generatedSources);
                    }
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
                        _identity.cachedWeaponKey,
                        observedKey,
                        visualKeyStats.rootCount,
                        visualKeyStats.nodeCount,
                        visualKeyStats.visibleTriShapeCount,
                        generatedCount,
                        visualKeyStats.missingRendererCount + visualKeyStats.emptyGeometryCount,
                        visualKeyStats.invisibleNodeCount);

                    const bool sameEquippedIdentity =
                        observedIdentityKey != 0 &&
                        _identity.cachedIdentityKey != 0 &&
                        observedIdentityKey == _identity.cachedIdentityKey &&
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
                            _identity.cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            getWeaponBodyCount(),
                            _sources.visualUnavailableRetainSeconds,
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
                            _identity.cachedWeaponKey,
                            observedKey,
                            observedVisualKey,
                            _sources.visualUnavailableRetainSeconds,
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
                    _identity.cachedWeaponKey = 0;
                    _identity.cachedVisualKey = 0;
                    _identity.cachedIdentityKey = 0;
                    _identity.cachedOwnershipKey = 0;
                    _identity.cachedFormID = 0;
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
                        _identity.cachedWeaponKey,
                        observedKey,
                        generatedCount);
                    if (!replacingExisting) {
                        clearAtomicBodyIds();
                        resetWeaponBodySetGeneration();
                        _identity.cachedWeaponKey = 0;
                        _identity.cachedVisualKey = 0;
                        _identity.cachedIdentityKey = 0;
                        _identity.cachedOwnershipKey = 0;
                        _identity.cachedFormID = 0;
                        clearGeneratedSourceCompletenessTracking();
                    }
                    clearPendingWeaponVisualRebuild();
                    return;
                }

                performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildQueued);

                ROCK_LOG_INFO(Weapon,
                    "Generated weapon collision staged create queued cachedKey={:016X} observedKey={:016X} sources={} replacingExisting={} driveRebuild={} workbenchExit={} cachedSources={} batch={}",
                    _identity.cachedWeaponKey,
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
            _drive.rebuildRequested.store(true, std::memory_order_release);
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
            _drive.failureCount.store(0, std::memory_order_release);
            return;
        }

        if (!result.shouldRequestRebuild()) {
            return;
        }

        const auto failures = _drive.failureCount.fetch_add(1, std::memory_order_acq_rel) + 1;
        _drive.rebuildRequested.store(true, std::memory_order_release);
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
