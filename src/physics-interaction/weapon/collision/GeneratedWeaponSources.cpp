#include "physics-interaction/weapon/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/WeaponGapDecomposition.h"
#include "physics-interaction/core/RockRuntimeState.h"

#include <chrono>

// Generated weapon collision sources: visual-source discovery, completeness tracking, source cache, incremental pending builds, and body creation.

namespace rock
{
    namespace
    {
        weapon_geometry_work::Task dedupeWeaponPointsDeferred(const std::vector<RE::NiPoint3>& points,
            float grid, std::vector<RE::NiPoint3>& unique)
        {
            weapon_geometry_work::Quantum quantum;
            std::unordered_set<QuantizedPointKey, QuantizedPointKeyHash> seen;
            seen.reserve(points.size());
            unique.reserve(points.size());
            for (const auto& point : points) {
                if (quantum.tick()) { co_yield 0; }
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                    seen.insert(quantizePoint(point, grid)).second) { unique.push_back(point); }
            }
            while (!seen.empty()) {
                if (quantum.tick()) { co_yield 0; }
                seen.erase(seen.begin());
            }
        }
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
            weapon_visual_composition_policy::mixValue(signature, source.geometry->childLocalPointCloudsGame.size());
            mixQuantizedPoint(geometryHash, source.localCenterGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMinGame, kGeometryHashQuantizationScale);
            mixQuantizedPoint(geometryHash, source.localMaxGame, kGeometryHashQuantizationScale);
            summary.boundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);

            summary.pointCount += source.geometry->localPointsGame.size();
            summary.childClusterCount += source.geometry->childLocalPointCloudsGame.size();
            summary.semanticPartMask |= partMask(source.semantic.partKind);
            const bool transientReloadSource = isTransientReloadPart(source.semantic.partKind);
            if (transientReloadSource) {
                ++summary.transientReloadSourceCount;
            } else {
                hasDurableGeometry = true;
                ++summary.durableSourceCount;
                summary.durableChildClusterCount += source.geometry->childLocalPointCloudsGame.size();
                summary.durablePointCount += source.geometry->localPointsGame.size();
                summary.durableBoundsExtentScore += extentScoreForBounds(source.localMinGame, source.localMaxGame);
                weapon_visual_composition_policy::mixString(durableGeometryHash, source.sourceName);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.driveRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, reinterpret_cast<std::uintptr_t>(source.sourceRoot));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, source.sourceGroupId);
                weapon_visual_composition_policy::mixValue(durableGeometryHash, static_cast<std::uint32_t>(source.semantic.partKind));
                weapon_visual_composition_policy::mixValue(durableGeometryHash, source.geometry->childLocalPointCloudsGame.size());
                mixQuantizedPoint(durableGeometryHash, source.localCenterGame, kGeometryHashQuantizationScale);
                mixQuantizedPoint(durableGeometryHash, source.localMinGame, kGeometryHashQuantizationScale);
                mixQuantizedPoint(durableGeometryHash, source.localMaxGame, kGeometryHashQuantizationScale);
            }
            if (source.semantic.gameplayCritical &&
                (partMask(source.semantic.partKind) & permanentGameplayCriticalPartMask()) != 0) {
                ++summary.gameplayCriticalCount;
            }

            for (std::size_t i = 0; i < source.geometry->localPointsGame.size(); i += kGeometryPointSampleStride) {
                mixQuantizedPoint(geometryHash, source.geometry->localPointsGame[i], kGeometryHashQuantizationScale);
                if (!transientReloadSource) {
                    mixQuantizedPoint(durableGeometryHash, source.geometry->localPointsGame[i], kGeometryHashQuantizationScale);
                }
            }
            if (!source.geometry->localPointsGame.empty()) {
                mixQuantizedPoint(geometryHash, source.geometry->localPointsGame.back(), kGeometryHashQuantizationScale);
                if (!transientReloadSource) {
                    mixQuantizedPoint(durableGeometryHash, source.geometry->localPointsGame.back(), kGeometryHashQuantizationScale);
                }
            }
            for (const auto& child : source.geometry->childLocalPointCloudsGame) {
                weapon_visual_composition_policy::mixValue(geometryHash, child.size());
                const auto childBounds = pointCloudBounds(child);
                mixQuantizedPoint(geometryHash, childBounds.min, kGeometryHashQuantizationScale);
                mixQuantizedPoint(geometryHash, childBounds.max, kGeometryHashQuantizationScale);
                summary.boundsExtentScore += extentScoreForBounds(childBounds.min, childBounds.max);
                if (!transientReloadSource) {
                    weapon_visual_composition_policy::mixValue(durableGeometryHash, child.size());
                    mixQuantizedPoint(durableGeometryHash, childBounds.min, kGeometryHashQuantizationScale);
                    mixQuantizedPoint(durableGeometryHash, childBounds.max, kGeometryHashQuantizationScale);
                    summary.durableBoundsExtentScore += extentScoreForBounds(childBounds.min, childBounds.max);
                }
                for (std::size_t i = 0; i < child.size(); i += kGeometryPointSampleStride) {
                    mixQuantizedPoint(geometryHash, child[i], kGeometryHashQuantizationScale);
                    if (!transientReloadSource) {
                        mixQuantizedPoint(durableGeometryHash, child[i], kGeometryHashQuantizationScale);
                    }
                }
                if (!child.empty()) {
                    mixQuantizedPoint(geometryHash, child.back(), kGeometryHashQuantizationScale);
                    if (!transientReloadSource) {
                        mixQuantizedPoint(durableGeometryHash, child.back(), kGeometryHashQuantizationScale);
                    }
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
        _sources.cachedCompleteness = {};
    }

    void WeaponCollision::clearPendingWeaponVisualRebuild()
    {
        _sources.pendingVisualRebuildKey = 0;
        _sources.pendingVisualWitnessKey = 0;
        _sources.pendingVisualVisibleTriShapeCount = 0;
        _sources.pendingVisualStableSeconds = 0.0f;
    }

    void WeaponCollision::clearGeneratedSourceCache()
    {
        _sources.preparation.reset();
        _sources.cache = {};
    }

    bool WeaponCollision::advanceSourcePreparation()
    {
        auto& work = *_sources.preparation;
        const auto start = std::chrono::steady_clock::now();
        constexpr double kPreparationSliceMilliseconds = 2.0;
        constexpr std::size_t kMaxResumesPerFrame = 1024;
        try {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponGapColliderBuild);
            for (std::size_t resumed = 0; resumed < kMaxResumesPerFrame; ++resumed) {
                if (!work.task.step()) {
                    work.ready = true;
                    work.task = {};
                    break;
                }
                if (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() >= kPreparationSliceMilliseconds) {
                    break;
                }
            }
        } catch (const std::exception& error) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation failed: key={:016X} reason={}", work.equippedKey, error.what());
            _sources.preparation.reset();
            return false;
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation failed: key={:016X} unknown exception", work.equippedKey);
            _sources.preparation.reset();
            return false;
        }
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        ++work.frames;
        work.activeMilliseconds += elapsed;
        work.maxSliceMilliseconds = (std::max)(work.maxSliceMilliseconds, elapsed);
        if (elapsed > kPreparationSliceMilliseconds * 2.0) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation long slice: key={:016X} elapsedMs={:.3f} targetMs={:.1f}",
                work.equippedKey, elapsed, kPreparationSliceMilliseconds);
        }
        if (work.ready) {
            ROCK_LOG_INFO(Weapon, "Weapon geometry preparation completed: key={:016X} sources={} frames={} activeMs={:.3f} maxSliceMs={:.3f}",
                work.equippedKey, work.sources.size(), work.frames, work.activeMilliseconds, work.maxSliceMilliseconds);
        }
        return work.ready;
    }

    void WeaponCollision::resetVisualSourceUnavailableRetention()
    {
        _sources.visualUnavailableRetainIdentityKey = 0;
        _sources.visualUnavailableRetainRoot = 0;
        _sources.visualUnavailableRetainSeconds = 0.0f;
    }

    bool WeaponCollision::canRetainCurrentWeaponBodiesForVisualSourceMiss(
        std::uint64_t observedIdentityKey,
        RE::NiAVObject* currentWeaponRoot,
        float retainSecondsLimit,
        float measuredDeltaSeconds)
    {
        if (observedIdentityKey == 0 || !currentWeaponRoot) {
            resetVisualSourceUnavailableRetention();
            return false;
        }

        retainSecondsLimit = (std::max)(0.011f, retainSecondsLimit);
        const auto currentRoot = reinterpret_cast<std::uintptr_t>(currentWeaponRoot);
        if (_sources.visualUnavailableRetainIdentityKey != observedIdentityKey ||
            _sources.visualUnavailableRetainRoot != currentRoot) {
            _sources.visualUnavailableRetainIdentityKey = observedIdentityKey;
            _sources.visualUnavailableRetainRoot = currentRoot;
            _sources.visualUnavailableRetainSeconds = 0.0f;
        }

        if (_sources.visualUnavailableRetainSeconds >= retainSecondsLimit) {
            return false;
        }

        // Measured elapsed retention only: an unmeasurable frame holds the
        // window instead of advancing it.
        _sources.visualUnavailableRetainSeconds +=
            std::isfinite(measuredDeltaSeconds) && measuredDeltaSeconds > 0.0f ? measuredDeltaSeconds : 0.0f;
        return true;
    }

    bool WeaponCollision::generatedSourceCacheMatches(
        std::uint64_t equippedKey,
        std::uint64_t ownershipKey,
        std::uint64_t visualKey,
        const RE::NiAVObject* weaponRoot) const
    {
        return _sources.cache.valid &&
               _sources.cache.equippedKey == equippedKey &&
               _sources.cache.ownershipKey == ownershipKey &&
               _sources.cache.visualKey == visualKey &&
               _sources.cache.weaponRootAddress == reinterpret_cast<std::uintptr_t>(weaponRoot) &&
               _sources.cache.sources && !_sources.cache.sources->empty() &&
               _sources.cache.summary.signature != 0;
    }

    void WeaponCollision::storeGeneratedSourceCache(std::uint64_t equippedKey,
        std::uint64_t ownershipKey,
        std::uint64_t visualKey,
        const RE::NiAVObject* weaponRoot,
        std::shared_ptr<const std::vector<GeneratedHullSource>> sources,
        const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary)
    {
        if (equippedKey == 0 || ownershipKey == 0 || visualKey == 0 || !weaponRoot ||
            !sources || sources->empty() || summary.signature == 0) {
            clearGeneratedSourceCache();
            return;
        }

        _sources.cache.valid = true;
        _sources.cache.equippedKey = equippedKey;
        _sources.cache.ownershipKey = ownershipKey;
        _sources.cache.visualKey = visualKey;
        _sources.cache.weaponRootAddress = reinterpret_cast<std::uintptr_t>(weaponRoot);
        _sources.cache.sources = std::move(sources);
        _sources.cache.summary = summary;
    }

    void WeaponCollision::clearPendingGeneratedWeaponBuild(RE::hknpWorld* world, bool destroyTargetBank)
    {
        _sources.preparation.reset();
        auto structuralMutation = destroyTargetBank && _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_sources.pendingBuild.active && destroyTargetBank) {
            destroyWeaponBodyBank(_sources.pendingBuild.replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies(), true);
        }
        _sources.pendingBuild = {};
        (void)world;
    }

    bool WeaponCollision::beginPendingGeneratedWeaponBuild(std::uint64_t equippedKey,
        std::uint64_t visualKey,
        std::uint64_t identityKey,
        std::uint64_t ownershipKey,
        const RE::NiAVObject* weaponRoot,
        std::uint32_t weaponFormID,
        const WeaponVisualKeyStats& visualKeyStats,
        bool replacingExisting,
        bool driveRequestedRebuild,
        std::shared_ptr<const std::vector<GeneratedHullSource>> sources,
        const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary)
    {
        if (equippedKey == 0 || ownershipKey == 0 || !weaponRoot || weaponFormID == 0 ||
            !sources || sources->empty() || summary.signature == 0) {
            return false;
        }

        _sources.pendingBuild = {};
        _sources.pendingBuild.active = true;
        _sources.pendingBuild.replacingExisting = replacingExisting;
        _sources.pendingBuild.driveRequestedRebuild = driveRequestedRebuild;
        _sources.pendingBuild.equippedKey = equippedKey;
        _sources.pendingBuild.visualKey = visualKey;
        _sources.pendingBuild.identityKey = identityKey;
        _sources.pendingBuild.ownershipKey = ownershipKey;
        _sources.pendingBuild.weaponRootAddress = reinterpret_cast<std::uintptr_t>(weaponRoot);
        _sources.pendingBuild.weaponFormID = weaponFormID;
        _sources.pendingBuild.visualRootCount = visualKeyStats.rootCount;
        _sources.pendingBuild.visibleTriShapeCount = visualKeyStats.visibleTriShapeCount;
        _sources.pendingBuild.sources = std::move(sources);
        _sources.pendingBuild.summary = summary;
        return true;
    }

    bool WeaponCollision::pendingGeneratedWeaponBuildMatches(
        std::uint64_t equippedKey,
        std::uint64_t ownershipKey,
        const RE::NiAVObject* weaponRoot,
        std::uint32_t weaponFormID) const
    {
        return _sources.pendingBuild.active &&
               _sources.pendingBuild.equippedKey == equippedKey &&
               _sources.pendingBuild.ownershipKey == ownershipKey &&
               _sources.pendingBuild.weaponRootAddress == reinterpret_cast<std::uintptr_t>(weaponRoot) &&
               _sources.pendingBuild.weaponFormID == weaponFormID;
    }

    bool WeaponCollision::advancePendingGeneratedWeaponBuild(RE::hknpWorld* world)
    {
        if (!_sources.pendingBuild.active) {
            return false;
        }
        if (!world || !_cachedBhkWorld) {
            clearPendingGeneratedWeaponBuild(world, true);
            return false;
        }

        auto& pending = _sources.pendingBuild;
        auto& targetBank = pending.replacingExisting ? inactiveWeaponBodies() : activeWeaponBodies();
        {
            performance_profiler::ScopedTimer profilerTimer(_sources.preserveGaps ?
                performance_profiler::Scope::WeaponGapColliderCreate : performance_profiler::Scope::WeaponColliderCreate);
            pending.createdCount += createGeneratedWeaponBodiesInBankSlice(
                world,
                *pending.sources,
                targetBank,
                GeneratedWeaponBodyCreateOptions{ .collisionEnabledOnCreate = false },
                pending.nextSourceIndex,
                GENERATED_WEAPON_BODY_CREATION_BATCH);
        }

        if (pending.nextSourceIndex < pending.sources->size()) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon collision staged create pending key={:016X} created={} nextSource={}/{} batch={}",
                pending.equippedKey,
                pending.createdCount,
                pending.nextSourceIndex,
                pending.sources->size(),
                GENERATED_WEAPON_BODY_CREATION_BATCH);
            return false;
        }

        if (pending.createdCount == 0) {
            ROCK_LOG_WARN(Weapon,
                "Generated weapon staged creation failed - no bodies created key={:016X} sources={}",
                pending.equippedKey,
                pending.sources->size());
            const bool replacingExisting = pending.replacingExisting;
            clearPendingGeneratedWeaponBuild(world, true);
            if (!replacingExisting) {
                _identity.cachedWeaponKey = 0;
                _identity.cachedVisualKey = 0;
                _identity.cachedIdentityKey = 0;
                _identity.cachedOwnershipKey = 0;
                _identity.cachedFormID = 0;
                clearGeneratedSourceCompletenessTracking();
                clearPendingWeaponVisualRebuild();
                clearAtomicBodyIds();
                resetWeaponBodySetGeneration();
            }
            return false;
        }

        const auto equippedKey = pending.equippedKey;
        const auto sourceCount = pending.sources->size();
        const auto createdCount = pending.createdCount;
        const auto visualRootCount = pending.visualRootCount;
        const auto visibleTriShapeCount = pending.visibleTriShapeCount;
        const bool replacingExisting = pending.replacingExisting;
        const bool driveRequestedRebuild = pending.driveRequestedRebuild;
        const auto summary = pending.summary;
        const auto ownershipKey = pending.ownershipKey;
        const auto weaponFormID = pending.weaponFormID;

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        if (replacingExisting) {
            ROCK_LOG_INFO(Weapon,
                "Replacing generated weapon collision bodies cachedKey={:016X} observedKey={:016X} sources={} replacementBodies={} driveRebuild={} staged=yes",
                _identity.cachedWeaponKey,
                equippedKey,
                sourceCount,
                createdCount,
                driveRequestedRebuild);
            clearAtomicBodyIds();
            destroyWeaponBodyBank(activeWeaponBodies(), true);
            _bodies.usingReplacementBank = !_bodies.usingReplacementBank;
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

        _identity.cachedWeaponKey = equippedKey;
        _identity.cachedVisualKey = pending.visualKey;
        _identity.cachedIdentityKey = pending.identityKey;
        _identity.cachedOwnershipKey = ownershipKey;
        _identity.cachedFormID = weaponFormID;
        _sources.cachedCompleteness = summary;
        _sources.activePreserveGaps = _sources.preserveGaps;
        clearPendingWeaponVisualRebuild();
        publishWeaponBodySetGeneration(summary);
        publishAtomicBodyIds(activeWeaponBodies());
        setWeaponBodyBankCollisionEnabled(world, activeWeaponBodies(), true);
        _drive.rebuildRequested.store(false, std::memory_order_release);
        _drive.failureCount.store(0, std::memory_order_release);
        performance_profiler::addCounter(performance_profiler::Counter::WeaponRebuildCompleted);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildVisibleTriShapes, visibleTriShapeCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildGeneratedSources, sourceCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildBodiesCreated, createdCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildTransientReloadSources, summary.transientReloadSourceCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildBodyCount, finalBodyCount);
        std::size_t convexCount = 0, pointCount = 0;
        for (const auto& source : *pending.sources) {
            convexCount += source.geometry->childLocalPointCloudsGame.empty() ? 1 : source.geometry->childLocalPointCloudsGame.size();
            pointCount += source.geometry->localPointsGame.size();
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildGapMode, _sources.preserveGaps ? 1 : 0);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildConvexes, convexCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponBuildPoints, pointCount);
        ROCK_LOG_INFO(Weapon, "Weapon collider generation completed: preserveGaps={} bodies={} requestedConvexes={} points={}",
            _sources.preserveGaps, finalBodyCount, convexCount, pointCount);
        _sources.pendingBuild = {};
        return true;
    }

    void WeaponCollision::resetWeaponBodySetGeneration()
    {
        _identity.cachedBodySetKey = 0;
        _published.setKey.store(0, std::memory_order_release);
    }

    void WeaponCollision::publishWeaponBodySetGeneration(const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& sourceCompleteness)
    {
        if (_identity.bodySetEpoch == (std::numeric_limits<std::uint64_t>::max)()) {
            _identity.bodySetEpoch = 1;
        } else {
            ++_identity.bodySetEpoch;
        }
        _identity.cachedBodySetKey = weapon_generated_source_completeness_policy::makeGeneratedWeaponBodySetKey(
            _identity.cachedWeaponKey,
            sourceCompleteness,
            _identity.bodySetEpoch);
    }


    void WeaponCollision::bindPhysicalSource(RE::TESObjectREFR* reference, RE::EquippedWeaponData* data)
    {
        if (_physicalReference.get() == reference && _physicalData.get() == data) return;
        _physicalReference.reset(reference);
        _physicalData.reset(data);
        _identity.classificationValid = false;
    }

    WeaponCollisionSource WeaponCollision::weaponSource(bool includeMods) const
    {
        if (_physicalReference) {
            auto* list = _physicalReference->extraList.get();
            const auto* instance = list ? list->GetByType<RE::ExtraInstanceData>() : nullptr;
            return {_physicalReference->GetObjectReference(), instance ? instance->data.get() : nullptr,
                _physicalData.get(), includeMods && list ? list->GetByType<RE::BGSObjectInstanceExtra>() : nullptr};
        }
        const auto* item = f4vr::getEquippedWeaponItem();
        if (!item) return {};
        return {item->item.object, item->item.instanceData.get(),
            item->data ? static_cast<RE::EquippedWeaponData*>(item->data.get()) : nullptr,
            includeMods ? findEquippedWeaponObjectInstanceExtra(f4vr::getPlayer(), item->item.object, item->item.instanceData.get()) : nullptr};
    }

    std::uint64_t WeaponCollision::getEquippedWeaponIdentityKey(
        std::uint64_t* outIdentityKey,
        std::uint64_t* outOwnershipKey,
        WeaponSizeClass* outSizeClass,
        std::uint32_t* outFormID,
        std::uint64_t* outInstanceContentKey) const
    {
        const auto identity = getEquippedWeaponClassification();
        const auto identityKey = _physicalReference ?
            weapon_generation_identity_policy::makePhysicalWeaponIdentityKey(identity, _physicalReference->GetHandle().native_handle()) :
            weapon_generation_identity_policy::makeEquippedWeaponIdentityKey(identity);
        if (outIdentityKey) {
            *outIdentityKey = identityKey;
        }
        if (outOwnershipKey) {
            *outOwnershipKey = weapon_generation_identity_policy::makeEquippedWeaponOwnershipKey(identity);
        }
        if (outSizeClass) {
            *outSizeClass = identity.sizeClass;
        }
        if (outFormID) {
            *outFormID = identity.formID;
        }
        if (outInstanceContentKey) {
            *outInstanceContentKey = identity.instanceContentKey;
        }

        return identityKey;
    }

    weapon_generation_identity_policy::EquippedWeaponGenerationIdentity WeaponCollision::getEquippedWeaponClassification() const
    {
        const auto frame = runtime_state::currentFrame().frameIndex;
        const auto source = weaponSource(false);
        const auto* data = source.data;
        const auto& cached = _identity.frameClassification;
        if (!_identity.classificationValid || frame == 0 || _identity.classificationFrame != frame ||
            cached.formAddress != reinterpret_cast<std::uintptr_t>(source.form) ||
            cached.instanceDataAddress != reinterpret_cast<std::uintptr_t>(source.instance) ||
            cached.equippedDataAddress != reinterpret_cast<std::uintptr_t>(data) ||
            cached.equippedObjectAddress != reinterpret_cast<std::uintptr_t>(data ? data->fireNode : nullptr) ||
            _drive.workbenchExitRebuildRequested.load(std::memory_order_acquire)) {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponIdentityRead);
            _identity.frameClassification = readEquippedWeaponGenerationIdentity(weaponSource());
            _identity.classificationFrame = frame;
            _identity.classificationValid = true;
        }
        return _identity.frameClassification;
    }

    std::uint64_t WeaponCollision::getWeaponVisualCompositionKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const
    {
        std::uint64_t visualKey = 0;
        if (weaponNode) {
            visualKey = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode, !_physicalReference);
            for (const auto& candidate : candidates) {
                if (!candidate.root) {
                    continue;
                }

                ++stats.rootCount;
                mixWeaponVisualString(visualKey, candidate.label);
                mixWeaponVisualKey(visualKey, reinterpret_cast<std::uintptr_t>(candidate.root));
                accumulateWeaponVisualKey(candidate.root, nullptr, 0, 0, visualKey, stats);
            }

            if (visualKey == weapon_visual_composition_policy::kWeaponVisualCompositionOffset) {
                visualKey = reinterpret_cast<std::uint64_t>(weaponNode);
            }
        }
        return visualKey;
    }

    std::size_t WeaponCollision::findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey, std::vector<GeneratedHullSource>& outSources)
    {
        try {
            auto task = prepareGeneratedWeaponShapeSources(RE::NiPointer<RE::NiAVObject>{weaponNode}, equippedWeaponKey, outSources, false);
            while (task.step()) {}
        } catch (const std::exception& error) {
            outSources.clear();
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation failed: key={:016X} reason={}", equippedWeaponKey, error.what());
        } catch (...) {
            outSources.clear();
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Weapon geometry preparation failed: key={:016X} unknown exception", equippedWeaponKey);
        }
        return outSources.size();
    }

    weapon_geometry_work::Task WeaponCollision::prepareGeneratedWeaponShapeSources(
        RE::NiPointer<RE::NiAVObject> root,
        std::uint64_t equippedWeaponKey,
        std::vector<GeneratedHullSource>& outSources, bool preserveGaps)
    {
        auto* weaponNode = root.get();
        outSources.clear();
        if (!weaponNode) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon drive root");
            co_return;
        }

        const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode, !_physicalReference);
        std::vector<RE::NiPointer<RE::NiAVObject>> candidateOwners;
        candidateOwners.reserve(candidates.size());
        for (const auto& candidate : candidates) { candidateOwners.emplace_back(candidate.root); }
        if (candidates.empty()) {
            ROCK_LOG_DEBUG(Weapon, "Generated weapon mesh source scan: no weapon root candidates");
            co_return;
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
        if (_sources.detachedExclusionEquippedKey != equippedWeaponKey) {
            _sources.detachedExclusionEquippedKey = equippedWeaponKey;
            _sources.detachedExclusionGroups.clear();
            _sources.detachedExclusionGroups.reserve(64);
        }
        std::unordered_set<std::uintptr_t> claimedSourceGroups;
        claimedSourceGroups.reserve(256);
        std::size_t acceptedCandidateCount = 0;
        std::uint32_t totalVisitedShapes = 0;
        std::uint32_t totalExtractedTriangles = 0;
        std::uint32_t totalCulledForEffectGeometry = 0;
        for (const auto& candidate : candidates) {
            std::vector<GeneratedHullSource> candidateSources;
            std::unordered_set<std::uintptr_t> candidateExtractedSourceGroups;
            candidateExtractedSourceGroups.reserve(64);
            std::uint32_t visitedShapes = 0;
            std::uint32_t extractedTriangles = 0;
            std::uint32_t culledForEffectGeometry = 0;
            auto candidateTask = findGeneratedWeaponShapeSourcesRecursive(
                RE::NiPointer<RE::NiAVObject>{candidate.root},
                root,
                packageDriveRootTransform,
                0,
                candidateSources,
                visitedShapes,
                extractedTriangles,
                claimedSourceGroups,
                candidateExtractedSourceGroups,
                culledForEffectGeometry, preserveGaps);
            while (candidateTask.step()) { co_yield 0; }
            totalCulledForEffectGeometry += culledForEffectGeometry;

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh candidate: label='{}' root='{}' addr={:x} packageRoot='{}' grouping={} acceptedShapes={} visitedShapes={} triangles={} hulls={} effectShapesCulled={}",
                candidate.label,
                safeNodeName(candidate.root),
                reinterpret_cast<std::uintptr_t>(candidate.root),
                safeNodeName(packageDriveRoot),
                weapon_collision_grouping_policy::kProductionWeaponCollisionGroupingName,
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
            co_return;
        }

        /*
         * Refine physical module kinds from the same installed-OMOD and live
         * emitter evidence that ROCK publishes through the provider API.
         * Exact OMOD identity is authoritative for standard slots. Mod-added
         * P-* slots have no vanilla attach-point FormID, so they use only the
         * bounded owner subtree captured during this same traversal; candidate
         * root fallbacks are explicitly rejected by ownerRootStructural.
         */
        const auto omodByAttachPointFormId = readEquippedOmodsByAttachPointFormId(weaponSource());
        std::unordered_set<std::uint32_t> nativeScopeOverlayOmods;
        nativeScopeOverlayOmods.reserve(omodByAttachPointFormId.size());
        for (const auto& [attachPointFormId, omodFormId] : omodByAttachPointFormId) {
            (void)attachPointFormId;
            if (attachmentModHasScopeFlagProperty(omodFormId)) {
                nativeScopeOverlayOmods.insert(omodFormId);
            }
        }

        const auto emitterSnapshot = collectWeaponEmitterSnapshot(
            weaponNode,
            omodByAttachPointFormId,
            equippedWeaponKey,
            equippedWeaponKey,
            makeWeaponEmitterRootSetKey(weaponNode, !_physicalReference), !_physicalReference);
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
            return source.sourceGroupId != 0 && _sources.detachedExclusionGroups.contains(source.sourceGroupId);
        });
        if (cachedDetachedSourceCount != 0) {
            ROCK_LOG_SAMPLE_INFO(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon detached component cache: excluded {} source(s) for equippedKey={:016X} cachedGroups={}",
                cachedDetachedSourceCount,
                equippedWeaponKey,
                _sources.detachedExclusionGroups.size());
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
                        _sources.detachedExclusionGroups.size() < MAX_CACHED_DETACHED_SOURCE_GROUPS) {
                        newlyCachedGroups += _sources.detachedExclusionGroups.insert(sourceGroupId).second ? 1u : 0u;
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
                    _sources.detachedExclusionGroups.size(),
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
            co_return;
        }

        auto generatedSourceConvexCount = [](const GeneratedHullSource& source) {
            return source.geometry->childLocalPointCloudsGame.empty() ? std::size_t{ 1 } : source.geometry->childLocalPointCloudsGame.size();
        };

        auto generatedSourceSemanticMask = [](const std::vector<GeneratedHullSource>& sources) {
            std::uint32_t mask = 0;
            for (const auto& source : sources) {
                mask |= weapon_generated_source_completeness_policy::partMask(source.semantic.partKind);
            }
            return mask;
        };

        auto totalGeneratedConvexCount = [&](const std::vector<GeneratedHullSource>& sources) {
            std::size_t count = 0;
            for (const auto& source : sources) {
                count += generatedSourceConvexCount(source);
            }
            return count;
        };

        auto logGeneratedSourceInventory = [&](const char* reason, const std::vector<GeneratedHullSource>& sources) {
            const auto semanticMask = generatedSourceSemanticMask(sources);
            const auto convexCount = totalGeneratedConvexCount(sources);
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated weapon mesh source inventory: reason={} label='mergedCandidates' root='{}' candidates={} acceptedCandidates={} sources={} convexes={} maxConvexes={} semanticMask=0x{:08X} parts='{}'",
                reason,
                safeNodeName(packageDriveRoot),
                candidates.size(),
                acceptedCandidateCount,
                sources.size(),
                convexCount,
                MAX_WEAPON_BODIES,
                semanticMask,
                generatedWeaponSemanticMaskNames(semanticMask));

            for (std::size_t i = 0; i < sources.size(); ++i) {
                const auto& source = sources[i];
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated weapon mesh source inventory[{}]: source='{}' driveRoot='{}' sourceRoot='{}' part={} partKind={} points={} children={} convexes={} group={:x} boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f})",
                    i,
                    source.sourceName,
                    safeNodeName(source.driveRoot),
                    safeNodeName(source.sourceRoot),
                    generatedWeaponPartKindName(source.semantic.partKind),
                    static_cast<int>(source.semantic.partKind),
                    source.geometry->localPointsGame.size(),
                    source.geometry->childLocalPointCloudsGame.size(),
                    generatedSourceConvexCount(source),
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
                    source.geometry->localPointsGame.size(),
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

        std::unordered_map<const GeneratedWeaponMeshGeometry*, std::shared_ptr<const GeneratedWeaponMeshIndices>> retainedIndices;
        retainedIndices.reserve(outSources.size());
        for (auto& source : outSources) {
            const auto* mesh = source.geometry->mesh.get();
            auto [it, inserted] = retainedIndices.try_emplace(mesh);
            if (inserted) {
                auto indices = std::make_shared<GeneratedWeaponMeshIndices>();
                indices->localIndex.build(mesh->localTrianglesGame);
                co_yield 0;
                indices->sourceIndex.build(mesh->sourceLocalTrianglesGame);
                co_yield 0;
                it->second = std::move(indices);
            }
            source.indices = it->second;
        }

        assignCollisionSoundMaterials(weaponSource(), packageDriveRoot, outSources);

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
                "Generated weapon mesh selected[{}]: category={} source='{}' driveRoot='{}' sourceRoot='{}' points={} soundMaterial=0x{:08X} center=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f}) sourceLocalCenter=({:.2f},{:.2f},{:.2f}) sourceLocalBoundsMin=({:.2f},{:.2f},{:.2f}) sourceLocalBoundsMax=({:.2f},{:.2f},{:.2f}) sourceNodeScale={:.4f} weaponRootScale={:.4f}",
                i,
                coverage.label,
                source.sourceName,
                safeNodeName(source.driveRoot),
                safeNodeName(source.sourceRoot),
                source.geometry->localPointsGame.size(),
                source.collisionSoundMaterialId,
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

        co_return;
    }

    weapon_geometry_work::Task WeaponCollision::findGeneratedWeaponShapeSourcesRecursive(RE::NiPointer<RE::NiAVObject> nodeOwner,
        RE::NiPointer<RE::NiAVObject> sourceOwner,
        RE::NiTransform weaponRootTransform,
        int depth,
        std::vector<GeneratedHullSource>& outSources,
        std::uint32_t& visitedShapes,
        std::uint32_t& extractedTriangles,
        const std::unordered_set<std::uintptr_t>& claimedSourceGroups,
        std::unordered_set<std::uintptr_t>& candidateExtractedSourceGroups,
        std::uint32_t& culledForEffectGeometry, bool preserveGaps)
    {
        auto* node = nodeOwner.get();
        auto* sourceRoot = sourceOwner.get();
        weapon_geometry_work::Quantum quantum;
        if (!node || depth > 15) {
            co_return;
        }
        if (node->GetAppCulled()) {
            ROCK_LOG_TRACE(Weapon,
                "{}generated mesh source branch skipped '{}': ancestor branch is app-culled",
                std::string(depth * 2, ' '),
                safeNodeName(node));
            co_return;
        }
        auto* triShape = node->IsTriShape();
        if (triShape) {
            const auto sourceGroupId = reinterpret_cast<std::uintptr_t>(triShape);
            if (claimedSourceGroups.find(sourceGroupId) != claimedSourceGroups.end()) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': duplicate TriShape already claimed by earlier candidate", std::string(depth * 2, ' '), safeNodeName(node));
                co_return;
            }
            if (!weaponVisualNodeVisible(node)) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': TriShape is hidden or locally zero-scale", std::string(depth * 2, ' '), safeNodeName(node));
                co_return;
            }

            const auto effectExclusionReason = classifyGeneratedWeaponEffectGeometry(triShape);
            if (effectExclusionReason != weapon_effect_geometry_policy::ExclusionReason::None) {
                ++culledForEffectGeometry;
                ROCK_LOG_TRACE(Weapon,
                    "{}generated mesh source skipped '{}': visual effect geometry reason={}",
                    std::string(depth * 2, ' '),
                    safeNodeName(node),
                    weapon_effect_geometry_policy::exclusionReasonName(effectExclusionReason));
                co_return;
            }
            ++visitedShapes;

            std::vector<TriangleData> triangles;
            std::vector<TriangleData> directSourceLocalTriangles;
            co_yield 0;
            // Snapshot every transform with the extraction. A later frame may
            // move this source or the player while its value-only work resumes.
            const bool skinned = isSkinned(triShape);
            weaponRootTransform = sourceRoot->world;
            const RE::NiTransform capturedSourceWorld = node->world;
            RE::NiTransform sourceInWeapon{};
            const bool sourceInWeaponAvailable =
                tryResolveDescendantLocalTransform(sourceRoot, node, sourceInWeapon);
            RE::NiTransform sourceWorldForDrive = capturedSourceWorld;
            if (sourceInWeaponAvailable) {
                sourceWorldForDrive = transform_math::composeTransforms(
                    weaponRootTransform,
                    sourceInWeapon);
            }
            int added = 0;
            {
                performance_profiler::ScopedTimer extractionTimer(performance_profiler::Scope::WeaponMeshExtraction);
                added = skinned ?
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
            }
            co_yield 0;
            if (added <= 0) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': no extractable triangles", std::string(depth * 2, ' '), safeNodeName(node));
                co_return;
            }
            extractedTriangles += static_cast<std::uint32_t>(added);
            candidateExtractedSourceGroups.insert(sourceGroupId);

            std::vector<RE::NiPoint3> localPoints;
            localPoints.reserve(triangles.size() * 3);
            std::vector<TriangleData> localTriangles;
            localTriangles.reserve(triangles.size());
            std::vector<TriangleData> sourceLocalTriangles;
            sourceLocalTriangles.reserve(triangles.size());
            const bool hasDirectSourceLocalTriangles =
                directSourceLocalTriangles.size() == triangles.size();
            for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
                if (quantum.tick()) { co_yield 0; }
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
                    sourceLocalTriangle.v0 = weapon_collision_geometry_math::worldPointToLocal(capturedSourceWorld.rotate, capturedSourceWorld.translate, capturedSourceWorld.scale, triangle.v0);
                    sourceLocalTriangle.v1 = weapon_collision_geometry_math::worldPointToLocal(capturedSourceWorld.rotate, capturedSourceWorld.translate, capturedSourceWorld.scale, triangle.v1);
                    sourceLocalTriangle.v2 = weapon_collision_geometry_math::worldPointToLocal(capturedSourceWorld.rotate, capturedSourceWorld.translate, capturedSourceWorld.scale, triangle.v2);
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

            const float dedupGridGame = (std::max)(WEAPON_COLLISION_POINT_DEDUP_GRID_HAVOK * havokToGameScale(), 0.01f);
            std::vector<RE::NiPoint3> uniquePoints;
            auto dedup = dedupeWeaponPointsDeferred(localPoints, dedupGridGame, uniquePoints);
            while (dedup.step()) { co_yield 0; }
            localPoints = std::move(uniquePoints);
            if (!pointCloudCanBuildHull(localPoints)) {
                ROCK_LOG_TRACE(Weapon, "{}generated mesh source skipped '{}': degenerate point cloud points={}", std::string(depth * 2, ' '), safeNodeName(node),
                    localPoints.size());
                co_return;
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
            GeneratedPointCloudClusterSet clusterSet;
            std::vector<std::vector<RE::NiPoint3>> compoundChildren;
            if (preserveGaps) {
                weapon_gap_decomposition::Result partition;
                if (localTriangles.size() <= weapon_gap_decomposition::kMaxInputTriangles) {
                    std::vector<weapon_gap_decomposition::Triangle> mesh;
                    mesh.reserve(localTriangles.size());
                    const auto toPoint = [](const RE::NiPoint3& p) { return weapon_gap_decomposition::Point{ p.x, p.y, p.z }; };
                    for (const auto& t : localTriangles) { if (quantum.tick()) { co_yield 0; } mesh.push_back({ toPoint(t.v0), toPoint(t.v1), toPoint(t.v2) }); }
                    auto decomposition = weapon_gap_decomposition::decomposeDeferred(mesh, 1.0, 0.01, partition);
                    for (;;) {
                        bool pending;
                        {
                            performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponGapDecomposition);
                            pending = decomposition.step();
                        }
                        if (!pending) { break; }
                        co_yield 0;
                    }
                } else {
                    partition.budgetLimited = true;
                }
                std::vector<RE::NiPoint3> combinedPoints;
                bool accepted = !partition.pieces.empty();
                for (const auto& piece : partition.pieces) {
                    std::vector<RE::NiPoint3> points;
                    points.reserve(piece.size());
                    for (const auto& p : piece) { if (quantum.tick()) { co_yield 0; } points.push_back({ static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z) }); }
                    std::vector<RE::NiPoint3> uniquePiece;
                    auto childDedup = dedupeWeaponPointsDeferred(points, dedupGridGame, uniquePiece);
                    while (childDedup.step()) { co_yield 0; }
                    points = std::move(uniquePiece);
                    weapon_collision_geometry_math::ConvexSupportFitResult<RE::NiPoint3> fit;
                    auto fitting = weapon_collision_geometry_math::fitConvexSupportPointCloudDeferred(
                        points, WEAPON_COLLISION_SUPPORT_FIT_TARGET_POINTS, MAX_CONVEX_HULL_POINTS,
                        WEAPON_COLLISION_SUPPORT_FIT_MAX_ERROR_GAME_UNITS, true, fit);
                    for (;;) {
                        bool pending;
                        {
                            performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponPointFitting);
                            pending = fitting.step();
                        }
                        if (!pending) { break; }
                        co_yield 0;
                    }
                    if (!fit.accepted || !pointCloudCanBuildHull(fit.points)) { accepted = false; break; }
                    combinedPoints.insert(combinedPoints.end(), fit.points.begin(), fit.points.end());
                    compoundChildren.push_back(fit.points);
                }
                if (accepted) {
                    clusterSet.clusters.push_back(std::move(combinedPoints));
                    if (compoundChildren.size() == 1) { compoundChildren.clear(); }
                } else {
                    // Bounded decomposition must keep source coverage. If any
                    // child cannot be represented, retain the original source
                    // policy as a whole, never a partial set of children.
                    compoundChildren.clear();
                    auto fallback = splitGeneratedWeaponPointCloudForCollisionDeferred(localPoints, clusterSet);
                    while (fallback.step()) { co_yield 0; }
                }
                ROCK_LOG_DEBUG(Weapon,
                    "Generated weapon gap fit: source='{}' accepted={} islands={} cuts={} children={} queryWork={} budgetLimited={}",
                    safeNodeName(node), accepted, partition.islands, partition.cuts,
                    compoundChildren.empty() ? clusterSet.clusters.size() : compoundChildren.size(), partition.queryWork, partition.budgetLimited);
                if (!accepted || partition.budgetLimited) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "Generated weapon gap fit limited: source='{}' originalPolicy={} budgetLimited={} cuts={} unresolved gaps may remain",
                        safeNodeName(node), !accepted, partition.budgetLimited, partition.cuts);
                }
            } else {
                auto fallback = splitGeneratedWeaponPointCloudForCollisionDeferred(localPoints, clusterSet);
                while (fallback.step()) { co_yield 0; }
            }
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
                    WEAPON_COLLISION_SUPPORT_FIT_TARGET_POINTS,
                    clusterSet.supportFitRepairPoints,
                    clusterSet.supportFitValidationDirections);
            }
            std::shared_ptr<const GeneratedWeaponMeshGeometry> mesh;
            for (std::size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {
                auto cluster = compoundChildren.empty() ?
                    weapon_collision_geometry_math::limitPointCloud(std::move(clusters[clusterIndex]), MAX_CONVEX_HULL_POINTS) :
                    std::move(clusters[clusterIndex]);
                if (!pointCloudCanBuildHull(cluster)) {
                    continue;
                }

                if (!mesh) {
                    auto prepared = std::make_shared<GeneratedWeaponMeshGeometry>();
                    prepared->localTrianglesGame = std::move(localTriangles);
                    prepared->sourceLocalTrianglesGame = std::move(sourceLocalTriangles);
                    mesh = std::move(prepared);
                }
                auto geometry = std::make_shared<GeneratedWeaponHullGeometry>();
                geometry->mesh = mesh;
                GeneratedHullSource source;
                source.localCenterGame = weapon_collision_geometry_math::pointCenter(cluster);
                geometry->sourceLocalPointsGame.reserve(cluster.size());
                for (const auto& point : cluster) {
                    if (quantum.tick()) { co_yield 0; }
                    if (sourceInWeaponAvailable) {
                        geometry->sourceLocalPointsGame.push_back(
                            transform_math::worldPointToLocal(sourceInWeapon, point));
                    } else {
                        const RE::NiPoint3 pointWorld = weapon_collision_geometry_math::localPointToWorld(
                            weaponRootTransform.rotate,
                            weaponRootTransform.translate,
                            weaponRootTransform.scale,
                            point);
                        geometry->sourceLocalPointsGame.push_back(weapon_collision_geometry_math::worldPointToLocal(
                            capturedSourceWorld.rotate,
                            capturedSourceWorld.translate,
                            capturedSourceWorld.scale,
                            pointWorld));
                    }
                }
                source.sourceLocalCenterGame = weapon_collision_geometry_math::pointCenter(geometry->sourceLocalPointsGame);
                // All children move rigidly with this TriShape. Bake them in
                // exactly the same source-local frame/scale as the simple hull.
                if (!compoundChildren.empty()) {
                    std::size_t offset = 0;
                    for (const auto& child : compoundChildren) {
                        geometry->childLocalPointCloudsGame.emplace_back(
                            geometry->sourceLocalPointsGame.begin() + offset,
                            geometry->sourceLocalPointsGame.begin() + offset + child.size());
                        offset += child.size();
                    }
                }
                const auto bounds = pointCloudBounds(cluster);
                const auto sourceBounds = pointCloudBounds(geometry->sourceLocalPointsGame);
                source.localMinGame = bounds.min;
                source.localMaxGame = bounds.max;
                source.sourceLocalMinGame = sourceBounds.min;
                source.sourceLocalMaxGame = sourceBounds.max;
                geometry->localPointsGame = std::move(cluster);
                source.geometry = std::move(geometry);
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
                    source.geometry->localPointsGame.size(), source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
                outSources.push_back(std::move(source));
            }
            co_return;
        }

        auto* niNode = node->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            std::vector<RE::NiPointer<RE::NiAVObject>> children;
            visitWeaponChildSlots(kids, [&](auto* kid, auto slot) {
                if (slot >= kids.size()) {
                    ROCK_LOG_DEBUG(Weapon, "Generated weapon sparse child: parent='{}' child='{}' slot={} populated={} slots={}",
                        safeNodeName(node), safeNodeName(kid), slot, kids.size(), kids.capacity());
                }
                children.emplace_back(kid);
                return true;
            });
            for (const auto& child : children) {
                auto childTask = findGeneratedWeaponShapeSourcesRecursive(child, sourceOwner, weaponRootTransform,
                    depth + 1, outSources, visitedShapes, extractedTriangles, claimedSourceGroups,
                    candidateExtractedSourceGroups, culledForEffectGeometry, preserveGaps);
                while (childTask.step()) { co_yield 0; }
                co_yield 0;
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

    std::size_t WeaponCollision::createGeneratedWeaponBodiesInBank(RE::hknpWorld* world,
        const std::vector<GeneratedHullSource>& sources,
        WeaponBodyBank& bank,
        const GeneratedWeaponBodyCreateOptions& options)
    {
        std::size_t nextSourceIndex = 0;
        const auto createdCount = createGeneratedWeaponBodiesInBankSlice(world, sources, bank, options, nextSourceIndex, MAX_WEAPON_BODIES);
        if (createdCount > 0) {
            ROCK_LOG_INFO(Weapon, "Generated weapon mesh collision created {}/{} hull bodies", createdCount, sources.size());
        }
        return createdCount;
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
        static_assert(weapon_gap_decomposition::kMaxChildren <= GENERATED_WEAPON_BODY_CREATION_BATCH);
        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(options.collisionEnabledOnCreate);
        auto buildSourceShape = [&](const GeneratedHullSource& source) -> RE::hknpShape* {
            const auto tagCollisionSoundMaterial = [&](RE::hknpShape* shape) {
                if (shape && source.collisionSoundMaterialId != 0) {
                    // FOCollisionListener resolves this exact field for both
                    // simple shapes and compound leaves. The body material
                    // remains the independently registered solver material.
                    shape->userData = static_cast<std::uintptr_t>(
                        source.collisionSoundMaterialId);
                }
                return shape;
            };

            if (source.geometry->childLocalPointCloudsGame.size() <= 1) {
                const bool useSourceLocal = !source.geometry->sourceLocalPointsGame.empty();
                const auto& sourcePoints = useSourceLocal ? source.geometry->sourceLocalPointsGame : source.geometry->localPointsGame;
                const auto& sourceCenter = useSourceLocal ? source.sourceLocalCenterGame : source.localCenterGame;
                const float sourceScale = useSourceLocal ? source.sourceNodeScale : 1.0f;
                auto centeredHavokPoints = makeCenteredHavokPointCloud(sourcePoints, sourceCenter, sourceScale);
                return tagCollisionSoundMaterial(
                    havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(
                        centeredHavokPoints,
                        WEAPON_COLLISION_CONVEX_RADIUS_HAVOK,
                        havok_convex_shape_builder::ConvexFit::PreserveSharpFeatures));
            }

            std::vector<std::unique_ptr<RE::hknpShape, havok_compound_shape_builder::HavokShapeRelease>> childShapes;
            std::vector<havok_compound_shape_builder::CompoundChild> children;
            childShapes.reserve(source.geometry->childLocalPointCloudsGame.size());
            children.reserve(source.geometry->childLocalPointCloudsGame.size());

            for (const auto& childLocalPointsGame : source.geometry->childLocalPointCloudsGame) {
                if (!pointCloudCanBuildHull(childLocalPointsGame, source.sourceNodeScale)) {
                    return nullptr;
                }

                const auto childCenterGame = weapon_collision_geometry_math::pointCenter(childLocalPointsGame);
                auto centeredChildHavokPoints = makeCenteredHavokPointCloud(childLocalPointsGame, childCenterGame, source.sourceNodeScale);
                auto* childShape =
                    havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(centeredChildHavokPoints, WEAPON_COLLISION_CONVEX_RADIUS_HAVOK,
                        havok_convex_shape_builder::ConvexFit::PreserveSharpFeatures);
                if (!childShape) {
                    ROCK_LOG_WARN(Weapon, "Generated weapon compound source '{}' failed child convex build", source.sourceName);
                    return nullptr;
                }

                tagCollisionSoundMaterial(childShape);

                childShapes.emplace_back(childShape);

                havok_compound_shape_builder::CompoundChild child{};
                child.shape = childShape;
                child.transform.translation.x = (childCenterGame.x - source.sourceLocalCenterGame.x) * source.sourceNodeScale * gameToHavokScale();
                child.transform.translation.y = (childCenterGame.y - source.sourceLocalCenterGame.y) * source.sourceNodeScale * gameToHavokScale();
                child.transform.translation.z = (childCenterGame.z - source.sourceLocalCenterGame.z) * source.sourceNodeScale * gameToHavokScale();
                child.transform.translation.w = 1.0f;
                children.push_back(child);
            }

            // Native compound construction retains each child. Local ownership
            // releases every temporary reference on success and on any failure.
            return tagCollisionSoundMaterial(havok_compound_shape_builder::buildStaticCompoundShape(children));
        };

        while (nextSourceIndex < sources.size() && createdCount < MAX_WEAPON_BODIES && attemptedThisFrame < maxSourceAttemptsThisFrame) {
            // Count native convex builds, not compound bodies: an eight-child
            // source consumes the complete eight-hull slice, never eight times it.
            const auto cost = (std::max)(std::size_t{ 1 }, sources[nextSourceIndex].geometry->childLocalPointCloudsGame.size());
            if (cost > maxSourceAttemptsThisFrame - attemptedThisFrame) { break; }
            const std::size_t sourceIndex = nextSourceIndex++;
            attemptedThisFrame += cost;
            const auto& source = sources[sourceIndex];
            const bool useSourceLocal = !source.geometry->sourceLocalPointsGame.empty();
            const auto& shapePoints = useSourceLocal ? source.geometry->sourceLocalPointsGame : source.geometry->localPointsGame;
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
            instance.driveNodeRef = RE::NiPointer<RE::NiAVObject>{ instance.driveNode };
            instance.sourceNodeRef = RE::NiPointer<RE::NiAVObject>{ instance.sourceNode };
            instance.sourceName = source.sourceName;
            instance.driveRootName = instance.driveNode ? safeNodeName(instance.driveNode) : "";
            instance.sourceRootName = source.sourceRoot ? safeNodeName(source.sourceRoot) : "";
            instance.generatedLocalCenterGame = source.localCenterGame;
            instance.generatedSourceLocalCenterGame = source.sourceLocalCenterGame;
            instance.generatedLocalMinGame = source.localMinGame;
            instance.generatedLocalMaxGame = source.localMaxGame;
            instance.generatedSourceLocalMinGame = source.sourceLocalMinGame;
            instance.generatedSourceLocalMaxGame = source.sourceLocalMaxGame;
            instance.geometry = source.geometry;
            instance.indices = source.indices;
            instance.generatedPointCount = static_cast<std::uint32_t>(
                (std::min)(source.geometry->localPointsGame.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            instance.generatedSourceGroupId = source.sourceGroupId;
            instance.semantic = source.semantic;
            instance.ownsShapeRef = true;
            clearGeneratedKeyframedBodyDriveState(instance.driveState);

            const bool ok =
                instance.body.create(world, _cachedBhkWorld, shape, filterInfo, { 0 }, BethesdaMotionType::Keyframed, "ROCK_WeaponMeshCollision");

            if (!ok) {
                ROCK_LOG_ERROR(Weapon, "BethesdaPhysicsBody::create failed for generated weapon mesh hull '{}'", source.sourceName);
                havok_ref_count::release(shape);
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
                havok_ref_count::release(shape);
                continue;
            }
            initializeGeneratedKeyframedBodyDriveState(instance.driveState, initialTransform);

            ROCK_LOG_DEBUG(Weapon,
                "Generated weapon mesh collision body created: meshIndex={} bodyId={} source='{}' driveRoot='{}' sourceRoot='{}' partKind={} supportRole={} reloadRole={} points={} children={} soundMaterial=0x{:08X} center=({:.2f},{:.2f},{:.2f}) layer=44",
                createdCount, instance.body.getBodyId().value, source.sourceName, safeNodeName(source.driveRoot), safeNodeName(source.sourceRoot), static_cast<int>(source.semantic.partKind),
                static_cast<int>(source.semantic.supportGripRole), static_cast<int>(source.semantic.reloadRole), source.geometry->localPointsGame.size(),
                source.geometry->childLocalPointCloudsGame.size(), source.collisionSoundMaterialId,
                source.localCenterGame.x, source.localCenterGame.y, source.localCenterGame.z);
            ++createdCount;
            ++createdThisFrame;
        }

        if (createdThisFrame > 0) {
            _drive.rebuildRequested.store(false, std::memory_order_release);
            _drive.failureCount.store(0, std::memory_order_release);
        }
        return createdThisFrame;
    }
}
