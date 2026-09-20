#include "physics-interaction/performance/PerformanceProfiler.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include <F4SE/Logger.h>

namespace rock::performance_profiler
{
    namespace
    {
        struct ScopeAccum
        {
            std::atomic<std::uint64_t> totalTicks{ 0 };
            std::atomic<std::uint64_t> maxTicks{ 0 };
            std::atomic<std::uint64_t> samples{ 0 };
            std::atomic<std::uint64_t> events{ 0 };
            std::array<std::atomic<std::uint64_t>, 3> memoryQueries{};
            std::atomic<std::uint64_t> memoryQueryFailures{ 0 };
            std::atomic<std::uint64_t> memoryQueryTimedSamples{ 0 };
            std::atomic<std::uint64_t> memoryQueryTotalTicks{ 0 };
            std::atomic<std::uint64_t> memoryQueryMaxTicks{ 0 };
        };

        struct CounterAccum
        {
            std::atomic<std::uint64_t> count{ 0 };
        };

        struct ValueAccum
        {
            std::atomic<std::uint64_t> total{ 0 };
            std::atomic<std::uint64_t> max{ 0 };
            std::atomic<std::uint64_t> samples{ 0 };
        };

        struct Settings
        {
            std::atomic<bool> enabled{ false };
            std::atomic<bool> overlayText{ false };
            std::atomic<std::uint32_t> logIntervalFrames{ 300 };
            std::atomic<std::uint32_t> warmupFrames{ 120 };
            std::atomic<std::uint64_t> frameIndex{ 0 };
            std::atomic<std::uint64_t> intervalStartFrame{ 0 };
            std::atomic<std::uint64_t> generation{ 1 };
        };

        std::array<ScopeAccum, static_cast<std::size_t>(Scope::Count)> s_accum;
        std::array<CounterAccum, static_cast<std::size_t>(Counter::Count)> s_counterAccum;
        std::array<ValueAccum, static_cast<std::size_t>(ValueMetric::Count)> s_valueAccum;
        Settings s_settings;
        // All native/game/render threads merge into the existing atomic window.
        // TLS retains only a scope ID and sampling phase, never engine pointers.
        thread_local Scope t_memoryQueryScope = Scope::UnattributedMemoryQueries;
        thread_local std::uint32_t t_memoryQuerySequence = 0;
        LARGE_INTEGER s_frequency{};
        std::atomic<bool> s_frequencyReady{ false };
        std::mutex s_overlayMutex;
        OverlayLines s_overlayLines{};
        std::uint32_t s_overlayLineCount = 0;

        constexpr std::uint32_t sanitizeIntervalFrames(int value) noexcept
        {
            return static_cast<std::uint32_t>(std::clamp(value, 30, 54000));
        }

        constexpr std::uint32_t sanitizeWarmupFrames(int value) noexcept
        {
            return static_cast<std::uint32_t>(std::clamp(value, 0, 54000));
        }

        constexpr const char* scopeName(Scope scope) noexcept
        {
            switch (scope) {
            case Scope::FrameUpdate:
                return "frame";
            case Scope::RuntimePreparation:
                return "runtimePreparation";
            case Scope::WeaponEquipTransition:
                return "weaponEquipTransition";
            case Scope::AuthoredPrimaryGrip:
                return "authoredPrimaryGrip";
            case Scope::InteractionUpdate:
                return "interactionUpdate";
            case Scope::EquippedWeaponInteraction:
                return "equippedWeaponInteraction";
            case Scope::InteractionFinalize:
                return "interactionFinalize";
            case Scope::RenderedHandCapture:
                return "renderedHandCapture";
            case Scope::ProviderPublication:
                return "providerPublication";
            case Scope::HandColliderUpdate:
                return "handColliders";
            case Scope::BodyColliderUpdate:
                return "bodyColliders";
            case Scope::GeneratedColliderPhysicsFlush:
                return "generatedFlush";
            case Scope::WeaponCollision:
                return "weaponCollision";
            case Scope::WeaponCollisionTransforms:
                return "weaponCollisionTransforms";
            case Scope::WeaponContactProbe:
                return "weaponContactProbe";
            case Scope::WeaponEmitterRefresh:
                return "weaponEmitterRefresh";
            case Scope::WeaponIdentityRead:
                return "weaponIdentityRead";
            case Scope::WeaponVisualObservation:
                return "weaponVisualObservation";
            case Scope::GeneratedBodyContactRegistry:
                return "generatedBodyRegistry";
            case Scope::WeaponColliderBuild:
                return "weaponColliderBuild";
            case Scope::WeaponColliderCreate:
                return "weaponColliderCreate";
            case Scope::WeaponGapDecomposition:
                return "weaponGapDecomposition";
            case Scope::WeaponMeshExtraction:
                return "weaponMeshExtraction";
            case Scope::WeaponPointFitting:
                return "weaponPointFitting";
            case Scope::WeaponGapColliderBuild:
                return "weaponGapColliderBuild";
            case Scope::WeaponGapColliderCreate:
                return "weaponGapColliderCreate";
            case Scope::TwoHandedGripStart:
                return "twoHandedGripStart";
            case Scope::EquippedWeaponFingerPoseCapture:
                return "equippedWeaponFingerPoseCapture";
            case Scope::SupportGripSuppression:
                return "supportGripSuppression";
            case Scope::SelectionCasts:
                return "selectionCasts";
            case Scope::DynamicHandCollisionFrame:
                return "dynamicHandFrame";
            case Scope::DynamicHandCollisionPhysicsDrive:
                return "dynamicHandDrive";
            case Scope::DynamicHandCollisionPostSolve:
                return "dynamicHandPostSolve";
            case Scope::DebugOverlayPublish:
                return "debugOverlayPublish";
            case Scope::DebugOverlayRender:
                return "debugOverlayRender";
            case Scope::ContactResolve:
                return "contactResolve";
            case Scope::NativeContactCallback:
                return "nativeContactCallbacks";
            case Scope::NativeMeleeCallback:
                return "nativeMeleeCallback";
            case Scope::NativeMeleeDispatch:
                return "nativeMeleeDispatch";
            case Scope::GrabAcquisitionBodyScan:
                return "grabAcquisitionBodyScan";
            case Scope::GrabAcquisitionActivePrep:
                return "grabAcquisitionActivePrep";
            case Scope::GrabMeshExtraction:
                return "grabMeshExtraction";
            case Scope::GrabNearbyDampingBegin:
                return "grabNearbyDampingBegin";
            case Scope::GrabHeldObjectUpdate:
                return "grabHeldObjectUpdate";
            case Scope::GrabAuthorityFlush:
                return "grabAuthorityFlush";
            case Scope::GrabAuthorityAfterSolveDiagnostics:
                return "grabAuthorityAfterSolveDiagnostics";
            case Scope::GrabNearbyDampingRestore:
                return "grabNearbyDampingRestore";
            case Scope::GrabNearbyDampingRestoreBodySearch:
                return "grabNearbyDampingRestoreBodySearch";
            case Scope::FramePrelude: return "framePrelude";
            case Scope::FrameBeginPreparation: return "frameBeginPreparation";
            case Scope::WeaponPresentation: return "weaponPresentation";
            case Scope::FinalPresentation: return "finalPresentation";
            case Scope::HandFrameResolve: return "handFrameResolve";
            case Scope::HandBoneCapture: return "handBoneCapture";
            case Scope::BodyBoneCapture: return "bodyBoneCapture";
            case Scope::FingerBoneCapture: return "fingerBoneCapture";
            case Scope::SelectionHitProcessing: return "selectionHitProcessing";
            case Scope::PhysicsSystemBodyScan: return "physicsSystemBodyScan";
            case Scope::NativePlayerRefresh: return "nativePlayerRefresh";
            case Scope::NativePlayerPairFilter: return "nativePlayerPairFilter";
            case Scope::HeldSceneWriter: return "heldSceneWriter";
            case Scope::ProviderFrameDispatch: return "providerFrameDispatch";
            case Scope::ProviderFrameConsumer: return "providerFrameConsumer";
            case Scope::ProviderAnimationDispatch: return "providerAnimationDispatch";
            case Scope::ProviderAnimationConsumer: return "providerAnimationConsumer";
            case Scope::NativeWorldReadWait: return "nativeWorldReadWait";
            case Scope::CallbackQuiescenceWait: return "callbackQuiescenceWait";
            case Scope::NearbyDampingWait: return "nearbyDampingWait";
            case Scope::NativeIdleGripHarvest: return "nativeIdleGripHarvest";
            case Scope::UnattributedMemoryQueries: return "unattributedMemoryQueries";
            case Scope::GrabAcquisition: return "grabAcquisition";
            case Scope::GrabSurfaceResolution: return "grabSurfaceResolution";
            case Scope::GrabFingerSolve: return "grabFingerSolve";
            case Scope::GrabFingerIndexBuild: return "grabFingerIndexBuild";
            case Scope::GrabFingerPadProbes: return "grabFingerPadProbes";
            case Scope::NativePhysicsUpdate: return "nativePhysicsUpdate";
            case Scope::NativePhysicsCollideInterval: return "nativePhysicsCollideInterval";
            case Scope::NativePhysicsSolveInterval: return "nativePhysicsSolveInterval";
            case Scope::GrabSelectionValidation: return "grabSelectionValidation";
            case Scope::GrabBodyPreparation: return "grabBodyPreparation";
            case Scope::GrabProxyPreparation: return "grabProxyPreparation";
            case Scope::GrabMeshCapturePreparation: return "grabMeshCapturePreparation";
            case Scope::GrabBodyResolution: return "grabBodyResolution";
            case Scope::GrabResolvedBodyCapture: return "grabResolvedBodyCapture";
            case Scope::GrabPivotEvidence: return "grabPivotEvidence";
            case Scope::GrabContactPatch: return "grabContactPatch";
            case Scope::GrabPinchPocket: return "grabPinchPocket";
            case Scope::GrabFingerEvidence: return "grabFingerEvidence";
            case Scope::GrabCommitPreparation: return "grabCommitPreparation";
            case Scope::GrabBodyFrameCapture: return "grabBodyFrameCapture";
            case Scope::GrabSeatCapture: return "grabSeatCapture";
            case Scope::GrabGripSupport: return "grabGripSupport";
            case Scope::GrabFrozenCommit: return "grabFrozenCommit";
            case Scope::GrabPostFreeze: return "grabPostFreeze";
            case Scope::GrabConstraintCommit: return "grabConstraintCommit";
            case Scope::GrabLocalTriangleCapture: return "grabLocalTriangleCapture";
            case Scope::GrabTriangleSelection: return "grabTriangleSelection";
            case Scope::MeshStaticExtraction: return "meshStaticExtraction";
            case Scope::MeshDynamicExtraction: return "meshDynamicExtraction";
            case Scope::MeshSkinnedExtraction: return "meshSkinnedExtraction";
            case Scope::MeshPointQuery: return "meshPointQuery";
            case Scope::MeshDirectionalQuery: return "meshDirectionalQuery";
            case Scope::Count:
                break;
            }
            return "unknown";
        }

        constexpr const char* counterName(Counter counter) noexcept
        {
            switch (counter) {
            case Counter::WeaponRebuildQueued:
                return "weaponRebuildQueued";
            case Counter::WeaponRebuildCanceled:
                return "weaponRebuildCanceled";
            case Counter::WeaponRebuildCompleted:
                return "weaponRebuildCompleted";
            case Counter::WeaponRebuildVisualRootDeferred:
                return "weaponRebuildVisualRootDeferred";
            case Counter::WeaponRebuildVisualStableWait:
                return "weaponRebuildVisualStableWait";
            case Counter::WeaponRebuildVisualSourceUnavailableRetained:
                return "weaponRebuildVisualSourceUnavailableRetained";
            case Counter::WeaponRebuildVisualSourceUnavailableRetainExpired:
                return "weaponRebuildVisualSourceUnavailableRetainExpired";
            case Counter::WeaponRebuildReasonDriveRequested:
                return "weaponRebuildReasonDriveRequested";
            case Counter::WeaponRebuildReasonKeyChanged:
                return "weaponRebuildReasonKeyChanged";
            case Counter::WeaponRebuildReasonMissingBodies:
                return "weaponRebuildReasonMissingBodies";
            case Counter::WeaponKeyChangeVisualOnly:
                return "weaponKeyChangeVisualOnly";
            case Counter::WeaponKeyChangeIdentityOnly:
                return "weaponKeyChangeIdentityOnly";
            case Counter::WeaponKeyChangeVisualAndIdentity:
                return "weaponKeyChangeVisualAndIdentity";
            case Counter::GrabAcquisitionCachePrewarm:
                return "grabAcquisitionCachePrewarm";
            case Counter::GrabAcquisitionCacheHit:
                return "grabAcquisitionCacheHit";
            case Counter::GrabAcquisitionCacheMiss:
                return "grabAcquisitionCacheMiss";
            case Counter::GrabAcquisitionCacheInvalidated:
                return "grabAcquisitionCacheInvalidated";
            case Counter::GrabNearbyDampingRestoreFailed:
                return "grabNearbyDampingRestoreFailed";
            case Counter::NativeMeleeRockPartnerDropped:
                return "nativeMeleeRockPartnerDropped";
            case Counter::NativeMeleeDecodeFailed:
                return "nativeMeleeDecodeFailed";
            case Counter::NativeReadRangeRejected: return "nativeReadRangeRejected";
            case Counter::NativeWriteRangeRejected: return "nativeWriteRangeRejected";
            case Counter::PhysicsTimingSubstepsIncreased: return "physicsTimingSubstepsIncreased";
            case Counter::GrabAcquisitionPeerHeld: return "grabAcquisitionPeerHeld";
            case Counter::GrabAcquisitionEquippedTransfer: return "grabAcquisitionEquippedTransfer";
            case Counter::GrabAcquisitionSucceeded: return "grabAcquisitionSucceeded";
            case Counter::Count:
                break;
            }
            return "unknown";
        }

        constexpr const char* valueMetricName(ValueMetric metric) noexcept
        {
            switch (metric) {
            case ValueMetric::WeaponBuildVisibleTriShapes:
                return "weaponBuildVisibleTriShapes";
            case ValueMetric::WeaponBuildGeneratedSources:
                return "weaponBuildGeneratedSources";
            case ValueMetric::WeaponBuildBodiesCreated:
                return "weaponBuildBodiesCreated";
            case ValueMetric::WeaponBuildTransientReloadSources:
                return "weaponBuildTransientReloadSources";
            case ValueMetric::WeaponBuildBodyCount:
                return "weaponBuildBodyCount";
            case ValueMetric::WeaponBuildGapMode:
                return "weaponBuildGapMode";
            case ValueMetric::WeaponBuildConvexes:
                return "weaponBuildConvexes";
            case ValueMetric::WeaponBuildPoints:
                return "weaponBuildPoints";
            case ValueMetric::GrabAcquisitionVisitedNodes:
                return "grabAcquisitionVisitedNodes";
            case ValueMetric::GrabAcquisitionCollisionObjects:
                return "grabAcquisitionCollisionObjects";
            case ValueMetric::GrabAcquisitionBodyIds:
                return "grabAcquisitionBodyIds";
            case ValueMetric::GrabMeshTriangles:
                return "grabMeshTriangles";
            case ValueMetric::GrabNearbyDampingMotions:
                return "grabNearbyDampingMotions";
            case ValueMetric::EquippedWeaponFingerPoseSourceTriangles:
                return "equippedWeaponFingerPoseSourceTriangles";
            case ValueMetric::EquippedWeaponFingerPoseSelectedTriangles:
                return "equippedWeaponFingerPoseSelectedTriangles";
            case ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits:
                return "equippedWeaponFingerPoseSpatialNodeVisits";
            case ValueMetric::EquippedWeaponFingerPoseTriangleTests:
                return "equippedWeaponFingerPoseTriangleTests";
            case ValueMetric::NativeMeleeCallbacksPerFrame:
                return "nativeMeleeCallbacksPerFrame";
            case ValueMetric::RenderedSkeletonBones: return "renderedSkeletonBones";
            case ValueMetric::ControllerSkeletonBones: return "controllerSkeletonBones";
            case ValueMetric::SelectionRawHits: return "selectionRawHits";
            case ValueMetric::PhysicsOriginalSubsteps: return "physicsOriginalSubsteps";
            case ValueMetric::PhysicsRequestedSubsteps: return "physicsRequestedSubsteps";
            case ValueMetric::PhysicsCompletedSubsteps: return "physicsCompletedSubsteps";
            case ValueMetric::PhysicsRawDeltaMicroseconds: return "physicsRawDeltaMicroseconds";
            case ValueMetric::GeneratedHandBodies: return "generatedHandBodies";
            case ValueMetric::GeneratedBodyBodies: return "generatedBodyBodies";
            case ValueMetric::GeneratedWeaponBodies: return "generatedWeaponBodies";
            case ValueMetric::FingerPadCandidateTriangles: return "fingerPadCandidateTriangles";
            case ValueMetric::FingerPadTriangleTests: return "fingerPadTriangleTests";
            case ValueMetric::GrabMeshStaticTriangles: return "grabMeshStaticTriangles";
            case ValueMetric::GrabMeshDynamicTriangles: return "grabMeshDynamicTriangles";
            case ValueMetric::GrabMeshSkinnedTriangles: return "grabMeshSkinnedTriangles";
            case ValueMetric::GrabMeshCaptureAttempts: return "grabMeshCaptureAttempts";
            case ValueMetric::GrabMeshPayloadBytes: return "grabMeshPayloadBytes";
            case ValueMetric::MeshPointQueryTriangles: return "meshPointQueryTriangles";
            case ValueMetric::MeshDirectionalQueryTriangles: return "meshDirectionalQueryTriangles";
            case ValueMetric::Count:
                break;
            }
            return "unknown";
        }

        bool ensureFrequencyReady() noexcept
        {
            if (s_frequencyReady.load(std::memory_order_acquire)) {
                return true;
            }

            LARGE_INTEGER frequency{};
            if (!::QueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0) {
                return false;
            }

            s_frequency = frequency;
            s_frequencyReady.store(true, std::memory_order_release);
            return true;
        }

        std::uint64_t queryPerformanceTicks() noexcept
        {
            LARGE_INTEGER counter{};
            if (!::QueryPerformanceCounter(&counter)) {
                return 0;
            }
            return static_cast<std::uint64_t>(counter.QuadPart);
        }

        double ticksToMilliseconds(std::uint64_t ticks) noexcept
        {
            if (!ensureFrequencyReady()) {
                return 0.0;
            }
            return (static_cast<double>(ticks) * 1000.0) / static_cast<double>(s_frequency.QuadPart);
        }

        bool validScope(Scope scope) noexcept
        {
            return static_cast<std::uint8_t>(scope) < static_cast<std::uint8_t>(Scope::Count);
        }

        bool validCounter(Counter counter) noexcept
        {
            return static_cast<std::uint8_t>(counter) < static_cast<std::uint8_t>(Counter::Count);
        }

        bool validValueMetric(ValueMetric metric) noexcept
        {
            return static_cast<std::uint8_t>(metric) < static_cast<std::uint8_t>(ValueMetric::Count);
        }

        ScopeAccum& accumFor(Scope scope) noexcept
        {
            return s_accum[static_cast<std::size_t>(scope)];
        }

        CounterAccum& accumFor(Counter counter) noexcept
        {
            return s_counterAccum[static_cast<std::size_t>(counter)];
        }

        ValueAccum& accumFor(ValueMetric metric) noexcept
        {
            return s_valueAccum[static_cast<std::size_t>(metric)];
        }

        void clearAccumulators() noexcept
        {
            for (auto& slot : s_accum) {
                slot.totalTicks.store(0, std::memory_order_release);
                slot.maxTicks.store(0, std::memory_order_release);
                slot.samples.store(0, std::memory_order_release);
                slot.events.store(0, std::memory_order_release);
                for (auto& count : slot.memoryQueries) count.store(0, std::memory_order_relaxed);
                slot.memoryQueryFailures.store(0, std::memory_order_relaxed);
                slot.memoryQueryTimedSamples.store(0, std::memory_order_relaxed);
                slot.memoryQueryTotalTicks.store(0, std::memory_order_relaxed);
                slot.memoryQueryMaxTicks.store(0, std::memory_order_relaxed);
            }
        }

        void clearCounterAccumulators() noexcept
        {
            for (auto& slot : s_counterAccum) {
                slot.count.store(0, std::memory_order_release);
            }
        }

        void clearValueAccumulators() noexcept
        {
            for (auto& slot : s_valueAccum) {
                slot.total.store(0, std::memory_order_release);
                slot.max.store(0, std::memory_order_release);
                slot.samples.store(0, std::memory_order_release);
            }
        }

        void clearOverlayLines() noexcept
        {
            std::scoped_lock lock(s_overlayMutex);
            s_overlayLines = {};
            s_overlayLineCount = 0;
        }

        void atomicMax(std::atomic<std::uint64_t>& target, std::uint64_t value) noexcept
        {
            auto current = target.load(std::memory_order_relaxed);
            while (value > current && !target.compare_exchange_weak(current, value, std::memory_order_release, std::memory_order_relaxed)) {
            }
        }

        void recordTicks(Scope scope, std::uint64_t ticks) noexcept
        {
            if (!validScope(scope)) {
                return;
            }

            auto& slot = accumFor(scope);
            slot.totalTicks.fetch_add(ticks, std::memory_order_relaxed);
            slot.samples.fetch_add(1, std::memory_order_relaxed);
            atomicMax(slot.maxTicks, ticks);
        }

        struct MemoryQueryTotals
        {
            std::array<std::uint64_t, 3> calls{};
            std::uint64_t apiFailures{ 0 };
            std::uint64_t timedSamples{ 0 };
            std::uint64_t totalTicks{ 0 };
            std::uint64_t maxTicks{ 0 };
        };

        struct ScopeSnapshot
        {
            Scope scope{ Scope::Count };
            std::uint64_t totalTicks{ 0 };
            std::uint64_t maxTicks{ 0 };
            std::uint64_t samples{ 0 };
            std::uint64_t events{ 0 };
            MemoryQueryTotals memory{};

            // Concurrent queries can straddle the individual atomic exchanges
            // at a window boundary. Retain timing/failure-only tails as well.
            [[nodiscard]] bool hasQueries() const noexcept
            {
                return memory.calls[0] || memory.calls[1] || memory.calls[2] || memory.apiFailures || memory.timedSamples;
            }
            [[nodiscard]] bool hasData() const noexcept { return samples > 0 || events > 0 || hasQueries(); }
            [[nodiscard]] double totalMs() const noexcept { return ticksToMilliseconds(totalTicks); }
            [[nodiscard]] double maxMs() const noexcept { return ticksToMilliseconds(maxTicks); }
            [[nodiscard]] double avgMs() const noexcept { return samples > 0 ? totalMs() / static_cast<double>(samples) : 0.0; }
        };

        struct CounterSnapshot
        {
            Counter counter{ Counter::Count };
            std::uint64_t count{ 0 };

            [[nodiscard]] bool hasData() const noexcept { return count > 0; }
        };

        struct ValueSnapshot
        {
            ValueMetric metric{ ValueMetric::Count };
            std::uint64_t total{ 0 };
            std::uint64_t max{ 0 };
            std::uint64_t samples{ 0 };

            [[nodiscard]] bool hasData() const noexcept { return samples > 0; }
            [[nodiscard]] double avg() const noexcept { return samples > 0 ? static_cast<double>(total) / static_cast<double>(samples) : 0.0; }
        };

        struct QueuedSnapshot
        {
            std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)> scopes{};
            std::array<CounterSnapshot, static_cast<std::size_t>(Counter::Count)> counters{};
            std::array<ValueSnapshot, static_cast<std::size_t>(ValueMetric::Count)> values{};
            std::uint64_t frames{ 0 };
            std::uint64_t droppedSnapshotsBeforeThis{ 0 };
        };

        class AsyncProfilerWriter
        {
        public:
            ~AsyncProfilerWriter() noexcept { stop(); }

            bool start() noexcept
            {
                if (_startFailed.load(std::memory_order_acquire)) {
                    return false;
                }

                if (_started.load(std::memory_order_acquire)) {
                    return true;
                }

                std::scoped_lock lock(_threadMutex);
                if (_started.load(std::memory_order_acquire)) {
                    return true;
                }

                _stopRequested.store(false, std::memory_order_release);
                try {
                    _thread = std::thread([this]() noexcept { run(); });
                    _started.store(true, std::memory_order_release);
                    return true;
                } catch (...) {
                    _started.store(false, std::memory_order_release);
                    _startFailed.store(true, std::memory_order_release);
                    _droppedSnapshots.fetch_add(1, std::memory_order_relaxed);
                    return false;
                }
            }

            void enqueue(QueuedSnapshot snapshot) noexcept
            {
                if (!_started.load(std::memory_order_acquire)) {
                    _droppedSnapshots.fetch_add(1, std::memory_order_relaxed);
                    return;
                }

                const auto writeSeq = _writeSeq.load(std::memory_order_relaxed);
                const auto readSeq = _readSeq.load(std::memory_order_acquire);
                if (writeSeq - readSeq >= static_cast<std::uint64_t>(kQueueCapacity)) {
                    _droppedSnapshots.fetch_add(1, std::memory_order_relaxed);
                    return;
                }

                snapshot.droppedSnapshotsBeforeThis = _droppedSnapshots.exchange(0, std::memory_order_acq_rel);
                _queue[static_cast<std::size_t>(writeSeq % kQueueCapacity)] = snapshot;
                _writeSeq.store(writeSeq + 1, std::memory_order_release);
                _wakeCv.notify_one();
            }

            void stop() noexcept
            {
                std::thread threadToJoin;
                {
                    std::scoped_lock lock(_threadMutex);
                    if (!_started.load(std::memory_order_acquire) && !_thread.joinable()) {
                        return;
                    }
                    _stopRequested.store(true, std::memory_order_release);
                    _wakeCv.notify_one();
                    if (_thread.joinable()) {
                        threadToJoin = std::move(_thread);
                    }
                }

                if (threadToJoin.joinable()) {
                    try {
                        threadToJoin.join();
                    } catch (...) {
                    }
                }

                _started.store(false, std::memory_order_release);
                _stopRequested.store(false, std::memory_order_release);
            }

        private:
            static constexpr std::size_t kQueueCapacity = 16;
            static constexpr auto kFlushInterval = std::chrono::seconds(2);

            [[nodiscard]] bool hasPending() const noexcept
            {
                return _readSeq.load(std::memory_order_acquire) != _writeSeq.load(std::memory_order_acquire);
            }

            bool tryDequeue(QueuedSnapshot& outSnapshot) noexcept
            {
                const auto readSeq = _readSeq.load(std::memory_order_relaxed);
                const auto writeSeq = _writeSeq.load(std::memory_order_acquire);
                if (readSeq == writeSeq) {
                    return false;
                }

                outSnapshot = _queue[static_cast<std::size_t>(readSeq % kQueueCapacity)];
                _readSeq.store(readSeq + 1, std::memory_order_release);
                return true;
            }

            std::shared_ptr<spdlog::logger> ensureLogger() noexcept
            {
                if (_logger) {
                    return _logger;
                }

                try {
                    auto path = F4SE::log::log_directory();
                    if (!path.has_value()) {
                        return nullptr;
                    }

                    const auto gamepath = REL::Module::IsVR() ? "Fallout4VR/F4SE" : "Fallout4/F4SE";
                    if (!path.value().generic_string().ends_with(gamepath)) {
                        path = path.value().parent_path().append(gamepath);
                    }

                    *path /= "ROCK_Profiler.log";
                    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_st>(path->string(), 1024 * 1024 * 10, 5, true);
                    _logger = std::make_shared<spdlog::logger>("ROCK_PROFILER_ASYNC", sink);
                    _logger->set_level(spdlog::level::info);
                    _logger->flush_on(spdlog::level::critical);
                    _logger->set_formatter(std::make_unique<spdlog::pattern_formatter>("%Y-%m-%d %H:%M:%S.%e [%l] %v"));
                    return _logger;
                } catch (...) {
                    return nullptr;
                }
            }

            void flushLogger() noexcept
            {
                try {
                    if (_logger) {
                        _logger->flush();
                    }
                } catch (...) {
                }
            }

            void flushIfDue() noexcept
            {
                const auto now = std::chrono::steady_clock::now();
                if (now < _nextFlush) {
                    return;
                }

                flushLogger();
                _nextFlush = now + kFlushInterval;
            }

            void writeSnapshot(const QueuedSnapshot& snapshot) noexcept
            {
                try {
                    const auto logger = ensureLogger();
                    if (!logger) {
                        _droppedSnapshots.fetch_add(1, std::memory_order_relaxed);
                        return;
                    }

                    if (snapshot.droppedSnapshotsBeforeThis > 0) {
                        logger->warn(
                            "[ROCK::Performance] Profiler writer dropped {} snapshot window(s) before this window",
                            snapshot.droppedSnapshotsBeforeThis);
                    }

                    logger->info("[ROCK::Performance] Profiler window: frames={} warmupComplete=yes schema=3 pid={} scopeTimes=inclusive queryCounts=exclusive queryTimingSampleEvery=64 nativePhysicsTimes=callbackBoundedWall grabAcquisitionBreakdown=1", snapshot.frames, GetCurrentProcessId());
                    for (const auto& item : snapshot.scopes) {
                        if (!item.hasData()) {
                            continue;
                        }
                        logger->info(
                            "[ROCK::Performance] Profiler {}: avgMs={:.4f} maxMs={:.4f} totalMs={:.4f} samples={} events={}",
                            scopeName(item.scope),
                            item.avgMs(),
                            item.maxMs(),
                            item.totalMs(),
                            item.samples,
                            item.events);
                        if (item.hasQueries()) {
                            const auto& query = item.memory;
                            logger->info(
                                "[ROCK::Performance] Profiler memory {}: readQueries={} writeQueries={} executeQueries={} apiFailures={} timedQueries={} sampledAvgUs={:.3f} sampledMaxUs={:.3f} sampledTotalUs={:.3f}",
                                scopeName(item.scope), query.calls[0], query.calls[1], query.calls[2], query.apiFailures,
                                query.timedSamples,
                                query.timedSamples ? ticksToMilliseconds(query.totalTicks) * 1000.0 / static_cast<double>(query.timedSamples) : 0.0,
                                ticksToMilliseconds(query.maxTicks) * 1000.0,
                                ticksToMilliseconds(query.totalTicks) * 1000.0);
                        }
                    }

                    for (const auto& item : snapshot.counters) {
                        if (!item.hasData()) {
                            continue;
                        }
                        logger->info("[ROCK::Performance] Profiler counter {}: count={}", counterName(item.counter), item.count);
                    }

                    for (const auto& item : snapshot.values) {
                        if (!item.hasData()) {
                            continue;
                        }
                        logger->info(
                            "[ROCK::Performance] Profiler value {}: avg={:.2f} max={} samples={}",
                            valueMetricName(item.metric),
                            item.avg(),
                            item.max,
                            item.samples);
                    }

                    flushIfDue();
                } catch (...) {
                    _droppedSnapshots.fetch_add(1, std::memory_order_relaxed);
                }
            }

            void drainQueue() noexcept
            {
                QueuedSnapshot snapshot{};
                while (tryDequeue(snapshot)) {
                    writeSnapshot(snapshot);
                }
            }

            void run() noexcept
            {
                _nextFlush = std::chrono::steady_clock::now() + kFlushInterval;

                for (;;) {
                    drainQueue();
                    if (_stopRequested.load(std::memory_order_acquire)) {
                        break;
                    }

                    std::unique_lock lock(_wakeMutex);
                    _wakeCv.wait_for(lock, std::chrono::milliseconds(250), [this]() noexcept {
                        return _stopRequested.load(std::memory_order_acquire) || hasPending();
                    });
                }

                drainQueue();
                flushLogger();
                _logger.reset();
            }

            std::array<QueuedSnapshot, kQueueCapacity> _queue{};
            std::atomic<std::uint64_t> _writeSeq{ 0 };
            std::atomic<std::uint64_t> _readSeq{ 0 };
            std::atomic<std::uint64_t> _droppedSnapshots{ 0 };
            std::atomic<bool> _started{ false };
            std::atomic<bool> _startFailed{ false };
            std::atomic<bool> _stopRequested{ false };
            std::mutex _threadMutex;
            std::mutex _wakeMutex;
            std::condition_variable _wakeCv;
            std::thread _thread;
            std::shared_ptr<spdlog::logger> _logger;
            std::chrono::steady_clock::time_point _nextFlush{};
        };

        AsyncProfilerWriter& asyncProfilerWriter() noexcept
        {
            static AsyncProfilerWriter writer;
            return writer;
        }

        std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)> takeSnapshot() noexcept
        {
            std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)> snapshot{};
            for (std::size_t i = 0; i < s_accum.size(); ++i) {
                auto& slot = s_accum[i];
                snapshot[i] = ScopeSnapshot{
                    .scope = static_cast<Scope>(i),
                    .totalTicks = slot.totalTicks.exchange(0, std::memory_order_acq_rel),
                    .maxTicks = slot.maxTicks.exchange(0, std::memory_order_acq_rel),
                    .samples = slot.samples.exchange(0, std::memory_order_acq_rel),
                    .events = slot.events.exchange(0, std::memory_order_acq_rel),
                    .memory = {
                        .calls = { slot.memoryQueries[0].exchange(0, std::memory_order_acq_rel),
                            slot.memoryQueries[1].exchange(0, std::memory_order_acq_rel),
                            slot.memoryQueries[2].exchange(0, std::memory_order_acq_rel) },
                        .apiFailures = slot.memoryQueryFailures.exchange(0, std::memory_order_acq_rel),
                        .timedSamples = slot.memoryQueryTimedSamples.exchange(0, std::memory_order_acq_rel),
                        .totalTicks = slot.memoryQueryTotalTicks.exchange(0, std::memory_order_acq_rel),
                        .maxTicks = slot.memoryQueryMaxTicks.exchange(0, std::memory_order_acq_rel),
                    },
                };
            }
            return snapshot;
        }

        std::array<CounterSnapshot, static_cast<std::size_t>(Counter::Count)> takeCounterSnapshot() noexcept
        {
            std::array<CounterSnapshot, static_cast<std::size_t>(Counter::Count)> snapshot{};
            for (std::size_t i = 0; i < s_counterAccum.size(); ++i) {
                auto& slot = s_counterAccum[i];
                snapshot[i] = CounterSnapshot{
                    .counter = static_cast<Counter>(i),
                    .count = slot.count.exchange(0, std::memory_order_acq_rel),
                };
            }
            return snapshot;
        }

        std::array<ValueSnapshot, static_cast<std::size_t>(ValueMetric::Count)> takeValueSnapshot() noexcept
        {
            std::array<ValueSnapshot, static_cast<std::size_t>(ValueMetric::Count)> snapshot{};
            for (std::size_t i = 0; i < s_valueAccum.size(); ++i) {
                auto& slot = s_valueAccum[i];
                snapshot[i] = ValueSnapshot{
                    .metric = static_cast<ValueMetric>(i),
                    .total = slot.total.exchange(0, std::memory_order_acq_rel),
                    .max = slot.max.exchange(0, std::memory_order_acq_rel),
                    .samples = slot.samples.exchange(0, std::memory_order_acq_rel),
                };
            }
            return snapshot;
        }

        void publishOverlayLines(const std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)>& snapshot, std::uint64_t frames) noexcept
        {
            std::scoped_lock lock(s_overlayMutex);
            s_overlayLines = {};
            s_overlayLineCount = 0;

            auto addLine = [&](const char* format, auto&&... args) {
                if (s_overlayLineCount >= s_overlayLines.size()) {
                    return;
                }
                auto& line = s_overlayLines[s_overlayLineCount++];
                std::snprintf(line.data(), line.size(), format, std::forward<decltype(args)>(args)...);
                line.back() = '\0';
            };

            addLine("ROCK PERF %lluf", static_cast<unsigned long long>(frames));
            for (const auto& item : snapshot) {
                if (!item.hasData() || s_overlayLineCount >= s_overlayLines.size()) {
                    continue;
                }
                addLine("%s avg %.3f max %.3f n %llu e %llu",
                    scopeName(item.scope),
                    item.avgMs(),
                    item.maxMs(),
                    static_cast<unsigned long long>(item.samples),
                    static_cast<unsigned long long>(item.events));
            }
        }

        void queueSnapshot(const std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)>& snapshot,
            const std::array<CounterSnapshot, static_cast<std::size_t>(Counter::Count)>& counterSnapshot,
            const std::array<ValueSnapshot, static_cast<std::size_t>(ValueMetric::Count)>& valueSnapshot,
            std::uint64_t frames) noexcept
        {
            // The frame thread publishes one fixed-size snapshot and never formats strings,
            // writes files, or waits for diagnostics; if the writer falls behind, snapshots are dropped.
            asyncProfilerWriter().enqueue(QueuedSnapshot{
                .scopes = snapshot,
                .counters = counterSnapshot,
                .values = valueSnapshot,
                .frames = frames,
            });
        }
    }

    void refreshSettings(bool enabled, int logIntervalFrames, int warmupFrames, bool overlayTextEnabled) noexcept
    {
        const auto sanitizedInterval = sanitizeIntervalFrames(logIntervalFrames);
        const auto sanitizedWarmup = sanitizeWarmupFrames(warmupFrames);
        const bool wasEnabled = s_settings.enabled.load(std::memory_order_acquire);
        const bool settingsChanged =
            s_settings.logIntervalFrames.load(std::memory_order_acquire) != sanitizedInterval ||
            s_settings.warmupFrames.load(std::memory_order_acquire) != sanitizedWarmup;

        if (wasEnabled != enabled || settingsChanged) {
            s_settings.generation.fetch_add(1, std::memory_order_acq_rel);
        }

        s_settings.logIntervalFrames.store(sanitizedInterval, std::memory_order_release);
        s_settings.warmupFrames.store(sanitizedWarmup, std::memory_order_release);
        s_settings.overlayText.store(overlayTextEnabled, std::memory_order_release);

        if (!enabled) {
            if (wasEnabled) {
                clearAccumulators();
                clearCounterAccumulators();
                clearValueAccumulators();
                clearOverlayLines();
                s_settings.frameIndex.store(0, std::memory_order_release);
                s_settings.intervalStartFrame.store(0, std::memory_order_release);
            }
            s_settings.enabled.store(false, std::memory_order_release);
            return;
        }

        if (!ensureFrequencyReady()) {
            s_settings.enabled.store(false, std::memory_order_release);
            clearCounterAccumulators();
            clearValueAccumulators();
            clearOverlayLines();
            return;
        }

        asyncProfilerWriter().start();

        if (!wasEnabled || settingsChanged) {
            clearAccumulators();
            clearCounterAccumulators();
            clearValueAccumulators();
            clearOverlayLines();
            s_settings.frameIndex.store(0, std::memory_order_release);
            s_settings.intervalStartFrame.store(0, std::memory_order_release);
        }
        s_settings.enabled.store(true, std::memory_order_release);
    }

    bool enabled() noexcept
    {
        return s_settings.enabled.load(std::memory_order_acquire);
    }

    void beginFrame() noexcept
    {
        if (!s_settings.enabled.load(std::memory_order_acquire)) {
            return;
        }
        s_settings.frameIndex.fetch_add(1, std::memory_order_acq_rel);
    }

    void endFrame() noexcept
    {
        if (!s_settings.enabled.load(std::memory_order_acquire)) {
            return;
        }

        const auto frame = s_settings.frameIndex.load(std::memory_order_acquire);
        const auto warmup = s_settings.warmupFrames.load(std::memory_order_acquire);
        if (frame <= warmup) {
            clearAccumulators();
            clearCounterAccumulators();
            clearValueAccumulators();
            s_settings.intervalStartFrame.store(frame, std::memory_order_release);
            return;
        }

        const auto interval = s_settings.logIntervalFrames.load(std::memory_order_acquire);
        const auto intervalStart = s_settings.intervalStartFrame.load(std::memory_order_acquire);
        if (frame - intervalStart < interval) {
            return;
        }

        s_settings.intervalStartFrame.store(frame, std::memory_order_release);
        const auto snapshot = takeSnapshot();
        const auto counterSnapshot = takeCounterSnapshot();
        const auto valueSnapshot = takeValueSnapshot();
        const auto frames = frame - intervalStart;
        if (s_settings.overlayText.load(std::memory_order_acquire)) {
            publishOverlayLines(snapshot, frames);
        }
        queueSnapshot(snapshot, counterSnapshot, valueSnapshot, frames);
    }

    void addEventCount(Scope scope, std::uint64_t count) noexcept
    {
        if (!s_settings.enabled.load(std::memory_order_acquire) || !validScope(scope) || count == 0) {
            return;
        }
        accumFor(scope).events.fetch_add(count, std::memory_order_relaxed);
    }

    void addCounter(Counter counter, std::uint64_t count) noexcept
    {
        if (!s_settings.enabled.load(std::memory_order_acquire) || !validCounter(counter) || count == 0) {
            return;
        }
        accumFor(counter).count.fetch_add(count, std::memory_order_relaxed);
    }

    void observeValue(ValueMetric metric, std::uint64_t value) noexcept
    {
        if (!s_settings.enabled.load(std::memory_order_acquire) || !validValueMetric(metric)) {
            return;
        }
        auto& slot = accumFor(metric);
        slot.total.fetch_add(value, std::memory_order_relaxed);
        slot.samples.fetch_add(1, std::memory_order_relaxed);
        atomicMax(slot.max, value);
    }

    bool overlayTextEnabled() noexcept
    {
        return s_settings.enabled.load(std::memory_order_acquire) && s_settings.overlayText.load(std::memory_order_acquire);
    }

    std::uint32_t copyOverlayLines(OverlayLines& outLines) noexcept
    {
        outLines = {};
        if (!overlayTextEnabled()) {
            return 0;
        }

        std::scoped_lock lock(s_overlayMutex);
        outLines = s_overlayLines;
        return s_overlayLineCount;
    }

    MemoryQuerySample beginMemoryQuery() noexcept
    {
        if (!enabled()) return {};
        const bool timed = (t_memoryQuerySequence++ & 63u) == 0;
        return { timed ? queryPerformanceTicks() : 0, t_memoryQueryScope, true };
    }

    void endMemoryQuery(MemoryQuerySample sample, MemoryQueryKind kind, bool apiSucceeded) noexcept
    {
        if (!sample.active) return;
        const auto endTicks = sample.startTicks ? queryPerformanceTicks() : 0;
        auto& slot = accumFor(sample.scope);
        slot.memoryQueries[static_cast<std::size_t>(kind)].fetch_add(1, std::memory_order_relaxed);
        if (!apiSucceeded) slot.memoryQueryFailures.fetch_add(1, std::memory_order_relaxed);
        if (sample.startTicks && endTicks >= sample.startTicks) {
            const auto ticks = endTicks - sample.startTicks;
            slot.memoryQueryTimedSamples.fetch_add(1, std::memory_order_relaxed);
            slot.memoryQueryTotalTicks.fetch_add(ticks, std::memory_order_relaxed);
            atomicMax(slot.memoryQueryMaxTicks, ticks);
        }
    }

    IntervalSample beginInterval() noexcept
    {
        if (!enabled()) return {};
        const auto generation = s_settings.generation.load(std::memory_order_acquire);
        return { queryPerformanceTicks(), generation };
    }

    bool endInterval(Scope scope, IntervalSample& sample) noexcept
    {
        const auto completed = std::exchange(sample, {});
        if (!completed.startTicks || !enabled() || !validScope(scope) ||
            completed.generation != s_settings.generation.load(std::memory_order_acquire)) return false;
        const auto endTicks = queryPerformanceTicks();
        // A completed interval shorter than one clock tick still represents a
        // real callback pair and must count toward completed physics substeps.
        if (endTicks < completed.startTicks) return false;
        recordTicks(scope, endTicks - completed.startTicks);
        return true;
    }

    ScopedTimer::ScopedTimer(Scope scope) noexcept :
        _scope(scope)
    {
        if (!s_settings.enabled.load(std::memory_order_acquire) || !validScope(scope)) {
            return;
        }

        _startTicks = queryPerformanceTicks();
        _parentScope = std::exchange(t_memoryQueryScope, _scope);
        _active = true;
    }

    ScopedTimer::~ScopedTimer()
    {
        stop();
    }

    void ScopedTimer::stop() noexcept
    {
        if (!_active) {
            return;
        }

        const auto endTicks = queryPerformanceTicks();
        if (_startTicks && endTicks > _startTicks) {
            recordTicks(_scope, endTicks - _startTicks);
        }
        t_memoryQueryScope = _parentScope;
        _active = false;
    }

}
