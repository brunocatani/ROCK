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
        };

        std::array<ScopeAccum, static_cast<std::size_t>(Scope::Count)> s_accum;
        std::array<CounterAccum, static_cast<std::size_t>(Counter::Count)> s_counterAccum;
        std::array<ValueAccum, static_cast<std::size_t>(ValueMetric::Count)> s_valueAccum;
        Settings s_settings;
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
            case Scope::GeneratedBodyContactRegistry:
                return "generatedBodyRegistry";
            case Scope::WeaponColliderBuild:
                return "weaponColliderBuild";
            case Scope::WeaponColliderCreate:
                return "weaponColliderCreate";
            case Scope::TwoHandedGripStart:
                return "twoHandedGripStart";
            case Scope::SupportGripSuppression:
                return "supportGripSuppression";
            case Scope::SelectionCasts:
                return "selectionCasts";
            case Scope::SoftContact:
                return "softContact";
            case Scope::DebugOverlayPublish:
                return "debugOverlayPublish";
            case Scope::DebugOverlayRender:
                return "debugOverlayRender";
            case Scope::ContactResolve:
                return "contactResolve";
            case Scope::NativeContactCallback:
                return "nativeContactCallbacks";
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
            case Counter::WeaponRebuildReasonSettingsChanged:
                return "weaponRebuildReasonSettingsChanged";
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
            if (!validScope(scope) || ticks == 0) {
                return;
            }

            auto& slot = accumFor(scope);
            slot.totalTicks.fetch_add(ticks, std::memory_order_relaxed);
            slot.samples.fetch_add(1, std::memory_order_relaxed);
            atomicMax(slot.maxTicks, ticks);
        }

        struct ScopeSnapshot
        {
            Scope scope{ Scope::Count };
            std::uint64_t totalTicks{ 0 };
            std::uint64_t maxTicks{ 0 };
            std::uint64_t samples{ 0 };
            std::uint64_t events{ 0 };

            [[nodiscard]] bool hasData() const noexcept { return samples > 0 || events > 0; }
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

                    logger->info("[ROCK::Performance] Profiler window: frames={} warmupComplete=yes", snapshot.frames);
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

    ScopedTimer::ScopedTimer(Scope scope) noexcept :
        _scope(scope)
    {
        if (!s_settings.enabled.load(std::memory_order_acquire) || !validScope(scope)) {
            return;
        }

        _startTicks = queryPerformanceTicks();
        _active = _startTicks != 0;
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
        if (endTicks > _startTicks) {
            recordTicks(_scope, endTicks - _startTicks);
        }
        _active = false;
    }

    FrameScope::FrameScope() noexcept :
        _timer(Scope::FrameUpdate)
    {
        beginFrame();
    }

    FrameScope::~FrameScope()
    {
        _timer.stop();
        endFrame();
    }
}
