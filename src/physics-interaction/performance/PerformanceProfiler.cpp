#include "physics-interaction/performance/PerformanceProfiler.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "physics-interaction/PhysicsLog.h"

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
        std::mutex s_profilerLoggerMutex;
        std::shared_ptr<spdlog::logger> s_profilerLogger;
        std::string s_profilerLoggerPattern;

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

        std::string profilerLogPattern()
        {
            const auto& pattern = ::f4cf::logger::internal::_logPattern;
            return pattern.empty() ? "%Y-%m-%d %H:%M:%S.%e [%l] %v" : pattern;
        }

        std::shared_ptr<spdlog::logger> ensureProfilerLogger() noexcept
        {
            try {
                std::scoped_lock lock(s_profilerLoggerMutex);
                const auto pattern = profilerLogPattern();

                if (s_profilerLogger) {
                    if (s_profilerLoggerPattern != pattern) {
                        s_profilerLogger->set_formatter(std::make_unique<spdlog::pattern_formatter>(pattern));
                        s_profilerLoggerPattern = pattern;
                    }
                    return s_profilerLogger;
                }

                auto path = F4SE::log::log_directory();
                if (!path.has_value()) {
                    return nullptr;
                }

                const auto gamepath = REL::Module::IsVR() ? "Fallout4VR/F4SE" : "Fallout4/F4SE";
                if (!path.value().generic_string().ends_with(gamepath)) {
                    path = path.value().parent_path().append(gamepath);
                }

                *path /= "ROCK_Profiler.log";
                auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path->string(), 1024 * 1024 * 10, 5, true);
                s_profilerLogger = std::make_shared<spdlog::logger>("ROCK_PROFILER", sink);
                s_profilerLogger->set_level(spdlog::level::info);
                s_profilerLogger->flush_on(spdlog::level::info);
                s_profilerLogger->set_formatter(std::make_unique<spdlog::pattern_formatter>(pattern));
                s_profilerLoggerPattern = pattern;
                return s_profilerLogger;
            } catch (...) {
                return nullptr;
            }
        }

        void writeProfilerFileLine(const std::string& message) noexcept
        {
            try {
                const auto logger = ensureProfilerLogger();
                if (!logger) {
                    return;
                }
                logger->log(spdlog::level::info, "{}", message);
            } catch (...) {
            }
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

        void logSnapshot(const std::array<ScopeSnapshot, static_cast<std::size_t>(Scope::Count)>& snapshot,
            const std::array<CounterSnapshot, static_cast<std::size_t>(Counter::Count)>& counterSnapshot,
            const std::array<ValueSnapshot, static_cast<std::size_t>(ValueMetric::Count)>& valueSnapshot,
            std::uint64_t frames) noexcept
        {
            ROCK_LOG_INFO(Performance, "Profiler window: frames={} warmupComplete=yes", frames);
            writeProfilerFileLine(fmt::format("[ROCK::Performance] Profiler window: frames={} warmupComplete=yes", frames));
            for (const auto& item : snapshot) {
                if (!item.hasData()) {
                    continue;
                }
                ROCK_LOG_INFO(Performance,
                    "Profiler {}: avgMs={:.4f} maxMs={:.4f} totalMs={:.4f} samples={} events={}",
                    scopeName(item.scope),
                    item.avgMs(),
                    item.maxMs(),
                    item.totalMs(),
                    item.samples,
                    item.events);
                writeProfilerFileLine(fmt::format(
                    "[ROCK::Performance] Profiler {}: avgMs={:.4f} maxMs={:.4f} totalMs={:.4f} samples={} events={}",
                    scopeName(item.scope),
                    item.avgMs(),
                    item.maxMs(),
                    item.totalMs(),
                    item.samples,
                    item.events));
            }

            for (const auto& item : counterSnapshot) {
                if (!item.hasData()) {
                    continue;
                }
                ROCK_LOG_INFO(Performance, "Profiler counter {}: count={}", counterName(item.counter), item.count);
                writeProfilerFileLine(fmt::format("[ROCK::Performance] Profiler counter {}: count={}", counterName(item.counter), item.count));
            }

            for (const auto& item : valueSnapshot) {
                if (!item.hasData()) {
                    continue;
                }
                ROCK_LOG_INFO(Performance,
                    "Profiler value {}: avg={:.2f} max={} samples={}",
                    valueMetricName(item.metric),
                    item.avg(),
                    item.max,
                    item.samples);
                writeProfilerFileLine(fmt::format(
                    "[ROCK::Performance] Profiler value {}: avg={:.2f} max={} samples={}",
                    valueMetricName(item.metric),
                    item.avg(),
                    item.max,
                    item.samples));
            }
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
        logSnapshot(snapshot, counterSnapshot, valueSnapshot, frames);
        if (s_settings.overlayText.load(std::memory_order_acquire)) {
            publishOverlayLines(snapshot, frames);
        }
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
