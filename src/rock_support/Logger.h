#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <source_location>
#include <string>
#include <string_view>
#include <utility>

#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#define ROCK_MAKE_SOURCE_LOGGER(log_func, log_level)                                                                                      \
    template <class... Args>                                                                                                             \
    struct [[maybe_unused]] log_func                                                                                                     \
    {                                                                                                                                    \
        log_func() = delete;                                                                                                             \
                                                                                                                                         \
        explicit log_func(                                                                                                               \
            spdlog::format_string_t<Args...> fmt,                                                                                        \
            Args&&... args,                                                                                                              \
            const std::source_location& loc = std::source_location::current())                                                          \
        {                                                                                                                                \
            constexpr auto level = spdlog::level::log_level;                                                                             \
            if (!internal::loggerInstance || !internal::loggerInstance->should_log(level)) {                                             \
                return;                                                                                                                  \
            }                                                                                                                            \
            const spdlog::source_loc sourceLoc{                                                                                          \
                loc.file_name(),                                                                                                         \
                static_cast<int>(loc.line()),                                                                                            \
                loc.function_name()                                                                                                      \
            };                                                                                                                           \
            internal::loggerInstance->log(sourceLoc, level, fmt, std::forward<Args>(args)...);                                           \
        }                                                                                                                                \
    };                                                                                                                                   \
                                                                                                                                         \
    template <class... Args>                                                                                                             \
    log_func(spdlog::format_string_t<Args...>, Args&&...) -> log_func<Args...>;

namespace rock::logger::internal
{
    inline constexpr std::string_view kRawLoggerName{ "RAW" };
    inline constexpr std::string_view kDefaultPattern{ "%Y-%m-%d %H:%M:%S.%e [%l] %v" };
    inline constexpr auto kFlushOnLevel = spdlog::level::err;
    inline constexpr std::size_t kSampleThrottleSlotCount = 512;

    inline std::shared_ptr<spdlog::logger> loggerInstance;
    inline std::shared_ptr<spdlog::logger> rawLoggerInstance;
    inline int logLevel = -1;
    inline std::string logPattern{ kDefaultPattern };

    struct SampleThrottleSlot
    {
        std::atomic<std::uint64_t> keyHash{ 0 };
        std::atomic<std::int64_t> nextAllowedMs{ 0 };
    };

    // Fixed storage keeps sampled logging allocation-free before formatting.
    // Hash collisions conservatively suppress a diagnostic until its slot expires.
    inline std::array<SampleThrottleSlot, kSampleThrottleSlotCount> sampleThrottleSlots;

    [[nodiscard]] constexpr int clampLogLevel(const int value) noexcept
    {
        return std::clamp(value, static_cast<int>(spdlog::level::trace), static_cast<int>(spdlog::level::off));
    }

    [[nodiscard]] constexpr const char* logLevelName(const int value) noexcept
    {
        switch (static_cast<spdlog::level::level_enum>(clampLogLevel(value))) {
        case spdlog::level::trace:
            return "trace";
        case spdlog::level::debug:
            return "debug";
        case spdlog::level::info:
            return "info";
        case spdlog::level::warn:
            return "warn";
        case spdlog::level::err:
            return "error";
        case spdlog::level::critical:
            return "critical";
        case spdlog::level::off:
            return "off";
        default:
            return "unknown";
        }
    }

    [[nodiscard]] inline std::uint64_t sampleKeyHash(const std::string_view key) noexcept
    {
        std::uint64_t hash = 14695981039346656037ull;
        for (const unsigned char character : key) {
            hash ^= character;
            hash *= 1099511628211ull;
        }
        return hash == 0 ? 1 : hash;
    }

    [[nodiscard]] inline std::int64_t steadyClockMilliseconds() noexcept
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    [[nodiscard]] inline bool shouldEmitSample(const std::string_view key, const int intervalMs) noexcept
    {
        const auto hash = sampleKeyHash(key);
        auto& slot = sampleThrottleSlots[hash % sampleThrottleSlots.size()];
        const auto nowMs = steadyClockMilliseconds();
        const auto throttleMs = std::max<std::int64_t>(0, intervalMs);
        const auto nextAllowedMs = nowMs + throttleMs;

        const auto storedHash = slot.keyHash.load(std::memory_order_acquire);
        if (storedHash != hash) {
            if (storedHash != 0 && nowMs < slot.nextAllowedMs.load(std::memory_order_acquire)) {
                return false;
            }
            slot.keyHash.store(hash, std::memory_order_release);
            slot.nextAllowedMs.store(nextAllowedMs, std::memory_order_release);
            return true;
        }

        auto observedNextMs = slot.nextAllowedMs.load(std::memory_order_acquire);
        while (nowMs >= observedNextMs) {
            if (slot.nextAllowedMs.compare_exchange_weak(
                    observedNextMs,
                    nextAllowedMs,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                return true;
            }
        }
        return false;
    }

    inline void applyFlushPolicy()
    {
        if (loggerInstance) {
            loggerInstance->flush_on(kFlushOnLevel);
        }
        if (rawLoggerInstance) {
            rawLoggerInstance->flush_on(kFlushOnLevel);
        }
    }

    template <class... Args>
    void sampleImpl(
        const spdlog::level::level_enum level,
        const int intervalMs,
        spdlog::format_string_t<Args...> format,
        Args&&... args)
    {
        if (!loggerInstance || !loggerInstance->should_log(level)) {
            return;
        }

        const fmt::basic_string_view<char> formatView = format;
        if (!shouldEmitSample(std::string_view(formatView.data(), formatView.size()), intervalMs)) {
            return;
        }

        auto formatted = fmt::format(format, std::forward<Args>(args)...);
        loggerInstance->log(level, "[sample] {}", formatted);
    }

    class HybridFormatter final : public spdlog::formatter
    {
    public:
        HybridFormatter() :
            _inner(std::make_unique<spdlog::pattern_formatter>(logPattern))
        {}

        [[nodiscard]] std::unique_ptr<formatter> clone() const override
        {
            return std::make_unique<HybridFormatter>();
        }

        void format(const spdlog::details::log_msg& message, spdlog::memory_buf_t& destination) override
        {
            if (message.logger_name == kRawLoggerName) {
                destination.append(message.payload);
                destination.push_back('\n');
                return;
            }
            _inner->format(message, destination);
        }

    private:
        std::unique_ptr<spdlog::pattern_formatter> _inner;
    };
}

namespace rock::logger
{
    ROCK_MAKE_SOURCE_LOGGER(trace, trace);
    ROCK_MAKE_SOURCE_LOGGER(debug, debug);
    ROCK_MAKE_SOURCE_LOGGER(info, info);
    ROCK_MAKE_SOURCE_LOGGER(warn, warn);
    ROCK_MAKE_SOURCE_LOGGER(error, err);
    ROCK_MAKE_SOURCE_LOGGER(critical, critical);

    [[nodiscard]] inline bool isTraceEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::trace);
    }

    [[nodiscard]] inline bool isDebugEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::debug);
    }

    [[nodiscard]] inline bool isInfoEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::info);
    }

    [[nodiscard]] inline bool isWarnEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::warn);
    }

    [[nodiscard]] inline bool isErrorEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::err);
    }

    [[nodiscard]] inline bool isCriticalEnabled()
    {
        return internal::loggerInstance && internal::loggerInstance->should_log(spdlog::level::critical);
    }

    template <class... Args>
    void sample(spdlog::format_string_t<Args...> format, Args&&... args)
    {
        internal::sampleImpl(spdlog::level::info, 1000, format, std::forward<Args>(args)...);
    }

    template <class... Args>
    void sample(const int intervalMs, spdlog::format_string_t<Args...> format, Args&&... args)
    {
        internal::sampleImpl(spdlog::level::info, intervalMs, format, std::forward<Args>(args)...);
    }

    template <class... Args>
    void sampleDebug(const int intervalMs, spdlog::format_string_t<Args...> format, Args&&... args)
    {
        internal::sampleImpl(spdlog::level::debug, intervalMs, format, std::forward<Args>(args)...);
    }

    template <class... Args>
    void sampleWarn(const int intervalMs, spdlog::format_string_t<Args...> format, Args&&... args)
    {
        internal::sampleImpl(spdlog::level::warn, intervalMs, format, std::forward<Args>(args)...);
    }

    template <class... Args>
    void infoRaw(spdlog::format_string_t<Args...> format, Args&&... args)
    {
        if (!internal::rawLoggerInstance || !internal::rawLoggerInstance->should_log(spdlog::level::info)) {
            return;
        }
        internal::rawLoggerInstance->log(spdlog::level::info, format, std::forward<Args>(args)...);
    }

    inline void init(const std::string_view logFileName)
    {
        auto path = F4SE::log::log_directory();
        const std::string_view expectedGamePath = REL::Module::IsVR() ? "Fallout4VR/F4SE" : "Fallout4/F4SE";
        if (!path.value().generic_string().ends_with(expectedGamePath)) {
            path = path.value().parent_path().append(expectedGamePath);
        }

        *path /= fmt::format("{}.log", logFileName);
        auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            path->string(),
            10 * 1024 * 1024,
            5,
            true);

        internal::loggerInstance = std::make_shared<spdlog::logger>("GLOBAL", sink);
        internal::loggerInstance->set_level(spdlog::level::info);
        internal::loggerInstance->set_formatter(std::make_unique<internal::HybridFormatter>());
        spdlog::set_default_logger(internal::loggerInstance);

        internal::rawLoggerInstance = std::make_shared<spdlog::logger>(std::string(internal::kRawLoggerName), sink);
        internal::rawLoggerInstance->set_level(spdlog::level::info);
        internal::applyFlushPolicy();
    }

    inline void setLogLevelAndPattern(const int requestedLogLevel, const std::string& requestedPattern)
    {
        const int normalizedLevel = internal::clampLogLevel(requestedLogLevel);
        if (internal::logLevel != normalizedLevel) {
            info("Set log level = {} ({})", normalizedLevel, internal::logLevelName(normalizedLevel));
            internal::logLevel = normalizedLevel;
            const auto level = static_cast<spdlog::level::level_enum>(normalizedLevel);
            if (internal::loggerInstance) {
                internal::loggerInstance->set_level(level);
            }
            if (internal::rawLoggerInstance) {
                internal::rawLoggerInstance->set_level(level);
            }
            internal::applyFlushPolicy();
        }

        const std::string normalizedPattern = requestedPattern.empty() ?
            std::string(internal::kDefaultPattern) : requestedPattern;
        if (internal::logPattern != normalizedPattern) {
            info("Set log pattern = {}", normalizedPattern);
            internal::logPattern = normalizedPattern;
            if (internal::loggerInstance) {
                internal::loggerInstance->set_formatter(std::make_unique<internal::HybridFormatter>());
            }
        }
    }
}

namespace logger = rock::logger;

#undef ROCK_MAKE_SOURCE_LOGGER
