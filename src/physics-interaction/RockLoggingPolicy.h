#pragma once

#include <algorithm>

namespace rock::logging_policy
{
    /*
     * ROCK shares FRIK's spdlog-backed level numbering so both DLLs can be
     * diagnosed from the same mental model and the same INI shape. The policy
     * stays pure so config parsing, macro gating, and tests agree without
     * pulling the runtime logger into focused regression tests.
     */
    enum class LogLevel : int
    {
        Trace = 0,
        Debug = 1,
        Info = 2,
        Warn = 3,
        Error = 4,
        Critical = 5,
        Off = 6,
    };

    constexpr int DefaultLogLevel = static_cast<int>(LogLevel::Info);
    constexpr int DefaultLogSampleMilliseconds = 2000;
    constexpr const char* DefaultLogPattern = "%Y-%m-%d %H:%M:%S.%e [%l] %v";

    constexpr int clampLogLevel(const int rawLevel)
    {
        return std::clamp(rawLevel, static_cast<int>(LogLevel::Trace), static_cast<int>(LogLevel::Off));
    }

    constexpr int sanitizeSampleMilliseconds(const int rawMilliseconds)
    {
        return std::clamp(rawMilliseconds, 250, 60000);
    }

    constexpr bool shouldEmit(const int configuredLevel, const LogLevel messageLevel)
    {
        const int normalizedConfigured = clampLogLevel(configuredLevel);
        return normalizedConfigured != static_cast<int>(LogLevel::Off) && static_cast<int>(messageLevel) >= normalizedConfigured;
    }

    constexpr const char* logLevelName(const int rawLevel)
    {
        switch (static_cast<LogLevel>(clampLogLevel(rawLevel))) {
        case LogLevel::Trace:
            return "trace";
        case LogLevel::Debug:
            return "debug";
        case LogLevel::Info:
            return "info";
        case LogLevel::Warn:
            return "warn";
        case LogLevel::Error:
            return "error";
        case LogLevel::Critical:
            return "critical";
        case LogLevel::Off:
            return "off";
        default:
            return "unknown";
        }
    }
}
