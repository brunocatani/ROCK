#pragma once

#if defined(ROCK_POLICY_TEST_NO_RUNTIME_LOGGER)

#define ROCK_LOG_TRACE(category, ...) do { } while (false)
#define ROCK_LOG_DEBUG(category, ...) do { } while (false)
#define ROCK_LOG_INFO(category, ...) do { } while (false)
#define ROCK_LOG_WARN(category, ...) do { } while (false)
#define ROCK_LOG_ERROR(category, ...) do { } while (false)
#define ROCK_LOG_CRITICAL(category, ...) do { } while (false)
#define ROCK_LOG_SAMPLE_DEBUG(category, time_ms, ...) do { } while (false)
#define ROCK_LOG_SAMPLE_INFO(category, time_ms, ...) do { } while (false)
#define ROCK_LOG_SAMPLE_WARN(category, time_ms, ...) do { } while (false)

#else

#include <filesystem>
#include <memory>
#include <source_location>
#include <string>
#include <string_view>
#include <utility>

using namespace std::literals;

#include "rock_support/Logger.h"

#define ROCK_LOG_TRACE(category, ...)                         \
    do {                                                      \
        if (::rock::logger::isTraceEnabled()) {               \
            ::rock::logger::trace("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_DEBUG(category, ...)                         \
    do {                                                      \
        if (::rock::logger::isDebugEnabled()) {               \
            ::rock::logger::debug("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_INFO(category, ...)                          \
    do {                                                      \
        if (::rock::logger::isInfoEnabled()) {                \
            ::rock::logger::info("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_WARN(category, ...)                          \
    do {                                                      \
        if (::rock::logger::isWarnEnabled()) {                \
            ::rock::logger::warn("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_ERROR(category, ...)                         \
    do {                                                      \
        if (::rock::logger::isErrorEnabled()) {               \
            ::rock::logger::error("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_CRITICAL(category, ...)                      \
    do {                                                      \
        if (::rock::logger::isCriticalEnabled()) {            \
            ::rock::logger::critical("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

// The format literal is the throttle key. It is extracted without touching
// the remaining arguments, so a rejected sample never evaluates them. The
// EXPAND wrapper keeps __VA_ARGS__ splitting correct under both the
// traditional and the conforming MSVC preprocessor.
#define ROCK_LOG_DETAIL_EXPAND(x) x
#define ROCK_LOG_DETAIL_FIRST_ARG(first, ...) first
#define ROCK_LOG_DETAIL_SAMPLE(level, category, time_ms, ...)                                      \
    do {                                                                                        \
        if (::rock::logger::admitSample((level),                                                \
                "[ROCK::" #category "] " ROCK_LOG_DETAIL_EXPAND(ROCK_LOG_DETAIL_FIRST_ARG(__VA_ARGS__, 0)), \
                (time_ms))) {                                                                   \
            ::rock::logger::emitSample((level), "[ROCK::" #category "] " __VA_ARGS__);          \
        }                                                                                       \
    } while (false)

#define ROCK_LOG_SAMPLE_DEBUG(category, time_ms, ...) \
    ROCK_LOG_DETAIL_SAMPLE(::spdlog::level::debug, category, time_ms, __VA_ARGS__)

#define ROCK_LOG_SAMPLE_INFO(category, time_ms, ...) \
    ROCK_LOG_DETAIL_SAMPLE(::spdlog::level::info, category, time_ms, __VA_ARGS__)

#define ROCK_LOG_SAMPLE_WARN(category, time_ms, ...) \
    ROCK_LOG_DETAIL_SAMPLE(::spdlog::level::warn, category, time_ms, __VA_ARGS__)

#endif
