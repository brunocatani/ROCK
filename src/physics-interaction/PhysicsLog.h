#pragma once

#define ROCK_LOG_TRACE(category, ...)                         \
    do {                                                      \
        if (::f4cf::logger::isTraceEnabled()) {               \
            ::f4cf::logger::trace("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_DEBUG(category, ...)                         \
    do {                                                      \
        if (::f4cf::logger::isDebugEnabled()) {               \
            ::f4cf::logger::debug("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_INFO(category, ...)                          \
    do {                                                      \
        if (::f4cf::logger::isInfoEnabled()) {                \
            ::f4cf::logger::info("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_WARN(category, ...)                          \
    do {                                                      \
        if (::f4cf::logger::isWarnEnabled()) {                \
            ::f4cf::logger::warn("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_ERROR(category, ...)                         \
    do {                                                      \
        if (::f4cf::logger::isErrorEnabled()) {               \
            ::f4cf::logger::error("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_CRITICAL(category, ...)                      \
    do {                                                      \
        if (::f4cf::logger::isCriticalEnabled()) {            \
            ::f4cf::logger::critical("[ROCK::" #category "] " __VA_ARGS__); \
        }                                                     \
    } while (false)

#define ROCK_LOG_SAMPLE_DEBUG(category, time_ms, ...)         \
    do {                                                      \
        ::f4cf::logger::sampleDebug((time_ms), "[ROCK::" #category "] " __VA_ARGS__); \
    } while (false)

#define ROCK_LOG_SAMPLE_INFO(category, time_ms, ...)          \
    do {                                                      \
        ::f4cf::logger::sample((time_ms), "[ROCK::" #category "] " __VA_ARGS__); \
    } while (false)

#define ROCK_LOG_SAMPLE_WARN(category, time_ms, ...)          \
    do {                                                      \
        ::f4cf::logger::sampleWarn((time_ms), "[ROCK::" #category "] " __VA_ARGS__); \
    } while (false)
