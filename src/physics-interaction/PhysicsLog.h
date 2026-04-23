#pragma once

// PhysicsLog.h — Structured logging macros for the ROCK physics module.
//
// WHY: All ROCK subsystems need consistent, categorized logging to aid debugging
// in a complex physics system where per-frame data, state transitions, and lifecycle
// events need to be distinguishable. Using category-prefixed macros means log output
// can be filtered by subsystem even when everything goes to the same F4SE log file.
//
// Categories: Init, Update, Hand, Constraint, Config
// Levels: TRACE (per-frame), DEBUG (state transitions), INFO (lifecycle),
//         WARN (recoverable), ERROR (failures), CRITICAL (fatal/disabled)

#define ROCK_LOG_TRACE(category, ...) logger::trace("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_DEBUG(category, ...) logger::debug("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_INFO(category, ...)  logger::info("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_WARN(category, ...)  logger::warn("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_ERROR(category, ...) logger::error("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_CRITICAL(category, ...) logger::critical("[ROCK::" #category "] " __VA_ARGS__)
