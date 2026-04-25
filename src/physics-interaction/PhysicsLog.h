#pragma once

#define ROCK_LOG_TRACE(category, ...) logger::trace("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_DEBUG(category, ...) logger::debug("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_INFO(category, ...) logger::info("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_WARN(category, ...) logger::warn("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_ERROR(category, ...) logger::error("[ROCK::" #category "] " __VA_ARGS__)
#define ROCK_LOG_CRITICAL(category, ...) logger::critical("[ROCK::" #category "] " __VA_ARGS__)
