/*
 * ROCK's devbench tool. devbench is a separate F4SE plugin that runs a local
 * MCP and REST server inside the game; ROCK registers one tool with it, "rock",
 * whose required "action" argument picks the operation. The generic actions
 * match F4VR-CommonFramework's (health, state, config, set, clear, overrides,
 * perf), mapped onto ROCK's settings store and profiler, and ROCK publishes
 * its lifecycle as "rock.*" events. devbench is a development tool: without it
 * ROCK logs one line and registers nothing.
 *
 * Threads: devbench calls the tool on its listener thread. health, state and
 * perf answer there; every other action is queued for ROCK's next FrameBegin,
 * or for devbench's main-thread queue while FRIK's frame is not running, and
 * the caller waits up to 2 seconds. See docs/dev/devbench.md.
 */
#pragma once

#include <concepts>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "devbench/RockDevBenchState.h"

namespace rock::devbench
{
    void registerTool();

    void onFrameBegin();

    void publishState(RockState&& state);

    [[nodiscard]] bool isArmed() noexcept;

    namespace internal
    {
        [[nodiscard]] bool canEmit() noexcept;

        void emit(std::string_view topic, const nlohmann::json& payload);
    }

    /**
     * Publishes "rock.<topic>" through devbench. makePayload is called only when the event can go somewhere, so
     * without devbench an emit is one atomic load. Events of every mod share one small ring in devbench: emit on a
     * change, never every frame. Any thread.
     */
    template <std::invocable MakePayload>
    void emit(std::string_view topic, MakePayload&& makePayload)
    {
        if (internal::canEmit())
            internal::emit(topic, std::forward<MakePayload>(makePayload)());
    }

    inline void emit(std::string_view topic)
    {
        emit(topic, [] { return nlohmann::json::object(); });
    }
}
