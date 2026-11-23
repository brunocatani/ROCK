/*
 * ROCK's devbench tool. devbench is a separate F4SE plugin that runs a local
 * MCP and REST server inside the game; ROCK registers one tool with it, "rock",
 * whose required "action" argument picks the operation. The generic actions
 * match F4VR-CommonFramework's (health, config, set, clear, overrides, perf),
 * mapped onto ROCK's settings store and profiler. devbench is a development
 * tool: without it ROCK logs one line and registers nothing.
 *
 * Threads: devbench calls the tool on its listener thread. health and perf
 * answer there; every other action is queued for ROCK's next FrameBegin, or for
 * devbench's main-thread queue while FRIK's frame is not running, and the
 * caller waits up to 2 seconds. See docs/dev/devbench.md.
 */
#pragma once

namespace rock::devbench
{
    void registerTool();

    void onFrameBegin();

    [[nodiscard]] bool isArmed() noexcept;
}
