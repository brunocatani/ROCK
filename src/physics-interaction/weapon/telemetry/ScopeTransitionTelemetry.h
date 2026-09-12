#pragma once

#include <cstdint>
#include <utility>
#include <spdlog/logger.h>

namespace rock::scope_transition_telemetry
{
    enum class Phase : unsigned { BeforeFrik, AfterFrik, AfterRock };

    // Game-thread session; gated by the existing bDebugHandWorldAuthority.
    // The bounded async writer receives formatted values, never engine nodes.
    void initialize() noexcept;
    void shutdown() noexcept;
    // Existing UI sink may call this on its event thread; atomic data only.
    void onMenuEvent(bool open) noexcept;
    void capture(Phase phase, std::uint64_t schedulerSequence) noexcept;

    // Borrowed only for a synchronous game-thread diagnostic. No ownership or
    // retained pointer crosses a frame; null outside the bounded window.
    [[nodiscard]] spdlog::logger* activeLogger() noexcept;
    [[nodiscard]] std::uint64_t sequence() noexcept;
    void suppressAfterError() noexcept;

    template<class... Args>
    void write(spdlog::format_string_t<Args...> format, Args&&... args) noexcept
    {
        if (auto* log = activeLogger()) {
            try { log->info(format, std::forward<Args>(args)...); }
            catch (...) { suppressAfterError(); }
        }
    }
}
