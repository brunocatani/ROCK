#pragma once

#include <cstdint>
#include <utility>
#include <spdlog/logger.h>

namespace rock::dynamic_collider_trace
{
    // Provisioned at skeleton-ready when bDebugGrabFrameLogging is enabled.
    // Game/physics callers submit sampled values only; the bounded async writer
    // owns disk I/O. Shutdown follows physics callback quiescence.
    void initialize() noexcept;
    void shutdown() noexcept;
    void beginFrame(bool enabled, std::uint64_t frame) noexcept;
    void capturePresentedHands(std::uint64_t frame) noexcept;
    [[nodiscard]] bool enabled() noexcept;
    [[nodiscard]] bool sample(std::uint64_t sequence) noexcept;
    [[nodiscard]] spdlog::logger* activeLogger() noexcept;
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
