#pragma once

#include <cstdint>
#include <utility>
#include <spdlog/logger.h>

namespace rock { struct PhysicsFrameContext; }

namespace rock::dynamic_collider_trace
{
    // Provisioned at skeleton-ready for grab-frame logging or skeleton overlays.
    // Overlays enable only the sampled held-presentation trace, not dense physics.
    // Game/physics callers submit sampled values only; the bounded async writer
    // owns disk I/O. Shutdown follows physics callback quiescence.
    void initialize() noexcept;
    void shutdown() noexcept;
    void beginFrame(bool enabled, std::uint64_t frame) noexcept;
    void capturePresentedHands(std::uint64_t frame) noexcept;
    // Game-thread, read-only census for missing NPC contacts across animations.
    void captureNpcCollisionState(const PhysicsFrameContext& frame) noexcept;
    [[nodiscard]] bool enabled() noexcept;
    [[nodiscard]] bool motorOutputEnabled() noexcept;
    [[nodiscard]] bool presentationEnabled() noexcept;
    [[nodiscard]] bool sample(std::uint64_t sequence) noexcept;
    [[nodiscard]] spdlog::logger* activeLogger() noexcept;
    [[nodiscard]] spdlog::logger* activeWeaponLogger() noexcept;
    void suppressAfterError() noexcept;

    template<class... Args>
    void write(spdlog::format_string_t<Args...> format, Args&&... args) noexcept
    {
        if (auto* log = activeLogger()) {
            try { log->info(format, std::forward<Args>(args)...); }
            catch (...) { suppressAfterError(); }
        }
    }

    // Independent file retention, shared bounded queue and writer lifetime.
    template<class... Args>
    void writeWeapon(spdlog::format_string_t<Args...> format, Args&&... args) noexcept
    {
        if (auto* log = activeWeaponLogger()) {
            try { log->info(format, std::forward<Args>(args)...); }
            catch (...) { suppressAfterError(); }
        }
    }
}
