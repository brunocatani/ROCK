#pragma once

#include <cstdint>

namespace RE { class InputEvent; }
namespace rock::equipped_weapon_visual_state { struct Snapshot; }

namespace rock::trigger_equip_readiness_trace
{
    void install() noexcept;
    // Temporary investigation of delayed trigger-equip firing. Uses the existing
    // debug-logging switch, at most five seconds per explicit trigger equip,
    // fixed storage, and the owning game thread. Remove after diagnosis.
    void begin(std::uint32_t form, std::uint32_t reference) noexcept;
    void stage(const char* name) noexcept;
    void frame(const equipped_weapon_visual_state::Snapshot& visual,
        bool bridgeVisible, bool gripPending, bool nativeAnimation, bool menu) noexcept;
    void input(const RE::InputEvent* event, bool rockBlocked,
        bool nativeCalled, bool nativeAccepted, const void* handler = nullptr) noexcept;
    void stop(const char* reason) noexcept;
}
