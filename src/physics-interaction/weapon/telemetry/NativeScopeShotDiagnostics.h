#pragma once

#include <cstdint>

namespace RE { class NiNode; }
namespace rock::debug { struct BodyOverlayFrame; }

namespace rock::native_scope_shot_diagnostics
{
    // Process-lifetime native shot hooks. Diagnostics remain optional; the
    // explicit carried-weapon dispatch also supplies its own muzzle frame.
    bool install() noexcept;
    bool hooksReady() noexcept;
    // Main-thread lifecycle/frame calls. Shutdown stops the file worker;
    // disabled diagnostics leave the native/carried shot routing active.
    void beginFrame() noexcept;
    void shutdown() noexcept;
    void clearPresentation() noexcept;
    void publishPresentation(RE::NiNode* weapon, std::uint64_t generation, std::uint32_t form,
        bool contactKnown, bool contact, bool bipodLatched) noexcept;
    void appendOverlay(debug::BodyOverlayFrame& frame) noexcept;
}
