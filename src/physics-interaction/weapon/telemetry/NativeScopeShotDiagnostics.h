#pragma once

#include <cstdint>

namespace RE { class NiNode; }
namespace rock::debug { struct BodyOverlayFrame; }

namespace rock::native_scope_shot_diagnostics
{
    // Process-lifetime observational call hooks; no engine state is written.
    bool install() noexcept;
    // Main-thread lifecycle/frame calls. Shutdown stops the file worker;
    // disabled hooks only call their original native targets.
    void beginFrame() noexcept;
    void shutdown() noexcept;
    void clearPresentation() noexcept;
    void publishPresentation(RE::NiNode* weapon, std::uint64_t generation, std::uint32_t form,
        bool contactKnown, bool contact, bool bipodLatched) noexcept;
    void appendOverlay(debug::BodyOverlayFrame& frame) noexcept;
}
