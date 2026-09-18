#pragma once

#include <cstdint>

namespace RE { class NiAVObject; }

// Read-only renderer witness for the same-frame held-object locomotion audit.
// Remove after rendered consumption and the corrected presentation are qualified.
namespace rock::held_render_trace
{
    enum class Phase : std::uint8_t { FrameBegin, BeforeRock, AfterRock, AfterWorldFinal };
    void install(); // bootstrap only, before active rendering
    void initialize();
    void shutdown() noexcept;
    void registerRoot(bool isLeft, std::uint64_t trace, RE::NiAVObject* root);
    void clearHand(bool isLeft) noexcept;
    void recordPhase(Phase phase, std::uint64_t frame);
}
