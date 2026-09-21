#pragma once

#include <cstdint>

namespace RE { class NiAVObject; }

// Read-only renderer witness for the same-frame held-object locomotion audit.
// Remove after rendered consumption and the corrected presentation are qualified.
namespace rock::held_render_trace
{
    enum class Phase : std::uint8_t { FrameBegin, BeforeRock, AfterRock, AfterWorldFinal };
    // Consecutive samples expose alternating-frame errors without dense logging
    // for the entire run. The phase and renderer witnesses use the same window.
    [[nodiscard]] inline constexpr bool sampleFrame(std::uint64_t frame) noexcept
    {
        return frame != 0 && frame % 120 < 12;
    }
    // Physics callbacks borrow only the atomically published frame identity;
    // they must not read the mutable game-thread runtime frame for diagnostics.
    [[nodiscard]] std::uint64_t sampledFrame() noexcept;
    void install(); // bootstrap only, before active rendering
    void initialize();
    void shutdown() noexcept;
    void registerRoot(bool isLeft, std::uint64_t trace, RE::NiAVObject* root);
    void clearHand(bool isLeft) noexcept;
    void recordPhase(Phase phase, std::uint64_t frame);
}
