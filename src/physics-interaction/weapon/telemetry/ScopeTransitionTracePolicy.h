#pragma once

#include <cstdint>

namespace rock::scope_transition_trace_policy
{
    inline constexpr std::uint64_t kTailFrames = 12;
    inline constexpr std::uint64_t kMaximumBurstFrames = 48;

    struct Signals
    {
        std::uint64_t menuEvent = 0;
        bool button = false;
        bool rendererValid = false;
        bool renderer = false;
        bool operator==(const Signals&) const = default;
    };

    // One clock for all three phases. A delayed menu-close event starts its
    // own tail, even after the button-release samples have finished. Repeated
    // pulses cannot extend one continuous burst beyond the fixed limit.
    struct Window
    {
        Signals previous{};
        std::uint64_t frame = 0;
        std::uint64_t edge = 0;
        std::uint64_t lastEdge = 0;
        std::uint64_t burstStart = 0;
        std::uint8_t phaseMask = 0;
        bool observed = false;
        bool active = false;
        bool capped = false;

        bool observe(std::uint64_t sequence, unsigned phase, Signals signals) noexcept
        {
            if (sequence == 0 || phase >= 3) { active = false; return false; }
            if (sequence < frame) *this = {};
            if (sequence != frame) {
                frame = sequence;
                phaseMask = 0;
            }
            const bool changed = observed ? signals != previous :
                signals.button || (signals.rendererValid && signals.renderer) || (signals.menuEvent & 1) != 0;
            previous = signals;
            observed = true;
            if (changed) {
                if (lastEdge == 0 || sequence - lastEdge >= kTailFrames) {
                    burstStart = sequence;
                    capped = false;
                }
                ++edge;
                lastEdge = sequence;
            }
            active = lastEdge != 0 && sequence - lastEdge < kTailFrames &&
                sequence - burstStart < kMaximumBurstFrames;
            capped = capped || (lastEdge != 0 && sequence - lastEdge < kTailFrames &&
                sequence - burstStart >= kMaximumBurstFrames);
            const auto phaseBit = static_cast<std::uint8_t>(1u << phase);
            if (!active || (phaseMask & phaseBit) != 0) return false;
            phaseMask |= phaseBit;
            return true;
        }
    };
}
