#pragma once

#include <cstdint>

namespace RE { class PlayerCharacter; }

namespace rock::weapon_action_trace
{
    // Initialize once at skeleton readiness. The immutable writer/module map
    // lives until process shutdown; its worker receives text, never game pointers.
    void initialize() noexcept;
    void invalidateContext() noexcept;
    // Provider frame thread only. Bits per hand: valid=1, surface latch=2,
    // loose object=4, firing grip=8. Right is the low nibble.
    void publishHands(std::uint64_t frame, std::uint8_t hands) noexcept;
    // Called inside the existing draw detour, before its original dispatch.
    void recordDraw(RE::PlayerCharacter* player, bool draw,
        bool suppressedByHeldEquip, std::uintptr_t caller) noexcept;
}
