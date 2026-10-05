#pragma once

#include <cstdint>

namespace rock::native_weapon_equip_audio
{
    // Each verified equip/unequip caller passes actor+0xD0 as the sound's
    // position pointer. Compare identity only; never dereference this address.
    [[nodiscard]] constexpr bool isPlayerSource(std::uintptr_t player, std::uintptr_t position) noexcept
    {
        return player != 0 && position == player + 0xD0;
    }

    // Startup-only, process-lifetime hooks. Suppress player weapon equip,
    // unequip and fast-equip playback while retaining the surrounding events.
    [[nodiscard]] bool install() noexcept;
}
