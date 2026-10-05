#pragma once

namespace rock::native_weapon_fade_suppression
{
    // Process-lifetime patch installed during Load, before player updates.
    // Native weapon alpha is always opaque, independently of equip source,
    // skeleton availability and diagnostics. Combat timers stay native.
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] bool installed() noexcept;
}
