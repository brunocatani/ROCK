#pragma once

namespace RE
{
    class NiAVObject;
}

namespace rock::native_weapon_fade_suppression
{
    // Process-lifetime patch installed during Load, before player updates.
    // Native weapon alpha is always opaque, independently of equip source,
    // skeleton availability and diagnostics. Combat timers stay native.
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] bool installed() noexcept;
    // The native alpha commit reaches only the index-0 attachment. The game
    // thread calls this every frame the index-1 model is presented, with that
    // model's attachment, so it is opaque by the same native write.
    void presentSecondary(RE::NiAVObject* attachment) noexcept;
}
