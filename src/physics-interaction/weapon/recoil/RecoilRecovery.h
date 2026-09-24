#pragma once

namespace rock::recoil_recovery
{
    // Exact zero only: weak but nonzero springs remain authored values.
    inline constexpr bool repairZeroSpring(float& spring) noexcept
    {
        if (spring != 0.0f) return false;
        spring = 150.0f;
        return true;
    }

    // Called on the existing equipped-weapon game-frame path. No engine pointer
    // survives the call; the engine owns and destroys the corrected runtime copy.
    void repairEquippedWeapon() noexcept;
}
