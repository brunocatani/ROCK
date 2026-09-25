#pragma once
#include <cstdint>

namespace rock::loose_weapon_experiment
{
    // The experiment admits one physical reference, with either one or two
    // hands on it. Two different guns and a native equipped gun fail closed.
    constexpr std::uint32_t selectReference(std::uint32_t right, std::uint32_t left) noexcept
    {
        return right && left && right != left ? 0 : right ? right : left;
    }
    constexpr bool canAdmit(bool runtimeReady, bool nativeWeaponPresent, std::uint32_t selected) noexcept
    {
        return runtimeReady && !nativeWeaponPresent && selected != 0;
    }
    constexpr bool suppressNativePress(bool heldGun, bool pressed, bool gameplay, bool menu) noexcept
    {
        return heldGun && pressed && gameplay && !menu;
    }
}
