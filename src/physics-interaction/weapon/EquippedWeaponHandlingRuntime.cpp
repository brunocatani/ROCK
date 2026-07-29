#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"

namespace rock::equipped_weapon_handling_runtime
{
    namespace
    {
        EquippedWeaponHandlingSettings s_settings{};
    }

    void publish(const EquippedWeaponHandlingSettings& settings) noexcept
    {
        s_settings = settings;
    }

    const EquippedWeaponHandlingSettings& current() noexcept
    {
        return s_settings;
    }

    void reset() noexcept
    {
        s_settings = {};
    }
}
