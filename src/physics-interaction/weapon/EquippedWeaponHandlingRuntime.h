#pragma once

#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"

namespace rock::equipped_weapon_handling_runtime
{
    /*
     * Main-frame value service for low-level grab/equip helpers that cannot
     * retain a PhysicsInteraction reference. ROCK publishes one complete
     * settings snapshot before any frame work, and resets it during shutdown.
     * All access is confined to the game thread; no engine pointers cross it.
     */
    void publish(const EquippedWeaponHandlingSettings& settings) noexcept;
    [[nodiscard]] const EquippedWeaponHandlingSettings& current() noexcept;
    void reset() noexcept;
}
