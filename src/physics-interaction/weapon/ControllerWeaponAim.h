#pragma once

#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"

namespace rock::controller_weapon_aim
{
    // Use the same frame-qualified, scope-recovered native offset-chain input
    // as the positional hand solver. FRIK's weapon preset is applied downstream
    // to Weapon, not this driver. No scene pointer survives this call. Left
    // carry mirrors this canonical right-in-wand frame through its existing
    // aim path, preserving left aim trim and damped follow.
    [[nodiscard]] inline bool tryResolveCarrier(
        const RE::NiTransform& weaponWorld, RE::NiTransform& outWorld)
    {
        RE::NiTransform driverWorld{};
        if (!frik_hand_world_authority::tryGetInputDriverWorld(false, driverWorld)) {
            outWorld = {};
            return false;
        }
        outWorld = authored_weapon_grip_capture_policy::withControllerWeaponAim(weaponWorld, driverWorld);
        return true;
    }
}
