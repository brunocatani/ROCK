#pragma once

namespace frik::rock::weapon_authority_lifecycle_policy
{
    /*
     * HIGGS stops advancing hand interaction when game-stopping menus or world
     * interruptions are active, but it also makes owned interaction state yield
     * before returning to the game. ROCK keeps FRIK weapon authority, input
     * blocking, and left-hand collision suppression in separate systems, so the
     * interruption decision is centralized here to keep all three lifetimes in
     * lockstep.
     */
    inline bool shouldClearWeaponAuthorityForUpdateInterruption(bool menuOpen, bool rockDisabled, bool skeletonUnavailable)
    {
        return menuOpen || rockDisabled || skeletonUnavailable;
    }
}
