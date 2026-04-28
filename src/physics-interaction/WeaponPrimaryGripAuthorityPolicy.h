#pragma once

namespace frik::rock::weapon_primary_grip_authority_policy
{
    /*
     * One-handed primary weapon authority is only valid when ROCK has a real
     * mesh-derived grip frame and FRIK is not already being driven by the
     * two-handed support authority. This keeps fallback/current and melee cases
     * inside FRIK's existing weapon handling instead of publishing low-confidence
     * frames that would fight authored weapon offsets.
     */
    inline bool shouldPublishPrimaryGripAuthority(bool meshGripSelected, bool supportGripActive, bool weaponDrawn, bool meleeWeaponDrawn, bool authorityEnabled)
    {
        return authorityEnabled && meshGripSelected && !supportGripActive && weaponDrawn && !meleeWeaponDrawn;
    }
}
