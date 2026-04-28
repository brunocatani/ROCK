#pragma once

namespace frik::rock::weapon_muzzle_authority_math
{
    /*
     * ROCK writes the final equipped weapon transform after FRIK's normal weapon
     * update has already run. FRIK's proven muzzle fix copies the projectile node
     * world transform into the fire node so flashes/projectiles originate at the
     * current barrel tip. Keeping the transform rule here gives ROCK the same
     * final-owner behavior without depending on FRIK's weapon adjuster pass.
     */
    template <class Transform>
    [[nodiscard]] Transform fireNodeLocalFromProjectileWorld(const Transform& projectileWorld)
    {
        return projectileWorld;
    }
}
