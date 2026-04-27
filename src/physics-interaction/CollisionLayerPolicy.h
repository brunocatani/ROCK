#pragma once

#include <cstdint>

namespace frik::rock::collision_layer_policy
{
    /*
     * Weapons of Fate turns firearm shots into real projectile bodies. ROCK's
     * weapon collision layer still needs to collide with clutter and world
     * geometry, but it must not broadly collide with projectile layers by
     * default or the player's bullets can hit the weapon's own generated hulls
     * while aiming down sights. This is kept as a layer-mask policy instead of
     * a projectile hook because owner-aware projectile filtering needs separate
     * verified collector layout work.
     */
    inline constexpr std::uint32_t FO4_LAYER_PROJECTILE = 6;
    inline constexpr std::uint32_t FO4_LAYER_SPELL = 7;
    inline constexpr std::uint32_t FO4_LAYER_CONEPROJECTILE = 38;
    inline constexpr std::uint32_t ROCK_LAYER_WEAPON = 44;

    inline void setPair(std::uint64_t* matrix, std::uint32_t layerA, std::uint32_t layerB, bool enabled)
    {
        if (!matrix) {
            return;
        }

        const std::uint64_t bitA = 1ULL << layerA;
        const std::uint64_t bitB = 1ULL << layerB;
        if (enabled) {
            matrix[layerA] |= bitB;
            matrix[layerB] |= bitA;
        } else {
            matrix[layerA] &= ~bitB;
            matrix[layerB] &= ~bitA;
        }
    }

    inline void applyWeaponProjectileBlockingPolicy(std::uint64_t* matrix, std::uint32_t weaponLayer, bool blocksProjectiles, bool blocksSpells)
    {
        setPair(matrix, weaponLayer, FO4_LAYER_PROJECTILE, blocksProjectiles);
        setPair(matrix, weaponLayer, FO4_LAYER_CONEPROJECTILE, blocksProjectiles);
        setPair(matrix, weaponLayer, FO4_LAYER_SPELL, blocksSpells);
    }
}
