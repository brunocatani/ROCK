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
    inline constexpr std::uint32_t FO4_LAYER_UNIDENTIFIED = 0;
    inline constexpr std::uint32_t FO4_LAYER_PROJECTILE = 6;
    inline constexpr std::uint32_t FO4_LAYER_SPELL = 7;
    inline constexpr std::uint32_t FO4_LAYER_CONEPROJECTILE = 38;
    inline constexpr std::uint32_t FO4_LAYER_STATIC = 1;
    inline constexpr std::uint32_t FO4_LAYER_ANIMSTATIC = 2;
    inline constexpr std::uint32_t FO4_LAYER_CLUTTER = 4;
    inline constexpr std::uint32_t FO4_LAYER_WEAPON = 5;
    inline constexpr std::uint32_t FO4_LAYER_BIPED = 8;
    inline constexpr std::uint32_t FO4_LAYER_NONCOLLIDABLE = 15;
    inline constexpr std::uint32_t FO4_LAYER_DEBRIS_SMALL = 19;
    inline constexpr std::uint32_t FO4_LAYER_DEBRIS_LARGE = 20;
    inline constexpr std::uint32_t FO4_LAYER_SHELLCASING = 25;
    inline constexpr std::uint32_t FO4_LAYER_CLUTTER_LARGE = 29;
    inline constexpr std::uint32_t FO4_LAYER_CHARCONTROLLER = 30;
    inline constexpr std::uint32_t FO4_LAYER_DEADBIP = 32;
    inline constexpr std::uint32_t FO4_LAYER_BIPED_NO_CC = 33;
    inline constexpr std::uint32_t FO4_LAYER_CAMERASPHERE = 36;
    inline constexpr std::uint32_t FO4_LAYER_ITEMPICK = 40;
    inline constexpr std::uint32_t FO4_LAYER_LINEOFSIGHT = 41;
    inline constexpr std::uint32_t FO4_LAYER_PATHPICK = 42;
    inline constexpr std::uint32_t ROCK_LAYER_HAND = 43;
    inline constexpr std::uint32_t ROCK_LAYER_WEAPON = 44;
    inline constexpr std::uint32_t FO4_LAYER_SPELLEXPLOSION = 45;
    inline constexpr std::uint32_t FO4_LAYER_DROPPINGPICK = 46;
    /*
     * Reload mobile ammo bodies need to collide with equipped weapon socket
     * bodies. Layer 45 is native SPELLEXPLOSION, so the reviewed split is:
     * mobile reload ammo uses the ROCK hand/tool layer 43, weapon-owned sockets
     * and action bodies use the ROCK weapon/tool layer 44, and role/owner
     * metadata distinguishes reload contacts from real hands and weapon hulls.
     */
    inline constexpr std::uint32_t ROCK_LAYER_RELOAD = ROCK_LAYER_HAND;
    inline constexpr std::uint32_t FO4_LAYER_MAX_CONFIGURED = 47;

    inline constexpr bool isRockOwnedReusableLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON;
    }

    inline constexpr bool isDynamicPropInteractionLayer(std::uint32_t layer)
    {
        switch (layer) {
        case FO4_LAYER_CLUTTER:
        case FO4_LAYER_WEAPON:
        case FO4_LAYER_DEBRIS_SMALL:
        case FO4_LAYER_DEBRIS_LARGE:
        case FO4_LAYER_SHELLCASING:
        case FO4_LAYER_CLUTTER_LARGE:
            return true;
        default:
            return false;
        }
    }

    inline constexpr bool isActorOrBipedLayer(std::uint32_t layer)
    {
        return layer == FO4_LAYER_BIPED || layer == FO4_LAYER_DEADBIP || layer == FO4_LAYER_BIPED_NO_CC;
    }

    inline constexpr bool isWorldSurfaceLayer(std::uint32_t layer)
    {
        return layer == FO4_LAYER_STATIC || layer == FO4_LAYER_ANIMSTATIC;
    }

    inline constexpr bool isQueryOnlyLayer(std::uint32_t layer)
    {
        return layer == FO4_LAYER_ITEMPICK || layer == FO4_LAYER_LINEOFSIGHT || layer == FO4_LAYER_PATHPICK || layer == FO4_LAYER_CAMERASPHERE;
    }

    inline constexpr bool isPassivePushInteractionLayer(std::uint32_t layer)
    {
        return isDynamicPropInteractionLayer(layer) || isActorOrBipedLayer(layer);
    }

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

    inline constexpr std::uint64_t allConfiguredLayerBits()
    {
        return (FO4_LAYER_MAX_CONFIGURED >= 63) ? ~0ULL : ((1ULL << (FO4_LAYER_MAX_CONFIGURED + 1)) - 1ULL);
    }

    inline constexpr std::uint64_t configuredMask(std::uint64_t mask) { return mask & allConfiguredLayerBits(); }

    inline constexpr bool configuredLayerMaskMatches(std::uint64_t currentMask, std::uint64_t expectedMask)
    {
        return configuredMask(currentMask) == configuredMask(expectedMask);
    }

    inline constexpr std::uint64_t withoutLayer(std::uint64_t mask, std::uint32_t layer) { return mask & ~(1ULL << layer); }

    inline constexpr std::uint64_t withLayer(std::uint64_t mask, std::uint32_t layer) { return mask | (1ULL << layer); }

    inline constexpr std::uint64_t buildRockHandExpectedMask(bool includeWeaponLayer)
    {
        /*
         * HIGGS' hand and weapon tool bodies use a broad collision layer for
         * physical contact; ROCK keeps that passive-contact behavior here so
         * bone-derived finger bodies can shove dead bodies like the old hand
         * box, while active grab eligibility remains gated by the classifier.
         */
        std::uint64_t mask = allConfiguredLayerBits();
        mask = withoutLayer(mask, FO4_LAYER_UNIDENTIFIED);
        mask = withoutLayer(mask, FO4_LAYER_STATIC);
        mask = withoutLayer(mask, FO4_LAYER_ANIMSTATIC);
        mask = withoutLayer(mask, FO4_LAYER_PROJECTILE);
        mask = withoutLayer(mask, FO4_LAYER_SPELL);
        mask = withoutLayer(mask, FO4_LAYER_CONEPROJECTILE);
        mask = withoutLayer(mask, FO4_LAYER_NONCOLLIDABLE);
        mask = withoutLayer(mask, ROCK_LAYER_HAND);
        mask = withoutLayer(mask, FO4_LAYER_CAMERASPHERE);
        mask = withoutLayer(mask, FO4_LAYER_ITEMPICK);
        mask = withoutLayer(mask, FO4_LAYER_LINEOFSIGHT);
        mask = withoutLayer(mask, FO4_LAYER_PATHPICK);
        mask = withoutLayer(mask, FO4_LAYER_CHARCONTROLLER);
        if (includeWeaponLayer) {
            mask = withLayer(mask, ROCK_LAYER_WEAPON);
        }
        return mask;
    }

    inline constexpr std::uint64_t buildRockWeaponExpectedMask(bool blocksProjectiles, bool blocksSpells)
    {
        std::uint64_t mask = allConfiguredLayerBits();
        mask = withoutLayer(mask, FO4_LAYER_UNIDENTIFIED);
        mask = withoutLayer(mask, FO4_LAYER_STATIC);
        mask = withoutLayer(mask, FO4_LAYER_NONCOLLIDABLE);
        mask = withoutLayer(mask, FO4_LAYER_CHARCONTROLLER);
        mask = withoutLayer(mask, ROCK_LAYER_WEAPON);
        mask = withoutLayer(mask, FO4_LAYER_CAMERASPHERE);
        mask = withoutLayer(mask, FO4_LAYER_ITEMPICK);
        mask = withoutLayer(mask, FO4_LAYER_LINEOFSIGHT);
        mask = withoutLayer(mask, FO4_LAYER_PATHPICK);
        if (!blocksProjectiles) {
            mask = withoutLayer(mask, FO4_LAYER_PROJECTILE);
            mask = withoutLayer(mask, FO4_LAYER_CONEPROJECTILE);
        }
        if (!blocksSpells) {
            mask = withoutLayer(mask, FO4_LAYER_SPELL);
        }
        return mask;
    }

    inline constexpr std::uint64_t buildRockReloadExpectedMask(bool blocksProjectiles, bool blocksSpells)
    {
        if constexpr (ROCK_LAYER_RELOAD == ROCK_LAYER_HAND) {
            return buildRockHandExpectedMask(true);
        } else {
            return buildRockWeaponExpectedMask(blocksProjectiles, blocksSpells);
        }
    }

    inline void applyLayerExpectedMask(std::uint64_t* matrix, std::uint32_t layer, std::uint64_t expectedMask)
    {
        if (!matrix) {
            return;
        }

        for (std::uint32_t other = 0; other <= FO4_LAYER_MAX_CONFIGURED; ++other) {
            setPair(matrix, layer, other, (expectedMask & (1ULL << other)) != 0);
        }
    }

    inline void applyRockHandLayerPolicy(std::uint64_t* matrix, bool includeWeaponLayer)
    {
        applyLayerExpectedMask(matrix, ROCK_LAYER_HAND, buildRockHandExpectedMask(includeWeaponLayer));
    }

    inline void applyRockWeaponLayerPolicy(std::uint64_t* matrix, bool blocksProjectiles, bool blocksSpells)
    {
        applyLayerExpectedMask(matrix, ROCK_LAYER_WEAPON, buildRockWeaponExpectedMask(blocksProjectiles, blocksSpells));
    }

    inline void applyRockReloadLayerPolicy(std::uint64_t* matrix, bool blocksProjectiles, bool blocksSpells)
    {
        applyLayerExpectedMask(matrix, ROCK_LAYER_RELOAD, buildRockReloadExpectedMask(blocksProjectiles, blocksSpells));
    }
}
