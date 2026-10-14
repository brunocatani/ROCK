#pragma once

#include <cstdint>

namespace rock::collision_layer_policy
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
    inline constexpr std::uint32_t FO4_LAYER_FILTER_MASK = 0x7F;
    inline constexpr std::uint32_t FO4_LAYER_STATIC = 1;
    inline constexpr std::uint32_t FO4_LAYER_ANIMSTATIC = 2;
    inline constexpr std::uint32_t FO4_LAYER_TRANSPARENT = 3;
    inline constexpr std::uint32_t FO4_LAYER_CLUTTER = 4;
    inline constexpr std::uint32_t FO4_LAYER_WEAPON = 5;
    inline constexpr std::uint32_t FO4_LAYER_PROJECTILE = 6;
    inline constexpr std::uint32_t FO4_LAYER_SPELL = 7;
    inline constexpr std::uint32_t FO4_LAYER_BIPED = 8;
    inline constexpr std::uint32_t FO4_LAYER_TREES = 9;
    inline constexpr std::uint32_t FO4_LAYER_PROPS = 10;
    inline constexpr std::uint32_t FO4_LAYER_TERRAIN = 13;
    inline constexpr std::uint32_t FO4_LAYER_NONCOLLIDABLE = 15;
    inline constexpr std::uint32_t FO4_LAYER_GROUND = 17;
    inline constexpr std::uint32_t FO4_LAYER_DEBRIS_SMALL = 19;
    inline constexpr std::uint32_t FO4_LAYER_DEBRIS_LARGE = 20;
    inline constexpr std::uint32_t FO4_LAYER_SHELLCASING = 25;
    inline constexpr std::uint32_t FO4_LAYER_TRANSPARENT_SMALL = 26;
    inline constexpr std::uint32_t FO4_LAYER_INVISIBLE_WALL = 27;
    inline constexpr std::uint32_t FO4_LAYER_TRANSPARENT_SMALL_ANIM = 28;
    inline constexpr std::uint32_t FO4_LAYER_CLUTTER_LARGE = 29;
    inline constexpr std::uint32_t FO4_LAYER_CHARCONTROLLER = 30;
    inline constexpr std::uint32_t FO4_LAYER_STAIRHELPER = 31;
    inline constexpr std::uint32_t FO4_LAYER_DEADBIP = 32;
    inline constexpr std::uint32_t FO4_LAYER_BIPED_NO_CC = 33;
    inline constexpr std::uint32_t FO4_LAYER_AVOIDBOX = 34;
    inline constexpr std::uint32_t FO4_LAYER_COLLISIONBOX = 35;
    inline constexpr std::uint32_t FO4_LAYER_CAMERASPHERE = 36;
    inline constexpr std::uint32_t FO4_LAYER_CONEPROJECTILE = 38;
    inline constexpr std::uint32_t FO4_LAYER_ITEMPICK = 40;
    inline constexpr std::uint32_t FO4_LAYER_LINEOFSIGHT = 41;
    inline constexpr std::uint32_t FO4_LAYER_PATHPICK = 42;
    inline constexpr std::uint32_t ROCK_LAYER_HAND = 43;
    inline constexpr std::uint32_t ROCK_LAYER_WEAPON = 44;
    inline constexpr std::uint32_t FO4_LAYER_SPELLEXPLOSION = 45;
    inline constexpr std::uint32_t FO4_LAYER_DROPPINGPICK = 46;
    /*
     * FO4VR's vanilla BGSCollisionLayer/name table is configured only through
     * layer 46. The hknp filter matrix is still 64 rows wide, so ROCK owns 47 as
     * explicit extended matrix capacity for generated body colliders. Do not
     * treat this as a vanilla configured layer or depend on BGSCollisionLayer
     * records to initialize the Havok row/column.
     */
    inline constexpr std::uint32_t ROCK_LAYER_BODY = 47;
    /*
     * Dynamic hand proxy bodies live on their own extended row so they collide
     * with static world-surface layers and explicitly tagged car bodies, plus
     * layer 33 when experimental NPC dynamic collisions are enabled.
     * Native clutter stays excluded: car identity is carried by the dedicated
     * rows below, never inferred from the shared clutter layers in the solver.
     * The proxy is a solver-side visual-stop driver, not ordinary generated
     * gameplay contact evidence. A separate bounded callback channel may use
     * its palm/fingertip pairs for provider-scoped fixed-surface latches.
     */
    inline constexpr std::uint32_t ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY = 48;
    /*
     * ExplodableCar movable statics temporarily move onto one of these rows
     * while a close hand selection owns them. Keeping separate rows preserves
     * the native CLUTTER vs CLUTTER_LARGE collision contracts while exposing
     * only those tagged bodies to the dynamic-hand solver proxy.
     */
    inline constexpr std::uint32_t ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER = 49;
    inline constexpr std::uint32_t ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER = 50;
    /*
     * The dynamic weapon proxy is deliberately limited to solver obstacles:
     * static world surfaces, explicitly tagged ExplodableCar bodies, and
     * optionally native layer-33 NPC bodies.
     * Layer-44 generated hulls remain the sole weapon contact/evidence path
     * for hands, actors, projectiles, and ordinary dynamic props.
     */
    inline constexpr std::uint32_t ROCK_LAYER_DYNAMIC_WEAPON_PROXY = 51;
    /*
     * The left-hand twins need a distinct stable row. This keeps each hand's
     * own 17-body compound internally non-colliding while allowing only the
     * intended right<->left, right<->weapon, and left<->weapon graph edges.
     * Bodies never change rows when ownership changes; exact owned hand/weapon
     * pairs are suppressed by the native counted pair filter instead.
     */
    inline constexpr std::uint32_t ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY = 52;
    /*
     * Generated hulls of the second equipped weapon (native equipment index
     * 1). Same world contract as layer 44, plus contact with the primary's
     * hulls for weapon-on-weapon collision. Hands (43) are excluded: with two
     * guns equipped both hands hold one, and the holding hand must not touch
     * its own gun. The player's body colliders (47) and dynamic proxies are
     * excluded as well; this row only produces contacts and pushes props.
     */
    inline constexpr std::uint32_t ROCK_LAYER_SECONDARY_WEAPON = 53;
    // Opt-in stored weapon displays. The simulation filter further restricts
    // these pairs to the exact empty-hand bodies published by their owner.
    inline constexpr std::uint32_t ROCK_LAYER_STORED_DISPLAY = 54;
    // Passive copies of native animated limbs while their native bodies are
    // absent from collision. Only dynamic hands and the equipped weapon stop
    // against them; they are not native actors, damage targets or grab surfaces.
    inline constexpr std::uint32_t ROCK_LAYER_PASSIVE_OBSTACLE = 55;
    /*
     * Dynamic proxy of the second equipped weapon: the same solver-obstacle
     * contract as layer 51, plus layer 51 itself, so the two held guns stop
     * against each other visibly. A separate row keeps each proxy's contact
     * routing and pair suppression independent.
     */
    inline constexpr std::uint32_t ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY = 56;
    // Optional solver targets available to negotiated interaction extensions.
    inline constexpr std::uint64_t kOptionalObstacleLayers =
        (std::uint64_t{1} << FO4_LAYER_BIPED_NO_CC) | (std::uint64_t{1} << ROCK_LAYER_PASSIVE_OBSTACLE);

    inline constexpr std::uint32_t FO4_LAYER_VANILLA_CONFIGURED_COUNT = 47;
    inline constexpr std::uint32_t FO4_LAYER_MATRIX_ADDRESSABLE_COUNT = 64;

    // Generated player tools/proxies only: the tagged car rows are real world
    // objects and must remain eligible for native weapon hits and sound.
    inline constexpr bool isRockGeneratedColliderLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON ||
               layer == ROCK_LAYER_BODY || layer == ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY || layer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY ||
               layer == ROCK_LAYER_SECONDARY_WEAPON || layer == ROCK_LAYER_STORED_DISPLAY ||
               layer == ROCK_LAYER_PASSIVE_OBSTACLE || layer == ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY;
    }

    inline constexpr bool isRockOwnedReusableLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON || layer == ROCK_LAYER_BODY ||
               layer == ROCK_LAYER_SECONDARY_WEAPON;
    }

    /*
     * Every matrix row ROCK creates bodies on. Used by the VRMeleeImpact hook
     * to identify contact partners that must never be visible to the native
     * melee contact-to-hit path: that event stream bypasses the collision
     * filter entirely (verified in-game: events arrive for matrix-disabled
     * pairs and for bodies carrying the no-collide bit), so ROCK bodies can
     * only be excluded at the hook boundary.
     */
    inline constexpr bool isRockOwnedMatrixLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON ||
               (layer >= ROCK_LAYER_BODY && layer <= ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY);
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
        case ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER:
        case ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER:
            return true;
        default:
            return false;
        }
    }

    inline constexpr bool isDynamicWorldCarLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER ||
               layer == ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER;
    }

    inline constexpr std::uint32_t dynamicWorldCarLayerForNativeLayer(std::uint32_t layer)
    {
        if (layer == FO4_LAYER_CLUTTER) {
            return ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER;
        }
        if (layer == FO4_LAYER_CLUTTER_LARGE) {
            return ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER;
        }
        return FO4_LAYER_UNIDENTIFIED;
    }

    inline constexpr std::uint32_t nativeLayerForDynamicWorldCarLayer(std::uint32_t layer)
    {
        if (layer == ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER) {
            return FO4_LAYER_CLUTTER;
        }
        if (layer == ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER) {
            return FO4_LAYER_CLUTTER_LARGE;
        }
        return FO4_LAYER_UNIDENTIFIED;
    }

    inline constexpr bool isActorOrBipedLayer(std::uint32_t layer)
    {
        return layer == FO4_LAYER_BIPED || layer == FO4_LAYER_DEADBIP || layer == FO4_LAYER_BIPED_NO_CC;
    }

    inline constexpr bool isWorldSurfaceLayer(std::uint32_t layer)
    {
        switch (layer) {
        case FO4_LAYER_STATIC:
        case FO4_LAYER_ANIMSTATIC:
        case FO4_LAYER_TRANSPARENT:
        case FO4_LAYER_TREES:
        case FO4_LAYER_TERRAIN:
        case FO4_LAYER_GROUND:
        case FO4_LAYER_TRANSPARENT_SMALL:
        case FO4_LAYER_INVISIBLE_WALL:
        case FO4_LAYER_TRANSPARENT_SMALL_ANIM:
        case FO4_LAYER_STAIRHELPER:
        case FO4_LAYER_AVOIDBOX:
        case FO4_LAYER_COLLISIONBOX:
            return true;
        default:
            return false;
        }
    }

    inline constexpr bool isDynamicWeaponProxyLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY || layer == ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY;
    }

    inline constexpr bool isDynamicWeaponProxyObstacleLayer(std::uint32_t layer)
    {
        // The matrix owns experimental admission. Once admitted, NPC contact
        // must reach the same solver/visual response as world contact. Keep
        // surface-grab and weapon-bracing eligibility separate. The other
        // held weapon's proxy is an obstacle like a wall: a proxy never
        // contacts its own row, so this only ever matches the peer.
        return isWorldSurfaceLayer(layer) || isDynamicWorldCarLayer(layer) ||
               layer == FO4_LAYER_BIPED_NO_CC || layer == ROCK_LAYER_PASSIVE_OBSTACLE ||
               isDynamicWeaponProxyLayer(layer);
    }

    inline constexpr bool isDynamicWeaponProxySolverObstacleLayer(
        std::uint32_t layer)
    {
        return isDynamicWeaponProxyObstacleLayer(layer) ||
               layer == ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY;
    }

    inline constexpr bool isDynamicHandInteractionLayer(std::uint32_t layer)
    {
        return layer == ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY;
    }

    inline constexpr std::uint32_t dynamicHandProxyLayerForHand(bool isLeft)
    {
        return isLeft ? ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY :
                        ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY;
    }

    inline constexpr bool isDynamicHandProxySurfaceLayer(std::uint32_t layer)
    {
        return isWorldSurfaceLayer(layer) || isDynamicWorldCarLayer(layer);
    }

    /*
     * Fallout VR's native player character controller must keep authoritative
     * world support, actor collision and hard blockers, but its broad contact
     * bubble should not impart impulses to clutter, loose guns, or ROCK-generated
     * tool bodies. This policy lives beside the layer constants
     * so hooks can make a per-contact decision without rewriting the global
     * layer matrix or changing ROCK hand/body collider layers.
     */
    struct PlayerCharacterControllerContactPolicyInput
    {
        bool filterEnabled = false;
        bool playerController = false;
        bool targetLayerKnown = false;
        std::uint32_t targetLayer = FO4_LAYER_UNIDENTIFIED;
        bool targetIsMovableStatic = false;
        bool targetIsCar = false;
        bool targetIsLooseWeapon = false;
    };

    struct PlayerCharacterControllerContactPolicyDecision
    {
        bool suppress = false;
        const char* reason = "native";
    };

    inline constexpr bool isPlayerCharacterControllerSupportLayer(std::uint32_t layer)
    {
        return isWorldSurfaceLayer(layer);
    }

    inline constexpr bool isNativePlayerCollisionBodyLayer(std::uint32_t layer)
    {
        return !isRockOwnedReusableLayer(layer) &&
               layer != FO4_LAYER_CHARCONTROLLER &&
               isActorOrBipedLayer(layer);
    }

    inline constexpr PlayerCharacterControllerContactPolicyDecision evaluatePlayerCharacterControllerContact(
        const PlayerCharacterControllerContactPolicyInput& input)
    {
        if (!input.filterEnabled) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "filterDisabled" };
        }
        if (!input.playerController) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "nonPlayerController" };
        }
        if (!input.targetLayerKnown) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "unknownTargetLayer" };
        }
        // Keep the engine-admitted NPC movement constraints. Removing these
        // permits walking through actors even when melee contacts still work.
        // Native filters still decide which actor/ragdoll pairs exist.
        if (input.targetLayer == FO4_LAYER_CHARCONTROLLER || isActorOrBipedLayer(input.targetLayer)) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "nativeActor" };
        }
        // Loose weapon references share layer 5 with equipped attack bodies.
        // Only positive loose-object ownership permits push suppression here.
        if (input.targetLayer == FO4_LAYER_WEAPON && input.targetIsLooseWeapon) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = true, .reason = "looseWeapon" };
        }
        // Locomotion push suppression must not consume incoming attack contacts.
        if (input.targetLayer == FO4_LAYER_WEAPON || input.targetLayer == FO4_LAYER_PROJECTILE ||
            input.targetLayer == FO4_LAYER_SPELL || input.targetLayer == FO4_LAYER_CONEPROJECTILE ||
            input.targetLayer == FO4_LAYER_SPELLEXPLOSION) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "nativeAttack" };
        }
        if (input.targetIsCar) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "carCollision" };
        }
        if (input.targetIsMovableStatic && isPlayerCharacterControllerSupportLayer(input.targetLayer)) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = true, .reason = "movableStaticSupportLayer" };
        }
        if (isPlayerCharacterControllerSupportLayer(input.targetLayer)) {
            return PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "supportLayer" };
        }
        return PlayerCharacterControllerContactPolicyDecision{ .suppress = true, .reason = "nonSupportLayer" };
    }

    inline constexpr bool isQueryOnlyLayer(std::uint32_t layer)
    {
        return layer == FO4_LAYER_ITEMPICK || layer == FO4_LAYER_LINEOFSIGHT || layer == FO4_LAYER_PATHPICK || layer == FO4_LAYER_CAMERASPHERE;
    }

    inline constexpr bool isMatrixAddressableLayer(std::uint32_t layer)
    {
        return layer < FO4_LAYER_MATRIX_ADDRESSABLE_COUNT;
    }

    inline constexpr std::uint64_t layerBitOrZero(std::uint32_t layer)
    {
        return isMatrixAddressableLayer(layer) ? (1ULL << layer) : 0ULL;
    }

    inline void setPair(std::uint64_t* matrix, std::uint32_t layerA, std::uint32_t layerB, bool enabled)
    {
        if (!matrix || !isMatrixAddressableLayer(layerA) || !isMatrixAddressableLayer(layerB)) {
            return;
        }

        const std::uint64_t bitA = layerBitOrZero(layerA);
        const std::uint64_t bitB = layerBitOrZero(layerB);
        if (bitA == 0 || bitB == 0) {
            return;
        }

        if (enabled) {
            matrix[layerA] |= bitB;
            matrix[layerB] |= bitA;
        } else {
            matrix[layerA] &= ~bitB;
            matrix[layerB] &= ~bitA;
        }
    }

    inline constexpr std::uint64_t allVanillaConfiguredLayerBits()
    {
        return (FO4_LAYER_VANILLA_CONFIGURED_COUNT >= 64) ? ~0ULL : ((1ULL << FO4_LAYER_VANILLA_CONFIGURED_COUNT) - 1ULL);
    }

    inline constexpr std::uint64_t withoutLayer(std::uint64_t mask, std::uint32_t layer) { return mask & ~layerBitOrZero(layer); }

    inline constexpr std::uint64_t withLayer(std::uint64_t mask, std::uint32_t layer) { return mask | layerBitOrZero(layer); }

    inline constexpr bool maskEnablesLayer(std::uint64_t mask, std::uint32_t layer)
    {
        const auto bit = layerBitOrZero(layer);
        return bit != 0 && (mask & bit) != 0;
    }

    inline constexpr std::uint64_t buildRockHandExpectedMask(bool includeStaticWorld)
    {
        /*
         * ROCK's generated hand and weapon tool bodies use real keyframed Havok
         * collision as contact evidence while higher-level ownership gates
         * decide visual hand authority. Hand bodies collide broadly enough to
         * produce body/world contacts, but hand-hand self collision and query-only
         * layers stay excluded so the generated palm and finger set does not
         * become its own solver noise source.
         */
        std::uint64_t mask = allVanillaConfiguredLayerBits();
        mask = withoutLayer(mask, FO4_LAYER_UNIDENTIFIED);
        if (!includeStaticWorld) {
            mask = withoutLayer(mask, FO4_LAYER_STATIC);
            mask = withoutLayer(mask, FO4_LAYER_ANIMSTATIC);
        }
        mask = withoutLayer(mask, FO4_LAYER_PROJECTILE);
        mask = withoutLayer(mask, FO4_LAYER_SPELL);
        mask = withoutLayer(mask, FO4_LAYER_CONEPROJECTILE);
        mask = withoutLayer(mask, FO4_LAYER_NONCOLLIDABLE);
        mask = withoutLayer(mask, ROCK_LAYER_HAND);
        mask = withoutLayer(mask, ROCK_LAYER_BODY);
        mask = withoutLayer(mask, FO4_LAYER_CAMERASPHERE);
        mask = withoutLayer(mask, FO4_LAYER_ITEMPICK);
        mask = withoutLayer(mask, FO4_LAYER_LINEOFSIGHT);
        mask = withoutLayer(mask, FO4_LAYER_PATHPICK);
        mask = withoutLayer(mask, FO4_LAYER_CHARCONTROLLER);
        // Weapon hulls (44): support, part and socket contact need them.
        mask = withLayer(mask, ROCK_LAYER_WEAPON);
        return mask;
    }

    inline constexpr std::uint64_t buildRockWeaponExpectedMask(bool blocksProjectiles, bool blocksSpells, bool includeBodyLayer = false)
    {
        // Static and animstatic world collision are mandatory for generated weapons.
        std::uint64_t mask = allVanillaConfiguredLayerBits();
        mask = withoutLayer(mask, FO4_LAYER_UNIDENTIFIED);
        mask = withoutLayer(mask, FO4_LAYER_NONCOLLIDABLE);
        mask = withoutLayer(mask, FO4_LAYER_CHARCONTROLLER);
        // FO4_LAYER_WEAPON stays enabled: dropped weapons, NPC weapons, and
        // the player's equipped native weapon bodies keep physical contact
        // with ROCK's generated weapon. Native melee protection lives in the
        // VRMeleeImpact hook, which drops ROCK-owned contact partners before
        // native hit processing sees them (that event stream ignores this
        // matrix entirely).
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
        if (includeBodyLayer) {
            mask = withLayer(mask, ROCK_LAYER_BODY);
        }
        return mask;
    }

    inline constexpr std::uint64_t buildRockSecondaryWeaponExpectedMask(bool blocksProjectiles, bool blocksSpells)
    {
        std::uint64_t mask = buildRockWeaponExpectedMask(blocksProjectiles, blocksSpells, false);
        mask = withoutLayer(mask, ROCK_LAYER_HAND);
        mask = withLayer(mask, ROCK_LAYER_WEAPON);
        return mask;
    }

    inline constexpr std::uint64_t buildRockBodyExpectedMask()
    {
        // Generated body colliders always retain static and animstatic contact.
        std::uint64_t mask = 0;
        mask = withLayer(mask, FO4_LAYER_STATIC);
        mask = withLayer(mask, FO4_LAYER_ANIMSTATIC);
        mask = withLayer(mask, FO4_LAYER_CLUTTER);
        mask = withLayer(mask, FO4_LAYER_WEAPON);
        mask = withLayer(mask, FO4_LAYER_DEBRIS_SMALL);
        mask = withLayer(mask, FO4_LAYER_DEBRIS_LARGE);
        mask = withLayer(mask, FO4_LAYER_SHELLCASING);
        mask = withLayer(mask, FO4_LAYER_CLUTTER_LARGE);
        mask = withLayer(mask, ROCK_LAYER_WEAPON);
        return mask;
    }

    inline void applyLayerExpectedMask(std::uint64_t* matrix, std::uint32_t layer, std::uint64_t expectedMask)
    {
        if (!matrix || !isMatrixAddressableLayer(layer)) {
            return;
        }

        for (std::uint32_t other = 0; other < FO4_LAYER_MATRIX_ADDRESSABLE_COUNT; ++other) {
            setPair(matrix, layer, other, maskEnablesLayer(expectedMask, other));
        }
    }

    inline constexpr bool layerPairEnabledFromRow(const std::uint64_t* matrix, std::uint32_t rowLayer, std::uint32_t otherLayer)
    {
        const auto bit = layerBitOrZero(otherLayer);
        return matrix && isMatrixAddressableLayer(rowLayer) && bit != 0 && ((matrix[rowLayer] & bit) != 0);
    }

    inline constexpr bool layerPairSymmetricMatches(const std::uint64_t* matrix, std::uint32_t layerA, std::uint32_t layerB, bool expectedEnabled)
    {
        return layerPairEnabledFromRow(matrix, layerA, layerB) == expectedEnabled &&
               layerPairEnabledFromRow(matrix, layerB, layerA) == expectedEnabled;
    }

    inline constexpr bool rockDynamicExtensionPairsMatch(const std::uint64_t* matrix, std::uint64_t requested)
    {
        if (!matrix) return false;
        for (std::uint32_t layer=0; layer<64; ++layer) {
            if (!maskEnablesLayer(kOptionalObstacleLayers, layer)) continue;
            const bool enabled=maskEnablesLayer(requested, layer);
            if (!layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY, layer, enabled) ||
                !layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, layer, enabled) ||
                !layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_WEAPON_PROXY, layer, enabled) ||
                !layerPairSymmetricMatches(matrix, ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY, layer, enabled)) return false;
        }
        return true;
    }

    inline void applyRockHandLayerPolicy(std::uint64_t* matrix, bool includeStaticWorld)
    {
        applyLayerExpectedMask(matrix, ROCK_LAYER_HAND, buildRockHandExpectedMask(includeStaticWorld));
    }

    inline void applyRockBodyLayerPolicy(std::uint64_t* matrix)
    {
        applyLayerExpectedMask(matrix, ROCK_LAYER_BODY, buildRockBodyExpectedMask());
    }

    inline constexpr std::uint64_t buildRockDynamicHandProxyExpectedMask(
        bool isLeft = false,
        std::uint64_t extensionObstacleLayers = 0,
        bool retiredWeaponHandCollision = false)
    {
        std::uint64_t mask = 0;
        for (std::uint32_t layer = 0; layer < FO4_LAYER_MATRIX_ADDRESSABLE_COUNT; ++layer) {
            if (isWorldSurfaceLayer(layer)) {
                mask = withLayer(mask, layer);
            }
        }
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER);
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER);
        mask = withLayer(
            mask,
            isLeft ? ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY :
                     ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        mask = withLayer(mask, ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY);
        mask = withLayer(mask, ROCK_LAYER_STORED_DISPLAY);
        // The simulation-pair filter admits only fixed world WEAP bodies on
        // this shared native row. Dynamic and actor-owned weapons stay excluded.
        if (retiredWeaponHandCollision) {
            mask = withLayer(mask, FO4_LAYER_WEAPON);
        }
        mask |= extensionObstacleLayers & kOptionalObstacleLayers;
        return mask;
    }

    inline constexpr std::uint64_t buildRockDynamicWeaponProxyExpectedMask(
        std::uint64_t extensionObstacleLayers = 0)
    {
        std::uint64_t mask = 0;
        for (std::uint32_t layer = 0; layer < FO4_LAYER_MATRIX_ADDRESSABLE_COUNT; ++layer) {
            if (isWorldSurfaceLayer(layer) || isDynamicWorldCarLayer(layer)) {
                mask = withLayer(mask, layer);
            }
        }
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
        mask = withLayer(mask, ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY);
        mask |= extensionObstacleLayers & kOptionalObstacleLayers;
        return mask;
    }

    // The primary contract with the peer bit swapped: each proxy row names
    // the other, so neither row write depends on the order they are applied.
    inline constexpr std::uint64_t buildRockSecondaryDynamicWeaponProxyExpectedMask(
        std::uint64_t extensionObstacleLayers = 0)
    {
        return withLayer(withoutLayer(buildRockDynamicWeaponProxyExpectedMask(extensionObstacleLayers),
            ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY), ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
    }

    inline constexpr std::uint64_t buildRockDynamicWorldCarExpectedMask(
        std::uint64_t nativeSourceMask,
        std::uint32_t nativeSourceLayer)
    {
        if (nativeSourceLayer != FO4_LAYER_CLUTTER && nativeSourceLayer != FO4_LAYER_CLUTTER_LARGE) {
            return 0;
        }

        /*
         * The source clutter rows have already been touched symmetrically by
         * ROCK's generated hand/weapon/body policies. Preserve only vanilla
         * row semantics here; inheriting those generated bits would make the
         * body colliders push cars.
         */
        std::uint64_t mask = nativeSourceMask & allVanillaConfiguredLayerBits();
        mask = withoutLayer(mask, ROCK_LAYER_HAND);
        mask = withoutLayer(mask, ROCK_LAYER_WEAPON);
        mask = withoutLayer(mask, ROCK_LAYER_BODY);
        mask = withoutLayer(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER);
        mask = withoutLayer(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER);
        mask = withoutLayer(mask, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
        mask = withoutLayer(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        mask = withoutLayer(mask, ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY);
        mask = withoutLayer(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);

        const auto sameCarLayer = dynamicWorldCarLayerForNativeLayer(nativeSourceLayer);
        const auto peerNativeLayer = nativeSourceLayer == FO4_LAYER_CLUTTER ? FO4_LAYER_CLUTTER_LARGE : FO4_LAYER_CLUTTER;
        const auto peerCarLayer = dynamicWorldCarLayerForNativeLayer(peerNativeLayer);
        if (maskEnablesLayer(nativeSourceMask, nativeSourceLayer)) {
            mask = withLayer(mask, sameCarLayer);
        }
        if (maskEnablesLayer(nativeSourceMask, peerNativeLayer)) {
            mask = withLayer(mask, peerCarLayer);
        }

        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY);
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY);
        mask = withLayer(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        mask = withLayer(mask, ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY);
        mask = withLayer(mask, FO4_LAYER_CHARCONTROLLER);
        return mask;
    }

    inline void applyRockDynamicHandProxyLayerPolicies(
        std::uint64_t* matrix,
        std::uint64_t extensionObstacleLayers = 0,
        bool retiredWeaponHandCollision = false)
    {
        applyLayerExpectedMask(
            matrix,
            ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY,
            buildRockDynamicHandProxyExpectedMask(false, extensionObstacleLayers, retiredWeaponHandCollision));
        applyLayerExpectedMask(
            matrix,
            ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY,
            buildRockDynamicHandProxyExpectedMask(true, extensionObstacleLayers, retiredWeaponHandCollision));
    }

    inline void applyRockDynamicWeaponProxyLayerPolicy(
        std::uint64_t* matrix,
        std::uint64_t extensionObstacleLayers = 0)
    {
        applyLayerExpectedMask(
            matrix,
            ROCK_LAYER_DYNAMIC_WEAPON_PROXY,
            buildRockDynamicWeaponProxyExpectedMask(extensionObstacleLayers));
        applyLayerExpectedMask(
            matrix,
            ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY,
            buildRockSecondaryDynamicWeaponProxyExpectedMask(extensionObstacleLayers));
    }

    inline void applyRockDynamicWorldCarLayerPolicies(std::uint64_t* matrix)
    {
        if (!matrix) {
            return;
        }

        const auto nativeClutterMask = matrix[FO4_LAYER_CLUTTER];
        const auto nativeLargeClutterMask = matrix[FO4_LAYER_CLUTTER_LARGE];
        applyLayerExpectedMask(matrix,
            ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER,
            buildRockDynamicWorldCarExpectedMask(nativeClutterMask, FO4_LAYER_CLUTTER));
        applyLayerExpectedMask(matrix,
            ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER,
            buildRockDynamicWorldCarExpectedMask(nativeLargeClutterMask, FO4_LAYER_CLUTTER_LARGE));
    }

    inline void applyRockGeneratedLayerPolicies(
        std::uint64_t* matrix,
        bool handStaticWorld,
        bool weaponBlocksProjectiles,
        bool weaponBlocksSpells,
        std::uint64_t extensionObstacleLayers = 0,
        bool retiredWeaponHandCollision = false)
    {
        /*
         * Runtime registration uses one aggregate helper because layer 47 is an
         * extended ROCK-owned matrix row: body<->weapon requires both the weapon
         * row and body row to be normalized together instead of depending on a
         * later standalone body-policy call to repair symmetry.
         */
        applyRockHandLayerPolicy(matrix, handStaticWorld);
        applyLayerExpectedMask(matrix,
            ROCK_LAYER_WEAPON,
            buildRockWeaponExpectedMask(weaponBlocksProjectiles, weaponBlocksSpells, true));
        // After the 44 row (row writes are symmetric): restores 44<->53.
        applyLayerExpectedMask(matrix,
            ROCK_LAYER_SECONDARY_WEAPON,
            buildRockSecondaryWeaponExpectedMask(weaponBlocksProjectiles, weaponBlocksSpells));
        applyRockBodyLayerPolicy(matrix);
        applyRockDynamicHandProxyLayerPolicies(matrix, extensionObstacleLayers, retiredWeaponHandCollision);
        applyRockDynamicWeaponProxyLayerPolicy(matrix, extensionObstacleLayers);
        applyRockDynamicWorldCarLayerPolicies(matrix);
        applyLayerExpectedMask(matrix, ROCK_LAYER_STORED_DISPLAY,
            (std::uint64_t{1}<<ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY) | (std::uint64_t{1}<<ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY));
        applyLayerExpectedMask(matrix, ROCK_LAYER_PASSIVE_OBSTACLE, maskEnablesLayer(extensionObstacleLayers, ROCK_LAYER_PASSIVE_OBSTACLE) ?
            layerBitOrZero(ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY) |
                layerBitOrZero(ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY) |
                layerBitOrZero(ROCK_LAYER_DYNAMIC_WEAPON_PROXY) |
                layerBitOrZero(ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY) : 0);
    }
}
