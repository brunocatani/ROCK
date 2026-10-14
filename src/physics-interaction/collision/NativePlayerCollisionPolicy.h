#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"

#include <cstdint>

namespace rock::native_player_collision
{
    inline constexpr bool isRetiredWeaponHandPair(std::uint32_t layerA, std::uint32_t layerB)
    {
        using namespace collision_layer_policy;
        return (layerA == FO4_LAYER_WEAPON && isDynamicHandInteractionLayer(layerB)) ||
               (layerB == FO4_LAYER_WEAPON && isDynamicHandInteractionLayer(layerA));
    }

    inline constexpr bool allowsRetiredWeaponHandContact(
        physics_body_classifier::BodyMotionType motion, bool looseWeapon)
    {
        using physics_body_classifier::BodyMotionType;
        // Anchoring preserves native static parts beside the keyframed bodies.
        // Active grab/pull preparation converts movable bodies back to dynamic.
        return looseWeapon && (motion == BodyMotionType::Keyframed || motion == BodyMotionType::Static);
    }

    // FO4VR proxy ctor 1E4B268/1E4B28C, dtor 1E4B3E3/1E4B4C0,
    // and processConstraintsCallback 1E4B83E use this same base adjustment.
    // The actor stores the controller interface; the callback receives the
    // preceding listener base. A direct pointer comparison never identifies it.
    inline constexpr std::uintptr_t kProxyListenerControllerOffset = 0x10;

    inline constexpr bool proxyListenerMatchesPlayer(std::uintptr_t listener, std::uintptr_t playerController)
    {
        return listener != 0 && playerController > listener &&
               playerController - listener == kProxyListenerControllerOffset;
    }

    struct BodyIdentity
    {
        std::uint32_t bodyId{ 0x7FFF'FFFFu };
        std::uint32_t motionIndex{ 0 };
        std::uintptr_t collisionObject{ 0 };
        std::uintptr_t ownerNode{ 0 };

        bool operator==(const BodyIdentity&) const = default;
    };

    inline bool matchesLiveBody(const BodyIdentity& expected, const BodyIdentity& live)
    {
        return expected.bodyId != 0x7FFF'FFFFu && expected.collisionObject != 0 &&
               expected.ownerNode != 0 && expected == live;
    }

    inline bool suppressAuthoredHandWorldPair(const BodyIdentity& expected,
        const BodyIdentity& live, std::uint32_t otherLayer)
    {
        return matchesLiveBody(expected, live) && collision_layer_policy::isWorldSurfaceLayer(otherLayer);
    }

    // These are duplicate physical obstacles for the native player skeleton.
    // ROCK's generated bodies and the native controller own world response.
    // Weapon, projectile, spell, actor and query layers deliberately stay native.
    inline constexpr bool isDuplicatePhysicalContactLayer(std::uint32_t layer)
    {
        using namespace collision_layer_policy;
        return isWorldSurfaceLayer(layer) ||
               layer == FO4_LAYER_CLUTTER || layer == FO4_LAYER_DEBRIS_SMALL ||
               layer == FO4_LAYER_DEBRIS_LARGE || layer == FO4_LAYER_SHELLCASING ||
               layer == FO4_LAYER_CLUTTER_LARGE || layer == FO4_LAYER_PROPS ||
               layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON ||
               layer == ROCK_LAYER_BODY || layer == ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY ||
               layer == ROCK_LAYER_SECONDARY_WEAPON ||
               layer == ROCK_LAYER_SECONDARY_DYNAMIC_WEAPON_PROXY;
    }

    inline constexpr bool suppressPhysicalPair(bool playerA, bool playerB,
        std::uint32_t layerA, std::uint32_t layerB, bool looseWeaponContact = false)
    {
        return (playerA && playerB) ||
               (playerA && isDuplicatePhysicalContactLayer(layerB)) ||
               (playerB && isDuplicatePhysicalContactLayer(layerA)) ||
               (looseWeaponContact &&
                   ((playerA && layerB == collision_layer_policy::FO4_LAYER_WEAPON) ||
                       (playerB && layerA == collision_layer_policy::FO4_LAYER_WEAPON)));
    }

    /*
     * Explodable cars are heavy world obstacles on the native clutter rows.
     * The dynamic hand and weapon proxies stop against them; ROCK's keyframed
     * hands, weapon hulls and body colliders have infinite mass and would
     * shove them, so they pass through. A car a hand holds (power armor) is an
     * ordinary held object: the proxies pass through it and the keyframed
     * rows keep their native pairs.
     */
    enum class CarRowPairKind : std::uint8_t { None, Proxy, Keyframed };

    struct CarRowPair
    {
        CarRowPairKind kind = CarRowPairKind::None;
        bool carIsA = false;
    };

    inline constexpr CarRowPairKind carRowPeerKind(std::uint32_t layer)
    {
        using namespace collision_layer_policy;
        if (isDynamicHandInteractionLayer(layer) || isDynamicWeaponProxyLayer(layer)) return CarRowPairKind::Proxy;
        if (layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON || layer == ROCK_LAYER_BODY ||
            layer == ROCK_LAYER_SECONDARY_WEAPON) return CarRowPairKind::Keyframed;
        return CarRowPairKind::None;
    }

    // A pair of a car-row body with one of ROCK's generated rows, and which
    // side the car-row body is.
    inline constexpr CarRowPair classifyCarRowPair(std::uint32_t layerA, std::uint32_t layerB)
    {
        using namespace collision_layer_policy;
        if (isCarRowLayer(layerA)) return { carRowPeerKind(layerB), true };
        if (isCarRowLayer(layerB)) return { carRowPeerKind(layerA), false };
        return {};
    }

    inline constexpr bool suppressCarRowContact(CarRowPairKind kind, bool explodableCar, bool held)
    {
        switch (kind) {
        case CarRowPairKind::Proxy:
            return !explodableCar || held;
        case CarRowPairKind::Keyframed:
            return explodableCar && !held;
        default:
            return false;
        }
    }

    /*
     * A hand that owns the equipped weapon (firing hand, part or support grip,
     * or just after it dropped the weapon) must not become a second physical
     * owner of that weapon. Only its pairs with weapon bodies are removed:
     * ROCK weapon hulls and dynamic weapon proxies, plus native-layer weapons
     * right after a drop, when the dropped weapon is a native world body.
     * Props, world and body contacts are kept.
     */
    inline constexpr bool suppressWeaponHandContact(std::uint32_t otherLayer, bool includeLooseWeapons)
    {
        using namespace collision_layer_policy;
        return otherLayer == ROCK_LAYER_WEAPON || otherLayer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY ||
               (includeLooseWeapons && otherLayer == FO4_LAYER_WEAPON);
    }

    inline constexpr bool isNativeWeaponSelfContactCandidate(std::uint32_t layerA, std::uint32_t layerB)
    {
        using namespace collision_layer_policy;
        return (layerA == FO4_LAYER_WEAPON && isRockGeneratedColliderLayer(layerB)) ||
               (layerB == FO4_LAYER_WEAPON && isRockGeneratedColliderLayer(layerA));
    }

    // Native equipped-weapon contacts against generated player geometry duplicate
    // ROCK's physical response. Positive player ownership is required; NPC attacks,
    // dropped weapons, world/NPC targets and unknown owners retain native admission.
    inline constexpr bool suppressNativeWeaponSelfContact(std::uint32_t layerA, std::uint32_t layerB,
        bool weaponOwnedByPlayer)
    {
        return weaponOwnedByPlayer && isNativeWeaponSelfContactCandidate(layerA, layerB);
    }

    struct BodyPair
    {
        std::uint32_t bodyA;
        std::uint32_t bodyB;
    };
    static_assert(sizeof(BodyPair) == 8);

    // One controlled equipped tool may exclude one exact target. The extension
    // chooses the target; identity and layer checks prevent recycled admission.
    struct ToolContactExclusion
    {
        std::uintptr_t world{ 0 };
        BodyIdentity weapon{};
        BodyIdentity target{};
        std::uint32_t targetLayer{ 0xFFFF'FFFFu };

        bool valid() const
        {
            return world != 0 && weapon.bodyId != target.bodyId &&
                matchesLiveBody(weapon, weapon) && matchesLiveBody(target, target);
        }

        bool matchesIds(const BodyPair& pair) const
        {
            return (pair.bodyA == weapon.bodyId && pair.bodyB == target.bodyId) ||
                (pair.bodyB == weapon.bodyId && pair.bodyA == target.bodyId);
        }

        bool suppresses(std::uintptr_t liveWorld, const BodyIdentity& bodyA,
            const BodyIdentity& bodyB, std::uint32_t layerA, std::uint32_t layerB) const
        {
            using namespace collision_layer_policy;
            if (!valid() || liveWorld != world || !matchesIds({ bodyA.bodyId, bodyB.bodyId })) return false;
            const bool weaponIsA = bodyA.bodyId == weapon.bodyId;
            return matchesLiveBody(weapon, weaponIsA ? bodyA : bodyB) &&
                matchesLiveBody(target, weaponIsA ? bodyB : bodyA) &&
                (weaponIsA ? layerA : layerB) == ROCK_LAYER_DYNAMIC_WEAPON_PROXY &&
                (weaponIsA ? layerB : layerA) == targetLayer;
        }
    };

    // Only the engine's admitted simulation-pair prefix is compacted. Query
    // filters never call this function and no body's filter bits are changed.
    template <class ShouldSuppress>
    int filterPhysicalPairs(BodyPair* pairs, int count, ShouldSuppress&& suppress)
    {
        if (!pairs || count <= 0) {
            return count;
        }
        int kept = 0;
        for (int i = 0; i < count; ++i) {
            if (!suppress(pairs[i])) {
                pairs[kept++] = pairs[i];
            }
        }
        return kept;
    }
}
