#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>

namespace rock::native_player_collision
{
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

    // These are duplicate physical obstacles for the native player skeleton.
    // ROCK's generated bodies and the native controller own world response.
    // Weapon, projectile, spell, actor and query layers deliberately stay native.
    inline constexpr bool isDuplicatePhysicalContactLayer(std::uint32_t layer)
    {
        using namespace collision_layer_policy;
        return isWorldSurfaceLayer(layer) || isDynamicWorldCarLayer(layer) ||
               layer == FO4_LAYER_CLUTTER || layer == FO4_LAYER_DEBRIS_SMALL ||
               layer == FO4_LAYER_DEBRIS_LARGE || layer == FO4_LAYER_SHELLCASING ||
               layer == FO4_LAYER_CLUTTER_LARGE || layer == FO4_LAYER_PROPS ||
               layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON ||
               layer == ROCK_LAYER_BODY || layer == ROCK_LAYER_DYNAMIC_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY ||
               layer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY;
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

    // One equipped blade may overlap one NPC BODY while its insertion guide
    // owns the weapon. Identity checks prevent a recycled ID inheriting it.
    struct BladeCollisionPair
    {
        std::uintptr_t world{ 0 };
        BodyIdentity weapon{};
        BodyIdentity target{};

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
                (weaponIsA ? layerB : layerA) == FO4_LAYER_BIPED_NO_CC;
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
