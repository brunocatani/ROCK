#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>

namespace rock::native_player_collision
{
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

    struct BodyPair
    {
        std::uint32_t bodyA;
        std::uint32_t bodyB;
    };
    static_assert(sizeof(BodyPair) == 8);

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
