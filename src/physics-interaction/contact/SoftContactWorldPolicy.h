#pragma once

/*
 * World soft contact stays query-driven because ROCK's generated hand layers
 * deliberately do not physically collide with static geometry yet. This keeps
 * swept-sphere world evidence available while avoiding native collision matrix
 * changes until the real static collision stack is explicitly enabled.
 */

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>

namespace rock::soft_contact_world_policy
{
    inline constexpr std::uint32_t kFilterLayerMask = 0x7Fu;
    inline constexpr std::uint32_t kInvalidWorldBodyId = 0x7FFF'FFFFu;

    inline constexpr std::uint32_t layerFromFilterInfo(std::uint32_t filterInfo)
    {
        return filterInfo & kFilterLayerMask;
    }

    inline constexpr bool acceptsWorldSurfaceLayer(std::uint32_t layer)
    {
        return collision_layer_policy::isWorldSurfaceLayer(layer);
    }

    inline constexpr bool acceptsWorldSurfaceFilterInfo(std::uint32_t filterInfo)
    {
        return acceptsWorldSurfaceLayer(layerFromFilterInfo(filterInfo));
    }

    inline constexpr bool rejectsWorldSurfaceLayer(std::uint32_t layer)
    {
        return !acceptsWorldSurfaceLayer(layer);
    }
}
