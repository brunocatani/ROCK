#pragma once

/*
 * World soft contact consumes native hand/world evidence when available and
 * falls back to swept-sphere queries over stable support/blocker layers. Dynamic
 * props stay outside this policy so visual hand stops do not fight grab, push,
 * weapon, or held-object systems.
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
