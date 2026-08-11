#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>

namespace rock::global_surface_grab_policy
{
    inline constexpr std::uint64_t kOwnerToken = 0x524F'434B'474C'4F42ULL;
    inline constexpr std::uint64_t kScopeToken = 0x5355'5246'4143'4501ULL;
    inline constexpr std::uint64_t kRightTargetId = 0x474C'4F42'5249'4748ULL;
    inline constexpr std::uint64_t kLeftTargetId = 0x474C'4F42'4C45'4654ULL;
    inline constexpr std::uint32_t kTargetGeneration = 1;

    [[nodiscard]] inline constexpr bool shouldUseFallback(
        const bool enabled,
        const bool providerMatched,
        const bool wildcardPass,
        const bool dynamicSurfaceContact,
        const std::uint32_t collisionLayer) noexcept
    {
        return enabled &&
               !providerMatched &&
               wildcardPass &&
               dynamicSurfaceContact &&
               collision_layer_policy::isDynamicHandProxySurfaceLayer(
                   collisionLayer);
    }

    [[nodiscard]] inline constexpr std::uint64_t targetIdForHand(
        const bool isLeft) noexcept
    {
        return isLeft ? kLeftTargetId : kRightTargetId;
    }

    [[nodiscard]] inline constexpr std::uint64_t allowedLayerMask() noexcept
    {
        return collision_layer_policy::
            buildRockDynamicHandProxyExpectedMask();
    }
}
