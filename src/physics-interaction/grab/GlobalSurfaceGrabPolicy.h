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

    struct FallbackContext
    {
        bool enabled = false;
        bool providerMatched = false;
        bool wildcardPass = false;
        bool dynamicSurfaceContact = false;
        bool closeObjectCandidate = false;
        std::uint32_t collisionLayer = 0xFFFF'FFFFu;
    };

    [[nodiscard]] inline constexpr bool shouldUseFallback(
        const FallbackContext& context) noexcept
    {
        return context.enabled &&
               !context.providerMatched &&
               !context.closeObjectCandidate &&
               context.wildcardPass &&
               context.dynamicSurfaceContact &&
               collision_layer_policy::isDynamicHandProxySurfaceLayer(
                   context.collisionLayer);
    }

    /*
     * A built-in FixedAnchor only follows the target body's live transform; it
     * never changes motion properties, velocities, activation, or constraints
     * on that body. That makes noncanonical Havok motion-property handles safe
     * for this one path. Provider mechanisms still require a classified motion
     * type because their creation and restoration mutate target state.
     */
    [[nodiscard]] inline constexpr bool canFollowUnclassifiedMotion(
        const bool globalSurfaceFallback,
        const bool fixedAnchor) noexcept
    {
        return globalSurfaceFallback && fixedAnchor;
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
