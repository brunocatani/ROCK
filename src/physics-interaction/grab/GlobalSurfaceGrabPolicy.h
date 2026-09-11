#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>

namespace rock::global_surface_grab_policy
{
    inline constexpr std::uint64_t kOwnerToken = 0x524F'434B'474C'4F42ULL;
    inline constexpr std::uint64_t kScopeToken = 0x5355'5246'4143'4501ULL;
    inline constexpr std::uint64_t kTargetIdPrefix = 0x474C'4F42'0000'0000ULL;
    inline constexpr std::uint32_t kTargetGeneration = 1;

    struct FallbackContext
    {
        bool enabled = false;
        bool providerMatched = false;
        bool wildcardPass = false;
        bool dynamicSurfaceContact = false;
        std::uint32_t collisionLayer = 0xFFFF'FFFFu;
    };

    [[nodiscard]] inline constexpr bool shouldUseFallback(
        const FallbackContext& context) noexcept
    {
        return context.enabled &&
               !context.providerMatched &&
               context.wildcardPass &&
               context.dynamicSurfaceContact &&
               collision_layer_policy::isDynamicHandProxySurfaceLayer(
                   context.collisionLayer);
    }

    [[nodiscard]] inline constexpr bool shouldYieldToCloseObject(
        const bool wildcardTarget,
        const bool closeObjectCandidate) noexcept
    {
        return wildcardTarget && closeObjectCandidate;
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

    // Either hand may leave a shared target and grab a different body while
    // its peer retains the original. Target identity therefore belongs to the body.
    [[nodiscard]] inline constexpr std::uint64_t targetIdForBody(
        const std::uint32_t bodyId) noexcept
    {
        return kTargetIdPrefix | bodyId;
    }

    [[nodiscard]] inline constexpr bool releaseOnInput(bool commandOwned, bool held, bool released) noexcept
    {
        return released || (!held && !commandOwned);
    }

    [[nodiscard]] inline constexpr std::uint64_t allowedLayerMask() noexcept
    {
        return collision_layer_policy::
            buildRockDynamicHandProxyExpectedMask();
    }
}
