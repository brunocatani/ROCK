#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace rock::impact_audio_policy
{
    // Car rows contain native world objects, not generated player colliders.
    [[nodiscard]] constexpr bool isGeneratedColliderLayer(std::uint32_t layer) noexcept
    {
        using namespace collision_layer_policy;
        return layer == ROCK_LAYER_HAND || layer == ROCK_LAYER_WEAPON ||
            layer == ROCK_LAYER_BODY || layer == ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY ||
            layer == ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY || layer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY;
    }

    [[nodiscard]] constexpr bool muteShellPair(std::uint32_t a, std::uint32_t b) noexcept
    {
        using namespace collision_layer_policy;
        return (a == FO4_LAYER_SHELLCASING && isGeneratedColliderLayer(b)) ||
            (b == FO4_LAYER_SHELLCASING && isGeneratedColliderLayer(a));
    }
}
