#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace rock::impact_audio_policy
{
    [[nodiscard]] constexpr bool muteShellPair(std::uint32_t a, std::uint32_t b) noexcept
    {
        using namespace collision_layer_policy;
        return (a == FO4_LAYER_SHELLCASING && isRockGeneratedColliderLayer(b)) ||
            (b == FO4_LAYER_SHELLCASING && isRockGeneratedColliderLayer(a));
    }
}
