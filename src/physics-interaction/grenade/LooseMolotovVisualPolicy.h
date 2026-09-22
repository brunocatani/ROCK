#pragma once

#include "physics-interaction/TransformMath.h"

#include <cmath>
#include <cstdint>
#include <string_view>

namespace rock::loose_molotov_visual_policy
{
    inline constexpr char kVisualShapeName[] = "ROCK_ArmedMolotovFlame";

    [[nodiscard]] constexpr bool isVisualOnlyShape(const char* name) noexcept
    {
        return name && std::string_view(name) == kVisualShapeName;
    }

    // Both vanilla models use the same wick vertices in different local frames.
    // Matching topology and bounds admits that geometry without guessing the tip
    // of an unrelated replacement mesh from its name alone.
    template <class Bound>
    [[nodiscard]] bool compatibleWick(std::uint32_t vertices, std::uint32_t triangles,
        const Bound& candidate, const Bound& authored) noexcept
    {
        constexpr float tolerance = 0.01f;
        return vertices == 126 && triangles == 172 &&
            std::isfinite(candidate.fRadius) && candidate.fRadius > 0.0f &&
            std::abs(candidate.fRadius - authored.fRadius) <= tolerance &&
            std::abs(candidate.center.x - authored.center.x) <= tolerance &&
            std::abs(candidate.center.y - authored.center.y) <= tolerance &&
            std::abs(candidate.center.z - authored.center.z) <= tolerance;
    }

    template <class Transform>
    [[nodiscard]] Transform atWick(const Transform& destinationWick,
        const Transform& sourceWick, const Transform& sourceFlame) noexcept
    {
        return transform_math::composeTransforms(destinationWick,
            transform_math::composeTransforms(transform_math::invertTransform(sourceWick), sourceFlame));
    }
}
