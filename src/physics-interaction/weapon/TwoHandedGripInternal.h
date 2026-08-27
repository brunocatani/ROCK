#pragma once

#include "physics-interaction/weapon/TwoHandedGrip.h"

namespace rock::two_handed_grip_internal
{
    inline constexpr std::uint32_t SCOPE_TRANSITION_TRACE_FRAMES = 6;

    [[nodiscard]] bool isFiniteTransform(const RE::NiTransform& transform);
    [[nodiscard]] bool tryGetRootFlattenedHandBoneTransform(
        bool isLeft,
        RE::NiTransform& outTransform);
}
