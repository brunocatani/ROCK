#pragma once

#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "RE/NetImmerse/NiPoint.h"

#include <array>
#include <cstdint>

namespace rock
{
    struct RockConfigValues;
}

namespace rock::collider_tuning
{
    struct HandRole
    {
        float radiusScale = 1.0f;
        RE::NiPoint3 palmDimensionScale{ 1.0f, 1.0f, 1.0f };
    };

    struct HandProfile
    {
        std::uint64_t signature = 0;
        std::array<HandRole, hand_collider_semantics::kHandColliderBodyCountPerHand> roles{};
    };

    // Pure value preparation. Owners call these only when config revision or
    // armor profile changes; the existing precedence and signatures are kept.
    HandProfile prepareHand(const RockConfigValues& config, bool powerArmor);
}
