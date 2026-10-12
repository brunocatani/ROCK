#pragma once

#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"

namespace rock::native_secondary_pose
{
    inline bool resolveBehaviorRootWorld(const RE::NiTransform& weaponAtIdentityRoot,
        const RE::NiTransform& target,RE::NiTransform& rootWorld) noexcept
    {
        if (!holding_grip_math::valid(holding_grip_math::fromNative(weaponAtIdentityRoot)) ||
            !holding_grip_math::valid(holding_grip_math::fromNative(target))) return false;
        rootWorld=transform_math::composeTransforms(target,transform_math::invertTransform(weaponAtIdentityRoot));
        return holding_grip_math::valid(holding_grip_math::fromNative(rootWorld));
    }

    // Weapon orientation follows the physical controller; authored contact
    // seats the unchanged physical palm. No world-space pose is frozen.
    inline bool solve(const RE::NiTransform& controller, const RE::NiTransform& physicalHand,
        const RE::NiTransform& seat, const RE::NiPoint3& palm, float modelScale,
        RE::NiTransform& weapon, RE::NiTransform& hand) noexcept
    {
        weapon = {};
        hand = {};
        if (!holding_grip_math::valid(holding_grip_math::fromNative(controller)) ||
            !holding_grip_math::valid(holding_grip_math::fromNative(physicalHand)) ||
            !holding_grip_math::valid(holding_grip_math::fromNative(seat)) ||
            !std::isfinite(palm.x) || !std::isfinite(palm.y) || !std::isfinite(palm.z) ||
            !weapon_aim_basis::tryResolveWorld(controller, modelScale, weapon)) return false;
        const auto seatPalm = transform_math::localPointToWorld(seat, palm);
        weapon.translate = {};
        weapon.translate = transform_math::localPointToWorld(physicalHand, palm) -
            transform_math::localPointToWorld(weapon, seatPalm);
        hand = transform_math::composeTransforms(weapon, seat);
        return holding_grip_math::valid(holding_grip_math::fromNative(weapon)) &&
            holding_grip_math::valid(holding_grip_math::fromNative(hand));
    }
}
