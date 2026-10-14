#pragma once

#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

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

    // The firing hand's weapon orientation in its wand, as for the first
    // weapon: ROCK's controller basis, which the left hand mirrors laterally
    // and trims by the player's left aim trim.
    inline RE::NiTransform firingWeaponInWand(bool isLeft, float leftYawDegrees, float leftPitchDegrees) noexcept
    {
        const auto basis = weapon_aim_basis::weaponInController();
        if (!isLeft) return basis;
        return transform_math::composeTransforms(
            left_firing_position_only_math::leftAimTrim<RE::NiTransform>(leftYawDegrees, leftPitchDegrees),
            left_firing_position_only_math::mirrorRightWeaponInWandOrientation(basis));
    }

    // Weapon orientation follows the controller through weaponInWand;
    // authored contact seats the physical palm, shifted by handTargetOffset
    // in hand space (the left firing grip offset). No world pose is frozen.
    inline bool solve(const RE::NiTransform& controller, const RE::NiTransform& physicalHand,
        const RE::NiTransform& seat, const RE::NiPoint3& palm, float modelScale,
        const RE::NiTransform& weaponInWand, const RE::NiPoint3& handTargetOffset,
        RE::NiTransform& weapon, RE::NiTransform& hand) noexcept
    {
        weapon = {};
        hand = {};
        if (!holding_grip_math::valid(holding_grip_math::fromNative(controller)) ||
            !holding_grip_math::valid(holding_grip_math::fromNative(physicalHand)) ||
            !holding_grip_math::valid(holding_grip_math::fromNative(seat)) ||
            !std::isfinite(palm.x) || !std::isfinite(palm.y) || !std::isfinite(palm.z) ||
            !std::isfinite(handTargetOffset.x) || !std::isfinite(handTargetOffset.y) || !std::isfinite(handTargetOffset.z) ||
            !weapon_aim_basis::tryResolveWorld(controller, modelScale, weaponInWand, weapon)) return false;
        const auto seatPalm = transform_math::localPointToWorld(seat, palm);
        weapon.translate = {};
        weapon.translate = transform_math::localPointToWorld(physicalHand, palm + handTargetOffset) -
            transform_math::localPointToWorld(weapon, seatPalm);
        hand = transform_math::composeTransforms(weapon, seat);
        return holding_grip_math::valid(holding_grip_math::fromNative(weapon)) &&
            holding_grip_math::valid(holding_grip_math::fromNative(hand));
    }
}
