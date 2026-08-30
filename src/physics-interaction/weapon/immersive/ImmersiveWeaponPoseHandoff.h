#pragma once

#include "physics-interaction/TransformMath.h"

namespace rock::immersive_weapon_pose_handoff
{
    /*
     * Rebase a surviving part grip at the firing-hand detach boundary. With
     * the same carry-hand frame on the first solve, resolving this relation
     * reconstructs preservedWeaponWorld exactly; later solves contain only
     * post-detach hand motion.
     */
    template <class Transform>
    [[nodiscard]] inline Transform captureHandWeaponLocal(
        const Transform& preservedWeaponWorld,
        const Transform& currentCarryHandWorld)
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(preservedWeaponWorld),
            currentCarryHandWorld);
    }

    template <class Transform>
    [[nodiscard]] inline Transform resolveWeaponWorld(
        const Transform& currentCarryHandWorld,
        const Transform& handWeaponLocal)
    {
        return transform_math::composeTransforms(
            currentCarryHandWorld,
            transform_math::invertTransform(handWeaponLocal));
    }
}
