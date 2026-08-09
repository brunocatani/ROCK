#pragma once

#include "physics-interaction/TransformMath.h"

namespace rock::weapon_recapture_frame_policy
{
    template <class Transform>
    [[nodiscard]] inline Transform makePostUndrawFrameCorrection(
        const Transform& authoritativeSourceInWeapon,
        const Transform& currentSourceInWeapon)
    {
        return transform_math::composeTransforms(
            authoritativeSourceInWeapon,
            transform_math::invertTransform(currentSourceInWeapon));
    }

    template <class Transform>
    [[nodiscard]] inline Transform applyPostUndrawFrameCorrection(
        const Transform& correction,
        const Transform& currentSourceInWeapon)
    {
        return transform_math::composeTransforms(correction, currentSourceInWeapon);
    }
}
