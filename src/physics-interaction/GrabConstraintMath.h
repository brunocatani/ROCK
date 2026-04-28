#pragma once

#include "TransformMath.h"

namespace frik::rock::grab_constraint_math
{
    /*
     * HIGGS creates grab angular constraints in the object-to-hand frame:
     * transform B and target_bRca both receive inverse(desired body in hand).
     * ROCK's linear pivots can still line up when only the runtime target uses
     * that inverse frame, but the ragdoll angular atom then starts from a
     * different local transform and solves against stale frame state. Keeping
     * the conversion here makes constraint creation and per-frame target writes
     * share the same convention.
     */

    template <class Matrix>
    inline Matrix desiredBodyToHandRotation(const Matrix& desiredBodyTransformHandSpaceRotation)
    {
        return transform_math::transposeRotation(desiredBodyTransformHandSpaceRotation);
    }

    template <class Matrix>
    inline void writeHavokRotationColumns(float* target, const Matrix& rotation)
    {
        if (!target) {
            return;
        }

        target[0] = rotation.entry[0][0];
        target[1] = rotation.entry[1][0];
        target[2] = rotation.entry[2][0];
        target[3] = 0.0f;

        target[4] = rotation.entry[0][1];
        target[5] = rotation.entry[1][1];
        target[6] = rotation.entry[2][1];
        target[7] = 0.0f;

        target[8] = rotation.entry[0][2];
        target[9] = rotation.entry[1][2];
        target[10] = rotation.entry[2][2];
        target[11] = 0.0f;
    }

    template <class Transform>
    inline void writeInitialGrabAngularFrame(float* transformBRotation, float* targetBRca, const Transform& desiredBodyTransformHandSpace)
    {
        const auto bodyToHandRotation = desiredBodyToHandRotation(desiredBodyTransformHandSpace.rotate);
        writeHavokRotationColumns(transformBRotation, bodyToHandRotation);
        writeHavokRotationColumns(targetBRca, bodyToHandRotation);
    }
}
