#pragma once

#include "TransformMath.h"

#include <algorithm>
#include <cmath>

namespace frik::rock::held_player_space_math
{
    /*
     * HIGGS treats smooth/snap turning as a room-space warp instead of a large
     * velocity. ROCK keeps that convention in a pure helper first: the runtime
     * can either write the verified FO4VR body transform or only log the warp
     * decision, but the math for preserving the held body's room-local pose stays
     * independent of native Havok calls.
     */

    template <class Matrix>
    inline float rotationDeltaDegrees(const Matrix& previous, const Matrix& current)
    {
        const float trace =
            previous.entry[0][0] * current.entry[0][0] + previous.entry[0][1] * current.entry[0][1] + previous.entry[0][2] * current.entry[0][2] +
            previous.entry[1][0] * current.entry[1][0] + previous.entry[1][1] * current.entry[1][1] + previous.entry[1][2] * current.entry[1][2] +
            previous.entry[2][0] * current.entry[2][0] + previous.entry[2][1] * current.entry[2][1] + previous.entry[2][2] * current.entry[2][2];
        const float cosAngle = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosAngle) * (180.0f / 3.14159265358979323846f);
    }

    template <class Matrix>
    inline bool shouldWarpPlayerSpaceRotation(const Matrix& previous, const Matrix& current, float minRotationDegrees)
    {
        if (!std::isfinite(minRotationDegrees) || minRotationDegrees <= 0.0f) {
            return false;
        }
        return rotationDeltaDegrees(previous, current) > minRotationDegrees;
    }

    inline constexpr bool shouldApplyRuntimeTransformWarp(bool enabled, bool diagnosticWarp, bool hasWarpTransforms)
    {
        return enabled && diagnosticWarp && hasWarpTransforms;
    }

    template <class Transform>
    inline Transform warpBodyWorldThroughPlayerSpace(
        const Transform& previousPlayerSpaceWorld,
        const Transform& currentPlayerSpaceWorld,
        const Transform& bodyWorld)
    {
        const Transform bodyInPreviousPlayerSpace =
            transform_math::composeTransforms(transform_math::invertTransform(previousPlayerSpaceWorld), bodyWorld);
        return transform_math::composeTransforms(currentPlayerSpaceWorld, bodyInPreviousPlayerSpace);
    }
}
