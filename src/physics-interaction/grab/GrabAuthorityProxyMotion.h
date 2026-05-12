#pragma once

/*
 * Hidden grab authority proxies are keyframed solver anchors, not contact
 * evidence. Their target transform and velocity must describe the same
 * root-flattened palm motion; otherwise the body slot can read back at the
 * right pose while the solver receives stale or zero angular motion.
 */

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cmath>

namespace rock::grab_authority_proxy_motion
{
    inline void computeLinearVelocityHavok(
        const RE::NiTransform& previous,
        const RE::NiTransform& current,
        float deltaSeconds,
        float gameToHavokScale,
        float outVelocity[4])
    {
        outVelocity[0] = 0.0f;
        outVelocity[1] = 0.0f;
        outVelocity[2] = 0.0f;
        outVelocity[3] = 0.0f;
        if (!havok_physics_timing::isUsableDelta(deltaSeconds)) {
            return;
        }

        const float scale = gameToHavokScale / deltaSeconds;
        outVelocity[0] = (current.translate.x - previous.translate.x) * scale;
        outVelocity[1] = (current.translate.y - previous.translate.y) * scale;
        outVelocity[2] = (current.translate.z - previous.translate.z) * scale;
    }

    inline RE::NiPoint3 matrixColumn(const RE::NiMatrix3& matrix, int column)
    {
        return RE::NiPoint3{ matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column] };
    }

    inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
    {
        const float squareLength = lengthSquared(value);
        if (squareLength <= 1.0e-8f || !std::isfinite(squareLength)) {
            return {};
        }

        const float inverseLength = 1.0f / std::sqrt(squareLength);
        return RE::NiPoint3{ value.x * inverseLength, value.y * inverseLength, value.z * inverseLength };
    }

    inline float matrixDot(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs)
    {
        return lhs.entry[0][0] * rhs.entry[0][0] + lhs.entry[0][1] * rhs.entry[0][1] + lhs.entry[0][2] * rhs.entry[0][2] +
               lhs.entry[1][0] * rhs.entry[1][0] + lhs.entry[1][1] * rhs.entry[1][1] + lhs.entry[1][2] * rhs.entry[1][2] +
               lhs.entry[2][0] * rhs.entry[2][0] + lhs.entry[2][1] * rhs.entry[2][1] + lhs.entry[2][2] * rhs.entry[2][2];
    }

    inline void computeAngularVelocityRadiansPerSecond(
        const RE::NiTransform& previous,
        const RE::NiTransform& current,
        float deltaSeconds,
        float outVelocity[4])
    {
        outVelocity[0] = 0.0f;
        outVelocity[1] = 0.0f;
        outVelocity[2] = 0.0f;
        outVelocity[3] = 0.0f;
        if (!havok_physics_timing::isUsableDelta(deltaSeconds)) {
            return;
        }

        RE::NiPoint3 axisSum{};
        for (int column = 0; column < 3; ++column) {
            axisSum = axisSum + cross(matrixColumn(previous.rotate, column), matrixColumn(current.rotate, column));
        }

        const RE::NiPoint3 axis = normalizeOrZero(axisSum);
        if (lengthSquared(axis) <= 1.0e-8f) {
            return;
        }

        const float angle = std::acos(std::clamp((matrixDot(previous.rotate, current.rotate) - 1.0f) * 0.5f, -1.0f, 1.0f));
        if (!std::isfinite(angle) || angle <= 1.0e-6f) {
            return;
        }

        const float inverseDelta = 1.0f / deltaSeconds;
        outVelocity[0] = axis.x * angle * inverseDelta;
        outVelocity[1] = axis.y * angle * inverseDelta;
        outVelocity[2] = axis.z * angle * inverseDelta;
    }
}
