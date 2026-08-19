#pragma once

#include <algorithm>
#include <cmath>

namespace rock::touch_grab_math
{
    struct Vector3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    [[nodiscard]] inline constexpr Vector3 operator-(
        const Vector3& left,
        const Vector3& right)
    {
        return {
            left.x - right.x,
            left.y - right.y,
            left.z - right.z
        };
    }

    [[nodiscard]] inline constexpr float dot(
        const Vector3& left,
        const Vector3& right)
    {
        return left.x * right.x +
               left.y * right.y +
               left.z * right.z;
    }

    [[nodiscard]] inline constexpr Vector3 cross(
        const Vector3& left,
        const Vector3& right)
    {
        return {
            left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x
        };
    }

    [[nodiscard]] inline bool normalize(
        const Vector3& value,
        Vector3& outNormalized)
    {
        const float lengthSquared = dot(value, value);
        if (!std::isfinite(lengthSquared) ||
            lengthSquared <= 1.0e-8f) {
            outNormalized = {};
            return false;
        }
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        outNormalized = {
            value.x * inverseLength,
            value.y * inverseLength,
            value.z * inverseLength
        };
        return true;
    }

    [[nodiscard]] inline bool makePerpendicularWitness(
        const Vector3& axisValue,
        Vector3& outWitness)
    {
        Vector3 axis{};
        if (!normalize(axisValue, axis)) {
            return false;
        }
        const Vector3 basis =
            std::abs(axis.z) < 0.75f ?
            Vector3{ 0.0f, 0.0f, 1.0f } :
            Vector3{ 0.0f, 1.0f, 0.0f };
        return normalize(cross(axis, basis), outWitness);
    }

    [[nodiscard]] inline float signedAngle(
        const Vector3& fromValue,
        const Vector3& toValue,
        const Vector3& axisValue)
    {
        Vector3 from{};
        Vector3 to{};
        Vector3 axis{};
        if (!normalize(fromValue, from) ||
            !normalize(toValue, to) ||
            !normalize(axisValue, axis)) {
            return 0.0f;
        }
        const float sine = dot(axis, cross(from, to));
        const float cosine = std::clamp(dot(from, to), -1.0f, 1.0f);
        return std::atan2(sine, cosine);
    }

    [[nodiscard]] inline float hingeCoordinate(
        const float initialCoordinate,
        const Vector3& initialWitnessWorld,
        const Vector3& currentWitnessWorld,
        const Vector3& axisWorld)
    {
        return initialCoordinate +
               signedAngle(
                   initialWitnessWorld,
                   currentWitnessWorld,
                   axisWorld);
    }

    [[nodiscard]] inline float prismaticCoordinate(
        const float initialCoordinate,
        const Vector3& initialOriginWorld,
        const Vector3& currentOriginWorld,
        const Vector3& axisWorld)
    {
        Vector3 axis{};
        if (!normalize(axisWorld, axis)) {
            return initialCoordinate;
        }
        return initialCoordinate +
               dot(currentOriginWorld - initialOriginWorld, axis);
    }
}
