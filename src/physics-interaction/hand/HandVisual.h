#pragma once

/*
 * Visual interpolation helpers only. ROCK derives a rendered-hand target from
 * the live held object and the frozen object-in-hand transform, then lerps the
 * visible hand toward that target. This header stays outside motor authority:
 * these helpers never feed object targets, pivots, collision bodies, or rotation
 * decisions.
 */

// ---- HandVisualLerpMath.h ----

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>

namespace rock::hand_visual_lerp_math
{
    /*
     * Visual hand lerp is intentionally isolated from collision and grab
     * constraint math. The physics hand frame remains authoritative; this helper
     * only advances the rendered hand toward the solved held-object-relative
     * pose.
     */
    template <class Transform>
    struct AdvanceResult
    {
        Transform transform{};
        bool reachedTarget = true;
    };

    struct Quaternion
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
    };

    inline Quaternion normalize(Quaternion q)
    {
        const float length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (length <= 0.000001f) {
            return {};
        }
        const float inv = 1.0f / length;
        q.x *= inv;
        q.y *= inv;
        q.z *= inv;
        q.w *= inv;
        return q;
    }

    inline float dot(const Quaternion& a, const Quaternion& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    template <class Matrix>
    inline Quaternion matrixToQuaternion(const Matrix& matrix)
    {
        Quaternion q{};
        const float trace = matrix.entry[0][0] + matrix.entry[1][1] + matrix.entry[2][2];
        if (trace > 0.0f) {
            const float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (matrix.entry[2][1] - matrix.entry[1][2]) / s;
            q.y = (matrix.entry[0][2] - matrix.entry[2][0]) / s;
            q.z = (matrix.entry[1][0] - matrix.entry[0][1]) / s;
        } else if (matrix.entry[0][0] > matrix.entry[1][1] && matrix.entry[0][0] > matrix.entry[2][2]) {
            const float s = std::sqrt(1.0f + matrix.entry[0][0] - matrix.entry[1][1] - matrix.entry[2][2]) * 2.0f;
            q.w = (matrix.entry[2][1] - matrix.entry[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (matrix.entry[0][1] + matrix.entry[1][0]) / s;
            q.z = (matrix.entry[0][2] + matrix.entry[2][0]) / s;
        } else if (matrix.entry[1][1] > matrix.entry[2][2]) {
            const float s = std::sqrt(1.0f + matrix.entry[1][1] - matrix.entry[0][0] - matrix.entry[2][2]) * 2.0f;
            q.w = (matrix.entry[0][2] - matrix.entry[2][0]) / s;
            q.x = (matrix.entry[0][1] + matrix.entry[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (matrix.entry[1][2] + matrix.entry[2][1]) / s;
        } else {
            const float s = std::sqrt(1.0f + matrix.entry[2][2] - matrix.entry[0][0] - matrix.entry[1][1]) * 2.0f;
            q.w = (matrix.entry[1][0] - matrix.entry[0][1]) / s;
            q.x = (matrix.entry[0][2] + matrix.entry[2][0]) / s;
            q.y = (matrix.entry[1][2] + matrix.entry[2][1]) / s;
            q.z = 0.25f * s;
        }
        return normalize(q);
    }

    template <class Matrix>
    inline Matrix quaternionToMatrix(const Quaternion& q)
    {
        const Quaternion n = normalize(q);
        Matrix matrix{};
        matrix.entry[0][0] = 1.0f - 2.0f * (n.y * n.y + n.z * n.z);
        matrix.entry[0][1] = 2.0f * (n.x * n.y - n.w * n.z);
        matrix.entry[0][2] = 2.0f * (n.x * n.z + n.w * n.y);
        matrix.entry[1][0] = 2.0f * (n.x * n.y + n.w * n.z);
        matrix.entry[1][1] = 1.0f - 2.0f * (n.x * n.x + n.z * n.z);
        matrix.entry[1][2] = 2.0f * (n.y * n.z - n.w * n.x);
        matrix.entry[2][0] = 2.0f * (n.x * n.z - n.w * n.y);
        matrix.entry[2][1] = 2.0f * (n.y * n.z + n.w * n.x);
        matrix.entry[2][2] = 1.0f - 2.0f * (n.x * n.x + n.y * n.y);
        return matrix;
    }

    inline Quaternion slerp(Quaternion current, Quaternion target, float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        float cosTheta = dot(current, target);
        if (cosTheta < 0.0f) {
            target.x *= -1.0f;
            target.y *= -1.0f;
            target.z *= -1.0f;
            target.w *= -1.0f;
            cosTheta *= -1.0f;
        }

        if (cosTheta > 0.9995f) {
            return normalize(Quaternion{
                current.x + (target.x - current.x) * t,
                current.y + (target.y - current.y) * t,
                current.z + (target.z - current.z) * t,
                current.w + (target.w - current.w) * t,
            });
        }

        const float theta = std::acos(std::clamp(cosTheta, -1.0f, 1.0f));
        const float sinTheta = std::sin(theta);
        const float a = std::sin((1.0f - t) * theta) / sinTheta;
        const float b = std::sin(t * theta) / sinTheta;
        return normalize(Quaternion{
            current.x * a + target.x * b,
            current.y * a + target.y * b,
            current.z * a + target.z * b,
            current.w * a + target.w * b,
        });
    }

    inline float quaternionAngleRadians(const Quaternion& a, const Quaternion& b)
    {
        const float cosTheta = std::abs(dot(a, b));
        return 2.0f * std::acos(std::clamp(cosTheta, -1.0f, 1.0f));
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    template <class Vector>
    inline float distanceGameUnits(const Vector& lhs, const Vector& rhs)
    {
        const Vector delta{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        return std::sqrt(lengthSquared(delta));
    }

    inline float sanitizeNonNegative(float value, float fallback)
    {
        return std::isfinite(value) && value >= 0.0f ? value : fallback;
    }

    inline float computeDistanceMappedDurationGameUnits(float distanceGameUnits, float minSeconds, float maxSeconds, float minDistanceGameUnits, float maxDistanceGameUnits)
    {
        const float distance = sanitizeNonNegative(distanceGameUnits, 0.0f);
        const float minTime = sanitizeNonNegative(minSeconds, 0.0f);
        const float maxTime = (std::max)(minTime, sanitizeNonNegative(maxSeconds, minTime));
        const float minDistance = sanitizeNonNegative(minDistanceGameUnits, 0.0f);
        const float maxDistance = (std::max)(minDistance, sanitizeNonNegative(maxDistanceGameUnits, minDistance));
        if (distance <= minDistance || maxTime <= 0.0f) {
            return 0.0f;
        }
        if (maxDistance <= minDistance) {
            return maxTime;
        }

        const float alpha = std::clamp((distance - minDistance) / (maxDistance - minDistance), 0.0f, 1.0f);
        return minTime + (maxTime - minTime) * alpha;
    }

    inline float advanceTimedBlendElapsed(float elapsedSeconds, float deltaTime, float durationSeconds)
    {
        if (durationSeconds <= 0.0f) {
            return durationSeconds;
        }
        const float elapsed = sanitizeNonNegative(elapsedSeconds, 0.0f);
        const float delta = sanitizeNonNegative(deltaTime, 0.0f);
        return (std::min)(durationSeconds, elapsed + delta);
    }

    inline float timedBlendAlpha(float elapsedSeconds, float durationSeconds)
    {
        if (durationSeconds <= 0.0f) {
            return 1.0f;
        }
        return std::clamp(sanitizeNonNegative(elapsedSeconds, 0.0f) / durationSeconds, 0.0f, 1.0f);
    }

    template <class Transform>
    inline Transform interpolateTransform(const Transform& start, const Transform& target, float alpha)
    {
        const float t = std::clamp(alpha, 0.0f, 1.0f);
        if (t <= 0.0f) {
            return start;
        }
        if (t >= 1.0f) {
            return target;
        }

        Transform result = target;
        result.translate = {
            start.translate.x + (target.translate.x - start.translate.x) * t,
            start.translate.y + (target.translate.y - start.translate.y) * t,
            start.translate.z + (target.translate.z - start.translate.z) * t,
        };
        result.rotate = quaternionToMatrix<decltype(result.rotate)>(slerp(matrixToQuaternion(start.rotate), matrixToQuaternion(target.rotate), t));
        return result;
    }

    template <class Transform>
    inline AdvanceResult<Transform> blendTransformOverDuration(const Transform& start, const Transform& target, float elapsedSeconds, float durationSeconds)
    {
        AdvanceResult<Transform> result{};
        const float alpha = timedBlendAlpha(elapsedSeconds, durationSeconds);
        result.transform = interpolateTransform(start, target, alpha);
        result.reachedTarget = alpha >= 1.0f;
        return result;
    }

    template <class Vector>
    inline Vector advancePosition(const Vector& current, const Vector& target, float speed, float deltaTime, bool& reached)
    {
        const Vector delta{ target.x - current.x, target.y - current.y, target.z - current.z };
        const float distSq = lengthSquared(delta);
        const float step = (std::max)(0.0f, speed) * (std::max)(0.0f, deltaTime);
        if (distSq <= step * step || distSq <= 0.000001f) {
            reached = true;
            return target;
        }

        const float invDist = 1.0f / std::sqrt(distSq);
        reached = false;
        return Vector{ current.x + delta.x * invDist * step, current.y + delta.y * invDist * step, current.z + delta.z * invDist * step };
    }

    template <class Transform>
    inline Transform buildHeldObjectRelativeHandWorld(const Transform& heldObjectWorld, const Transform& frozenObjectHandSpace)
    {
        return transform_math::composeTransforms(heldObjectWorld, transform_math::invertTransform(frozenObjectHandSpace));
    }

    inline bool shouldSmoothHeldObjectRelativeHand(bool lerpEnabled, bool touchHeldPhase, bool acquisitionVisual)
    {
        return lerpEnabled && acquisitionVisual && !touchHeldPhase;
    }

    template <class Transform>
    inline AdvanceResult<Transform> advanceTransform(const Transform& current, const Transform& target, float positionSpeed, float angularSpeedDegrees, float deltaTime)
    {
        AdvanceResult<Transform> result{};
        result.transform = target;

        bool positionReached = true;
        result.transform.translate = advancePosition(current.translate, target.translate, positionSpeed, deltaTime, positionReached);

        const Quaternion currentRotation = matrixToQuaternion(current.rotate);
        const Quaternion targetRotation = matrixToQuaternion(target.rotate);
        const float angle = quaternionAngleRadians(currentRotation, targetRotation);
        const float maxAngle = (std::max)(0.0f, angularSpeedDegrees) * 0.01745329251994329577f * (std::max)(0.0f, deltaTime);
        const bool rotationReached = angle <= maxAngle || angle <= 0.000001f;
        if (!rotationReached) {
            result.transform.rotate = quaternionToMatrix<decltype(result.transform.rotate)>(slerp(currentRotation, targetRotation, maxAngle / angle));
        }

        result.reachedTarget = positionReached && rotationReached;
        return result;
    }
}
