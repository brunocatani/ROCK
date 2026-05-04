#pragma once

#include "DebugAxisMath.h"
#include "TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace frik::rock::hand_bone_collider_geometry_math
{
    /*
     * These helpers convert the validated live bone graph into stable collider
     * frames. Child bone translation defines each segment direction, while bone
     * rotation supplies roll. Palm landmarks are derived from the hand node plus
     * live finger bases so the new constraint anchor follows the visual hand
     * instead of the removed legacy box.
     */

    template <class Vector>
    inline Vector makeVector(float x, float y, float z)
    {
        Vector out{};
        out.x = x;
        out.y = y;
        out.z = z;
        return out;
    }

    template <class Vector>
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return makeVector<Vector>(value.x * scale, value.y * scale, value.z * scale);
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vector>
    inline Vector normalizeOr(const Vector& value, const Vector& fallback)
    {
        const float len = length(value);
        if (len <= 1.0e-5f || !std::isfinite(len)) {
            return fallback;
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Matrix, class Vector>
    inline Matrix matrixFromAxes(const Vector& xAxis, const Vector& yAxis, const Vector& zAxis)
    {
        Matrix matrix{};
        matrix.entry[0][0] = xAxis.x;
        matrix.entry[1][0] = xAxis.y;
        matrix.entry[2][0] = xAxis.z;
        matrix.entry[0][1] = yAxis.x;
        matrix.entry[1][1] = yAxis.y;
        matrix.entry[2][1] = yAxis.z;
        matrix.entry[0][2] = zAxis.x;
        matrix.entry[1][2] = zAxis.y;
        matrix.entry[2][2] = zAxis.z;
        return matrix;
    }

    template <class Transform, class Vector>
    struct BoneColliderFrameInput
    {
        Transform start{};
        Transform end{};
        Transform previous{};
        float radius = 0.5f;
        float convexRadius = 0.1f;
        bool extrapolateFromPrevious = false;
        float extrapolatedLengthScale = 0.65f;
    };

    template <class Transform, class Vector>
    struct BoneColliderFrameResult
    {
        Transform transform{};
        Vector xAxis{};
        Vector yAxis{};
        Vector zAxis{};
        Vector backAxis{};
        float length = 0.0f;
        float radius = 0.0f;
        float convexRadius = 0.0f;
        bool valid = false;
    };

    template <class Transform, class Vector>
    inline BoneColliderFrameResult<Transform, Vector> buildSegmentColliderFrame(const BoneColliderFrameInput<Transform, Vector>& input)
    {
        BoneColliderFrameResult<Transform, Vector> result{};
        result.radius = (std::max)(0.01f, input.radius);
        result.convexRadius = (std::max)(0.0f, input.convexRadius);

        Vector start = input.start.translate;
        Vector end = input.end.translate;
        if (input.extrapolateFromPrevious) {
            const Vector parentToTip = sub(input.start.translate, input.previous.translate);
            const float parentLength = length(parentToTip);
            if (parentLength <= 1.0e-5f) {
                return result;
            }
            end = add(input.start.translate, mul(parentToTip, input.extrapolatedLengthScale));
        }

        const Vector segment = sub(end, start);
        result.length = length(segment);
        if (result.length <= 1.0e-5f) {
            return result;
        }

        const Vector fallbackX = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector fallbackY = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector fallbackZ = makeVector<Vector>(0.0f, 0.0f, 1.0f);
        result.xAxis = normalizeOr(segment, fallbackX);

        Vector rollHint = debug_axis_math::rotateNiLocalToWorld(input.start.rotate, fallbackY);
        rollHint = projectOntoPlane(rollHint, result.xAxis);
        result.yAxis = normalizeOr(rollHint, std::fabs(dot(result.xAxis, fallbackY)) < 0.9f ? fallbackY : fallbackZ);
        result.zAxis = normalizeOr(cross(result.xAxis, result.yAxis), fallbackZ);
        result.yAxis = normalizeOr(cross(result.zAxis, result.xAxis), fallbackY);

        result.transform = input.start;
        result.transform.translate = add(start, mul(segment, 0.5f));
        result.transform.rotate = matrixFromAxes<decltype(result.transform.rotate)>(result.xAxis, result.yAxis, result.zAxis);
        result.transform.scale = 1.0f;
        result.valid = true;
        return result;
    }

    template <class Vector>
    inline std::vector<Vector> makeCapsuleLikeHullPoints(float lengthValue, float radius)
    {
        const float half = (std::max)(0.05f, lengthValue) * 0.5f;
        const float r = (std::max)(0.01f, radius);
        std::vector<Vector> points;
        points.reserve(16);
        for (float x : { -half, half }) {
            points.push_back(makeVector<Vector>(x, r, 0.0f));
            points.push_back(makeVector<Vector>(x, -r, 0.0f));
            points.push_back(makeVector<Vector>(x, 0.0f, r));
            points.push_back(makeVector<Vector>(x, 0.0f, -r));
            points.push_back(makeVector<Vector>(x, r * 0.7071067f, r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, -r * 0.7071067f, r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, r * 0.7071067f, -r * 0.7071067f));
            points.push_back(makeVector<Vector>(x, -r * 0.7071067f, -r * 0.7071067f));
        }
        return points;
    }

    template <class Vector>
    inline std::vector<Vector> makeRoundedBoxHullPoints(float lengthValue, float widthValue, float thicknessValue)
    {
        const float hx = (std::max)(0.05f, lengthValue) * 0.5f;
        const float hy = (std::max)(0.05f, widthValue) * 0.5f;
        const float hz = (std::max)(0.05f, thicknessValue) * 0.5f;
        std::vector<Vector> points;
        points.reserve(14);

        for (float x : { -hx, hx }) {
            for (float y : { -hy, hy }) {
                for (float z : { -hz, hz }) {
                    points.push_back(makeVector<Vector>(x, y, z));
                }
            }
        }

        points.push_back(makeVector<Vector>(-hx, 0.0f, 0.0f));
        points.push_back(makeVector<Vector>(hx, 0.0f, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, -hy, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, hy, 0.0f));
        points.push_back(makeVector<Vector>(0.0f, 0.0f, -hz));
        points.push_back(makeVector<Vector>(0.0f, 0.0f, hz));
        return points;
    }

    template <class Transform, class Vector>
    inline BoneColliderFrameResult<Transform, Vector> buildPalmAnchorFrame(
        const Transform& hand,
        const std::array<Vector, 5>& fingerBases,
        const Vector& backOfHandDirection,
        float palmDepth)
    {
        BoneColliderFrameResult<Transform, Vector> result{};
        const Vector fallbackX = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector fallbackY = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector fallbackZ = makeVector<Vector>(0.0f, 0.0f, 1.0f);

        Vector fingerCenter{};
        for (const auto& point : fingerBases) {
            fingerCenter = add(fingerCenter, point);
        }
        fingerCenter = mul(fingerCenter, 1.0f / static_cast<float>(fingerBases.size()));
        Vector palmCenter = hand.translate;
        for (const auto& point : fingerBases) {
            palmCenter = add(palmCenter, point);
        }
        palmCenter = mul(palmCenter, 1.0f / static_cast<float>(fingerBases.size() + 1));

        result.xAxis = normalizeOr(sub(fingerCenter, hand.translate), fallbackX);
        result.backAxis = normalizeOr(backOfHandDirection, fallbackZ);
        result.yAxis = normalizeOr(cross(result.backAxis, result.xAxis), fallbackY);
        result.xAxis = normalizeOr(cross(result.yAxis, result.backAxis), fallbackX);
        result.zAxis = result.backAxis;
        result.length = (std::max)(1.0f, length(sub(fingerCenter, hand.translate)));
        result.radius = (std::max)(0.5f, result.length * 0.45f);
        result.convexRadius = 0.1f;

        result.transform = hand;
        const float currentBackOffset = dot(sub(palmCenter, hand.translate), result.backAxis);
        palmCenter = sub(palmCenter, mul(result.backAxis, currentBackOffset));
        result.transform.translate = add(palmCenter, mul(result.backAxis, -std::fabs(palmDepth) / 3.0f));
        result.transform.rotate = matrixFromAxes<decltype(result.transform.rotate)>(result.xAxis, result.yAxis, result.zAxis);
        result.transform.scale = 1.0f;
        result.valid = true;
        return result;
    }
}
