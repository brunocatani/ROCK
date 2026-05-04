#pragma once

/*
 * ROCK's opposition grab frame uses hand/finger colliders as semantic contact
 * evidence without letting raw collider collision fight the held-object
 * constraint. HIGGS captures one coherent object-in-hand frame and computes
 * finger curves from mesh geometry; this layer adds ROCK's bone-derived
 * thumb-versus-finger convention before that frame is captured, so the object
 * is seated between the live thumb and opposing finger when both contacts are
 * trustworthy.
 */

#include "TransformMath.h"

#include <cmath>

namespace frik::rock::grab_opposition_frame_math
{
    template <class Transform, class Vector>
    struct OppositionFrameInput
    {
        bool enabled = false;
        Transform objectWorld{};
        Vector thumbObjectLocal{};
        Vector opposingObjectLocal{};
        Vector thumbHandWorld{};
        Vector opposingHandWorld{};
        Vector objectRollAxisLocal{};
        Vector handRollAxisWorld{};
    };

    template <class Transform, class Vector>
    struct OppositionFrameResult
    {
        bool valid = false;
        Transform desiredObjectWorld{};
        Vector pivotWorld{};
        const char* reason = "disabled";
    };

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
        return makeVector<Vector>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x);
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
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float len = length(value);
        if (len <= 1.0e-6f || !std::isfinite(len)) {
            return {};
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector yAxis = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector zAxis = makeVector<Vector>(0.0f, 0.0f, 1.0f);
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }

    template <class Matrix, class Vector>
    inline void setRows(Matrix& matrix, const Vector& row0, const Vector& row1, const Vector& row2)
    {
        matrix.entry[0][0] = row0.x;
        matrix.entry[0][1] = row0.y;
        matrix.entry[0][2] = row0.z;
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][0] = row1.x;
        matrix.entry[1][1] = row1.y;
        matrix.entry[1][2] = row1.z;
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][0] = row2.x;
        matrix.entry[2][1] = row2.y;
        matrix.entry[2][2] = row2.z;
        matrix.entry[2][3] = 0.0f;
    }

    template <class Matrix, class Vector>
    inline Matrix buildRotationFromBasisMap(const Vector& localOpposition,
        const Vector& localRoll,
        const Vector& localPalm,
        const Vector& targetOpposition,
        const Vector& targetRoll,
        const Vector& targetPalm)
    {
        Matrix result{};
        const Vector row0 = add(add(mul(targetOpposition, localOpposition.x), mul(targetRoll, localRoll.x)), mul(targetPalm, localPalm.x));
        const Vector row1 = add(add(mul(targetOpposition, localOpposition.y), mul(targetRoll, localRoll.y)), mul(targetPalm, localPalm.y));
        const Vector row2 = add(add(mul(targetOpposition, localOpposition.z), mul(targetRoll, localRoll.z)), mul(targetPalm, localPalm.z));
        setRows(result, row0, row1, row2);
        return result;
    }

    template <class Transform, class Vector>
    inline OppositionFrameResult<Transform, Vector> buildOppositionDesiredObjectWorld(const OppositionFrameInput<Transform, Vector>& input)
    {
        OppositionFrameResult<Transform, Vector> result{};
        result.desiredObjectWorld = input.objectWorld;
        result.pivotWorld = mul(add(input.thumbHandWorld, input.opposingHandWorld), 0.5f);
        if (!input.enabled) {
            result.reason = "disabled";
            return result;
        }

        const Vector localOpposition = normalizeOrZero(sub(input.opposingObjectLocal, input.thumbObjectLocal));
        const Vector targetOpposition = normalizeOrZero(sub(input.opposingHandWorld, input.thumbHandWorld));
        if (lengthSquared(targetOpposition) <= 0.0f) {
            result.reason = "degenerateHandSpan";
            return result;
        }
        if (lengthSquared(localOpposition) <= 0.0f) {
            result.reason = "degenerateObjectSpan";
            return result;
        }

        Vector localRoll = normalizeOrZero(projectOntoPlane(input.objectRollAxisLocal, localOpposition));
        if (lengthSquared(localRoll) <= 0.0f) {
            localRoll = stablePerpendicular(localOpposition);
        }
        Vector targetRoll = normalizeOrZero(projectOntoPlane(input.handRollAxisWorld, targetOpposition));
        if (lengthSquared(targetRoll) <= 0.0f) {
            targetRoll = stablePerpendicular(targetOpposition);
        }

        const Vector localPalm = normalizeOrZero(cross(localOpposition, localRoll));
        const Vector targetPalm = normalizeOrZero(cross(targetOpposition, targetRoll));
        if (lengthSquared(localPalm) <= 0.0f) {
            result.reason = "invalidObjectBasis";
            return result;
        }
        if (lengthSquared(targetPalm) <= 0.0f) {
            result.reason = "invalidHandBasis";
            return result;
        }

        result.desiredObjectWorld = input.objectWorld;
        result.desiredObjectWorld.rotate = buildRotationFromBasisMap<decltype(input.objectWorld.rotate), Vector>(
            localOpposition,
            localRoll,
            localPalm,
            targetOpposition,
            targetRoll,
            targetPalm);
        result.desiredObjectWorld.translate = sub(result.pivotWorld,
            transform_math::localVectorToWorld(result.desiredObjectWorld, mul(add(input.thumbObjectLocal, input.opposingObjectLocal), 0.5f)));
        result.valid = true;
        result.reason = "thumbOpposition";
        return result;
    }
}
