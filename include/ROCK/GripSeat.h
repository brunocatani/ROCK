#pragma once

#include <ROCK/TransformMath.h>

#include <algorithm>
#include <cmath>
#include <limits>

/*
 * Support-hand surface seat: the game-independent math ROCK uses to put a
 * hand onto a weapon surface, shared with the offline tools (Reload Maker)
 * so a grip they show is the grip ROCK computes.
 *
 * Not a provider interface: no table, version or ABI. Types are the caller's,
 * read the way rock::transform_math reads them (Transform{rotate, translate,
 * scale}, Matrix{entry[3][3]} with Ni's stored basis, Vector{x, y, z}
 * brace-constructible from three floats). Arithmetic is float in a fixed
 * order, so ROCK's results are unchanged by the move here.
 */
namespace rock::grip_seat
{
    namespace detail
    {
        template <class Vector>
        inline float dot(const Vector& lhs, const Vector& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        template <class Vector>
        inline Vector cross(const Vector& lhs, const Vector& rhs)
        {
            Vector result{};
            result.x = lhs.y * rhs.z - lhs.z * rhs.y;
            result.y = lhs.z * rhs.x - lhs.x * rhs.z;
            result.z = lhs.x * rhs.y - lhs.y * rhs.x;
            return result;
        }

        template <class Vector>
        inline Vector add(const Vector& lhs, const Vector& rhs)
        {
            return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        template <class Vector>
        inline Vector sub(const Vector& lhs, const Vector& rhs)
        {
            return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        template <class Vector>
        inline Vector scale(const Vector& vector, float factor)
        {
            return Vector{ vector.x * factor, vector.y * factor, vector.z * factor };
        }

        template <class Vector>
        inline float length(const Vector& vector)
        {
            return std::sqrt(dot(vector, vector));
        }

        template <class Vector>
        inline Vector normalize(const Vector& vector)
        {
            const float value = length(vector);
            if (value <= 0.000001f) {
                return Vector{};
            }
            return scale(vector, 1.0f / value);
        }
    }

    template <class Vector>
    inline bool isFiniteVector(const Vector& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    template <class Matrix>
    inline bool isFiniteRotation(const Matrix& rotation)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    template <class Transform>
    inline bool isFiniteTransform(const Transform& transform)
    {
        return isFiniteRotation(transform.rotate) && isFiniteVector(transform.translate) && std::isfinite(transform.scale);
    }

    template <class Transform>
    inline bool isUsableTransform(const Transform& transform)
    {
        return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f;
    }

    // Stored Ni rotation from the rows of a conventional (column-vector) rotation matrix.
    template <class Matrix, class Vector>
    inline Matrix storedRotationFromConventionalRows(const Vector rows[3])
    {
        Matrix result{};
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }

    template <class Vector>
    inline Vector orthogonalAxis(const Vector& value)
    {
        Vector axis = std::abs(value.x) < 0.7f ? Vector{ 1.0f, 0.0f, 0.0f } : Vector{ 0.0f, 1.0f, 0.0f };
        return detail::normalize(detail::cross(value, axis));
    }

    // Shortest-arc world rotation (stored basis) taking direction fromRaw onto toRaw.
    template <class Matrix, class Vector>
    inline Matrix rotationBetweenStored(const Vector& fromRaw, const Vector& toRaw)
    {
        const Vector from = detail::normalize(fromRaw);
        const Vector to = detail::normalize(toRaw);
        float cosTheta = (std::max)(-1.0f, (std::min)(1.0f, detail::dot(from, to)));

        /*
         * The parallel guard exists only to keep the cross-product axis
         * numerically stable; it must stay far below perception. At 0.9999
         * (0.81deg) it becomes a visible deadzone for any solve whose base
         * transform is its own previous output: part-carry accumulated slow
         * hand motion inside the deadzone and released it as ~1deg snaps at
         * ~10Hz. 0.999999 (~0.08deg) still leaves sin(theta) ~1.4e-3, orders
         * of magnitude above float32 cross-product noise.
         */
        if (cosTheta > 0.999999f) {
            return transform_math::makeIdentityRotation<Matrix>();
        }

        Vector axis{};
        float sinTheta = 0.0f;
        if (cosTheta < -0.9999f) {
            axis = orthogonalAxis(from);
            sinTheta = 0.0f;
            cosTheta = -1.0f;
        } else {
            axis = detail::normalize(detail::cross(from, to));
            sinTheta = std::sqrt((std::max)(0.0f, 1.0f - cosTheta * cosTheta));
        }

        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float oneMinusCos = 1.0f - cosTheta;

        Vector conventionalRows[3]{
            Vector{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            Vector{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            Vector{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };

        return storedRotationFromConventionalRows<Matrix, Vector>(conventionalRows);
    }

    template <class Matrix, class Vector>
    inline Matrix applyWorldRotationToStoredBasis(const Matrix& worldRotationStored, const Matrix& baseRotation)
    {
        Matrix result{};
        const Vector basis[3]{
            Vector{ baseRotation.entry[0][0], baseRotation.entry[0][1], baseRotation.entry[0][2] },
            Vector{ baseRotation.entry[1][0], baseRotation.entry[1][1], baseRotation.entry[1][2] },
            Vector{ baseRotation.entry[2][0], baseRotation.entry[2][1], baseRotation.entry[2][2] },
        };

        for (int axis = 0; axis < 3; ++axis) {
            const Vector rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis[axis]);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }

    inline bool normalizeQuaternion(float quaternion[4])
    {
        float lengthSquared = 0.0f;
        for (int index = 0; index < 4; ++index) {
            if (!std::isfinite(quaternion[index])) {
                return false;
            }
            lengthSquared += quaternion[index] * quaternion[index];
        }
        if (!std::isfinite(lengthSquared) || lengthSquared <= 0.000001f) {
            return false;
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        for (int index = 0; index < 4; ++index) {
            quaternion[index] *= inverseLength;
        }
        return true;
    }

    template <class Matrix>
    inline bool shortestArcSlerpFromIdentity(const Matrix& fullRotationDelta, float alpha, Matrix& outPartialRotationDelta)
    {
        outPartialRotationDelta = transform_math::makeIdentityRotation<Matrix>();
        if (!std::isfinite(alpha) || !isFiniteRotation(fullRotationDelta)) {
            return false;
        }

        const float t = std::clamp(alpha, 0.0f, 1.0f);
        if (t <= 0.0f) {
            return true;
        }
        if (t >= 1.0f) {
            outPartialRotationDelta = fullRotationDelta;
            return true;
        }

        float target[4]{};
        transform_math::niRowsToHavokQuaternion(fullRotationDelta, target);
        if (!normalizeQuaternion(target)) {
            return false;
        }

        float cosTheta = target[3];
        if (cosTheta < 0.0f) {
            for (float& component : target) {
                component = -component;
            }
            cosTheta = -cosTheta;
        }
        cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);

        float partial[4]{};
        if (cosTheta > 0.9995f) {
            partial[0] = target[0] * t;
            partial[1] = target[1] * t;
            partial[2] = target[2] * t;
            partial[3] = 1.0f + (target[3] - 1.0f) * t;
        } else {
            const float theta = std::acos(cosTheta);
            const float sinTheta = std::sin(theta);
            if (!std::isfinite(sinTheta) || std::abs(sinTheta) <= 0.000001f) {
                return false;
            }
            const float identityWeight = std::sin((1.0f - t) * theta) / sinTheta;
            const float targetWeight = std::sin(t * theta) / sinTheta;
            partial[0] = target[0] * targetWeight;
            partial[1] = target[1] * targetWeight;
            partial[2] = target[2] * targetWeight;
            partial[3] = identityWeight + target[3] * targetWeight;
        }

        if (!normalizeQuaternion(partial)) {
            return false;
        }
        outPartialRotationDelta = transform_math::havokQuaternionToNiRows<Matrix>(partial);
        return isFiniteRotation(outPartialRotationDelta);
    }

    template <class Matrix>
    inline float rotationDistanceRadians(const Matrix& from, const Matrix& to)
    {
        if (!isFiniteRotation(from) || !isFiniteRotation(to)) {
            return (std::numeric_limits<float>::quiet_NaN)();
        }

        float fromQuaternion[4]{};
        float toQuaternion[4]{};
        transform_math::niRowsToHavokQuaternion(from, fromQuaternion);
        transform_math::niRowsToHavokQuaternion(to, toQuaternion);
        if (!normalizeQuaternion(fromQuaternion) || !normalizeQuaternion(toQuaternion)) {
            return (std::numeric_limits<float>::quiet_NaN)();
        }

        const float dotValue = std::abs(fromQuaternion[0] * toQuaternion[0] + fromQuaternion[1] * toQuaternion[1] + fromQuaternion[2] * toQuaternion[2] +
                                        fromQuaternion[3] * toQuaternion[3]);
        return 2.0f * std::acos(std::clamp(dotValue, 0.0f, 1.0f));
    }

    template <class Matrix>
    inline float rotationAngleRadians(const Matrix& rotation)
    {
        return rotationDistanceRadians(transform_math::makeIdentityRotation<Matrix>(), rotation);
    }

    template <class Transform, class Vector>
    struct SurfaceAlignedHandFrameResult
    {
        Transform handWorld{};
        float appliedRotationRadians{ 0.0f };
        bool usedSurfaceNormal{ false };
        bool valid{ false };
    };

    /*
     * Rotate the support hand, never the weapon, around the captured palm
     * pivot. The palm-facing axis targets the inward surface direction
     * (-surfaceNormal), shortest-arc rotation is bounded, and the pivot is
     * translated onto the selected mesh point exactly.
     */
    template <class Transform, class Vector>
    inline SurfaceAlignedHandFrameResult<Transform, Vector> alignHandFrameToGripSurface(const Transform& handWorld, const Vector& palmPivotWorld,
        const Vector& palmNormalWorld, const Vector& targetGripPointWorld, const Vector& surfaceNormalWorld, float maxCorrectionRadians)
    {
        using Matrix = decltype(handWorld.rotate);
        SurfaceAlignedHandFrameResult<Transform, Vector> result{};
        result.handWorld = handWorld;
        if (!isUsableTransform(handWorld) || !isFiniteVector(palmPivotWorld) || !isFiniteVector(targetGripPointWorld)) {
            return result;
        }

        auto seatWithoutRotation = [&]() {
            result.handWorld.translate = detail::add(result.handWorld.translate, detail::sub(targetGripPointWorld, palmPivotWorld));
            result.valid = isUsableTransform(result.handWorld);
        };

        const float palmLength = detail::length(palmNormalWorld);
        const float surfaceLength = detail::length(surfaceNormalWorld);
        if (!std::isfinite(maxCorrectionRadians) || maxCorrectionRadians <= 0.0f || palmLength <= 0.0001f || surfaceLength <= 0.0001f) {
            seatWithoutRotation();
            return result;
        }

        const Vector currentPalmNormal = detail::normalize(palmNormalWorld);
        const Vector desiredPalmNormal = detail::scale(detail::normalize(surfaceNormalWorld), -1.0f);
        const auto fullRotation = rotationBetweenStored<Matrix, Vector>(currentPalmNormal, desiredPalmNormal);
        const float fullAngle = rotationAngleRadians(fullRotation);
        if (!std::isfinite(fullAngle)) {
            seatWithoutRotation();
            return result;
        }

        Matrix boundedRotation = transform_math::makeIdentityRotation<Matrix>();
        const float alpha = fullAngle > 0.000001f ? std::clamp(maxCorrectionRadians / fullAngle, 0.0f, 1.0f) : 0.0f;
        if (fullAngle > 0.000001f && !shortestArcSlerpFromIdentity(fullRotation, alpha, boundedRotation)) {
            seatWithoutRotation();
            return result;
        }

        result.handWorld.rotate = applyWorldRotationToStoredBasis<Matrix, Vector>(boundedRotation, handWorld.rotate);
        const Vector originFromPalm = detail::sub(handWorld.translate, palmPivotWorld);
        result.handWorld.translate = detail::add(targetGripPointWorld, transform_math::rotateLocalVectorToWorld(boundedRotation, originFromPalm));
        result.appliedRotationRadians = rotationAngleRadians(boundedRotation);
        result.usedSurfaceNormal = fullAngle > 0.000001f;
        result.valid = isUsableTransform(result.handWorld) && std::isfinite(result.appliedRotationRadians);
        return result;
    }
}
