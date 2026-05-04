#pragma once

#include <cmath>

namespace rock::transform_math
{
    /*
     * Shared transform convention:
     * HIGGS keeps grab and collision frames as NiTransform relationships, and
     * FO4VR's compose helper at 0x1401A8D60 confirms that child-local vectors
     * are applied through the stored Ni basis as Transpose()*v. Centralizing
     * that here prevents hand grabs, debug markers, two-handed grip, and mesh
     * weapon bodies from drifting back into the older row-vector/wand math.
     */

    template <class Matrix>
    inline Matrix makeIdentityRotation()
    {
        Matrix result{};
        result.entry[0][0] = 1.0f;
        result.entry[1][1] = 1.0f;
        result.entry[2][2] = 1.0f;
        return result;
    }

    template <class Transform>
    inline Transform makeIdentityTransform()
    {
        Transform result{};
        result.rotate = makeIdentityRotation<decltype(result.rotate)>();
        result.scale = 1.0f;
        return result;
    }

    template <class Matrix>
    inline Matrix transposeRotation(const Matrix& matrix)
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.entry[row][column] = matrix.entry[column][row];
            }
        }
        return result;
    }

    template <class Matrix>
    inline Matrix multiplyStoredRotations(const Matrix& lhs, const Matrix& rhs)
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.entry[row][column] = lhs.entry[row][0] * rhs.entry[0][column] + lhs.entry[row][1] * rhs.entry[1][column] +
                                            lhs.entry[row][2] * rhs.entry[2][column];
            }
        }
        return result;
    }

    template <class Matrix, class Vector>
    inline Vector rotateLocalVectorToWorld(const Matrix& matrix, const Vector& vector)
    {
        return Vector{
            matrix.entry[0][0] * vector.x + matrix.entry[1][0] * vector.y + matrix.entry[2][0] * vector.z,
            matrix.entry[0][1] * vector.x + matrix.entry[1][1] * vector.y + matrix.entry[2][1] * vector.z,
            matrix.entry[0][2] * vector.x + matrix.entry[1][2] * vector.y + matrix.entry[2][2] * vector.z,
        };
    }

    template <class Matrix, class Vector>
    inline Vector rotateWorldVectorToLocal(const Matrix& matrix, const Vector& vector)
    {
        return Vector{
            matrix.entry[0][0] * vector.x + matrix.entry[0][1] * vector.y + matrix.entry[0][2] * vector.z,
            matrix.entry[1][0] * vector.x + matrix.entry[1][1] * vector.y + matrix.entry[1][2] * vector.z,
            matrix.entry[2][0] * vector.x + matrix.entry[2][1] * vector.y + matrix.entry[2][2] * vector.z,
        };
    }

    template <class Transform, class Vector>
    inline Vector localVectorToWorld(const Transform& transform, const Vector& vector)
    {
        const Vector rotated = rotateLocalVectorToWorld(transform.rotate, vector);
        return Vector{ rotated.x * transform.scale, rotated.y * transform.scale, rotated.z * transform.scale };
    }

    template <class Transform, class Vector>
    inline Vector worldVectorToLocal(const Transform& transform, const Vector& vector)
    {
        const float inverseScale = (std::abs(transform.scale) > 0.0001f) ? (1.0f / transform.scale) : 1.0f;
        const Vector rotated = rotateWorldVectorToLocal(transform.rotate, vector);
        return Vector{ rotated.x * inverseScale, rotated.y * inverseScale, rotated.z * inverseScale };
    }

    template <class Transform, class Vector>
    inline Vector localPointToWorld(const Transform& transform, const Vector& point)
    {
        const Vector offset = localVectorToWorld(transform, point);
        return Vector{ transform.translate.x + offset.x, transform.translate.y + offset.y, transform.translate.z + offset.z };
    }

    template <class Transform, class Vector>
    inline Vector worldPointToLocal(const Transform& transform, const Vector& point)
    {
        return worldVectorToLocal(transform,
            Vector{ point.x - transform.translate.x, point.y - transform.translate.y, point.z - transform.translate.z });
    }

    template <class Transform>
    inline Transform composeTransforms(const Transform& parent, const Transform& child)
    {
        Transform result = makeIdentityTransform<Transform>();
        result.rotate = multiplyStoredRotations(child.rotate, parent.rotate);
        result.translate = localPointToWorld(parent, child.translate);
        result.scale = parent.scale * child.scale;
        return result;
    }

    template <class Transform>
    inline Transform invertTransform(const Transform& transform)
    {
        Transform result = makeIdentityTransform<Transform>();
        result.rotate = transposeRotation(transform.rotate);
        result.scale = (std::abs(transform.scale) > 0.0001f) ? (1.0f / transform.scale) : 1.0f;
        const decltype(transform.translate) origin{};
        result.translate = worldPointToLocal(transform, origin);
        return result;
    }

    // FO4VR hknp consumes hkTransformf rotation as Havok column blocks, while
    // CommonLib NiMatrix3 math is written in row form. Keep this conversion
    // explicit so hand, grab, and weapon bodies all use one verified convention.
    template <class Matrix>
    inline Matrix niRowsToHavokColumns(const Matrix& matrix)
    {
        Matrix result{};
        result.entry[0][0] = matrix.entry[0][0];
        result.entry[0][1] = matrix.entry[1][0];
        result.entry[0][2] = matrix.entry[2][0];
        result.entry[0][3] = 0.0f;

        result.entry[1][0] = matrix.entry[0][1];
        result.entry[1][1] = matrix.entry[1][1];
        result.entry[1][2] = matrix.entry[2][1];
        result.entry[1][3] = 0.0f;

        result.entry[2][0] = matrix.entry[0][2];
        result.entry[2][1] = matrix.entry[1][2];
        result.entry[2][2] = matrix.entry[2][2];
        result.entry[2][3] = 0.0f;
        return result;
    }

    template <class Matrix>
    inline Matrix havokColumnsToNiRows(const float* bodyFloats)
    {
        Matrix result{};
        result.entry[0][0] = bodyFloats[0];
        result.entry[0][1] = bodyFloats[4];
        result.entry[0][2] = bodyFloats[8];
        result.entry[0][3] = 0.0f;

        result.entry[1][0] = bodyFloats[1];
        result.entry[1][1] = bodyFloats[5];
        result.entry[1][2] = bodyFloats[9];
        result.entry[1][3] = 0.0f;

        result.entry[2][0] = bodyFloats[2];
        result.entry[2][1] = bodyFloats[6];
        result.entry[2][2] = bodyFloats[10];
        result.entry[2][3] = 0.0f;
        return result;
    }

    template <class Matrix>
    inline Matrix havokQuaternionToNiRows(const float quaternion[4])
    {
        Matrix matrix = makeIdentityRotation<Matrix>();
        float x = quaternion[0];
        float y = quaternion[1];
        float z = quaternion[2];
        float w = quaternion[3];

        const float length = std::sqrt(x * x + y * y + z * z + w * w);
        if (length <= 0.000001f) {
            return matrix;
        }

        const float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        w *= invLength;

        matrix.entry[0][0] = 1.0f - 2.0f * (y * y + z * z);
        matrix.entry[0][1] = 2.0f * (x * y - w * z);
        matrix.entry[0][2] = 2.0f * (x * z + w * y);
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][0] = 2.0f * (x * y + w * z);
        matrix.entry[1][1] = 1.0f - 2.0f * (x * x + z * z);
        matrix.entry[1][2] = 2.0f * (y * z - w * x);
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][0] = 2.0f * (x * z - w * y);
        matrix.entry[2][1] = 2.0f * (y * z + w * x);
        matrix.entry[2][2] = 1.0f - 2.0f * (x * x + y * y);
        matrix.entry[2][3] = 0.0f;
        return matrix;
    }

    template <class Matrix>
    inline void niRowsToHavokQuaternion(const Matrix& matrix, float outQuat[4])
    {
        const float trace = matrix.entry[0][0] + matrix.entry[1][1] + matrix.entry[2][2];
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;

        if (trace > 0.0f) {
            const float root = std::sqrt(trace + 1.0f);
            const float inv = 0.5f / root;
            w = root * 0.5f;
            x = (matrix.entry[2][1] - matrix.entry[1][2]) * inv;
            y = (matrix.entry[0][2] - matrix.entry[2][0]) * inv;
            z = (matrix.entry[1][0] - matrix.entry[0][1]) * inv;
        } else if (matrix.entry[0][0] > matrix.entry[1][1] && matrix.entry[0][0] > matrix.entry[2][2]) {
            const float root = std::sqrt((matrix.entry[0][0] - (matrix.entry[1][1] + matrix.entry[2][2])) + 1.0f);
            const float inv = 0.5f / root;
            x = root * 0.5f;
            y = (matrix.entry[1][0] + matrix.entry[0][1]) * inv;
            z = (matrix.entry[0][2] + matrix.entry[2][0]) * inv;
            w = (matrix.entry[2][1] - matrix.entry[1][2]) * inv;
        } else if (matrix.entry[1][1] > matrix.entry[2][2]) {
            const float root = std::sqrt((matrix.entry[1][1] - (matrix.entry[2][2] + matrix.entry[0][0])) + 1.0f);
            const float inv = 0.5f / root;
            x = (matrix.entry[1][0] + matrix.entry[0][1]) * inv;
            y = root * 0.5f;
            z = (matrix.entry[2][1] + matrix.entry[1][2]) * inv;
            w = (matrix.entry[0][2] - matrix.entry[2][0]) * inv;
        } else {
            const float root = std::sqrt((matrix.entry[2][2] - (matrix.entry[0][0] + matrix.entry[1][1])) + 1.0f);
            const float inv = 0.5f / root;
            x = (matrix.entry[0][2] + matrix.entry[2][0]) * inv;
            y = (matrix.entry[2][1] + matrix.entry[1][2]) * inv;
            z = root * 0.5f;
            w = (matrix.entry[1][0] - matrix.entry[0][1]) * inv;
        }

        const float length = std::sqrt(x * x + y * y + z * z + w * w);
        if (length > 0.0f) {
            const float invLength = 1.0f / length;
            x *= invLength;
            y *= invLength;
            z *= invLength;
            w *= invLength;
        }

        outQuat[0] = x;
        outQuat[1] = y;
        outQuat[2] = z;
        outQuat[3] = w;
    }
}
