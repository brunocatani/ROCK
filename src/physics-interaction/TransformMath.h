#pragma once

#include <cmath>
#include <type_traits>

namespace rock::transform_math
{
    /*
     * Shared transform convention:
     * ROCK keeps grab and collision frames as NiTransform relationships. FO4VR's
     * compose helper at 0x1401A8D60 confirms that child-local vectors are applied
     * through the stored Ni basis as Transpose()*v. Centralizing that here
     * prevents hand grabs, debug markers, two-handed grip, and mesh weapon
     * bodies from drifting back into the older row-vector/wand math.
     *
     * Intermediate arithmetic intentionally promotes to double before writing
     * back to Ni's float storage. This mirrors HIGGS' double transform path for
     * ROCK's shared grab/weapon transform math without changing engine types.
     */

    namespace detail
    {
        template <class Value>
        using StoredValue = std::remove_cv_t<std::remove_reference_t<Value>>;

        template <class Value>
        inline StoredValue<Value> narrowDouble(double value)
        {
            return static_cast<StoredValue<Value>>(value);
        }

        template <class Vector>
        inline Vector makeVector(double x, double y, double z)
        {
            Vector result{};
            result.x = narrowDouble<decltype(result.x)>(x);
            result.y = narrowDouble<decltype(result.y)>(y);
            result.z = narrowDouble<decltype(result.z)>(z);
            return result;
        }

        template <class Matrix>
        inline void setMatrixEntry(Matrix& matrix, int row, int column, double value)
        {
            matrix.entry[row][column] = narrowDouble<decltype(matrix.entry[row][column])>(value);
        }

        template <class Matrix>
        inline double multiplyStoredRotationEntry(const Matrix& lhs, const Matrix& rhs, int row, int column)
        {
            return static_cast<double>(lhs.entry[row][0]) * static_cast<double>(rhs.entry[0][column]) +
                   static_cast<double>(lhs.entry[row][1]) * static_cast<double>(rhs.entry[1][column]) +
                   static_cast<double>(lhs.entry[row][2]) * static_cast<double>(rhs.entry[2][column]);
        }

        template <class Matrix, class Vector>
        inline double rotateLocalVectorToWorldAxis(const Matrix& matrix, const Vector& vector, int column)
        {
            return static_cast<double>(matrix.entry[0][column]) * static_cast<double>(vector.x) +
                   static_cast<double>(matrix.entry[1][column]) * static_cast<double>(vector.y) +
                   static_cast<double>(matrix.entry[2][column]) * static_cast<double>(vector.z);
        }

        template <class Matrix, class Vector>
        inline double rotateWorldVectorToLocalAxis(const Matrix& matrix, const Vector& vector, int row)
        {
            return static_cast<double>(matrix.entry[row][0]) * static_cast<double>(vector.x) +
                   static_cast<double>(matrix.entry[row][1]) * static_cast<double>(vector.y) +
                   static_cast<double>(matrix.entry[row][2]) * static_cast<double>(vector.z);
        }

        template <class Matrix>
        inline double rotateWorldOffsetToLocalAxis(const Matrix& matrix, double offsetX, double offsetY, double offsetZ, int row)
        {
            return static_cast<double>(matrix.entry[row][0]) * offsetX + static_cast<double>(matrix.entry[row][1]) * offsetY +
                   static_cast<double>(matrix.entry[row][2]) * offsetZ;
        }
    }

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
                detail::setMatrixEntry(result, row, column, detail::multiplyStoredRotationEntry(lhs, rhs, row, column));
            }
        }
        return result;
    }

    template <class Matrix, class Vector>
    inline Vector rotateLocalVectorToWorld(const Matrix& matrix, const Vector& vector)
    {
        return detail::makeVector<Vector>(
            detail::rotateLocalVectorToWorldAxis(matrix, vector, 0),
            detail::rotateLocalVectorToWorldAxis(matrix, vector, 1),
            detail::rotateLocalVectorToWorldAxis(matrix, vector, 2));
    }

    template <class Matrix, class Vector>
    inline Vector rotateWorldVectorToLocal(const Matrix& matrix, const Vector& vector)
    {
        return detail::makeVector<Vector>(
            detail::rotateWorldVectorToLocalAxis(matrix, vector, 0),
            detail::rotateWorldVectorToLocalAxis(matrix, vector, 1),
            detail::rotateWorldVectorToLocalAxis(matrix, vector, 2));
    }

    template <class Transform, class Vector>
    inline Vector localVectorToWorld(const Transform& transform, const Vector& vector)
    {
        const double scale = static_cast<double>(transform.scale);
        return detail::makeVector<Vector>(
            detail::rotateLocalVectorToWorldAxis(transform.rotate, vector, 0) * scale,
            detail::rotateLocalVectorToWorldAxis(transform.rotate, vector, 1) * scale,
            detail::rotateLocalVectorToWorldAxis(transform.rotate, vector, 2) * scale);
    }

    template <class Transform, class Vector>
    inline Vector worldVectorToLocal(const Transform& transform, const Vector& vector)
    {
        const double scale = static_cast<double>(transform.scale);
        const double inverseScale = (std::abs(scale) > 0.0001) ? (1.0 / scale) : 1.0;
        return detail::makeVector<Vector>(
            detail::rotateWorldVectorToLocalAxis(transform.rotate, vector, 0) * inverseScale,
            detail::rotateWorldVectorToLocalAxis(transform.rotate, vector, 1) * inverseScale,
            detail::rotateWorldVectorToLocalAxis(transform.rotate, vector, 2) * inverseScale);
    }

    template <class Transform, class Vector>
    inline Vector localPointToWorld(const Transform& transform, const Vector& point)
    {
        const double scale = static_cast<double>(transform.scale);
        return detail::makeVector<Vector>(
            static_cast<double>(transform.translate.x) + detail::rotateLocalVectorToWorldAxis(transform.rotate, point, 0) * scale,
            static_cast<double>(transform.translate.y) + detail::rotateLocalVectorToWorldAxis(transform.rotate, point, 1) * scale,
            static_cast<double>(transform.translate.z) + detail::rotateLocalVectorToWorldAxis(transform.rotate, point, 2) * scale);
    }

    template <class Transform, class Vector>
    inline Vector worldPointToLocal(const Transform& transform, const Vector& point)
    {
        const double scale = static_cast<double>(transform.scale);
        const double inverseScale = (std::abs(scale) > 0.0001) ? (1.0 / scale) : 1.0;
        const double offsetX = static_cast<double>(point.x) - static_cast<double>(transform.translate.x);
        const double offsetY = static_cast<double>(point.y) - static_cast<double>(transform.translate.y);
        const double offsetZ = static_cast<double>(point.z) - static_cast<double>(transform.translate.z);

        return detail::makeVector<Vector>(
            detail::rotateWorldOffsetToLocalAxis(transform.rotate, offsetX, offsetY, offsetZ, 0) * inverseScale,
            detail::rotateWorldOffsetToLocalAxis(transform.rotate, offsetX, offsetY, offsetZ, 1) * inverseScale,
            detail::rotateWorldOffsetToLocalAxis(transform.rotate, offsetX, offsetY, offsetZ, 2) * inverseScale);
    }

    template <class Transform>
    inline Transform composeTransforms(const Transform& parent, const Transform& child)
    {
        Transform result = makeIdentityTransform<Transform>();
        result.rotate = multiplyStoredRotations(child.rotate, parent.rotate);
        result.translate = localPointToWorld(parent, child.translate);
        result.scale = detail::narrowDouble<decltype(result.scale)>(static_cast<double>(parent.scale) * static_cast<double>(child.scale));
        return result;
    }

    template <class Transform>
    inline Transform invertTransform(const Transform& transform)
    {
        Transform result = makeIdentityTransform<Transform>();
        result.rotate = transposeRotation(transform.rotate);
        const double scale = static_cast<double>(transform.scale);
        result.scale = detail::narrowDouble<decltype(result.scale)>((std::abs(scale) > 0.0001) ? (1.0 / scale) : 1.0);
        const decltype(transform.translate) origin{};
        result.translate = worldPointToLocal(transform, origin);
        return result;
    }

    /*
     * FO4VR hknp BODY slots store the three local axes as 4-float blocks:
     * [0,1,2], [4,5,6], [8,9,10]. ROCK's grab-space NiTransform helpers also
     * store local axes as rows, because localVectorToWorld applies x*row0 +
     * y*row1 + z*row2. BODY readback and deferred BODY writes must therefore
     * copy those axis blocks directly. The older column transpose helpers below
     * remain available for native helpers that intentionally build a column-axis
     * NiMatrix before passing it across the hkTransform boundary.
     */
    template <class Matrix>
    inline Matrix hknpBodyColumnsToNiStoredAxes(const float* bodyFloats)
    {
        Matrix result{};
        result.entry[0][0] = bodyFloats[0];
        result.entry[0][1] = bodyFloats[1];
        result.entry[0][2] = bodyFloats[2];
        result.entry[0][3] = 0.0f;

        result.entry[1][0] = bodyFloats[4];
        result.entry[1][1] = bodyFloats[5];
        result.entry[1][2] = bodyFloats[6];
        result.entry[1][3] = 0.0f;

        result.entry[2][0] = bodyFloats[8];
        result.entry[2][1] = bodyFloats[9];
        result.entry[2][2] = bodyFloats[10];
        result.entry[2][3] = 0.0f;
        return result;
    }

    template <class Matrix>
    inline Matrix niStoredAxesToHknpBodyColumns(const Matrix& matrix)
    {
        Matrix result{};
        result.entry[0][0] = matrix.entry[0][0];
        result.entry[0][1] = matrix.entry[0][1];
        result.entry[0][2] = matrix.entry[0][2];
        result.entry[0][3] = 0.0f;

        result.entry[1][0] = matrix.entry[1][0];
        result.entry[1][1] = matrix.entry[1][1];
        result.entry[1][2] = matrix.entry[1][2];
        result.entry[1][3] = 0.0f;

        result.entry[2][0] = matrix.entry[2][0];
        result.entry[2][1] = matrix.entry[2][1];
        result.entry[2][2] = matrix.entry[2][2];
        result.entry[2][3] = 0.0f;
        return result;
    }

    // Transpose between ROCK stored rows and native helpers that deliberately
    // use NiMatrix columns as physical axes before building an hkTransformf.
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
