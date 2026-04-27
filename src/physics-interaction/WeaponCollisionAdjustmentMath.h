#pragma once

#include <cmath>

namespace frik::rock::weapon_collision_adjustment_math
{
    /*
     * Weapon collision bodies copy Bethesda's source shape, but some FO4VR weapon
     * source bodies are authored with their local basis rotated away from the visible
     * stock-to-barrel mesh. Apply an optional local Euler correction only to ROCK's
     * duplicate body transform; the source body, shape pointer, and filter data remain
     * untouched. NiMatrix3 rows are the local basis vectors used by ROCK's hand and
     * debug-axis math, so the correction is pre-multiplied to stay in weapon-local
     * space instead of becoming a world-space roll.
     */
    template <class Matrix>
    inline Matrix identityMatrix()
    {
        Matrix result{};
        result.entry[0][0] = 1.0f;
        result.entry[1][1] = 1.0f;
        result.entry[2][2] = 1.0f;
        return result;
    }

    template <class Matrix>
    inline Matrix multiplyMatrix(const Matrix& a, const Matrix& b)
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.entry[row][col] = a.entry[row][0] * b.entry[0][col] + a.entry[row][1] * b.entry[1][col] + a.entry[row][2] * b.entry[2][col];
            }
        }
        return result;
    }

    template <class Matrix>
    inline Matrix localRotationX(float radians)
    {
        Matrix result = identityMatrix<Matrix>();
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        result.entry[1][1] = c;
        result.entry[1][2] = -s;
        result.entry[2][1] = s;
        result.entry[2][2] = c;
        return result;
    }

    template <class Matrix>
    inline Matrix localRotationY(float radians)
    {
        Matrix result = identityMatrix<Matrix>();
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        result.entry[0][0] = c;
        result.entry[0][2] = s;
        result.entry[2][0] = -s;
        result.entry[2][2] = c;
        return result;
    }

    template <class Matrix>
    inline Matrix localRotationZ(float radians)
    {
        Matrix result = identityMatrix<Matrix>();
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        result.entry[0][0] = c;
        result.entry[0][1] = -s;
        result.entry[1][0] = s;
        result.entry[1][1] = c;
        return result;
    }

    template <class Matrix, class Vector>
    inline Matrix makeLocalEulerCorrection(Vector degrees)
    {
        constexpr float kDegreesToRadians = 3.14159265358979323846f / 180.0f;
        const Matrix rx = localRotationX<Matrix>(degrees.x * kDegreesToRadians);
        const Matrix ry = localRotationY<Matrix>(degrees.y * kDegreesToRadians);
        const Matrix rz = localRotationZ<Matrix>(degrees.z * kDegreesToRadians);
        return multiplyMatrix(multiplyMatrix(rz, ry), rx);
    }

    template <class Matrix, class Vector>
    inline Matrix applyLocalEulerCorrection(const Matrix& sourceRotation, Vector degrees, bool enabled)
    {
        if (!enabled) {
            return sourceRotation;
        }

        const Matrix correction = makeLocalEulerCorrection<Matrix>(degrees);
        return multiplyMatrix(correction, sourceRotation);
    }
}
