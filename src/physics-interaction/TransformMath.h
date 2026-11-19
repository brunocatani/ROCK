// Transform value math. The composition and conversion helpers are the public
// header <ROCK/TransformMath.h>, so ROCK and its API consumers share one
// convention. The validity predicates below are ROCK's own: every subsystem
// asks the same question of a transform through them instead of keeping a copy.
#pragma once

#include <ROCK/TransformMath.h>

#include <algorithm>
#include <cmath>

namespace rock::transform_math
{
    // invertTransform and the world-to-local helpers divide by scale only above
    // this magnitude; at or below it they keep scale 1 and the result is not an
    // inverse.
    inline constexpr float kInvertibleScaleMagnitude = 0.0001f;

    // A scale at or below this magnitude has collapsed the transform to a point.
    inline constexpr float kCollapsedScaleMagnitude = 0.000001f;

    // The two magnitudes are not interchangeable. A node hidden for the scope
    // view carries scale 0.00001, between them: it still has a usable position
    // and rotation, so code that only reads or composes it accepts it through
    // isNonCollapsedTransform, while code that needs its inverse refuses it
    // through isInvertibleTransform. Swapping one check for the other changes
    // what a call site does with every hidden node; trace that site first.

    template <class Matrix>
    [[nodiscard]] inline bool isFiniteRotation(const Matrix& rotation) noexcept
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

    // Every stored component is finite. Says nothing about scale magnitude or
    // whether the rotation is orthonormal.
    template <class Transform>
    [[nodiscard]] inline bool isFiniteTransform(const Transform& transform) noexcept
    {
        return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) &&
            std::isfinite(transform.translate.y) && std::isfinite(transform.translate.z) &&
            std::isfinite(transform.scale);
    }

    // Finite, and invertTransform returns its true inverse.
    template <class Transform>
    [[nodiscard]] inline bool isInvertibleTransform(const Transform& transform) noexcept
    {
        return isFiniteTransform(transform) && std::abs(transform.scale) > kInvertibleScaleMagnitude;
    }

    // Finite with a scale that has not collapsed. Weaker than
    // isInvertibleTransform: a scale between the two magnitudes passes here and
    // does not invert.
    template <class Transform>
    [[nodiscard]] inline bool isNonCollapsedTransform(const Transform& transform) noexcept
    {
        return isFiniteTransform(transform) && std::abs(transform.scale) > kCollapsedScaleMagnitude;
    }

    inline constexpr float kRadiansToDegrees = 57.29577951308232f;

    // Angle between two rotation matrices in degrees, from the trace of
    // first^T * second. acos is flat near 1, so differences well under a degree
    // come out as rounding noise; a comparison against a sub-degree limit uses
    // rotationDeltaDegreesPrecise.
    template <class Matrix>
    [[nodiscard]] inline float rotationDeltaDegrees(const Matrix& first, const Matrix& second) noexcept
    {
        float trace = 0.0f;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                trace += first.entry[row][column] * second.entry[row][column];
            }
        }
        const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosine) * kRadiansToDegrees;
    }

    // The same angle from both the cosine (trace) and the sine (skew part) of
    // first^T * second, so a sub-degree difference is exact.
    template <class Matrix>
    [[nodiscard]] inline float rotationDeltaDegreesPrecise(const Matrix& first, const Matrix& second) noexcept
    {
        float relative[3][3]{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    sum += first.entry[k][row] * second.entry[k][column];
                }
                relative[row][column] = sum;
            }
        }
        const float trace = relative[0][0] + relative[1][1] + relative[2][2];
        const float skewX = relative[2][1] - relative[1][2];
        const float skewY = relative[0][2] - relative[2][0];
        const float skewZ = relative[1][0] - relative[0][1];
        const float sine = 0.5f * std::sqrt(skewX * skewX + skewY * skewY + skewZ * skewZ);
        const float cosine = 0.5f * (trace - 1.0f);
        return std::atan2(sine, cosine) * kRadiansToDegrees;
    }
}
