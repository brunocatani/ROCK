// Transform value math. The composition and conversion helpers are the public
// header <ROCK/TransformMath.h>, so ROCK and its API consumers share one
// convention. The validity predicates below are ROCK's own: every subsystem
// asks the same question of a transform through them instead of keeping a copy.
#pragma once

#include <ROCK/TransformMath.h>

#include <cmath>

namespace rock::transform_math
{
    // invertTransform and the world-to-local helpers divide by scale only above
    // this magnitude; at or below it they keep scale 1 and the result is not an
    // inverse.
    inline constexpr float kInvertibleScaleMagnitude = 0.0001f;

    // A scale at or below this magnitude has collapsed the transform to a point.
    inline constexpr float kCollapsedScaleMagnitude = 0.000001f;

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
}
