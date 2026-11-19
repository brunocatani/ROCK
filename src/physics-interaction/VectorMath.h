// Dot, cross and length helpers for any type with x, y and z members.
#pragma once

#include <cmath>

namespace rock::vector_math
{
    /*
     * Dependency-light arithmetic for three-component vector value types.
     * This header deliberately owns no normalization threshold or invalid-input
     * fallback policy. Callers retain those subsystem-specific decisions.
     */
    template <class Vector>
    [[nodiscard]] constexpr auto dot(const Vector& lhs, const Vector& rhs) noexcept
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    [[nodiscard]] constexpr Vector cross(const Vector& lhs, const Vector& rhs) noexcept
    {
        Vector result{};
        result.x = lhs.y * rhs.z - lhs.z * rhs.y;
        result.y = lhs.z * rhs.x - lhs.x * rhs.z;
        result.z = lhs.x * rhs.y - lhs.y * rhs.x;
        return result;
    }

    template <class Vector>
    [[nodiscard]] constexpr auto lengthSquared(const Vector& value) noexcept
    {
        return dot(value, value);
    }

    // Plain Euclidean length and distance. A non-finite input gives a
    // non-finite result; a caller that needs a fallback states it itself.
    template <class Vector>
    [[nodiscard]] inline auto length(const Vector& value) noexcept
    {
        // Qualified so a caller namespace's own lengthSquared is never picked up through the argument type.
        return std::sqrt(vector_math::lengthSquared(value));
    }

    template <class Vector>
    [[nodiscard]] inline auto distance(const Vector& lhs, const Vector& rhs) noexcept
    {
        const auto x = lhs.x - rhs.x;
        const auto y = lhs.y - rhs.y;
        const auto z = lhs.z - rhs.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    template <class Vector>
    [[nodiscard]] constexpr bool hasFiniteComponents(const Vector& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }
}
