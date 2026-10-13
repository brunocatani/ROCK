#pragma once
#include <cmath>

namespace rock::equipped_drop_pickup_policy
{
    struct RayAdmission { bool valid{}; float along{}; float lateral{}; };

    template<class Point>
    RayAdmission rayAdmission(const Point& origin, const Point& direction, const Point& point,
        float maximumDistance, float radius)
    {
        const float length = std::hypot(direction.x, direction.y, direction.z);
        if (!std::isfinite(length) || length < 0.0001f || !std::isfinite(maximumDistance) ||
            !std::isfinite(radius) || maximumDistance <= 0.0f || radius < 0.0f) return {};
        const float x = point.x - origin.x, y = point.y - origin.y, z = point.z - origin.z;
        const float along = (x * direction.x + y * direction.y + z * direction.z) / length;
        const float lateral = std::hypot(x - direction.x * along / length,
            y - direction.y * along / length, z - direction.z * along / length);
        return { std::isfinite(along) && std::isfinite(lateral) && along >= 0.0f &&
            along <= maximumDistance && lateral <= radius, along, lateral };
    }

    inline constexpr bool canBypassGripZone(bool equippedDrop, bool selectedCatch) noexcept
    {
        return equippedDrop && selectedCatch;
    }
}
