#pragma once
#include <cmath>

namespace rock::bipod_retention_policy
{
    template<class Point>
    bool outsideRadius(const Point& observer, const Point& anchor, double gameToMeters, double radiusMeters) noexcept
    {
        if (!std::isfinite(gameToMeters) || gameToMeters <= 0 ||
            !std::isfinite(radiusMeters) || radiusMeters <= 0) return false;
        const double x = (static_cast<double>(observer.x) - anchor.x) * gameToMeters;
        const double y = (static_cast<double>(observer.y) - anchor.y) * gameToMeters;
        const double z = (static_cast<double>(observer.z) - anchor.z) * gameToMeters;
        const double distance = std::hypot(x, y, z);
        return std::isfinite(distance) && distance > radiusMeters;
    }
}
