#pragma once

#include <cmath>

namespace rock::far_pull_gesture
{
    enum class Mode : int { Immediate = 1, Gesture = 2 };
    inline constexpr float kSpeedThresholdMetersPerSecond = 1.2f;

    [[nodiscard]] constexpr int sanitizeMode(int value) noexcept
    {
        return value == static_cast<int>(Mode::Gesture) ? value : static_cast<int>(Mode::Immediate);
    }

    template <class Point>
    [[nodiscard]] float speedAwayFromTarget(const Point& velocityMetersPerSecond,
        const Point& handWorld, const Point& targetWorld) noexcept
    {
        const float x = handWorld.x - targetWorld.x;
        const float y = handWorld.y - targetWorld.y;
        const float z = handWorld.z - targetWorld.z;
        const float lengthSquared = x * x + y * y + z * z;
        const float dot = velocityMetersPerSecond.x * x + velocityMetersPerSecond.y * y + velocityMetersPerSecond.z * z;
        if (!std::isfinite(lengthSquared) || lengthSquared < 0.000001f || !std::isfinite(dot)) return 0.0f;
        return dot / std::sqrt(lengthSquared);
    }

    [[nodiscard]] inline bool confirmsPull(bool held, bool released, bool validMotion, float speed) noexcept
    {
        return held && !released && validMotion && std::isfinite(speed) && speed > kSpeedThresholdMetersPerSecond;
    }
}
