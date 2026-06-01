#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "RE/NetImmerse/NiPoint.h"

namespace rock::selection_beam_policy
{
    inline constexpr std::size_t kSegmentCount = 18;
    inline constexpr float kMinVisibleLengthGameUnits = 6.0f;
    inline constexpr float kMaxVisibleLengthGameUnits = 600.0f;
    inline constexpr float kDefaultSegmentSizeGameUnits = 1.15f;
    inline constexpr float kDefaultCurveLiftGameUnits = 8.0f;
    inline constexpr float kDefaultAlpha = 0.72f;

    struct Config
    {
        bool enabled = true;
        float segmentSizeGameUnits = kDefaultSegmentSizeGameUnits;
        float curveLiftGameUnits = kDefaultCurveLiftGameUnits;
        float alpha = kDefaultAlpha;
    };

    struct Frame
    {
        bool active = false;
        RE::NiPoint3 startWorld{};
        RE::NiPoint3 endWorld{};
        Config config{};
    };

    [[nodiscard]] inline bool isFinitePoint(const RE::NiPoint3& point) noexcept
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    [[nodiscard]] inline float distanceGameUnits(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
    {
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        const float dz = lhs.z - rhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    [[nodiscard]] inline Config sanitizeConfig(Config config) noexcept
    {
        if (!std::isfinite(config.segmentSizeGameUnits)) {
            config.segmentSizeGameUnits = kDefaultSegmentSizeGameUnits;
        }
        if (!std::isfinite(config.curveLiftGameUnits)) {
            config.curveLiftGameUnits = kDefaultCurveLiftGameUnits;
        }
        if (!std::isfinite(config.alpha)) {
            config.alpha = kDefaultAlpha;
        }

        config.segmentSizeGameUnits = std::clamp(config.segmentSizeGameUnits, 0.2f, 6.0f);
        config.curveLiftGameUnits = std::clamp(config.curveLiftGameUnits, 0.0f, 80.0f);
        config.alpha = std::clamp(config.alpha, 0.05f, 1.0f);
        return config;
    }

    [[nodiscard]] inline bool shouldRender(const Frame& frame) noexcept
    {
        if (!frame.active || !frame.config.enabled || !isFinitePoint(frame.startWorld) || !isFinitePoint(frame.endWorld)) {
            return false;
        }

        const float distance = distanceGameUnits(frame.startWorld, frame.endWorld);
        return std::isfinite(distance) && distance >= kMinVisibleLengthGameUnits && distance <= kMaxVisibleLengthGameUnits;
    }

    [[nodiscard]] inline RE::NiPoint3 makeControlPoint(const RE::NiPoint3& startWorld, const RE::NiPoint3& endWorld, float curveLiftGameUnits) noexcept
    {
        const float distance = distanceGameUnits(startWorld, endWorld);
        const float lift = std::min(std::max(0.0f, curveLiftGameUnits), distance * 0.30f);
        return RE::NiPoint3{
            (startWorld.x + endWorld.x) * 0.5f,
            (startWorld.y + endWorld.y) * 0.5f,
            (startWorld.z + endWorld.z) * 0.5f + lift,
        };
    }

    [[nodiscard]] inline RE::NiPoint3 sampleQuadraticBezier(
        const RE::NiPoint3& startWorld,
        const RE::NiPoint3& controlWorld,
        const RE::NiPoint3& endWorld,
        float t) noexcept
    {
        const float clampedT = std::clamp(t, 0.0f, 1.0f);
        const float invT = 1.0f - clampedT;
        const float startWeight = invT * invT;
        const float controlWeight = 2.0f * invT * clampedT;
        const float endWeight = clampedT * clampedT;
        return RE::NiPoint3{
            startWorld.x * startWeight + controlWorld.x * controlWeight + endWorld.x * endWeight,
            startWorld.y * startWeight + controlWorld.y * controlWeight + endWorld.y * endWeight,
            startWorld.z * startWeight + controlWorld.z * controlWeight + endWorld.z * endWeight,
        };
    }
}
