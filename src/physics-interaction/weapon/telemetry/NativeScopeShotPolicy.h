#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::native_scope_shot_policy
{
    inline constexpr std::uint64_t kSampleMilliseconds = 250;
    inline constexpr std::uint64_t kDisplayMilliseconds = 8000;
    inline constexpr float kGuideLengthGameUnits = 1500.0f;
    inline constexpr float kRadiansToDegrees = 57.2957795131f;

    struct Point { float x{}, y{}, z{}; };
    struct Ray { Point origin{}, direction{}; bool valid{}; };

    inline bool finite(Point p) noexcept
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }
    inline Point subtract(Point a, Point b) noexcept { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline float dot(Point a, Point b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Ray ray(Point origin, Point direction) noexcept
    {
        const float length = std::sqrt(dot(direction, direction));
        if (!finite(origin) || !finite(direction) || !std::isfinite(length) || length < 0.0001f) return {};
        return { origin, { direction.x / length, direction.y / length, direction.z / length }, true };
    }
    inline Point end(const Ray& r, float length = kGuideLengthGameUnits) noexcept
    {
        return { r.origin.x + r.direction.x * length, r.origin.y + r.direction.y * length, r.origin.z + r.direction.z * length };
    }
    // Inverse of FO4VR 0x14113A6D0: yaw=atan2(x,y), pitch=-asin(z/length).
    // Native 0x1411395C0 independently constructs +Y rotated by -pitch/-yaw.
    inline Ray launchRay(Point origin, float yaw, float pitch) noexcept
    {
        if (!std::isfinite(yaw) || !std::isfinite(pitch)) return {};
        return ray(origin, { std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch), -std::sin(pitch) });
    }
    inline float angleDegrees(const Ray& a, const Ray& b) noexcept
    {
        if (!a.valid || !b.valid) return -1.0f;
        return std::acos(std::clamp(dot(a.direction, b.direction), -1.0f, 1.0f)) * kRadiansToDegrees;
    }
    inline bool fresh(std::uint64_t now, std::uint64_t then, std::uint64_t limit) noexcept
    {
        return then != 0 && now >= then && now - then <= limit;
    }
    struct AngularOffset { float rightDegrees{}, upDegrees{}; bool valid{}; };
    inline AngularOffset angularOffset(const Ray& sight, Point right, Point up, const Ray& other) noexcept
    {
        if (!sight.valid || !other.valid || !finite(right) || !finite(up)) return {};
        const float forward = dot(sight.direction, other.direction);
        if (forward <= 0.0001f) return {};
        return { std::atan2(dot(right, other.direction), forward) * kRadiansToDegrees,
            std::atan2(dot(up, other.direction), forward) * kRadiansToDegrees, true };
    }
}
