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
    template<class Matrix>
    inline Ray nodeAxisRay(Point origin, const Matrix& rotation, unsigned axis) noexcept
    {
        if (axis >= 3) return {};
        // FO4VR stores the world-space node axes as padded rows. Native
        // 0x104F3A0 takes ProjectileNode world rotation+0x10 (+Y);
        // 0x1C11B00 independently uses matrix+0x10/+0x14/+0x18 for yaw/pitch.
        const auto& row = rotation.entry[axis];
        return ray(origin, {row[0], row[1], row[2]});
    }
    struct LaunchAngles { float yaw{}, pitch{}; bool valid{}; };
    inline LaunchAngles launchAngles(const Ray& direction) noexcept
    {
        if (!direction.valid || !finite(direction.origin)) return {};
        const auto normalized = ray(direction.origin, direction.direction);
        if (!normalized.valid) return {};
        const auto& d = normalized.direction;
        return {std::atan2(d.x, d.y), -std::atan2(d.z, std::sqrt(d.x*d.x + d.y*d.y)), true};
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
        const double x = static_cast<double>(a.direction.y) * b.direction.z - static_cast<double>(a.direction.z) * b.direction.y;
        const double y = static_cast<double>(a.direction.z) * b.direction.x - static_cast<double>(a.direction.x) * b.direction.z;
        const double z = static_cast<double>(a.direction.x) * b.direction.y - static_cast<double>(a.direction.y) * b.direction.x;
        const double cosine = static_cast<double>(a.direction.x) * b.direction.x + static_cast<double>(a.direction.y) * b.direction.y + static_cast<double>(a.direction.z) * b.direction.z;
        return static_cast<float>(std::atan2(std::sqrt(x * x + y * y + z * z), cosine) * kRadiansToDegrees);
    }
    inline bool fresh(std::uint64_t now, std::uint64_t then, std::uint64_t limit) noexcept
    {
        return then != 0 && now >= then && now - then <= limit;
    }
    struct PlaneOffset { float rightGameUnits{}, upGameUnits{}; Point intersection{}; bool valid{}; };
    // Intersect the forward ray with the plane normal to the sight at one
    // stated depth. Parallel sight/bullet directions retain their bore offset.
    inline PlaneOffset planeOffset(const Ray& sight, Point right, Point up, const Ray& other,
        float depth = kGuideLengthGameUnits) noexcept
    {
        if (!sight.valid || !other.valid || !finite(right) || !finite(up) || !std::isfinite(depth) || depth <= 0) return {};
        const Point cross{ right.y * up.z - right.z * up.y, right.z * up.x - right.x * up.z, right.x * up.y - right.y * up.x };
        if (dot(cross, cross) < 0.000001f) return {};
        const double forward = dot(sight.direction, other.direction);
        if (forward <= 0.0001) return {};
        const auto delta = subtract(other.origin, sight.origin);
        const double t = (depth - static_cast<double>(dot(delta, sight.direction))) / forward;
        if (!std::isfinite(t) || t < 0) return {};
        const double x = delta.x + t * other.direction.x - static_cast<double>(depth) * sight.direction.x;
        const double y = delta.y + t * other.direction.y - static_cast<double>(depth) * sight.direction.y;
        const double z = delta.z + t * other.direction.z - static_cast<double>(depth) * sight.direction.z;
        PlaneOffset result{
            static_cast<float>(x * right.x + y * right.y + z * right.z),
            static_cast<float>(x * up.x + y * up.y + z * up.z),
            { static_cast<float>(other.origin.x + t * other.direction.x), static_cast<float>(other.origin.y + t * other.direction.y), static_cast<float>(other.origin.z + t * other.direction.z) }, true };
        if (!std::isfinite(result.rightGameUnits) || !std::isfinite(result.upGameUnits) || !finite(result.intersection)) return {};
        return result;
    }
}
