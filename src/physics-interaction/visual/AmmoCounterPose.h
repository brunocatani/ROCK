#pragma once

#include <algorithm>
#include <array>
#include <cmath>

/*
 * Ammo counter placement on the back of a firing hand, with DIAMOND's
 * visibility behavior: the panels are fixed to the hand (looking never moves
 * or rotates them), face outward from the back of the hand, and fade in only
 * while that side faces the head: hidden at 60 degrees, the subtle full
 * opacity at 45. Like the vanilla counter, the loaded rounds sit left of the
 * anchor and the reserve, smaller, to its right with aligned bottoms.
 *
 * Panels are RPS UI number overlays: width/height is the overlay contract's
 * aspect and the digits fill 85% of the height, centered in the panel.
 */
namespace rock::ammo_counter_pose
{
    using Point = std::array<float, 3>;

    inline constexpr float kNumberAspect = 1024.0f / 480.0f;
    inline constexpr float kDigitHeightFraction = 0.85f;
    inline constexpr float kLoadedWidth = 2.6f;
    inline constexpr float kReserveWidth = 1.6f;
    // Panel-plane offsets of each number's center from the anchor.
    inline constexpr float kLoadedRight = -1.0f;
    inline constexpr float kReserveRight = 1.2f;
    // Anchor: from the wrist bone along the fingers, then out of the back.
    inline constexpr float kAlongFingers = 4.0f;
    inline constexpr float kAboveBack = 2.0f;
    inline constexpr float kMaxOpacity = 0.55f;

    struct Panel
    {
        Point center{};
        float width{ 0.0f };
        float height{ 0.0f };
    };

    struct Pose
    {
        Point right{};
        Point up{};
        Point front{};
        Panel loaded{};
        Panel reserve{};
        float opacity{ 0.0f };
        bool valid{ false };
    };

    namespace detail
    {
        inline bool normalize(Point& value) noexcept
        {
            const float length = std::hypot(value[0], value[1], value[2]);
            if (!std::isfinite(length) || length < 0.0001f) return false;
            for (auto& component : value) component /= length;
            return true;
        }

        inline float dot(const Point& a, const Point& b) noexcept
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        inline Point cross(const Point& a, const Point& b) noexcept
        {
            return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        }

        inline Point offset(const Point& origin, const Point& axisA, float a, const Point& axisB, float b) noexcept
        {
            return { origin[0] + axisA[0] * a + axisB[0] * b, origin[1] + axisA[1] * a + axisB[1] * b,
                origin[2] + axisA[2] * a + axisB[2] * b };
        }
    }

    // fingers and backOfHand are world directions of the rendered hand; up
    // follows the fingers within the back-of-hand plane.
    [[nodiscard]] inline Pose resolve(const Point& wrist, const Point& fingers, const Point& backOfHand,
        const Point& head) noexcept
    {
        using namespace detail;
        for (unsigned i = 0; i < 3; ++i) {
            if (!std::isfinite(wrist[i]) || !std::isfinite(head[i])) return {};
        }
        Pose pose{};
        pose.front = backOfHand;
        if (!normalize(pose.front)) return {};
        const float along = dot(fingers, pose.front);
        pose.up = { fingers[0] - pose.front[0] * along, fingers[1] - pose.front[1] * along,
            fingers[2] - pose.front[2] * along };
        if (!normalize(pose.up)) return {};
        pose.right = cross(pose.up, pose.front);
        if (!normalize(pose.right)) return {};

        const Point anchor = offset(wrist, pose.up, kAlongFingers, pose.front, kAboveBack);
        pose.loaded = { offset(anchor, pose.right, kLoadedRight, pose.up, 0.0f), kLoadedWidth, kLoadedWidth / kNumberAspect };
        const float reserveDrop = (pose.loaded.height - kReserveWidth / kNumberAspect) * kDigitHeightFraction * 0.5f;
        pose.reserve = { offset(anchor, pose.right, kReserveRight, pose.up, -reserveDrop), kReserveWidth,
            kReserveWidth / kNumberAspect };

        Point toHead{ head[0] - anchor[0], head[1] - anchor[1], head[2] - anchor[2] };
        if (!normalize(toHead)) return {};
        pose.opacity = kMaxOpacity * std::clamp((dot(pose.front, toHead) - 0.5f) / (0.70710678f - 0.5f), 0.0f, 1.0f);
        pose.valid = true;
        return pose;
    }
}
