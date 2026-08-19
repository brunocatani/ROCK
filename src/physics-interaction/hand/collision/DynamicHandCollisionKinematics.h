#pragma once

/*
 * Render-authority mapping for dynamic forearm contacts. FRIK consumes a hand
 * target, not a forearm target: translating the hand target moves a point on
 * the forearm by only part of that amount. The radial shoulder-to-point ratio
 * is the local rigid-arm lever approximation, so its inverse maps a blocked
 * forearm displacement back into the hand-target displacement needed to keep
 * that point out of the surface.
 */

#include <algorithm>
#include <cmath>

namespace rock::dynamic_hand_collision_kinematics
{
    inline constexpr float kMinimumHandTargetResponseScale = 1.0f;
    inline constexpr float kMaximumHandTargetResponseScale = 2.5f;

    template <class Vector>
    [[nodiscard]] inline float distance(const Vector& lhs, const Vector& rhs) noexcept
    {
        const float x = lhs.x - rhs.x;
        const float y = lhs.y - rhs.y;
        const float z = lhs.z - rhs.z;
        const float lengthSquared = x * x + y * y + z * z;
        return std::isfinite(lengthSquared) && lengthSquared >= 0.0f ? std::sqrt(lengthSquared) : 0.0f;
    }

    template <class Vector>
    [[nodiscard]] inline float forearmHandTargetResponseScale(
        const Vector& shoulderWorld,
        const Vector& handWorld,
        const Vector& forearmCenterWorld) noexcept
    {
        constexpr float kMinimumReachGameUnits = 0.25f;
        const float handReach = distance(handWorld, shoulderWorld);
        const float forearmReach = distance(forearmCenterWorld, shoulderWorld);
        if (handReach < kMinimumReachGameUnits || forearmReach < kMinimumReachGameUnits) {
            return kMinimumHandTargetResponseScale;
        }

        const float scale = handReach / forearmReach;
        return std::isfinite(scale) ?
                   std::clamp(scale, kMinimumHandTargetResponseScale, kMaximumHandTargetResponseScale) :
                   kMinimumHandTargetResponseScale;
    }

    [[nodiscard]] inline float sanitizeHandTargetResponseScale(float scale) noexcept
    {
        return std::isfinite(scale) ?
                   std::clamp(scale, kMinimumHandTargetResponseScale, kMaximumHandTargetResponseScale) :
                   kMinimumHandTargetResponseScale;
    }
}
