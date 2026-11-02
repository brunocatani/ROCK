#pragma once

#include "physics-interaction/weapon/authored/AuthoredWeaponGripActivationPolicy.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::firing_grip_reattach_zone_policy
{
    using Vec3 = authored_weapon_grip_activation_policy::Vec3;

    /*
     * Firing-grip reattach zone, one region per side of the weapon around the
     * captured grip point. The free palm re-takes the grip only while it sits
     * inside one of them. Either free hand may squeeze from either side
     * (ambidextrous takeover); the gate is symmetric in the lateral sign and
     * only reports which side the palm sits on. Neither shape needs approach
     * history.
     *
     * Sector (default): a quarter disc in the plane of the lateral and grip
     * DOWN axes, swept from straight out to straight down the grip, so a palm
     * arriving from the side or from below the grip both count. Its top edge
     * sits the cross radius above the grip point (the grip top on a pistol);
     * the cross radius also bounds it front and back along the barrel.
     * Cylinder (handling-provider opt-in): a cylinder along the lateral axis,
     * the reach long and the cross radius wide.
     */
    inline constexpr float kIndicatorOffsetGameUnits =
        authored_weapon_grip_activation_policy::kIndicatorOffsetGameUnits;
    inline constexpr float kAxisOrthogonalityTolerance = 0.05f;

    enum class Shape : std::uint8_t
    {
        Sector,
        Cylinder,
    };

    enum class Side : std::uint8_t
    {
        None,
        Left,
        Right,
    };

    struct ZoneInput
    {
        Vec3 gripWorld{};
        Vec3 palmWorld{};
        // Weapon LEFT at the grip: the seated canonical right palm normal.
        Vec3 weaponLeftAxisWorld{};
        // Down the grip from the same seated hand; used by the sector only.
        Vec3 weaponDownAxisWorld{};
        // Sector radius, or cylinder length from the grip along each side.
        float reachGameUnits{ 0.0f };
        // Cylinder cross-section radius; sector rise above the grip point
        // and half-thickness along the barrel.
        float radiusGameUnits{ 0.0f };
        Shape shape{ Shape::Sector };
    };

    struct ZoneResult
    {
        // Signed distance along the lateral axis; positive is weapon left.
        float alongAxisGameUnits{ 0.0f };
        float perpendicularDistanceGameUnits{ 0.0f };
        float radialDistanceGameUnits{ 0.0f };
        // Sector only: distance down the grip (negative above the grip
        // point) and along the barrel.
        float downGameUnits{ 0.0f };
        float forwardGameUnits{ 0.0f };
        Side side{ Side::None };
        bool axisValid{ false };
        // Cylinder: within the length. Sector: within the quarter disc.
        bool reachPass{ false };
        // Cylinder: within the cross radius. Sector: within the top edge and
        // the front/back thickness.
        bool radiusPass{ false };
        bool inside{ false };
        // Indicator marker seat: the grip point pushed out along the side the
        // palm sits on, so the marker never sits inside the weapon.
        Vec3 indicatorWorld{};
        bool indicatorValid{ false };
    };

    [[nodiscard]] inline bool isFinite(const Vec3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    [[nodiscard]] constexpr Vec3 cross(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    [[nodiscard]] inline ZoneResult evaluateZone(const ZoneInput& input)
    {
        namespace activation = authored_weapon_grip_activation_policy;
        ZoneResult result{};
        if (!isFinite(input.gripWorld) || !isFinite(input.palmWorld)) {
            return result;
        }
        const bool reachValid = std::isfinite(input.reachGameUnits) && input.reachGameUnits >= 0.0f;
        const bool radiusValid = std::isfinite(input.radiusGameUnits) && input.radiusGameUnits >= 0.0f;
        Vec3 lateralAxis{};
        result.axisValid = activation::tryNormalize(input.weaponLeftAxisWorld, lateralAxis);
        // The sector's DOWN must come from the same seated hand frame as the
        // lateral axis; a skewed pair means the frame is not a rotation.
        Vec3 downAxis{};
        if (result.axisValid && input.shape == Shape::Sector) {
            result.axisValid = activation::tryNormalize(input.weaponDownAxisWorld, downAxis) &&
                std::abs(activation::dot(lateralAxis, downAxis)) <= kAxisOrthogonalityTolerance;
        }
        if (!reachValid || !radiusValid || !result.axisValid) {
            return result;
        }

        const Vec3 offset = activation::subtract(input.palmWorld, input.gripWorld);
        result.radialDistanceGameUnits = activation::length(offset);
        const float along = activation::dot(offset, lateralAxis);
        const Vec3 perpendicular{
            offset.x - lateralAxis.x * along,
            offset.y - lateralAxis.y * along,
            offset.z - lateralAxis.z * along,
        };
        const float perpendicularDistance = activation::length(perpendicular);
        if (!std::isfinite(along) || !std::isfinite(perpendicularDistance)) {
            return result;
        }
        result.alongAxisGameUnits = along;
        result.perpendicularDistanceGameUnits = perpendicularDistance;
        result.side = along < 0.0f ? Side::Right : Side::Left;

        if (input.shape == Shape::Sector) {
            const float down = activation::dot(offset, downAxis);
            const float forward = activation::dot(offset, cross(lateralAxis, downAxis));
            if (!std::isfinite(down) || !std::isfinite(forward)) {
                return result;
            }
            result.downGameUnits = down;
            result.forwardGameUnits = forward;
            // Above the grip point the disc continues as a band up to the
            // top edge, so the outline runs straight down before it curves.
            const float discDown = (std::max)(down, 0.0f);
            result.reachPass = along * along + discDown * discDown <= input.reachGameUnits * input.reachGameUnits;
            result.radiusPass = down >= -input.radiusGameUnits && std::abs(forward) <= input.radiusGameUnits;
        } else {
            result.reachPass = std::abs(along) <= input.reachGameUnits;
            result.radiusPass = perpendicularDistance <= input.radiusGameUnits;
        }
        result.inside = result.reachPass && result.radiusPass;
        if (result.inside) {
            const float indicatorSign = result.side == Side::Left ? 1.0f : -1.0f;
            result.indicatorWorld = Vec3{
                input.gripWorld.x + lateralAxis.x * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.y + lateralAxis.y * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.z + lateralAxis.z * indicatorSign * kIndicatorOffsetGameUnits,
            };
            result.indicatorValid = isFinite(result.indicatorWorld);
        }
        return result;
    }

    [[nodiscard]] constexpr const char* sideName(const Side side)
    {
        switch (side) {
        case Side::Left:
            return "LEFT";
        case Side::Right:
            return "RIGHT";
        case Side::None:
        default:
            return "NONE";
        }
    }

    [[nodiscard]] constexpr const char* shapeName(const Shape shape)
    {
        switch (shape) {
        case Shape::Cylinder:
            return "CYLINDER";
        case Shape::Sector:
        default:
            return "SECTOR";
        }
    }
}
