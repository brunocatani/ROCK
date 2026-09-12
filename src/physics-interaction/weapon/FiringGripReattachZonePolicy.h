#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"

#include <cmath>
#include <cstdint>

namespace rock::firing_grip_reattach_zone_policy
{
    using Vec3 = authored_weapon_grip_activation_policy::Vec3;

    /*
     * Firing-grip reattach zone: two cylinders start at the captured grip
     * point and run out along the weapon's lateral axis, one per side. The
     * free palm re-takes the grip only while it sits inside one of them:
     * within the configured reach along the axis and within the configured
     * radius of that axis. A cylinder keeps the same tolerance at the grip
     * as at the end of the reach, so no approach history is needed. Either
     * free hand may squeeze from either side (ambidextrous takeover); the
     * gate is symmetric in the lateral sign and only reports which side the
     * palm sits on.
     */
    inline constexpr float kIndicatorOffsetGameUnits =
        authored_weapon_grip_activation_policy::kIndicatorOffsetGameUnits;

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
        // Length of each cylinder from the grip point along its side.
        float reachGameUnits{ 0.0f };
        // Cross-section radius shared by both cylinders.
        float radiusGameUnits{ 0.0f };
    };

    struct ZoneResult
    {
        // Signed distance along the lateral axis; positive is weapon left.
        float alongAxisGameUnits{ 0.0f };
        float perpendicularDistanceGameUnits{ 0.0f };
        float radialDistanceGameUnits{ 0.0f };
        Side side{ Side::None };
        bool axisValid{ false };
        bool reachPass{ false };
        bool radiusPass{ false };
        bool inside{ false };
        // Indicator marker seat: the grip point pushed out along the side the
        // palm sits on, so the marker never sits inside the weapon.
        Vec3 indicatorWorld{};
        bool indicatorValid{ false };
    };

    [[nodiscard]] inline bool isFinite(const Vec3& value)
    {
        return std::isfinite(value.x) &&
               std::isfinite(value.y) &&
               std::isfinite(value.z);
    }

    [[nodiscard]] inline ZoneResult evaluateZone(const ZoneInput& input)
    {
        namespace activation = authored_weapon_grip_activation_policy;
        ZoneResult result{};
        if (!isFinite(input.gripWorld) || !isFinite(input.palmWorld)) {
            return result;
        }
        const bool reachValid =
            std::isfinite(input.reachGameUnits) && input.reachGameUnits >= 0.0f;
        const bool radiusValid =
            std::isfinite(input.radiusGameUnits) && input.radiusGameUnits >= 0.0f;
        Vec3 lateralAxis{};
        result.axisValid =
            activation::tryNormalize(input.weaponLeftAxisWorld, lateralAxis);
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
        result.reachPass = std::abs(along) <= input.reachGameUnits;
        result.radiusPass = perpendicularDistance <= input.radiusGameUnits;
        result.inside = result.reachPass && result.radiusPass;
        if (result.inside) {
            const float indicatorSign = result.side == Side::Left ? 1.0f : -1.0f;
            result.indicatorWorld = Vec3{
                input.gripWorld.x +
                    lateralAxis.x * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.y +
                    lateralAxis.y * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.z +
                    lateralAxis.z * indicatorSign * kIndicatorOffsetGameUnits,
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
}
