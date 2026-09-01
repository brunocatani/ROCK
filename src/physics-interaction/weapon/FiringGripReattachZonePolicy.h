#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"

#include <cmath>
#include <cstdint>

namespace rock::firing_grip_reattach_zone_policy
{
    using Vec3 = authored_weapon_grip_activation_policy::Vec3;

    /*
     * Firing-grip reattach zone: the free palm re-takes the grip only when it
     * approaches the captured grip point from a SIDE of the weapon. Two
     * opposed cones share the lateral axis through the grip point, each
     * kConeApertureDegrees wide, and the configured reattach radius clips
     * them as a sphere centered on the grip point. Either free hand may
     * squeeze from either side (ambidextrous takeover), so the gate is
     * symmetric in the lateral sign and only reports which side was used.
     *
     * Inside the minimum direction distance the radial vector no longer
     * carries a usable direction; the caller supplies the last stable
     * approach direction captured further out, exactly like the authored
     * support activation gate. Without one the zone fails closed.
     */
    inline constexpr float kConeApertureDegrees = 20.0f;
    inline constexpr float kConeHalfAngleDegrees = kConeApertureDegrees * 0.5f;
    // cos(kConeHalfAngleDegrees)
    inline constexpr float kConeMinimumDot = 0.98480775301220805936f;
    inline constexpr float kMinimumDirectionDistanceGameUnits =
        authored_weapon_grip_activation_policy::
            kMinimumDirectionDistanceGameUnits;
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
        Vec3 lastStableDirectionWorld{};
        float radialCapGameUnits{ 0.0f };
        bool lastStableDirectionValid{ false };
    };

    struct ZoneResult
    {
        Vec3 approachDirectionWorld{};
        float radialDistanceGameUnits{ 0.0f };
        // Signed lateral component of the approach; positive is weapon left.
        float lateralDot{ 0.0f };
        Side side{ Side::None };
        bool directionValid{ false };
        bool usedLastStableDirection{ false };
        bool radialPass{ false };
        bool directionPass{ false };
        bool inside{ false };
        // Indicator marker seat: the grip point pushed out along the side the
        // palm entered from, so the marker never sits inside the weapon.
        Vec3 indicatorWorld{};
        bool indicatorValid{ false };
    };

    struct ConeBoundaryDimensions
    {
        float axialGameUnits{ 0.0f };
        float rimRadiusGameUnits{ 0.0f };
        bool valid{ false };
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

        const Vec3 radialVector =
            activation::subtract(input.palmWorld, input.gripWorld);
        result.radialDistanceGameUnits = activation::length(radialVector);
        result.radialPass =
            std::isfinite(input.radialCapGameUnits) &&
            input.radialCapGameUnits >= 0.0f &&
            result.radialDistanceGameUnits <= input.radialCapGameUnits;

        if (result.radialDistanceGameUnits >=
            kMinimumDirectionDistanceGameUnits) {
            result.directionValid = activation::tryNormalize(
                radialVector,
                result.approachDirectionWorld);
        } else if (input.lastStableDirectionValid) {
            result.directionValid = activation::tryNormalize(
                input.lastStableDirectionWorld,
                result.approachDirectionWorld);
            result.usedLastStableDirection = result.directionValid;
        }

        Vec3 lateralAxis{};
        if (result.directionValid &&
            activation::tryNormalize(input.weaponLeftAxisWorld, lateralAxis)) {
            const float lateralDot =
                activation::dot(result.approachDirectionWorld, lateralAxis);
            if (std::isfinite(lateralDot)) {
                result.lateralDot = lateralDot;
                if (std::abs(lateralDot) >= kConeMinimumDot) {
                    result.directionPass = true;
                    result.side = lateralDot > 0.0f ? Side::Left : Side::Right;
                    const float indicatorSign =
                        result.side == Side::Left ? 1.0f : -1.0f;
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
            }
        }

        result.inside = result.radialPass && result.directionPass;
        return result;
    }

    /*
     * Debug boundary of one cone clipped by the reattach radius: the rim lies
     * on the clipping sphere, so the cone height is the radius scaled by the
     * half-angle cosine and the rim radius by its sine.
     */
    [[nodiscard]] inline ConeBoundaryDimensions resolveConeBoundaryDimensions(
        const float radialCapGameUnits)
    {
        if (!std::isfinite(radialCapGameUnits) || radialCapGameUnits <= 0.0f) {
            return {};
        }
        const float rimFactorSquared = 1.0f - kConeMinimumDot * kConeMinimumDot;
        if (!std::isfinite(rimFactorSquared) || rimFactorSquared < 0.0f) {
            return {};
        }
        const float axialGameUnits = radialCapGameUnits * kConeMinimumDot;
        const float rimRadiusGameUnits =
            radialCapGameUnits * std::sqrt(rimFactorSquared);
        return ConeBoundaryDimensions{
            .axialGameUnits = axialGameUnits,
            .rimRadiusGameUnits = rimRadiusGameUnits,
            .valid = std::isfinite(axialGameUnits) &&
                     std::isfinite(rimRadiusGameUnits),
        };
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
