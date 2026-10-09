#pragma once
#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"
#include <cmath>
namespace rock::firing_grip_reattach_zone_policy
{
    using Vec3 = authored_weapon_grip_activation_policy::Vec3;
    struct ZoneInput { Vec3 gripWorld{}, palmWorld{}; float radiusGameUnits{}; };
    struct ZoneResult {
        float radialDistanceGameUnits{};
        bool valid{}, inside{};
        Vec3 indicatorWorld{};
        bool indicatorValid{};
    };
    inline ZoneResult evaluateZone(const ZoneInput& input)
    {
        const auto finite = [](Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
        ZoneResult result{};
        result.valid = finite(input.gripWorld) && finite(input.palmWorld) &&
            std::isfinite(input.radiusGameUnits) && input.radiusGameUnits > 0.0f;
        if (!result.valid) return result;
        const auto delta = authored_weapon_grip_activation_policy::subtract(input.palmWorld, input.gripWorld);
        const float squared = authored_weapon_grip_activation_policy::dot(delta, delta);
        if (!std::isfinite(squared)) { result.valid = false; return result; }
        result.radialDistanceGameUnits = std::sqrt(squared);
        result.inside = result.radialDistanceGameUnits <= input.radiusGameUnits;
        result.indicatorWorld = input.gripWorld;
        result.indicatorValid = result.inside;
        return result;
    }
}
