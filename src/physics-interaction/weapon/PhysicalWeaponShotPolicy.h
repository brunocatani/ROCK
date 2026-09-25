#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"

namespace rock::physical_weapon_shot_policy
{
    struct Aim
    {
        native_scope_shot_policy::Ray ray{};
        float yaw{}, pitch{};
    };

    template<class Transform>
    Aim muzzleAim(const Transform& world) noexcept
    {
        using Point = decltype(world.translate);
        // Ni stores the local basis in rows. Use the same local-to-world
        // convention as the mesh, grip and collision transforms.
        const auto forward = transform_math::localVectorToWorld(world, Point{0.0f, 1.0f, 0.0f});
        Aim result{};
        result.ray = native_scope_shot_policy::ray({world.translate.x, world.translate.y, world.translate.z},
            {forward.x, forward.y, forward.z});
        if (result.ray.valid) {
            const auto& d = result.ray.direction;
            result.yaw = std::atan2(d.x, d.y);
            result.pitch = -std::atan2(d.z, std::sqrt(d.x*d.x + d.y*d.y));
        }
        return result;
    }
}
