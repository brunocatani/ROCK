#pragma once

#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock::weapon_physics_time_scale
{
    struct HandlingScale
    {
        float velocity = 1.0f;
        float force = 1.0f;
        float responseDeltaSeconds = 0.0f;
        bool valid = false;
    };

    inline HandlingScale resolve(bool enabled, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        HandlingScale result{};
        float dt = 0.0f;
        if (!havok_physics_timing::tryGetDriveDeltaSeconds(timing, dt)) return result;
        if (enabled) {
            if (!std::isfinite(timing.timeMultiplier) || timing.timeMultiplier <= 0.0f) return result;
            // Compensation is for slow motion. Speed-up and the disabled
            // option retain the existing simulation-time handling profile.
            result.velocity = 1.0f / (std::min)(timing.timeMultiplier, 1.0f);
        }
        result.force = result.velocity * result.velocity;
        result.responseDeltaSeconds = dt * result.velocity;
        result.valid = std::isfinite(result.velocity) && std::isfinite(result.force) &&
            havok_physics_timing::isUsableDelta(result.responseDeltaSeconds);
        return result;
    }

    // v_sim = v_real / s, a_sim = a_real / s^2. Rebase the existing
    // velocity only when the compensation changes, including hot disable.
    // Applying the absolute scale each step would compound velocity forever.
    inline float velocityRebase(float previousScale, float nextScale)
    {
        if (!std::isfinite(previousScale) || previousScale <= 0.0f ||
            !std::isfinite(nextScale) || nextScale <= 0.0f) return 0.0f;
        const float ratio = nextScale / previousScale;
        return std::isfinite(ratio) && ratio > 0.0f ? ratio : 0.0f;
    }

    inline bool rebaseWorldAngularVelocity(const float quaternion[4], const RE::NiPoint3& local,
        float ratio, RE::NiPoint3& world)
    {
        const float lengthSquared = quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] +
            quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3];
        if (!std::isfinite(lengthSquared) || std::abs(lengthSquared - 1.0f) > 0.01f ||
            !std::isfinite(ratio) || ratio <= 0.0f) return false;
        // havokQuaternionToNiRows returns the generated-body matrix. Convert
        // its columns to Ni's stored local axes before local -> world rotation.
        const auto rotation = transform_math::transposeRotation(
            transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion));
        world = transform_math::rotateLocalVectorToWorld(rotation,
            RE::NiPoint3{local.x * ratio, local.y * ratio, local.z * ratio});
        return std::isfinite(world.x) && std::isfinite(world.y) && std::isfinite(world.z);
    }
}
