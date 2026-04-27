#pragma once

#include <algorithm>
#include <cmath>

namespace frik::rock::held_object_damping_math
{
    // Held-object smoothing is applied as a velocity attenuation on the already constrained
    // body instead of changing hand/body frames. The grab frame math and pivots remain the
    // source of truth; this only removes carried solver velocity that makes held objects
    // visually chase or snap around the target from one physics update to the next.
    inline float clampVelocityDamping(float damping)
    {
        if (!std::isfinite(damping)) {
            return 0.0f;
        }

        return std::clamp(damping, 0.0f, 1.0f);
    }

    inline float velocityKeepFactor(float damping)
    {
        return 1.0f - clampVelocityDamping(damping);
    }

    template <class Vec3>
    inline Vec3 applyVelocityDamping(const Vec3& velocity, float damping)
    {
        const float keep = velocityKeepFactor(damping);
        Vec3 result = velocity;
        result.x *= keep;
        result.y *= keep;
        result.z *= keep;
        return result;
    }
}
