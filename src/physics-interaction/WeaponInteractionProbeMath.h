#pragma once

#include <algorithm>
#include <cmath>

namespace frik::rock::weapon_interaction_probe_math
{
    /*
     * HIGGS drives two-handed weapon selection from an active near-hand search
     * instead of relying on solver contact events. ROCK's generated hand and
     * weapon bodies are both keyframed, so FO4VR may not report a contact even
     * when the left hand is visually touching the weapon. This helper keeps the
     * proximity test explicit and testable.
     */
    template <class Vector>
    inline float pointAabbDistanceSquared(const Vector& point, const Vector& boundsMin, const Vector& boundsMax)
    {
        const float clampedX = (std::max)(boundsMin.x, (std::min)(point.x, boundsMax.x));
        const float clampedY = (std::max)(boundsMin.y, (std::min)(point.y, boundsMax.y));
        const float clampedZ = (std::max)(boundsMin.z, (std::min)(point.z, boundsMax.z));

        const float dx = point.x - clampedX;
        const float dy = point.y - clampedY;
        const float dz = point.z - clampedZ;
        return dx * dx + dy * dy + dz * dz;
    }

    inline bool isWithinProbeRadiusSquared(float distanceSquared, float radius)
    {
        const float safeRadius = (std::max)(0.0f, radius);
        return distanceSquared <= safeRadius * safeRadius;
    }
}
