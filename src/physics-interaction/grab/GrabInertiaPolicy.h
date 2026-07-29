#pragma once

#include <algorithm>
#include <cmath>

namespace rock::grab_inertia_policy
{
    struct InverseInertiaNormalizationResult
    {
        float normalized[3] = { 0.0f, 0.0f, 0.0f };
        float originalRatio = 0.0f;
        float normalizedRatio = 0.0f;
        float ratioLimit = 10.0f;
        float maxInverseInertiaFromMinimum = 100.0f;
        bool valid = false;
        bool modified = false;
    };

    inline InverseInertiaNormalizationResult normalizeInverseInertiaAxesForGrab(
        float invX,
        float invY,
        float invZ,
        float maxInertiaRatio,
        float minInertia)
    {
        InverseInertiaNormalizationResult result{};
        if (!std::isfinite(invX) || !std::isfinite(invY) || !std::isfinite(invZ) || invX <= 0.0f || invY <= 0.0f || invZ <= 0.0f) {
            return result;
        }

        result.valid = true;
        result.normalized[0] = invX;
        result.normalized[1] = invY;
        result.normalized[2] = invZ;
        result.ratioLimit = (std::isfinite(maxInertiaRatio) && maxInertiaRatio >= 1.0f) ? maxInertiaRatio : 10.0f;
        const float safeMinInertia = (std::isfinite(minInertia) && minInertia > 0.0f) ? minInertia : 0.01f;
        result.maxInverseInertiaFromMinimum = 1.0f / safeMinInertia;

        const float minAxis = (std::min)((std::min)(invX, invY), invZ);
        const float maxAxis = (std::max)((std::max)(invX, invY), invZ);
        result.originalRatio = maxAxis / minAxis;
        const float maxAllowedByRatio = minAxis * result.ratioLimit;
        const float maxAllowed = (std::min)(maxAllowedByRatio, result.maxInverseInertiaFromMinimum);

        for (float& axis : result.normalized) {
            const float clamped = (std::min)(axis, maxAllowed);
            if (std::abs(clamped - axis) > 0.0f) {
                result.modified = true;
                axis = clamped;
            }
        }

        const float normalizedMinAxis = (std::min)((std::min)(result.normalized[0], result.normalized[1]), result.normalized[2]);
        const float normalizedMaxAxis = (std::max)((std::max)(result.normalized[0], result.normalized[1]), result.normalized[2]);
        result.normalizedRatio = normalizedMaxAxis / normalizedMinAxis;
        return result;
    }
}
