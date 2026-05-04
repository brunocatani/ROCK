#pragma once

#include <algorithm>
#include <cmath>

namespace frik::rock::selected_close_finger_policy
{
    /*
     * HIGGS only pre-curls fingers around a close-selected object when the hand
     * is moving slowly enough. ROCK keeps the same behavior as a policy helper
     * so config parsing, selection state, and FRIK hand-pose publishing cannot
     * drift apart.
     */
    inline bool shouldApplyPreCurl(bool enabled, bool isSelectedClose, bool selectionValid, float handSpeedMetersPerSecond, float maxHandSpeedMetersPerSecond)
    {
        if (!enabled || !isSelectedClose || !selectionValid) {
            return false;
        }
        if (!std::isfinite(handSpeedMetersPerSecond)) {
            return false;
        }
        if (maxHandSpeedMetersPerSecond <= 0.0f) {
            return true;
        }
        return handSpeedMetersPerSecond <= maxHandSpeedMetersPerSecond;
    }

    inline float estimateHandSpeedMetersPerSecond(float distanceGameUnits, float deltaTimeSeconds, float gameUnitsPerMeter)
    {
        if (!std::isfinite(distanceGameUnits) || !std::isfinite(deltaTimeSeconds) || !std::isfinite(gameUnitsPerMeter) || deltaTimeSeconds <= 0.0f ||
            gameUnitsPerMeter <= 0.0f) {
            return 0.0f;
        }
        return (std::max)(0.0f, distanceGameUnits) / gameUnitsPerMeter / deltaTimeSeconds;
    }
}

