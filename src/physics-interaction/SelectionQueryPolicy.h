#pragma once

#include <cstdint>

namespace frik::rock::selection_query_policy
{
    constexpr std::uint32_t kDefaultShapeCastFilterInfo = 0x000B002D;
    constexpr std::uint32_t kDefaultFarClipRayFilterInfo = 0x02420028;

    inline std::uint32_t sanitizeFilterInfo(std::uint32_t configuredValue, std::uint32_t fallback)
    {
        return configuredValue != 0 ? configuredValue : fallback;
    }

    inline bool shouldReplaceSelectionForSameRef(bool currentIsFarSelection, bool nextIsFarSelection, std::uint32_t currentBodyId, std::uint32_t nextBodyId)
    {
        return currentIsFarSelection != nextIsFarSelection || currentBodyId != nextBodyId;
    }

    inline bool isBetterShapeCastCandidate(bool isFarSelection, float lateralDistance, float alongDistance, float bestLateralDistance, float bestAlongDistance)
    {
        constexpr float kTieEpsilon = 0.001f;
        if (!isFarSelection) {
            if (alongDistance < bestAlongDistance - kTieEpsilon) {
                return true;
            }
            if (alongDistance > bestAlongDistance + kTieEpsilon) {
                return false;
            }
        }
        if (lateralDistance < bestLateralDistance - kTieEpsilon) {
            return true;
        }
        if (lateralDistance > bestLateralDistance + kTieEpsilon) {
            return false;
        }
        return isFarSelection && alongDistance < bestAlongDistance;
    }

    inline bool shouldPromoteFarHitToClose(float hitDistanceFromHand, float nearReachDistance)
    {
        return nearReachDistance > 0.0f && hitDistanceFromHand <= nearReachDistance;
    }

    inline bool shouldKeepSelectionAfterMiss(bool currentIsFarSelection, int heldFrames, int minimumHoldFrames, float currentDistance, float hysteresisRange)
    {
        if (heldFrames < minimumHoldFrames) {
            return true;
        }

        if (currentIsFarSelection) {
            return false;
        }

        return currentDistance <= hysteresisRange;
    }
}
