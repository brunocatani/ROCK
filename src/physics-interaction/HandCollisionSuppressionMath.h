#pragma once

#include <cstdint>

namespace frik::rock::hand_collision_suppression_math
{
    /*
     * ROCK suppresses its own hand bodies during physical grabs because the hand body
     * is the constraint driver, not an obstacle the held object should keep solving
     * against. HIGGS gates this with collision-filter bit 14; FO4VR Ghidra analysis
     * confirms that bit still disables hknp pair filtering, while also exposing a
     * separate broadphase body toggle that must be driven by runtime code.
     */
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kNoCollideBit = 1u << 14;

    struct SuppressionState
    {
        bool active = false;
        bool wasNoCollideBeforeSuppression = false;
        std::uint32_t bodyId = kInvalidBodyId;
    };

    inline std::uint32_t beginSuppression(SuppressionState& state, std::uint32_t bodyId, std::uint32_t currentFilter)
    {
        if (!state.active || state.bodyId != bodyId) {
            state.active = true;
            state.wasNoCollideBeforeSuppression = (currentFilter & kNoCollideBit) != 0;
            state.bodyId = bodyId;
        }

        return currentFilter | kNoCollideBit;
    }

    inline std::uint32_t restoreFilter(const SuppressionState& state, std::uint32_t currentFilter)
    {
        return state.wasNoCollideBeforeSuppression ? (currentFilter | kNoCollideBit) : (currentFilter & ~kNoCollideBit);
    }

    inline void clear(SuppressionState& state)
    {
        state.active = false;
        state.wasNoCollideBeforeSuppression = false;
        state.bodyId = kInvalidBodyId;
    }
}
