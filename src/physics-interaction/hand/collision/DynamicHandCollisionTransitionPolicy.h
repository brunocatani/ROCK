#pragma once

#include <cstdint>

namespace rock::dynamic_hand_collision_transition
{
    inline constexpr std::uint32_t kStableWitnessFramesRequired = 3;

    enum class Phase : std::uint8_t
    {
        Active,
        Suspended,
        Stabilizing,
    };

    struct State
    {
        Phase phase = Phase::Active;
        std::uint32_t stableWitnessFrames = 0;
    };

    struct Step
    {
        State state{};
        bool suppressCollision = false;
        bool collisionStateChanged = false;
    };

    [[nodiscard]] constexpr Step advance(
        State state,
        bool animationBoundaryActive,
        bool targetsStable)
    {
        const bool wasSuppressed = state.phase != Phase::Active;

        if (animationBoundaryActive) {
            state.phase = Phase::Suspended;
            state.stableWitnessFrames = 0;
        } else if (state.phase != Phase::Active) {
            state.phase = Phase::Stabilizing;
            state.stableWitnessFrames = targetsStable ? state.stableWitnessFrames + 1 : 0;
            if (state.stableWitnessFrames >= kStableWitnessFramesRequired) {
                state.phase = Phase::Active;
                state.stableWitnessFrames = 0;
            }
        }

        const bool suppressCollision = state.phase != Phase::Active;
        return {
            .state = state,
            .suppressCollision = suppressCollision,
            .collisionStateChanged = suppressCollision != wasSuppressed,
        };
    }
}
