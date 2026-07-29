#include "physics-interaction/hand/DynamicHandCollisionTransitionPolicy.h"

#include <cassert>

int main()
{
    using namespace rock::dynamic_hand_collision_transition;

    State state{};
    auto step = advance(state, false, true);
    assert(step.state.phase == Phase::Active);
    assert(!step.suppressCollision);

    step = advance(step.state, true, false);
    assert(step.state.phase == Phase::Suspended);
    assert(step.suppressCollision);
    assert(step.collisionStateChanged);

    step = advance(step.state, false, true);
    assert(step.state.phase == Phase::Stabilizing);
    assert(step.state.stableWitnessFrames == 1);
    assert(step.suppressCollision);

    step = advance(step.state, false, false);
    assert(step.state.stableWitnessFrames == 0);
    assert(step.suppressCollision);

    for (std::uint32_t witness = 0; witness < kStableWitnessFramesRequired; ++witness) {
        step = advance(step.state, false, true);
    }
    assert(step.state.phase == Phase::Active);
    assert(!step.suppressCollision);
    assert(step.collisionStateChanged);
    return 0;
}
