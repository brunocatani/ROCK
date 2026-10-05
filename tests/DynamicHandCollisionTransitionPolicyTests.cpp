#include "physics-interaction/hand/DynamicHandCollisionTransitionPolicy.h"
#include "physics-interaction/hand/WeaponReleaseCollisionLease.h"

#include <cassert>

int main()
{
    using namespace rock::dynamic_hand_collision_transition;
    {
        using Lease = rock::weapon_release_collision_lease::State;
        using Identity = rock::weapon_release_collision_lease::Identity;
        Lease release;
        const Identity weapon{0x10000, 0x20000, 83};
        assert(release.arm(weapon, 0.8f));
        assert(release.advance(weapon, 0.3f));
        assert(release.advance(weapon, 0.3f));
        assert(!release.advance(weapon, 0.3f));
        assert(!release.active());
        assert(release.arm(weapon, 0.8f));
        assert(!release.advance({weapon.world, 0x30000, weapon.body}, 0.0f)); // Reused body ID.
        assert(release.arm(weapon, 0.8f));
        assert(!release.advance({0x40000, weapon.collisionObject, weapon.body}, 0.0f)); // New world.
        assert(release.arm(weapon, 0.8f));
        assert(!release.advance({weapon.world, weapon.collisionObject, 84}, 0.0f)); // Weapon replaced.
        assert(release.arm(weapon, 0.8f));
        release.reset();
        assert(!release.active());
        assert(!release.arm({}, 0.8f));
        assert(!release.arm(weapon, -1.0f));
        assert(release.arm(weapon, 0.0f) && !release.active());
    }

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
