#include "physics-interaction/weapon/WeaponTransitionAnimationAccelerationPolicy.h"

#include <cstdio>

namespace
{
    bool expect(const char* label, const bool condition)
    {
        if (condition) {
            return true;
        }
        std::printf("%s\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::weapon_transition_animation_acceleration_policy;
    using NativeWeaponState =
        rock::held_weapon_equip_state_policy::NativeWeaponState;

    constexpr auto state = [](const NativeWeaponState value) {
        return static_cast<std::uint32_t>(value);
    };
    bool ok = true;

    ok &= expect("draw acceleration must target only native drawing",
        shouldAccelerateSample(
            true,
            Direction::Draw,
            state(NativeWeaponState::Drawing)) &&
            !shouldAccelerateSample(
                true,
                Direction::Draw,
                state(NativeWeaponState::Drawn)));
    ok &= expect("sheathe acceleration must target only native sheathing",
        shouldAccelerateSample(
            true,
            Direction::Sheathe,
            state(NativeWeaponState::Sheathing)) &&
            !shouldAccelerateSample(
                true,
                Direction::Sheathe,
                state(NativeWeaponState::Drawn)));
    ok &= expect("another actor's animation channel must never be accelerated",
        !shouldAccelerateSample(
            false,
            Direction::Draw,
            state(NativeWeaponState::Drawing)));

    ok &= expect("draw remains pending before native drawing begins",
        classifyLifecycle(
            true,
            true,
            Direction::Draw,
            state(NativeWeaponState::Sheathed),
            0.1f) == LifecycleAction::KeepPending);
    ok &= expect("native drawing activates the draw lease",
        classifyLifecycle(
            true,
            true,
            Direction::Draw,
            state(NativeWeaponState::Drawing),
            0.1f) == LifecycleAction::Accelerate);
    ok &= expect("native drawn completes before the watchdog can cancel",
        classifyLifecycle(
            true,
            true,
            Direction::Draw,
            state(NativeWeaponState::Drawn),
            kLeaseTimeoutSeconds) == LifecycleAction::Complete);
    ok &= expect("native sheathed completes a sheathe lease",
        classifyLifecycle(
            true,
            true,
            Direction::Sheathe,
            state(NativeWeaponState::Sheathed),
            0.1f) == LifecycleAction::Complete);
    ok &= expect("identity loss cancels the lease",
        classifyLifecycle(
            true,
            false,
            Direction::Draw,
            state(NativeWeaponState::Drawing),
            0.1f) == LifecycleAction::Cancel);
    ok &= expect("menu/runtime loss cancels the lease",
        classifyLifecycle(
            false,
            true,
            Direction::Sheathe,
            state(NativeWeaponState::Sheathing),
            0.1f) == LifecycleAction::Cancel);
    ok &= expect("invalid native state cancels the lease",
        classifyLifecycle(
            true,
            true,
            Direction::Draw,
            99,
            0.1f) == LifecycleAction::Cancel);
    ok &= expect("a stuck transition expires through the bounded watchdog",
        classifyLifecycle(
            true,
            true,
            Direction::Draw,
            state(NativeWeaponState::WantToDraw),
            kLeaseTimeoutSeconds) == LifecycleAction::Cancel);
    ok &= expect("the finite acceleration multiplier is positive and bounded",
        kAcceleratedSpeedMultiplier == 100.0f);

    return ok ? 0 : 1;
}
