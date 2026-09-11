#include "physics-interaction/weapon/VirtualHolstersCompatibility.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"

#include <cstdio>

namespace vh = rock::virtual_holsters;
namespace toggle = rock::equipped_weapon_toggle_grab_policy;

namespace
{
    bool expect(const char* name, bool result)
    {
        if (!result) {
            std::printf("FAILED: %s\n", name);
        }
        return result;
    }

    vh::Input inZone(bool toggleGrab = true, bool isLeft = false)
    {
        return {
            .holster = { .ready = true, .isLeft = isLeft, .inZone = true, .slot = 3, .buttonId = 2 },
            .ownershipKey = 100,
            .grabButtonId = 2,
            .isLeft = isLeft,
            .weaponEngaged = true,
            .toggleGrab = toggleGrab,
            .held = true,
            .pressed = true,
        };
    }
}

int main()
{
    bool ok = true;
    for (const bool left : { false, true }) {
        auto input = inZone(true, left);
        vh::HandState state{};
        auto decision = vh::advance(state, input);
        ok &= expect("configured hand retains toggle grip", decision.retainGrip && decision.consumeInput && decision.started);
        input.pressed = false;
        input.holster.inZone = false;
        decision = vh::advance(state, input);
        ok &= expect("sphere exit while held still owns gesture", decision.retainGrip && decision.consumeInput && !decision.started);
        input.held = false;
        input.released = true;
        decision = vh::advance(state, input);
        ok &= expect("toggle gesture consumes its final release", decision.retainGrip && decision.consumeInput);
        input.released = false;
        decision = vh::advance(state, input);
        ok &= expect("toggle resumes after complete cycle", !decision.consumeInput);
        input.held = input.pressed = true;
        decision = vh::advance(state, input);
        ok &= expect("next outside press is available to drop", !decision.consumeInput);

        state = {};
        input = inZone(false, left);
        (void)vh::advance(state, input);
        input.held = input.pressed = false;
        input.released = true;
        decision = vh::advance(state, input);
        ok &= expect("hold release inside sphere retains grip", decision.retainGrip && decision.consumeInput);
        input.holster.inZone = false;
        input.released = false;
        decision = vh::advance(state, input);
        ok &= expect("spent hold release cannot drop on exit", decision.retainGrip && decision.consumeInput);
        input.held = input.pressed = true;
        decision = vh::advance(state, input);
        ok &= expect("fresh outside squeeze rearms hold mode", !decision.consumeInput);
        input.held = input.pressed = false;
        input.released = true;
        decision = vh::advance(state, input);
        ok &= expect("fresh outside release is available to drop", !decision.consumeInput);
    }

    for (int invalid = 0; invalid < 7; ++invalid) {
        vh::HandState state{};
        auto input = inZone();
        switch (invalid) {
        case 0: input.holster.ready = false; break;
        case 1: input.holster.inZone = false; break;
        case 2: input.holster.isLeft = true; break;
        case 3: input.holster.buttonId = 7; break;
        case 4: input.holster.slot = 8; break;
        case 5: input.weaponEngaged = false; break;
        case 6: input.ownershipKey = 0; break;
        }
        const auto decision = vh::advance(state, input);
        ok &= expect("unavailable or unrelated interaction is untouched", !decision.consumeInput && !decision.retainGrip);
    }

    {
        vh::HandState state{};
        auto input = inZone();
        (void)vh::advance(state, input);
        input.weaponEngaged = false;
        input.ownershipKey = 0;
        input.holster = {};
        input.pressed = false;
        auto decision = vh::advance(state, input);
        ok &= expect("native unequip drains held press without acquiring", decision.consumeInput && !decision.retainGrip);
        input.held = false;
        input.released = true;
        decision = vh::advance(state, input);
        ok &= expect("native unequip drains release without acquiring", decision.consumeInput && !decision.retainGrip);
        input.released = false;
        decision = vh::advance(state, input);
        ok &= expect("native unequip clears cycle after release", !decision.consumeInput);
    }

    {
        vh::HandState state{};
        auto input = inZone(false);
        input.held = input.pressed = false;
        input.released = true;
        (void)vh::advance(state, input);
        input.holster = {};
        input.released = false;
        auto decision = vh::advance(state, input);
        ok &= expect("provider loss does not replay consumed hold release", decision.retainGrip);
        input.ownershipKey = 101;
        decision = vh::advance(state, input);
        ok &= expect("retention belongs only to original weapon", !decision.consumeInput);

        state = {};
        input = inZone(false);
        input.held = input.pressed = false;
        (void)vh::advance(state, input);
        input.holster = {};
        input.toggleGrab = true;
        decision = vh::advance(state, input);
        ok &= expect("mode change clears hold release retention", !decision.consumeInput);
    }

    {
        // Exercise the production ordering with an already pending toggle
        // release: the holster veto retains ownership and reconciliation must
        // relatch, otherwise leaving the sphere would reopen the grip.
        toggle::RuntimeState toggleState{ .weaponOwnershipKey = 100 };
        toggleState.hands[toggle::handIndex(false)] = toggle::HandState::ReleasePending;
        vh::HandState holsterState{};
        const toggle::HandGripOccupancy firingGrip{ .firingGripActive = true };
        auto input = inZone(firingGrip.usesToggleGrab(false));
        const auto holster = vh::advance(holsterState, input);
        auto result = toggle::prepare(toggleState, {
            .toggleGrabEnabled = false, .inputAllowed = true, .weaponOwnershipKey = 100,
            .occupancy = { .right = { .firingGripActive = true } }, .right = { .held = holster.retainGrip },
        });
        if (holster.consumeInput) {
            result.right = { .held = holster.retainGrip };
        }
        ok &= expect("pending toggle release is closed before grip update", result.right.held && !result.right.released);
        (void)toggle::reconcile(toggleState, false, 100, { .right = { .firingGripActive = true } }, { .right = holster.retainGrip });
        result = toggle::prepare(toggleState, {
            .toggleGrabEnabled = false, .inputAllowed = true, .weaponOwnershipKey = 100,
            .occupancy = { .right = { .firingGripActive = true } },
        });
        ok &= expect("refused toggle release stays latched outside sphere", result.right.held);
        result = toggle::prepare(toggleState, {
            .toggleGrabEnabled = false, .inputAllowed = true, .weaponOwnershipKey = 100,
            .occupancy = { .right = { .firingGripActive = true } }, .right = { .held = true, .pressed = true },
        });
        ok &= expect("later toggle press still releases normally", !result.right.held && result.right.released);
    }

    return ok ? 0 : 1;
}
