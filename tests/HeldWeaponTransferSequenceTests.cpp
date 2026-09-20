#include "physics-interaction/weapon/HeldWeaponTransferPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/TransferredWeaponGrabPolicy.h"

#include <cstdio>
#include <initializer_list>

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) std::printf("FAIL: %s\n", message);
        return condition;
    }
}

int main()
{
    using namespace rock::held_weapon_transfer;
    namespace input = rock::input_remap_policy;
    namespace drop = rock::equipped_weapon_drop_policy;
    namespace toggle = rock::equipped_weapon_toggle_grab_policy;
    bool ok = true;
    ok &= expect(!toggle::confirmedFiringGripOccupied(1, 0, false, true, false),
        "inventory commit cannot publish a phantom native-right firing grip during physical acquisition");
    ok &= expect(toggle::confirmedFiringGripOccupied(1, 1, false, true, true),
        "actual manual acquisition publishes its firing grip in the same frame");
    ok &= expect(!toggle::confirmedFiringGripOccupied(1, 1, true, true, true),
        "support acquisition never publishes a firing grip");
    for (const bool left : { false, true }) {
        for (const auto role : { Role::Firing, Role::Support, Role::Paired }) {
            for (const bool outgoingFirst : { false, true }) {
                State state;
                Request request{ .reference = 0xFF001234, .grab = 7, .world = 1, .skeleton = 2,
                    .isLeft = left, .role = role, .retainOutgoing = true, .previousForm = 0x1234, .previousInstance = 0x1111 };
                ok &= expect(admit(state, request), "one exact loose acquisition admits a transfer");
                ok &= expect(!admit(state, request), "a duplicate press cannot replace the accepted transfer");
                ok &= expect(state.blocksFire() && !state.ownsEmptySlot(0), "admission blocks shots without granting an unarmed exemption early");
                ok &= expect(outgoingRemoved(state, 0xFF002345), "native outgoing removal acknowledged");
                ok &= expect(state.ownsEmptySlot(0) && !state.ownsEmptySlot(0x9999), "only this replacement's empty slot bypasses holstering");
                ok &= expect(!outgoingRemoved(state, 0xFF002345), "native removal is not repeated");
                const auto sequence = state.sequence;
                if (outgoingFirst) ok &= expect(outgoingFinished(state, sequence, true), "outgoing can finish before incoming equip");
                // Observe the same reference on repeated native-wait frames;
                // the adapter continues its held update without another drop.
                for (int frame = 0; frame < 120; ++frame) {
                    ok &= expect(state.wantsEquip(left, request.reference, request.grab), "accepted source survives native readiness wait");
                    ok &= expect(!state.wantsEquip(!left, request.reference, request.grab) &&
                        !state.wantsEquip(left, request.reference, request.grab + 1), "another hand or grab cannot replay equip");
                }
                ok &= expect(inventoryCommitted(state, 0x1234, 0x2222), "exact incoming stack commits once");
                ok &= expect(!inventoryCommitted(state, 0x1234, 0x2222) && !state.wantsEquip(left, request.reference, request.grab), "pickup cannot repeat after commit");
                ok &= expect(!state.ownsEmptySlot(0), "replacement exemption ends at incoming commit");
                ok &= expect(!acquireGrip(state, 0x1234, 0x1111, left, role), "old same-base instance cannot acquire requested grip");
                ok &= expect(!acquireGrip(state, 0x5678, 0x2222, left, role), "another form cannot acquire requested grip");
                ok &= expect(!acquireGrip(state, 0x1234, 0x2222, !left, role), "native default hand cannot acknowledge the destination");
                ok &= expect(acquireGrip(state, 0x1234, 0x3333, left, role), "validated native clone acquires intended role");
                ok &= expect(!state.matchesTarget(0x1234, 0x4444), "accepted clone binds exact identity against later same-base replacement");
                ok &= expect(!state.blocksFire(), "incoming acquisition does not wait on outgoing rendering");
                presentationAcquired(state);
                if (!outgoingFirst) {
                    ok &= expect(state.active(), "success waits for exact outgoing disposition");
                    ok &= expect(!outgoingFinished(state, sequence + 1, true), "foreign acknowledgement is ignored");
                    ok &= expect(outgoingFinished(state, sequence, true), "outgoing can finish after incoming grip");
                }
                ok &= expect(state.phase == Phase::Terminal && state.outcome == Outcome::Completed, "both acknowledgements complete one transaction");
                ok &= expect(!outgoingFinished(state, sequence, true), "duplicate completion has no effect");
                ok &= expect(admit(state, request) && state.sequence != sequence, "next swap has an independent sequence");
                ok &= expect(!outgoingFinished(state, sequence, false), "old result cannot cancel next swap");
            }
        }

        input::NativeActionSuppressionInput native{ .weaponDrawn = true, .eventMatched = true };
        rock::weapon_trigger_routing::Snapshot routing{
            .leftFiring = left, .rightLoose = left, .leftLoose = !left };
        const auto copied = rock::weapon_trigger_routing::Snapshot::unpack(routing.pack());
        auto shot = input::routeWeaponInput(native, copied, true, true);
        ok &= expect(shot.triggerSourceIsLeft == left && !input::shouldSuppressNativeTriggerAction(shot),
            "native primary attack resolves to firing hand despite opposite loose gun");
        auto physicalGrip = input::routeWeaponInput(native, copied, true, false);
        ok &= expect(!physicalGrip.triggerSourceIsLeft, "grip input keeps physical native identity");
        routing.detached = true;
        shot = input::routeWeaponInput(native, routing, true, true);
        ok &= expect(input::shouldSuppressNativeTriggerAction(shot), "support-only carry never admits a shot");
        routing.detached = false;
        routing.transferPending = true;
        shot = input::routeWeaponInput(native, routing, true, true);
        ok &= expect(input::shouldSuppressNativeTriggerAction(shot), "inventory success cannot fire before hand acquisition");
        routing.transferPending = false;
        shot = input::routeWeaponInput(native, routing, true, true);
        shot.triggerAwaitingRelease = true;
        ok &= expect(input::shouldSuppressNativeTriggerAction(shot), "acceptance press cannot become a shot");
        shot.triggerAwaitingRelease = false;
        ok &= expect(!input::shouldSuppressNativeTriggerAction(shot), "subsequent firing gesture is admitted");
    }

    for (const bool committed : { false, true }) {
        State state;
        admit(state, { .reference = 1, .grab = 1, .retainOutgoing = true });
        outgoingRemoved(state, 2);
        if (committed) inventoryCommitted(state, 3, 4);
        cancel(state, Outcome::Cancelled);
        ok &= expect(!state.wantsEquip(false, 1, 1) && !state.ownsEmptySlot(0), "cancellation retires retry and holster exemption");
        ok &= expect((state.phase == Phase::Recovering) == committed, "only committed pickup needs inventory compensation");
        if (!committed) ok &= expect(!state.blocksFire(), "cancelled outgoing cleanup cannot block a superseding equipped weapon");
        outgoingFinished(state, state.sequence, true);
        if (committed) {
            ok &= expect(state.active() && state.blocksFire(), "outgoing success cannot terminate incoming compensation");
            recovered(state);
        }
        ok &= expect(!state.active() && state.outcome == Outcome::Cancelled, "cancellation ends after actual dispositions");
    }
    State failure;
    admit(failure, { .reference = 1, .grab = 1 });
    inventoryCommitted(failure, 2, 3);
    cancel(failure);
    ok &= expect(failure.phase == Phase::Recovering && failure.blocksFire(), "bridge/grip failure enters closed recovery");
    recovered(failure);
    ok &= expect(failure.outcome == Outcome::Failed && !failure.active(), "failed transfer never reports successful equip");

    for (const auto mode : { drop::Mode::Off, drop::Mode::ToggleDrop, drop::Mode::AutoDrop }) {
        for (const bool left : { false, true }) {
            auto source = drop::simultaneousReleaseSource(mode != drop::Mode::Off, true, true, true, true, true, left, !left);
            ok &= expect(source == (mode == drop::Mode::Off ? drop::SourceHand::None : left ? drop::SourceHand::Left : drop::SourceHand::Right),
                "drop mode resolves simultaneous carrier release once");
            ok &= expect(drop::simultaneousReleaseSource(true, true, true, left, !left, true, left, false) == drop::SourceHand::None,
                "one release cannot drop while another carrier remains");
        }
    }
    ok &= expect(drop::captureOwnerCurrent(false, false, 100, 0, 200, true),
        "native equipped carry can drop before any manual session exists");
    ok &= expect(!drop::captureOwnerCurrent(false, false, 100, 0, 200, false),
        "native carry still needs the current canonical");
    ok &= expect(!drop::captureOwnerCurrent(false, true, 100, 0, 200, true),
        "native right capture cannot invent a left carrier");
    ok &= expect(drop::captureOwnerCurrent(true, true, 100, 100, 200, false) &&
        !drop::captureOwnerCurrent(true, true, 100, 99, 200, true),
        "manual carrier must own the exact current item");
    State source;
    admit(source, { .reference = 1, .grab = 2, .world = 3, .skeleton = 4 });
    ok &= expect(sourceCurrent(source, true, 1, 2, 3, 4), "accepted incoming source remains current");
    ok &= expect(!sourceCurrent(source, false, 1, 2, 3, 4) && !sourceCurrent(source, true, 1, 5, 3, 4),
        "release or regrab invalidates an uncommitted source");
    inventoryCommitted(source, 9, 10);
    ok &= expect(sourceCurrent(source, false, 0, 0, 3, 4) && !sourceCurrent(source, false, 0, 0, 5, 4),
        "inventory commit retires loose identity but never lifecycle identity");
    ok &= expect(sameMenuItem(9, 10, 9, 10) && !sameMenuItem(9, 10, 9, 11) && !sameMenuItem(0, 0, 0, 0),
        "menu carry cannot bind another same-base stack or an empty slot");
    for (auto role : { Role::Firing, Role::Support, Role::Paired }) {
        State menu;
        ok &= expect(resumeMenu(menu, 9, 10, { .world = 3, .skeleton = 4, .isLeft = true, .role = role }),
            "same equipped menu item resumes its physical role without a loose pickup");
        ok &= expect(!menu.wantsEquip(true, 0, 0) && menu.blocksFire(), "menu restoration waits for actual grip acquisition");
        ok &= expect(!acquireGrip(menu, 9, 11, true, role), "menu restoration cannot accept an arbitrary clone");
        ok &= expect(acquireGrip(menu, 9, 10, true, role), "menu restoration acquires saved role");
        presentationAcquired(menu);
        ok &= expect(!menu.active(), "menu restoration ends without waiting for a nonexistent outgoing drop");
    }
    // All configured input modes feed accepted logical releases to the same
    // atomic carrier decision. Physical button-up alone is not a toggle release.
    for (auto mode : { toggle::Mode::ToggleBoth, toggle::Mode::ToggleFiringOnly, toggle::Mode::HoldBoth }) {
        toggle::TransferReleaseState firing, support;
        firing.observe(mode, true, { .released = true });
        support.observe(mode, false, { .released = true });
        if (toggle::usesToggleForRole(mode, true)) firing.observe(mode, true, { .held = true, .pressed = true });
        if (toggle::usesToggleForRole(mode, false)) support.observe(mode, false, { .held = true, .pressed = true });
        for (auto dropMode : { drop::Mode::Off, drop::Mode::ToggleDrop, drop::Mode::AutoDrop }) {
            auto selected = drop::simultaneousReleaseSource(dropMode != drop::Mode::Off, true, true,
                support.releaseRequested, firing.releaseRequested, true, false, true);
            ok &= expect(selected == (dropMode == drop::Mode::Off ? drop::SourceHand::None : drop::SourceHand::Right),
                "all grab/drop mode combinations resolve final carrier ownership consistently");
        }
    }
    State waiting;
    admit(waiting, { .reference = 1, .grab = 1, .retainOutgoing = true, .previousForm = 9, .previousInstance = 10 });
    ok &= expect(equippedSourceCurrent(waiting, 9, 10) && !equippedSourceCurrent(waiting, 9, 11),
        "native readiness wait cannot consume an external same-base equip");
    outgoingRemoved(waiting, 2);
    ok &= expect(equippedSourceCurrent(waiting, 0, 0) && !equippedSourceCurrent(waiting, 9, 10),
        "only the acknowledged empty slot belongs to a replacement wait");
    State uncertain;
    admit(uncertain, { .reference = 1, .grab = 1, .previousForm = 9, .previousInstance = 10 });
    inventoryCommitted(uncertain, 9, 0);
    ok &= expect(!uncertain.matchesTarget(9, 10), "unresolved incoming instance must not recover the old same-base stack");
    toggle::TransferReleaseState pendingHold;
    pendingHold.observe(toggle::Mode::HoldBoth, true, {}, true);
    ok &= expect(!pendingHold.releaseRequested, "accepted retained loose carry does not fabricate a hold-mode release");
    pendingHold.observe(toggle::Mode::HoldBoth, true, { .held = true }, true);
    pendingHold.observe(toggle::Mode::HoldBoth, true, { .released = true }, true);
    ok &= expect(pendingHold.releaseRequested, "physical release during pending acquisition survives inventory commit");
    toggle::TransferReleaseState pendingToggle;
    pendingToggle.observe(toggle::Mode::ToggleBoth, true, { .released = true }, true);
    pendingToggle.observe(toggle::Mode::ToggleBoth, true, { .held = true, .pressed = true }, true);
    ok &= expect(pendingToggle.releaseRequested, "second toggle before acquisition uses the destination's drop mode");
    auto retained = rock::transferred_weapon_grab_policy::State::AwaitInitialRelease;
    ok &= expect(!rock::transferred_weapon_grab_policy::advance(retained, false, false, true), "original drop gesture cannot release newly retained weapon");
    ok &= expect(!rock::transferred_weapon_grab_policy::advance(retained, true, true, false), "retained release requires complete new gesture");
    ok &= expect(rock::transferred_weapon_grab_policy::advance(retained, false, false, true), "new release completes retained drop");
    return ok ? 0 : 1;
}
