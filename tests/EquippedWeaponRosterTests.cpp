#include "physics-interaction/weapon/EquippedWeaponRoster.h"

#include <cstdio>

namespace
{
    using namespace rock::equipped_weapon_roster;

    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }

    Facts slotZeroFiring(Hand firing)
    {
        Facts facts{};
        facts.slot0.holdsHand = true;
        facts.slot0.sessionFiringHand = firing;
        facts.slot0.firingHand = firing;
        facts.slot0.firingGripConfirmed = true;
        return facts;
    }
}

int main()
{
    bool ok = true;

    {
        const Roster roster(slotZeroFiring(Hand::Right));
        ok &= expect("right only: slot 0 holds the right hand", roster.slotZeroHolds(false) && !roster.slotZeroHolds(true));
        ok &= expect("right only: the right firing grip is confirmed",
            roster.slotZeroGrips(false).firingGripActive && !roster.slotZeroGrips(true).weaponEngaged());
        ok &= expect("right only: one weapon does not fill both hands", !roster.bothHandsHoldWeapons());
    }
    {
        const Roster roster(slotZeroFiring(Hand::Left));
        ok &= expect("left only: slot 0 holds the left hand", roster.slotZeroHolds(true) && !roster.slotZeroHolds(false));
        ok &= expect("left only: the left firing grip is confirmed",
            roster.slotZeroGrips(true).firingGripActive && !roster.slotZeroGrips(false).weaponEngaged());
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot1 = { .equipped = true, .holdsHand = true, .hand = Hand::Left };
        const Roster roster(facts);
        ok &= expect("two weapons: each slot holds its own hand",
            roster.slotZeroHolds(false) && roster.slotOneHolds(true) && !roster.slotOneHolds(false) && !roster.slotZeroHolds(true));
        ok &= expect("two weapons: both hands hold weapons", roster.bothHandsHoldWeapons());
        facts.slot1.holdsHand = false;
        ok &= expect("a sheathed second weapon holds no hand", !Roster(facts).slotOneHolds(true) && !Roster(facts).bothHandsHoldWeapons());
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot0.partGrips[1] = { .active = true };
        const Roster roster(facts);
        ok &= expect("supported: both hands hold slot 0", roster.slotZeroHolds(false) && roster.slotZeroHolds(true));
        ok &= expect("supported: the left hand holds a part grip, not the firing grip",
            roster.slotZeroGrips(true).partGripActive && !roster.slotZeroGrips(true).firingGripActive);
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot0.firingGripDetached = true;
        facts.slot0.firingGripConfirmed = false;
        facts.slot0.partGrips[1] = { .active = true };
        const Roster roster(facts);
        ok &= expect("support only: the free firing hand is not held", !roster.slotZeroHolds(false));
        ok &= expect("support only: the supporting hand is held", roster.slotZeroHolds(true));
        ok &= expect("support only: no hand holds the firing grip",
            !roster.slotZeroGrips(false).firingGripActive && !roster.slotZeroGrips(true).firingGripActive);
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot0.partGrips[1] = { .active = true, .attachOnly = true };
        const auto grips = Roster(facts).slotZeroGrips(true);
        ok &= expect("an attach-only part grip engages the hand but does not carry the weapon",
            grips.weaponEngaged() && !grips.carriesWeapon());
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot0.firingHand = Hand::Left;
        ok &= expect("a pending transfer moves slot 0's hold to the new firing hand",
            Roster(facts).slotZeroHolds(true) && !Roster(facts).slotZeroHolds(false));
        ok &= expect("a pending transfer leaves the confirmed session grip in place",
            Roster(facts).slotZeroGrips(false).firingGripActive);
    }
    {
        Facts facts{};
        facts.pending.bothHands = true;
        ok &= expect("a pending paired grip reserves both hands", Roster(facts).slotZeroHolds(false) && Roster(facts).slotZeroHolds(true));
        facts.pending = { .supportHand = Hand::Left };
        facts.slot0 = slotZeroFiring(Hand::Left).slot0;
        ok &= expect("a pending support grip reserves only its hand",
            Roster(facts).slotZeroHolds(true) && !Roster(facts).slotZeroHolds(false));
    }
    {
        auto facts = slotZeroFiring(Hand::Right);
        facts.slot0.holdsHand = false;
        ok &= expect("a sheathed slot 0 holds no hand by its firing grip", !Roster(facts).slotZeroHolds(false));
        facts.slot0.partGrips[0] = { .active = true };
        ok &= expect("a part grip holds its hand even while the weapon sheathes", Roster(facts).slotZeroHolds(false));
    }

    return ok ? 0 : 1;
}
