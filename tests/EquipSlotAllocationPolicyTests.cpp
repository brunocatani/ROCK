#include "physics-interaction/weapon/EquipSlotAllocationPolicy.h"

#include <cstdio>

namespace
{
    using namespace rock::equip_slot_allocation;

    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }

    bool equips(const Decision& decision, std::uint32_t slot)
    {
        return decision.action == Action::Equip && decision.slot == slot;
    }
}

int main()
{
    bool ok = true;
    const SlotState empty{};
    const SlotState rightFired{ .worn = true, .firingHand = Hand::Right };
    const SlotState leftFired{ .worn = true, .firingHand = Hand::Left };
    const SlotState supportHeld{ .worn = true, .firingHand = Hand::None };

    ok &= expect("an empty roster fills slot 0", equips(decide(Hand::Left, { { empty, empty } }), 0));
    ok &= expect("the free hand fills slot 1 beside a firing slot 0", equips(decide(Hand::Left, { { rightFired, empty } }), 1));
    ok &= expect("the hand firing slot 0 replaces it", equips(decide(Hand::Right, { { rightFired, empty } }), 0));
    ok &= expect("the hand firing slot 1 replaces it", equips(decide(Hand::Left, { { rightFired, leftFired } }), 1));
    ok &= expect("with both slots worn the support-held weapon is replaced",
        equips(decide(Hand::Right, { { supportHeld, leftFired } }), 0));
    ok &= expect("a free hand beside a support-held slot 0 fills slot 1",
        equips(decide(Hand::Right, { { supportHeld, empty } }), 1));

    auto ownItem = rightFired;
    ownItem.requestedItem = true;
    ok &= expect("the same hand on its own item toggles that slot", equips(decide(Hand::Right, { { ownItem, empty } }), 0));

    const auto move = decide(Hand::Left, { { ownItem, empty } });
    ok &= expect("the other hand moves the worn item's firing grip",
        move.action == Action::MoveFiringGrip && move.slot == 0 && move.displacedSlot < 0);
    const auto swap = decide(Hand::Left, { { ownItem, leftFired } });
    ok &= expect("a move displaces the weapon the acting hand fired",
        swap.action == Action::MoveFiringGrip && swap.slot == 0 && swap.displacedSlot == 1);

    ok &= expect("an equip without a hand never claims a firing slot",
        equips(decide(Hand::None, { { rightFired, empty } }), 1));
    return ok ? 0 : 1;
}
