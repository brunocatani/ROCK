#pragma once

#include "physics-interaction/weapon/equip/EquippedWeaponRoster.h"

#include <array>
#include <cstdint>

/*
 * Which equip slot an equip lands in. Every ROCK equip path asks this one
 * rule; the acting hand (or the default hand for equips without one)
 * decides, and the slot follows the weapon:
 * - a press on the item a slot already wears: the same hand toggles it; when
 *   the other hand fires it, the acting hand toggles itself as its support
 *   grip (never taking the weapon), first returning the weapon it held to
 *   inventory; a weapon no hand fires moves its firing grip to the acting
 *   hand, displacing the weapon that hand fired;
 * - the acting hand fires a weapon, or holds one no hand fires: that slot is
 *   replaced;
 * - otherwise the first free slot, else the slot whose weapon no hand
 *   fires (support-held). Nothing else is ever evicted.
 */
namespace rock::equip_slot_allocation
{
    using Hand = equipped_weapon_roster::Hand;

    struct SlotState
    {
        bool worn{ false };
        // The hand holding this weapon's firing grip; None when support-held.
        Hand firingHand{ Hand::None };
        // Support-held: the hand carrying it by a part grip.
        Hand holdingHand{ Hand::None };
        // The requested item is this slot's item.
        bool requestedItem{ false };
    };

    enum class Action : std::uint8_t
    {
        // Equip into `slot` (replacing its weapon, or the native toggle when
        // it already wears the requested item).
        Equip,
        // Move `slot`'s firing grip to the acting hand without a re-equip;
        // `displacedSlot` (if any) returns to inventory.
        MoveFiringGrip,
        // The acting hand joins `slot`'s weapon as its support grip, or
        // leaves it; the hand firing it keeps it. `displacedSlot` (if any),
        // the weapon the acting hand held, returns to inventory first.
        ToggleSupport,
    };

    struct Decision
    {
        Action action{ Action::Equip };
        std::uint32_t slot{ 0 };
        int displacedSlot{ -1 };
    };

    [[nodiscard]] constexpr Decision decide(Hand acting, const std::array<SlotState, 2>& slots) noexcept
    {
        const auto fires = [&](int index) {
            return slots[index].worn && acting != Hand::None && slots[index].firingHand == acting;
        };
        const auto holds = [&](int index) {
            return slots[index].worn && acting != Hand::None && slots[index].firingHand == Hand::None &&
                slots[index].holdingHand == acting;
        };
        const int actingSlot = fires(0) ? 0 : fires(1) ? 1 : -1;
        const int heldSlot = holds(0) ? 0 : holds(1) ? 1 : -1;
        const auto wears = [&](int index) { return index >= 0 && slots[index].worn && slots[index].requestedItem; };
        const int wornSlot = wears(actingSlot) ? actingSlot : wears(0) ? 0 : wears(1) ? 1 : -1;
        if (wornSlot >= 0 && wornSlot != actingSlot) {
            const auto firing = slots[wornSlot].firingHand;
            if (firing != Hand::None && firing != acting) {
                return { .action = Action::ToggleSupport,
                    .slot = static_cast<std::uint32_t>(wornSlot),
                    .displacedSlot = actingSlot >= 0 ? actingSlot : heldSlot };
            }
            return { .action = Action::MoveFiringGrip,
                .slot = static_cast<std::uint32_t>(wornSlot),
                .displacedSlot = actingSlot };
        }
        if (wornSlot >= 0) {
            return { .slot = static_cast<std::uint32_t>(wornSlot) };
        }
        if (actingSlot >= 0) {
            return { .slot = static_cast<std::uint32_t>(actingSlot) };
        }
        if (heldSlot >= 0) {
            return { .slot = static_cast<std::uint32_t>(heldSlot) };
        }
        if (!slots[0].worn) {
            return { .slot = 0 };
        }
        if (!slots[1].worn) {
            return { .slot = 1 };
        }
        return { .slot = slots[0].firingHand == Hand::None ? 0u : 1u };
    }
}
