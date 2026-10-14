#pragma once

#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"

#include <array>
#include <cstdint>

/*
 * Equipped weapon roster: the one answer to "which hand holds which equipped
 * weapon, and in which role". A slot is the native equipment index a weapon
 * occupies for its whole equipped life: 0 is the player's own graph, 1 is
 * ROCK's private graph. Per weapon, a hand holds its firing grip, a part grip
 * (support or other part), or nothing.
 *
 * PhysicsInteraction::equippedWeaponRoster() gathers Facts on every call.
 * Grip ownership changes during the frame (an earlier hand's equip or
 * transfer takes or frees the other), so readers ask when they decide.
 * Slot 1's native equipment facts are sampled by the frame owner.
 */
namespace rock::equipped_weapon_roster
{
    enum class Hand : std::uint8_t
    {
        None,
        Right,
        Left,
    };

    [[nodiscard]] constexpr Hand handFor(bool isLeft) noexcept
    {
        return isLeft ? Hand::Left : Hand::Right;
    }

    struct PartGripFacts
    {
        bool active{ false };
        bool attachOnly{ false };
    };

    struct SlotZeroFacts
    {
        // Equipped and not terminally sheathed: drawing and sheathing still
        // hold the hand.
        bool holdsHand{ false };
        // The grip session's firing hand.
        Hand sessionFiringHand{ Hand::Right };
        // The firing hand once a pending equipped-grip transfer for this
        // weapon applies; the session's firing hand otherwise.
        Hand firingHand{ Hand::Right };
        // ROCK's grip session confirmed the firing grip for the drawn item.
        bool firingGripConfirmed{ false };
        // No hand holds the firing grip: part carry or hands-free retention.
        bool firingGripDetached{ false };
        // Indexed by hand: [0] right, [1] left.
        std::array<PartGripFacts, 2> partGrips{};
    };

    struct SlotOneFacts
    {
        bool equipped{ false };
        // Equipped and not terminally sheathed.
        bool holdsHand{ false };
        Hand hand{ Hand::None };
    };

    // A pending equipped-grip start reserves its hands before the grip
    // session takes them.
    struct PendingGripFacts
    {
        bool bothHands{ false };
        Hand supportHand{ Hand::None };
    };

    struct Facts
    {
        SlotZeroFacts slot0{};
        SlotOneFacts slot1{};
        PendingGripFacts pending{};
    };

    class Roster
    {
    public:
        explicit constexpr Roster(const Facts& facts) noexcept :
            _facts(facts)
        {}

        [[nodiscard]] constexpr const Facts& facts() const noexcept { return _facts; }

        // Slot 0 holds this hand for every grab, force-grab, transfer and
        // grenade gate: by a part grip, by its attached firing grip, or by a
        // pending equipped grip reserved for this hand.
        [[nodiscard]] constexpr bool slotZeroHolds(bool isLeft) const noexcept
        {
            const auto& slot = _facts.slot0;
            if (_facts.pending.bothHands) {
                return true;
            }
            if (_facts.pending.supportHand != Hand::None) {
                return _facts.pending.supportHand == handFor(isLeft);
            }
            return slot.partGrips[isLeft ? 1 : 0].active ||
                (slot.holdsHand && !slot.firingGripDetached && slot.firingHand == handFor(isLeft));
        }

        [[nodiscard]] constexpr bool slotOneHolds(bool isLeft) const noexcept
        {
            return _facts.slot1.holdsHand && _facts.slot1.hand == handFor(isLeft);
        }

        // Slot 0's grip-session occupancy of this hand: its confirmed firing
        // grip and its part grips. It does not include pending reservations.
        [[nodiscard]] constexpr equipped_weapon_toggle_grab_policy::HandGripOccupancy slotZeroGrips(
            bool isLeft) const noexcept
        {
            const auto& slot = _facts.slot0;
            const auto& part = slot.partGrips[isLeft ? 1 : 0];
            return {
                .firingGripActive = slot.firingGripConfirmed && slot.sessionFiringHand == handFor(isLeft),
                .partGripActive = part.active,
                .partGripAttachOnly = part.attachOnly,
            };
        }

        [[nodiscard]] constexpr equipped_weapon_toggle_grab_policy::GripOccupancy slotZeroGrips() const noexcept
        {
            return { .left = slotZeroGrips(true), .right = slotZeroGrips(false) };
        }

        [[nodiscard]] constexpr bool bothHandsHoldWeapons() const noexcept
        {
            return _facts.slot0.holdsHand && _facts.slot1.holdsHand;
        }

    private:
        Facts _facts;
    };
}
