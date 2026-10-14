#pragma once

#include <cstdint>

namespace rock::weapon_trigger_routing
{
    // Written by the interaction frame; copied as one atomic word to input
    // hooks. Physical hand facts must not straddle a firing-owner change.
    // Each hand's trigger and reload button act on the equip slot whose
    // firing grip that hand holds: slot 0 (the player's own graph) through
    // the game's primary trigger, slot 1 (ROCK's private graph) directly.
    struct Snapshot
    {
        // Slot 0's firing hand is the left (its last firing hand while detached).
        bool leftFiring{ false };
        // Slot 0 is support-held: no hand holds its firing grip.
        bool detached{ false };
        bool rightLoose{ false };
        bool leftLoose{ false };
        bool transferPending{ false };
        bool slotOneEquipped{ false };
        // Slot 1's hand; kept while sheathed so its reload button still routes.
        bool slotOneLeft{ false };
        // Slot 1 is drawn: its hand's trigger belongs to it, not to the game.
        bool slotOneDrawn{ false };

        [[nodiscard]] constexpr std::uint32_t pack() const noexcept
        {
            return (leftFiring ? 1u : 0u) | (detached ? 2u : 0u) |
                (rightLoose ? 4u : 0u) | (leftLoose ? 8u : 0u) | (transferPending ? 16u : 0u) |
                (slotOneEquipped ? 32u : 0u) | (slotOneLeft ? 64u : 0u) | (slotOneDrawn ? 128u : 0u);
        }
        [[nodiscard]] static constexpr Snapshot unpack(std::uint32_t bits) noexcept
        {
            return { (bits & 1u) != 0, (bits & 2u) != 0, (bits & 4u) != 0,
                (bits & 8u) != 0, (bits & 16u) != 0, (bits & 32u) != 0,
                (bits & 64u) != 0, (bits & 128u) != 0 };
        }
        [[nodiscard]] constexpr bool slotOneOwnsTrigger(bool left) const noexcept
        {
            return slotOneDrawn && slotOneLeft == left;
        }
        // The hand that reloads slot 0, if any: a support-held weapon has no
        // firing hand and does not reload.
        [[nodiscard]] constexpr bool slotZeroReloadsWith(bool left) const noexcept
        {
            return !detached && leftFiring == left;
        }
        [[nodiscard]] constexpr bool physicalSourceIsLeft(bool nativePrimary, bool triggerEvent) const noexcept
        {
            return triggerEvent && nativePrimary && leftFiring ? true : !nativePrimary;
        }
        [[nodiscard]] constexpr bool holdsLoose(bool left) const noexcept { return left ? leftLoose : rightLoose; }
    };
}
