#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::loot_to_hand
{
    // One item the player took from a container: its base form and the stack identity
    // TESContainerChangedEvent reported (0 when the stack carries no unique ID).
    struct LootedItem
    {
        std::uint32_t baseFormId{ 0 };
        std::uint16_t uniqueId{ 0 };
    };

    // At most this many items wait for hands; two hands can take two, and the queue only has
    // to hold one container session. Items beyond it stay in inventory.
    inline constexpr std::size_t kQueueCapacity = 4;

    // Registers the container-changed sink once. Safe to call every frame; retries while the
    // event source is unavailable.
    void ensureRegistered();

    // Frame thread: moves the queued items into `out`, oldest first, and empties the queue.
    [[nodiscard]] std::size_t drain(std::array<LootedItem, kQueueCapacity>& out);
}
