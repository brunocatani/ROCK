// How many copies of an inventory stack the player wears. Identical copies of
// one item share a stack and the engine ORs their worn bits into it, so a stack
// can hold worn and unworn copies at once.
#pragma once

#include "RE/Bethesda/BGSInventoryItem.h"

#include <cstdint>

namespace rock::inventory_stack_wear
{
    [[nodiscard]] inline std::uint32_t wornCopies(const RE::BGSInventoryItem::Stack& stack) noexcept
    {
        using Flag = RE::BGSInventoryItem::Stack::Flag;
        return static_cast<std::uint32_t>(stack.flags.any(Flag::kSlotIndex1)) + stack.flags.any(Flag::kSlotIndex2) +
            stack.flags.any(Flag::kSlotIndex3);
    }

    [[nodiscard]] inline bool hasUnwornCopy(const RE::BGSInventoryItem::Stack& stack) noexcept
    {
        return stack.GetCount() > wornCopies(stack);
    }
}
