// Which inventory stack an item handed to the hand is dropped from.
#pragma once

#include <cstdint>
#include <optional>
#include <span>

/*
 * Never the copy the player wears: a stack qualifies only while it holds more
 * copies than are worn, and the drop takes one of those. A nonzero uniqueId
 * names the stack by its ExtraUniqueID. uniqueId 0 prefers a stack with
 * nothing worn, then a worn stack's spare copy: an item identical to a worn
 * one merges into the worn stack. Taking the first stack dropped the equipped
 * weapon or worn armor whenever the player looted an identical one.
 */
namespace rock::inventory_drop_stack_policy
{
    struct StackFacts
    {
        std::uint32_t count{ 0 };
        std::uint32_t wornCopies{ 0 };
        std::uint16_t uniqueId{ 0 };
    };

    [[nodiscard]] constexpr std::optional<std::uint32_t> choose(std::span<const StackFacts> stacks,
        const std::uint16_t uniqueId) noexcept
    {
        std::optional<std::uint32_t> spareOfWorn{};
        for (std::uint32_t index = 0; index < stacks.size(); ++index) {
            const auto& stack = stacks[index];
            if (stack.count <= stack.wornCopies) {
                continue;
            }
            if (uniqueId != 0) {
                if (stack.uniqueId == uniqueId) {
                    return index;
                }
            } else if (stack.wornCopies == 0) {
                return index;
            } else if (!spareOfWorn) {
                spareOfWorn = index;
            }
        }
        return spareOfWorn;
    }
}
