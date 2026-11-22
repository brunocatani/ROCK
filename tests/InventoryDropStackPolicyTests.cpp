// Grab requestInventoryItemGrab never drops the copy the player wears: looting
// an item identical to the equipped weapon or worn armor dropped the worn one.
#include "physics-interaction/native/player/InventoryDropStackPolicy.h"

#include <array>
#include <cassert>

using rock::inventory_drop_stack_policy::choose;
using rock::inventory_drop_stack_policy::StackFacts;

int main()
{
    // The equipped pistol's stack comes first and a looted copy sits in its
    // own stack: the looted copy is dropped, not the equipped one.
    {
        const std::array<StackFacts, 2> stacks{ { { .count = 1, .wornCopies = 1 }, { .count = 1 } } };
        assert(choose(stacks, 0) == 1u);
    }
    // The looted copy merged into the worn stack: its spare copy is dropped.
    {
        const std::array<StackFacts, 1> stacks{ { { .count = 2, .wornCopies = 1 } } };
        assert(choose(stacks, 0) == 0u);
    }
    // Only the worn copy is owned: nothing is dropped.
    {
        const std::array<StackFacts, 1> stacks{ { { .count = 1, .wornCopies = 1 } } };
        assert(!choose(stacks, 0));
    }
    // A named stack is taken only while it has an unworn copy.
    {
        const std::array<StackFacts, 2> stacks{ { { .count = 1, .wornCopies = 1, .uniqueId = 7 },
            { .count = 1, .uniqueId = 9 } } };
        assert(choose(stacks, 9) == 1u);
        assert(!choose(stacks, 7));
    }
    return 0;
}
