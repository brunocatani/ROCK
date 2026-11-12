// Pip-Boy inventory rows for identical weapon copies: a worn copy gets its own
// row instead of sharing one with the unworn copies (see the .cpp).
#pragma once

#include "RE/Bethesda/BGSInventoryItem.h"

namespace rock::native_pipboy_worn_rows
{
    // Installs the row hooks. Load owner, once.
    [[nodiscard]] bool install() noexcept;

    // The Pip-Boy's row predicate with ROCK's rule applied as for a hand
    // weapon: true when both stacks belong on one row. Any thread that may
    // read the stacks.
    [[nodiscard]] bool sameWeaponRow(const RE::BGSInventoryItem::Stack& a, const RE::BGSInventoryItem::Stack& b) noexcept;
}
