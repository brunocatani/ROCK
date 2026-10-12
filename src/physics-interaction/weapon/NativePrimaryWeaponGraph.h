#pragma once

#include "RE/Bethesda/BSTArray.h"
namespace RE { class Actor; class IKeywordFormBase; }

namespace rock::native_primary_weapon_graph
{
    // Keep the player's existing action graph bound to the real primary item
    // while another gun is equipped. Secondary actions own a separate graph.
    [[nodiscard]] bool install() noexcept;
    // Called by the shared native keyword collector, including preload and
    // graph comparison. False leaves ordinary engine selection untouched.
    [[nodiscard]] bool collect(RE::Actor* actor, RE::BSScrapArray<RE::IKeywordFormBase*>* output);
}
