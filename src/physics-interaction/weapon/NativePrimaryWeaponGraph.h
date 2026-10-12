#pragma once

namespace rock::native_primary_weapon_graph
{
    // Keep the player's existing action graph bound to the real primary item
    // while another gun is equipped. Secondary actions own a separate graph.
    [[nodiscard]] bool install() noexcept;
}
