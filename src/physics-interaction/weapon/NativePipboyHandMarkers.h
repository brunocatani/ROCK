#pragma once

namespace rock::native_pipboy_hand_markers
{
    // Pip-Boy inventory entries of the (up to two) equipped weapons show the
    // physical hand holding each one ("R  name" / "L  name") instead of the
    // native equipped square. Display only: equipment and Pip-Boy actions
    // are unchanged. Install once at plugin load; fails closed.
    [[nodiscard]] bool install() noexcept;
}
