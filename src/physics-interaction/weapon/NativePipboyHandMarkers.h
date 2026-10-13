#pragma once

namespace rock::native_pipboy_hand_markers
{
    // Pip-Boy inventory rows of equipped weapons show the physical hand
    // holding each one ("R" / "L") in place of the native equipped square.
    // Name, sorting and the square's native visibility and tint are kept.
    // Display only: equipment and Pip-Boy actions are unchanged. Install
    // once at plugin load; fails closed.
    [[nodiscard]] bool install() noexcept;

    // PipboyMenu's code-object call (movie thread): rows also refresh on
    // every AS3-to-code call, e.g. selection and list changes.
    void onPipboyCodeCall(void* menu) noexcept;
}
