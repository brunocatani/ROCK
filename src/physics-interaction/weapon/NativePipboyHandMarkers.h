#pragma once

namespace RE::Scaleform::GFx { class Movie; }

namespace rock::native_pipboy_hand_markers
{
    // Pip-Boy inventory rows of equipped weapons show the physical hand
    // holding each one ("R" / "L") in place of the native equipped square.
    // Name, sorting and the square's native visibility and tint are kept.
    // Display only: equipment and Pip-Boy actions are unchanged. Install
    // once at plugin load; fails closed.
    [[nodiscard]] bool install() noexcept;

    // Pip-Boy movie thread: draw the letters in this PipboyMenu movie (once
    // per movie). detach: the movie is being destroyed (any thread).
    void attach(RE::Scaleform::GFx::Movie* movie) noexcept;
    void detach(void* movie) noexcept;
}
