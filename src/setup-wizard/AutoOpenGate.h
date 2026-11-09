#pragma once

#include <string>

/*
 * Whether the game is free for the setup wizard to open by itself: the player is in a
 * loaded cell and has the game's own movement and looking controls, and no loading screen,
 * fade, character creation (the mirror's LooksMenu) or menu that takes the controllers is
 * up. Scripted scenes and the new-game intro switch those controls off; message boxes from
 * other mods are menus, so the wizard waits behind them.
 *
 * Frame thread only.
 */
namespace rock::setup_wizard::auto_open_gate
{
    struct Reading
    {
        bool inWorld{ false };
        bool menuOpen{ false };
        bool fading{ false };
        bool mirrorOpen{ false };
        // False when the game's control check is unavailable (unexpected bytes): the gate stays closed.
        bool controlsKnown{ false };
        bool movement{ false };
        bool looking{ false };

        [[nodiscard]] bool ready() const noexcept
        {
            return inWorld && !menuOpen && !fading && !mirrorOpen && controlsKnown && movement && looking;
        }

        bool operator==(const Reading&) const = default;
    };

    [[nodiscard]] Reading read() noexcept;

    [[nodiscard]] std::string describe(const Reading& reading);
}
