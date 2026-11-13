#pragma once

/*
 * Workshop mode on a hold of the left stick click.
 *
 * ROCK keeps the off-hand menu button for its Pip-Boy tap and Pause hold, so
 * the game's own Workshop hold on that button never runs. Holding the left
 * stick click for the game's fEnterWorkshopDelay:Controls instead calls the
 * game's Workshop entry, the one its StartWorkshop console command and the
 * menu button's hold both use. That entry decides everything about the
 * workshop itself (inside a buildable area, not already building, available
 * to the player), so outside a settlement the hold does nothing. A click
 * still reaches the game, the bipod and held placement as before.
 *
 * Frame thread only.
 */
namespace rock::workshop_entry_gesture
{
    // Verifies the game's Workshop entry once; without it the hold does nothing.
    void install();

    // Once per frame, before gameplay input is read.
    void update();
}
