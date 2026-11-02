#pragma once

/*
 * The vanilla quick loot menu (the world_Loot panel) stands at the container
 * it lists, facing the player, instead of riding the right wand. The native
 * controls (the primary wand's buttons and stick) and their hints are
 * unchanged.
 *
 * FO4VR fills the panel only from the primary (right) wand's pick, so a
 * container picked by the left wand alone got the plain Search prompt and an
 * empty panel. The quick container data model update is redirected right
 * after it copies both wand picks: when only the left pick is lootable, the
 * model's primary and secondary picks are swapped, so the native target
 * choice and the panel data list the left wand's container. A wand's Search
 * prompt hides while the panel lists the container that wand points at.
 *
 * The game thread moves the panel's root to a ROCK node under the room node
 * once per frame and places it at the listed container's near side, scaled
 * with distance so it reads at the size it has on the hand.
 */
namespace rock::native_quick_loot_panel
{
    // Plugin load, before the HUD runs. False when the native code does not
    // match: the panel then stays on the right hand and lists right picks only.
    bool install() noexcept;

    // Game thread, every frame: places the panel at the container the last
    // model update listed.
    void update() noexcept;
}
