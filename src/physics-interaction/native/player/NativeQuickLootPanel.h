#pragma once

#include <cstdint>

/*
 * Where the vanilla quick loot menu (the world_Loot panel) appears, chosen by
 * Grabbing.QuickLootPanelPlacement in ROCK.toml and applied every frame:
 *
 * - RightHand: the game's own behaviour. The panel rides the right wand
 *   (PrimaryUIAttachNode) and lists only the right wand's pick; a container
 *   picked by the left wand alone gets the plain Search prompt and an empty
 *   panel.
 * - CloserHand: the panel rides the hand whose wand picks its container,
 *   mirrored on the left hand as the native left-handed layout does. When
 *   both wands pick containers, the hand closer to its own container takes the
 *   panel past a switch margin.
 * - AtContainer: the panel stands at the container it lists, facing the
 *   player, scaled with distance so it reads at the size it has on the hand.
 *
 * The native controls (the primary wand's buttons and stick) and their hints
 * are unchanged in every mode.
 *
 * FO4VR fills the panel only from the primary (right) wand's pick. For
 * CloserHand and AtContainer, the quick container data model update is
 * redirected right after it copies both wand picks: when the left wand owns
 * the listing, the model's primary and secondary picks are swapped, so the
 * native target choice and the panel data list the left wand's container. The
 * Search prompt of a wand that points at the listed container hides while the
 * panel lists it. In RightHand mode the redirect passes the picks through
 * untouched and the prompts keep their native answers.
 *
 * The game thread moves the panel's root between the right wand's attach node
 * and ROCK's own carrier nodes (one under the left wand, one under the room
 * node) and returns it to the right wand when the mode changes.
 */
namespace rock::native_quick_loot_panel
{
    // Grabbing.QuickLootPanelPlacement values.
    enum class Placement : std::uint8_t
    {
        RightHand = 1,
        CloserHand = 2,
        AtContainer = 3,
    };

    // Plugin load, before the HUD runs. False when the native code does not
    // match: the panel then stays on the right hand and lists right picks only,
    // whatever the setting says.
    bool install() noexcept;

    // Game thread, every frame: applies the placement setting and places the
    // panel for the container the last model update listed. The next model
    // update lists by the same placement.
    void update(Placement placement) noexcept;
}
