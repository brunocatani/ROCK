#pragma once

/*
 * The vanilla quick loot menu (the world_Loot panel) on the hand whose wand
 * picks its container. FO4VR keeps the panel under the right wand
 * (PrimaryUIAttachNode) and fills it only from the primary (right) wand's
 * pick, because ROCK holds bLeftHandedMode:VR off. A container picked by the
 * left wand alone got the plain Search prompt and an empty panel.
 *
 * Two parts. The quick container data model update is redirected right
 * after it copies both wand picks: when the left hand owns the panel, the
 * model's primary and secondary picks are swapped, so the native target
 * choice and the panel data treat the left pick as the primary one. The left
 * hand owns the panel when only its wand picks a lootable container, or when
 * both do and the left wand is closer to its container by a switch margin.
 * The game thread measures those distances and moves the panel to the owning
 * hand once per frame, mirrored across the wand's X axis as the native
 * left-handed layout does. Only one hand ever carries the panel.
 */
namespace rock::native_quick_loot_hand
{
    // Plugin load, before the HUD runs. False when the native code does not
    // match: the panel then stays on the right hand.
    bool install() noexcept;

    // Game thread, every frame: moves the panel to the hand the last model
    // update named and measures both wands against their picks for the next
    // update.
    void update() noexcept;
}
