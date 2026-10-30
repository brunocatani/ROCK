#pragma once

/*
 * The vanilla quick loot menu (the world_Loot panel) on whichever hand is
 * closer to its container. FO4VR keeps the panel under the right wand
 * (PrimaryUIAttachNode) because ROCK holds bLeftHandedMode:VR off, so a
 * container picked by the left hand listed its loot on the right hand.
 *
 * Two parts. The quick container data model's target choice prefers the
 * right wand whenever its pick is a valid container; the hook keeps that
 * choice unless both wands pick containers and the left one is closer to its
 * own. It then names the hand that owns the shown container: the picking
 * wand, or the closer wand when both pick the same container. The game
 * thread moves the panel to that hand once per frame, mirrored across the
 * wand's X axis as the native left-handed layout does. Only one hand ever
 * carries the panel.
 */
namespace rock::native_quick_loot_hand
{
    // Plugin load, before the HUD runs. False when the native call sites do
    // not match: the panel then stays on the right hand.
    bool install() noexcept;

    // Game thread, every frame: moves the panel to the hand the last target
    // choice named and measures both wands against their picks for the next
    // choice.
    void update() noexcept;
}
