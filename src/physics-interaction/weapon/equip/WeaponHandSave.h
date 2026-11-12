// The HNDS co-save record (see WeaponHandSave.cpp).
#pragma once

namespace rock::weapon_hand_save
{
    /*
     * Keeps which hand holds which equipped weapon, and each hand's
     * one-handed melee reverse grip, in the save game (F4SE co-save, record
     * 'HNDS'). The engine's equip slot follows the weapon, not the hand, and a
     * load restores only the weapons; without the record they return to the
     * default hands and the normal grip.
     *
     * The save callback reads native_hand_equipment::savedHands() and
     * knife_reverse_grip::savedGrip(); the load callback resolves the base
     * forms into the loaded load order and hands them to their restore
     * functions; the revert callback (new game, before every load) clears
     * them, so a save without the record keeps today's default hands. All
     * three run on the game's save/load thread and touch only those modules'
     * lock- or atomic-guarded records.
     */
    // Load owner, after F4SE::Init. False when the serialization interface is unavailable.
    [[nodiscard]] bool install() noexcept;
}
