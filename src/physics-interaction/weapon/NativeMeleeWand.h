#pragma once

namespace rock::native_melee_wand
{
    /*
     * Native VR melee for the index-0 weapon follows the hand that holds it.
     *
     * FO4VR reads the primary (right) wand for everything the equipped
     * weapon does in melee: the velocity gate that starts a swing or gun bash
     * (140EFE880) and the contact callback's speed check, hit position and hit
     * velocity (140EFF000; only the left fist node reads the left wand). With
     * ROCK's left-hand carry the weapon is in the other hand, so its swings
     * never registered and right-arm swings did. Those four primary-wand loads
     * read a ROCK slot instead, which holds the wand of the hand carrying
     * index 0: the right one unless index 0 sits in the left hand, so
     * right-hand and unarmed behaviour stays native. The unarmed left-fist
     * path keeps reading the left wand itself.
     *
     * Frame thread: update() patches once both wand singletons exist, then
     * republishes the slot every frame. The native readers may run on contact
     * threads; they read one aligned pointer, always a live wand singleton.
     */
    void update(bool indexZeroInLeftHand) noexcept;
}
