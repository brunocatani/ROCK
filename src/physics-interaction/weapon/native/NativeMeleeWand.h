#pragma once

namespace rock::native_melee_wand
{
    /*
     * Native VR melee follows the hand that holds each weapon.
     *
     * FO4VR reads the primary (right) wand for everything the equipped
     * weapon does in melee: the velocity gate that starts a swing or gun bash
     * (140EFE880) and the contact callback's gun-bash speed, hit wand and hit
     * velocity (140EFF000; only the left fist node reads the left wand). With
     * ROCK's carry a weapon can be in either hand and index 1 adds a second
     * one, so those four primary-wand loads call ROCK instead:
     *
     * - the gate reads the index-0 hand's wand, or, with a weapon in both
     *   hands, whichever of the two is moving faster;
     * - a contact reads the wand of the hand whose weapon made it: the index-1
     *   hand for a contact the index-1 route claims, else the index-0 hand.
     *
     * Before the first update, and with nothing in the left hand, every load
     * returns the native right wand, so right-hand and unarmed behaviour stays
     * native. The unarmed left-fist path keeps reading the left wand itself.
     *
     * install() runs once at plugin load. update() runs on the frame thread
     * and publishes wand singleton pointers; the native readers may run on
     * contact threads and only ever receive a live singleton.
     */
    [[nodiscard]] bool install() noexcept;

    struct Holders
    {
        bool indexZeroLeft = false;
        bool indexZeroHeld = false;
        bool indexOneHeld = false;
        bool indexOneLeft = false;
    };

    void update(const Holders& holders) noexcept;
}
