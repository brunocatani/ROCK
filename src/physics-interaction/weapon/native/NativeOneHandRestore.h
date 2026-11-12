#pragma once

namespace rock::native_one_hand_restore
{
    /*
     * FO4VR remembers the player's one-hand items that an equip on a
     * multi-hand slot displaced (140F7D000, called by the equip executors
     * 140E72780 and 140E72DA0) and re-equips them through 140E714A0 at four
     * sites: a later single-hand equip re-equips those of other slots
     * (140E72CE9, 140E732AA) and the unequip executors re-equip them once a
     * multi-hand item leaves (140E7453D, 140E7486D).
     *
     * ROCK places every weapon, so such a restore only puts back a weapon the
     * player replaced, over the one in hand: a second-weapon equip after two
     * holster swaps rebuilt the swapped-out rifle from 140E732AF and lost its
     * own pistol (observed at runtime). The four calls go to a no-op;
     * the remembered list stays native and each site still clears it.
     */
    [[nodiscard]] bool install() noexcept;

    // True once every restore site is disabled. ROCK equips, unequips and
    // drops refuse while false: a live restore would put a weapon back.
    [[nodiscard]] bool disabled() noexcept;
}
