// Keeps a worn weapon copy on its own stack and Pip-Boy row when an identical
// copy enters the inventory later (pickup, container, script, a loaded save):
// the engine merges that copy into the worn stack. Equips separate their own
// copies at once (NativeHandEquipment useItem, WeaponEquipTransfer).
#pragma once

namespace rock::spare_weapon_copy_rows
{
    // Registers the container-changed sink once. Safe to call every frame;
    // retries while the event source is unavailable.
    void ensureRegistered() noexcept;

    // Frame thread. Separates pending copies once no menu is open, so an open
    // container or barter list never sees its rows change under it.
    void service(bool menuOpen, bool playerReady) noexcept;
}
