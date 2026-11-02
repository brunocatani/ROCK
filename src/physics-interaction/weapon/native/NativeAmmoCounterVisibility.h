#pragma once

/*
 * Hides the vanilla ammo counter while ROCK's counters replace it. Health,
 * action points and the explosive counter stay. Without a renewal for 250 ms
 * (RPS UI lost, ROCK stopped presenting) the vanilla counter returns on its
 * own; no engine object is written.
 */
namespace rock::native_ammo_counter_visibility
{
    // Game thread. Installs once; false when the native widget does not match
    // (another mod owns the slot, or a different binary): the vanilla counter
    // then stays.
    bool install() noexcept;

    // Game thread, every presented frame.
    void replace(bool active) noexcept;
}
