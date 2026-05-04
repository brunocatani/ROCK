# Input Remap Menu Gate Fix — 2026-05-03

The input remap hooks were installed correctly, but the refactor's menu gate classified `WSLootMenu` as game-stopping. Runtime logs showed `WSLootMenu opened` immediately after load and then stayed open during normal gameplay. That kept `menuInputActive=true`, so ROCK bypassed the OpenVR filtering path and did not enqueue right-stick weapon toggles. The same stale menu gate let Favorites/ReadyWeapon input leak through.

`WSLootMenu` is a gameplay quick-loot overlay, not a blocking menu like `ContainerMenu`, `BarterMenu`, `PipboyMenu`, or `FavoritesMenu`. It must not disable remapping globally.

The fix removes `WSLootMenu` from the game-stopping menu list while keeping real menu screens in the gate. `ROCK/tests/InputRemapRuntimeSourceTests.ps1` now prevents reintroducing it.
