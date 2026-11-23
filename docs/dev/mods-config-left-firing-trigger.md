# Ticket: left-hand firing moves the Mods Config menu's trigger

**Status:** done. Headset check passed on 2026-11-23.

## Problem

While the left hand holds the equipped weapon's firing grip, the Mods Config menu takes its trigger from the left controller instead of the primary (right) one. Seen in game on 2026-11-19 while testing the firing-hand transforms.

## Cause

`InputRemapRuntime.cpp`, `shouldRemapLeftHandFireTriggerForGame` and `applyLeftHandFireTriggerRemapForGame`: while left-firing, ROCK presents the left physical trigger as the right wand's trigger (the game fires only from the primary wand's trigger), and blanks both physical triggers. The remap applies to every OpenVR state read except RPS UI's raw read, and it cannot fork by caller (the comment at the function says why), so Mods Config's pointer, which reads through CommonFramework's `VRControllers`, sees the remapped state.

The remap stops behind an input-blocking menu (`isInputBlockingMenuActive`). Mods Config's menu pushes no input context and opens no game menu, so nothing ROCK could see without Mods Config's API stopped it.

## Decision

- While Mods Config's menu is open and shown, only the left-hand fire remap stops (and the right-to-left game haptic redirect, which shares its check). Grabs, `[Controls]`, `[Bindings]` and provider suppression keep running: the game isn't paused, and live preview means trying settings with the menu open.
- `mods_config_client::update()` (FrameBegin) polls `getMenuState()` and calls `input_remap_runtime::setPointerMenuInUse(state == Open)`. A minimized menu takes no input (the setup wizard reads raw through RPS UI; Adjust In World hides every wand from the game and uses grip, A and thumbsticks), so the remap comes back while minimized. Menu events aren't used: they don't report minimize and restore.
- With the remap off, the right trigger works the menu; off the panel it fires the gun the game holds in the primary hand, as for a right-handed player. On the panel, Mods Config's pointer hides the pointing wand from the game, so clicks don't fire.
- No fix without a connected Mods Config: no other signal exists.
- Triggers held across a switch aren't latched: opening the menu mid-fire releases the game's trigger; a click that closes or minimizes the menu was hidden from the game by the pointer.
- The player guide's firing-hand section says it in one line; the spec has no warning.

## Answered

- Firing behind the menu: covered above.
- RPS UI: the remap skips its raw read (`rawInputReadActive`, `uiCapturedButtons`). PALM: its ROCK pages are retired (`mods-config-palm-removal.md`).

## Headset check (passed)

Left-firing with a gun:

1. Open Mods Config: the right trigger clicks, the left trigger doesn't fire.
2. Minimize it (or open the setup wizard from ROCK's page): the left trigger fires again.
3. Restore, then close: firing from the left trigger is back to normal.
