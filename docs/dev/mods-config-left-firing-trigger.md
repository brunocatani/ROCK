# Ticket: left-hand firing moves the Mods Config menu's trigger

**Status:** open, not started. **Depends on:** probably `mods-config-native-api.md` (menu events).

## Problem

While the left hand holds the equipped weapon's firing grip, the Mods Config menu takes its trigger from the left controller instead of the primary (right) one. Seen in game on 2026-11-19 while testing the firing-hand transforms.

## Cause

`InputRemapRuntime.cpp`, `shouldRemapLeftHandFireTriggerForGame` and `applyLeftHandFireTriggerRemapForGame`: while left-firing, ROCK presents the left physical trigger as the right wand's trigger (the game fires only from the primary wand's trigger), and blanks both physical triggers. The remap applies to every OpenVR state read except the UI's raw read, so Mods Config reads the remapped state too.

The remap stops behind an input-blocking menu (`isInputBlockingMenuActive`: a game-stopping input context or FRIK's compatibility config). The Mods Config menu is neither, so the remap stays on while it's open.

## Options

1. **Menu events from the native client.** Once ROCK connects to Mods Config (`mods-config-native-api.md`), `subscribeToMenuEvents` tells it when the menu opens and closes. Count an open Mods Config menu as input-blocking, as the game's menus are.
2. **Detect the menu without the client.** Check whether Mods Config's menu pushes an input context or a game menu ROCK can see, and add it to `kRemapBlockingInputContexts` or the tracked menus. Works when Mods Config is installed but ROCK's client isn't connected.
3. **Ask Mods Config to read raw controller state** for its pointer. A change in Mods Config, and other mods that remap input would still affect it.

Lean: option 1 when the native client lands; check option 2 first, since it needs no client.

## To check

- Whether the left trigger also fires the gun while the menu is open (the game sees the remapped trigger behind a menu that doesn't block input).
- The same with RPS UI's settings menu and RobCo PALM, which also aren't game menus.
