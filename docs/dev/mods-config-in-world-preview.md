# Ticket: Show in the world what a Mods Config row sets

**Status:** implemented; the headset check is left.

## Goal

While the player points at **Grip Zone Marker Size** or **Firing Grip Reach** (Weapons page, Fine-Tuning), show it on the drawn weapon. Reference: Mods Config's `docs/for-mod-authors/native-api.md` ("What the Ray Is On").

## Decisions

- **Pointing only.** Markers show while the ray is on the row; Mods Config keeps that across the gaps between rows, while the row's editor is open and during Adjust in world. No page `id`, no `subscribeToMenuEvents` use.
- **Explicit ids.** The two settings get `id`s in `spec.json`: `gripZoneMarkerSize` and `firingGripReach`.
- **Polled at FrameBegin.** `ModsConfigClient` asks `isPointedAt` for both ids where it already polls whether the menu is shown, and exposes one enum (none, marker size, reach). Mods Config calls everything on the main thread. The marker code in `EquippedWeaponFrame` reads it.
- **A drawn weapon only.** Without one nothing shows. Both rows' help gains "Draw a weapon to see it." (catalog help in `src/config/SettingMetadata.h` and `spec.json` together, then regenerate the examples).
- **Grip Zone Marker Size** shows every grip zone marker of the drawn weapon (authored support zones and the firing grip marker) at the previewed size, whatever their usual conditions (a hand holding, the frame's `visible` flag). Path: `publishGripZoneIndicatorRenderFrame` (`core/interaction/EquippedWeaponFrame.cpp`).
- **Firing Grip Reach** draws the regrab zone as a wireframe outline in the marker color, built from the same `tryBuildFiringGripZoneInput` (`weapon/grip/FiringGrip.cpp`) the regrab test uses: the quarter-disc arcs on both sides for Sector (default), rings and joining lines for Cylinder (a handling provider's opt-in). The second weapon has no firing grip reach, so only the main weapon draws it. Lines go through RPS UI's world draw (`visual/WorldDrawClient.h`).
- **No new test.** Drawing only; the zone math is `FiringGripReattachZonePolicy`'s. Checked in the headset.
- **Player guide:** one line in `docs/README.md`'s Weapons settings saying pointing at either row shows it on the drawn weapon.

ROCK's frame runs behind Mods Config's menu: only game-stopping menus and the compatibility config block it (`isInputBlockingMenuActive`, `input/InputRemapRuntime.cpp`).

## Work

- [ ] Headset check: both rows on a weapon with authored grips and on one without, a Cylinder-zone weapon if one is at hand, the outline following the slider, nothing without a drawn weapon, and both clearing as the ray leaves, the menu closes or minimizes.
