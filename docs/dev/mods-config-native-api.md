# Ticket: Mods Config native API in ROCK (live preview and more)

**Status:** open, not started. **Depends on:** `mods-config-setup-wizard-launch.md`, which connects the client first.

## Goal

Connect `ROCK.dll` to Mods Config FO4 VR's native API, so its page previews changes live and gains what only ROCK's code knows. Reference: Mods Config's `docs/for-mod-authors/native-api.md` and the client header `src/api/ModsConfigFO4Api.h` (copied into a mod as-is). ROCK isn't built on the F4VR Common Framework, so nothing connects it automatically: ROCK connects its own client under `modName` `ROCK` from `kPostLoad`, and runs fine when Mods Config isn't installed.

## Today

- `spec.json` has `applies: save`. A Save writes `ROCK.toml`, and ROCK's file watcher reloads it about 200 ms later, on `FrameBegin` (`RockConfig::processPendingConfigReload`).
- Without a client the page has no version line and no preview.
- The client itself, and the setup wizard button and quick launch entry, come from `mods-config-setup-wizard-launch.md`.

## What the client would add

| API | Use in ROCK |
| --- | --- |
| `subscribeToChanges` (live preview) | Apply each pending value without writing the file; restore it on Discard. Then set `applies: live` on the file. |
| `subscribeToSaves` | Reload at once after a Save instead of waiting for the watcher. Optional: the watcher already works. |
| `addState("modVersion")` | `Version {clientState:modVersion}` on the front page, as FRIK and Immersive Flashlight show it. |
| `addState("configFile")` | Not needed: ROCK's file is always `Mods_Config/ROCK/ROCK.toml` under the Documents known folder, which is the menu's `myGames` root. |
| `isPointedAt` / `subscribeToPointing` | Show in the world what a row sets while the player points at it: grip zone markers for Grip Zone Marker Size, the reach zone for Firing Grip Reach. |
| `subscribeToMenuEvents` | Show those markers while the Weapons page is shown (give it an `id`). |

## Design questions

- **Where a preview lives.** ROCK's values come from RPS Settings (`src/config/RockSettings.cpp`) through `RockConfig::parseValues`. A preview needs an in-memory overlay over the loaded values: a pending map that `reload()` applies on top of the file, cleared on Discard and on Save. Decide whether that belongs in ROCK (`RockConfig`) or in RPS Settings (an override layer other mods could use too).
- **Threads.** Mods Config calls on the game's main thread. ROCK changes settings on the frame thread at `FrameBegin`. Queue previews and apply them there, as the watcher does.
- **Settings that don't apply live.**
  - `EnableImmersiveScopes` takes the game's scope hook until restart.
  - `ImmersiveGrenades` and `EnableImmersiveScopes` need their button released.
  - The Button Layout and Bindings change input mid-press.
  - Mark these `applies: save` or `restart` in the spec, or have the preview skip them.
- **The wizard during a preview.** The setup wizard saves straight to `ROCK.toml`. Decide what a pending preview does when the wizard opens from the page.

## Work

- [ ] States: `modVersion`.
- [ ] Preview overlay, applied on `FrameBegin`; Discard restores it; Save clears it.
- [ ] Spec: version text, `applies: live` with per-setting exceptions, page `id`s for pointing.
- [ ] Test the preview overlay's contract (preview, then Discard, gives the file's values; preview, then Save, gives the saved values).
- [ ] Adjust in world for the firing-hand transforms (`AimOffset`, `GripOffset`, `SupportGripOffset`): check which frame each position is in (the grip offsets are hand-local: along the fingers, palm, across; the aim offset's frame isn't documented), and that the sticks move the value along the axes the player sees, also on the mirrored right hand.
