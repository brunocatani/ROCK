# Ticket: Mods Config native API in ROCK (live preview)

**Status:** in progress. The client exists (`src/mods-config/ModsConfigClient.cpp`, from `mods-config-setup-wizard-launch.md`); this ticket extends it. Showing in the world what a row sets, and Adjust in world for the firing-hand transforms, moved to `mods-config-in-world-preview.md`.

## Goal

Connect `ROCK.dll` to Mods Config FO4 VR's native API, so its page previews changes live and shows ROCK's version. Reference: Mods Config's `docs/for-mod-authors/native-api.md` and the client header `src/api/ModsConfigFO4Api.h` (copied into a mod as-is). ROCK isn't built on the F4VR Common Framework, so nothing connects it automatically: ROCK connects its own client under `modName` `ROCK`, and runs fine when Mods Config isn't installed.

## Today

- `spec.json` has `applies: save`. A Save writes `ROCK.toml`, and ROCK's file watcher reloads it about 200 ms later, on `FrameBegin` (`RockConfig::processPendingConfigReload`).
- Without a client the page has no version line and no preview.
- The client connects as `ROCK` and has the setup wizard's function (`mods-config-setup-wizard-launch.md`).

## Design

- **Preview values** live in `RockConfig`: a map from `settingPath(table, key)` to the value's canonical text (`normalizeValue`). A change of a `live` setting sets its entry; a value that doesn't normalize is logged and dropped. A Discard removes the entry, so the file's value shows through. Mods Config's baseline value isn't used.
- **Applying:** the change handler only records the value and raises a flag. The next `FrameBegin` (`processPendingConfigReload`) applies every recorded change in one reparse, without the file watcher's 200 ms debounce: the store's cached loaded values with the preview values on top, through `parseValues`. No file is read, and the reparse logs at DEBUG. Leaving the mod or a Discard sends one change per setting, and queuing makes them one reparse. Values change in one place in the frame.
- **Saves:** a Save clears the preview values and reloads the files at once, on the next `FrameBegin`. The watcher's later reload finds nothing new.
- **Menu closed:** clears any preview values left, in case a Discard never came.
- **`configRevision()`** advances with every applied reload, of the files or of the preview values.
- **Bindings:** a reload reconfigures the function bindings, and resets their gesture state, only when `[Bindings]` changed. Before, every reload reset it, so a slider dragged in the menu cancelled a held binding.
- **Spec:** `applies: live` for the config spec, the Developer page included. `applies: save` on:
  - the Controls page: the button layout and bindings would swap mid-press while the player clicks in the menu, and Block VATS is read inside the B-button decision;
  - Immersive Scopes: turning it on takes the game's scope hook for good, and it needs the scope button released;
  - Immersive Grenades: it needs the grenade button released.
- **`modVersion`** state: ROCK's version, shown as `Version: {clientState:modVersion}` on the front page, as FRIK and Immersive Flashlight show it.
- **Setup wizard:** unchanged. It writes `ROCK.toml` while the menu is minimized with its edits pending, and a setting the player is previewing keeps its preview value over the file's, as the page shows it.

## Known limits

- Settings that act only on the next grab, equip or draw (`FarGrabStyle`, `GripOffset`, `LooseGrabFingerPoseMode`, ...) preview with no visible change until then. Their help says so.
- OpenVR input threads read `rockEnableImmersiveScopes`, `rockBindingSeconds` and `rockFunctionBindings` without a lock while a reload assigns the whole `RockConfigValues` on the frame thread. A preview reload writes the bytes those fields already hold (they are `save` settings), so it adds nothing to the race a Save or a file edit already has. No work planned.

## Work

- [ ] Preview values in `RockConfig`, applied on `FrameBegin`; Discard restores the file's value; Save and menu close clear them.
- [ ] Client: change, Save and menu event handlers, `modVersion` state.
- [ ] Bindings reconfigured only when they changed.
- [ ] Spec: version text, `applies: live` with the exceptions above; the Developer page's generator text.
- [ ] Test the preview contract in `RockConfigTests` (preview, then Discard, gives the file's values; preview, then Save, gives the saved values).
- [ ] Time a preview reparse with the profiler in game; consider applying only the changed field if it costs over about 0.3 ms.
