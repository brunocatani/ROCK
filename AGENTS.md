# AGENTS.md - ROCK

Orientation for any agent working on ROCK or building a mod on top of it.

ROCK is a Fallout 4 VR F4SE plugin (`ROCK.dll`) that gives the player physical hands: grabbing, hand, body and weapon collision, equipped-weapon handling, and a provider API other F4SE plugins consume. It runs inside FRIK's frame phases (FRIK 0.79.1 or newer, FRIK API 2.3) and links CommonLibF4VR directly. It targets Fallout 4 VR 1.2.72.0 only.

## Building a mod on ROCK

- ROCK publishes provider API 2.0: 13 interfaces whose headers are in `include/ROCK/`. They include only the C++ standard library; nothing links against ROCK.
- Add ROCK's `include/` directory through a CMake cache variable such as `ROCK_INCLUDE_DIR`, include `<ROCK/...>`, resolve `ROCKAPI_QueryInterfaceV1` from the loaded `ROCK.dll`, and connect with `rock::api::Client`. Never include headers from `src/` or copy ROCK's headers into the consumer.
- Read `docs/api/Overview.md` first, then the page of each interface you use. The pages state each function's thread, permission and limits.
- When ROCK is absent or refuses discovery, the consumer turns its ROCK features off and keeps running.

## Repository map

| Path | Contents |
| --- | --- |
| `src/ROCKMain.cpp` | F4SE Query/Load, version guard, FRIK registration, lifecycle |
| `src/physics-interaction/` | Physics, grabbing, collision, weapons, input, animation and the rest of the runtime, one folder per subsystem |
| `src/api/` | Provider API: discovery, owner registry, the per-interface tables (`providers/`) and their services |
| `src/config/`, `src/RockConfig*.cpp` | Settings catalog (`SettingMetadata.h`) and INI loading, organizing and saving |
| `src/rock_support/` | Logging, runtime helpers, menu tracking and controller input |
| `include/ROCK/` | Public provider API headers, the only copy |
| `docs/api/` | One page per provider interface |
| `data/config/` | Example INIs, generated from the catalog |
| `data/mod/` | Files shipped in the mod package |
| `steamvr_bindings/` | SteamVR binding templates for non-Quest controllers, shipped only in release packages |
| `third_party/openvr/` | Vendored OpenVR ABI for raw controller input and haptics |
| `tests/`, `tools/Invoke-RockTests.ps1` | Tests and the affected-test selector |
| `cmake/` | Version resource, test registration and release packaging |

## Build and test

Requirements: Windows x64, Visual Studio 2022 (v143), CMake 4.2 or newer, PowerShell 7, vcpkg with `VCPKG_ROOT` set, and a CommonLibF4VR checkout. ROCK uses C++23.

Copy `CMakeUserPresets.json.template` to `CMakeUserPresets.json` and set `COMMON_LIB_F4VR_PATH` and `COPY_PLUGIN_BASE_PATH` (the mod folder the build copies `ROCK.dll` and `ROCK.pdb` into, under `F4SE/Plugins`). Then:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK
```

Tests live in `build-fast` but outside the default build. Every test target name contains `Tests`. Run the tests a change affects with:

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

The selector picks tests from staged, unstaged and untracked files (or the latest commit on a clean tree) and widens the selection when the mapping is unsafe. `-PlanOnly` shows the selection, `-BaseRef <ref>` covers a committed range plus the working tree, and `-All` runs everything. To build and run every test directly: `cmake --build build-fast --config Release --target build_tests`, then `ctest --test-dir build-fast -C Release --output-on-failure`.

Tests link CommonLib's real `NiPoint.cpp` and `NiMatrix3.cpp`; never re-implement engine math in test stubs. `ROCKPublicHeaderTests` compiles every public header on its own and checks the ABI layout of published records and tables.

`custom-release` builds into `build-release` and packages a `.7z` with the DLL and PDB, `data/mod`, the license, `include/ROCK/` and `steamvr_bindings/`.

## Runtime files

- Plugin: `Data/F4SE/Plugins/ROCK.dll`. Log: `ROCK.log` in `Documents/My Games/Fallout4VR/F4SE/`.
- Settings: `ROCK.ini` and the optional `ROCK_Developer.ini` in `Documents/My Games/Fallout4VR/Mods_Config/ROCK/`, with `SavedGrabOffsets/` and `AuthoredWeaponGripCache/` beside them. Runtime code resolves the Documents known folder.
- Co-save: unique ID `ROCK`, record `HNDS` version 1, written by `src/physics-interaction/weapon/equip/WeaponHandSave.cpp`: which hand holds each equipped weapon and each hand's one-handed melee reverse grip. A layout change needs a new record version.

## What ROCK does not own

- **NPC interactions** belong to SCISSORS Redux, which builds on the NativeInteraction interface and creates, reads and retires its own NPC bodies and constraints. ROCK keeps only shared mechanisms: native allocation and quarantine, callback lifetime, hand and weapon authority, connected-body grab execution, native safety and the grenade lifecycle. For blade contact, ROCK publishes the contact point and the optional manifold normal and count; blade profiles, melee damage hooks and actor-limb policy are not ROCK's. ROCK has no NPC settings.
- **World drawing** belongs to RPS UI Framework. ROCK's indicators and debug shapes go through its world drawing (`src/physics-interaction/visual/WorldDrawClient.cpp`) and the ammo counters through its number overlays (`src/physics-interaction/visual/AmmoCounters.cpp`); ROCK has no renderer. It keeps only its own hand and weapon visualization setting.
- **Reload and manual-cycle animation** belong to PAPER.
- **Settings screens** (RobCo PALM) read ROCK's catalog and write through the Configuration interface. Nothing else writes ROCK's INIs.

## Invariants

### Interaction

- A grab always places the object. Never answer a poor grab pose by rejecting the grab; improve the seat and the finger posing. A held object always ends in the palm, locked to the hand (`placeHeldBodiesOnHandTarget`); a grab is never marked held while the object is still away from the hand.
- Per-frame limits (distance per frame, frame-time gates, sample ages, retention windows) scale with frame length (`frameMotionScale`, `ContactClock::freshnessWindowSeconds`), never with 90 Hz frame counts. Players on slow PCs rarely send logs, so their bugs have to be prevented in code.
- Every IVRSystem controller-state read passes through the `[Controls]` button layout (`src/physics-interaction/input/ButtonLayoutPolicy.h`) before ROCK's capture, so ROCK, its consumers and the game read one logical Quest-layout button set. Features read those logical bits; none keeps its own physical button table.

### Provider API

- Public headers include only the C++ standard library and other `include/ROCK/` headers: never CommonLibF4VR, Havok, engine or F4SE types.
- Every entry point validates the caller's record `size` and `version` before reading it. Records use fixed-width types.
- A released interface version is frozen. A compatible addition goes at the end of a table; an incompatible change needs a new major. Retired interface IDs, slots, enum values and flag bits are marked in the headers and never reused.
- A header change updates its `docs/api/` page in the same commit and passes `ROCKPublicHeaderTests`.

### Configuration

- `src/config/SettingMetadata.h` is the single source of every option: section, key, default, owning file (its `Group`) and help text. Each key belongs to exactly one of `ROCK.ini` (players) and `ROCK_Developer.ini` (developer overrides).
- `data/config/ROCK_example.ini` and `ROCK_Developer_example.ini` document every option. Regenerate them with `ROCKConfigurationStoreTests --write-examples data/config` after any catalog change; the test fails when they differ. They are references only: never read, embedded, packaged, deployed or used to create runtime files. INIs never ship in the mod package.
- At startup and on every configuration API write, each existing file is organized the same way: every recognized key once, with its effective (last) value, under its catalog section and category, preceded by the catalog's help as `;` lines. `#` lines above a recognized key are the player's notes and stay. `ROCK.ini` also gains every missing player option at its default. Entries ROCK does not read from that file (retired options, options owned by the other file, foreign keys) become comment lines in the trailing `[Unused]` block and never apply. Startup writes a file only when its organized text differs; hot reload only reads.
- A missing `ROCK.ini` is created from the compiled defaults and the catalog, with no templates, resources or fallback paths. `ROCK_Developer.ini` is never created by loading: it appears only when a developer setting differs from its default, holds only overrides, loses a key restored to its default, and is removed when no read entries remain. Entries a supplied developer file sets to defaults are kept.
- Both files reload while the game runs, including the developer file appearing and disappearing.
- Mandatory behaviour has no INI switch: weapon collision and its generation mode (only the per-TriShape path), dynamic hand collision, interaction and haptics, body colliders, native character-controller filtering, the far-selection HMD cone, grabbing on the logical OpenVR grip bit, native V.A.N.S. suppression, native right-stick Favorites suppression, and grab, surface, impact, shoulder-stash and mouth-consume haptics. ROCK's APIs control only behaviour ROCK already owns.
