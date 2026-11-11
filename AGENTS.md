# AGENTS.md - ROCK

Orientation for any agent working on ROCK or building a mod on top of it.

ROCK is a Fallout 4 VR F4SE plugin (`ROCK.dll`) that gives the player physical hands: grabbing, hand, body and weapon collision, equipped-weapon handling, and a provider API other F4SE plugins consume. It runs inside FRIK's frame phases (FRIK 0.79.1 or newer, FRIK API 2.3) and links CommonLibF4VR directly. It targets Fallout 4 VR 1.2.72.0 only.

## Building a mod on ROCK

- ROCK publishes provider API 2.0: 13 interfaces whose headers are in `include/ROCK/`. They include only the C++ standard library; nothing links against ROCK.
- Add ROCK's `include/` directory through a CMake cache variable such as `ROCK_INCLUDE_DIR`, include `<ROCK/...>`, resolve `ROCKAPI_QueryInterfaceV1` from the loaded `ROCK.dll`, and connect with `rock::api::Client`. Never include headers from `src/` or copy ROCK's headers into the consumer.
- Read `docs/api/Overview.md` first, then the page of each interface you use. The pages state each function's thread, permission and limits.
- When ROCK is absent, the consumer turns its ROCK features off and keeps running. A version mismatch never runs silently: ROCK names a consumer built for another provider API in an error box at the main menu and closes the game when the box is closed. A consumer that finds a loaded ROCK without the API it was built for does the same; RPS Framework's `RPS/Runtime/RequiredUpdate.h` provides the box.
- Request exactly the major, minor and table size your headers declare (`rock::api::Client` does). Never probe for optional versions with trial queries: every version refusal stops the game.

## Repository map

| Path | Contents |
| --- | --- |
| `src/ROCKMain.cpp` | F4SE Query/Load, version guard, FRIK registration, lifecycle |
| `src/physics-interaction/` | Physics, grabbing, collision, weapons, input, animation and the rest of the runtime, one folder per subsystem |
| `src/api/` | Provider API: discovery, owner registry, the per-interface tables (`providers/`) and their services |
| `src/config/`, `src/RockConfig*.cpp` | Settings catalog (`SettingMetadata.h`), the old INI key table, and ROCK's settings on RPS Framework's settings store (`RPS::Settings`) |
| `src/rock_support/` | Logging, runtime helpers, menu tracking and controller input |
| `src/setup-wizard/` | The setup wizard: an RPS UI panel for the function bindings and the main options |
| `include/ROCK/` | Public provider API headers, the only copy |
| `docs/api/` | One page per provider interface |
| `data/config/` | Example TOML settings files, generated from the catalog |
| `data/mod/` | Game data shipped in the mod package, when there is any (absent today) |
| `steamvr_bindings/` | SteamVR binding templates for non-Quest controllers, shipped in release packages; the Index and Vive ones are also embedded in `ROCK.dll` (`cmake/steamvr_bindings.rc.in`) for the setup wizard |
| `third_party/openvr/` | Vendored OpenVR ABI for raw controller input and haptics |
| `tests/`, `tools/Invoke-RockTests.ps1` | Tests and the affected-test selector |
| `cmake/` | Version resource, test registration and release packaging |

## Build and test

Requirements: Windows x64, Visual Studio 2022 (v143), CMake 4.2 or newer, PowerShell 7, vcpkg with `VCPKG_ROOT` set, and checkouts of CommonLibF4VR, RPS UI Framework and RobCo PALM (their `include/` headers; RPS UI's `support/` headers too), Dear ImGui at the revision RPS UI Framework's `docs/api/Framework.md` names, and RPS Framework (built with its `custom-fast` preset, which stages its package). Set `RPSUI_INCLUDE_DIR`, `PALM_INCLUDE_DIR`, `ROCK_IMGUI_DIR` and `RPSFramework_DIR` (the staged package's `lib/cmake/RPSFramework`) when they are not where `CMakeLists.txt` looks by default. ROCK uses C++23.

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

`custom-release` builds into `build-release` and writes a release's two files to `build-release/package`: a `.zip` that installs as it is, holding the DLL, any `data/mod` game data and `steamvr_bindings/`, and `README.md` beside it. The PDB, the license, headers and every other development file stay out of the zip.

## Runtime files

- Plugin: `Data/F4SE/Plugins/ROCK.dll`. Log: `ROCK.log` in `Documents/My Games/Fallout4VR/F4SE/`.
- Settings: `ROCK.toml` and the optional `ROCK_Developer.toml` in `Documents/My Games/Fallout4VR/Mods_Config/ROCK/`, with `SavedGrabOffsets/`, `AuthoredWeaponGripCache/` and `SteamVR/` (the binding templates the setup wizard installs) beside them. Runtime code resolves the Documents known folder.
- SteamVR: on the player's choice in the setup wizard, ROCK sets Fallout 4 VR's legacy binding for Index controllers or Vive wands through OpenVR's `IVRSettings` (`src/setup-wizard/SteamVrBinding.cpp`), only over SteamVR's default binding and only when that interface is SteamVR's own (not OpenComposite's). A binding the player chose in SteamVR is never replaced, and ROCK never edits SteamVR's files.
- Co-save: unique ID `ROCK`, record `HNDS` version 1, written by `src/physics-interaction/weapon/equip/WeaponHandSave.cpp`: which hand holds each equipped weapon and each hand's one-handed melee reverse grip. A layout change needs a new record version.

## What ROCK does not own

- **NPC interactions** belong to SCISSORS, which builds on the NativeInteraction interface and creates, reads and retires its own NPC bodies and constraints. ROCK keeps only shared mechanisms: native allocation and quarantine, callback lifetime, hand and weapon authority, connected-body grab execution, native safety and the grenade lifecycle. For blade contact, ROCK publishes the contact point, the optional manifold normal and count, and the manifold's signed separation (Havok publishes contacts before shapes touch), and keeps a weapon its driver marks lodged in a body after the hand lets go (`src/physics-interaction/weapon/equip/LodgedWeapons.cpp`: the loose weapon takes the target body's collision group and rides it by velocity, never constrained to an engine body); blade profiles, melee damage hooks and actor-limb policy are not ROCK's. ROCK has no NPC settings.
- **World drawing** belongs to RPS UI Framework. ROCK's indicators and debug shapes go through its world drawing (`src/physics-interaction/visual/WorldDrawClient.cpp`) and the ammo counters through its number overlays (`src/physics-interaction/visual/AmmoCounters.cpp`); ROCK has no renderer. It keeps only its own hand and weapon visualization setting.
- **Reload and manual-cycle animation** belong to PAPER.
- **Settings screens** (RobCo PALM) read ROCK's catalog and write through the Configuration interface. Nothing else writes ROCK's settings files.

## Invariants

### Interaction

- A pull or forced arrival of a loose non-weapon or throwable seats from, per hand: the player's saved offset, a provider default pose, ROCK's built-in pose (`src/physics-interaction/grab/BuiltInGrabOffsets.h`), the library pose, then the dynamic seat. The library pose (`src/physics-interaction/grab/GrabPoseLibrary.h`, `Hand::prepareLibraryGrabPose`) is borrowed from the authored objects shaped most like the object: every authored pose with a known mesh is an exemplar in `GrabPoseLibraryData.cpp`, generated with the built-in table from the grab-offset authoring data (never edited by hand), the candidates are tried with the finger solve, and the result is kept per base form, scale and count-based ammo variant (a single round and a pile are different meshes). `ROCKGrabPoseLibraryTests` holds the shape functions to the authoring tool's reference. Fixed poses seat every grab, close ones included, and replace the pinch: the immersive aid pose, and a grenade's pose (its own pose, otherwise ROCK's role pose `built_in::kGrenadePose`, or `kMolotovPose` for a Molotov); the one exception is a second hand joining a grenade the other hand holds, which grabs dynamically (`Hand::resolveAuthoredGrabOffset`). `BuiltInGrabOffsetData.cpp` is generated from the grab-offset authoring data: regenerate it, never edit it by hand. A saved offset of an object whose bodies move against each other (ragdoll and skinned clutter, tongs) is measured against the node of the grabbed body (`HandOffset::bodyNode`) and applies only to a grab of that body; a root-relative offset is never applied to such an object.
- A close grab holds the object where the hand found it only when a finger probe touches the visual mesh (`Hand::evaluateGrabProbeContact`, the same solve the probe overlay draws), or the mesh crosses the straight line between the live thumb and index tips: an object smaller than the fingers lies on neither closing arc. Without that contact, whatever a pull could bring comes to the hand and seats like a pull that has just arrived (a reach arrival, `Hand::grabSelectedObject`): a loose object in its authored or library pose, a gun or melee weapon in its authored grip. A body (an actor's limb, a corpse, a severed part) cannot come to the hand: the grab takes its mesh point nearest the palm within close reach and the hold closes the gap. Any other object the other hand holds is refused and retried while the grip is held. Two hands on one loose object aim it along the line between their grip points (`Hand::coordinateLooseObjectProxy`), except when those points are under 6 game units apart on a non-weapon: hand tremor is larger than that, so the object is not aimed, it stays as the first hand holds it, and when either hand lets go the other holds it the way it grabbed it. Native collision never stands in for the mesh: the held point is always a mesh point or an authored weapon seat, and a pull flies a visual mesh point to the palm, not the collision's centre of mass. When the arrival will seat an authored pose, the pull flies that point to its seated position on the live palm and turns the object toward its seated orientation for the whole flight (`Hand::capturePullSeat`), so it arrives already posed; the catch only completes the last units. ROCK-placed arrivals (pull catch, force grab, weapon transfer) seat the mesh themselves and skip the probe check. A grab refused for no probe contact leases the object to that hand (`Hand::updateMeshReachLease`): its colliders and the proxies pass through the object's collision until the grab lands, or until the grip is released and the hand is outside the object's bound.
- Once that contact exists, a grab always places the object. Never answer a poor grab pose by rejecting the grab; improve the seat and the finger posing. A held object always ends in the palm, locked to the hand (`placeHeldBodiesOnHandTarget`); a grab is never marked held while the object is still away from the hand. A pinch grab locks the object where thumb and finger found it instead of the palm: the object is not moved, only the fingers close onto it, and thumb and index stop at the object's thickness (the pinch closure), never past it; each then bends its three joints separately to bring its tip onto the mesh (`reachPinchFinger`), because a small object rarely lies on a finger's single-curl arc. Count-based loose ammo is one object for the pinch: its carried root body is not a second body. A close grab is a pinch whenever the object sits between thumb and index (its mesh span along the pinch axis fits the live gap, `grab_pinch_pocket_policy`), whatever the object's size: plants, handles and rims included. A live NPC's limb is the one exception to the lock: its body keeps it, so the hold is reduced to a point at a fraction of an object's force (`LIVE_LIMB_AUTHORITY_SCALE`, `LIVE_LIMB_ANGULAR_SHARE`) and the limb may lag the hand, because a full-force hold pulls his joints apart.
- Per-frame limits (distance per frame, frame-time gates, sample ages, retention windows) scale with frame length (`frameMotionScale`, `ContactClock::freshnessWindowSeconds`), never with 90 Hz frame counts. Players on slow PCs rarely send logs, so their bugs have to be prevented in code.
- Every IVRSystem controller-state read passes through the `[Controls]` button layout (`src/physics-interaction/input/ButtonLayoutPolicy.h`) before ROCK's capture, so ROCK, its consumers and the game read one logical Quest-layout button set. Features read those logical bits; none keeps its own physical button table. The `[Bindings]` composer (`src/physics-interaction/input/FunctionBindingPolicy.h`) then presents each moved function as a press of its home button, tagged with the function; the policies that split one button between two functions act on the tag. Default bindings leave every read untouched.

### Provider API

- Public headers include only the C++ standard library and other `include/ROCK/` headers: never CommonLibF4VR, Havok, engine or F4SE types.
- Every entry point validates the caller's record `size` and `version` before reading it. Records use fixed-width types.
- Every version refusal from `ROCKAPI_QueryInterfaceV1` (unknown interface, other major, newer minor, larger table) is logged with the calling DLL and stops the game at the main menu (`src/api/OutdatedConsumers.cpp`).
- A released interface version is frozen. A compatible addition goes at the end of a table; an incompatible change needs a new major. Retired interface IDs, slots, enum values and flag bits are marked in the headers and never reused.
- A header change updates its `docs/api/` page in the same commit and passes `ROCKPublicHeaderTests`.

### Configuration

- `src/config/SettingMetadata.h` is the single source of every option: TOML table path, key, owning file (its `Group`), category and help text; `RockConfigLoad.cpp` declares each option's type and default through the read it makes. An option is identified by its exact, case-sensitive table and key, and belongs to exactly one of `ROCK.toml` (players) and `ROCK_Developer.toml` (developer overrides). Construction fails when the catalog and the loader disagree.
- Values are typed: booleans, whole numbers, decimal numbers (a whole number is accepted), one-line strings, and `Float3` lists of exactly three numbers for vectors and `[pitch, yaw, roll]` rotations.
- `data/config/ROCK_example.toml` and `ROCK_Developer_example.toml` document every option. Regenerate them with `ROCKConfigurationStoreTests --write-examples data/config` after any catalog change; the test fails when they differ. They are references only: never read, embedded, packaged, deployed or used to create runtime files. Settings files (TOML or INI) never ship in the mod package.
- RPS Framework's settings store (`RPS::Settings`, `src/config/RockSettings.cpp` supplies ROCK's catalog, file names, headers and old INI keys) reads and writes the files; its own tests cover the file behaviour below. At startup and on every configuration API write, each file that parses is organized the same way: every recognized key once, grouped by table in catalog order under its category heading, preceded by the catalog's help as `##` lines. Single-`#` lines above a key are the player's notes and stay with it. `ROCK.toml` also gains every missing player option at its default. Entries ROCK does not read from that file (retired options, options owned by the other file, foreign keys) become `#` comment lines in the trailing "Not read by this version of ROCK" block and never apply. A recognized value of the wrong type is kept as written, logged with its line, and the option keeps its last value. Startup writes a file only when its organized text differs; hot reload only reads.
- A file that does not parse is never rewritten: the error is logged with file, line and column, that file's options keep the last values read from it, the other file still applies, and API writes to it are refused until it is fixed.
- A missing `ROCK.toml` is created from the compiled defaults and the catalog, with no templates, resources or fallback paths. At startup only, when `ROCK.toml` is missing, the INI files of earlier versions (`ROCK.ini`, `ROCK_Developer.ini`) are migrated once: each option the player changed from its old default lands in the TOML file that owns it now, through the frozen key table `src/config/LegacyIniMigrationTable.inl` (never regenerated; the store refuses to start when a row names a missing option or the wrong type). The developer file is written before `ROCK.toml`, both are read back, and only then are the INIs deleted; any failure keeps them for the next start. When `ROCK.toml` exists, leftover INIs are deleted unread. `ROCK_Developer.toml` is never created by loading: it appears only when a developer setting differs from its default, holds only overrides, loses a key restored to its default, and is removed when no read entries remain. Entries a supplied developer file sets to defaults are kept.
- Both files reload while the game runs, including the developer file appearing and disappearing.
- Mandatory behaviour has no settings switch: weapon collision and its generation mode (only the per-TriShape path), dynamic hand collision, interaction and haptics, body colliders, native character-controller filtering, the far-selection HMD cone, grabbing on the logical OpenVR grip bit, native V.A.N.S. suppression, and grab, surface, impact, shoulder-stash and mouth-consume haptics. ROCK's APIs control only behaviour ROCK already owns.
