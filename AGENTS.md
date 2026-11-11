# AGENTS.md - ROCK

ROCK is a Fallout 4 VR F4SE plugin (`ROCK.dll`) that gives the player physical hands: grabbing, hand, body and weapon collision, equipped-weapon handling, and a provider API other plugins consume. It runs inside FRIK's frame (FRIK 0.79.1 or newer, FRIK API 2.3), links CommonLibF4VR directly and targets Fallout 4 VR 1.2.72.0 only.

To change ROCK: read [Architecture](#architecture), find the task in [Where to start](#where-to-start), and read the opening comment of each file it names; almost every file opens with one saying what it owns. [Invariants](#invariants) lists what every change must keep true.

## Building a mod on ROCK

- Provider API 2.0: 13 interfaces in `include/ROCK/`, standard library only, nothing to link. Add ROCK's `include/` through a CMake cache variable such as `ROCK_INCLUDE_DIR`, resolve `ROCKAPI_QueryInterfaceV1` from `ROCK.dll` and connect with `rock::api::Client`. Never include `src/` headers or copy ROCK's headers.
- Read `docs/api/Overview.md`, then each interface's page; they state every call's thread, permission and limits.
- A mod built for ROCK 0.92: follow `skills/rock-1.0-migration/SKILL.md` (it also covers PAPER, RPS UI Framework and RobCo PALM) and `docs/api/Migration.md`.
- Without ROCK, turn ROCK features off and keep running. Request exactly the versions your headers declare and never probe: every version refusal shows an error box at the main menu and closes the game.

## Repository map

| Path | Contents |
| --- | --- |
| `src/` | The plugin (see [Source map](#source-map)) |
| `include/ROCK/` | Public provider API headers, the only copy |
| `docs/api/` | One page per interface, and `Migration.md` for mods built for ROCK 0.92 |
| `skills/rock-1.0-migration/` | Agent skill for updating a mod built for ROCK 0.92 |
| `tests/`, `tools/Invoke-RockTests.ps1` | Tests and the affected-test selector |
| `preview/` | The setup wizard preview (`ROCKSetupWizardPreview.exe`): the wizard on the desktop, without the game |
| `data/config/` | Example settings files, generated from the catalog |
| `data/mod/` | Game data for the mod package, if any |
| `steamvr_bindings/` | SteamVR binding templates; shipped in the package and embedded in `ROCK.dll` for the setup wizard |
| `third_party/openvr/` | Vendored OpenVR ABI |
| `cmake/` | Version resource, test registration, packaging |

## Architecture

### Lifecycle

`src/ROCKMain.cpp` is the entry point and frame scheduler.

1. `F4SEPlugin_Query` rejects the editor, non-VR and any `Fallout4VR.exe` other than 1.2.72.0. `F4SEPlugin_Load` installs the process-lifetime hooks.
2. On `kGameLoaded`, ROCK binds FRIK's API v2 (`src/api/FRIKApiV2.h`), loads its settings, installs the hooks that need game data and registers one FRIK frame callback per phase.
3. One `PhysicsInteraction` (`src/physics-interaction/core/PhysicsInteraction.h`) exists per FRIK skeleton: created after `kSkeletonReady`, destroyed on `kSkeletonDestroying` or a new session. Teardown requested off the frame thread waits for the next `FrameBegin`.

### One frame

| FRIK phase | ROCK work |
| --- | --- |
| `FrameBegin` | Pending teardown, settings reload, `PhysicsInteraction` creation, native HUD placement |
| `NativeGraphOutput` | Authored weapon grip capture |
| `AfterArmSolve` | The tick: provider `BeforeRock` callbacks, `PhysicsInteraction::update`, `AfterRock`, `Complete` |
| `AfterWeaponPosition` | Weapon presentation |
| `AfterWorldFinal` | Rendered-pose capture, `finalizeFramePose`, provider publication, `Presented` callbacks |
| `FrameEnd` | A tick without physics when the skeleton phases did not run, so leases still expire |

`PhysicsInteraction::update` (`core/interaction/PhysicsInteractionUpdate.cpp`) stops early without FRIK, a skeleton or a Havok world, or behind a menu; builds one `PhysicsFrameContext`; updates hand and body colliders; runs the equipped-weapon frame; then selection, grab input, held placement, hand collision, haptics, contacts and grenades. `finalizeFramePose` publishes the final collider pose and registers the next physics-step callbacks.

### Threads

- **Frame thread** (FRIK's callbacks, the game thread): owns `PhysicsInteraction`, both `Hand`s and every structural change to ROCK's Havok bodies. Provider calls marked "Game" must come from here.
- **Physics threads**: contact signals (`core/interaction/ContactEvents.cpp`) and the step drive (`native/havok/PhysicsStepDriveCoordinator.h`), which moves ROCK's bodies and the grab authority between substeps. They never change structure; the frame thread changes it only inside `PhysicsCallbackQuiescenceGate::pauseForMutation()`.
- **Any thread**: provider reads marked "Any" copy published snapshots.
- **Background workers** handle file and diagnostic work on copied data: the saved-grab-offset writer, the profiler's report writer and the debug overlay shape builder. Weapon geometry is built in bounded slices on the frame thread (`weapon/collision/WeaponGeometryTask.h`).

### Ownership

`PhysicsInteraction` owns both `Hand`s, the body colliders, the weapon collision of both equipped weapons, loose and lodged weapon collision, the two-handed grip controller (`weapon/grip/TwoHandedGrip.h`), hand collision, haptics and the published provider state. Its implementation is split by topic across `core/interaction/`; `PhysicsInteraction.h` lists the files.

`Hand` (`hand/Hand.h`) owns one hand's state machine, selection, grab, held object, finger pose and colliders, split by stage into `hand/HandGrabPull`, `HandGrabCapture`, `HandGrabSeat`, `HandGrabCommit`, `HandGrabDrive`, `HandHeldObject` and `HandGrabRelease`.

Subsystems read the per-frame snapshot `core/RockRuntimeState.h` instead of polling the engine.

### Provider API path

1. `include/ROCK/<Interface>.h`: the table.
2. `src/api/providers/<Interface>Provider.cpp`: the boundary. `Boundary.h`'s `invoke` checks owner, permission and thread; each entry point checks record `size` and `version`.
3. `src/api/ProviderRuntime.cpp` and `src/api/services/*.inl`: owners, leases, published state.
4. `PhysicsInteraction`: queries in `core/interaction/ProviderQueries.cpp`, commands in `core/interaction/ProviderCommands.cpp` and the topic files.

NativeInteraction and Storage keep table and service together (`src/api/NativeInteractionService.cpp`, `src/api/StorageService.cpp`). Discovery: `src/api/Discovery.cpp`. Refusals and the error box: `src/api/OutdatedConsumers.cpp`.

### Conventions

- `*Policy.h` headers hold decisions and math, mostly free of engine types so tests compile them. Put a new rule in a policy with a test whenever it can be expressed without the engine.
- Native addresses and layouts are named constants (`native/havok/HavokOffsets.h` for Havok). A hook verifies the live bytes at its site before patching; copy an existing `install()` such as `native/safety/NativeShapeCastSafety.cpp`.
- Convert game and Havok units only through `physics_scale` (`native/havok/PhysicsScale.h`; `gameToHavokScale()`, `havokToGameScale()`), never with a literal factor or FRIK's player scale.
- Log through `physics-interaction/PhysicsLog.h`; per-frame paths use the rate-limited `ROCK_LOG_SAMPLE_*`.
- Read settings from `g_rockConfig` (`src/RockConfig.h`).
- Sources are globbed: new files under `src/` need no CMake change. A test is an executable named `*Tests*`, registered with `add_rock_test`.

## Source map

Under `src/physics-interaction/` unless the path says otherwise.

| Folder | Owns |
| --- | --- |
| `core/`, `core/interaction/` | `PhysicsInteraction`, frame pipeline, lifecycle, runtime snapshot, native grab hooks |
| `hand/` | `Hand`, grab stages, hand colliders, hand collision, skeleton and finger reads |
| `grab/` | Grab mechanics: authority proxy, constraint, motion controller, finger solve, pinch, seats, grab poses, touch grab, held placement |
| `body/` | Body bone colliders |
| `object/` | Grab targets: detection, body sets, ragdolls, skinned bodies, count-based ammo |
| `collision/`, `contact/` | Collision layers, suppression ownership, contact classification; the contact clock |
| `weapon/equip/` | Equip, drop, transitions, weapon roster, shoulder stash gesture, co-save, lodged weapons |
| `weapon/grip/` | Two-handed grip, firing seats, grip zones, reverse grip |
| `weapon/authored/` | Authored grips and holding-grip records |
| `weapon/collision/` | Generated weapon collision: geometry, equipped, dynamic, loose and dropped bodies |
| `weapon/handling/` | Handling settings and leases, gunstock mode, aim, FRIK weapon offsets, bipod |
| `weapon/secondary/` | The second equipped weapon |
| `weapon/native/` | Native weapon hooks: equipment by hand, private action graphs, model lifetime |
| `weapon/scope/`, `recoil/`, `parts/`, `classification/`, `scene/` | Manual scope, recoil, provider part targets and drives, classification, scene traversal |
| `input/` | Button layout, function bindings, input remap hooks, grab intent, input leases |
| `stash/`, `consume/`, `grenade/` | Shoulder stash, mouth consume and immersive aid, grenades and Molotovs |
| `animation/` | Authored grip capture from the native graph |
| `visual/`, `debug/` | Hand world authority, ammo counters, indicators, RPS UI drawing; debug overlays |
| `native/havok/` | Havok bodies, shapes, materials, world lock, step drive, timing fix, offsets |
| `native/player/`, `scene/`, `query/`, `safety/` | Character controller, native HUD placement, reference placement, ray and shape casts, native fault boundaries |
| `performance/`, `timing/` | Profiler, game-frame clock |
| `src/api/` | Provider API and FRIK API binding |
| `src/config/`, `src/RockConfig*.cpp` | Settings catalog and loader |
| `src/setup-wizard/` | Setup wizard: its panel (`SetupWizardPanel`, shared with the preview), the game's host and SteamVR binding |
| `src/rock_support/` | Logger, OpenVR controller state, menus, game INI overrides, developer telemetry |

## Where to start

| To work on | Start in |
| --- | --- |
| Grab input and selection | `core/interaction/GrabInput.cpp`, `hand/HandSelection.h`, `input/GrabInputIntentPolicy.h` |
| Grab stages: pull, catch, seat, hold, release | `hand/HandGrabPull.cpp`, `HandGrabCapture.cpp`, `HandGrabSeat.cpp`, `HandGrabCommit.cpp`, `HandHeldObject.cpp`, `HandGrabRelease.cpp` |
| Held object feel and force | `grab/GrabMotionController.h`, `grab/GrabAuthorityProxy.h`, `grab/GrabConstraint.h`, `hand/HandGrabDrive.cpp` |
| Finger posing | `grab/GrabFingerCapsuleSweep.h`, `grab/GrabPinchPocket.h`, `grab/GrabLocalGraspSolve.h` |
| Grab poses: built-in, saved, library | `grab/BuiltInGrabOffsets.h`, `grab/SavedGrabOffsetStore.cpp`, `grab/GrabPoseLibrary.h` |
| Surface grabs and climbing latches | `grab/GlobalSurfaceGrabPolicy.h`, `grab/SurfaceMeshGrabPolicy.h` |
| Touch grabs | `grab/TouchGrabRuntime.cpp`, `src/api/TouchGrabRegistry.h` |
| Hand collision | `hand/DynamicHandCollision.cpp`, `hand/HandBoneColliderSet.cpp`, `core/interaction/HandCollisionSuppression.cpp` |
| Body colliders | `body/BodyBoneColliderSet.cpp` |
| Collision layers and filtering | `collision/CollisionLayerPolicy.h`, `collision/CollisionSuppressionRegistry.h`, `native/havok/HavokPairCollisionFilter.h` |
| Contacts and impact sounds | `core/interaction/ContactEvents.cpp`, `native/havok/NativeImpactAudio.h` |
| Equip, draw, holster, drop | `core/interaction/EquippedWeaponFrame.cpp`, `core/interaction/EquippedWeaponDrop.cpp`, `weapon/equip/EquippedWeaponTransitionCoordinator.cpp`, `weapon/native/NativeHandEquipment.cpp` |
| Holding a gun: firing seat, support hand, aim | `weapon/grip/TwoHandedGripController.cpp`, `weapon/grip/WeaponGripSeat.cpp`, `weapon/handling/GunstockMode.h` |
| Authored grips | `weapon/authored/`, `animation/AuthoredWeaponGripCapture.cpp` |
| Weapon collision | `weapon/collision/WeaponCollision.h`, `weapon/collision/DynamicWeaponCollision.cpp` |
| The second weapon | `weapon/secondary/NativeSecondaryPresentation.cpp`, `weapon/equip/EquippedWeaponRoster.h` |
| Scopes | `weapon/scope/NativeScopeData.cpp`, `input/ManualScopeInputPolicy.h` |
| Recoil | `weapon/recoil/RecoilController.cpp` |
| Shoulder stash | `stash/ShoulderStashDetector.cpp`, `weapon/equip/EquippedWeaponShoulderCoordinator.h` |
| Eating, drinking, stimpaks | `consume/MouthConsumeDetector.cpp`, `consume/ImmersiveAid.cpp` |
| Grenades | `grenade/LooseGrenadeRuntime.cpp`, `core/interaction/ForceGrabAndGrenades.cpp` |
| Loose objects and ammo | `object/ObjectDetection.cpp`, `object/CountBasedAmmo.cpp` |
| Buttons and bindings | `input/ButtonLayoutPolicy.h`, `input/FunctionBindingPolicy.h`, `input/InputRemapRuntime.cpp` |
| Ammo counters | `visual/AmmoCounters.cpp`, `core/interaction/AmmoCounterFrame.cpp` |
| Indicators and debug drawing | `visual/WorldDrawClient.cpp`, `debug/`, `core/interaction/DebugOverlayPublish.cpp` |
| Native HUD placement | `native/player/` |
| Character controller | `native/player/CharacterControllerRuntime.cpp` |
| Native crashes in Havok or ragdolls | `native/safety/` |
| A setting | `src/config/SettingMetadata.h`, `src/RockConfigLoad.cpp`, `src/RockConfig.h`, then regenerate the examples |
| The setup wizard | `src/setup-wizard/SetupWizardPanel.cpp` (pages), `SetupWizard.cpp` (game host); try it with the preview |
| A provider API call | Its header, its `docs/api/` page, then [Provider API path](#provider-api-path) |
| Frame-time cost | `performance/PerformanceProfiler.cpp` |

## Build and test

Requirements: Windows x64, Visual Studio 2022 (v143), CMake 4.2+, PowerShell 7, vcpkg with `VCPKG_ROOT`, and checkouts of CommonLibF4VR, RPS UI Framework (`include/` and `support/`), RobCo PALM (`include/`), Dear ImGui at the revision RPS UI Framework's `docs/api/Framework.md` names, and RPS Framework built with its `custom-fast` preset. Set `RPSUI_INCLUDE_DIR`, `PALM_INCLUDE_DIR`, `ROCK_IMGUI_DIR` and `RPSFramework_DIR` when they are not at the defaults in `CMakeLists.txt`. C++23.

Copy `CMakeUserPresets.json.template` to `CMakeUserPresets.json` and set `COMMON_LIB_F4VR_PATH` and `COPY_PLUGIN_BASE_PATH` (the mod folder the build copies `ROCK.dll` into, under `F4SE/Plugins`). Then:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK
```

Tests are outside the default build. Run the ones a change affects:

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

It selects from the working tree (or the latest commit when clean) and widens the selection when unsure; `-PlanOnly`, `-BaseRef <ref>` and `-All` adjust it. Everything: `cmake --build build-fast --config Release --target build_tests`, then `ctest --test-dir build-fast -C Release --output-on-failure`.

The setup wizard preview, `ROCKSetupWizardPreview.exe`, runs the wizard's own panel through RPS UI's `LocalPanelHost` and ROCK's settings store over a sandbox folder (`Settings sandbox` beside the exe, or `--settings <folder>`), with stand-ins for the controllers, the pointer and SteamVR. It builds with ROCK (`ROCK_SETUP_WIZARD_PREVIEW`, off in the release preset) and needs only Windows' own DLLs; `ROCK_SETUP_WIZARD_PREVIEW_COPY_PATH` in `CMakeUserPresets.json` copies it to a folder to open it from. `ROCKSetupWizardPreview.exe --capture <folder>` writes every page to PNG instead of opening the window.

A wizard change goes in `SetupWizardPanel`, never in a preview-only copy; the preview only stands in for what the game supplies.

Tests link CommonLib's real `NiPoint.cpp` and `NiMatrix3.cpp` for engine math. `ROCKPublicHeaderTests` compiles each public header alone and checks the published ABI layouts.

## Runtime files

- `Data/F4SE/Plugins/ROCK.dll`; log `Documents/My Games/Fallout4VR/F4SE/ROCK.log`.
- `Documents/My Games/Fallout4VR/Mods_Config/ROCK/`: `ROCK.toml`, the optional `ROCK_Developer.toml`, `SavedGrabOffsets/`, `AuthoredWeaponGripCache/` and `SteamVR/`. Runtime code resolves the Documents known folder.
- SteamVR: at the player's choice in the setup wizard, ROCK sets Fallout 4 VR's binding for Index or Vive controllers through `IVRSettings`, only over SteamVR's default binding and never under OpenComposite. It never replaces a binding the player chose or edits SteamVR's files.
- Co-save `ROCK`, record `HNDS` version 1 (`weapon/equip/WeaponHandSave.cpp`): each equipped weapon's hand and each hand's melee reverse grip. A layout change needs a new record version.

## What ROCK does not own

- **NPC interactions**: SCISSORS, through the NativeInteraction interface, owns its NPC bodies, constraints, blade and limb policy. ROCK keeps the shared mechanisms (native allocation, callback lifetime, hand and weapon authority, grab execution, native safety) and has no NPC settings.
- **World drawing**: RPS UI Framework. ROCK draws through it (`visual/WorldDrawClient.cpp`, `visual/AmmoCounters.cpp`) and has no renderer.
- **Reload and manual-cycle animation**: PAPER.
- **Settings screens**: RobCo PALM, through the Configuration interface. Nothing else writes ROCK's settings files.

## Invariants

### Interaction

- A pull or forced arrival of a loose object seats from, per hand: the player's saved offset, a provider default pose, ROCK's built-in pose, the library pose, then the dynamic seat. Fixed poses (immersive aids, grenades) seat every grab and replace the pinch, except a second hand joining a held grenade.
- `BuiltInGrabOffsetData.cpp` and `GrabPoseLibraryData.cpp` are generated from the grab-offset authoring data; regenerate them, never edit them.
- A saved offset of an object whose bodies move against each other (ragdolls, tongs) is relative to the grabbed body and applies only to that body.
- A close grab holds the object where the hand found it only when a finger probe touches the visual mesh or the mesh crosses the thumb-index line; otherwise the object comes to the hand like a pull. A body (limb, corpse) never comes to the hand; the hold closes the gap.
- Once that contact exists a grab always places the object, ending in the palm, or for a pinch where thumb and finger found it. Fix a poor pose by improving the seat and fingers, never by rejecting the grab.
- The held point is always a mesh point or an authored weapon seat, never native collision's centre of mass.
- Two hands on one loose object aim it along the line between their grips, except under 6 game units apart on a non-weapon, where tremor exceeds the distance.
- A live NPC's limb keeps its own body: the hold is a reduced-force point and the limb may lag the hand.
- Per-frame limits scale with frame length, never with 90 Hz frame counts.
- Every controller-state read passes through the `[Controls]` button layout before ROCK sees it, so everything reads one logical button set; features never keep their own physical button tables. `[Bindings]` then presents a moved function as a tagged press of its home button.

### Provider API

- Public headers include only the standard library and other `include/ROCK/` headers.
- Every entry point validates the caller's `size` and `version` before reading. Records use fixed-width types.
- Every version refusal is logged with the calling DLL and stops the game at the main menu.

### Configuration

- `src/config/SettingMetadata.h` is the single source of every option (table, key, owning file, category, help); `RockConfigLoad.cpp` gives its type and default. Each option belongs to exactly one of `ROCK.toml` (players) and `ROCK_Developer.toml` (developer overrides only). Startup fails when catalog and loader disagree.
- After any catalog change, regenerate `data/config/*_example.toml` with `ROCKConfigurationStoreTests --write-examples data/config`; the test fails when they differ. ROCK never reads the examples.
- RPS Framework's settings store reads, organizes and writes the files (`src/config/RockSettings.cpp`). A file that does not parse is never rewritten, keeps its last values and refuses API writes until fixed. Both files reload while the game runs.
- Old `ROCK.ini` files migrate once, only when `ROCK.toml` is missing, through the frozen table `src/config/LegacyIniMigrationTable.inl` (never regenerated), and are deleted only after the TOML files read back.
