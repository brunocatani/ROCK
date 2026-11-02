# AGENTS.md — ROCK

ROCK is the FO4VR F4SE plugin for physics-based hand interaction: object grabbing, hand collision, equipped-weapon interaction and the provider APIs other mods consume. Suite ownership and the 1.0 provider API reset come from the ROCK suite workspace instructions.

## Identity and paths

- Remote: `origin` at `https://github.com/brunocatani/ROCK.git` (public, the active development source). A private legacy repository exists but receives no development or pushes unless the user asks.
- Build output: `build-fast/Release/ROCK.dll` and `ROCK.pdb`, auto-deployed to `F4SE\Plugins` under the mod folder set by `COPY_PLUGIN_BASE_PATH` in `CMakeUserPresets.json`.
- Config: `Documents\My Games\Fallout4VR\Mods_Config\ROCK\ROCK.ini` and optional `ROCK_Developer.ini`. `SavedGrabOffsets\` and `AuthoredWeaponGripCache\` live in the same folder.
- Verify the deployed DLL's VersionInfo and timestamp rather than trusting a recorded version.
- Save game: F4SE co-save unique ID `ROCK`, record `HNDS` (version 1): which hand holds each equipped weapon and each hand's one-handed melee reverse grip (`src/physics-interaction/weapon/WeaponHandSave.cpp`). A layout change needs a new record version.

## Build

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK
```

## Tests

When a change touches tested behaviour, run the affected-test selector before committing:

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

It selects from staged, unstaged and untracked files (or the latest commit on a clean tree), configures `custom-fast` (tests share `build-fast` but stay out of the plugin's default build), builds the selected tests in `Release`, runs CTest, and escalates to broader coverage when mapping is unsafe. Do not narrow its selection or bypass escalation. `-BaseRef <ref>` validates a committed range plus the worktree, `-PlanOnly` shows the selection, and `-All` is used only when requested or when a cross-repository change cannot be inferred from ROCK's diff.

Policy tests link CommonLib's real `NiPoint.cpp` and `NiMatrix3.cpp`; never re-implement engine math in test stubs.

## Architecture

- ROCK links `libraries_and_tools/CommonLibF4VR` directly and does not consume F4VR-CommonFramework.
- ROCK-owned logging, runtime helpers, menu tracking, resource helpers and controller input live under `src/rock_support/`.
- The OpenVR ABI used for raw controller input and haptics is vendored under `third_party/openvr/`; do not add a separate workspace dependency for it.
- Controller buttons: every IVRSystem controller-state read passes through the `[Controls]` button layout (`src/physics-interaction/input/ButtonLayoutPolicy.h`) before ROCK's capture, so ROCK, its consumers and the game all read one logical Quest-layout button set. Features read those logical bits; none keeps its own physical button table.
- `steamvr_bindings/` holds SteamVR legacy binding templates for non-Quest controllers. They ship only in release packages, never in a normal build or deployment.
- Skeleton and provider integration reference: RR's upstream `dependancies_and_dependant_mods/Fallout-4-VR-Body` (FRIK 0.79, API 2.3). ROCK's provider interfaces are negotiated independently of the FRIK API.
- Provider API major 2 is published from `include/ROCK/` (self-contained headers, compiled by ROCK itself), with one page per interface in `docs/api/` and standalone-compile and ABI layout tests in `tests/`. Until the major-2 headers move into `include/ROCK/`, ROCK 1.0 builds against RPS_SDK's `1.0` branch; add no contracts there.
- NPC behaviour belongs to SCISSORS_Redux. ROCK keeps only shared mechanisms: native allocation and quarantine, callback lifetime, hand and weapon authority, connected-body grab execution, native safety and grenade lifecycle. For contact-only blade damage, ROCK publishes the contact point and the optional NativeInteraction manifold normal and count; it never owns blade profiles, native melee damage hooks, actor-limb policy or their experiment flags. For 1.0, Redux creates and reads its own NPC bodies and constraints and reads Havok directly; ROCK stops proxying NPC body work (`Docs/ROCK/docs/ProviderApiV2Plan.md`).
- All world drawing, gameplay indicators and debug shapes moves to RPS UI Framework, and ROCK keeps no renderer (`Docs/ROCK/docs/DebugRenderingMigrationPlan.md`). ROCK keeps its own hand and weapon visualization master setting.

## Behaviour invariants

- A grab always places the object. Never answer a poor grab pose by rejecting the grab; improve the seat and the finger posing. A held object always ends in the palm, locked to the hand (`placeHeldBodiesOnHandTarget`): a grab is never marked held while the object is still away from the hand.
- Players on weak PCs never send logs, so their bugs are found from code. Per-frame limits (distance per frame, frame-time gates, sample ages, retention windows) scale with frame length (`frameMotionScale`, `ContactClock::freshnessWindowSeconds`), never with 90 Hz frame counts.

## Configuration contract

- Runtime authority belongs only to `ROCK.ini` (consumer) and `ROCK_Developer.ini` (developer overrides). Each supported key has exactly one owning file, following the split in `src/config/DeveloperSettingMetadata.h`, including advanced options without a Debug prefix.
- `data/config/ROCK_example.ini` documents every consumer option and `ROCK_Developer_example.ini` every developer option. They are human references only: never read, embedded, packaged, deployed, copied or used to create runtime files. Runtime INIs never ship, deploy or embed.
- Load an existing `ROCK.ini` unchanged: no merging defaults, appending keys, regenerating or replacing. Missing keys use compiled defaults without modifying the file.
- If `ROCK.ini` is missing, create it with every consumer default directly from the compiled defaults and loader catalog; no templates, resources or fallback paths.
- Never create `ROCK_Developer.ini` during loading. Create it only when a developer setting differs from its compiled default; persist only overrides, remove a key restored to default, and remove the file when no entries remain. A supplied developer file, including entries at defaults, loads unchanged, and editing one setting preserves unrelated entries.
- Both files hot-reload, including developer-file creation and removal. PALM reads the compiled catalog and routes writes through ROCK's configuration API; it never duplicates defaults or writes ROCK's INIs itself.
- A configuration addition, removal, default change, comment change or value change updates the active production file first (in place), then the owning example and compiled defaults.
- Configuration tests check that both examples have complete, disjoint key coverage agreeing with compiled defaults, that packaging rejects shipped or embedded INIs, and that first-run creation uses compiled defaults only.
- Mandatory production behaviour is never configurable through either INI: weapon collision enablement and generation mode (generated weapon collision uses only the optimized per-TriShape path); dynamic hand collision, interaction and haptics; body collider enablement; native character-controller filtering; the far-selection HMD cone; grab on the logical OpenVR grip bit (the `[Controls]` button layout chooses its physical source); native V.A.N.S. suppression; native right-stick Favorites suppression; and grab, surface, impact, shoulder-stash and mouth-consume haptic enablement. Existing ROCK APIs may control only behaviour they already own.
- NPC settings belong exclusively to SCISSORS_Redux; ROCK keeps no NPC keys, aliases, forwarding setters or old-file fallbacks.
