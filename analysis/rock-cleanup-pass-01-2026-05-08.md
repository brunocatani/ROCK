# ROCK Cleanup Pass 01 - Map And Questions

Why this pass is being done this way: ROCK is late enough in development that cleanup cannot mean deleting anything that merely looks old. The safer approach is to first map runtime ownership, config parity, HIGGS reference behavior, and ABI boundaries, then remove or rewrite only the pieces that are provably stale, conflicting, or detached from production behavior. This pass is read-only apart from this note file.

## Ground Rules Observed

- No web was used.
- No Ghidra operations were run. Ghidra is still required before changing native offsets, struct layout assumptions, constraint ABI code, or hknp filter/layer assumptions.
- The active prod INI was read from `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`.
- The active prod INI was not modified or replaced.
- The ROCK worktree was already dirty. Existing edits were treated as user/workspace state.

## Current Worktree Shape

`git status` shows the ROCK repo on `main`, tracking `origin/main`, with a large dirty set:

- Modified: `.gitignore`, `AGENTS.md`, `CMakeLists.txt`, `README.md`, `data/config/ROCK.ini`, `src/RockConfig.*`, `src/api/ROCKApi.cpp`, multiple hand/core/native/weapon/debug files, and tests.
- Deleted: several docs under `docs/` dated 2026-05-06 and 2026-05-07.
- Added: `src/physics-interaction/debug/DebugConvexHullMesh.h`, `src/physics-interaction/debug/DebugOverlayLineBatch.h`, `tests/DebugOverlayPolicyTests.cpp`.

The dirty diff is large: about 2075 insertions and 1548 deletions before this note.

## Prod INI Vs Packaged INI

Parser coverage:

- Prod INI has 264 unique keys.
- Packaged `data/config/ROCK.ini` has 265 unique keys.
- `RockConfig.cpp` reads 267 keys when helper readers are included.
- No prod keys are currently unread by the parser.

Keys present only in prod:

- `bDebugShowFrikFingerSkeletonMarkers`
- `fDebugFrikFingerSkeletonMarkerSize`

Keys present only in packaged INI:

- `bDebugShowRootFlattenedFingerSkeletonMarkers`
- `fCloseSelectionBehindPalmToleranceGameUnits`
- `fDebugRootFlattenedFingerSkeletonMarkerSize`

The skeleton debug rename is backward-compatible in code: `RockConfig.cpp` reads the new root-flattened key and falls back to the older FRIK-named key. Prod still uses the old names.

Value differences where prod and packaged INI both have the key:

- `iWeaponCollisionGroupingMode`: prod `3`, packaged `1`.
- `bHandBoneCollidersRequireAllFingerBones`: prod `false`, packaged `true`.
- `iDebugMaxConvexSupportVertices`: prod `8`, packaged `6`.
- `iDebugWorldObjectOriginLogIntervalFrames`: prod `30`, packaged `120`.
- Debug overlay booleans are much quieter in prod: target colliders, hand axes, palm vectors, hand colliders, hand bone colliders, hand bone contacts, weapon colliders, skeleton axes/logging, and skeleton visualizer are false in prod but true in packaged.

Important packaging behavior:

- `ROCK.ini` is embedded as a DLL resource from `data/config/ROCK.ini`.
- Runtime config path resolves to Documents: `My Games\Fallout4VR\ROCK_Config\ROCK.ini`.
- `createFileFromResourceIfNotExists` only creates the INI if it does not already exist; it returns immediately if the file exists. This matches the production rule to update prod INIs in place and never replace them.
- Release packaging copies DLL/PDB only. There is no external `ROCK.ini` copied into `F4SE/Plugins`.

## Module Map

Largest source areas by size:

- `hand`: 17 files, about 515 KB. Biggest file: `HandGrab.cpp`.
- `weapon`: 12 files, about 432 KB. Biggest file: `WeaponCollision.cpp`.
- `grab`: 20 files, about 321 KB.
- `core`: 10 files, about 301 KB. Biggest file: `PhysicsInteraction.cpp`.
- `native`: 28 files, about 196 KB.
- `debug`: 11 files, about 184 KB. Biggest file: `DebugBodyOverlay.cpp`.
- `contact`, `object`, `input`, `collision`, and `actor` are smaller and more policy-focused.

High-risk cleanup files:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/weapon/WeaponCollision.cpp`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/physics-interaction/debug/DebugBodyOverlay.cpp`
- `src/RockConfig.cpp`
- `src/api/ROCKApi.cpp`

## HIGGS Baseline Checked

Local HIGGS references checked:

- `analysis/HIGGS_SOFT_COLLIDER_CHECK.md`
- `analysis/subsystems/05_configuration.md`
- `analysis/subsystems/08_main_entry_and_update.md`
- `analysis/subsystems/12_hand_class.md`
- `src/hand.cpp` around `ShouldUsePhysicsBasedGrab`, physics-held update, constraint update, and input leeway/force handling.

Relevant baseline:

- HIGGS hand and weapon bodies are real keyframed Havok bodies on HIGGS layer 56.
- ROCK's generated hand collider layer is 43 per workspace notes.
- The production HIGGS install should be treated as dynamic-grab behavior because `ForcePhysicsGrab=1`; old keyframed object grab should not be reintroduced as a ROCK production path.
- HIGGS dynamic held-object behavior uses a hand body plus motor constraint, collision listeners, hand visual adjustment toward the object, max hand-distance release, tau lerping, mass force caps, and inertia normalization.
- ROCK currently keeps single-hand loose grabs on FO4VR native mouse spring, and uses HIGGS-style constraints for shared/two-hand loose-object ownership. That is intentionally FO4VR-specific, not a HIGGS feature downgrade.

## Cleanup Candidates From Pass 01

1. Make prod INI the source of truth for packaged defaults, compiled defaults, and `ConfigDefaultParitySourceTests.ps1`.

This is the cleanest first cleanup because it removes configuration drift without touching runtime behavior. It should update `data/config/ROCK.ini`, `RockConfig.h`, `RockConfig.cpp`, and parity tests together.

2. Resolve skeleton debug key naming.

Prod still has `bDebugShowFrikFingerSkeletonMarkers` and `fDebugFrikFingerSkeletonMarkerSize`; packaged uses root-flattened names. Code reads both. Options are: keep compatibility and leave prod old keys, or update prod INI in place to the new names while preserving parser fallback.

3. Decide whether weapon grouping mode 3 is the real production default.

Packaged comments say mode 1 is production default, but prod uses mode 3. If prod mode 3 is the tested working path, the comment/default/test should be corrected.

4. Treat reload detachment cleanup as intentional, not stale.

`ReloadDetachmentSourceTests.ps1` requires ROCK to keep reload-owned code out and also requires CMake deploy cleanup for stale `ROCK.esp`, `Scripts/ROCK`, and `Meshes/ROCK`. Those CMake cleanup lines look old at first glance, but the test makes them intentional until deploy history says otherwise.

5. Review `ROCKApi.cpp` legacy v3 reload stub slots separately.

Public `ROCKApi.h` no longer exposes reload functions, but `ROCKApi.cpp` still returns a compatibility table with detached reload stub slots. This may be intentional ABI padding for older consumers. Removing it is ABI cleanup, not ordinary dead-code cleanup.

6. README/status is stale.

`README.md` still says early development and links to external pages. The status no longer matches the current final-step cleanup phase. The external links are not a runtime problem, but the status text should be updated in a later documentation pass.

7. Existing docs deletion needs a decision.

Several cleanup/review tracker docs are deleted in the current worktree. This new pass file starts a replacement cleanup map, but the deleted docs may contain findings worth preserving before final deletion.

## Compatibility And Legacy Inventory

Things that look old but are probably intentional boundaries:

- `ROCKApiCompatV3` in `ROCKApi.cpp`: preserves a larger legacy table shape with reload stubs even though `ROCKApi.h` no longer exposes those functions.
- `ReloadDetachmentSourceTests.ps1`: protects the split where PAPER owns reload and ROCK owns generic hand/weapon/contact/provider evidence.
- `GrabNodeNamePolicy.h`: rejects old `HIGGS:` grab node names and falls back to `ROCK:GrabR/L`. This is asset-contract cleanup, not stale behavior.
- `ContactActivityTracker.h`: accepts `0x7FFFFFFF` as a legacy invalid body id. That is a compatibility guard around historical invalid-id representations.
- `GrabConstraint.cpp`: has a `legacyState` restoration fallback for saved inertia state. It appears to preserve older `SavedObjectState` shape while new per-motion state exists.

Things that are still exposed as selectable legacy modes:

- `iGrabContactQualityMode = 0` maps to `LegacyPermissive`. Prod uses `1` (`HybridEvidence`), but the permissive path still exists and still has tests.
- `iWeaponCollisionGroupingMode = 0` maps to `LegacyTriShape`.
- `iWeaponCollisionGroupingMode = 3` maps to `LegacyTriShapeSupportFit`; prod currently uses this despite code comments calling mode 1 the production default.

Source-boundary tests already reject many previously removed paths, including:

- Dead live hknp palm-anchor readback config.
- Legacy visual-authority/surface-frame/opposition-frame/pinch grab authority.
- Native mouse-spring keyframing of held objects.
- Direct unguarded body activation/filter writes.
- Generated collider direct `CreateBody`/manual back-pointer paths.
- Weapon visual-refresh retry state machine.
- Reload-owned code/assets in ROCK.
- FRIK API hand transforms as interaction-frame authority.

## Questions For Next Pass

User answers from 2026-05-08:

- Packaged/default config is not authoritative. The prod INI is the law unless the user manually updates packaged config.
- `iWeaponCollisionGroupingMode = 3` is still being tested. Do not lock packaged/defaults to mode 3 yet.
- User-selectable legacy modes must remain exposed for private/testing builds. They should only be cleaned up when the user decides the mod is ready for public release.
- Skeleton debug key rename approved for active prod INI only. The prod INI was updated in place from `FrikFingerSkeleton` key names to `RootFlattenedFingerSkeleton` key names; values were preserved.
- Legacy `ROCKApi.cpp` v3 reload stub slots were approved for removal. `ROCKApiCompatV3` and the detached reload stub functions were removed, and reload detachment source tests now reject those tokens in `ROCKApi.cpp`.

1. Should pass 02 make `data/config/ROCK.ini`, `RockConfig` compiled defaults, and `ConfigDefaultParitySourceTests.ps1` match the active prod INI exactly, except where we deliberately rename keys?

2. For the skeleton debug rename, do you want the active prod INI updated in place from `FrikFingerSkeleton` names to `RootFlattenedFingerSkeleton` names, while keeping parser fallback for old installs?

3. Is `iWeaponCollisionGroupingMode = 3` the current tested-good production mode, or was prod temporarily set to 3 for debugging AMCAR/support-fit behavior?

4. Should the legacy v3 reload stub slots in `ROCKApi.cpp` stay for ABI compatibility until PAPER and any external consumers are checked, or should they be removed once we verify no one calls the old table shape?

5. Should user-selectable legacy modes stay visible in the INI (`iGrabContactQualityMode=0`, `iWeaponCollisionGroupingMode=0`, and possibly mode `3` if not production), or should cleanup remove/rename modes that are no longer supported production paths?

6. When cleanup reaches native Havok offsets, custom constraint data, filter bits, or hard ABI assumptions, do I have approval to use Ghidra MCP for those specific verifications after I state the exact target?
