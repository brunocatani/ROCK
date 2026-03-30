# ROCK Cleanup Pass 02 - Config Authority Map - 2026-05-08

This pass maps the config boundary before changing more source because the active production INI is now the authority while older tests and packaged defaults still describe the packaged INI as if it were canonical. The chosen approach is to separate runtime behavior from fallback/package behavior: first prove what the active prod file contains, then identify source/test policy that still encodes stale assumptions, and only then decide what to remove.

## Scope

- No web.
- No Ghidra.
- No FO4 Mods MCP.
- No build run in this pass because another agent was also rebuilding ROCK.
- Files inspected:
  - `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`
  - `data/config/ROCK.ini`
  - `src/RockConfig.cpp`
  - `src/RockConfig.h`
  - `tests/ConfigDefaultParitySourceTests.ps1`

## Current Config Authority State

- Active prod INI now uses the renamed root-flattened skeleton debug keys:
  - `bDebugShowRootFlattenedFingerSkeletonMarkers = false`
  - `fDebugRootFlattenedFingerSkeletonMarkerSize = 1.4`
- The old prod keys were removed from the active prod file:
  - `bDebugShowFrikFingerSkeletonMarkers`
  - `fDebugFrikFingerSkeletonMarkerSize`
- Current key counts:
  - Active prod INI: 264 unique keys.
  - Packaged `data/config/ROCK.ini`: 265 unique keys.
  - Source direct `Get*Value`/`readVec3` detected keys: 246.
  - Source string-referenced known INI keys: 265.

The 246 direct-reader count is lower because some keys are read through helper lambdas such as `readClampedFloat()` and `readHexFilter()`. Those helper-read keys are still referenced in `src/RockConfig.cpp`.

## Prod vs Packaged Key Difference

Packaged-only key:

- `fCloseSelectionBehindPalmToleranceGameUnits`

This key is read by `RockConfig.cpp` and has compiled fallback `2.0f`. Because it is absent from prod, runtime currently uses the fallback value `2.0`. Adding it to prod with `2.0` would make the law explicit without changing behavior.

No active prod-only keys remain after the root-flattened skeleton rename.

## Remaining Old-Key Compatibility Reads

`src/RockConfig.cpp` still accepts the old skeleton debug key names as fallback aliases:

- `bDebugShowFrikFingerSkeletonMarkers`
- `fDebugFrikFingerSkeletonMarkerSize`

Those aliases are now unnecessary for the active prod INI. They are compatibility/migration reads, not live prod keys. Since the item-3 approval was "only update the ini", these code fallbacks were not removed in this pass.

## Value Differences

The active prod INI and packaged INI currently differ on 16 shared values:

- `bDebugDrawHandBoneColliders`: prod `false`, packaged `true`
- `bDebugDrawHandBoneContacts`: prod `false`, packaged `true`
- `bDebugDrawHandColliders`: prod `false`, packaged `true`
- `bDebugDrawSkeletonBoneAxes`: prod `false`, packaged `true`
- `bDebugDrawWeaponColliders`: prod `false`, packaged `true`
- `bDebugLogSkeletonBones`: prod `false`, packaged `true`
- `bDebugLogSkeletonBoneTruncation`: prod `false`, packaged `true`
- `bDebugShowHandAxes`: prod `false`, packaged `true`
- `bDebugShowPalmVectors`: prod `false`, packaged `true`
- `bDebugShowRootFlattenedFingerSkeletonMarkers`: prod `false`, packaged `true`
- `bDebugShowSkeletonBoneVisualizer`: prod `false`, packaged `true`
- `bDebugShowTargetColliders`: prod `false`, packaged `true`
- `bHandBoneCollidersRequireAllFingerBones`: prod `false`, packaged `true`
- `iDebugMaxConvexSupportVertices`: prod `8`, packaged `6`
- `iDebugWorldObjectOriginLogIntervalFrames`: prod `30`, packaged `120`
- `iWeaponCollisionGroupingMode`: prod `3`, packaged `1`

These are not runtime mismatches as long as the active prod INI exists and is loaded. They are fallback/package mismatches and should not be "fixed" by updating packaged config unless the user says the packaged file has been manually refreshed.

## Test Policy Conflict

`tests/ConfigDefaultParitySourceTests.ps1` passes right now, but its language and checks still treat packaged `data/config/ROCK.ini` as the value authority:

- It repeatedly says packaged `ROCK.ini` must match compiled defaults.
- It does not know that the active prod INI is the law.
- It cannot safely hard-require the local prod path in CI/release contexts without an opt-in.

This makes the test mechanically green but semantically stale. The likely cleanup is to keep fallback safety checks while renaming/reframing the test away from "packaged is authority." A better contract would be:

- Packaged INI and compiled defaults are fallback artifacts only.
- Source must continue to read every current prod key.
- Deleted knobs must stay deleted from source and packaged fallback.
- Optional local audit can compare against the active prod INI when the file exists.

## Questions For Next Pass

1. Should I add `fCloseSelectionBehindPalmToleranceGameUnits = 2.0` to the active prod INI in place so the prod file explicitly owns that key?
2. Should I remove the old `FrikFingerSkeleton` fallback reads from `src/RockConfig.cpp` now that the active prod INI has been renamed?
3. Should I rewrite `ConfigDefaultParitySourceTests.ps1` so it no longer treats packaged `data/config/ROCK.ini` as canonical value authority, while still protecting fallback/default safety?

## Approved Follow-Up Applied

User approved all three questions.

- Active prod INI now explicitly contains `fCloseSelectionBehindPalmToleranceGameUnits = 2.0`.
- `src/RockConfig.cpp` no longer accepts the removed skeleton debug aliases:
  - `bDebugShowFrikFingerSkeletonMarkers`
  - `fDebugFrikFingerSkeletonMarkerSize`
- `tests/ConfigDefaultParitySourceTests.ps1` now documents that prod INI is runtime authority while packaged INI and compiled defaults are fallback artifacts.
- The same test now optionally audits the local active prod INI when present, requiring every prod key to be represented in `RockConfig.cpp` and requiring the removed skeleton debug aliases to stay absent.

## Prod INI Recovery Note

During the in-place edit to add `fCloseSelectionBehindPalmToleranceGameUnits`, the initial PowerShell regex replacement timed out and left the active prod INI empty. The file was immediately recovered in place from `data/config/ROCK.ini` plus the mapped prod overrides from this pass. The recovered active prod INI has 265 keys and preserves the known prod-tuned differences:

- `iWeaponCollisionGroupingMode = 3`
- `bHandBoneCollidersRequireAllFingerBones = false`
- quiet debug overlays remain `false`
- `iDebugMaxConvexSupportVertices = 8`
- `iDebugWorldObjectOriginLogIntervalFrames = 30`
- `fCloseSelectionBehindPalmToleranceGameUnits = 2.0`

The old May 5 backup at `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini.bak_20260505_004837` was not used because it predates the current key set.
