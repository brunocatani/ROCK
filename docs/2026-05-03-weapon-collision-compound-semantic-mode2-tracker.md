# Weapon Collision Compound Semantic Mode 2 Tracker

Created: 2026-05-03
Status: mode-2 policy/runtime aggregation, native static-compound body path, and provider/PAPER v3 evidence bridge implemented; in-game weapon validation pending

## Goal

Add a new ROCK generated weapon collision mode for PAPER-quality physical reloads. The new mode must aggregate authored weapon part naming conventions into one logical body per reload-relevant part, while keeping existing generation modes available for comparison and fallback.

## User Decisions Locked

- Keep old generation available:
  - `0 = LegacyTriShape`
  - `1 = current SemanticPartNode`
  - `2 = new CompoundSemanticPart`
- Make mode `2` the new packaged/default config once implemented.
- Aggregate by weapon semantic naming conventions such as `WeaponMagazine`, `WeaponBolt`, `WeaponExtra*`, `Stock`, `ChargingHandle`, `Slide`, `Pump`, and `Cylinder`.
- For MK18 quick/taped magazine variants, treat the whole taped magazine cluster as one physical reload object.
- Use runtime generation as the main path, with PAPER profile overrides for edge cases.

## Investigation Findings

Extracted mesh root provided by user:

`E:\fo4dev\fallout4vr_mods\FO4 Meshes`

Important local tooling found:

- NifSkope exists at `E:\fo4dev\NifSkope_2_0_2018-02-22-x64`.
- `nif.xml` was used read-only to confirm FO4 NIF block layout.
- No Ghidra operation was used for this investigation.
- Ghidra approval is still required before implementing/porting compound Havok constructor offsets.

MK18 findings:

- `Meshes\Weapons\MK18\MK18_Mag.nif`
  - `Mag -> WeaponMagazine -> WeaponMagazineHelper -> Mag:0`
  - The named `WeaponMagazineChild*` nodes are mostly animation/helper structure; the actual magazine triangles are under `Mag:0`.
- `Meshes\Weapons\MK18\MK18_MagQuick.nif`
  - Primary mag: `WeaponMagazine -> ... -> Mag:0`
  - Second taped mag: `WeaponExtra3 -> WeaponExtra3Helper -> MagSecond:0`
- `Meshes\Weapons\MK18\Attachments\MK18_Mag_P-Mag30_Quick.nif`
  - Same quick-mag structure.
  - Also has `MagBullet:0`, which must be excluded as cosmetic/visible ammo.
- `Meshes\Weapons\MK18\MK18_Stock.nif`
  - `Stock -> Stock:0`
  - `Stock -> StockTube:0`
  - These should form one stock body in mode 2.
- `Meshes\Weapons\MK18\MK18_ChargingHandle.nif`
  - `ChargingHandle -> WeaponExtra1 -> WeaponExtra1Helper -> ChargingHandle:0`
  - These should form one charging-handle/action body in mode 2.

Base-game `.44` findings:

- `Meshes\Weapons\44\44.nif`
  - Root `WEAPON`
  - `WeaponMagazine` contains:
    - `44MagnumCylinder:0`
    - `44MagnumEjector:0`
    - `44MagnumBullets:0`
    - `44MagnumBulletCase:0`
    - `44MagnumMoonClip:0`
    - `44 MagnumGate:0`
  - This is not a detachable magazine for PAPER.
  - Mode 2 must resolve this as a revolver cylinder/action compound and exclude bullet/case display geometry from permanent weapon collision.

## Current Code Facts

ROCK owns runtime generated weapon collision and provider evidence:

- `ROCK/src/physics-interaction/WeaponCollisionGroupingPolicy.h`
  - Modes now include `0 LegacyTriShape`, `1 SemanticPartNode`, `2 CompoundSemanticPart`.
  - Compiled default is now `2 CompoundSemanticPart`.
  - Mode 2 adds a context-aware compound classifier for MK18-style `Mag`, `MagSecond`, `WeaponExtra*`, stock tube, charging-handle helpers, and `.44` revolver magazine/cylinder cases.
- `ROCK/src/physics-interaction/WeaponCollision.cpp`
  - Mode 1 still builds `GeneratedHullSource` objects from authored semantic nodes.
  - Mode 2 now merges compatible compound part clouds before hull source creation.
  - Oversized mode-2 point clouds are split into child convex hulls, then published as one body/source through a native static-compound shape.
- `ROCK/src/physics-interaction/HavokCompoundShapeBuilder.*`
  - Added a ROCK-local native static-compound builder using FO4VR's verified cinfo, shape-instance setter, and static-compound constructor entry points.
- `ROCK/src/physics-interaction/WeaponPartClassifier.h`
  - Existing name classifier already recognizes magazine, bolt, charging handle, stock, cylinder, ammo display, etc.
- `ROCK/src/api/ROCKProviderApi.h`
  - ROCK-side provider header is now API version 3.
  - `ROCK_PROVIDER_MAX_WEAPON_BODIES` is still 8 for frame snapshots.
  - Legacy 128-byte evidence descriptors remain available.
  - V3 detail calls expose generated evidence bounds and body-id keyed local point copy for PAPER.

PAPER consumes provider evidence and owns reload behavior:

- `PAPER/src/reload/WeaponProfileEvidenceAggregator.h`
  - Already has profile selection fields for local bounds and local mesh points.
  - Reviewed/user-selected profile data is authoritative.
- `PAPER/src/api/ROCKProviderApi.h`
  - PAPER-side provider header is synced to ROCK provider API version 3.
  - PAPER now requires ROCK provider v3 during discovery.

HIGGS reference:

- HIGGS weapon collision clones one owned weapon collision shape and drives it as a coherent keyframed weapon package.
- HIGGS does not generate reload part meshes from visible firearm geometry.
- Useful principle for ROCK: collision ownership and visual evidence are separate. Mode 2 should generate better semantic evidence but still keep one coherent weapon drive root.

## Planned Architecture

Added `WeaponCollisionGroupingMode::CompoundSemanticPart = 2`.

Modes must behave as follows:

- Mode 0: preserve current legacy one-TriShape source behavior.
- Mode 1: preserve current semantic part node behavior.
- Mode 2: new compound semantic aggregator for reload-quality evidence.

Mode 2 should:

- Build one logical generated source per semantic part.
- Choose compound roots by authored naming conventions first.
- Aggregate child/helper geometry into the parent part when it is part of the same authored reload component.
- Exclude visible ammo/cosmetic display geometry.
- Resolve revolver `WeaponMagazine` as cylinder/action evidence instead of detachable magazine evidence.
- If a semantic part point cloud exceeds the convex point cap, split into child convex hulls but publish/create one body ID using a static compound shape.

Implemented so far:

- `data/config/ROCK.ini` packaged/default toggle is now `iWeaponCollisionGroupingMode = 2`.
- Invalid grouping mode sanitizes to mode 2.
- Mode names and mode-change rebuild checks include mode 2.
- Mode 2 classifies MK18 `Mag:0` and `MagSecond:0` as magazine bodies without treating `44Magnum*` as magazine aliases.
- Mode 2 treats `WeaponExtra*` as helper structure, not receiver geometry.
- Mode 2 excludes `MagBullet`, `.44` bullets/cases, rounds/shells, and moon clips from permanent generated weapon collision.
- Mode 2 keeps stock tube under stock and charging helper nodes under charging handle.
- Mode 2 lets `.44` cylinder evidence split out of `WeaponMagazine`, preventing a detachable-magazine profile from being generated for the revolver combo node.
- Mode 2 runtime traversal merges compatible semantic part clouds before hull creation, so same-role sibling evidence such as MK18 quick/taped magazines shares the same generated source intent.
- Provider v3 keeps the old descriptor path but adds:
  - `getWeaponEvidenceDetailCountV3`
  - `copyWeaponEvidenceDetailsV3`
  - `getWeaponEvidenceDetailPointCountV3`
  - `copyWeaponEvidenceDetailPointsV3`
- PAPER has a tested bridge helper that fetches v3 detail rows, copies each generated local point cloud by body id, and converts the data into `WeaponCollisionProfileEvidenceDescriptor`.

## Havok Compound Verification

CollisionForgeVR already contains a static compound shape builder:

- `CollisionForgeVR/src/havok/HavokCompoundShapeBuilder.h`
- `CollisionForgeVR/src/havok/HavokCompoundShapeBuilder.cpp`
- `CollisionForgeVR/src/havok/HavokOffsets.h`

Ghidra approval was granted by the user on 2026-05-03.

Verified FO4VR offsets/layouts:

- `0x1416E1CF0` / offset `0x16E1CF0`: cinfo constructor.
  - Writes instances pointer at `+0x00`, count at `+0x08`, capacity/flags at `+0x0C`, flags byte at `+0x10`, mass config at `+0x18`, output IDs at `+0x20`.
  - Verdict: confirmed.
- `0x1416E1780` / offset `0x16E1780`: shape-instance set shape.
  - Retains incoming shape, releases old shape at instance `+0x50`, writes new shape at `+0x50`, and updates flags at `+0x2C`.
  - Verdict: confirmed.
- `0x1416E1840` / offset `0x16E1840`: shape-instance set transform.
  - Copies transform columns/translation into the instance and updates transform flags.
  - Verdict: confirmed.
- `0x1416E1910` / offset `0x16E1910`: shape-instance set scale.
  - Writes scale vector at instance `+0x40`, uses scale mode, and updates scale flags.
  - Verdict: confirmed.
- `0x141E9C950` / offset `0x1E9C950`: native static-compound constructor body.
  - Ghidra labels it under `hknpShapeMassProperties`, but the decompile shows an inlined `hknpStaticCompoundShape::hknpStaticCompoundShape` path: it calls `hknpCompoundShape::hknpCompoundShape`, assigns the static-compound vtable, calls the static-compound build routine, and installs optional mass/material config.
  - Important correction: native callers pass four arguments: storage, cinfo, child count, and optional pointer/null. ROCK uses the four-argument signature. CollisionForgeVR's existing two-argument typedef is therefore incomplete for this address.
  - Verdict: identity confirmed, documented call signature corrected.

## Provider/PAPER Plan

Provider ABI was extended to version 3 while preserving old calls:

- Keep existing frame snapshot body ID array at 8 for compatibility.
- Add a separate evidence count/detail path for all generated weapon evidence.
- Expose:
  - body ID
  - part kind / reload role / socket role / action role / support role
  - interaction root and source root
  - source name
  - local bounds in game units
  - full point count
  - caller-buffer point copy for local mesh points
- Sync `ROCK/src/api/ROCKProviderApi.h` and `PAPER/src/api/ROCKProviderApi.h`.

PAPER now:

- Prefers provider v3 detail evidence through `copyCollisionProfileEvidenceFromProviderV3`.
- Imports local mesh points into PAPER collision profile evidence descriptors.
- Keep reviewed/user-selected profile data authoritative.
- Does not use collision descriptors to auto-fill reload roles; generated evidence remains authoring/support data until reviewed.

## Test Targets

ROCK tests:

- Implemented mode sanitizing tests:
  - `0 -> LegacyTriShape`
  - `1 -> SemanticPartNode`
  - `2 -> CompoundSemanticPart`
  - invalid -> `CompoundSemanticPart`
- Implemented mode name string test for `CompoundSemanticPart`.
- Implemented mode change detection between `1` and `2`.
- Implemented MK18 quick mag policy tests:
  - `WeaponMagazine + WeaponExtra3/MagSecond` aggregate to one MagazineBody compound.
  - `MagBullet` excluded.
- Implemented MK18 stock policy test:
  - `Stock:0 + StockTube:0` aggregate to one Stock compound.
- Implemented MK18 charging handle policy test:
  - `WeaponExtra1/Helper + ChargingHandle:0` aggregate to one ChargingHandle compound.
- Implemented `.44` policy tests:
  - `WeaponMagazine` resolves as Cylinder/action compound when cylinder evidence is present.
  - bullets/cases are excluded.
- Pending oversized semantic part test:
  - many local points split into child convex hulls but publish one logical body/source.

PAPER/provider tests:

- Implemented provider v3 static layout tests in `ROCKProviderBoundaryTests`.
- Implemented PAPER v3 detail conversion and provider-function-table fetch tests in `PAPERReloadTests`.
- Existing tests still verify collision descriptors do not auto-insert reload roles and user-selected legacy evidence remains review-visible.

## Resume Checklist

1. Done: update `WeaponCollisionGroupingPolicy.h` with mode 2 and policy naming.
2. Done: add pure policy helpers for mode-2 compound root/child decisions.
3. Done: write failing ROCK tests for mode 2 policy behavior.
4. Ask user for Ghidra approval before compound Havok implementation work.
5. Done: port verified compound builder into ROCK.
6. Done: update `WeaponCollision.cpp` so mode 2 builds one body per semantic part, using compound shape children when needed.
7. Done: add provider v3 evidence detail calls and sync PAPER API header.
8. Done: update PAPER provider bridge to consume v3 precise points as collision profile evidence.
9. Done: run focused ROCK and PAPER tests.
10. Validate in game with MK18 quick mag, MK18 stock, MK18 charging handle, and base-game `.44`.

## 2026-05-03 Implementation Log

Changed files:

- `ROCK/src/physics-interaction/WeaponCollisionGroupingPolicy.h`
- `ROCK/src/physics-interaction/WeaponCollision.cpp`
- `ROCK/src/physics-interaction/HavokCompoundShapeBuilder.h`
- `ROCK/src/physics-interaction/HavokCompoundShapeBuilder.cpp`
- `ROCK/src/physics-interaction/HavokOffsets.h`
- `ROCK/src/physics-interaction/PhysicsInteraction.h`
- `ROCK/src/api/ROCKProviderApi.h`
- `ROCK/src/api/ROCKProviderApi.cpp`
- `ROCK/tests/TransformConventionTests.cpp`
- `ROCK/tests/ProviderBoundaryTests.cpp`
- `ROCK/data/config/ROCK.ini`
- `PAPER/src/api/ROCKProviderApi.h`
- `PAPER/src/reload/PaperProviderBridge.h`
- `PAPER/tests/PAPERReloadTests.cpp`

Verification run:

- Red build was observed after tests referenced `CompoundSemanticPart` and compound grouping helpers before implementation.
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCKTransformConventionTests`
- `cd ROCK; .\build\Release\ROCKTransformConventionTests.exe`
- `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCKTransformConventionTests ROCK`
- `cd ROCK; .\build\Release\ROCKTransformConventionTests.exe`
- After Ghidra-approved compound builder implementation:
  - `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCKTransformConventionTests ROCK`
  - `cd ROCK; .\build\Release\ROCKTransformConventionTests.exe`
- Provider/PAPER v3 implementation:
  - Red build was observed after tests referenced missing v3 provider structs/functions and PAPER conversion helpers.
  - `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCKProviderBoundaryTests`
  - `cd ROCK; .\build\Release\ROCKProviderBoundaryTests.exe`
  - `cd PAPER; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target PAPERReloadTests`
  - `cd PAPER; .\build\Release\PAPERReloadTests.exe`
  - `cd ROCK; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target ROCK`
  - `cd PAPER; $env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release --target PAPER`

Build note:

- The Release `ROCK` target's existing post-build step copied `ROCK.dll`/`.pdb` to `D:/FO4/mods/ROCK/F4SE/Plugins/` and created a package archive.

## Implementation Guardrails

- Do not remove or change behavior of modes 0 and 1.
- Do not make PAPER own ROCK collision generation.
- Do not treat all `WeaponMagazine` nodes as detachable magazines.
- Do not include cosmetic ammo display geometry in permanent weapon collision.
- Do not rely on only the first 8 weapon body IDs for reload authoring evidence.
- Do not use Ghidra without explicit user approval.
