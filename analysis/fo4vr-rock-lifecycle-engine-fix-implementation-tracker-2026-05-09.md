# FO4VR ROCK Lifecycle Engine-Fix Implementation Tracker

Date: 2026-05-09

Purpose: persistent tracker for implementing Skyrim-HDT-style lifecycle hardening in ROCK for FO4VR. This file is the resume point if context compacts.

Reference map:

- `ROCK/analysis/skyrim-hdt-smp-engine-fix-map-2026-05-09.md`

## User authorization

- Implement the FO4VR improvements in ROCK.
- Ghidra verification is approved for any native hook or reverse-engineered boundary.
- Track work in markdown to avoid memory loss.

## Constraints

- Do not port Skyrim offsets or byte patches.
- Do not assume CommonLibF4VR layout correctness when native fields matter.
- Do not add new native hooks without Ghidra verification.
- Keep SCISSORS downstream of ROCK provider state; do not add a second frame authority.
- Preserve existing dirty worktree changes. Do not revert unrelated edits.

## Implementation goal

Bring the transferable part of the Skyrim HDT-SMP engine fix into ROCK:

- explicit lifecycle state
- world and skeleton generations
- provider-exposed lifecycle flags/reasons
- write gates for physics/native mutation
- deterministic generated-object cleanup on world/skeleton/provider transitions
- transition settle/discontinuity policy where ROCK drives generated bodies
- tests proving stale generations cannot drive or leak state

## Current status

| Item | Status | Notes |
|---|---|---|
| Tracker created | Done | This file. |
| Current ROCK lifecycle mapped | Done | Provider snapshot, `PhysicsInteraction::init/update/shutdown`, FRIK lifecycle, generated body drive, and SCISSORS consumer path inspected. |
| Provider ABI plan | Done | Provider bumped to v7. `RockProviderFrameSnapshot` grew from 256 to 272 bytes. `getFrameSnapshot` accepts old v6-sized callers and copies only requested bytes. |
| Internal lifecycle state | Done | Added `physics-interaction/core/PhysicsLifecycleState.h` pure policy. |
| World generation tracking | Done | ROCK tracks world generation, bhk/hknp availability, transition settling, and lifecycle flags. |
| Skeleton generation tracking | Done | ROCKMain bumps skeleton generation on FRIK ready/destroying and power armor changes. |
| Provider generation tracking | Done | ROCKMain bumps provider generation on game load/new game transitions. |
| Write gates | Done | Native physics-step drive callbacks require `PhysicsWriteAllowed` and matching hknp world. Blocked/menu/world-unavailable frames publish provider snapshots. |
| Transition settle | Done | ROCK requires stable generated-body frames before opening `PhysicsWriteAllowed`. |
| Immediate invalidation | Done | Skeleton/provider lifecycle notifications close physics writes immediately, before the next world observation frame. |
| Review fix pass | Done | Fixed stale provider-loss snapshots, generated-body epoch validation, provider lifecycle call sites, earlier write gate ordering, and blocked-frame cached world consistency. |
| SCISSORS consumer update | Done | SCISSORS uses v7 lifecycle fields when present; older snapshots remain accepted. |
| Discontinuity detection | Partial | Generated drive already has teleport-distance handling; no new root/room discontinuity detector added yet. |
| Tests | Done | Added `ROCKPhysicsLifecycleStateTests`; updated provider ABI assertions. |
| Ghidra wave | Deferred | Only for new hook candidates beyond lifecycle/provider work. |

## Proposed code areas

- `ROCK/src/api/ROCKProviderApi.h`
- `ROCK/src/api/ROCKProviderApi.cpp`
- `ROCK/src/ROCKMain.cpp`
- `ROCK/src/physics-interaction/core/PhysicsFrameContext.h`
- `ROCK/src/physics-interaction/core/PhysicsInteraction.h`
- `ROCK/src/physics-interaction/core/PhysicsInteraction.cpp`
- `ROCK/src/physics-interaction/core/PhysicsInteractionFrame.inl`
- `ROCK/src/physics-interaction/core/PhysicsInteractionProvider.inl`
- `ROCK/src/physics-interaction/native/GeneratedKeyframedBodyDrive.*`
- tests under `ROCK/tests/`

## Lifecycle fields to add

Candidate provider-visible fields:

- `worldGeneration`
- `skeletonGeneration`
- `lifecycleFlags`
- `lastLifecycleReason`
- optional `stableFrameCount`

Candidate lifecycle flags:

- `WorldAvailable`
- `SkeletonReady`
- `ProviderReady`
- `MenuBlocking`
- `ConfigBlocking`
- `LoadingOrWorldTransition`
- `GeneratedBodiesValid`
- `PhysicsWriteAllowed`
- `VisualWriteAllowed`

Candidate lifecycle reasons:

- `None`
- `GameLoaded`
- `SkeletonReady`
- `SkeletonDestroying`
- `PowerArmorChanged`
- `WorldAvailable`
- `WorldChanged`
- `WorldUnavailable`
- `ProviderReady`
- `ProviderLost`
- `MenuBlocked`
- `ConfigBlocked`
- `GeneratedBodiesRebuilt`
- `GeneratedBodiesInvalidated`
- `TransitionSettled`
- `Shutdown`

## First implementation pass

1. Done: inspected existing provider snapshot size/version pattern.
2. Done: added an internal lifecycle state type.
3. Done: wired generation increments into existing world/skeleton/provider transition points.
4. Done: exposed lifecycle state through provider with ABI-safe struct growth.
5. Done: gated generated/native physics-step drive against lifecycle state.
6. Done: added tests.

## Files changed in this pass

ROCK:

- `src/api/ROCKProviderApi.h`
- `src/api/ROCKProviderApi.cpp`
- `src/ROCKMain.cpp`
- `src/physics-interaction/core/PhysicsLifecycleState.h`
- `src/physics-interaction/core/PhysicsInteraction.h`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/physics-interaction/core/PhysicsInteractionProvider.inl`
- `tests/ProviderBoundaryTests.cpp`
- `tests/PhysicsLifecycleStateTests.cpp`
- `CMakeLists.txt`

SCISSORS:

- `src/scissors/ScissorsRuntime.h`
- `src/scissors/ScissorsRuntime.cpp`

## Verification

Commands run from `E:\fo4dev\PROJECT_ROCK_V2\ROCK`:

- `cmake --preset default -DBUILD_ROCK_TESTS=ON`
- `cmake --build build --config Release --target ROCKProviderBoundaryTests ROCKPhysicsLifecycleStateTests`
- `build\Release\ROCKProviderBoundaryTests.exe`
- `build\Release\ROCKPhysicsLifecycleStateTests.exe`
- `cmake --build build --config Release --target ROCK`
- `ctest --test-dir build -C Release -R "ROCK(ProviderBoundary|PhysicsLifecycleState)Tests" --output-on-failure`
- Post-final safety patch rerun: `cmake --build build --config Release --target ROCKProviderBoundaryTests ROCKPhysicsLifecycleStateTests ROCK`
- Post-final safety patch rerun: `ctest --test-dir build -C Release -R "ROCK(ProviderBoundary|PhysicsLifecycleState)Tests" --output-on-failure`
- Review-fix rerun: `cmake --build build --config Release --target ROCKProviderBoundaryTests ROCKPhysicsLifecycleStateTests`
- Review-fix rerun: `ctest --test-dir build -C Release -R "ROCK(ProviderBoundary|PhysicsLifecycleState)Tests" --output-on-failure`
- Review-fix rerun: `cmake --build build --config Release --target ROCK`
- Review-fix rerun: `ctest --test-dir build -C Release --output-on-failure`

Commands run from `E:\fo4dev\PROJECT_ROCK_V2\SCISSORS`:

- `cmake --preset default`
- `cmake --build build --config Release --target SCISSORS`
- Post-final consumer smoke rerun: `cmake --build build --config Release --target SCISSORS`
- Review-fix rerun: `cmake --build build --config Release --target SCISSORS`
- Review-fix rerun: `ctest --test-dir build -C Release --output-on-failure`

Results:

- ROCK Release build passed.
- SCISSORS Release build passed.
- `ROCKProviderBoundaryTests` passed.
- `ROCKPhysicsLifecycleStateTests` passed.
- Post-final safety patch verification passed:
  - ROCK provider/lifecycle test targets rebuilt.
  - ROCK Release DLL rebuilt and copied to `D:/FO4/mods/ROCK/F4SE/Plugins/`.
  - CTest passed 2/2 targeted tests.
  - SCISSORS Release DLL rebuilt as provider-consumer smoke check.
- Review-fix verification passed:
  - ROCK focused lifecycle/provider CTest passed 2/2.
  - ROCK full CTest passed 54/54.
  - ROCK Release DLL rebuilt and copied to `D:/FO4/mods/ROCK/F4SE/Plugins/`.
  - SCISSORS Release DLL rebuilt.
  - SCISSORS full CTest passed 13/13.

## Current implemented behavior

- Provider API version is now 7.
- Provider frame snapshots expose:
  - `lifecycleFlags`
  - `lastLifecycleReason`
  - `worldGeneration`
  - `skeletonGeneration`
  - `providerGeneration`
  - `stableFrameCount`
- `apiGetFrameSnapshot` remains compatible with old v6-sized callers.
- ROCK publishes blocked snapshots for menu/world-unavailable lifecycle frames.
- Skeleton and provider lifecycle notifications immediately clear `PhysicsWriteAllowed`; skeleton invalidation also clears `GeneratedBodiesValid`, marks a world transition, and drops the lifecycle hknp pointer.
- ROCK publishes a final blocked provider snapshot before clearing the provider instance during destroy.
- Generated-body validity now requires body existence plus matching world/skeleton/provider generations.
- Provider generation changes are forwarded to live `PhysicsInteraction` instances on game/load lifecycle events.
- The frame update loop observes/rebuilds lifecycle state and checks `PhysicsWriteAllowed` before normal Havok mutation phases.
- ROCK native step callbacks refuse to drive generated bodies unless:
  - hknp world matches the current lifecycle world
  - provider is initialized
  - FRIK skeleton is ready
  - generated bodies are valid
  - menu/config are not blocking
  - transition settle is complete
- SCISSORS resets/waits when ROCK v7 reports `PhysicsWriteAllowed` closed.

## Ghidra verification queue

No new hook candidates are part of the first pass. If later needed, use the blind verification protocol before implementation:

1. FO4VR actor/equipment 3D attach/detach candidate.
2. FO4VR face/headpart skinning candidate.
3. Node/body owner pointer path back to `TESObjectREFR`.

Expected Ghidra binary for these waves:

- `Fallout4VR.exe` / FO4VR runtime `1.2.72`

Preflight:

- Ghidra MCP responded with FO4VR-shaped segments at base `0x140000000`.
- Disassembly around `0x140D8405E` resolved inside function `0x140D83DE0`, matching ROCK's current main-loop hook-site region.
- Final sanity check reconfirmed function `FUN_140d83de0` at `0x140D83DE0` with body `0x140D83DE0-0x140D846A7`.
- No new hook candidate was implemented in this pass, so no blind hook-verification wave has run yet.

## Resume notes

Start from this file, then inspect the listed code areas. The first code milestone should compile without requiring Ghidra because it only formalizes state already observed by ROCK/FRIK. Use Ghidra only when adding or validating new native hook sites.
