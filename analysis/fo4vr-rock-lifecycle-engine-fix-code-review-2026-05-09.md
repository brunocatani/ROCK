# FO4VR ROCK Lifecycle Engine-Fix Code Review

Date: 2026-05-09

Scope:

- ROCK provider v7 lifecycle fields.
- `PhysicsLifecycleState`.
- `PhysicsInteraction` lifecycle/write gates.
- ROCKMain FRIK/F4SE generation wiring.
- SCISSORS v7 lifecycle consumer behavior.

Verification:

- `ctest --test-dir build -C Release --output-on-failure`
- Initial review result: 54/54 ROCK tests passed, but semantic/runtime gaps were found.
- Fix pass result:
  - ROCK Release build passed.
  - ROCK CTest passed 54/54.
  - SCISSORS Release build passed.
  - SCISSORS CTest passed 13/13.

## Findings

### 1. High - Provider snapshot cache can remain stale after provider loss

Status: Fixed.

Evidence:

- `src/ROCKMain.cpp:98-103` clears ROCK/provider instances before deleting `PhysicsInteraction`.
- `src/physics-interaction/core/PhysicsInteraction.cpp:1957` observes `Shutdown`, but no provider snapshot is dispatched after that observation.
- `src/api/ROCKProviderApi.cpp:142-157` returns `s_lastSnapshot` as long as `s_hasSnapshot` is true.
- `src/api/ROCKProviderApi.cpp:463-479` updates `s_lastSnapshot` only during `dispatchFrameCallbacks`; `setPhysicsInteractionInstance(nullptr)` does not clear or poison it.

Impact:

- Polling consumers can call `getFrameSnapshot` after ROCK destroys `PhysicsInteraction` and receive the last good frame, potentially with stale `providerReady`, world pointers, generations, and `PhysicsWriteAllowed`.
- Callback consumers may not receive a final blocked/shutdown frame because the provider pointer is cleared before deletion and shutdown does not dispatch.

Fix direction:

- Publish a final provider-lost/shutdown snapshot before clearing the provider instance, or add a provider API helper that atomically poisons `s_lastSnapshot` on provider loss.
- The final snapshot should set `providerReady=0`, `bhkWorld=0`, `hknpWorld=0`, clear `PhysicsWriteAllowed`, and use `ProviderLost` or `Shutdown`.

Resolution:

- `destroyPhysicsInteraction` now calls `noteProviderLifecycle`, shuts down the live instance with the final reason, dispatches one final provider frame, then clears the API/provider instance pointers.
- The final frame is produced while the `PhysicsInteraction` object is still alive, so polling and callback consumers receive a blocked provider-lost/skeleton-destroying snapshot instead of the previous good frame.

### 2. High - Skeleton generation changes do not force generated body rebuild before writes reopen

Status: Fixed.

Evidence:

- `src/ROCKMain.cpp:219-229` handles `kPowerArmorChanged` by bumping skeleton generation and calling `noteSkeletonLifecycle`, but it does not destroy/rebuild generated collision bodies.
- `src/physics-interaction/core/PhysicsInteraction.cpp:825-829` considers generated bodies valid if hand/body bodies merely exist.
- `src/physics-interaction/core/PhysicsLifecycleState.h:164-166` immediately restores `state.generatedBodiesValid` from `input.generatedBodiesValid` once the world and skeleton are present.

Impact:

- After power armor or any skeleton remap where existing bodies still exist, the lifecycle gate can reopen after the settle frames without proving those bodies were rebuilt for the new skeleton generation.
- This weakens the main Skyrim-HDT-style guarantee: skeleton transitions should invalidate generated physics ownership until the generated set is rebuilt against the new skeleton epoch.

Fix direction:

- Track the generation that produced the current generated bodies, for example `generatedBodiesSkeletonGeneration` and `generatedBodiesWorldGeneration`.
- `generatedBodySetIsValid()` should require body existence plus generation match.
- Power armor/skeleton transition should either destroy/recreate bodies or mark a rebuild pending and keep `PhysicsWriteAllowed` closed until rebuild completes.
- Add a test where skeleton generation changes while bodies still exist; writes must stay closed until an explicit rebuild marks the new generation valid.

Resolution:

- `PhysicsLifecycleState` now tracks generated-body world/skeleton/provider generations.
- `PhysicsInteraction` stamps generated bodies when rebuilt and invalidates those stamps on skeleton/provider/world loss.
- The update loop performs a controlled generated-body rebuild before opening `PhysicsWriteAllowed`.
- The lifecycle unit test now covers stale generated-body generations after world and skeleton changes.

### 3. Medium - Provider generation tracking is only partially wired

Status: Fixed.

Evidence:

- `src/ROCKMain.cpp:244-246` bumps `s_providerGeneration` on `kGameLoaded`.
- `src/ROCKMain.cpp:289-291` bumps it on `kPostLoadGame`/`kNewGame`.
- `PhysicsInteraction::noteProviderLifecycle` exists, but `rg noteProviderLifecycle` shows no production call sites.

Impact:

- A live `PhysicsInteraction` does not learn about provider generation changes unless it is destroyed and recreated.
- If any provider/game-load transition occurs while an instance is still alive, provider generation exposed through snapshots can lag the global generation and writes may reopen based only on normal settle policy.

Fix direction:

- Call `noteProviderLifecycle` on live instances before teardown and on any provider/game-load event that can occur while the instance exists.
- Pair this with the final blocked snapshot from finding 1.

Resolution:

- `kGameLoaded`, `kPostLoadGame`, and `kNewGame` now notify a live `PhysicsInteraction` of provider generation changes.
- `destroyPhysicsInteraction` also publishes a provider-lost final frame before clearing the instance.

### 4. Medium - `PhysicsWriteAllowed` is not currently the first gate before all Havok mutation paths

Status: Fixed.

Evidence:

- `src/physics-interaction/core/PhysicsInteraction.cpp:1368-1391` performs suppression restore, hand collision update, body collision update, and weapon collision update before the lifecycle gate check.
- `src/physics-interaction/core/PhysicsInteraction.cpp:1394-1408` observes lifecycle and checks `physicsWritesAllowedForWorld` only after those phases.

Impact:

- The new gate blocks native step flushing and later update phases, but it is not a strict "no Havok mutation before safe" gate.
- During a transition, some collision/body/weapon paths can still update, create, destroy, or queue generated-body targets before the gate closes the frame.

Fix direction:

- Move an early lifecycle observation/gate before mutation phases.
- Split allowed behavior explicitly:
  - visual/provider-only publish while blocked
  - controlled rebuild phase while blocked
  - physics drive/write phase only after `PhysicsWriteAllowed`
- If the intended meaning is narrower, rename/document the flag as a physics-step-drive gate rather than a full physics-write gate.

Resolution:

- The update loop now observes lifecycle, performs only the controlled generated-body rebuild if needed, then checks `PhysicsWriteAllowed` before collision-layer drift repair, hand/body/weapon updates, suppression writes, generated-body drive registration, grab/contact logic, and provider callbacks.

### 5. Low - Menu-blocked snapshots can expose stale `bhkWorld` if world lookup fails

Status: Fixed.

Evidence:

- `src/physics-interaction/core/PhysicsInteraction.cpp:1211-1216` updates `_cachedBhkWorld` only when `snapshotBhk` is non-null.
- `src/physics-interaction/core/PhysicsInteractionProvider.inl:11-12` publishes cached world pointers directly.

Impact:

- A menu-blocked frame with no current `bhkWorld` can publish the previous `bhkWorld` while `hknpWorld` is null and lifecycle flags say the world is unavailable.
- Consumers that still inspect raw pointers may see inconsistent state.

Fix direction:

- Make `observeLifecycleFrame` own both cached pointers, or set `_cachedBhkWorld = snapshotBhk` unconditionally before dispatching blocked snapshots.

Resolution:

- `observeLifecycleFrame` now owns both cached world pointers and updates `_cachedBhkWorld` and `_cachedHknpWorld` together for normal, blocked, unavailable, and shutdown observations.

## Test Gaps

- Remaining: no direct unit test exercises the provider's cached final frame after provider loss because the provider cache is internal to the DLL boundary.
- Covered now: stale generated-body generations after world/skeleton changes.
- Covered by source/build path now: `noteProviderLifecycle` has production call sites in `ROCKMain.cpp`.
- Remaining: lifecycle gate ordering inside `PhysicsInteraction::update` is still primarily protected by source-boundary tests and review rather than a runtime harness.

## Overall Verdict

The original implementation compiled and passed tests but had lifecycle semantic gaps. The fix pass resolved the identified gaps: provider loss is observable, generated-body validity is bound to lifecycle epochs, provider generation notifications are wired, the write gate now precedes mutation phases, and blocked snapshots no longer retain stale `bhkWorld` when observation fails.
