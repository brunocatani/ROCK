# Skyrim HDT-SMP Engine Fix Map for FO4VR Reference

Date: 2026-05-09

Scope: read-only mapping of `E:\fo4dev\skirymvr_mods\source_codes\hdtSMP64` against the current `E:\fo4dev\PROJECT_ROCK_V2` FO4VR stack. No source code changes are proposed or applied here.

## Short version

The Skyrim HDT-SMP "engine fix" is not one patch. It is a small native lifecycle framework around Skyrim's missing physics and scene-graph events.

It does four important things:

1. It hooks engine moments that SKSE did not expose: game frame, shutdown, armor attach, and FaceGen head geometry skinning.
2. It turns those hooks into internal events so the physics layer does not depend directly on every detour.
3. It creates, suspends, resets, re-adds, and destroys Bullet physics systems at scene-graph lifecycle boundaries instead of only at plugin load.
4. It patches specific FaceGen bone-limit/skinning behavior so extra HDT bones can be merged into armor/head geometry without the engine dropping or corrupting them.

The FO4VR equivalent is not to port Skyrim offsets. The transferable idea is to keep a verified lifecycle layer around FRIK/ROCK/SCISSORS: frame, physics-step, menu/loading, skeleton ready/destroying, world pointer generation, generated body validity, and write-allowed gates. PROJECT_ROCK_V2 already has much of this. The main value from the Skyrim map is to identify the remaining lifecycle boundaries worth making explicit and observable.

## Source files inspected

Skyrim HDT-SMP reference:

- `hdtSMP64/Offsets.h`
- `hdtSMP64/Hooks.cpp`, `Hooks.h`
- `hdtSMP64/HookEvents.h`, `HookEvents.cpp`
- `hdtSMP64/EventDispatcherImpl.h`
- `hdtSMP64/ActorManager.h`, `ActorManager.cpp`
- `hdtSMP64/hdtSkyrimPhysicsWorld.h`, `hdtSkyrimPhysicsWorld.cpp`
- `hdtSMP64/hdtSkyrimSystem.h`, `hdtSkyrimSystem.cpp`
- `hdtSMP64/hdtSkyrimBone.cpp`, `hdtSkyrimBody.cpp`
- `hdtSMP64/hdtDefaultBBP.cpp`
- `hdtSMP64/main.cpp`
- `hdtSSEUtils/NetImmerseUtils.h`, `NetImmerseUtils.cpp`
- `hdtSSEUtils/FrameworkUtils.h`
- `configs/configs.xml`
- `README.md`

FO4VR comparison points:

- `ROCK/src/ROCKMain.cpp`
- `ROCK/src/api/FRIKApi.h`
- `ROCK/src/api/ROCKProviderApi.h`
- `ROCK/src/physics-interaction/core/PhysicsInteraction.*`
- `ROCK/src/physics-interaction/core/PhysicsInteractionFrame.inl`
- `ROCK/src/physics-interaction/core/PhysicsHooks.*`
- `ROCK/src/physics-interaction/native/HavokOffsets.h`
- `ROCK/src/physics-interaction/native/HavokPhysicsTiming.*`
- `ROCK/src/physics-interaction/native/PhysicsStepDriveCoordinator.*`
- `ROCK/analysis/runtime-clock-audit-2026-05-08.md`
- `SCISSORS/src/SCISSORSMain.cpp`
- `SCISSORS/src/scissors/ScissorsRuntime.*`
- `SCISSORS/src/scissors/FrameSimulationClock.h`
- `SCISSORS/notes/active-ragdoll-load-freeze-motor-review-2026-05-08.md`
- `SCISSORS/notes/planck-gravity-recheck-freeze-2026-05-09.md`
- `Fallout-4-VR-Body(FRIK)/src/FRIK.*`
- `Fallout-4-VR-Body(FRIK)/src/GameHooks.*`

## Skyrim native hook table

All offsets below are Skyrim-side reference offsets from `hdtSMP64/Offsets.h`. They are not FO4VR offsets.

| Skyrim hook or patch | Offset or symbol | What HDT-SMP does | Event or effect | FO4VR analogue |
|---|---:|---|---|---|
| Game loop | `GameLoopFunction = 0x005B2FF0` | Detours the frame function, calls original, reads pause byte from engine object, dispatches `FrameEvent`. | Drives `ActorManager` cleanup/re-add and `SkyrimPhysicsWorld` update. | ROCK main loop hook at `kHookSite_MainLoop`; FRIK `onFrameUpdate`; SCISSORS consumes ROCK provider frame callback. |
| Game shutdown | `GameShutdownFunction = 0x01293D20` | Replaces shutdown function with wrapper. | Dispatches `ShutdownEvent` before original shutdown. | F4SE unload/session cleanup plus ROCK/FRIK lifecycle destruction; no direct equivalent central shutdown event yet. |
| Armor attach | `ArmorAttachFunction = 0x001CAFB0` | Wraps the armor attach call. Dispatches once before original and once after original. | Before: merge extra bones and read BBP XML. After: create a physics system for the attached node. | Potential FO4VR actor/equipment 3D attach hook candidate, but requires Ghidra verification. Not needed for current player-hand ROCK path. |
| FaceGen skin all | `BSFaceGenNiNode_SkinAllGeometry = 0x003D87B0` | Wraps head full skinning. Dispatches before original, then after original with `hasSkinned=true`. | Allows pre-skin state tracking and post-skin scanning/physics creation. | FO4VR candidate only if we need head/hair/face geometry or actor worn-gear physics. Requires FO4VR binary mapping. |
| FaceGen skin single | `BSFaceGenNiNode_SkinSingleGeometry = 0x003D8840` | Wraps single head geometry skinning. For player or enabled NPC face parts, dispatches event instead of blindly calling through. | Lets `ActorManager` merge head bones and patch geometry skin bone refs before original skin logic continues. | Same as above: not directly relevant to ROCK hands; possibly relevant to future dynamic gear/hair/face systems. |
| Skin single bug patch | `SkinSingleGeometry + 0x96` | Writes one byte, `0x07`, at the bug site. | Source labels it as the SkinSingleGeometry bug patch. Exact instruction semantics need Skyrim disassembly to state precisely. | Do not port. Any FO4VR equivalent must be found from FO4VR disassembly. |
| FaceGen bone limit patch | `BSFaceGenModelExtraData_BoneLimit = 0x0036B4C8` | Xbyak branch patch clamps a loaded FaceGen bone count: reads `[rax+0x58]`; if count >= 9, forces `esi = 8`. | Prevents FaceGen extra-data bone handling from exceeding the engine path HDT-SMP expects. | Do not port. FO4VR face/geometry limits need independent verification. |
| Game step timer read | `GameStepTimer_SlowTime = 0x02F6B948` | Reads engine-scaled delta each frame. | Physics uses game time, not wall clock. | ROCK already has game-frame delta and native Havok timing; SCISSORS uses ROCK frame clock and avoids wall-clock simulation advancement. |
| Face geom path call | `TESNPC_GetFaceGeomPath = 0x00363210` | Calls the game helper to find original facegen NIF when `FMD` extra data is missing. | Recovers original headpart skeleton/bone names for merge. | FO4VR equivalent would only matter for future face/headpart work and must be verified. |

## Internal event model

HDT-SMP creates its own event bus in `HookEvents.*`:

- `FrameEvent`: contains `gamePaused`.
- `ShutdownEvent`: no payload.
- `ArmorAttachEvent`: contains source armor model, target skeleton, attached node, and `hasAttached`.
- `SkinSingleHeadGeometryEvent`: contains skeleton, head node, geometry, unknown arg.
- `SkinAllHeadGeometryEvent`: contains skeleton, head node, unknown arg, and `hasSkinned`.

`EventDispatcherImpl<T>` stores listeners, protects registration/dispatch with a recursive mutex, and keeps a cached listener vector so dispatch does not iterate the mutation set directly. The detour layer only emits events; `ActorManager` and `SkyrimPhysicsWorld` own the actual behavior.

That split is the important pattern for FO4VR. The hook should answer "what engine boundary just happened?", while the runtime should answer "what state is now safe to mutate?"

## Skyrim lifecycle flow

### Frame/update flow

```text
Skyrim game loop
  -> HDT hook calls original game frame
  -> dispatch FrameEvent(gamePaused)
  -> ActorManager:
       removes physics for inactive scene skeletons
       re-adds systems when skeletons become active again
       cleans detached armor/headpart generated nodes
  -> SkyrimPhysicsWorld:
       checks engine pause byte and MenuManager::IsGamePaused()
       suspends/resumes physics
       reads GameStepTimer_SlowTime
       runs Bullet step when active
       writes current transforms during non-loading pauses
```

The pause handling is a real fix. Old behavior could let physics advance or visually reset at the wrong time when menus/loading interrupted the scene. The current path distinguishes normal paused frames from loading suspension:

- Paused but not loading: keep writing transforms so visuals do not snap back.
- Loading or RaceSex menu: suspend as loading, reset systems on resume.
- Active frame: read transforms, simulate, write transforms.

### Loading/menu flow

`main.cpp` registers a `MenuOpenCloseEvent` sink. On `Loading Menu` or `RaceSex Menu` open, it calls `SkyrimPhysicsWorld::suspend(true)`. On `RaceSex Menu` close, it calls `resume()` and `ActorManager::reloadMeshes()`.

This is why the engine fix is more than a physics tick hook. It knows that some menus invalidate or rebuild scene nodes and treats those transitions as lifecycle resets.

### Shutdown flow

Shutdown dispatch clears both high-level tracking and physics:

- `ActorManager::onEvent(ShutdownEvent)` clears skeleton records.
- `SkyrimPhysicsWorld::onEvent(ShutdownEvent)` removes collision objects, constraints, rigid bodies, and systems.

The lesson for FO4VR is to make teardown a first-class boundary. Keeping stale generated bodies or stale FRIK skeleton pointers across load/shutdown is the equivalent failure mode.

## Armor attach flow

Skyrim armor attach is the cleanest example of the two-phase pattern:

```text
Before original armor attach:
  find actor skeleton from armor target
  create per-armor tracking record
  merge any missing armor skeleton bones into actor skeleton
  rename merged nodes with a unique prefix
  scan armor for HDT XML path

Original game armor attach:
  engine attaches the armor model

After original armor attach:
  create SkyrimSystem using the live attached node and rename map
  add system to SkyrimPhysicsWorld
```

Key details:

- Generated node prefix format is `hdtSSEPhysics_AutoRename_%016llX`.
- Bone names are stored in `renameMap` from original names to generated live names.
- `scanBBP()` reads `NiStringExtraData` named `HDT Skinned Mesh Physics Object`; if absent, it falls back to `defaultBBPs.xml` shape-name mapping.
- First-person skeletons are skipped for armor physics creation.
- `cleanArmor()` removes generated prefixed nodes and removes the physics system when armor is detached.

The fix here is deterministic ownership. HDT-SMP does not just find some nodes and run physics. It knows which generated nodes belong to which armor attach, can remove them, and can recreate systems when the scene graph reactivates.

## FaceGen/headpart flow

The head path exists because FaceGen geometry is skinned and rebuilt differently from regular armor. HDT-SMP handles both "skin all" and "skin single" paths.

```text
SkinAll before original:
  mark full head skinning
  clear stale head state as needed

SkinSingle:
  locate actor skeleton and head node
  find original headpart root from FMD extra data when available
  if FMD missing, call TESNPC_GetFaceGeomPath and load original NIF
  derive expected bone names from original geometry skin data
  merge missing head skeleton bones into actor skeleton
  patch geometry skin bone references/world transform pointers to live nodes
  scan for BBP XML

SkinAll after original:
  scan rebuilt head geometry
  create/recreate headpart physics systems
```

The important part is that geometry skin data is redirected to the merged live actor skeleton. The physics bones are not detached objects floating near the actor; they become part of the same scene graph the renderer and animation system are using.

For FO4VR this is not immediately transferable unless we are adding head/hair/face physics. The concept is transferable: for any generated FO4VR collider or visual helper, the owner node and world pointer must be current for the frame it is driven.

## Physics world behavior

`SkyrimPhysicsWorld` is a singleton `SkinnedMeshWorld` backed by Bullet. It listens for `FrameEvent`, `ShutdownEvent`, and SKSE camera events.

On active update:

1. Average/clamp the interval.
2. Read all game skeleton transforms into physics objects.
3. Update active state.
4. Translate all Bullet bodies around their mean position.
5. Step Bullet.
6. Translate bodies back.
7. Write physics transforms back to NiNodes.

The temporary translation is a precision fix. Skyrim world coordinates can get large, and Bullet is more stable when the active simulation is near a local origin.

On camera transition:

- If SKSE camera event changes from state `0` to `9`, HDT-SMP sets `m_resetPc = 3`.
- `SkyrimSystem::readTransform()` consumes that counter and forces player-character systems to reset for three frames.

This is another lifecycle reset, not a solver tweak.

## Player rotation and snap handling

`SkyrimSystem::readTransform()` detects dangerous root changes before feeding transforms into physics:

- If the root scene node changes, force `RESET_PHYSICS`.
- On first initialization, force `RESET_PHYSICS`.
- For player-attached systems, track last root rotation.
- If `clampRotations` is enabled, clamp root rotation delta to `10.0f * timeStep` and update child transforms.
- If unclamped reset mode is enabled, force reset when rotation exceeds `unclampedResetAngle * timeStep`.

This is the "large turn/body snap" part of the fix. The engine can rotate the player root much faster than the physics system should interpret as physical angular velocity. HDT-SMP either clamps that input before physics reads it, or resets physics instead of converting it into a violent impulse.

FO4VR analogue:

- FRIK and ROCK should treat large player-space/root-space discontinuities as reset or drive-suppression boundaries.
- SCISSORS already has root compensation and load-settle gates for active ragdoll work.
- ROCK already drives generated colliders from native physics-step timing, which is stronger than Skyrim's frame-only timer.

## Bone/body transform details

`SkyrimBone::readTransform()`:

- Reads the live `NiNode` world transform.
- Adjusts mass/inertia and collision shape scaling when node scale changes.
- On reset, copies the destination transform directly into the Bullet rigid body and clears velocity.
- For kinematic/static bodies, calculates velocity from old to new transform for the current time step.

`SkyrimBone::writeTransform()`:

- Converts the Bullet rigid body transform back into the `NiNode` world transform.
- Does not update local transform in the active code path.

`SkyrimBody::canCollideWith()`:

- Applies disabled state and sharing rules:
  - public: can collide globally if base rules allow it
  - internal: only within same skeleton
  - private: only within same mesh

That body sharing policy is separate from engine hooks, but it matters because the hook layer can create several systems attached to the same actor.

## Config switches that matter

From `configs/configs.xml` and `README.md`:

- `enableNPCFaceParts`: enables NPC face/headpart processing beyond the player.
- `clampRotations`: enables root-rotation clamping.
- `unclampedResets`: enables reset-on-large-rotation when clamping is disabled.
- `unclampedResetAngle`: angle threshold scale for reset mode.
- solver iteration settings and min-fps settings control the Bullet side, not the engine hook map.

README also notes required SKSE header edits:

- Add `MenuManager::IsGamePaused()`.
- Fix an event dispatcher field type.
- Replace `NiAVObject` unknown fields with `TESObjectREFR* m_owner` at `0xF8`.

That last point is important. HDT-SMP depends on owner pointers on scene nodes to connect skeletons back to actors. FO4VR equivalents must be verified against CommonLibF4VR and the VR binary, not assumed from Skyrim or flat FO4.

## What FO4VR already has

PROJECT_ROCK_V2 already covers several Skyrim-HDT ideas, often at a better boundary for FO4VR:

| Skyrim concept | Existing FO4VR mechanism |
|---|---|
| Frame hook and internal runtime update | ROCK hooks the main loop and calls `onFrameUpdate()`. FRIK also has its own main update path. |
| Physics delta source | ROCK uses FRIK/game delta plus native Havok timing; SCISSORS uses ROCK frame snapshots. |
| Pause/menu gate | FRIK exposes `isAnyMenuOpen()`. ROCK frame context has `menuBlocked`. ROCK provider snapshot exposes `menuBlocking` and `configBlocking`. |
| Loading/skeleton invalidation | FRIK dispatches `kSkeletonDestroying` before release and `kSkeletonReady` after init. ROCK creates/destroys `PhysicsInteraction` on those lifecycle events. |
| World pointer validity | ROCK tracks `bhkWorld` and `hknpWorld`, recreates generated bodies when the world changes, and defers when the world is null. |
| Physics-step drive | ROCK `PhysicsStepDriveCoordinator` registers a native listener and drives generated colliders before Havok steps/substeps. |
| Downstream event bus | ROCK provider API publishes frame snapshots and contact snapshots. SCISSORS consumes this instead of installing another frame hook. |
| Load-settle protection | SCISSORS has `WorldRecoveryGate` and frame-counted load-settle gating before actor lifecycle activation. |
| Multi-callsite frame clock | SCISSORS `FrameSimulationClock` prevents several native callsites in one ROCK frame from advancing simulation timers multiple times. |
| Validated hook installation | ROCK and SCISSORS check callsite bytes, call targets, vtables, and prologues before patching. This is safer than raw Skyrim-style address detours. |

## What is worth copying in spirit

### 1. Make lifecycle states explicit in the provider

Skyrim HDT-SMP gained stability by making hidden engine transitions visible. ROCK provider snapshots already expose readiness, menus, config blocking, worlds, and FRIK skeleton readiness. A future provider extension could make the boundary even clearer:

- `worldGeneration`
- `skeletonGeneration`
- `loadingOrWorldTransition`
- `physicsWriteAllowed`
- `visualWriteAllowed`
- `generatedBodiesValid`
- `lastLifecycleReason`

This would mostly document and expose state ROCK/SCISSORS already infer internally.

### 2. Keep two-phase attach as the model for actor/equipment work

If FO4VR later needs NPC equipment, armor, hair, or worn-object physics, copy the two-phase pattern:

```text
before engine attach:
  inspect owner and source geometry
  prepare rename/ownership map
  do not assume attached node is live yet

after engine attach:
  bind to live node
  create native/generated physics objects
  publish readiness
```

Do not use the Skyrim attach offset. Find FO4VR's attach path in Ghidra and validate callsites before any hook.

### 3. Treat root discontinuities as lifecycle events

Skyrim's clamp/reset logic is really a discontinuity detector. FO4VR should keep this policy for:

- FRIK skeleton rebuilds
- power armor swaps
- loading/menu transitions
- player space or room node discontinuities
- large generated collider target jumps
- active ragdoll root compensation jumps

SCISSORS already implements several of these ideas. ROCK should keep generated collider drive tied to native physics-step timing and reset generated body state when the target transform changes discontinuously.

### 4. Preserve visual output during blocked frames where safe

Skyrim writes transforms during non-loading paused frames so visuals do not appear to reset. FO4VR analogue:

- During menu/config block, avoid advancing physics authority.
- Preserve or publish the last coherent visual/provider snapshot when the world and skeleton are still valid.
- Do not mutate native hknp world state from unsafe menu/loading windows.

This is already close to ROCK's current model. The useful action is making the policy explicit in docs/logs.

### 5. Keep owner cleanup deterministic

HDT-SMP names and tracks generated nodes so it can remove only what it owns. FO4VR generated bodies, contact registrations, and downstream external body registrations need the same ownership discipline:

- token or generation for every generated object
- owner module
- owner hand/actor/body role
- world pointer/generation
- cleanup path on skeleton destroying, world change, provider loss, and shutdown

ROCK's external body registration and provider-loss cleanup already follow this direction.

## What should not be copied

- Do not port Skyrim offsets.
- Do not copy the FaceGen byte patches without FO4VR disassembly.
- Do not assume SKSE `NiNode::m_owner` layout applies to FO4VR.
- Do not add a second independent frame authority for SCISSORS; it should keep using ROCK provider frames.
- Do not mutate FO4VR hknp world internals from arbitrary animation callsites just because Planck or Skyrim hkp code did so. FO4VR's hknp boundary is different and has already produced freeze risk in SCISSORS notes.

## Recommended FO4VR follow-up map

No code change is recommended until these are verified:

1. Document current ROCK/FRIK/SCISSORS lifecycle state machine in one place:
   - F4SE load
   - FRIK skeleton ready
   - ROCK provider ready
   - bhk/hknp world available
   - menu/config blocked
   - loading/world unavailable
   - skeleton destroying
   - provider loss/shutdown

2. Add a read-only audit of whether ROCK provider snapshots should expose `worldGeneration` and `skeletonGeneration`.

3. If future actor/equipment physics is desired, start a separate Ghidra pass for FO4VR attach/skinning equivalents:
   - actor 3D attach/detach
   - biped/equipment model attach
   - face/headpart geometry skinning, only if needed
   - owner pointer or reference path from `NiAVObject`/collision body back to `TESObjectREFR`

4. Keep SCISSORS active-ragdoll work downstream of ROCK provider state. The Skyrim lesson supports this design: one lifecycle authority should own physics readiness.

5. Continue using validated hook preflight. Every FO4VR hook candidate should have:
   - expected bytes or expected call target
   - original target saved
   - rollback on mismatch
   - log that identifies the exact offset and reason for refusal

## Bottom line

The Skyrim HDT-SMP version fixes engine instability by manufacturing missing lifecycle events, then making physics obey those events. Its biggest ideas are not Bullet-specific:

- hook engine boundaries, not random symptoms
- separate hook capture from runtime policy
- do two-phase attach around engine ownership changes
- reset on skeleton/world/root discontinuities
- keep generated objects owned and removable
- drive simulation from game/native physics time, not wall time

For FO4VR, PROJECT_ROCK_V2 is already aligned with that architecture through FRIK lifecycle messages, ROCK provider snapshots, native physics-step coordination, and SCISSORS world recovery gates. The next similar move is not another raw hook immediately; it is a documented FO4VR lifecycle map plus generation/write-allowed state exposed through ROCK where downstream modules need it.
