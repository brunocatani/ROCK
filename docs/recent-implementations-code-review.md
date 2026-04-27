# Recent Implementations Code Review

Date: 2026-04-27

This review covers the recent ROCK changes around generated weapon collision, semantic weapon parts, two-handed grip, vanilla reload observation, projectile layer filtering, grab/finger tuning, API additions, and native melee suppression. The goal is to identify production gaps before more systems are layered on top.

## Verification Run

- `ROCK/build/Release/ROCKTransformConventionTests.exe` passed.
- `cmake --build build --config Release` passed and produced `ROCK/build/Release/ROCK.dll`.
- No fresh Ghidra verification was run during this review pass.

## Fix Pass

The fixes in this pass preserve the current architecture and remove unsafe lifetime edges before more physical reload, two-handed grip, and melee replacement code is layered on top. HIGGS was used as the practical reference for restoring temporary two-handed transforms and separating live weapon state from derived collision state. Previously verified FO4VR offsets were centralized, but no new reverse-engineering claim was introduced without a fresh Ghidra pass.

Fixed:

- Native melee hook install now runs after critical offset validation, validates target vtable slots, tracks weapon-swing and hit-frame hooks separately, and retries missing hook installs instead of poisoning the global installed flag.
- Native melee physical-swing state now uses a short timestamp lease instead of sticky booleans, preventing a future physical swing bridge from accidentally suppressing native hit frames forever.
- Reload event source offsets now live in `HavokOffsets.h`; the event bridge resolves both native sources before registration and marks itself installed only after registration succeeds.
- Reload physical-completion gating now respects `bReloadAllowStageFallbacks`. Until a verified physical insertion signal exists, ammo-commit can complete the coordinator when fallback is allowed, so the reload router does not remain permanently active.
- Follow-up runtime log review showed FO4VR can emit `VanillaReloadStarted` and `AmmoCommitted` without a later visible `Complete` event in ROCK's observer. The coordinator now treats ammo commit as runtime completion when physical completion is unsupported and fallback is allowed, restoring support-grip routing instead of leaving `supportAllowed=false`.
- Follow-up two-handed runtime review showed no `TwoHandedGrip` state transitions because ROCK was depending on hand/weapon contact events for two keyframed bodies. ROCK now has a HIGGS-style active left-hand weapon interaction probe as fallback/selector, explicitly keeps layer 43 vs 44 enabled, and suppresses the left hand collision body only while two-handed support grip is active so the support hand does not solve against the weapon package it is driving.
- Weapon collision world-change and shutdown paths destroy existing ROCK bodies instead of clearing local references while live Havok bodies may still exist.
- Left-hand weapon contact now has a freshness timeout and is cleared on shutdown, so API-visible part-kind data does not remain valid forever after contact ends.
- Two-handed grip now records/restores the weapon node local baseline and adds support-normal twist solving around the primary/support axis. This is still not the full semantic grip profile system, but it removes the most immediate roll/ownership gap.
- Grab pose release now guards the FRIK API pointer, and high-frequency finger pose logs are gated behind `bDebugGrabFrameLogging`.
- Semantic weapon hull budgeting now preserves at least one gameplay-critical hull per part kind before filling the remaining cap by priority.
- Weapon collision follow velocity clamps are now configurable through `fWeaponCollisionMaxLinearVelocity` and `fWeaponCollisionMaxAngularVelocity`, with defaults matching the previous hardcoded values.

Residual runtime validation:

- Native melee hook target validation still needs an in-game log pass to confirm the vtable slots match the verified FO4VR handlers on the user's runtime build.
- Reload event observation still needs in-game validation across weapon reloads because the physical ammo/socket completion path is intentionally scaffolded but not complete.
- Two-handed support grip is improved, but the full profile-driven finger/hand basis and collision-body manipulation system remains separate implementation work.

## Findings

The items below are the original review findings retained for auditability. The `Fix Pass` section above records which issues were addressed in this implementation pass and which ones still need runtime validation or a larger feature implementation.

### Native hook validation gaps

- `PhysicsInteraction` installs native hooks in the constructor before `validateCriticalOffsets()` runs.
- `installNativeMeleeVtableHook()` patches fixed vtable entries without validating that the original slot points at the expected FO4VR function.
- Failed native melee hook installation marks the hook set as installed before the individual installs succeed, so a transient or partial failure is not retried.

Relevant files:
- `src/physics-interaction/PhysicsInteraction.cpp`
- `src/physics-interaction/PhysicsHooks.cpp`
- `src/physics-interaction/HavokOffsets.h`

### Reload bridge runtime safety gaps

- `WeaponReloadEventBridge` uses hardcoded reload/ammo event offsets locally instead of centralizing them with the rest of the verified offsets.
- The bridge marks itself installed before calling the native registration functions.
- It calls the registration wrappers before checking/logging that the underlying event sources are present.

Relevant file:
- `src/physics-interaction/WeaponReloadEventBridge.cpp`

### Reload coordinator can stay active forever

With `bReloadRequirePhysicalCompletion=true`, vanilla `Complete` is blocked in the coordinator until `physicalAmmoInserted` becomes true. There is currently no real physical insertion path feeding that flag. After a vanilla reload reaches ammo commit and complete, the runtime state can remain `WeaponUnloaded`, which keeps support grip disabled and reload routing active.

Relevant files:
- `src/physics-interaction/PhysicsInteraction.cpp`
- `src/physics-interaction/WeaponReloadStageObserver.h`
- `data/config/ROCK.ini`

### Weapon body reset without destruction can orphan live Havok bodies

`WeaponCollision::resetWeaponBodiesWithoutDestroy()` releases ROCK's local shape reference and clears body state without removing the body from the Havok world. It is used on world changes and shutdown paths where body destruction was skipped. If the old world still references those bodies/shapes, this can leak bodies or leave stale pointers.

Relevant file:
- `src/physics-interaction/WeaponCollision.cpp`

### Two-handed grip is not yet the full semantic grip profile system

The current implementation routes by semantic body role, but the actual grip solve still records a closest mesh point at activation and uses a two-point axis solve. It does not yet use full `WeaponGripProfile` data such as palm normal, hand forward/up basis, snap radius, priority, or weapon-specific overrides. The solver also does not constrain roll around the primary-to-support axis.

Relevant files:
- `src/physics-interaction/TwoHandedGrip.cpp`
- `src/physics-interaction/WeaponTwoHandedSolver.h`
- `src/physics-interaction/WeaponInteractionRouter.h`

### Two-handed grip writes live weapon and hand nodes directly

`TwoHandedGrip::updateGripping()` overwrites the weapon node local transform and both hand node local positions. It does not retain a restoration baseline for the weapon transform and relies on FRIK/game updates to correct or supersede the node later. This can create transform ownership fights once reload actions, weapon position adjustment, or FRIK weapon updates run in the same frame.

Relevant file:
- `src/physics-interaction/TwoHandedGrip.cpp`

### Left weapon contact API can be stale

The runtime consumes the contacted weapon body ID per frame, but the published part-kind fields remain as the last successful contact indefinitely. API consumers get only `getLastTouchedWeaponPartKind()` with no body ID, validity flag, sequence age, or timeout.

Relevant files:
- `src/physics-interaction/PhysicsInteraction.cpp`
- `src/physics-interaction/PhysicsInteraction.h`
- `src/api/ROCKApi.cpp`

### Native melee suppression is suppress-only right now

The hook policy is structured well, but the physical swing bridge that later invokes native hit/damage side effects does not exist yet. The physical swing active flag is also a sticky boolean with no frame/timestamp lifetime, so future callers can accidentally leave hit-frame behavior permanently handled.

Relevant files:
- `src/physics-interaction/PhysicsHooks.cpp`
- `src/physics-interaction/NativeMeleeSuppressionPolicy.h`

### Grab pose release has an unchecked FRIK API pointer

`Hand::releaseGrabbedObject()` calls `FRIKApi::inst->clearHandPose(...)` without checking `FRIKApi::inst`. Most runtime paths probably have FRIK available, but release also runs during shutdown/skeleton transitions where this should be guarded.

Relevant file:
- `src/physics-interaction/HandGrab.cpp`

### High-frequency finger pose logs can become expensive

Finger pose application logs at grab start and during periodic held-object pose updates. With `iGrabFingerPoseUpdateInterval=3`, this can become very noisy while holding objects and can obscure more important runtime logs.

Relevant file:
- `src/physics-interaction/HandGrab.cpp`

### Weapon hull budgeting lost spatial balancing

The new budget preserves semantic priority, but it no longer keeps geometric spread as a secondary selection policy. Over-budget weapons with many similarly classified receiver/body hulls may preserve large high-point shapes while dropping smaller spatially important hulls.

Relevant files:
- `src/physics-interaction/WeaponSemanticHullBudget.h`
- `src/physics-interaction/WeaponPartClassifier.h`

### Config keys exist with no behavior

`bReloadAllowStageFallbacks` is read from the INI but is not used anywhere in runtime logic yet.

Relevant files:
- `src/RockConfig.h`
- `src/RockConfig.cpp`
- `data/config/ROCK.ini`

### Hardcoded weapon body velocity clamps

Generated weapon body follow uses hardcoded max linear and angular velocities rather than config values. This limits tuning and makes weapon collision smoothing inconsistent with the grab physics tuning path.

Relevant file:
- `src/physics-interaction/WeaponCollision.cpp`
