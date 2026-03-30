# Runtime Clock Audit - 2026-05-08

## Why

ROCK physics and interaction state must follow game/update frames or FO4VR Havok step timing. Wall-clock timers can expire during debugger stalls, loading, menus, reprojection spikes, or normal hitches, which can turn a paused/stalled simulation into skipped state, wrong hit windows, stale timeouts, or incorrect keyframed-body drive.

## Scan Scope

Searched the ROCK repo outside `build/` for direct wall-clock and sleep APIs:

- `GetTickCount64`, `GetTickCount`
- `QueryPerformanceCounter`
- `timeGetTime`
- `std::chrono::*clock`
- `std::chrono::milliseconds`
- `std::this_thread::sleep_for`, `sleep_until`
- `Sleep`, `SetTimer`, `KillTimer`

Also checked frame/delta paths for generated hand, body, weapon, held-object, soft-contact, grab, and debug systems.

## Findings

### Behavior-affecting runtime wall-clock: fixed

`PhysicsHooks.cpp` used `GetTickCount64()` for the native melee physical-swing lease. That path can affect native hit-frame suppression, so it was converted from a 250 ms wall-clock lease to a ROCK update-frame lease.

New behavior:

- `g_nativeMeleeFrameClock` advances from `PhysicsInteraction::update()`.
- `setNativeMeleePhysicalSwingActive()` stores an expiry frame instead of an expiry millisecond tick.
- `isNativeMeleePhysicalSwingActive()` and hook policy evaluation compare current frame to expiry frame.
- `PhysicsInteraction::shutdown()` clears outstanding native melee swing leases.

### Diagnostic wall-clock: fixed

Grab transform telemetry stamped logs/overlay with `tickMs = GetTickCount64()`. It was diagnostic-only, but it still put wall-clock reads in the interaction runtime. The telemetry stamp is now session/frame based only.

### Config watcher wall-clock: allowed

`RockConfig.cpp` still uses `std::chrono` and `std::this_thread::sleep_for` for the ROCK.ini file-watch debounce. This is not physics, input, collision, grab, or generated-body authority. It only waits for file writes to settle before setting `_reloadPending`.

### Logging sample throttle: external/logging only

ROCK uses `ROCK_LOG_SAMPLE_*`, which delegates to the F4VR CommonFramework logger. No direct ROCK gameplay state depends on those sampled logging calls. Treat as diagnostics only.

## Generated Body Timing Check

Generated hand/body/weapon colliders do not drive from wall-clock time:

- Frame update queues source targets with the frame delta.
- `PhysicsStepDriveCoordinator` registers a bhkWorld step listener.
- Whole-step native grab flushes use `havok_physics_timing::driveDeltaSeconds()`.
- Generated colliders flush from the FO4VR before-any-physics-step callback.
- `HavokPhysicsTiming.cpp` reads the binary-backed bhkWorld delta/substep globals, not FRIK or wall-clock time.

## Guard Added

`tests/RuntimeClockBoundarySourceTests.ps1` now rejects direct wall-clock and sleep APIs in runtime source files, with `RockConfig.cpp` as the only current allowed exception for INI debounce.
