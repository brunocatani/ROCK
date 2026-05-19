# Multipart Activation Backup Analysis

## Scope

This note compares current ROCK against the manual backup at:

`E:\fo4dev\Backups\before motor change\PROJECT_ROCK_V2\ROCK`

The goal is to explain why weapons and Giddyup Buttercup style multipart refs had better activation/handling behavior in that backup, and why the current owner-node fallback scan is probably not the right primary fix.

## Main Finding

The backup did not solve multipart activation by discovering more bodies. Its larger behavioral difference is that ordinary single-hand loose-object grabs still used FO4VR's native mouse-spring action.

Backup path:

- `HandGrab.cpp` kept `HeldObjectDriveMode::NativeMouseSpring` for normal single-hand grabs.
- `_nativeGrab.create(...)` created Bethesda's mouse-spring action against the selected body.
- `_nativeGrab.flush(...)` called the native mouse-spring update every held frame.
- The shared/custom constraint path was only used when a second hand joined or when native grab was promoted.

Current path:

- Ordinary single-hand grabs now always create a generated hidden proxy body plus ROCK's custom finite-force constraint.
- `NativeMouseSpringGrab` no longer exists in current `src/`.
- Current code manually scans body IDs, activates them, leases body flags, compensates player-space velocity, and restores lifecycle state.

That means the current implementation is not just missing a body from the held set. It replaced the engine's native held-object authority with a custom constraint authority, then tried to reconstruct the engine-owned multipart behavior from scene-tree scans.

## Why The Current Fallback Is The Wrong Center

The owner-node world fallback scan added for multipart grabs is expensive and still has the same conceptual boundary:

- It only accepts hknp bodies whose native owner node resolves under the selected ref root.
- It still depends on `Get3D()` scene-tree ownership being the same as the native physics ownership.
- It runs synchronously during grab and far-pull acquisition, so a close grab can scan before/after prep and a pull-to-grab can scan multiple times.

If a weapon or Giddyup multipart body is connected by native physics system ownership, constraint island, shared motion, or an engine-side held action contract rather than by a clean child node under the selected ref root, the fallback can still miss it. If it does not miss it, the scan cost can still cause the visible pickup stutter.

## Important Deltas

### Backup Native Single-Hand Drive

Backup `HandGrab.cpp`:

- `_heldDriveMode = HeldObjectDriveMode::NativeMouseSpring`
- `_nativeGrab.create(...)`
- `driveCreated = joiningPeerHeldObject ? _activeConstraint.isValid() : _nativeGrab.isValid()`
- Update path queues and flushes the native mouse-spring action.

This matters because the native action is likely doing more than just applying velocity to one body. It is the same engine boundary Fallout 4 VR expects for held physics objects.

### Current Proxy Constraint Everywhere

Current `HandGrab.cpp`:

- `kHeldObjectDriveName = "proxyConstraint"`
- Normal grab calls `createProxyConstraintGrabDrive(...)`
- Success requires `_activeConstraint.isValid() && _grabAuthorityProxy.isValid()`
- Manual support was added around it: `activateHeldObjectBodySet(...)`, held body flag leases, release reactivation, fallback body scanning.

This is a full replacement of the engine grab action, not a small motor tuning change.

### Lifecycle Capture Became Over-Broad

Backup `GrabCore.h` captured before-prep bodies for:

- accepted bodies
- static motion
- keyframed passive
- not-dynamic-after-active-prep

Current `GrabCore.h` also captures:

- actor layer
- non-collidable layer
- unsupported layer

Current `markPreparedBodies(...)` also marks captured records as changed when the later scan shows acceptance or motion/filter change. This may be useful for restore, but it also means broad scan results can become lifecycle authority even when they were only supplemental or diagnostic.

## What This Means For The Fix

Do not keep expanding the world fallback scan as the primary solution.

The production-quality path should be:

1. Restore the native mouse-spring drive for ordinary single-hand loose-object grabs, including its source files and `Hand` state.
2. Keep the proxy/custom constraint path for the cases it was actually solving: two-hand/shared grabs and explicit promotion from native to constraint.
3. Revert the broad owner-node fallback scan from the hot grab path unless new telemetry proves a narrow object-local scan still misses required bodies.
4. Keep held-body bookkeeping for contacts, velocity, inertia, and damping, but do not treat it as a replacement for the native held-object action.
5. Only after the native path is restored, retest weapons and Giddyup. If multipart deactivation is still incomplete, use targeted telemetry around native action creation/update/release and body-set membership.

## Open Runtime Checks

The current telemetry can still answer useful questions during one test:

- Does `Object body scan diagnostic` show large `highWater`/`slots` and measurable `elapsed` during the stutter?
- Do weapon/Giddyup records show accepted multipart bodies, or only one accepted primary?
- Are rejected bodies failing because of layer, motion type, unresolved ref, or foreign ref?

But these logs should guide the native-path restoration, not justify more full-world scanning.
