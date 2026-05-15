# Dynamic Grab Activation And Force Stage - 2026-05-15

## Scope

- Keep ragdoll angular atom work paused.
- Re-enable only the hidden grab authority proxy visualization for runtime inspection.
- Fix dynamic loose-object/loose-weapon activation and cleanup before returning to angular atom work.
- Fix slow-motion feel on heavy ordinary props without changing the grip pivot or using COM as authority.

## Runtime INI State

- Live file: `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`.
- `bDebugDrawGrabAuthorityProxy=true` was enabled in-place.
- Packaged default remains repository-controlled and should be updated only as part of a deliberate config change.

## Log Findings

### Hidden Proxy

- Current grab authority still reports `proxyFrame=livePalmAnchorMotionGrabFrame/ok`.
- The hidden proxy visualization is the right debug target for seeing the actual authority body, not all hand/body collider debug.

### Multipart Weapon Activation

Observed in rotated logs:

- `Bolt Action Hunting Rifle`:
  - `PULL scan ... beforeBodies=5 afterBodies=5 accepted=5 rejected=0`
  - `PULL start ... bodyCount=1`
  - held grab later reports `heldBodies=5`.
- `SA58 FAL`:
  - `PULL scan ... beforeBodies=7 afterBodies=7 accepted=7 rejected=0`
  - `PULL start ... bodyCount=1`
  - held grab later reports `heldBodies=7`.

Interpretation:

- The scanner is finding the multipart weapon bodies correctly.
- Pull stores `_pulledBodyIds = preparedBodySet.uniqueAcceptedMotionBodyIds()`.
- All weapon bodies share one hknp motion, so the unique-motion list collapses the accepted set to one body.
- Velocity dedupe by motion is correct, but activation/logging/ownership should still know all accepted bodies.
- This explains partial activation symptoms during pull without blaming scanner failure.

### Pull Prep Cleanup

- Pull prep calls:
  - `setMotionRecursive(rootNode, Dynamic, recursive=true, force=true, activate=true)`
  - `enableCollisionRecursive(rootNode, true, recursive=true, force=true)`
- Active grab has a lifecycle snapshot and release restore path.
- Pull currently does not keep an equivalent lifecycle snapshot once the pull is started.
- If a pull is cancelled before grab, or if active grab captures only the post-pull dynamic state, original motion/filter context can be lost.

Fix direction:

- Capture pull lifecycle before recursive active prep.
- Mark all prepared bodies after recursive active prep.
- Restore abandoned pull prep on pull cancellation.
- Hand pull lifecycle to active grab when a pulled object is promoted into held state.

### Heavy Object Slow-Motion

Observed examples:

- `Toaster` mass about `49.95`, generic object, force remains `linearForce=2000 angularForce=160`.
- `Sugar Bombs` mass about `9.99`, generic object, same `linearForce=2000 angularForce=160`.
- Loose weapons get higher linear force via loose-weapon multiplier, for example `linearForce=9000 angularForce=720`.

Interpretation:

- Generic objects use a fixed absolute base force.
- `fGrabMaxForceToMassRatio=500` currently only caps force down; it does not raise force for heavier objects.
- Heavy generic props therefore get very low acceleration compared with lighter props and can feel like slow motion.

Fix direction:

- Keep finite force and mass cap.
- Add mass-responsive generic force ceiling so heavy props can receive more absolute force up to a configured ceiling.
- Do not let this become infinite-strength follow. The cap remains `mass * fGrabMaxForceToMassRatio`.
- Do not boost loose weapons beyond their existing loose-weapon force ceiling unless explicitly configured later.

## Code Targets

- `src/physics-interaction/hand/HandGrab.cpp`
  - `Hand::startDynamicPull`
  - `Hand::grabSelectedObject`
  - pull lifecycle restore/consume helpers
  - motor input wiring
- `src/physics-interaction/hand/Hand.cpp`
  - `clearPullRuntimeState`
- `src/physics-interaction/hand/Hand.h`
  - pull lifecycle fields/helpers
- `src/physics-interaction/grab/GrabMotionController.h`
  - mass-responsive force ceiling
- `src/RockConfig.*`
  - config surface for mass-responsive force ceiling
- `data/config/ROCK.ini`
  - packaged default/comment
- `tests/DynamicPullWeaponScanSourceTests.ps1`
  - source guard for pull using accepted body ids and lifecycle handoff
- `tests/GrabMotionControllerPolicyTests.cpp`
  - mass-responsive force policy tests

## Expected Validation

- Far-pulled multipart weapons should log `bodyCount=5` or `bodyCount=7`, not `1`.
- Cancelled pull should log lifecycle cleanup instead of silently clearing runtime state.
- Pull-to-held active grab should retain the pre-pull lifecycle snapshot.
- Release lifecycle should still restore all held-body filters.
- Heavy generic props should no longer be limited to the same `2000` absolute force as light props, while still obeying mass ratio caps.

## Implemented In This Pass

- Live production INI now has `bDebugDrawGrabAuthorityProxy=true`, so only the hidden grab authority proxy can be inspected again without changing the packaged default debug state.
- Dynamic pull now keeps `preparedBodySet.acceptedBodyIds()` as the pull body ownership/activation set instead of collapsing multipart weapons through `uniqueAcceptedMotionBodyIds()`.
- Pull prep now captures a lifecycle snapshot before recursive dynamic/collision prep and marks the prepared bodies after prep.
- Abandoned pull cleanup restores the captured pull lifecycle when selection/pull state is cleared while the world is still valid.
- Pull arrival no longer clears the prep lifecycle. It stops pull motion state but keeps the lifecycle armed until the pull-catch grab consumes it or selection cleanup restores it.
- Pull-catch grab consumes the pull lifecycle snapshot instead of recapturing post-pull dynamic state as the original state.
- Generic object motor force now has `fGrabMassResponsiveMaxForce`, default `9000.0`, while `fGrabMaxForceToMassRatio` still caps light objects down by mass.
- Loose weapons are not double-boosted by the new mass-responsive ceiling: their existing loose-weapon multiplier still produces the 9000/720 force pair.

## Validation Already Run

- `powershell -ExecutionPolicy Bypass -File tests\DynamicPullWeaponScanSourceTests.ps1`
- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS`
- `ctest --test-dir build-tests -C Release -R "GrabMotionController|DynamicPullWeaponScan" --output-on-failure`
- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m`

## Next Runtime Checks

- Far pull a multipart loose weapon and confirm `PULL start ... bodyCount=` matches accepted scan count instead of `1`.
- Cancel a pull before touch grab and confirm lifecycle audit/restore logs appear.
- Pull to held grab and release the same object, then confirm release still restores all held body filters.
- Grab the hammer or another heavy ordinary prop and compare force logs: expected `linearForce` should rise above `2000` when mass allows, capped by `mass * fGrabMaxForceToMassRatio` and `fGrabMassResponsiveMaxForce`.
