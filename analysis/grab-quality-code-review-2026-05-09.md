# ROCK Grab Quality Code Review - 2026-05-09

Scope: adaptive held response, controller-derived release velocity, native mouse-spring target shaping, and alternate thumb local-transform safety added around the current ROCK grab path.

## Findings

1. Release-frame controller motion is not sampled on normal grip release.
   - `PhysicsInteraction::update` calls `releaseGrabbedObject` immediately when `grabInput.released` is true.
   - The new hand velocity and angular velocity histories are sampled only inside `Hand::updateHeldObject`, which is skipped on that release frame.
   - Result: throws can use the previous frame's controller velocity instead of the release peak.

2. `fGrabThrowMaxVelocityHavok` does not cap the final release velocity.
   - `composeControllerReleaseVelocity` clamps local velocity before applying `fThrowVelocityMultiplier`, then adds player-space velocity.
   - Result: the configured max can be exceeded by the throw multiplier and player velocity.

3. Release angular velocity is written directly from controller angular velocity without a clamp or object-inertia policy.
   - `releaseGrabbedObject` passes `handAngularVelocity` into `setHeldVelocity` with `overrideAngularVelocity = true`.
   - Result: fast wrist/controller rotation can spin the held object beyond a stable object-specific angular response.

4. Angular/tangential release uses the hand origin rather than the actual grab pivot.
   - `computeTangentialVelocityFromAngularSwing` receives `_lastHeldHandPositionHavok`.
   - ROCK already has frozen grip/pivot data; the release lever arm should be the held object's grip point or active pivot, not necessarily the raw hand node.

5. Alternate thumb surface safety does not know the real alternate-curve hit point.
   - `FingerCurlValue` records distance/value/kind but not the chosen intersection point or triangle normal.
   - `surfaceAimTarget[0]` falls back to the shared finger target for curve hits, so the guard can test the wrong surface plane.

6. Alternate thumb plane correction can report success even when no segment transform was actually written.
   - `applyAlternateThumbPlaneCorrection` returns true if it saw any usable segment, even if every proposed transform was skipped.

## Verification Context

Previous implementation verification passed:

- `ctest --test-dir build -C Release --output-on-failure`: 55/55 passed.
- Release build with `/m:1` passed and deployed `ROCK.dll`/`ROCK.pdb`.

These tests do not currently cover release-frame sampling, final velocity capping after multiplier, angular velocity clamp policy, or real alternate-thumb hit-point propagation.
