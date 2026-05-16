# Dynamic Grab Motion-Frame Authority Fix - 2026-05-15

## Why This Change Exists

The relation sweep showed that the hidden palm/proxy frame tracks correctly, but
the held object remains tens to more than one hundred degrees away from every
BODY-authored angular candidate. The same logs show the object has two live
frames:

- native `BODY`, used by the scene collision object and visual node relation;
- live `MOTION`, used by FO4VR hknp as the dynamic solver frame.

The failing grab was freezing transform-B, pivot-B, and angular relation in
`BODY` even when runtime telemetry reported the held object authority as
`MOTION`. That mixes object-side frames. The fix is not to use COM as the grip
pivot. The fix is to keep the contact pivot, but express that selected contact
point in the same object-side frame the constraint solver consumes.

## Scope

- Dynamic loose one-hand grab only.
- No collider convention changes.
- No visual hand changes.
- No force tuning.
- No weapon collider creation.
- No actor ragdoll/equipped weapon logic changes.

## Implementation Shape

- Keep `tryGetGrabAuthorityBodyWorldTransform` as the native BODY/visual helper.
- Change the active drive object frame to `tryResolveLiveBodyWorldTransform`,
  which prefers MOTION when FO4VR exposes it.
- At grab capture:
  - visual relation and object/node local data stay BODY-authored;
  - constraint relation uses MOTION when available;
  - pivot B is still the selected contact point, frozen into the constraint
    frame.
- At held update and release, any use of `pivotBConstraintLocalGame` must read
  the live drive object frame, not the BODY visual frame.
- After-solve telemetry must report both native BODY and drive/MOTION so future
  logs cannot hide this mismatch again.

## Expected Runtime Result

- `nativeBody[BODY]` and `heldBody[MOTION]` may still differ by about 90 degrees.
  That is allowed as a diagnostic, not a failure by itself.
- The object target error measured against the drive/MOTION frame should drop.
- The contact pivot should remain non-COM because pivot B is still frozen from
  the selected contact point.
- Wrist/object rotation should stop inheriting the BODY/MOTION basis mismatch.

## Implementation Notes

- `tryGetGrabAuthorityBodyWorldTransform` remains the BODY/visual helper.
- `tryGetGrabDriveObjectWorldTransform` now reads `tryResolveLiveBodyWorldTransform`
  so runtime pivot/error/release math uses the same body-B frame as the solver.
- Grab capture chooses `constraintBodyWorldAtGrab` from the live MOTION-backed
  solver frame when available, otherwise BODY. The selected world contact point
  is then frozen into that chosen frame.
- Debug/proxy readback paths now use live proxy readback for constraint anchors.
- After-solve telemetry now reports both solver object readback and native BODY
  readback with `nativeToDrive`, so future tests can see the split directly.

## Non-Goals

- This does not move the pivot to COM.
- This does not change collider placement conventions.
- This does not tune forces or long-object behavior.
- This does not touch equipped weapons, two-hand equipped weapon handling, actor
  ragdoll grab, or loose weapon collider creation.

## Verification

- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m`
- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure`

All source-boundary tests passed after the frame-authority guardrail updates.
