# Dynamic Grab Motion-Frame Authority Fix - 2026-05-15

## Superseded

This note is superseded by
`docs/grab-pivot-b-body-frame-correction-2026-05-15.md`.

The pivot frame split overlay proved that using MOTION as the plain
`transformB`/Pivot-B frame separates the solver pivot from the visible object
contact point. The corrected rule is BODY for object-side constraint Pivot B;
MOTION remains mass/COM/diagnostic data.

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
- Superseded: do not change the active drive object frame to
  `tryResolveLiveBodyWorldTransform` for PivotB authority.
- At grab capture:
  - visual relation and object/node local data stay BODY-authored;
  - constraint PivotB remains BODY-authored;
  - active transformB is computed from the paired body-A palm/body-B relation.
- After-solve telemetry still reports both BODY and MOTION so future logs cannot
  hide the diagnostic split again, but MOTION is not pivot authority.

## Expected Runtime Result

- `nativeBody[BODY]` and `heldBody[MOTION]` may still differ by about 90 degrees.
  That is allowed as a diagnostic, not a failure by itself.
- The object target error measured against the drive/BODY frame should drop.
- The contact pivot should remain non-COM because pivot B is still frozen from
  the selected contact point.
- Wrist/object rotation should stop inheriting the BODY/MOTION basis mismatch.

## Implementation Notes

- `tryGetGrabAuthorityBodyWorldTransform` remains the BODY/visual helper.
- `tryGetGrabDriveObjectWorldTransform` now reads the BODY helper again because
  the split pivot overlay proved that `transformB`/Pivot B belongs to the
  visible object BODY frame in this constraint path.
- Grab capture forces `constraintBodyWorldAtGrab` to the BODY readback. The
  selected world contact point is frozen into BODY space so solver Pivot B and
  visual Pivot B remain the same material point.
- Debug/proxy readback paths now use live proxy readback for constraint anchors.
- After-solve telemetry still reports MOTION readback separately as
  `motionObjectRead`/`motionToDrive`, so future tests can see BODY/MOTION drift
  without allowing MOTION to become pivot authority.

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
