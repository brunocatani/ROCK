# Grab Pivot B BODY Frame Correction - 2026-05-15

## Why This Exists

The pivot frame split overlay showed the important fact directly:

- visual/BODY Pivot B stayed near the visible grip/contact point;
- solver/MOTION Pivot B appeared far away on the tray;
- the orange split line between them was large.

That means the previous live-MOTION `transformB` assumption was wrong for the
object-side constraint pivot. `transformB` must be treated as BODY-local in this
FO4VR hknp constraint path. COM/MOTION remains useful for mass, inertia, and
diagnostics, but not for the grab pivot.

## Correction

- `tryGetGrabDriveObjectWorldTransform` returns the BODY helper again.
- Grab capture forces `constraintUsesMotionBodyAtGrab = false`.
- `constraintBodyWorldAtGrab` is the BODY readback.
- The visual/contact copy of Pivot B remains the selected contact point in BODY
  space.
- The active constraint `transformB` copy is computed in BODY space from the
  paired body-A palm/body-B relation, matching the HIGGS held-update formula.
- After-solve telemetry still reads MOTION separately as `motionObjectRead` and
  logs `motionToDrive`, so BODY/MOTION drift remains visible without becoming
  authority.

## Expected Overlay Result

- Green visual Pivot B and yellow solver Pivot B should overlap or be very close.
- Orange visual-to-solver split line should be near zero.
- Red PivotA-to-solver PivotB line remains the actual constraint error.

## Scope

- Dynamic loose held-object grab pivot path only.
- No COM pivot.
- No visual hand change.
- No collider convention change.
- No force tuning.
