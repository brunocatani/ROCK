# Grab Pivot B BODY Frame Correction - 2026-05-15

## Why This Exists

The pivot frame split overlay showed the important fact directly:

- visual/BODY Pivot B stayed near the visible grip/contact point;
- solver/MOTION Pivot B appeared far away on the tray;
- the orange split line between them was large.

That showed the previous live-MOTION `transformB` attempt was wrong because it
let the solver frame become the visible grip authority. The later logs showed
the opposite failure: BODY-only constraint data keeps the visible point sane,
but leaves the hknp angular rows with a persistent 90-degree BODY/MOTION split.

The current rule is split authority:

- BODY remains the visual/contact frame and owns the selected material grip
  point.
- MOTION is used only as the solver-facing body-B frame when FO4VR exposes it.
- The same selected contact point is re-expressed in the solver frame for
  constraint bytes.
- COM is still not a fallback grip point and not a visual pivot.

## Correction

- `tryGetGrabDriveObjectWorldTransform` reads the solver-facing live body frame
  and falls back to BODY only if MOTION is unavailable.
- Grab capture sets `constraintUsesMotionBodyAtGrab` from whether
  `tryResolveLiveBodyWorldTransform` returned `MotionCenterOfMass`.
- `constraintBodyWorldAtGrab` is the MOTION readback for solver rows and the
  BODY readback otherwise.
- The visual/contact copy of Pivot B remains the selected contact point in BODY
  space.
- The active constraint `transformB` copy is computed in the solver body-B frame
  from the paired body-A palm/body-B relation, matching the HIGGS held-update
  formula without moving the visual pivot to COM.
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
