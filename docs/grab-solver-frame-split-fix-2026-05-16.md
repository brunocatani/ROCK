# Grab Solver Frame Split Fix - 2026-05-16

## Why This Exists

The 2026-05-16 logs after commit `f6eff85` proved the paired transform-B update
was no longer the failing layer:

- `targetVsTransformB=0.000deg`
- `transformBErr=0.000gu`
- `rawProxy=0.00deg`
- `proxyTargetErr=0.000gu/0.00deg`

The remaining failure was on body B. The object stayed tens of game units and
roughly 80-170 degrees away from the desired frame while the proxy was exact.
Every bad sample still showed a fixed BODY/MOTION split:

- `motionToDrive=0.83gu/90.0deg`
- `nativeToHeld=90.000deg`

That means BODY-only constraint bytes preserve the visible contact point but
feed the hknp solver's angular rows through the wrong body-B frame.

## Current Rule

- BODY is still visual/contact authority.
- Pivot selection is still mesh/contact/authored evidence.
- COM is still weight, inertia, release, and diagnostic data only.
- Solver body-B data uses MOTION when FO4VR exposes it through
  `tryResolveLiveBodyWorldTransform`.
- The selected contact point is re-expressed in the solver frame; it is not
  replaced by COM.

## Code Shape

- `tryGetGrabAuthorityBodyWorldTransform(...)`
  - BODY helper.
  - Used for visible object relations, mesh/contact debug, visual PivotB, and
    scene-node reconstruction.
- `Hand::tryGetGrabDriveObjectWorldTransform(...)`
  - solver-facing helper.
  - Uses the live resolved body frame, which prefers MOTION when available.
  - Falls back to BODY only when no live solver frame exists.
- Grab capture:
  - `grabBodyWorldAtGrab` is BODY.
  - `constraintBodyWorldAtGrab` is MOTION when available, otherwise BODY.
  - `_grabFrame.pivotBBodyLocalGame` remains BODY-local selected grip point.
  - `_grabFrame.pivotBConstraintLocalGame` is recomputed from the paired
    body-A/body-B relation after the split frame is captured.

## Expected Runtime Evidence

After this build, `PROXY GRAB AFTER_SOLVE` should show:

- `objectSrc=MOTION` for normal dynamic props when MOTION is readable.
- `motionToDrive` near zero because drive and motion readback should be the same
  frame.
- `objectTargetErr` rotation should drop from the 80-170 degree range if this is
  the correct solver convention.
- The visible BODY pivot debug may still differ from the solver pivot marker;
  that is expected if the overlay draws both frames. The visible grip point must
  remain contact/BODY based.

## What This Does Not Change

- no COM fallback;
- no object-origin fallback;
- no mouse-spring authority;
- no hand collider convention change;
- no equipped weapon behavior change;
- no actor ragdoll grab change.
