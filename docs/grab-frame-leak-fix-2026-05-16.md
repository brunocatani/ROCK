# Dynamic Grab Frame Leak Fix - 2026-05-16

## Current State

This note tracks the code review finding from the solver-frame split pass.

The current dynamic grab architecture intentionally has two object-side frames:

- visible/contact BODY frame:
  - owns mesh/contact evidence;
  - owns the material grip point selected from the object;
  - must feed visual/debug/body-zone callers that ask for the held grab point.
- solver/live MOTION frame:
  - owns hknp custom constraint body-B reads when FO4VR exposes a motion frame;
  - may re-express the same selected material point in solver-local coordinates;
  - must not become user-facing grip authority.

COM remains weight/inertia data only. It is not a grip fallback.

## Review Finding

The latest solver split fixed the on-grab BODY/MOTION angular mismatch, but it
left two frame leaks:

1. `tryGetHeldObjectGrabPivotWorld(...)` is a public held-object pivot API, but
   it was returning the solver/MOTION pivot via `tryGetGrabDriveObjectWorldTransform`.
   Its current caller is shoulder stash probing, which wants the held material
   point, not a solver diagnostic point.

2. `GrabPivotDebugSnapshot::objectPivotWorld` was aliased to the solver pivot.
   Older telemetry still labels that value as `pivotFrame=NATIVE_BODY`, so logs
   and simple debug overlays could appear to show BODY/contact data while
   actually drawing solver data.

The initial review also flagged body-A local PivotA as a possible mixed-frame
capture. Source recheck found the create path already builds the proxy body from
`makeRawRotationPalmTranslationFrame(...)` before computing PivotA local. The
fix should still make that invariant explicit so future refactors cannot regress
it, but the behavioral frame leak is in the public/debug pivot APIs above.

## Fix Plan

- Keep `_grabFrame.pivotBBodyLocalGame` as the visible BODY material grip point.
- Keep `_grabFrame.pivotBConstraintLocalGame` / `_grabAuthorityPivotBConstraintLocalGame`
  as solver-local constraint data.
- Make `tryGetHeldObjectGrabPivotWorld(...)` return visible BODY PivotB.
- Make `GrabPivotDebugSnapshot::objectPivotWorld` mean visible BODY PivotB when
  available, with solver fallback only if BODY readback fails.
- Keep explicit `visualObjectPivotWorld` and `solverObjectPivotWorld` fields for
  the frame split overlay.
- Rename comments/log labels so simple telemetry no longer says native BODY while
  showing solver data.
- Add source-boundary tests so future solver-frame work cannot leak MOTION into
  the public visible pivot path again.

## Expected Result

This should not change the selected grip point, COM policy, or angular motor
target math. It removes misleading public/debug data flow and keeps solver-only
frame handling contained to custom constraint authority.

## Implemented In This Pass

- `tryGetHeldObjectGrabPivotWorld(...)` now returns the visible BODY/contact
  PivotB through `_grabFrame.pivotBBodyLocalGame`.
- `GrabPivotDebugSnapshot::objectPivotWorld` now aliases visible BODY PivotB
  when BODY readback is available. The solver pivot remains available through
  `solverObjectPivotWorld`.
- Simple telemetry now labels the legacy pivot field as `VISIBLE_BODY`.
- `createProxyConstraintGrabDrive(...)` now names its input
  `palmProxyWorldTransform` and documents that all body-A local values are
  captured against the raw-rotation proxy body frame.
- Source-boundary tests now reject solver/MOTION leakage into the public held
  pivot helper.
