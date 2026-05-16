# Grab Pivot Frame Split Debug - 2026-05-15

## Why This Exists

The held-object yellow cross was ambiguous after the dynamic grab frame change.
It was drawn from the solver/constraint Pivot B path, while the visible object
is still inspected through the native BODY/scene-object frame. If BODY and
MOTION differ, one yellow marker cannot prove whether the original visible grip
point is wrong or the solver-local pivot is wrong.

## New Debug Toggle

- `bDebugShowGrabPivotFrameSplit`
- Default: `false`
- Active test INI: enabled for the next runtime pass.

## What It Draws

- Pivot A: hand/proxy-side constraint pivot.
- Visual Pivot B: selected object grip point transformed through native BODY
  readback.
- Solver Pivot B: actual constraint `transformB` point transformed through the
  active grab drive frame.
- Red line: Pivot A to solver Pivot B, the actual constraint error.
- Orange line: visual Pivot B to solver Pivot B, the BODY-vs-solver frame split.

## How To Read It

- Visual Pivot B correct, solver Pivot B wrong:
  the object-side constraint frame is wrong. The 2026-05-15 tray screenshots
  confirmed that using MOTION/COM for `transformB` creates this exact split;
  corrected production logic keeps Pivot B in BODY space.
- Visual Pivot B wrong:
  contact/pivot capture is wrong before the solver gets involved.
- Visual and solver Pivot B overlap, but the held object still rotates wrong:
  pivot placement is not the remaining cause; angular target/body-pair math is.

## Scope

- Debug-only overlay and config.
- No grab physics behavior changed.
- No collider conventions changed.
- No force tuning changed.
