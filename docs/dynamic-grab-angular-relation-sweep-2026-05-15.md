# Dynamic Grab Angular Relation Sweep - 2026-05-15

## Purpose

The palm/proxy overlay and logs show the hidden authority proxy is tracking the
palm correctly, while the held object angular error can stay around 130-145
degrees. That points to the object angular constraint relation, not visual hand
attachment and not palm proxy placement.

This pass is diagnostic-only. It does not change motor behavior, pivot
selection, collider conventions, or target writes.

## What Was Added

- `GRAB ANGULAR RELSWEEP` after-solve telemetry in
  `src/physics-interaction/hand/HandGrab.cpp`.
- The observer copies `transformA`, `transformB`, and `target_bRca` bytes under
  the existing grab authority lock.
- After the solver runs, it compares the live object/proxy body-pair rotation
  against multiple plausible interpretations of the ragdoll angular target.

## Relations Being Compared

- `currentUndoA`: assumes current write convention
  `target_bRca = bodyToProxy * transformA`.
- `preUndoA`: assumes the same operands but opposite multiply order
  `target_bRca = transformA * bodyToProxy`.
- `targetNoA`: assumes `target_bRca` directly stores the body-pair target.
- `targetInv`: assumes the body-pair direction is inverted.
- `rowsUndoA`: assumes target bytes are being interpreted as rows, not Havok
  columns, before undoing `transformA`.
- `transformB`: compares the stored transform-B rotation as the body-pair
  target.
- `capturedRaw`: compares the raw hand/body captured relation.

Each candidate is scored against:

- `proxyErr`: live object body relative to the hidden proxy body;
- `authorityErr`: live object body relative to the raw-rotation/palm-translation
  authority frame.

## How To Read The Log

Look for:

```text
GRAB ANGULAR RELSWEEP ... bestProxy=<label>/<deg> bestAuthority=<label>/<deg>
```

Meaning:

- A stable low `bestProxy` means the FO4VR solver is consuming a proxy/body-pair
  relation matching that candidate.
- A stable low `bestAuthority` means the effective angular relation is closer to
  the raw-rotation authority frame than to the proxy body frame.
- If `targetInv` wins, the ragdoll atom body-pair direction is likely reversed.
- If `preUndoA` wins, `transformA` composition order is likely wrong.
- If `rowsUndoA` wins, byte/storage interpretation is likely wrong.
- If no candidate gets close, the custom ragdoll atom layout or FO4VR motor
  consumption path needs Ghidra verification.

## Expected Test

- Grab one object.
- Hold still for one second.
- Rotate wrist pitch/yaw/roll separately.
- Rotate player direction and repeat.

The important question is whether the winning relation changes with player
world direction. If it does, some angular path is still world-direction
dependent.

