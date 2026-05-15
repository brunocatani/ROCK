# Dynamic Grab Rotation Reference Code Review - 2026-05-14

## Current Production Result

The validated dynamic loose-object grab authority is `rawRotationPalmTranslation`.
This is now the only production rotation reference.

Production rule:

- the hidden no-contact proxy follows the live palm/generated-collider anchor
  for translation;
- the held object rotation follows the root-flattened raw hand/controller frame;
- the selected object-side grip point remains frozen in BODY-local space;
- COM/MOTION data is weight, inertia, and diagnostic data only;
- COM/MOTION is never pivot, grip-frame, target-frame, or orientation authority.

The older selectable rotation references and the former native grab action path
were removed after runtime testing showed the raw hand rotation plus palm/proxy
translation convention fixed the wrist-axis and N/S/E/W world-direction issue.

## Evidence That Drove The Fix

Runtime logs showed the palm/proxy side tracking correctly while the held object
rotation remained world-direction dependent. The visual generated colliders were
not the bug. The failing relationship was the object angular reference used by
the held-body drive.

The useful pattern was:

- proxy target error stayed near zero;
- the object stayed tied to a BODY/proxy angular relation rather than the raw
  controller/root-hand relation;
- rotating the player shifted the axis interpretation, which proved the object
  rotation was being derived from the wrong reference frame;
- COM/MOTION diagnostics were useful for mass/weight reasoning but invalid as
  grip authority.

## Code Contract

Current code must preserve these boundaries:

- `makeRawRotationPalmTranslationFrame` is the only production angular reference
  builder for dynamic loose-object grab.
- `tryGetGrabAuthorityBodyWorldTransform` reads the held object BODY frame for
  grip pivot and visual BODY relation capture.
- `updateProxyConstraintGrabDriveTarget` composes desired object/body targets
  from the raw-rotation/palm-translation authority frame.
- `resolveProxyConstraintAngularDriveTargetWorld` applies the FO4VR angular
  boundary conversion to that selected visual target before direct angular
  velocity is computed.
- `flushPendingCustomGrabAuthority` is the only dynamic held-object physics-step
  authority flush.
- No runtime config may switch back to alternate rotation references.
- No native grab action wrapper, tuning config, or held-update flush may remain
  as fallback.

## Validation Notes

The commit that fixed the runtime behavior was:

- `716fcb1 fix/grab: use raw hand rotation for proxy authority`

The cleanup following that fix removes the old runtime paths so future users and
future agents cannot accidentally select the wrong authority model.
