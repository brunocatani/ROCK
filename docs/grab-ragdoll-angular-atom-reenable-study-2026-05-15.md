# Ragdoll Angular Atom Re-enable Study - 2026-05-15

This document records the current research state before re-enabling the custom
grab ragdoll angular atom. The goal is not to restore the old broken path. The
goal is to restore solver-owned angular authority after the later rotation
reference fixes made the frame model sane.

## Scope

- Dynamic grab only.
- Ordinary one-hand loose objects are in scope.
- Loose non-equipped weapons are in scope as complex/multipart dynamic objects.
- Equipped weapon/two-hand weapon handling is out of scope.
- Actor ragdoll grab remains unchanged unless a shared helper must be protected.
- Missing loose-weapon generated collider creation/activation is explicitly out
  of scope for this pass.

## Commit Markers

- `db1a996 fix/grab: replace ragdoll angular atom authority`
  - Disabled the type-19 ragdoll angular atom.
  - Kept the three linear motor atoms active.
  - Added direct FO4VR angular velocity authority via
    `applyProxyConstraintAngularVelocityDrive(...)`.
- `32ebeae fix/grab: split proxy angular reference authority`
  - Split the hidden proxy/palm collider frame from angular reference authority.
- `716fcb1 fix/grab: use raw hand rotation for proxy authority`
  - Main fix for the hand/controller rotation relationship.
  - Production angular reference became `rawRotationPalmTranslation`.
  - Meaning: linear pivot uses live palm/proxy translation, while angular intent
    uses the raw root-flattened hand/controller rotation.
- `7376a6c fix/grab: use native angular velocity convention`
  - Later corrected the direct angular velocity boundary.
  - This did not make direct angular velocity equivalent to solver torque.
- `1185d43 feature/grab: scale held angular cap for long objects`
  - Added the current long-object angular speed cap heuristic.
- `9adf5e8 fix/grab: apply angular authority to held body set`
  - Applied direct angular velocity to all accepted unique held motions instead
    of only the primary body.

## Current ROCK Dynamic Grab Authority

Current one-hand dynamic grab uses a hidden no-contact proxy body as body A and
the held object's selected body as body B.

Current frame split:

- body A physical/proxy frame:
  - sourced from the live generated palm anchor body;
  - follows the same movement/convention as generated hand colliders;
  - is the correct linear anchor/pivot source;
  - is not the desired angular reference.
- angular authority frame:
  - produced by `makeRawRotationPalmTranslationFrame(rawHandWorld,
    palmProxyWorld)`;
  - translation comes from the live palm/proxy body;
  - rotation comes from the raw root-flattened hand/controller frame;
  - this is the validated fix for wrist axis and N/S/E/W dependency.

Important current code:

- `src/physics-interaction/hand/HandGrab.cpp`
  - `makeRawRotationPalmTranslationFrame(...)`
  - `createProxyConstraintGrabDrive(...)`
  - `updateProxyConstraintGrabDriveTarget(...)`
  - `applyProxyConstraintAngularVelocityDrive(...)`
  - `flushPendingCustomGrabAuthority(...)`
- `src/physics-interaction/grab/GrabConstraint.cpp`
  - `setGrabMotorAtomsActive(header, true, false)`
  - angular motor exists and is tuned, but the ragdoll atom is disabled.
- `src/physics-interaction/grab/GrabConstraintMath.h`
  - `writeInitialGrabAngularFrame(...)` writes transform-B rotation and
    `target_bRca` from the inverse body-in-hand relation.

Current production behavior:

- Linear authority is solver-owned by the custom constraint linear motor atoms.
- Angular authority is not solver-owned.
- Angular authority is a direct hknp angular velocity write to the accepted
  held body set.
- The direct write is finite/capped by the same angular motor budget, but it is
  still a velocity command, not a torque/constraint solve.

## What HIGGS Actually Does For Dynamic Angular Grab

HIGGS uses one custom constraint containing:

- one set-local-transforms atom;
- one ragdoll motor atom for angular authority;
- three linear motor atoms.

Relevant HIGGS source:

- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/constraint.cpp`
  - `GrabConstraintData::GrabConstraintData()`
  - `GrabConstraintData::setMotorsActive(...)`
  - `GrabConstraintData::setTargetRelativeOrientationOfBodies(...)`
- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/RE/havok.cpp`
  - `CreateGrabConstraint(...)`
- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/hand.cpp`
  - grab creation around `CreateGrabConstraint(...)`
  - held update around `constraintData->setTargetRelativeOrientationOfBodies(...)`
  - held update motor tuning around linear/angular max force and tau.

HIGGS creation flow:

1. Compute palm-side pivot A in hand body space.
2. Compute selected contact/grip pivot B in object body space.
3. Compute desired rigid body in hand space.
4. Compute hand transform in object space as the inverse desired relation.
5. Create the custom constraint with transform A and transform B.
6. Enable ragdoll angular motor plus all three linear motors before insertion.

HIGGS held update flow:

1. Compute `desiredTransformHandSpace`.
2. Invert it to get the hand frame in grabbed-object space.
3. Write ragdoll angular target with
   `setTargetRelativeOrientationOfBodies(...)`.
4. Recompute transform-B translation from the palm point transformed into the
   object-side frame.
5. Update linear/angular motor strength, tau, damping, recovery, collision
   response, fade, and mass caps.

Important HIGGS detail:

- HIGGS updates `target_bRca` every held update.
- HIGGS updates transform-B translation every held update.
- HIGGS does not appear to rewrite transform-B rotation every held update in the
  production held path; transform-B rotation is established at creation while
  the angular target carries the changing desired relative orientation.

## How HIGGS Handles Long Objects

HIGGS does not have a separate explicit "long object angular scale" helper like
ROCK's `computeLongObjectAngularSpeedScale(...)`.

HIGGS long-object feel comes from the whole solver model:

- the grip pivot is off-COM and selected from contact/object space;
- body B is solved through a finite ragdoll angular motor, not direct velocity;
- angular force is derived from linear force by
  `grabConstraintAngularToLinearForceRatio`;
- linear force is capped by mass through `grabConstraintMaxForceToMassRatio`;
- object inertia is normalized/clamped so impossible inertia ratios do not make
  the object uncontrollable;
- the object's actual Havok inertia and the grip-to-COM lever affect how much
  torque is needed to rotate it;
- visual hand deviation is allowed and measured; the hand can lag/separate
  rather than making the player feel infinitely strong;
- connected bodies are registered for mass/player-space/inertia handling.

HIGGS also contains release-time COM/tangential velocity logic. That is release
behavior, not held-pivot authority.

HIGGS has a commented experiment for computing inertia around a rotation axis on
release, but the held dynamic grab path does not use a custom long-object
inertia formula. It delegates the held rotational feel to the constraint solver,
body inertia, finite force, and off-COM pivot.

## What ROCK Has For Long Objects Now

ROCK currently captures a lever proxy:

- `_grabFrame.longObjectLeverGameUnits =
  computeLocalMeshMaxDistanceFromPoint(_grabFrame.localMeshTriangles,
  _grabFrame.gripPointLocal) * objectScaleForLever`

ROCK currently applies that only to direct angular velocity:

- `computeLongObjectAngularSpeedScale(...)`
- configured by:
  - `bGrabLongObjectAngularScalingEnabled`
  - `fGrabLongObjectReferenceLeverGameUnits`
  - `fGrabLongObjectMinAngularScale`

This is a cap on angular speed. It is useful for the current direct-angular
path, but it is not true rotational inertia and not equivalent to HIGGS. It
does not let the solver decide torque from inertia and lever arm; it clamps a
velocity vector after computing the desired angular correction.

## Why ROCK Does Not Yet Have HIGGS Long-Object Feel

ROCK lacks full HIGGS-style long-object feel because angular authority is direct
velocity, not solver torque.

Direct angular velocity does these things well:

- avoids relying on a previously suspect FO4VR ragdoll atom equation;
- can be clamped by mass budget;
- can be written to the accepted held body set;
- uses the now-validated raw hand/controller rotation reference.

But direct angular velocity loses these HIGGS properties:

- no true torque competition between hand and object inertia;
- no solver impulse history for angular correction;
- no natural grip-to-COM resistance from an off-COM constraint;
- no constraint-level angular violation that the solver can satisfy gradually;
- long-object behavior needs an artificial speed cap instead of emerging from
  inertia and finite torque;
- direct writes can make the object feel too obedient when the cap is high, or
  too damped when the cap is low.

This is why the current long-object solution is only a heuristic.

## Why The Old Ragdoll Atom Path Failed

The failure that led to `db1a996` was not only "ragdoll atom bad."

At that time ROCK still had wrong angular references:

- hidden/proxy collider rotation was being treated as the angular frame;
- BODY/MOTION/visual frame assumptions were mixed;
- transform-B and `target_bRca` writes were being debugged while the angular
  reference itself was wrong;
- the visual hand/object relation could be correct in one frame path while the
  solver angular path consumed another.

After `32ebeae` and `716fcb1`, the frame model changed:

- palm/proxy collider frame is linear authority only;
- raw root-flattened hand/controller frame is angular authority;
- `rawRotationPalmTranslation` is the production hand/object angular reference.

Therefore re-enabling the ragdoll atom must be studied against the current split
frame model, not the old db1a996-era model.

## Critical Re-enable Rule

There must be one angular authority.

Do not enable the ragdoll angular atom while also keeping
`applyProxyConstraintAngularVelocityDrive(...)` as production held-object
authority. That would give the same held object both:

- solver torque from the ragdoll motor atom; and
- direct per-frame angular velocity writes.

Those two writers can fight, hide each other's bugs, and produce misleading
telemetry.

The clean re-enable direction is:

1. Keep the proxy linear constraint authority.
2. Enable the ragdoll angular atom.
3. Feed the ragdoll angular target from the validated raw hand/controller
   rotation relation.
4. Remove or fully bypass production direct angular velocity writes while the
   ragdoll atom owns rotation.
5. Keep direct angular velocity only for release velocity and diagnostic-only
   code if explicitly needed.

## Required Frame Model For Re-enable

The re-enabled atom must keep the current split:

```text
               linear authority                       angular authority
               ----------------                       -----------------

physical hand  generated palm collider/proxy          raw hand/controller
reference      body A transform/translation           root-flattened rotation

object target  selected grip pivot in BODY space      object desired rotation
                                                       relative to raw hand frame

COM            mass/inertia/release data only         never pivot authority
```

The likely required structure is:

- transform A translation:
  - remains the palm/pivot A in proxy-body local space.
- transform A rotation:
  - must represent the raw angular frame relative to the proxy physical body if
    FO4VR's type-19 atom consumes transform-A rotation the same way HIGGS hkp
    does.
  - it must not simply be identity if proxy physical rotation differs from raw
    hand/controller rotation.
- transform B translation:
  - remains the selected object-side grip pivot in body-B local space.
- transform B rotation:
  - should be separated from target updates.
  - HIGGS establishes it at creation and does not rewrite it every held update.
- `target_bRca`:
  - should carry the current desired angular relation each physics update.
  - it must be computed from `rawRotationPalmTranslation`, not from the hidden
    collider/proxy rotation alone.

This means `writeInitialGrabAngularFrame(...)` is probably too coarse for the
re-enable architecture because it writes transform-B rotation and target
together. The code needs separate concepts:

- initial/frozen constraint angular frame setup;
- per-frame ragdoll angular target update;
- per-frame transform-B translation update.

No code change should be made until the exact FO4VR type-19 target equation is
verified or proven by controlled telemetry.

## Ghidra Verification Still Needed Before Code

Source comparison is enough to identify the architecture, but it is not enough
to safely flip the angular atom back on.

Before implementation, verify in FO4VR binary/Ghidra:

1. Whether hknp type-19 ragdoll motor consumes `target_bRca` with the same
   effective relation as HIGGS hkp:
   - HIGGS source does `target_bRca = bRa * transformA.rotation`.
   - ROCK must know whether FO4VR hknp does the same, inverts it, or composes it
     through body B differently.
2. Whether transform-A rotation participates in the target relation.
   - This decides whether the raw-hand-vs-proxy rotation adapter belongs in
     transform A, `target_bRca`, or both.
3. Whether rewriting transform-B rotation every frame resets or destabilizes the
   atom runtime.
   - HIGGS only refreshes target and transform-B translation during held update.
4. Whether enabling the angular atom with current runtime sizes/results still
   matches FO4VR's native constraint info utility when all four motor atoms are
   active.
5. Whether the atom must be enabled before constraint insertion, after insertion,
   or with runtime clear semantics.

## Implementation Plan After Verification

This is not implementation yet. This is the source-derived plan that must be
confirmed before code.

1. Split angular constraint writers.
   - Add separate helpers for:
     - initial transform-B angular frame setup;
     - per-frame `target_bRca` update;
     - per-frame transform-B translation update.
   - Stop using one helper that rewrites transform-B rotation and target every
     held update unless Ghidra proves that is correct for FO4VR.

2. Add a raw-angular-frame adapter.
   - Body A remains the hidden palm proxy for linear movement.
   - Angular target must use the raw hand/controller rotation.
   - If FO4VR hknp uses transform-A rotation, encode raw-hand relative to proxy
     there.
   - If FO4VR hknp ignores transform-A rotation or composes differently, encode
     the equivalent correction in `target_bRca`.

3. Enable the angular atom as the single angular writer.
   - Change the atom activation policy from linear-only to linear+angular.
   - Remove production direct held angular velocity writes from the between
     collide/solve flush path while the atom is active.

4. Keep motor budget logic.
   - `updateConstraintGrabDriveMotors(...)` already computes linear and angular
     finite budgets.
   - Keep aggregate held-body mass budget.
   - Keep collision tau and fade behavior.
   - Keep loose weapon max-force multiplier before mass cap.

5. Revisit long-object logic.
   - With solver-owned angular torque, the direct angular speed cap should not
     be the primary long-object behavior.
   - If extra long-object tuning is still needed, prefer angular max-force/torque
     scaling from lever length over direct velocity clamping.
   - Do not move pivot to COM.
   - Do not compute pivot from COM.

6. Update tests.
   - Tests that currently require `ComputeHardKeyFrame_t` as held-object angular
     authority must be replaced.
   - Tests must instead require:
     - angular atom enabled for dynamic grab;
     - direct angular velocity not used as production held authority;
     - raw hand/controller rotation feeds the angular target;
     - hidden proxy collider frame remains linear only;
     - COM is not used as pivot authority.

## Expected Benefit

If implemented correctly, re-enabling the ragdoll angular atom should improve:

- long-object feel;
- heavy-object swing resistance;
- grip-to-COM torque behavior;
- solver-consistent angular lag;
- collision-aware held stability;
- object feel under finite hand strength.

It should also reduce the need for the current long-object angular speed cap
because the object would again resist rotation through solver torque and inertia
instead of accepting a capped velocity command.

## Confirmed Facts

- Current branch is `feature/ghidra-grab-motor-mapping`.
- Existing unpushed commits were pushed before this study.
- `db1a996` is the commit that disabled the ragdoll angular atom and added
  direct angular velocity authority.
- `716fcb1` is the main commit that fixed the validated rotation reference by
  moving angular authority to raw hand/controller rotation.
- Current ROCK ragdoll angular motor object is still created and tuned, but the
  atom is disabled.
- Current ROCK long-object logic is an angular speed cap, not true inertia.
- HIGGS does not have ROCK's explicit long-object speed cap helper.
- HIGGS uses finite solver angular motors, mass caps, angular/linear force ratio,
  inertia normalization, collision tau, and hand deviation to make long/heavy
  objects feel weighted.

## Open Questions

- Exact FO4VR hknp type-19 ragdoll motor target equation.
- Whether transform-A rotation is the correct place to bridge proxy physical
  rotation to raw hand/controller angular reference.
- Whether transform-B rotation must be frozen after creation like HIGGS or can
  be safely rewritten every physics step in FO4VR hknp.
- Whether one primary-body angular constraint is sufficient for every multipart
  loose weapon once missing weapon collider/activation work is fixed, or whether
  some FO4VR objects expose disconnected motions that need a separate future
  body-set strategy.

