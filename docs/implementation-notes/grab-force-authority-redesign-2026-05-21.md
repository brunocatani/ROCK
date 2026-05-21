# Grab Force Authority Redesign - 2026-05-21

Project: ROCK

Source authority: current local ROCK source and user-approved local HIGGS comparison. No web, Ghidra, FO4 Mods MCP, or external sources used.

Confidence: implementation-level finding from source review, policy tests, and Release build validation.

## Problem

Live testing shows long and tiny objects still feel wrong:

- long objects with a thin/long discovered patch feel sluggish under lever load;
- tiny objects such as cigarettes can get too little force because the discovered patch is small or single-point;
- some grabs rotate around one found point because ROCK has pivot evidence but not enough orientation/support evidence;
- adaptive motor behavior is using unreliable patch/contact evidence to decide live authority.

The core design bug is coupling three different concerns:

1. pivot selection: where the object is grabbed;
2. support evidence: whether the contact/mesh evidence is trustworthy;
3. live motor strength: how much force the constraint can use.

Patch quality may be useful for pivot/pose/debug. It is not reliable enough to reduce live motor force or live orientation authority.

## Current ROCK Sources

- `src/physics-interaction/grab/GrabMotionController.h`
  - `computeAngularAuthorityScale()` reduces release angular safety from position-only pivots, low contact support, small-object classification, and support shape.
  - `classifyContactSupportShape()` can classify single-hit small objects as `SphereLike`, two-hit long objects as `LongHandle`, and one-point patches as `Point`.
  - `solveMotorTargetsWithAuthority()` keeps linear/angular max force mass-capped and no longer publishes patch-derived authority values as live motor output.
- `src/physics-interaction/hand/HandGrab.cpp`
  - `classifyContactSupportShapeFromGrabFrame()` adds extra shape classification from patch spans.
  - `evaluateRuntimeHeldAuthority()` feeds frame patch/lever state into `GrabMotionController`.
  - `updateHeldSupportRefreshFromLivePalm()` can mutate `pivotAuthorityPositionOnly`, normal trust, contact patch sample count, and lever length during hold.
  - release handling uses `releaseAngularVelocityScale`, long-object release scaling, and axis scaling.
- `src/physics-interaction/grab/GrabContact.h`
  - contact patch geometry currently scales probe spacing down for small objects and down for very long objects.
  - single-hit patches become pivot-only, two-hit patches can be treated as orientation reliable if tangent/bitangent exist.
- `tests/GrabMotionControllerPolicyTests.cpp`
  - tests currently assert several authority reductions that match the bad live feel, such as small low-support angular softening and point/twist limits.

## HIGGS Reference

HIGGS dynamic grab does not use patch quality, small-object classification, or long-object classification to reduce live motor strength.

Relevant behavior from local HIGGS:

- In `src/hand.cpp`, dynamic held objects update the grab constraint each frame.
- Generic objects use `grabConstraintLinearMaxForce`; weapons use `grabConstraintLinearMaxForceWeapon`.
- Angular force is derived from linear force by `grabConstraintAngularToLinearForceRatio`.
- Both are capped by Havok mass through `mass * grabConstraintMaxForceToMassRatio`.
- Tau only changes for collision softening or actor/ragdoll cases.
- Inertia normalization is applied to grabbed connected bodies to avoid bad elongated-body axes.

HIGGS still updates pivot/target transform from selected geometry, but its normal object live motor is simple: fixed motor budget, mass cap, inertia normalization, collision softening.

## Correct ROCK Design

### 1. Live Held Motors Must Be Mass/Inertia Driven, Not Patch Driven

For normal loose object holds, motor output should be:

- linear tau: configured base, except contact softening;
- angular tau: configured base, except contact softening;
- linear max force: generic or weapon budget, startup fade, mass cap, shared-hand force share;
- angular max force: linear max force divided by angular-to-linear force ratio;
- no patch-size, point/line/sphere, position-only, small-object, or long-lever reduction of live held force.

This keeps the HIGGS-style feel while preserving ROCK's FO4VR constraint/proxy architecture.

### 2. Patch Evidence Must Not Reduce Pull/Follow Authority

Contact patch evidence should be demoted to:

- pivot candidate scoring;
- seated palm/pinch promotion evidence;
- finger pose evidence for non-thumb/index curves;
- debug telemetry;
- release safety only when explicitly validated.

It should not decide:

- live linear max force;
- live angular max force;
- live angular tau;
- whether a tiny object can be pulled to the hand.

### 3. Replace "Authority Scale" With "Release/Validation Scale"

The existing `AngularAuthorityOutput::authorityScale` is a misleading name because it invites live motor weakening.

Redesign it into two separate outputs:

- `HeldMotorAuthority`
  - always full live motor authority except mass cap, force share, fade, collision softening;
  - no patch/lever classification inputs.
- `ReleaseAngularSafety`
  - may use weak pivot, long lever, untrusted normal, and contact state;
  - only affects throw/release angular velocity, not held constraint authority.

This preserves useful safety without making the held object feel dead.

### 4. Replace Long/Small Lever With Shape/Inertia Metrics

Current `longObjectLeverGameUnits` is "max mesh distance from pivot." It is not enough:

- a cigarette can be long relative to its thickness but tiny in mass/size;
- a broom can have a huge lever but should still pull 1:1 at the grabbed point;
- a pocket watch or ball can look like weak support if the patch is small.

Proper metrics should be captured from local mesh or body data:

- local AABB extents: min/mid/max extent;
- compactness: `minExtent / maxExtent`;
- rodness: `maxExtent >> midExtent`;
- approximate object radius around pivot;
- COM-to-pivot offset;
- Havok mass;
- inverse inertia axes before/after normalization.

Use these metrics for diagnostics and release safety first. Do not feed them into live motor weakening.

### 5. Minimum Authority Floors For Tiny Objects

Mass cap alone can make tiny/light objects too weak if mass is very low. HIGGS caps by mass, but Skyrim objects may have better mass data. FO4VR props may not.

ROCK needs a motor mass floor for loose grabbed objects:

- `effectiveMotorMass = max(havokMass, configuredMinGrabMotorMass)`
- use effective mass only for motor force cap;
- keep real Havok mass unchanged;
- probably use a small floor such as `2.0` to `4.0` game-physics mass units after live tuning.

This avoids cigarettes/coins getting nearly no pull force while preserving finite force for heavy objects.

### 6. Patch Sampling Should Improve Pivot, Not Motor

Patch discovery still needs work, but the correct direction is:

- expand or adapt sample pattern based on object extents, not lever alone;
- for tiny objects, do not shrink the probe below the ability to find at least two sides/surfaces;
- for rods, sample across the cross-section and along the local tangent;
- require owner/body coherence before accepting samples;
- keep single-hit patches as position evidence only;
- do not use single-hit patches for orientation or motor scaling.

For long rods/handles, pivot quality should come from the actual selected/palm/pinch point, not from reducing motor authority because the patch looks like a line.

### 7. Orientation Authority Should Be Relation-Based

The object spinning around one point is not solved by reducing force. It means orientation relation is not trustworthy or not seated.

For each grab, ROCK should choose one relation mode:

- authored grab node: authored orientation owns relation;
- palm pocket: object relation is captured from palm pocket and frozen;
- pinch pocket: object relation is captured from thumb/index pocket and frozen;
- mesh hit fallback: position-only until seated promotion, then promote to palm/pinch relation.

If the grab only has one mesh point and no trusted relation:

- keep full force so the object follows;
- mark orientation as "needs seated relation";
- run seated palm/pinch promotion when it reaches the pocket;
- avoid using the single point/normal to create a fake axis-limited orientation model.

## Implemented On 2026-05-21

- Removed patch-derived `angularAuthorityScale` and `weakPivotTwistScale` from live `MotorOutput`.
- Kept `AngularAuthorityOutput` as release/axis safety input only.
- Made live held motor force depend on configured base force, fade, shared-hand force scale, angular-to-linear ratio, and mass cap.
- Added an effective motor mass floor used only when calculating motor force caps.
- Added config keys:
  - `bGrabEffectiveMotorMassFloorEnabled`
  - `fGrabEffectiveMotorMassFloor`
- Added policy tests proving weak/position-only and long-handle patch shapes do not reduce held linear/angular force.
- Added policy tests proving tiny objects use the effective mass floor when enabled and raw Havok mass when disabled.
- Added source-boundary checks so future edits cannot quietly re-couple patch quality to live held motor force.

## Concrete Implementation Plan

### Phase 1: Split Motor Authority From Patch Authority

Files:

- `src/physics-interaction/grab/GrabMotionController.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `tests/GrabMotionControllerPolicyTests.cpp`
- source-boundary tests that mention adaptive motor authority.

Changes:

- Create a pure helper for live motor authority that does not accept pivot/patch/lever inputs.
- Keep `solveMotorTargetsWithAuthority()` or replace it with `solveHeldMotorTargets()`.
- Move patch/lever authority calculations to release-only helpers.
- Stop using `angularAuthorityScale` as held authority.
- Update tests so weak/point/small/long support does not reduce held motor output.
- Keep tests for release angular velocity safety.

Expected behavior:

- long objects pull/follow with the same configured finite force as normal props;
- tiny objects are not weakened because patch support is small;
- patch confidence no longer makes the live grab sluggish.

### Phase 2: Add Effective Motor Mass Floor

Files:

- `GrabMotionController.h`
- `RockConfig.h`
- `RockConfig.cpp`
- `data/config/ROCK.ini`
- policy tests.

Config:

- `bGrabEffectiveMotorMassFloorEnabled`
- `fGrabEffectiveMotorMassFloor`

Rules:

- apply only to loose non-actor objects;
- apply only to force cap calculation;
- never write back to Havok mass;
- still allow heavy objects to be heavy.

Expected behavior:

- cigarettes/coins/pocket watches get enough minimum pull force to actually follow the hand.

### Phase 3: Replace Lever Metric With Shape Metrics

Files:

- `HandGrab.cpp`
- probably new pure helper under `src/physics-interaction/grab/`.

Capture:

- mesh extents;
- compact/rod/flat classification;
- pivot-to-COM and pivot-to-AABB relation;
- inertia ratio diagnostics.

Use:

- debug logging;
- release angular safety;
- future patch sampling pattern.

Do not use:

- live motor force reduction.

### Phase 4: Patch Sampling Rework

Files:

- `GrabContact.h`
- `HandGrab.cpp`
- contact patch policy tests.

Rules:

- one-hit patch: position evidence only, never orientation;
- two-hit patch: line evidence only, never full orientation;
- three or more coherent hits: surface evidence;
- tiny objects: preserve/enlarge search enough to find usable support, do not shrink probes only because lever is small;
- rods: sample cross-section and along tangent, then still treat as evidence unless promoted to a real palm/pinch relation.

Expected behavior:

- patch quality improves pivot/promotion, but motor feel remains stable while sampling is being tuned.

## What Not To Do

- Do not add another adaptive force multiplier based on patch size.
- Do not reduce live angular force because a patch is `Point`, `Line`, `SphereLike`, or `LongHandle`.
- Do not make single mesh normals own object orientation.
- Do not use inventory weight for motor feel.
- Do not fake long-object realism by making the motor weak. Long-object realism should come from pivot placement, Havok mass/inertia, and release safety.

## Validation Needed

Policy tests:

- weak/position-only/single-point patch keeps full held linear and angular max force;
- long handle keeps full held max force;
- tiny object with low Havok mass gets effective mass floor when enabled;
- release angular velocity can still be reduced for unsafe weak/long/untrusted cases;
- patch sampling single-hit never promotes orientation authority.

Live tests:

- cigarette: can pull to hand and hold without low-force dead behavior;
- coin/pocket watch: follows hand without orbiting around a single point;
- broom/rifle/bottle: slow and snap rotations feel consistent, not step-activated;
- collision push into wall still softens via collision tau only;
- release throws still do not explode on weak pivots.
