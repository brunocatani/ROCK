# Custom Dynamic Grab Authority Tracker

Date: 2026-05-12

Current repo state when written:

- Workspace: `E:\fo4dev\PROJECT_ROCK_V2`
- Repo: `ROCK`
- Current branch: `feature/ghidra-grab-motor-mapping`
- Latest observed commits:
  - `55c5f68 feature/ghidra-grab-motor-mapping: document HIGGS motor parity data`
  - `350aec6 feature/ghidra-grab-motor-mapping: document mouse spring and motor evidence`
  - `f0082fa origin/develop develop: fix/weapon-visual-remap: authorize remap from workbench stack witness`

This file is a recovery tracker for the next phase. It is not implementation. It exists so the custom grab direction, corrected assumptions, and audit gates survive compaction.

## Scope

Research and later implementation scope:

- Ordinary dynamic one-hand grab.
- Loose non-equipped objects.
- Loose non-equipped weapons.
- Physics-driven pull, converge, hold, and release.
- Mass, inertia, finite force, angular authority, smoothing, deviation, and contact-based grip logic.

Out of scope unless a shared function must be understood:

- Keyframed grab.
- Actor ragdoll grab.
- Equipped weapon handling.
- Equipped two-hand weapon handling.
- Animation-only attachment.

## User Decisions Locked

These choices were explicitly selected for the future implementation branch:

- Runtime branch mode: hard replace.
  - Ordinary one-hand dynamic grabs must use the custom authority path in the feature branch.
  - No native mouse-spring fallback for ordinary one-hand dynamic grab.
  - If custom authority cannot create or update, the grab must fail or release cleanly and log the failing subsystem.
- Hand authority body: dedicated proxy.
  - Use a new per-hand grab authority proxy body as constraint body A.
  - Do not constrain the existing palm collider body.
- Research gate: audit first.
  - Finish Ghidra/source audit of custom constraint runtime offsets, step phases, proxy body creation, HIGGS parity, and ROCK integration before code changes.

Planned implementation branch:

- Create from `develop`, not from the current research branch.
- Suggested name: `feature/custom-dynamic-grab-authority`.

## Corrected Assumptions

Important correction from the user:

- The generated palm body is created from the root-flattened hand frame bone tree.
- Therefore, the generated palm body is not wrong because of a different upstream hand convention.

Remaining concern:

- The generated palm body currently has a contact/collider role and participates in generated hand collider lifecycle.
- `HandBoneColliderSet::update` already treats `palmAnchorBody.isConstrained()` as a special case that can defer rebuilds.
- For primary one-hand custom grab authority, the safer design is still a separate no-collision proxy body sourced from the same root-flattened data, so solver authority does not contaminate contact evidence or hand collider lifecycle.

## Core Physics Rules

Hard rules for the custom dynamic grab design:

- COM is never grip pivot authority.
- COM is allowed only for mass, inertia, lever length, release swing velocity, and weight effects.
- Grip pivot must come from contact, mesh evidence, authored explicit grab point, or the existing ROCK contact/palm selection pipeline.
- Object rotation must be preserved unless an explicit authored grip reference says otherwise.
- The object-hand relationship must be captured once at grab commit and not recaptured during converge or held settling.
- Heavy objects must be allowed to lag through finite force and deviation instead of being forced perfectly to the hand.
- Loose non-equipped weapons must use dynamic loose-object rules with weapon-specific motor tuning, not equipped weapon code.

## Current ROCK Facts

Current one-hand dynamic grab path:

- Ordinary single-hand grabs currently use `HeldObjectDriveMode::NativeMouseSpring`.
- `HandGrab.cpp` creates `_nativeGrab` for ordinary single-hand dynamic grabs.
- Native path queues a target frame derived from:
  - root-flattened hand transform;
  - captured `_grabFrame.rawHandSpace`;
  - `_grabFrame.bodyLocal`;
  - frozen `_grabFrame.pivotBBodyLocalGame`.
- Current native path is stable and mostly works, but heavy objects and loose guns feel too strong because authority is static/native-response based.

Current custom constraint path:

- Existing custom constraint path is used for shared/two-hand loose-object joins.
- `createConstraintGrabDrive` creates `_activeConstraint`.
- `updateConstraintGrabDriveTarget` refreshes angular target and transform-B translation.
- `updateConstraintGrabDriveMotors` uses `GrabMotionController`.
- The current shared path uses `_handBody`/palm anchor as body A.
- That is acceptable for the current shared path but should not become the primary one-hand authority without separating contact body from solver authority.

Current good scaffolding to preserve:

- Root-flattened hand frame resolution.
- Mesh/contact evidence for grip point selection.
- Canonical `_grabFrame` with `rawHandSpace`, `constraintHandSpace`, `bodyLocal`, `pivotAHandBodyLocalGame`, and `pivotBBodyLocalGame`.
- Hand/finger pose solve at commit and reuse while held.
- Multi-body held set tracking.
- Inertia normalization/restore.
- Player-space movement compensation.
- Release velocity composition.
- Held object deviation/drop policy.
- Loose weapon dynamic tuning config surfaces.

## HIGGS Dynamic Grab Facts To Preserve Conceptually

HIGGS dynamic grab behavior from local source research:

- `Hand::TransitionHeld` selects a mesh/contact point near the palm line.
- Selected point becomes object-side grip evidence.
- Palm/contact point becomes hand-side pivot.
- Desired object transform preserves object rotation and translates object so selected point seats at palm.
- Constraint body A is the hand body.
- Constraint body B is the selected object body.
- Pivot A is hand-local.
- Pivot B is object/body-local.
- `GrabConstraintData` uses:
  - local transform atom;
  - setup stabilization atom;
  - ragdoll motor atom for angular axes;
  - three linear motor atoms.
- Linear motor target positions stay zero.
- Linear goal is driven by transform A/B, especially dynamic transform-B translation.
- Angular target is updated by `setTargetRelativeOrientationOfBodies`.
- Per-frame motor tuning applies:
  - finite linear force;
  - angular force derived from linear force ratio;
  - mass cap using `mass * grabConstraintMaxForceToMassRatio`;
  - collision tau softening;
  - tau lerp;
  - startup angular fade-in;
  - inertia normalization and restore;
  - visual hand deviation and drop thresholds.
- Loose weapons are still dynamic grabbed objects, with higher base linear max force, then angular authority is still constrained by ratio and mass cap.

## Existing Research Docs

Useful current docs:

- `ROCK/docs/ghidra-mouse-spring-vs-hkp-motors-2026-05-12.md`
  - Ghidra-only mapping of FO4VR native mouse spring and hkp motor evidence.
- `ROCK/docs/higgs-custom-motor-parity-research-2026-05-12.md`
  - HIGGS source plus ROCK comparison for custom motor parity.

Do not blindly trust older random markdown files. Treat the two docs above as current starting points, then verify risky claims by source/Ghidra before implementation.

## Why Mouse Spring Feels Smooth

Ghidra/source research explains why native mouse spring stutters less than the old motor attempt:

- It is an hknp action-style per-body mechanism.
- It consumes body id, local grab anchor, target point, target transform, dt, response fields, cap fields, and previous/error state.
- It has damping, deadband/threshold behavior, dt-aware correction, cap behavior, and previous-state smoothing.
- It is designed for a moving world-space target.

Why it lacks the desired weight feel:

- It does not expose the full HIGGS-style finite-force, mass-cap, angular-ratio, collision tau, inertia, and hand-deviation behavior.
- Static native response makes heavy objects and heavy guns feel too easy to swing.

## Why The Previous Custom Direction Failed

Known failure causes to avoid:

- Treating COM/motion data as if it could participate in grip authority.
- Mixing weight simulation with pivot/reference-frame authority.
- Letting more than one system drive the same held body.
- Feeding custom motors from unstable or mismatched frame/phase data.
- Recreating or toggling motors in ways that reset solver runtime history.
- Reusing a contact/collider hand body as solver authority without accounting for its lifecycle and timing role.
- Assuming native mouse-spring conventions define the desired grab model.

Important Ghidra fact:

- FO4VR ragdoll motor enable/disable path at `0x1419B2640` clears runtime memory when toggled.
- Future custom grab must create a persistent held constraint and retune motor fields while held, not toggle motors every frame.

## Custom Authority Architecture

Planned architecture for the future branch:

1. Add per-hand `GrabAuthorityProxy`.
   - Dedicated generated body.
   - No collision/contact gameplay role.
   - Source transform from the same root-flattened hand/bone data as current hand colliders.
   - Serves only as body A for held-object custom constraint.
   - Separate lifecycle from palm/finger contact colliders.

2. Replace ordinary one-hand native drive with custom motor drive.
   - Body A: grab authority proxy.
   - Body B: grabbed object hknp body.
   - Pivot A: palm/contact pocket point in proxy-local space.
   - Pivot B: selected mesh/contact grip point in object/body-local space.
   - Target frame: captured hand/object relation from `_grabFrame.rawHandSpace`.
   - No COM fallback.

3. Drive proxy and constraint target in one verified physics phase.
   - Proxy body update and constraint target update must happen before solver consumption.
   - Do not update proxy from one phase and transform-B/motors from another without proof.
   - Candidate phases to verify:
     - existing substep-pre-collide callback;
     - `betweenCollideAndSolve` step listener slot;
     - whole-pre-step only if solver timing proves it is enough.

4. Reuse and harden existing custom constraint path.
   - Keep HIGGS-shaped 6-DOF atom chain.
   - Verify runtime offsets and solver result count before primary use.
   - Keep linear target zero and move transform A/B frames.
   - Update angular target from captured desired body-in-hand relation.

5. Apply `GrabMotionController` as the motor policy.
   - Linear finite force.
   - Mass cap.
   - Angular force ratio.
   - Collision tau softening.
   - Tau lerp.
   - Startup angular fade-in.
   - Loose weapon multipliers.

6. Preserve held visual hand behavior.
   - Visual hand should follow held object through captured relation.
   - Deviation between real hand and adjusted hand expresses heavy-object lag.
   - Deviation threshold can release/drop if object cannot keep up.

## Audit Items Before Implementation

### Constraint Runtime Audit

Verify these before making custom drive primary:

- `GRAB_CONSTRAINT_SIZE = 0x168`
- atom offsets:
  - `ATOM_TRANSFORMS = 0x20`
  - `ATOM_STABILIZE = 0xB0`
  - `ATOM_RAGDOLL_MOT = 0xC0`
  - `ATOM_LIN_MOTOR_0 = 0x120`
  - `ATOM_LIN_MOTOR_1 = 0x138`
  - `ATOM_LIN_MOTOR_2 = 0x150`
- `ATOMS_SIZE = 0x148`
- `RUNTIME_SOLVER_RESULTS = 12`
- `RUNTIME_REPORTED_SIZE = 0x100`
- ragdoll offsets:
  - `RT_RAGDOLL_INIT_OFFSET = 0x60`
  - `RT_RAGDOLL_PREV_ANG_OFFSET = 0x64`
- linear offsets currently written by ROCK:
  - init: `{0x40, 0x31, 0x22}`
  - previous target: `{0x44, 0x38, 0x2C}`

Concern:

- HIGGS source reports six solver results and runtime `sizeof(Runtime) * 2`.
- HIGGS linear runtime offsets are `offsetof(Runtime, m_initializedLinear[axis])` and `offsetof(Runtime, m_previousTargetPositions[axis])`.
- FO4VR prismatic constructor shows native single linear motor offsets around `0x50/0x54`.
- ROCK current three-axis custom offsets may be deliberate FO4VR/hknp custom convention, but they need a focused binary audit before relying on them for primary one-hand grab.

### Step Phase Audit

Verify:

- What each step-listener vtable slot means in FO4VR:
  - before whole;
  - before any;
  - between collide and solve;
  - after any;
  - after whole.
- Whether constraint atom writes are safe in:
  - game update;
  - whole-pre-step;
  - substep-pre-collide;
  - between-collide-and-solve.
- Whether proxy keyframe drive must happen in the same callback as constraint target update.

### Proxy Body Audit

Verify:

- Required collision layer/filter for a no-contact proxy body.
- Whether an empty/no-collision shape is safe or whether a tiny shape on a disabled matrix row is required.
- Whether keyframed proxy body can be constrained as body A without polluting collision/contact.
- Whether `BethesdaPhysicsBody::isConstrained()` behavior matters for the proxy lifecycle.
- Whether proxy creation can reuse current generated body infrastructure or needs a separate owner class.

### HIGGS Parity Audit

Confirm the exact HIGGS behavior that still matters:

- Dynamic pull/converge to held transition.
- `TransitionHeld` pivot and reference-frame capture.
- `CreateGrabConstraint` body/pivot setup.
- `HeldBody` transform-B and angular-target update.
- Mass cap and angular-force-ratio update.
- Collision tau softening.
- Inertia normalization.
- Hand deviation/lag/drop.
- Loose weapon base force branch.
- Release velocity and connected-body cleanup.

## Implementation Checklist For Future Branch

No code should start until the audit items above are complete.

When implementation starts:

1. Create branch from `develop`:
   - `feature/custom-dynamic-grab-authority`

2. Add documentation note at start of implementation explaining:
   - why custom authority is replacing native for ordinary dynamic grab;
   - why dedicated proxy is used instead of palm collider;
   - why COM is excluded from pivot authority;
   - why no native fallback exists in this branch.

3. Add `GrabAuthorityProxy` subsystem.
   - Per hand.
   - Generated body lifecycle.
   - No-contact collision/filter policy.
   - Root-flattened target queue.
   - Physics-phase flush.
   - Debug telemetry.

4. Extend drive mode.
   - Replace or add a one-hand custom motor mode.
   - Ordinary one-hand loose dynamic grabs must create custom constraint.
   - Shared/two-hand loose grabs should converge on the same constraint authority design where compatible.

5. Change ordinary grab creation.
   - Stop creating `_nativeGrab` for ordinary one-hand loose dynamic grabs.
   - Create proxy-backed custom constraint.
   - Keep pivot/reference capture unchanged.

6. Change held update.
   - Queue proxy target from root-flattened hand frame.
   - Update constraint transform A/B and angular target in verified physics phase.
   - Apply motor tuning per frame.
   - Keep deviation/visual hand behavior.

7. Change release/lifecycle.
   - Destroy constraint.
   - Release proxy hold state.
   - Restore object body flags, collision, inertia, damping, contact listeners, and player-space registration.
   - Keep multi-body/multipart cleanup.

8. Update tests.
   - Replace native one-hand policy tests.
   - Add custom authority policy tests.
   - Add motor/mass/weapon/deviation policy tests.
   - Keep actor/equipped weapon exclusion tests.

## Test Requirements

Source/policy tests:

- Ordinary one-hand dynamic grab must not create `_nativeGrab`.
- Ordinary one-hand dynamic grab must create custom motor constraint.
- Native mouse spring wrapper remains only for diagnostics/legacy paths, not ordinary one-hand authority.
- Palm collider body must not be constrained as ordinary one-hand grab body A.
- Grab proxy must be sourced from root-flattened hand data.
- COM/motion frame must not feed pivot or target frame.
- No old legacy surface/opposition/pinch authority returns.

Unit-style policy tests:

- Motor force caps by mass.
- Angular force derives from capped linear force.
- Collision state lowers tau target.
- Tau lerps rather than snaps.
- Startup fade begins with weak angular authority and ramps down ratio.
- Loose weapons use dynamic loose-weapon multipliers.
- Heavy object deviation accumulates and can trigger release.

Runtime validation:

- Heavy clutter cannot be swung like it has no weight.
- Heavy loose gun resists fast wrist motion without jitter.
- Long loose weapons do not spin freely around COM.
- Edge/corner grab pivots at selected contact point.
- Multipart weapon/object collision remains after release.
- Release velocity feels believable and does not erase collisions.
- Actor ragdoll grab unchanged.
- Equipped weapon behavior unchanged.

## Open Unknowns

Unresolved until audit:

- Correct FO4VR runtime layout for custom six-axis grab constraint.
- Correct solver result count and runtime size.
- Correct linear atom offsets for three linear axes.
- Best step-listener phase for proxy and constraint target update.
- Best no-collision proxy body filter/layer policy.
- Whether the existing `createGrabConstraint` raw motor ownership is sufficient for primary use or needs refcount/lifetime cleanup changes.
- Whether current transform-B update math exactly matches FO4VR hknp custom atom expectations under all rotations.
- Whether loose weapon should add lever-length scaling on top of mass force cap, using grip-to-COM as weight data only.

## Current Next Action

Continue read-only research:

1. Ghidra audit custom constraint runtime and atom offsets.
2. Ghidra/source audit FO4VR step phases and safe write timing.
3. Source audit generated body creation for no-contact proxy reuse.
4. Source audit ROCK tests that must change for hard replacement.
5. Only after this, write a decision-complete implementation plan and then implement on a branch created from `develop`.
