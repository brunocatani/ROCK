# Custom Dynamic Grab Authority Gap Map - 2026-05-15

## Why This File Exists

This is a mapping-only audit of
`E:\fo4dev\Backups\just in case\docs\custom-dynamic-grab-authority-current-findings-2026-05-12.md`
against the current ROCK source. The goal is to preserve what the backup
research still teaches us after the later custom dynamic grab work, without
turning old notes into blind implementation orders. This file does not propose
or apply code changes.

## Scope

Included:

- ordinary dynamic one-hand loose-object grab;
- loose non-equipped weapon grab as a loose-object variant;
- hidden proxy, custom finite-force constraint, pivot/reference frame, mass,
  inertia, angular authority, release, hand pose, and runtime timing.

Excluded:

- equipped weapon handling;
- two-hand equipped weapon handling;
- actor ragdoll grab;
- HIGGS keyframed object grab;
- missing generated weapon collider creation or reactivation work. That issue
  is known broken/separate and is intentionally left alone here.

## Sources Checked

- Backup research:
  `E:\fo4dev\Backups\just in case\docs\custom-dynamic-grab-authority-current-findings-2026-05-12.md`
  (6212 lines).
- Current source:
  - `src/physics-interaction/hand/HandGrab.cpp`
  - `src/physics-interaction/hand/Hand.cpp`
  - `src/physics-interaction/hand/HandBoneColliderSet.cpp`
  - `src/physics-interaction/hand/HandColliderTypes.h`
  - `src/physics-interaction/core/PhysicsInteraction.cpp`
  - `src/physics-interaction/grab/GrabAuthorityProxy.h`
  - `src/physics-interaction/grab/GrabConstraint.*`
  - `src/physics-interaction/grab/GrabMotionController.h`
  - `src/physics-interaction/native/GeneratedKeyframedBodyDrive.*`
  - `src/physics-interaction/native/GrabAuthorityPhase0Probe.*`
  - `data/config/ROCK.ini`
  - grab source-policy tests under `tests/`.
- No web used.
- No new Ghidra pass used for this audit.

## Status Legend

- Implemented: current source has the architecture from the backup research.
- Partial: current source has the main structure, but a backup risk remains.
- Open gap: backup research identifies a relevant item that current source does
  not close.
- Superseded: backup item was overtaken by later validated architecture.
- Excluded: intentionally outside this pass.

## High-Level Result

Most of the large backup requirements are now implemented:

- native mouse spring is no longer production authority for ordinary dynamic
  grab;
- ordinary and peer-held loose grabs route through proxy-backed custom
  authority;
- pivot B remains a selected object/contact point in body-local space;
- COM is diagnostic/weight/release data only;
- body A is a hidden no-contact keyframed proxy;
- body B remains the dynamic held object;
- finite-force linear motor control, mass caps, angular-to-linear force ratio,
  collision tau, fade-in, shared hand force scale, long-object angular scaling,
  and loose-weapon tuning surfaces exist;
- hand pose is mesh/contact solved and stored at commit;
- the old INI handspace pivot is not the production dynamic-grab pivot when the
  live palm anchor exists.

The main remaining backup-derived production gap is:

- the hidden grab authority proxy still uses the generated keyframed-body
  `DriveToKeyFrame` path. The backup research repeatedly marks this as risky
  for a solver authority proxy because `DriveToKeyFrame` can fall into a
  cap-triggered transform snap/zero-velocity branch and because proxy movement
  and constraint target writes need to be one coherent pre-solve unit.

## Current Architecture Map

### State and Authority

Implemented.

Current dynamic grab creates a proxy-backed custom authority path in
`HandGrab.cpp`:

- `createProxyConstraintGrabDrive(...)` creates a hidden keyframed proxy body
  and the custom finite-force constraint.
- `flushPendingCustomGrabAuthority(...)` runs from the physics between phase.
- `observeCustomGrabAuthorityAfterSolve(...)` reports post-solve diagnostics.
- `HandGrabNativeBoundarySourceTests.ps1` rejects native mouse-spring wrappers,
  `_nativeGrab.create`, `NativeMouseSpring`, removed native drive modes, and
  adaptive native target-leading logic.

The old backup line that said ordinary one-hand grabs still used
`HeldObjectDriveMode::NativeMouseSpring` is obsolete for current source.

### Physics Step Timing

Implemented.

`PhysicsInteraction.cpp` registers the generated body step coordinator with:

- pre-step generated collider flush;
- between-collide-and-solve grab authority callback;
- after-solve grab authority observation callback.

The between callback runs:

- right-hand `flushPendingCustomGrabAuthority`;
- left-hand `flushPendingCustomGrabAuthority`;
- optional `GrabAuthorityPhase0Probe::driveBetweenCollideAndSolve`.

This closes the backup's broad timing requirement: proxy/constraint/motor
updates no longer live only in normal game-frame update flow.

Remaining timing concern:

- current production proxy motion still delegates to `driveGeneratedKeyframedBody`
  inside that between callback. The timing slot is correct, but the drive
  primitive is still the generic generated-collider primitive.

### Hidden Proxy Body

Implemented with one open policy question.

Current proxy:

- is created through `BethesdaPhysicsBody::create`;
- uses `BethesdaMotionType::Keyframed`;
- has a tiny convex tetra hull from `GrabAuthorityProxy.h`;
- uses `FO4_LAYER_NONCOLLIDABLE | bit14`;
- validates filter readback through `hasNoContactFilterInfo`;
- is destroyed through the Bethesda generated-body lifecycle after destroying
  the custom constraint.

This closes the backup requirement for a dedicated no-contact per-hand proxy
instead of using the semantic palm collider as constraint body A.

Open:

- backup research left three no-contact policy choices: bit 14, layer 15, or a
  dedicated extended zero-row layer. Current source chose
  `nonCollidable+bit14`. That is a valid implemented policy, but the backup did
  not prove it is the final best policy for every future diagnostic/debug need.

### Proxy Frame Source

Implemented.

Current proxy authority resolves from the live generated palm-anchor body:

- `Hand::tryResolveLivePalmAnchorReference(...)` reads `_handBody` through
  `tryResolveLiveBodyWorldTransform`;
- `HandBoneColliderSet.cpp` captures `_latestPalmAnchorTarget` from the same
  sampled role frame used for the palm anchor;
- `HandColliderTypes.h::generatedColliderFrameToGrabAuthorityFrame(...)`
  returns the generated collider frame directly instead of transposing it.

Current tests reject the old transpose adapter and reject raw controller
fallbacks in the proxy frame path.

### Pivot and Reference Frame

Implemented.

Current grab commit:

- rejects grabs with no object-side contact/authored point instead of falling
  back to object origin or COM;
- uses `shiftObjectToAlignGripWithPocket(...)` for generic point-to-palm grabs;
- uses `buildDesiredObjectWorldFromAuthoredGrabNode(...)` only in the explicit
  authored grab-node branch;
- captures `objectToBodyAtGrab` from the visible object to FO4VR BODY readback;
- captures `objectToConstraintBodyAtGrab` separately for custom solver body B;
- freezes pivot B with `freezePivotBBodyLocal(...)` in both BODY and constraint
  body-local spaces;
- stores `_grabFrame.bodyLocal` and `_grabFrame.constraintBodyLocal`;
- composes desired object rotation through `rawRotationPalmTranslation`.

This matches the backup rule:

- COM is never grip pivot authority;
- selected object/contact point is pivot B;
- palm/proxy frame is body A;
- object rotation is preserved for generic grabs;
- authored nodes may override rotation only through an explicit branch.

### Old INI Pivot

Mostly superseded for production dynamic authority.

Current production `computeGrabPivotAWorld(...)` returns the live palm anchor
world translation when available. If the live palm anchor is unavailable, it
falls back to the hand transform origin, not the old configured handspace pivot.

The old configured handspace pivot still exists in:

- `HandFrame.h::computeGrabPivotAHandspacePosition`;
- `computePalmPositionFromHandBasis`;
- API/debug controller helpers;
- telemetry fields named `legacyConfiguredPivot...`;
- some nearby selection/pocket diagnostics.

Current tests explicitly require the old INI point to remain available only for
legacy telemetry and non-dynamic-grab callers. It should not be treated as the
current dynamic-grab pivot A authority.

Potential cleanup only:

- if future confusion returns, the config/debug naming around
  `fRightGrabPivotAHandspace*` and `fLeftGrabPivotAHandspace*` should be made
  visibly legacy. That is not required for the current mapped authority path.

### Custom Constraint Runtime

Partial / mostly closed by later research.

The backup contains contradictory runtime-offset conclusions:

- earlier sections claim native witnesses imply absolute offsets from runtime
  base and flag ROCK's linear offsets as likely overlapping solver results;
- later sections correct that after a focused atom-interpreter audit, saying
  type `0x0B` linear motor uses current runtime writer/cursor semantics and
  `RUNTIME_SOLVER_RESULTS = 12` is coherent for ROCK's atom stream.

Current source uses:

- atom stream: set-local-transforms, setup-stabilization, ragdoll motor, three
  linear motor atoms;
- `RUNTIME_SOLVER_RESULTS = 12`;
- `RUNTIME_REPORTED_SIZE = 0x100`;
- linear motor atoms enabled;
- ragdoll angular atom disabled;
- direct angular velocity authority instead of relying on type-19 ragdoll atom.

Current source comments still mention the linear offsets as relative to
per-atom pointers. The later backup research is more precise: they are coherent
relative to the live runtime writer/cursor, not independent per-atom runtime
blocks.

Remaining unresolved from backup:

- final exact helper-side warm-start behavior;
- runtime padding/safety margin beyond current successful insertion;
- exact type `0x0B` and type `0x13` case semantics if ROCK ever wants to return
  to solver-native angular motor authority.

This is not currently blocking production because angular atom authority is
disabled, but it remains relevant if the direct angular velocity path is ever
replaced by a solver-only angular motor.

### Linear Motor, Mass, Collision, and Deviation

Implemented.

Current `GrabMotionController.h` implements:

- error factor from position and rotation error;
- mass-capped linear max force;
- angular max force derived from linear force and angular-to-linear ratio;
- authority sharing between hands;
- collision tau;
- tau smoothing;
- startup fade-in;
- long-object angular scale helper.

Current `HandGrab.cpp` feeds:

- aggregate held body mass via `readHeldBodyMassSummary(..., _heldBodyIds)`;
- held body collision state;
- loose-weapon multipliers;
- per-hand authority scale;
- current position/rotation error;
- fade-in settings.

Current tests cover:

- mass cap;
- angular-to-linear force ratio;
- two-hand shared authority;
- long-object angular speed scale.

Open polish from backup:

- held-object motion-properties leasing for additional weight/damping is not
  implemented as a held-state system. ROCK has `NearbyGrabDamping`, but that is
  nearby-object damping, not a full held-object damping/velocity-cap lease.
  The backup marks this as a candidate, not a required first implementation.

### Angular Authority

Superseded by current architecture.

Backup target originally expected custom linear plus custom angular motors,
similar to HIGGS' hkp ragdoll angular atom. Current ROCK no longer trusts the
FO4VR type-19 ragdoll angular atom for production dynamic grab. Instead:

- the linear constraint atoms own selected-pivot translation;
- FO4VR native hard-keyframe angular velocity computation supplies the angular
  velocity vector convention;
- ROCK clamps that vector by the finite angular budget derived from the current
  motor controller;
- angular velocity is applied to the accepted held-body set, not only the
  primary body.

This is a deliberate FO4VR-native divergence from the backup's original
HIGGS-shaped angular atom plan.

Remaining future research only:

- if solver-only angular authority is desired later, the type-19 ragdoll atom
  path and/or newer FO4VR angular motor wrappers must be mapped again at the
  exact live solver case level. Current code should not be judged missing
  simply because it does not use the HIGGS angular atom model.

### Proxy Drive Primitive

Open gap / highest backup-derived risk.

Current source still drives the hidden proxy with:

- `queueGeneratedKeyframedBodyTarget(_grabAuthorityProxyDriveState, ...)`;
- `driveGeneratedKeyframedBody(..., "grab-authority-proxy", ...)`;
- telemetry string `proxyDrive=driveToKeyFrame`;
- `GeneratedKeyframedBodyDrive.cpp` calls `body.driveToKeyFrame(...)` during
  normal steady motion.

The backup repeatedly warned that this may be wrong for a hidden grab authority
proxy:

- `DriveToKeyFrame` can take a cap-triggered transform snap path;
- generated hand/body/weapon colliders are contact colliders and should be
  placed before collide;
- the hidden grab proxy is a solver authority anchor and must move coherently
  with constraint target/motor updates before solve;
- `ApplyHardKeyFrame`, direct set-transform/set-velocity, or a combined
  transform-plus-velocity primitive may be cleaner for the hidden proxy.

Current code moved the generated drive into the between phase for the proxy, so
it fixed the phase split. It did not replace the drive primitive.

This is the clearest "still missing from backup" item.

What must be proven before changing it:

- whether wrapper `setTransform` / `setVelocity` inside ROCK's actual between
  callback is direct or queues into a command list that drains before solve;
- whether direct set-transform after collide has acceptable broadphase/contact
  side effects for a no-contact body;
- whether the proxy must publish both target pose and physically meaningful
  velocity in the same phase;
- whether direct velocity only is stale by one solver step because the body slot
  transform remains previous-frame;
- whether the combined transform-plus-velocity pattern found in the backup is
  safer than pure direct velocity.

### Direct Setter / TLS Proof

Partial.

`GrabAuthorityPhase0Probe.cpp` exists and can:

- create no-contact proxy and optional dynamic receiver;
- call `setTransform` and `setVelocity` inside the between phase;
- log TLS command state;
- read back body transform;
- detect semantic contact leaks;
- create a test custom constraint.

However, the production proxy path does not use this direct setter primitive.
The probe is diagnostics, not proof encoded in current production behavior.

Remaining from backup:

- there is no source-level artifact in this audit proving that production
  `BethesdaPhysicsBody::setTransform` and `setVelocity` are always consumed by
  the solver in the same step in every callback context;
- the backup's TLS `+0x1528` direct-vs-queued uncertainty remains a runtime
  evidence question unless backed by captured logs or deeper binary mapping.

### Hand Pose and Finger Logic

Implemented.

Current dynamic grab:

- solves mesh/contact finger pose at commit;
- stores `_grabFingerPose`;
- stores local finger target points/normals in the grab frame;
- applies acquisition pose before final touch;
- applies held pose through `applyRockGrabHandPose`;
- supports custom joint positions and local transform correction;
- includes alternate thumb and surface safety logic in `GrabFinger.h`;
- does not require hardcoded HIGGS_R/L or ROCK_R/L mesh-only hacks for ordinary
  dynamic grab pose.

Open polish from backup:

- HIGGS' visual hand deviation concept is partly covered by ROCK's visual hand
  transform, max deviation release, and external FRIK transform path. If future
  testing still shows "superman arm" feel visually, the next mapping should
  compare visual-hand lag/deviation in runtime logs rather than changing grip
  authority.

### Release and Cleanup

Implemented / no backup-critical gap found in this pass.

Current release path:

- uses `activeProxyConstraintPivotBLocalGame()` for release lever origin;
- reads COM only for release tangential velocity data;
- composes controller angular release velocity through a pure capped policy;
- writes release velocity to the accepted held-body set;
- restores active grab lifecycle/body flags;
- restores inertia;
- destroys constraint before proxy cleanup;
- supports delayed hand-collision restore.

This matches the backup's key rule: COM can participate in release swing and
lever math, not grip authority.

### Player-Space Compensation

Partial / current architecture has the feature, but one backup question remains.

Current source has:

- `HeldPlayerSpaceRegistry`;
- central held-player-space velocity application;
- per-hand held object motion samples;
- player-space warp handling and residual velocity damping.

Backup unresolved item:

- whether some player-space transform/warp compensation should move into the
  same physics phase as proxy/constraint updates for custom authority.

This remains a possible investigation point if player rotation or room-space
movement still produces world-direction-dependent grab behavior. It is not
evidence that the current pivot or COM rule is wrong.

### Loose Non-Equipped Weapons

Implemented for current dynamic grab authority, excluding collider creation.

Current source:

- detects loose weapon targets through normal active-grab refs and `WEAP`;
- stores `_heldObjectIsLooseWeapon`;
- applies loose-weapon multipliers for linear tau, angular tau, collision tau,
  damping, max force, angular force, and recovery;
- keeps loose weapons in the loose-object dynamic path, not equipped weapon
  handling;
- uses aggregate held-body mass and accepted held body ids for multipart
  authority where body scanning succeeds.

Current packaged config exposes neutral per-axis/tau surfaces and
`fGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5`.

Explicit exclusion:

- missing or broken generated weapon collider creation / reactivation is not
  part of this audit. Loose grabbed weapons rely on native object collisions
  and the accepted held body set here.

### Detection, Acquisition, and Contact

Implemented relative to backup requirements.

Current source keeps:

- close/far selection;
- contact and mesh evidence;
- no COM candidate priority;
- no object-origin fallback;
- three-phase acquisition and frozen relation transition;
- contact patch / multi-finger evidence;
- object-side pivot capture from contact/authored data;
- collision-aware held-body response.

The hidden no-contact proxy is only body A for solver authority. The rest of
dynamic grab is still contact-driven.

### Native Mouse Spring

Superseded/removed from production dynamic grab.

Backup research correctly said mouse spring was smoother than old custom
motors but not the quality target. Current source has already moved beyond
that:

- `NativeMouseSpringGrab.*` files are rejected by tests;
- mouse-spring offsets are rejected by tests;
- current config does not expose mouse-spring tuning keys;
- physics step no longer flushes native held grab authority.

No remaining backup gap here.

### Native Easing, Malleable, Breakable, Direct hknp Body Creation

Not production gaps.

Backup research found these FO4VR-native mechanisms:

- `hknpEaseConstraintsAction`;
- `hknpSafeEaseConstraintsAction`;
- `hknpMalleableConstraintData`;
- `hknpBreakableConstraintData`;
- direct hknp body creation/lifetime commands.

Backup conclusions were conservative:

- ease action is not a generic smoother for arbitrary ROCK custom constraints;
- malleable semantics need another pass before use;
- breakable is future-only for overforce/snag release ideas;
- direct hknp body creation is not production-ready and still appears to need a
  valid shape/lifecycle discipline.

Current source not using these is not a missing implementation for the current
dynamic grab authority.

## Remaining Backup-Derived Gaps

### 1. Proxy Drive Primitive

Current production proxy uses `DriveToKeyFrame`. Backup research says this is
the most suspicious remaining architecture mismatch for a hidden solver
authority proxy.

Required later decision:

- keep generated drive because it matches the proven palm-collider convention;
- or replace only the hidden proxy drive with a dedicated same-phase
  transform/velocity primitive after runtime proof.

This must not affect generated hand/body/weapon contact collider driving. Those
colliders still need their current pre-collide generated drive behavior.

### 2. Direct Setter / Command Queue Proof

Phase0 diagnostics exist, but this audit did not find a committed runtime proof
that the production between callback's wrapper setter calls are direct or
solver-visible in the same substep.

This matters only if the proxy drive primitive changes away from
`DriveToKeyFrame`.

### 3. Held Motion-Properties Lease

Nearby damping is implemented, but a held-object motion-properties lease for
weight/damping/velocity-cap feel is not.

The backup marks this as a possible weight-polish surface, not a required
architecture block. It should remain secondary to the proxy drive primitive,
because changing velocity caps can interact badly with `DriveToKeyFrame`.

### 4. Final Solver Motor Runtime Audit

The later backup research makes the current 12-solver-result linear atom shape
plausible/coherent. Still unresolved:

- exact warm-start/helper behavior;
- exact type `0x0B` and type `0x13` case-level semantics if angular atom
  authority is revisited.

This is not currently blocking because direct angular velocity is the chosen
FO4VR-native angular authority.

### 5. Player-Space Phase Coupling

Current player-space compensation exists, but the backup left open whether
player-space movement/rotation should be coupled into the same between-phase
proxy/constraint update path. This remains a candidate only if runtime evidence
shows world-direction or player-rotation-dependent grab drift.

## Items That Are Not Missing Anymore

- Ordinary dynamic one-hand grab no longer uses native mouse spring.
- Hidden no-contact proxy body exists.
- Proxy and constraint are flushed in the between-collide-and-solve phase.
- COM fallback for grip authority is rejected.
- Body B capture uses BODY, not MOTION/COM.
- Visual object to BODY relation is preserved.
- Solver-local constraint body relation is separate from visual BODY relation.
- Generic grab preserves object rotation and shifts selected point to palm.
- Authored node rotation override is explicit.
- Dynamic finger pose is mesh/contact based and stored.
- Loose non-equipped weapons have explicit dynamic loose-object tuning.
- Direct angular authority applies to the accepted held body set.
- Aggregate held body mass feeds motor budgeting.
- Long-object angular speed scaling exists.

## Items Explicitly Left Alone

- Missing generated weapon collider creation / reactivation.
- Equipped weapon collisions and equipped weapon grip handling.
- Two-hand equipped weapon behavior.
- Actor ragdoll grab.
- HIGGS keyframed object grab.
- Any branch/history operation.

## Honest Confidence

High confidence:

- native mouse spring production authority is removed;
- no-contact proxy exists;
- between-phase callback exists;
- pivot/reference frame no longer uses COM fallback;
- loose non-equipped weapon dynamic tuning exists;
- old INI pivot is not current production pivot A when palm anchor is live;
- the current source still uses `DriveToKeyFrame` for the hidden proxy.

Medium confidence:

- current linear atom runtime layout is acceptable for the enabled linear
  motors, because the backup's later Ghidra audit corrected the earlier
  absolute-offset conclusion.

Low confidence / needs runtime or Ghidra evidence before implementation:

- whether direct wrapper `setTransform` and `setVelocity` are always same-step
  solver-visible in the production between callback;
- whether replacing `DriveToKeyFrame` with direct transform/velocity will remove
  residual proxy stutter without losing the proven palm-collider convention;
- whether held-object motion-property leases improve weight feel without
  causing cap/snap side effects.

