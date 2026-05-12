# Dynamic Grab Custom Authority Plan

Date: 2026-05-12
Branch: `feature/ghidra-grab-motor-mapping`
Status: Implementation in progress; Phase 0 validated; ordinary dynamic grab is being moved to proxy custom authority

This document supersedes scattered implementation planning notes for the next dynamic grab redesign. The evidence tracker remains
`docs/custom-dynamic-grab-authority-current-findings-2026-05-12.md`; this file turns those findings into a concrete plan.

## Direct Answer

We have enough information to plan the new grab architecture.

We do not yet have enough to safely implement the full replacement without first satisfying the validation gates in this document. The remaining unknowns are not broad design unknowns. They are specific FO4VR/hknp integration gates:

- whether wrapper `BethesdaPhysicsBody::setTransform(...)` / `setVelocity(...)` direct-call inside the actual between-collide-and-solve callback or queue globally;
- which hidden no-contact proxy filter policy is safest in production;
- which minimal proxy shape is valid and cheap;
- how two-hand total force is budgeted so two hands add stability without making the player infinitely strong.

Those gates were answered by the Phase 0 runtime probe before replacement work began. The active implementation now follows the validated proxy authority path below.

## 2026-05-12 Implementation Tracking

This section records the implementation direction currently being applied so compaction does not erase the intended architecture.

- Pre-replacement marker commit exists: `e4c2db0 feature/grab-authority: mark pre mouse spring replacement milestone`.
- Added `src/physics-interaction/grab/GrabAuthorityProxy.h` as the shared production utility for the hidden proxy body:
  - no-contact filter is native noncollidable layer plus confirmed suppression bit 14;
  - proxy shape is the same tiny tetrahedral convex hull pattern validated by Phase 0;
  - proxy transform conversion uses the same Ni-row to Havok-column convention as the Phase 0 probe.
- Added `HeldObjectDriveMode::ProxyConstraint`.
- Added per-hand proxy authority state:
  - hidden `BethesdaPhysicsBody` proxy;
  - owning `bhkWorld`/`hknpWorld` pointers for lifecycle validation;
  - proxy-local pivot A;
  - active object body-local pivot B computed from raw-hand relation;
  - queued game-frame target/errors consumed by physics-step flush;
  - release-pending flag for deterministic cleanup on physics drive failure.
- Ordinary dynamic loose-object and loose non-equipped weapon grab creation now targets `createProxyConstraintGrabDrive(...)` instead of `_nativeGrab.create(...)`.
- Joining a peer-held loose object uses the same proxy constraint path for the joining hand. The peer promotion helper now treats an existing `ProxyConstraint` as already promoted.
- Game-frame `updateHeldObject(...)` queues raw hand proxy target and current error values for `ProxyConstraint`; it does not write the constraint/motors directly.
- `PhysicsInteraction::driveGrabAuthorityPhase0ProbeFromBetweenStep(...)` now flushes both hands' custom grab authority before running the optional Phase 0 probe.
- `Hand::flushPendingCustomGrabAuthority(...)` drives the proxy body with direct `setTransform`/`setVelocity`, then refreshes transform A/B and linear/angular motor values in the same between-collide-and-solve callback.
- Release now destroys the custom constraint before destroying the proxy body, and reset/world-loss paths abandon proxy state together with native/action/constraint state.
- Native mouse spring code remains present as a verified boundary and fallback/diagnostic wrapper, but ordinary production dynamic grab authority is no longer supposed to create `_nativeGrab`.

Current source-test policy being updated:

- native wrapper tests must keep verifying offsets, cinfo layout, target packing, mutex discipline, and fallback flush handling;
- production dynamic grab tests must reject ordinary `_nativeGrab.create(...)`;
- production dynamic grab tests must require `createProxyConstraintGrabDrive(...)`, `queueProxyGrabAuthorityTarget(...)`, `flushPendingCustomGrabAuthority(...)`, and between-phase coordinator wiring.

## Scope

Target:

- ordinary dynamic one-hand grab;
- loose non-equipped objects;
- loose non-equipped weapons;
- dynamic pull / converge / touch grab / held / release behavior;
- finite force, mass, inertia, angular authority, collision response, deviation, and hand-pose polish;
- future dynamic two-hand loose-object support with one hand joining an already held loose object.

Non-target for this plan:

- keyframed object grab as a production target;
- actor ragdoll grab;
- equipped weapon handling;
- equipped two-hand weapon handling;
- native Fallout grab behavior as a design target.

HIGGS equipped two-hand weapon code is useful only as a boundary/polish reference. It is not the model for loose-object two-hand dynamic grab.

## Locked Design Rules

- COM is never grip pivot authority.
- COM is allowed only for mass, inertia, lever length, release velocity, swing feel, and diagnostic data.
- The object-side pivot comes from contact/mesh/authored grip evidence.
- The hand-side pivot comes from palm/contact hand space.
- Object rotation is preserved at grab capture unless a deliberate authored grip override exists.
- The held relation is hand-relative and contact-relative, not COM-relative.
- One held body must have one authority path per hand. Native mouse spring and custom motors must not fight the same body.
- The generated palm/contact body remains useful for detection and contact evidence, but should not be reused as the future solver authority body unless the phase/timing problem is explicitly solved.
- Mouse spring can remain as a diagnostic/reference path during development, but it must not be the conceptual model of the final custom authority grab.

## Evidence Base

### HIGGS source references

- `Hand::TransitionHeld` in `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp:1301`
- HIGGS dynamic held update in `hand.cpp:3799` through the constraint update around `hand.cpp:3999`
- `CreateGrabConstraint` in `src\RE\havok.cpp:126`
- `FingerAnimator::Update` and `FingerAnimator::SetFingerValues` in `src\finger_animator.cpp`
- `TransitionHeldTwoHanded` in `hand.cpp:1712`, treated as equipped-weapon boundary evidence only
- HIGGS config defaults in `include\config.h:204` through `include\config.h:234`

### ROCK source references

- `HeldObjectDriveMode` in `src\physics-interaction\hand\Hand.h`
- `Hand::createConstraintGrabDrive` in `src\physics-interaction\hand\HandGrab.cpp:2040`
- `Hand::updateConstraintGrabDriveTarget` in `src\physics-interaction\hand\HandGrab.cpp:2123`
- `Hand::updateConstraintGrabDriveMotors` in `src\physics-interaction\hand\HandGrab.cpp:2172`
- `Hand::promoteHeldObjectToConstraintDrive` in `src\physics-interaction\hand\HandGrab.cpp:2255`
- one-hand dynamic grab creation path in `src\physics-interaction\hand\HandGrab.cpp:3980`
- held update path in `src\physics-interaction\hand\HandGrab.cpp:4360`
- `Hand::flushPendingHeldNativeGrab` in `src\physics-interaction\hand\HandGrab.cpp:4957`
- `PhysicsInteraction::makeGrabSharedObjectContext` in `src\physics-interaction\core\PhysicsInteraction.cpp`
- `PhysicsInteraction::attemptSelectedGrab` peer promotion path in `src\physics-interaction\core\PhysicsInteraction.cpp`
- finger/contact math in `src\physics-interaction\grab\GrabFinger.h` and `src\physics-interaction\grab\GrabContact.h`

### FO4VR binary findings from the current tracker

These are copied as plan inputs from the current Ghidra-backed tracker. They should stay linked to that tracker for evidence details.

- `bhkWorld::vfunction43` at `0x141DF73A0` runs step phases in this order:
  before-whole, before-any, collide, between-collide-and-solve, local command drain, solve, after-any, after-whole.
- `0x141DFE010` / `0x1415433B0` cover collide-side work and leave the hknp mutate guard before the between callback.
- `0x141DFE1E0` / `0x1415435E0` cover solve-side work and enter after the between callback plus local command drain.
- `0x1415395E0` is the direct set body transform path.
- `0x141539F30` is the direct set body velocity path.
- `0x141DF55F0` and `0x141DF56F0` are direct-or-queued wrapper paths that check TLS byte `+0x1528`.
- `0x141DFC8D0` queues through a global/thread queue that is not proven to be the same local command list drained immediately before solve.
- `0x141DF5930` is the command-safe `ApplyHardKeyFrame` wrapper; it computes velocity but does not by itself prove same-frame body pose replacement in the constraint row builder.
- `0x14153A6A0` computes hard-keyframe linear/angular velocity from target transform and dt.
- `0x141E086E0` `DriveToKeyFrame` includes cap-triggered transform snap/zeroing and is not the desired hidden proxy drive loop.
- `0x141DF5CB0` is the set-body-keyframed wrapper used by generated body lifecycle.
- FO4VR confirms hknp filter bit 14 suppresses contacts before the normal layer matrix; production no-contact policy is still unresolved.

## HIGGS Dynamic Grab Pipeline

### Detection and candidate selection

HIGGS finds candidates from close/far object search, collision/geometry data, and selected body evidence. The important dynamic-grab behavior is not the search mechanism itself; it is what happens after a candidate is selected:

- the selected body is not assumed to be the object root;
- mesh/skin triangles are gathered from the object tree;
- the closest graphics/mesh point to the palm ray can override the raw physics hit point;
- `GetRigidBodyToGrabBasedOnGeometry` can choose the rigid body from geometry/bone weighting instead of simply using a root or COM owner.

ROCK already has a richer FO4VR detection layer than HIGGS:

- near/far selection;
- ray/sphere casts;
- HMD cone gate;
- target kind classification;
- semantic hand/finger contacts;
- contact patch and multi-finger grip evidence;
- peer-held close fallback for the second hand.

The future redesign should preserve ROCK detection and improve authority/held behavior, not replace detection with HIGGS 1:1.

### Transition into dynamic held state

HIGGS `TransitionHeld` performs a large atomic capture:

- validates selected object/node/body;
- converts keyframed bodies to dynamic;
- reads mass for haptics and later force policy;
- starts nearby damping;
- chooses `ptPos` from closest physics point, then overrides it with closest graphics triangle point when available;
- computes finger values from nearby triangles;
- preserves the object rotation;
- translates the object target so `ptPos` seats into `palmPos`;
- computes `desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform`;
- collects all connected grabbed rigid bodies;
- creates the grab constraint if physics grab is active;
- saves connected-body contact listener state;
- normalizes/saves inertia state;
- resets hand deviation history.

The key rule is that HIGGS captures the relationship once, then updates it consistently. It does not recapture the grip pivot from COM during held state.

### Pivot and reference-frame model

Confirmed HIGGS model:

- `ptPos = closestPoint / havokWorldScale`.
- If graphics geometry hit exists, `ptPos = triPos`.
- `desiredNodeTransform = adjustedTransform`.
- `desiredNodeTransform.pos += palmPos - ptPos`.
- `desiredNodeTransformHandSpace = inverse(handWorld) * desiredNodeTransform`.
- Constraint pivot A is `palmPos` transformed into the hand-body local frame.
- Constraint pivot B is `ptPos` transformed into the grabbed body local frame.
- During held update, HIGGS recomputes transform B translation from the desired hand transform in object/body space and palm position.

That is why COM is not a fallback pivot. COM can affect how hard the object is to move, but it is not where the hand is attached.

### Constraint and motor behavior

HIGGS `CreateGrabConstraint` creates a custom constraint with:

- body A = hand body;
- body B = grabbed object body;
- transform A = palm/hand pivot frame;
- transform B = selected object contact frame;
- active motors;
- target relative orientation set from transform B.

Held update then rewrites:

- target relative orientation;
- transform B translation;
- linear motor tau/damping/recovery/max force;
- angular motor tau/damping/recovery/max force.

Mass and collision behavior:

- loose generic object linear force starts from `grabConstraintLinearMaxForce`;
- loose weapon linear force starts from `grabConstraintLinearMaxForceWeapon`;
- angular max force is derived from linear max force divided by `grabConstraintAngularToLinearForceRatio`;
- startup can fade angular force from `grabConstraintFadeInStartAngularMaxForceRatio` toward the normal ratio;
- collision uses `grabConstraintCollidingLinearTau` and `grabConstraintCollidingAngularTau`;
- tau approaches target via `AdvanceFloat(..., grabConstraintTauLerpSpeed)`;
- final linear force is capped by `mass * grabConstraintMaxForceToMassRatio`;
- final angular force is capped by the resulting linear force divided by the current angular-to-linear ratio.

This is the HIGGS weight effect. Heavy objects are not made realistic by moving the grip to COM. They are made realistic by finite force, angular force budgeting, inertia normalization, collision-aware response, and visible deviation/lag.

### Hand visual deviation

HIGGS does not visually pin the hand perfectly to the controller while the held object lags. In dynamic held state:

- `heldTransform = collidableNode->m_worldTransform`;
- `m_adjustedHandTransform = heldTransform * inverse(desiredNodeTransformHandSpace)`;
- startup lerps from real hand to adjusted hand;
- hand deviation is measured as distance between adjusted hand and real controller hand;
- deviation is averaged over a small history deque;
- if average deviation exceeds the max after ignore windows, the object is dropped/released.

This is the polish layer that makes finite force visible. If the object lags because it is heavy, the visual hand follows the object relation rather than making the held object look glued to an infinitely strong controller.

### Hand pose and fingers

HIGGS dynamic hand pose is geometry/math driven:

- object triangles are gathered and transformed into the adjusted object pose;
- closest graphics point to palm ray is selected;
- nearby triangles are filtered around that point;
- each finger has a start point, normal, and zero-angle vector in hand space;
- those are transformed to world space from the hand transform;
- `palmToPoint` is added so finger intersection is solved as if the hand is already seated at the object point;
- intersections against nearby triangles produce curl values;
- alternate thumb curve can replace the standard thumb curve;
- values are clamped so fingers do not overcurl;
- `FingerAnimator::SetFingerValues` blends from current local transforms toward target finger curves using configured linear/angular speeds.

ROCK already has a more FO4VR-native finger stack:

- live root-flattened hand/finger snapshot;
- triangle-driven pose solve;
- local transform overrides through FRIK;
- optional surface aim;
- selected-close pre-curl;
- multi-finger contact evidence.

The plan should preserve that stack and use HIGGS only to validate the conceptual sequence: seat palm to selected point, solve pose from nearby object geometry, then blend.

### Loose weapons

In HIGGS dynamic held state, loose weapons are still dynamic objects but receive different force policy:

- base form weapon branch increases linear max force;
- angular force is still derived from linear force and ratio;
- mass cap still applies;
- the pivot remains contact/palm based, not weapon COM based.

For ROCK, loose non-equipped weapons must remain separate from equipped weapon behavior. They should use the dynamic loose-object path with weapon-specific motor tuning and long-object lever behavior.

## ROCK Current Dynamic Grab Map

### What ROCK already does correctly

- Keeps a captured grab frame with:
  - raw hand space;
  - constraint hand space;
  - body local transform;
  - object-side body-local pivot;
  - hand-side palm/body-local pivot;
  - contact/mesh/finger evidence telemetry.
- Preserves a body-composed target frame for the native one-hand path.
- Has a three-phase acquisition/converge/touch framework.
- Has finger/contact math that exceeds the old HIGGS mesh-only approach.
- Has player-space compensation and held body velocity sampling.
- Has nearby damping, hand collision suppression, held-body collision state, and release cleanup.
- Has a custom grab constraint scaffold:
  - `createConstraintGrabDrive`;
  - `updateConstraintGrabDriveTarget`;
  - `updateConstraintGrabDriveMotors`;
  - `GrabMotionController`;
  - mass force cap;
  - angular-to-linear force ratio;
  - collision tau;
  - tau lerp;
  - startup force fade;
  - loose weapon multipliers.
- Has external visual hand transform support through FRIK tag `ROCK_GrabVisual`.

### Current authority split

Current ordinary one-hand loose grab:

- sets `_heldDriveMode = HeldObjectDriveMode::NativeMouseSpring`;
- creates `_nativeGrab`;
- queues target in `Hand::updateHeldObject`;
- flushes native action in `Hand::flushPendingHeldNativeGrab`;
- uses mouse-spring response/clamp config for the drive.

Current shared/two-hand loose-object path:

- second hand can acquire peer-held close selection;
- joining hand creates `SharedConstraint`;
- peer hand is promoted from native mouse spring to `SharedConstraint`;
- both hands use current custom constraint drive after promotion.

This explains the current feel:

- one-hand is stable because mouse spring is native, damped, and dt-aware;
- one-hand feels too strong because it lacks the finite-force HIGGS policy surface;
- the custom finite-force policy exists but is mostly reserved for shared/two-hand path;
- the old custom attempt stuttered because it changed authority without solving body-A ownership and physics-phase timing.

## Why The Previous Replacement Failed

The bad replacement failed for architectural reasons, not because custom motors are impossible.

Failure causes:

- COM/mass concepts leaked into grip authority. COM must never choose pivot or target frame.
- The current `SharedConstraint` path uses the generated/contact hand body as body A. That body is not a proven same-phase solver authority anchor for ordinary one-hand held-object drive.
- `updateConstraintGrabDriveTarget` and `updateConstraintGrabDriveMotors` currently run from held-object/game update flow, not the verified between-collide-and-solve point where the constraint solver will consume rows.
- Switching ordinary one-hand grabs to the existing `SharedConstraint` path as-is repeats the same body-A/timing problem.
- Mouse spring and motor authority can fight if both are left active on the same body.
- Multipart state and cleanup become fragile if the implementation does not centralize ownership of connected bodies, collision restore, inertia restore, and final-object release.

Correct conclusion:

- Do not tune random values.
- Do not recenter on COM.
- Do not simply flip one-hand from `NativeMouseSpring` to current `SharedConstraint`.
- Build a custom authority unit where body A, target timing, motor writes, visual lag, and cleanup are one coherent system.

## Target Architecture

### Authority unit

The future dynamic grab authority unit should contain:

- one hand-owned hidden no-contact proxy body A;
- one held dynamic object body B;
- one custom finite-force constraint from proxy body A to object body B;
- one captured grip frame for that hand;
- one per-physics-step target snapshot derived from raw/root-flattened hand data;
- one visual hand state derived from the actual held object pose;
- one cleanup owner for the constraint and proxy;
- one registration path for mass/player-space/collision state.

For two hands on one loose object:

- each hand owns its own proxy body A;
- each hand owns its own constraint to the same object body B;
- the held object has one shared object coordinator that owns connected body state, mass registration, inertia restore, and final release decisions.

### Hidden proxy body A

The proxy exists to solve the old stutter/timing issue:

- it is not a visible hand collider;
- it is not semantic contact evidence;
- it is not registered as a hand/body/weapon contact metadata body;
- it has no contact response with the world;
- it is keyframed/controlled from the raw root-flattened hand frame;
- it is driven in the physics phase that feeds the solver.

Expected lifecycle:

- create with `BethesdaPhysicsBody::create(...)` style ownership, not raw hknp slot writes;
- create as non-static from the start so it has valid motion;
- apply keyframed motion type once after creation;
- give it a minimal valid hknp shape;
- assign no-contact filter from birth;
- add to world through Bethesda wrappers so back pointers and teardown are valid;
- destroy through the normal physics-system removal path.

### Proxy drive

The researched best candidate is:

- sample/prepare raw root-flattened hand target in game update;
- in between-collide-and-solve, update proxy transform and velocity;
- update constraint transform/motor fields in the same phase;
- let solve consume proxy body A, object body B, and the freshly written atom/motor data.

Do not use `DriveToKeyFrame` as the hidden proxy steady drive because the mapped path can snap/zero velocity when caps are exceeded.

Pure `ApplyHardKeyFrame` velocity is also not enough by itself because current evidence does not prove the row builder substitutes the target pose for body A in the same frame.

The plan is transform plus velocity before solve, with direct-call validation.

### Constraint target policy

Each hand constraint must preserve:

- pivot A = captured palm/contact point in proxy/hand local space;
- pivot B = captured contact/mesh/authored point in object body local space;
- angular target = captured object-to-hand relation;
- transform B translation update = HIGGS-style dynamic pivot update from the desired body-in-hand relation and palm point.

No target calculation should use COM as pivot authority.

### Force policy

Per hand, per step:

- compute position error at the captured grip point;
- compute rotation error from desired object/body relation;
- read held body mass;
- apply base finite linear force;
- apply mass cap: `linearMaxForce = min(baseLinearMaxForce, mass * forceToMassRatio)`;
- derive angular force from linear force and angular-to-linear ratio;
- apply startup angular fade;
- lerp tau toward collision or non-collision targets;
- apply loose weapon multipliers separately from equipped weapon logic;
- keep both min/max force symmetric.

For two-hand:

- keep each hand finite;
- cap total object authority so two hands do not double into superman force;
- allow independent pivots to create natural torque;
- avoid artificial COM torque except for mass/inertia/lever calculations.

### Visual hand and pose

Physical authority:

- raw/root-flattened hand drives proxy body A.

Visual authority:

- actual held object/body pose plus captured desired relation drives `ROCK_GrabVisual`;
- deviation is measured between visual adjusted hand and real/raw hand;
- release happens if averaged deviation exceeds configured threshold after ignore windows.

Finger pose:

- capture/solve dynamic finger target from contact/mesh/triangle evidence at grab;
- blend through FRIK local transform overrides;
- keep selected-close pre-curl and acquisition pose independent of drive mode;
- do not require `HIGGS_R/L` or `ROCK_R/L` authored nodes for ordinary dynamic grab.

## Implementation Plan

Phase 0 is now implemented as disabled-by-default diagnostics. It does not replace the current one-hand native mouse-spring dynamic grab path. Later phases still require runtime log review before any grab authority replacement.

### Phase 0 implementation record

Implemented files:

- `src/physics-interaction/native/PhysicsStepDriveCoordinator.h`
- `src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp`
- `src/physics-interaction/native/HavokPhysicsTiming.h`
- `src/physics-interaction/native/HavokTlsDiagnostics.h`
- `src/physics-interaction/native/HavokTlsDiagnostics.cpp`
- `src/physics-interaction/native/GrabAuthorityPhase0Probe.h`
- `src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp`
- `src/physics-interaction/core/PhysicsInteraction.h`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/RockConfig.h`
- `src/RockConfig.cpp`
- `data/config/ROCK.ini`
- `tests/GrabAuthorityPhase0ProbeSourceTests.ps1`

Runtime enablement keys, all under `[PhysicsInteraction]`:

- `bDebugGrabAuthorityPhase0ProbeEnabled=false`
- `bDebugGrabAuthorityPhase0SolverProbeEnabled=true`
- `iDebugGrabAuthorityPhase0ProxyFilterPolicy=0`
- `iDebugGrabAuthorityPhase0LogIntervalFrames=30`
- `fDebugGrabAuthorityPhase0MotionAmplitudeGameUnits=8.0`

Implemented diagnostic behavior:

- Adds a real `betweenCollideAndSolve` callback surface and an after-solve callback surface.
- Creates a hidden keyframed proxy body and optional hidden dynamic receiver body only when explicitly enabled.
- Uses Bethesda body lifecycle wrappers for create/destroy.
- Uses a minimal native convex shape.
- Applies the selected no-contact filter policy from birth.
- Creates an isolated custom grab constraint between proxy and receiver when solver probe is enabled.
- Drives proxy transform and velocity from the between-collide-and-solve callback.
- Reads proxy transform/filter in the same callback to test setter/readback behavior.
- Reads proxy and receiver after solve to test solver response.
- Logs Havok TLS command-mode byte `+0x1528`, physics-context byte `+0x1529`, and thread command index `+0x152C` when readable.
- Checks whether proxy or receiver body ids appear in current semantic hand/weapon contact ids.

Phase 0 does not:

- touch `HandGrab.cpp`;
- replace ordinary one-hand dynamic grab authority;
- disable or alter native mouse-spring flush;
- use COM as pivot or target authority;
- register probe bodies as semantic contact evidence;
- create a production fallback path.

### Phase 0 runtime log review - 2026-05-12

Reviewed log:

- `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log`

Confirmed from the 17:42-17:46 run:

- `GrabPhase0` emitted 838 lines.
- 4 proxy/receiver create events were paired by 4 destroy events.
- 415 `between` samples and 415 `afterSolve` samples were logged.
- There were no `GrabPhase0` error lines.
- There were no `setTransform=fail`, `setVelocity=fail`, `readback=fail`, `proxyRead=fail`, or `receiverRead=fail` lines.
- There were no `semanticLeak=yes` lines.
- There were no `noContact=no`, `proxyNoContact=no`, or `receiverNoContact=no` lines.
- The active gameplay grab path stayed on `drive=nativeMouseSpring`; Phase 0 did not replace active grab.
- Runtime filter policy was `nonCollidable+bit14`, filter `0x000B400F`.
- TLS command mode read back as readable and `tlsCmd=queue`; same-callback transform readback still returned `posErr=0.0000` and `rotErr=0.000`, so the wrapper state is queue-mode while the body readback path sees the written transform immediately.

Observed diagnostic bug and fix:

- `afterSolve` periodically reported `targetProxyErr=5.5097` and `receiverLagFromProxy=6.0490`.
- This was not accepted as solver evidence because the probe target used a hard `sequence % 240` base-phase wrap while Y and Z used non-integer multipliers (`0.7`, `0.37`), creating a false target discontinuity every 240 physics ticks.
- The probe target was corrected to use a continuous base phase and per-component `std::fmod(...)` only at the final trigonometric input.
- `GrabAuthorityPhase0ProbeSourceTests.ps1` now rejects `sequence % 240` in the probe target so this diagnostic artifact does not return.

Current interpretation:

- Proxy lifecycle, no-contact filtering, semantic invisibility, and same-callback setter/readback are validated by the log.
- Solver-consumption quality must be re-sampled after the continuous-target fix before using `receiverLagFromProxy` as a production design signal.
- The log does not show evidence that Phase 0 touched or degraded active mouse-spring dynamic grab behavior.

### Phase 0 - Validation gates and diagnostics

Purpose: prove the future drive surface before replacing the working native one-hand path.

Required outcomes:

- Add runtime telemetry that records TLS `+0x1528` direct/queue state from the actual future between callback.
- Prove whether `BethesdaPhysicsBody::setTransform(...)` and `setVelocity(...)` direct-call in that phase.
- Pick proxy no-contact policy:
  - layer 15;
  - ROCK layer plus hknp bit 14;
  - or dedicated ROCK extended no-contact layer with zero matrix row.
- Pick minimal proxy shape and verify it can be created, keyframed, hidden, no-contact, activated, and destroyed cleanly.
- Verify no proxy body appears in semantic contact records, hand collision evidence, held object connected body sets, or weapon/body metadata.

Gate to continue:

- proxy can be created/destroyed without visible collision or stale contacts;
- proxy transform and velocity can be applied before solve in a proven same-step path;
- no hidden proxy state leaks into detection or cleanup.

### Phase 1 - Physics-step authority surface

Purpose: move custom grab authority writes to the same solver-consumed phase.

Planned changes:

- extend `PhysicsStepDriveCoordinator` with a real between-collide-and-solve callback surface;
- add a central dynamic-grab physics-step driver that can update both hands and shared object state;
- keep game update responsible only for target snapshots and high-level state transitions;
- keep physics update responsible for proxy pose/velocity and constraint atom/motor writes.

Gate to continue:

- logs prove ordering: collide complete, proxy/constraint writes applied, local command behavior known, solve begins afterward.

### Phase 2 - Hidden proxy lifecycle

Purpose: replace generated contact hand body as solver authority body A.

Planned changes:

- introduce a hand-owned proxy body lifecycle object;
- create proxy through Bethesda physics-system ownership;
- apply keyframed motion type once;
- keep proxy no-contact from birth;
- expose body id and world transform for constraint creation;
- destroy with normal world/lifecycle cleanup.

Gate to continue:

- repeated grab/release/world-loss does not leak bodies, collision objects, filters, or motion entries.

### Phase 3 - Captured frame and target snapshot

Purpose: make the proxy follow the correct hand convention without using FRIK visual lag as feedback.

Planned changes:

- snapshot raw/root-flattened hand frame before visual overrides;
- store previous proxy target for velocity calculation;
- compute proxy transform from the same palm frame used to capture pivot A;
- keep `rawHandSpace`, `constraintHandSpace`, `bodyLocal`, `pivotAHandBodyLocalGame`, and `pivotBBodyLocalGame` as the authority frame data;
- forbid COM from this path by source tests.

Gate to continue:

- controller translation and rotation move proxy axes correctly;
- axes match the known working flattened-root conventions;
- no visual hand lag feeds back into next-frame physical target.

### Phase 4 - Custom constraint authority unit

Purpose: make one-hand use the finite-force custom authority with the correct body A and update phase.

Planned changes:

- refactor `createConstraintGrabDrive` into a reusable authority-unit creator that accepts body A explicitly;
- body A for the new path is the hidden proxy, not `_handBody`;
- body B remains the held object primary dynamic body;
- move `updateConstraintGrabDriveTarget` atom writes into the physics-step authority driver;
- move `updateConstraintGrabDriveMotors` into the same phase;
- keep existing math helpers where conventions are proven correct.

Gate to continue:

- one-hand dynamic object can be held with custom finite force and no native mouse spring active;
- no per-frame constraint rebuild;
- no motor enable/disable churn that clears solver history.

### Phase 5 - One-hand replacement

Purpose: replace ordinary one-hand mouse-spring authority only after the custom authority unit passes gates.

Planned changes:

- ordinary loose object and loose non-equipped weapon grabs create a custom authority unit;
- `_nativeGrab` becomes disabled for production ordinary dynamic grab, or remains only diagnostic behind explicit dev-only tooling;
- native mouse-spring tests/docs are rewritten so they no longer require ordinary one-hand native authority;
- fallback to native authority should not exist silently. If a temporary comparison path is kept during development, it must be explicit and documented.

Gate to continue:

- generic object edge/corner grip stays pivoted at the selected contact point;
- heavy object lags under force instead of snapping to hand;
- visual hand lag reflects object deviation;
- release velocity remains believable.

### Phase 6 - Two-hand loose-object custom authority

Purpose: support two hands on the same loose object without reusing equipped weapon logic.

Planned changes:

- introduce a shared held-object coordinator keyed by held reference/body set;
- second hand acquisition keeps current peer-held close selection behavior but creates its own proxy and constraint;
- peer hand no longer needs native-to-shared promotion if one-hand is already custom authority;
- each hand constraint has independent pivot A/B and captured relation;
- total force budget is capped at object level;
- releasing one hand removes only that hand's proxy/constraint;
- final hand release owns final object cleanup and restore.

Gate to continue:

- two hands produce natural torque through separated pivots;
- releasing either hand does not drop collision/inertia state for the other hand;
- total authority does not make heavy objects weightless;
- loose weapon long-object swing is stable without artificial locking.

### Phase 7 - Mass, inertia, collision, and multipart state

Purpose: reproduce HIGGS weight behavior using FO4VR-native body sets.

Planned changes:

- centralize connected-body collection and restore ownership;
- keep current ROCK body scan/lifecycle snapshots;
- register selected, connected, and contained body mass once per object coordinator;
- dedupe player-space motion compensation by motion index;
- keep inertia normalization/restore object-scoped, not hand-scoped;
- keep contact listener/collision state object-scoped with per-hand query flags;
- do not include hidden proxies in connected body sets.

Gate to continue:

- multipart weapons/objects keep collisions after release;
- final release restores all filters, flags, inertia, and collision state;
- peer hand release cannot restore state still needed by the remaining hand.

### Phase 8 - Visual hand and finger polish

Purpose: make the finite-force behavior visible and polished instead of just numerically correct.

Planned changes:

- keep raw hand as physical input;
- compute visual hand from actual held body/object pose plus inverse captured relation;
- publish through `ROCK_GrabVisual`;
- average deviation and release/drop if exceeded;
- keep acquisition finger pose and final held pose independent of drive mode;
- preserve root-flattened live finger math and FRIK local transform override;
- add telemetry for visual hand deviation, object grip error, finger solve source, and pose blend state.

Gate to continue:

- heavy swing visibly pulls the hand/object relation without breaking wrist axes;
- no feedback loop between visual hand override and physical target;
- fingers blend onto the object from geometry/contact evidence.

### Phase 9 - Release and failure cleanup

Purpose: make the new system safe under failure, world loss, peer release, and grab cancellation.

Planned changes:

- release removes constraint before proxy body destruction;
- final-object release restores collision, filters, body flags, inertia, and damping;
- non-final peer release keeps shared object state active;
- failure during creation rolls back in reverse order;
- world-loss abandon clears proxies and constraints without touching unrelated objects.

Gate to continue:

- failed proxy creation cannot leave held object collision suppressed;
- failed constraint creation cannot leave proxy bodies alive;
- release after native path removal still composes controller/object release velocity correctly.

### Phase 10 - Config, tests, and diagnostics

Purpose: make the behavior tunable and prevent regressions.

Planned changes:

- replace ordinary one-hand native mouse-spring config as production authority config;
- keep finite-force config surfaces:
  - linear tau;
  - angular tau;
  - collision tau;
  - tau lerp;
  - damping;
  - max force;
  - force-to-mass ratio;
  - angular-to-linear force ratio;
  - startup angular fade;
  - loose weapon multipliers;
  - two-hand total force cap;
  - max deviation and deviation time.
- add source policy tests:
  - COM never feeds pivot/target frame;
  - ordinary one-hand dynamic grab does not require native mouse spring;
  - proxy bodies are no-contact and not semantic contact bodies;
  - constraint target/motor writes are owned by the physics-step driver.
- add math/unit tests:
  - selected grip point seats to palm;
  - object rotation preservation;
  - transform B dynamic translation;
  - angular target convention;
  - mass force cap;
  - angular force ratio;
  - collision tau lerp;
  - startup fade;
  - two-hand total force cap.
- add runtime diagnostics:
  - proxy direct/queued setter state;
  - proxy transform and velocity;
  - body A/body B ids;
  - grip error;
  - visual hand deviation;
  - held object collision state;
  - connected body restore count;
  - per-object total force cap.

## Concrete Parity Gaps To Close

One-hand:

- native mouse spring still owns ordinary held-body drive;
- custom finite-force motors are not active for ordinary one-hand;
- custom target/motor writes are not in the proven solver-consumed phase;
- body A is currently the generated contact hand body in the shared constraint path;
- visual hand lag exists but cannot fully express weight while native authority follows too strongly.

Two-hand:

- current two-hand loose-object path depends on promoting native one-hand to shared constraint;
- force budget is per-hand, not centrally object-capped;
- lifecycle restore depends on peer/final release correctness and needs a shared coordinator for the new architecture;
- equipped weapon two-hand logic must not be confused with loose-object dynamic two-hand.

Loose weapons:

- ordinary loose weapons currently use native one-hand response tuning;
- long-object lever behavior needs finite angular force and inertia policy;
- weapon-specific tuning must stay on loose dynamic path, not equipped weapon path.

FO4VR integration:

- same-step proxy drive must be proven;
- wrapper direct/queue behavior must be measured in the real callback;
- no-contact proxy policy must be finalized;
- local command ABI is mapped enough to understand risk, but not enough to use casually.

## Implementation Stop Conditions

Stop and research again if any of these happen:

- proxy setter calls queue globally in the between callback;
- proxy appears in contact evidence or held object connected body sets;
- constraint rows consume stale body-A pose after the intended drive phase;
- two-hand force cap cannot be reasoned object-wide;
- visual hand override feeds back into physical hand target;
- any path uses COM as pivot/target fallback;
- release cleanup cannot prove final/non-final ownership.

## What We Can Do Next

The next work item is not another broad research pass. It is a controlled implementation design review against this plan:

- accept or modify the architecture;
- decide whether Phase 0 diagnostics are allowed as implementation;
- then implement Phase 0 only, with no grab behavior replacement yet.

After Phase 0 proves the FO4VR drive surface, the one-hand replacement becomes a staged implementation instead of another guess.
