# HIGGS Custom Motor Parity Research

Date: 2026-05-12

## Decision Note

This report maps HIGGS custom grab motors as source-fed behavior because the desired ROCK replacement is not a single motor object replacement. HIGGS gets smooth physics grab behavior by feeding a complete 6-DOF constraint, stable local-space targets, per-frame motor retuning, collision softening, mass and inertia safety, hand visual smoothing, release velocity handling, and player-space compensation into one held-object system. The parity target for ROCK should therefore be the full data contract around the motors, not only the `hkpPositionConstraintMotor` field values.

## Evidence Rules

- Information source for HIGGS behavior: local HIGGS source at `E:\fo4dev\skirymvr_mods\source_codes\higgs`.
- Information source for current ROCK parity comparison: local ROCK source in this workspace.
- Existing Markdown files were not used as source material.
- Web sources were not used.
- Ghidra was not used for this pass.
- HIGGS is used here because the user explicitly requested it for motor parity research.

## Main Finding

HIGGS feeds its motors through a custom `GrabConstraintData` class that derives from `hkpConstraintData`. The constraint atom chain has one local-transform atom, one setup-stabilization atom, one ragdoll motor atom for all three angular axes, and three linear motor atoms for the three linear axes.

The motors themselves are all `hkpPositionConstraintMotor` objects:

- one angular motor instance shared by ragdoll motor axes 0, 1, and 2.
- one linear motor instance shared by linear motor axes 0, 1, and 2.

The linear motor atom target positions remain `0.0f`. HIGGS does not drive linear target by changing `m_targetPosition` per axis. It drives the linear goal by updating the constraint local transforms, especially `transformB.m_translation`, so the solver keeps the hand pivot and object pivot coincident.

The angular target is fed through `m_ragdollMotors.m_target_bRca`, updated every held frame from the desired object-to-hand orientation.

## Source Files Mapped

| File | Role |
| --- | --- |
| `include/constraint.h` | Custom constraint data layout, runtime layout, atom chain, solver-result count |
| `src/constraint.cpp` | Motor allocation, atom initialization, target writes, enable/disable, refcount handling |
| `src/RE/havok.cpp` | `CreateGrabConstraint`, world insertion, body assignment, constraint priority |
| `src/hand.cpp` | Grab creation, per-frame target update, force/tau retuning, collision softening, release handling |
| `include/config.h` | Default tuning values used by motors and held-object support systems |
| `src/config.cpp` | INI registration names for those tuning values |
| `src/math_utils.cpp` | `AdvanceFloat`, transform conversion helpers |
| `src/main.cpp` | player-space compensation and nearby damping restore |
| `src/physics.cpp` / `include/physics.h` | collision listener used to soften motor tau |

## HIGGS Constraint Data Layout

Class:

- `GrabConstraintData : public hkpConstraintData`

Key source:

- `include/constraint.h`
- `src/constraint.cpp`

### Atom Chain

`GrabConstraintData::Atoms` contains:

| Order | Atom |
| --- | --- |
| 1 | `hkpSetLocalTransformsConstraintAtom m_transforms` |
| 2 | `hkpSetupStabilizationAtom m_setupStabilization` |
| 3 | `hkpRagdollMotorConstraintAtom m_ragdollMotors` |
| 4 | `hkpLinMotorConstraintAtom m_linearMotor0` |
| 5 | `hkpLinMotorConstraintAtom m_linearMotor1` |
| 6 | `hkpLinMotorConstraintAtom m_linearMotor2` |

The atom chain is passed to Havok through:

- `m_atoms.getAtoms()`
- `m_atoms.getSizeOfAllAtoms()`
- `hkpConstraintData_getConstraintInfoUtil(...)`

### Runtime Layout

`GrabConstraintData::Runtime` contains:

| Field | Source comment offset | Role |
| --- | --- | --- |
| `m_solverResults[6]` | `0x00` | solver result slots for 3 angular and 3 linear motors |
| `m_initialized[3]` | `0x30` | angular motor initialization bytes |
| `m_previousTargetAngles[3]` | `0x34` | angular previous target history |
| `m_initializedLinear[3]` | `0x40` | linear motor initialization bytes |
| `m_previousTargetPositions[3]` | `0x44` | linear previous target history |

Solver-result enum:

| Slot | Meaning |
| --- | --- |
| `SOLVER_RESULT_MOTOR_0` | angular motor 0 |
| `SOLVER_RESULT_MOTOR_1` | angular motor 1 |
| `SOLVER_RESULT_MOTOR_2` | angular motor 2 |
| `SOLVER_RESULT_MOTOR_3` | linear motor 0 |
| `SOLVER_RESULT_MOTOR_4` | linear motor 1 |
| `SOLVER_RESULT_MOTOR_5` | linear motor 2 |

Runtime info reported by HIGGS:

- `m_numSolverResults = 6`
- `m_sizeOfExternalRuntime = sizeof(Runtime) * 2`

HIGGS comments that `sizeof(Runtime)` and 16-byte alignment were not enough without crashing, so it deliberately gives Havok twice the runtime memory.

Parity requirement:

- ROCK must preserve the full runtime history contract for all six motor axes.
- If FO4VR/hknp requires a different offset encoding than HIGGS hkp, that must be verified against the binary before changing offsets. The HIGGS source-level intent is absolute `offsetof(Runtime, ...)` use.

## HIGGS Atom Initialization Data

### Local Transforms Atom

Constructor defaults:

- `m_transformA.setIdentity()`
- `m_transformB.setIdentity()`

Creation-time writes:

- `setInBodySpace(transformA, transformB)`

Creation source:

- `CreateGrabConstraint(...)` in `src/RE/havok.cpp`

`transformA` is hand-body local. `transformB` is object-body local.

### Ragdoll Motor Atom

Constructor data:

- `m_isEnabled = false`
- `m_initializedOffset = offsetof(Runtime, m_initialized)`
- `m_previousTargetAnglesOffset = offsetof(Runtime, m_previousTargetAngles)`
- `m_target_bRca.setIdentity()`
- `m_motors[0] = nullptr`
- `m_motors[1] = nullptr`
- `m_motors[2] = nullptr`

Motor assignment:

- `setMotor(0, angularMotor)`
- `setMotor(1, angularMotor)`
- `setMotor(2, angularMotor)`

Runtime target updates:

- `setTargetRelativeOrientationOfBodies(...)`
- writes `m_ragdollMotors.m_target_bRca`

Target math:

```cpp
hkMatrix3_setMul(m_atoms.m_ragdollMotors.m_target_bRca, bRa, m_atoms.m_transforms.m_transformA.m_rotation);
```

Since HIGGS creation gives `transformA` identity rotation in the main grab path, this is effectively the desired hand transform in object/body space. If `transformA` is not identity, HIGGS includes it in the target.

### Linear Motor Atoms

Each linear atom starts disabled, has an axis index, has runtime offsets, has target position zero, and receives the shared linear motor pointer.

Linear atom 0:

- `m_isEnabled = false`
- `m_motorAxis = 0`
- `m_initializedOffset = offsetof(Runtime, m_initializedLinear[0])`
- `m_previousTargetPositionOffset = offsetof(Runtime, m_previousTargetPositions[0])`
- `m_targetPosition = 0.0f`
- `m_motor = nullptr`

Linear atom 1:

- `m_motorAxis = 1`
- initialized/runtime offsets use array index 1.
- `m_targetPosition = 0.0f`

Linear atom 2:

- `m_motorAxis = 2`
- initialized/runtime offsets use array index 2.
- `m_targetPosition = 0.0f`

Motor assignment:

- `setMotor(3, linearMotor)`
- `setMotor(4, linearMotor)`
- `setMotor(5, linearMotor)`

Important parity point:

- HIGGS does not update `m_targetPosition` every frame.
- It keeps linear target position at zero and moves the local constraint frames so zero means "pivot A and pivot B coincide."

## HIGGS Motor Objects

Motor class:

- `hkpPositionConstraintMotor`

Allocation:

- `hkAllocReferencedObject<hkpPositionConstraintMotor>()`

Constructor:

- `hkpPositionConstraintMotor_ctor(motor)`

Lifetime:

- HIGGS calls `setMotor(...)`, which adds a reference to the motor.
- HIGGS then calls `hkReferencedObject_removeReference(motor)` to drop the local creation reference.
- `GrabConstraintData::~GrabConstraintData()` calls `setMotor(index, nullptr)` for all six motor slots, which removes references.

This is not raw pointer ownership. Parity requires refcount-correct motor attachment or an FO4VR-safe equivalent.

### Angular Motor Initial Data

The angular motor is one `hkpPositionConstraintMotor` shared by all three ragdoll motor axes.

Initial fields:

| Field | Value source | Default value in HIGGS config |
| --- | --- | --- |
| `m_tau` | `grabConstraintAngularTau` | `0.03` |
| max force | `grabConstraintLinearMaxForce / grabConstraintAngularToLinearForceRatio` | `2000 / 12.5 = 160` |
| `m_proportionalRecoveryVelocity` | `grabConstraintAngularProportionalRecoveryVelocity` | `2.0` |
| `m_constantRecoveryVelocity` | `grabConstraintAngularConstantRecoveryVelocity` | `1.0` |
| `m_damping` | `grabConstraintAngularDamping` | `0.8` |

### Linear Motor Initial Data

The linear motor is one `hkpPositionConstraintMotor` shared by all three linear motor atoms.

Initial fields:

| Field | Value source | Default value in HIGGS config |
| --- | --- | --- |
| `m_tau` | `grabConstraintLinearTau` | `0.03` |
| max force | `grabConstraintLinearMaxForce` | `2000` |
| `m_proportionalRecoveryVelocity` | `grabConstraintLinearProportionalRecoveryVelocity` | `2.0` |
| `m_constantRecoveryVelocity` | `grabConstraintLinearConstantRecoveryVelocity` | `1.0` |
| `m_damping` | `grabConstraintLinearDamping` | `0.8` |

The constructor path calls `setMaxForce(...)`. The held update path later writes `m_minForce = -m_maxForce` explicitly every frame.

## Creation-Time Data Fed To The Constraint

Primary creation path:

- `Hand::TransitionHeld(...)`
- `CreateGrabConstraint(...)`

### Body Pair

HIGGS creates a constraint between:

| Constraint body | Source |
| --- | --- |
| body A | `handBody->hkBody` |
| body B | `selectedObject.rigidBody->hkBody` |

The hand body is the live physics hand. The selected body is the grabbed object body.

### Pivot A

Source world point:

- `palmPos`

Conversion:

- `hkPivotA = NiPointToHkVector(palmPos * havokWorldScale)`
- `pivotA = inverseTransform(bodyA->getTransform(), hkPivotA)`

Fed into:

- `handTransformHandSpace.pos = HkVectorToNiPoint(pivotA)`
- then `setInBodySpace(transformA, transformB)`

Main meaning:

- pivot A is the palm point expressed in hand-body local space.

### Pivot B

Source world point:

- `ptPos`, derived from selected closest/contact/grab point.

Conversion:

- `hkPivotB = NiPointToHkVector(ptPos * havokWorldScale)`
- `pivotB = inverseTransform(bodyB->getTransform(), hkPivotB)`

Fed into:

- `handTransformObjSpace.pos = HkVectorToNiPoint(pivotB)`
- then `setInBodySpace(transformA, transformB)`

Main meaning:

- pivot B is the selected grab point expressed in object-body local space.

### Desired Body Transform In Hand Space

HIGGS computes:

```text
desiredHavokTransformHandSpace =
    desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)
```

Then:

```text
handTransformObjSpace = inverse(desiredHavokTransformHandSpace)
handTransformObjSpace.pos = pivotB
```

This becomes constraint transform B at creation.

Parity requirement:

- ROCK must keep visual node, rigid body, and `bhkRigidBodyT` local transform conventions separated.
- The motor target should operate in body/constraint space, not visual-node space.

### Constraint Instance Data

HIGGS `CreateGrabConstraint` uses:

- `hkConstraintCinfo cinfo{}`
- `cinfo.priority = hkpConstraintInstance::PRIORITY_TOI`
- `cinfo.rigidBodyA = rigidBodyA`
- `cinfo.rigidBodyB = rigidBodyB`
- `cinfo.constraintData = new GrabConstraintData()`
- `constraintData->setInBodySpace(...)`
- `bhkGroupConstraint_ctor(...)`
- `constraint->collisionGroup = rigidBodyB->getCollidable()->getCollisionFilterInfo() >> 16`
- `constraintData->setTargetRelativeOrientationOfBodies(transformB.rotation)`
- `constraintData->setMotorsActive(constraint->constraint, true)`
- world insertion through `bhkWorld_AddConstraint(...)`
- hand body wrapper array insertion through `bhkRigidBody_AddConstraintToArray(...)`

Parity requirement:

- ROCK's hknp wrapper path must supply equivalent body ids, priority/solver behavior where possible, constraint data, atom enablement before solver build, and ownership/lifetime.

## Per-Frame Data Fed While Held

Primary update source:

- `Hand::Update` held-object section around the `grabConstraint` update.

### Angular Target Update

HIGGS computes every frame:

```text
desiredTransformHandSpace =
    desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(selectedObject.rigidBody)

desiredHandTransformHavokObjSpace =
    inverse(desiredTransformHandSpace)
```

Then:

```cpp
constraintData->setTargetRelativeOrientationOfBodies(
    NiMatrixToHkMatrix(desiredHandTransformHavokObjSpace.rot));
```

This updates `m_ragdollMotors.m_target_bRca`.

Parity requirement:

- Angular parity requires writing the desired body-to-hand orientation into `target_bRca` every held frame.
- This target must be built from the same grab frame and body-local transform used at creation.

### Linear Target Update

HIGGS computes every frame:

```text
newPivotB =
    (desiredHandTransformHavokObjSpace * palmPosHandspace)
    * collidableNode->m_worldTransform.scale
    * havokWorldScale
```

Then:

```cpp
constraintData->m_atoms.m_transforms.m_transformB.m_translation =
    NiPointToHkVector(newPivotB);
```

This is the key HIGGS linear motor feed.

It does not update:

- `m_linearMotor0.m_targetPosition`
- `m_linearMotor1.m_targetPosition`
- `m_linearMotor2.m_targetPosition`

Those stay at zero. The target is changed by moving transform B's local pivot translation.

Parity requirement:

- ROCK custom hkp motors need dynamic transform-B translation updates from the desired object/body-in-hand transform.
- If ROCK instead freezes transform B and only changes force/tau, the linear and angular goals can disagree.

### Motor Object Pointers Used During Update

HIGGS reads:

```cpp
hkpPositionConstraintMotor* linearMotor =
    (hkpPositionConstraintMotor*)constraintData->m_atoms.m_linearMotor0.m_motor;

hkpPositionConstraintMotor* angularMotor =
    (hkpPositionConstraintMotor*)constraintData->m_atoms.m_ragdollMotors.m_motors[0];
```

Because the same pointer was assigned to all three axes in each group, changing motor 0 updates all corresponding axes.

Parity requirement:

- Shared motor pointers across axes are part of HIGGS behavior.
- If ROCK uses separate motors per axis, values must be kept exactly synchronized unless a deliberate per-axis design replaces the HIGGS convention.

## Per-Frame Motor Retuning

HIGGS retunes the same two motor instances every held frame.

### Shared Retuned Fields

Each frame, HIGGS writes:

Angular motor:

- `m_tau`
- `m_minForce`
- `m_maxForce`
- `m_proportionalRecoveryVelocity`
- `m_constantRecoveryVelocity`
- `m_damping`

Linear motor:

- `m_tau`
- `m_minForce`
- `m_maxForce`
- `m_proportionalRecoveryVelocity`
- `m_constantRecoveryVelocity`
- `m_damping`

It always enforces:

```text
minForce = -maxForce
```

### Ordinary Non-Actor Object Tuning

For non-actor objects:

| Condition | Linear max force | Angular max force |
| --- | --- | --- |
| ordinary object | `grabConstraintLinearMaxForce` | linear max force divided by angular-to-linear ratio |
| weapon form | `grabConstraintLinearMaxForceWeapon` | linear max force divided by angular-to-linear ratio |

Defaults:

- ordinary linear max force: `2000`
- weapon linear max force: `9000`
- angular-to-linear ratio: `12.5`

Angular force examples:

- ordinary object: `2000 / 12.5 = 160`
- weapon: `9000 / 12.5 = 720`

### Startup Angular Force Fade

If HIGGS had to sync a custom initial transform, `fadeInGrabConstraint` is true.

Then it computes:

```text
elapsedTimeFraction = (currentFrameTime - heldTime) / grabConstraintFadeInTime
angularToLinearForceRatio =
    lerp(grabConstraintFadeInStartAngularMaxForceRatio,
         grabConstraintAngularToLinearForceRatio,
         min(1.0, elapsedTimeFraction))
```

Defaults:

- fade time: `0.1` seconds
- start angular ratio: `100`
- final angular ratio: `12.5`

Meaning:

- angular force starts much weaker and ramps up.
- linear force remains available while rotational snap is softened.

Parity requirement:

- Startup fade should primarily soften angular correction, not globally weaken the entire constraint unless intentionally changed.

### Collision Tau Softening

For non-actor objects:

```text
isColliding = entity collision listener has active collisions
angularTauTarget = isColliding ? collidingAngularTau : angularTau
linearTauTarget = isColliding ? collidingLinearTau : linearTau
```

Defaults:

- normal angular tau: `0.03`
- normal linear tau: `0.03`
- colliding angular tau: `0.01`
- colliding linear tau: `0.01`
- tau lerp speed: `0.5`

The current tau moves toward the target through:

```text
AdvanceFloat(current, target, speed)
step = speed * deltaTime
```

This is a rate limiter. HIGGS does not snap tau instantly between normal and collision values.

Parity requirement:

- Motor tau needs contact-aware softening and rate-limited recovery.
- Contact detection must be tied to held connected bodies, not only the primary body, because HIGGS adds listeners to all connected grabbed rigid bodies.

### Mass-Based Force Cap

For non-actor objects:

```text
mass = 1 / body->getMassInv()
linearMaxForce = min(linearMaxForce, mass * grabConstraintMaxForceToMassRatio)
angularMaxForce = min(angularMaxForce, linearMaxForce / angularToLinearForceRatio)
```

Default:

- force-to-mass ratio: `500`

Examples:

- 1 kg-ish body: linear cap `500`
- 4 kg-ish body: linear cap `2000`
- 20 kg-ish ordinary body: capped by configured `2000`, not mass cap

Parity requirement:

- Without this cap, light objects can get much more force than HIGGS would allow.

### Actor Tuning

For actors:

```text
physicsFPS = 1 / physicsDeltaTime
linearMaxForce = GetMaxForceForFPS(physicsFPS, fpsToActorMaxForceMultiplierMapLinear)
               * grabConstraintLinearMaxForceActor
angularMaxForce = GetMaxForceForFPS(physicsFPS, fpsToActorMaxForceMultiplierMapAngular)
                * grabConstraintAngularMaxForceActor
```

Defaults:

- actor linear force base: `2500`
- actor angular force base: `40`

Linear FPS multiplier map:

| FPS | Multiplier |
| --- | --- |
| 72 | `0.7` |
| 90 | `1.0` |
| 120 | `1.6` |
| 144 | `2.0` |

Angular FPS multiplier map:

| FPS | Multiplier |
| --- | --- |
| 72 | `0.5` |
| 90 | `1.0` |
| 120 | `1.375` |
| 144 | `1.5` |

If actor is ragdolled:

- angular tau target: `grabConstraintAngularTauBody`, default `0.65`
- linear tau target: `grabConstraintLinearTauBody`, default `0.8`
- tau starts at `0.01` for both axes and lerps to target over `physicsGrabLerpTauTimeBody`, default `0.1` seconds.

If actor is not ragdolled:

- angular tau: `grabConstraintAngularTauActor`, default `0.65`
- linear tau: `grabConstraintLinearTauActor`, default `0.8`

Parity requirement:

- Actor/ragdoll motor behavior is separate from object behavior. It should not reuse ordinary clutter tau/force values.

## Body Preparation Around Motors

HIGGS does several things before or around constraint creation that materially affect smoothness.

### Dynamic Motion And Quality

Before physics grab:

- keyframed selected body is converted to dynamic.
- debris quality can be converted to moving quality.
- impacted projectile filter/motion can be repaired.
- mass at grab time is captured.

Parity requirement:

- Custom motors must run against a dynamic body with suitable quality/filter state.

### Connected Body Discovery

HIGGS collects all rigid bodies connected to the grabbed object. It uses this set to:

- ignore collisions among connected grabbed bodies.
- add collision listeners.
- set contact point callback delay to zero.
- normalize inertia across the connected component.
- restore values on release.
- register player-space compensation.

Parity requirement:

- A single primary body is not enough for parity when the reference contains constraints, ragdolls, books, wheels, skull jaws, or other connected rigid bodies.

### Contact Point Callback Delay

For each connected grabbed body:

- saved original contact point callback delay.
- sets callback delay to `0`.
- restores original value on release unless the other hand still owns it.

This improves collision signal immediacy for motor softening.

Parity requirement:

- Held-object contact evidence must be frame-fresh. Delayed callbacks will delay tau softening.

### Inertia Normalization

For each connected grabbed body:

1. Save inverse inertia.
2. Compute `minInvInertia = min(x, y, z)`.
3. Clamp each inverse-inertia axis to `min(axis, minInvInertia * grabbedObjectMaxInertiaRatio)`.
4. Clamp each axis to `min(axis, 1 / grabbedObjectMinInertia)`.
5. Write back the adjusted inverse inertia vector.

Defaults:

- `grabbedObjectMaxInertiaRatio = 10`
- `grabbedObjectMinInertia = 0.01`

Meaning:

- HIGGS prevents extreme inertia anisotropy and extremely low inertia from making motor correction unstable.

Parity requirement:

- ROCK already has inertia normalization scaffolding, but parity requires applying it to the held connected body set, not only the one visible body.

### Nearby Body Damping

At grab start, HIGGS calls `StartNearbyDamping`.

It:

- queries closest points against the selected object.
- ignores disabled collision and ragdoll layers.
- checks nearby movable bodies inside `nearbyGrabBodyRadius`.
- only damps bodies with current velocity below linear/angular thresholds.
- saves original linear/angular damping.
- writes high temporary damping.
- restores after `grabFreezeNearbyVelocityTime`.

Defaults:

- nearby radius: `0.1` meters
- max nearby linear velocity: `0.2`
- max nearby angular velocity: `1.0`
- temporary linear damping: `500`
- temporary angular damping: `50`
- damping time: `0.1` seconds

Parity requirement:

- Smooth grab startup is not only motor tuning. HIGGS also suppresses small nearby-body chain reactions during the grab impulse window.

## Hand And Target Smoothing Around Motors

HIGGS uses visual/hand smoothing around the constraint.

### Start Hand Lerp

At grab creation:

```text
handSyncDistance =
    length(palmPos - ptPosInOriginalSpace) * havokWorldScale

lerpAmount =
    (handSyncDistance - physicsGrabLerpHandMinDistance)
    / (physicsGrabLerpHandMaxDistance - physicsGrabLerpHandMinDistance)

startGrabLerpHandDuration =
    clamp(lerp(minTime, maxTime, lerpAmount), minTime, maxTime)
```

Defaults:

- min time: `0.1`
- max time: `0.2`
- min distance: `0.1`
- max distance: `0.2`

While held, during that startup window:

- `m_adjustedHandTransform` is lerped from the real hand transform toward the object-matched hand transform.

Parity requirement:

- If ROCK replaces mouse spring with motors, it still needs startup visual/authority smoothing so the player does not see a hard hand/object snap.

### Deviation Guard

HIGGS tracks five frames of hand deviation:

- current deviation is distance between adjusted hand transform and real hand transform.
- average of five frames is used.
- if too large beyond an initial ignore window, HIGGS drops the object.

Defaults:

- max hand distance: `0.7` meters
- ignore after grab: `0.2` seconds
- ignore after sneak transition: `0.1` seconds

Parity requirement:

- A motor-only system needs an explicit escape condition for solver failure or blocked motion.

## Player-Space Compensation

HIGGS registers held bodies into a player-space compensation system each frame.

For non-actors:

- contained bodies and connected bodies are registered unless attached to a fixed body.

For actors:

- adjacent actor bodies around the grabbed body are registered with no warp.

The global update then:

- subtracts prior player-space velocity from registered movable bodies.
- computes current room/player delta velocity.
- either warps bodies on significant room rotation/snap-turn and recollides them, or adds the current player-space velocity.
- moves hand and weapon collision by the same room-space delta.

Parity requirement:

- Motors alone will stutter or lag during player motion if the held body and hand/body target do not share the same player-space compensation convention.

## Release Data

Although release is not motor input, it is part of HIGGS's perceived quality.

HIGGS stores five frames of selected-object local linear velocity:

```text
localLinearVelocity = objectLinearVelocity - g_prevDeltaVelocity
```

On release:

- gets the max velocity from the five-frame deque.
- if the max is not at an endpoint, averages the peak with its neighbors.
- optionally boosts if above throw threshold.
- adds player delta velocity with VRIK smoothing only.
- writes the object's linear velocity.
- for non-physics/keyframed path, computes controller-derived linear, angular, and tangential velocity.
- temporarily disables contact with the hand if release velocity is above threshold.
- removes the constraint and restores contact callback delays and inertia.

Parity requirement:

- A high-quality custom motor replacement should keep release velocity tracking independent of the instantaneous solver velocity, because solver correction velocity may not match intended throw velocity.

## Data ROCK Needs For HIGGS-Quality hkp Motor Parity

### Constraint Data Parity

ROCK needs:

- custom constraint data object with hkp-compatible vtable behavior.
- local transform atom.
- setup stabilization atom with valid defaults.
- ragdoll motor atom for 3 angular axes.
- 3 linear motor atoms for axes 0, 1, and 2.
- 6 solver result slots.
- external runtime memory large enough for all solver history.
- angular initialized offsets and previous target angle offsets.
- linear initialized offsets and previous target position offsets.
- atom enable flags set before the solver runtime is built.
- stable constraint lifetime and teardown.

### Motor Object Parity

ROCK needs:

- `hkpPositionConstraintMotor` for angular axes.
- `hkpPositionConstraintMotor` for linear axes.
- angular motor shared across angular axes unless deliberately diverging.
- linear motor shared across linear axes unless deliberately diverging.
- fields:
  - `tau`
  - `damping`
  - `proportionalRecoveryVelocity`
  - `constantRecoveryVelocity`
  - `minForce`
  - `maxForce`
- refcount-correct attachment or FO4VR-safe equivalent.
- min/max force symmetry maintained every frame.

### Target Frame Parity

ROCK needs:

- body A: physics hand body.
- body B: held object body.
- pivot A: palm point in hand-body local space.
- pivot B: selected grab point in object-body local space.
- desired body transform in hand-body space.
- `bhkRigidBodyT` local transform folded into body target math.
- angular target written to `target_bRca`.
- transform-B translation updated every frame from desired body-in-hand transform and palm local point.
- linear atom target position left at zero unless a new architecture explicitly changes the convention.

### Runtime Retuning Parity

ROCK needs:

- ordinary object force path.
- weapon force path.
- actor force path with FPS-scaled max force.
- ragdolled actor tau ramp.
- non-ragdolled actor tau values.
- collision tau softening.
- rate-limited tau movement through `speed * dt`.
- startup angular-force fade from high angular ratio to normal ratio.
- mass-based force cap.
- per-frame recovery/damping refresh.

### Held Body Support Parity

ROCK needs:

- dynamic body conversion and quality repair where applicable.
- connected-body discovery.
- contact listener registration for held connected bodies.
- contact callback delay override and restore.
- connected-body inertia normalization and restore.
- nearby-body temporary damping.
- player-space compensation.
- deviation guard.
- startup hand/visual smoothing.
- release velocity history and throw reconstruction.

## Current ROCK Parity Snapshot

Current ROCK already has several HIGGS-like pieces:

- `GrabConstraintMotorTuning` with separate linear/angular tau, damping, recovery, and force fields.
- a custom constraint with ragdoll motor atom plus three linear motor atoms.
- shared angular and linear `HkPositionMotor` pointers.
- motor enable-before-create behavior.
- dynamic transform-B translation update helper.
- per-frame motor update through `grab_motion_controller::solveMotorTargets`.
- mass force cap.
- angular-force startup fade.
- tau rate limiting.
- inertia normalization scaffolding.
- player-space held-object helpers.

Current parity risks to verify before making custom hkp motors the primary one-hand grab path:

- HIGGS source-level linear runtime offsets are based on `offsetof(Runtime, m_initializedLinear[axis])` and `offsetof(Runtime, m_previousTargetPositions[axis])`. ROCK currently writes a different-looking offset pattern for the three linear atoms. That may be an FO4VR/hknp atom convention, but it must be binary-verified before relying on it for primary grab.
- HIGGS reports 6 solver results and gives external runtime `sizeof(Runtime) * 2`. ROCK reports 12 solver results and `0x100` runtime. That may be deliberate FO4VR padding, but parity requires confirming the solver consumes the intended slots.
- HIGGS uses `PRIORITY_TOI` on the hkp constraint instance. ROCK's hknp `CreateConstraint` path may not expose the same priority. The hknp equivalent needs to be identified or the difference accepted explicitly.
- HIGGS has actor-specific FPS-scaled force maps. ROCK's generic adaptive motor controller is richer for error scaling, but actor/ragdoll parity should be added explicitly if actor grab parity matters.
- HIGGS contact softening is driven by contact listeners installed on all connected grabbed rigid bodies. ROCK should confirm the held-body collision signal covers the same connected set.
- HIGGS startup smoothing includes both angular force fade and hand visual/object sync lerp. ROCK has motor fade state, but the final primary motor path should confirm the visual authority and motor authority ramp together.
- HIGGS updates transform-B translation every held frame from the desired hand/object relationship. ROCK has this helper, and this should remain mandatory for custom hkp one-hand grabs.
- HIGGS nearby-body damping is part of grab startup quality. ROCK has nearby damping config, but parity requires it to fire before the first strong motor correction.

## Why HIGGS Motors Feel Smooth

HIGGS smoothness is explained by the surrounding data pipeline:

1. The solver gets a true 6-DOF constraint, not a velocity shove.
2. Linear target is frame-consistent: target position is zero and the local frames move to preserve pivot coincidence.
3. Angular target is frame-consistent: the desired relative orientation is written as `target_bRca`.
4. Force is bounded by mass and object type.
5. Angular force fades in to avoid pickup snap.
6. Tau softens during collision and returns through a rate limit.
7. Inertia is normalized so odd rigid bodies do not destabilize the solver.
8. Contact events are made immediate through callback-delay changes.
9. Nearby bodies are temporarily damped so grab startup does not excite a pile.
10. Player movement is compensated outside the motor, so the motor is not fighting room/player translation.
11. Hand visual sync has its own lerp and deviation guard.
12. Release velocity is reconstructed from motion history rather than trusting the final constraint correction velocity.

## Implementation Implication For ROCK

If ROCK replaces mouse spring with custom hkp motors for one-hand grab, the correct parity target is:

```text
custom 6-axis hkp constraint
+ HIGGS-style local frame target updates
+ HIGGS-style motor retuning
+ connected-body collision/inertia policy
+ player-space compensation
+ startup and release policy
```

A replacement that creates hkp position motors but omits dynamic transform-B translation, runtime offsets, collision tau softening, mass force caps, connected-body contact listeners, inertia normalization, and player-space compensation will not reproduce HIGGS quality.

## Concrete Parity Checklist

- [ ] Verify FO4VR/hknp runtime offset encoding for ragdoll and linear motor atoms before primary deployment.
- [ ] Verify custom constraint runtime solver-result count and external runtime size against actual FO4VR solver consumption.
- [ ] Keep linear target positions at zero unless changing the architecture deliberately.
- [ ] Update transform-B translation every held frame from desired body-in-hand transform.
- [ ] Update `target_bRca` every held frame from desired body-to-hand orientation.
- [ ] Share one angular motor across all angular axes and one linear motor across all linear axes, or synchronize per-axis motors exactly.
- [ ] Keep min force equal to negative max force every frame.
- [ ] Preserve normal object, weapon, actor, and ragdoll tuning branches.
- [ ] Preserve startup angular force fade.
- [ ] Preserve contact-driven tau softening and rate-limited tau recovery.
- [ ] Preserve mass-based force caps.
- [ ] Preserve connected-body inertia normalization.
- [ ] Preserve connected-body contact listener registration and callback-delay restore.
- [ ] Preserve nearby-body damping before first strong correction.
- [ ] Preserve player-space compensation for held body sets.
- [ ] Preserve startup hand/object visual sync smoothing.
- [ ] Preserve deviation release guard.
- [ ] Preserve release velocity history reconstruction.

## Bottom Line

HIGGS feeds a complete constraint state machine into its custom motors. The data that matters is the full system: six motor axes, runtime history, local target frames, dynamic transform-B translation, `target_bRca`, per-frame force/tau retuning, collision softening, mass and inertia safety, connected-body policy, player-space compensation, and release reconstruction.

For ROCK, hkp custom motors can replace mouse spring cleanly only if the implementation targets that full contract. The motor objects alone are not enough.
