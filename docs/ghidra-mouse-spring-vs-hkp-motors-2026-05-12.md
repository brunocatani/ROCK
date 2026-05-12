# Ghidra Mapping - hknpBSMouseSpringAction vs hkp Constraint Motors

Date: 2026-05-12

## Decision Note

This report is written as a Ghidra-only mapping because the grab architecture crosses two different Havok integration styles: an hknp action-style mouse spring currently visible in the Fallout 4 VR binary, and older hkp constraint motor infrastructure that requires constraint atoms, solver offsets, and motor-owned tuning data. The chosen format keeps mouse spring and motor evidence separate, records exact binary addresses, then compares requirements and limits from observed code paths rather than from prior notes. Existing project Markdown files were not used as information sources.

## Scope And Evidence Rules

- Source of information: Ghidra MCP only.
- Existing `.md` documents were not read as source material for this mapping.
- HIGGS source and documentation were not used.
- Web sources were not used.
- Claims below are tied to Ghidra symbols, decompiled functions, disassembly, strings, xrefs, or reflected Havok class metadata found in the loaded Fallout 4 VR program.
- Names beginning with `FUN_` are Ghidra-generated names unless a named class/function symbol was present.
- Field meanings are marked as inferred when they come from code behavior rather than an explicit reflected member name.

## Loaded Binary Context

Ghidra reported the selected program as a Fallout 4 VR style executable image with base segment at `0x140000000`.

Observed memory segments:

| Segment | Range |
| --- | --- |
| `.text` | `0x140001000` to `0x142c40bff` |
| `.rdata` | `0x142c4c000` to `0x1436f05ff` |
| `.data` | `0x1436f1000` to `0x14689e1af` |

The analysis below uses addresses from this loaded image.

## High-Level Result

The current in-binary grab-style path that matches ROCK's mouse spring grab is an hknp action path centered on `hknpBSMouseSpringAction::hknpBSMouseSpringAction` at `0x141e4a850` and its apply/update virtual path through `0x141e4a920` into `FUN_141e4aa30`.

The older linear/angular motor path is not the same kind of mechanism. Ghidra shows hkp motor classes as constraint motor data objects consumed by hkp constraint data and constraint atoms. They are solver-side components for specific constraint types, not a standalone per-body action equivalent to the mouse spring.

Practical distinction:

- Mouse spring needs a valid hknp body id, world pointer, target point/orientation data, tuning scalars, action registration, and per-frame target updates.
- hkp motors need a complete hkp constraint data object, local frames/axes/pivots, one or more motor objects, enabled motor atoms, solver result offsets, refcount-safe pointer ownership, runtime buffers, and constraint-world integration.

## Part 1 - hknpBSMouseSpringAction

### Symbol And Xrefs

Named function:

- `hknpBSMouseSpringAction::hknpBSMouseSpringAction` at `0x141e4a850`

Named class string:

- `.?AVhknpBSMouseSpringAction@@` at `0x1438b1988`

Constructor xrefs:

- `0x140f19f51` inside `FUN_140f19250`
- `0x140f1ca4a` inside `FUN_140f1bf70`

The two xrefs are creation paths that allocate an `0xe0` byte object, call the constructor, add the action to the hknp world/action system, and activate the target body.

### Object Size And Lifetime

Observed allocation size:

- `0xe0` bytes in both constructor callers.

Destructor/free virtual:

- Vtable slot 0 points to `0x141e4afc0`.
- `0x141e4afc0` installs an `hkBaseObject` vtable before freeing when the caller passes the low destructor flag.
- Free size defaults to `0xe0`, unless the short at `this + 0x0a` overrides it.
- Freeing goes through a TLS allocator path, matching Havok-style allocation ownership.

This means a compatible ROCK object cannot be treated as an arbitrary plain C++ struct with unmanaged ownership. The engine object has Havok-style ref/lifetime conventions.

### Constructor Layout

Constructor at `0x141e4a850` initializes:

| Offset | Observed write | Meaning |
| --- | --- | --- |
| `this + 0x00` | vtable `0x142e88f28` | hknpBSMouseSpringAction vtable |
| `this + 0x08` | `0xffff0001` | Havok-style refcount/type word |
| `this + 0x10` | `0` | cleared qword |
| `this + 0x18` | copied from `param + 0x90` | hknp body id |
| `this + 0x20` to `0xa0` | copied from parameter block in 16-byte chunks | vector/matrix action state |
| `this + 0xb0` | copied from global vector at `0x1438644b0` | previous/error vector initialization |
| `this + 0xc0` to `0xd0` | scalar/vector tail copied from `param + 0x94` to `0xa4` | tuning and activation fields |

Constructor copy pattern:

- `param + 0x00` -> `this + 0x20`
- `param + 0x10` -> `this + 0x30`
- `param + 0x20` -> `this + 0x40`
- `param + 0x30` -> `this + 0x50`
- `param + 0x40` -> `this + 0x60`
- `param + 0x50` -> `this + 0x70`
- `param + 0x60` -> `this + 0x80`
- `param + 0x70` -> `this + 0x90`
- `param + 0x80` -> `this + 0xa0`
- `param + 0x94` -> `this + 0xc0`
- `param + 0x98` -> `this + 0xc4`
- `param + 0x9c` -> `this + 0xc8`
- `param + 0xa0` -> `this + 0xcc`
- `param + 0xa4` -> `this + 0xd0`

Inferred fields from update behavior:

| Offset | Inferred role | Evidence |
| --- | --- | --- |
| `0x18` | hknp body id | body validation and world body-array lookup |
| `0x60` | orientation/rotation input | passed to `FUN_141722c10` before target solve |
| `0x90` | target world point | subtracted from body point result |
| `0xa0` | body-local grabbed point or anchor | passed to `FUN_1416dbe90` with the body |
| `0xb0` | previous positional error/state | written when update flag is enabled |
| `0xc0` | angular/correction cap scalar | participates in cap `constant * dt * scalar` |
| `0xc4` | position stiffness-like scalar | multiplied by `1 / dt` and position error |
| `0xc8` | velocity/damping feed scalar | multiplied by sampled correction/velocity vector |
| `0xcc` | velocity damping scalar | multiplies current linear and angular velocity |
| `0xd0` | activation/quality flag mask | passed to body activation when nonzero |

The names above are behavioral names, not official member names.

### Vtable

Mouse spring vtable address:

- `0x142e88f28`

Observed entries:

| Vtable offset | Target | Observed role |
| --- | --- | --- |
| `+0x00` | `0x141e4afc0` | destructor/free path |
| `+0x08` | `0x140027980` | generic Havok base/ref function |
| `+0x10` | `0x141540950` | generic Havok virtual |
| `+0x18` | `0x141540960` | generic Havok virtual |
| `+0x20` | `0x14005c120` | shared virtual |
| `+0x28` | `0x14005c2d0` | shared virtual |
| `+0x30` | `0x141e4a920` | action apply/update path |
| `+0x38` | `0x14005c290` | shared virtual |
| `+0x40` | `0x14005c2a0` | shared virtual |
| `+0x48` | `0x14005c2b0` | shared virtual |

The action-specific update path is the vtable entry at `+0x30`.

### Action Update Entry

Function:

- `0x141e4a920`

Observed behavior:

- Calls `FUN_141e4aa30(this, world, dt, 1)`.
- Returns `0`.

The final `1` means normal virtual updates store the current previous/error vector at the end of `FUN_141e4aa30`.

### Main Mouse Spring Solver Helper

Function:

- `FUN_141e4aa30` at `0x141e4aa30`

Observed parameters:

| Parameter | Inferred role |
| --- | --- |
| `param_1` | hknpBSMouseSpringAction pointer |
| `param_2` | hknp world pointer |
| `param_3` | delta time |
| `param_4` | whether to update previous/error state |

#### Body Validation

The helper exits if:

- Body id at `action + 0x18` equals `0x7fffffff`.
- Body id is outside the world body table count.
- Body flag byte at `body + 0x40` does not have `& 3`.
- Motion id at `body + 0x6c` equals `-1`.

Observed hknp layout conventions:

| Structure | Convention |
| --- | --- |
| body id invalid sentinel | `0x7fffffff` |
| world body table base | `world + 0x20` |
| world body count source | `(world + 0x2c) & 0x3fffffff` |
| hknp body stride | `0x90` bytes |
| body flag byte | `body + 0x40` |
| body motion id | `body + 0x6c` |
| world motion table base | `world + 0xe0` |
| hknp motion stride | `0x80` bytes |

The motion pointer is resolved from the body motion id and world motion base. If the body is static, destroyed, invalid, inactive in the table, or lacks a motion id, the action does no work.

#### World Locking

The helper uses:

- world lock structure around `world + 0x690`
- TLS byte at `+0x1529`
- `FUN_141df5fb0`
- `FUN_141df5fd0`

The exact lock object name is not proven from the symbol table, but the call pattern surrounds access to world/body/motion data. Any native use of this path has to respect hknp world locking/thread ownership rather than writing body data from arbitrary timing.

#### Core Position Error

Observed call:

- `FUN_1416dbe90(&local_e8, body, action + 0xa0)`

The result is compared to:

- `action + 0x90`

The position error vector is:

- `world point derived from body and action + 0xa0`
- minus target point at `action + 0x90`

In behavior terms, this is the spring's positional miss between the body anchor and the target anchor.

#### Target Orientation / Velocity Sampling

Observed calls:

- `FUN_141722c10(&local_f8, action + 0x60)`
- `FUN_14153a6a0(world, bodyId, action + 0x90, &local_f8, dt, &local_b8, &local_c8)`

Inferred role:

- `action + 0x60` provides a rotation/orientation-like input.
- `FUN_141722c10` normalizes or converts it into a vector/quaternion-like local value.
- `FUN_14153a6a0` computes target-related velocity/correction outputs used by the spring helper.

This path means the mouse spring is not based only on a target position. The update uses target point plus orientation-derived state and delta time.

#### Stability No-Op Threshold

When body flag `(body + 0x40) & 8` is clear, the helper checks component thresholds before applying any correction.

Observed constants:

- Position/error threshold global near `0x143864450`
- Velocity/correction threshold global near `0x1438644c0`

If all checked XYZ components are inside threshold, the function skips the expensive/apply path and goes to cleanup.

Practical effect:

- Very small deltas can be ignored.
- The action has an internal deadband or resting threshold.
- Fine hand jitter may not always result in body velocity changes.
- This can help stability but can also produce a small perceived lag or settled offset depending on target scale and tuning.

#### Velocity Damping

Observed behavior:

- Current motion linear velocity is read from `motion + 0x40`.
- Angular velocity is obtained through `FUN_1417d1710(motion)`.
- Both are multiplied by scalar at `action + 0xcc`.
- Result is written via `FUN_141539f30(world, bodyId, dampedLinear, dampedAngular)`.

Inferred meaning:

- `action + 0xcc` is a damping multiplier over current linear and angular motion.
- Values lower than `1.0` damp existing velocity.
- Values near or above `1.0` preserve or amplify existing velocity depending on exact scalar value.

#### Linear Correction Term

Observed arithmetic pattern:

- reciprocal delta time is computed with SIMD reciprocal refinement.
- position error is scaled by `action + 0xc4 * (1 / dt)`.
- target/sample vector from `local_b8` is scaled by `action + 0xc8`.
- the two are combined into a correction vector.

Inferred formula shape:

```text
correction ~= positionError * action[0xc4] / dt
           + sampledTargetVector * action[0xc8]
```

This is not a solver constraint impulse formula. It is an action-level drive term derived from target miss, target/sample velocity, and frame time.

#### Angular / Secondary Correction Cap

The function computes a vector length and clamps it against a cap shaped like:

```text
cap ~= constant * dt * action[0xc0]
```

Observed behavior:

- vector length squared is compared against cap squared.
- if over cap, the vector is normalized and scaled to the cap.
- rsqrt uses Newton-refinement style SIMD math.

Inferred meaning:

- `action + 0xc0` limits a correction magnitude, likely rotational/secondary correction.
- The cap is delta-time dependent.
- A lower cap limits aggressive rotation/correction.
- A high cap permits abrupt correction but can make contact interactions harsher.

#### World Writes / Application Calls

Observed apply calls:

- `FUN_141539f30(world, bodyId, dampedLinear, dampedAngular)`
- `FUN_141539d30(world, bodyId, &local_c8)`
- `FUN_14153a250(world, bodyId, &local_d8)`

Observed activation call:

- `FUN_14153c090(world, bodyId, action + 0xd0, 1)` when `action + 0xd0` is nonzero.

These calls show that the mouse spring applies to a single hknp body through world/body APIs, not through an hkp constraint instance pair.

#### Previous State Update

When the helper's final parameter is nonzero:

- current positional error/state is written to `action + 0xb0` through `0xbc`.

This allows temporal comparison on later frames. The vtable update path passes nonzero, so normal action updates keep that history.

### Creation Path `FUN_140f19250`

Constructor xref:

- `0x140f19f51`

Observed sequence:

1. Prepares an input block with vectors, target data, body id, and tuning.
2. Allocates `0xe0` bytes through a TLS allocator path.
3. Calls `hknpBSMouseSpringAction::hknpBSMouseSpringAction`.
4. Adds/registers the action through `FUN_14155b6d0`.
5. Activates the body through `FUN_14153c090(world, bodyId, 0x8000000, 1)`.
6. Stores action/body references into surrounding state.
7. Calls `FUN_140e08a80` as part of the outer interaction path.

This path includes unit/scale conversion. Target positions are multiplied by global `DAT_145b29178` before action creation, while distance comparison elsewhere references `DAT_1437cea5c`. Exact unit names are not proven by Ghidra symbols, but the pair behaves like game-to-Havok or inverse scale conversion.

### Creation Path `FUN_140f1bf70`

Constructor xref:

- `0x140f1ca4a`

Observed sequence:

1. Similar allocation and constructor call for an `0xe0` mouse spring action.
2. Registers the action with `FUN_14155b6d0`.
3. Activates the body through `FUN_14153c090`.
4. Stores the resulting action/body state in a related interaction state object.
5. Copies global values `DAT_145c616e0`, `DAT_145c616e4`, and `DAT_145c616e8` into nearby state offsets around `+0xfc0`.

This appears to be a second interaction/create path using the same action object and same hknp registration pattern.

### Mouse Spring Data Requirements

A complete compatible mouse spring setup requires:

| Requirement | Reason |
| --- | --- |
| hknp world pointer | update uses world body/motion tables and world locking |
| valid hknp body id | stored at action `+0x18`, sentinel is `0x7fffffff` |
| body with motion id | bodies with motion id `-1` are skipped |
| target world point | copied into action `+0x90`, used as positional target |
| body-local grabbed point or anchor | copied into action `+0xa0`, transformed against body |
| orientation/rotation input | copied near action `+0x60`, used in target sampling |
| stiffness-like scalar | action `+0xc4`, scales position error over `dt` |
| velocity-feed/damping scalar | action `+0xc8`, scales sampled target vector |
| existing-velocity damping scalar | action `+0xcc`, multiplies current linear/angular velocity |
| correction cap scalar | action `+0xc0`, limits secondary correction magnitude |
| activation mask/flag | action `+0xd0`, passed to body activation when nonzero |
| action registration | creation paths call `FUN_14155b6d0` |
| body activation | creation paths call `FUN_14153c090` after construction |
| per-frame target updates | the action follows changing target state; stale data means stale grab target |

### Mouse Spring Conventions

Observed conventions:

- Uses 16-byte vector copies and aligned SIMD loads/stores.
- Uses hknp body ids, not hkp rigid body pointers.
- Uses body id invalid sentinel `0x7fffffff`.
- Uses hknp body stride `0x90`.
- Uses hknp motion stride `0x80`.
- Uses Havok-style refcount/lifetime words at object `+0x08`.
- Uses world locking around body/motion access.
- Uses scaled Havok/world units in creation paths.
- Uses action update virtual at vtable `+0x30`.

### Mouse Spring Limitations

The mouse spring is a body action, not a hard pair constraint.

Observed and inferred limitations:

- It drives one hknp body toward a target; it does not intrinsically constrain two bodies together.
- It does not provide native angular limit cones, linear limit ranges, breakable motor limits, or pairwise solver rows visible in the hkp motor code paths.
- It depends on frame delta time and tuning scalars; changes in update cadence alter correction strength.
- Small movement can be swallowed by the internal no-op thresholds.
- It can fight contact resolution because it writes velocities/corrections on a body rather than representing a full joint relationship.
- It requires a body with a valid motion id; fixed/static/no-motion bodies are skipped.
- It needs correct world lock timing and action lifecycle handling.
- It depends on correct scale conversion before target data enters the action.
- High stiffness or cap values can create abrupt correction, while low values can feel soft or lagged.
- It has no visible built-in concept of per-axis motor limits in the mapped function.

The benefit is lower integration weight: it needs fewer objects and less solver-side configuration than hkp constraint motors.

## Part 2 - hkp Constraint Motors

### Motor Classes Found

Searches for motor symbols found:

| Class / Function | Address |
| --- | --- |
| `hkpPositionConstraintMotor::hkpPositionConstraintMotor` | `0x141e726d0` |
| `hkpSpringDamperConstraintMotor::hkpSpringDamperConstraintMotor` | `0x141e7c470` |
| `hkpVelocityConstraintMotor::hkpVelocityConstraintMotor` | `0x141e827e0` |
| `hkpCallbackConstraintMotor::hkpCallbackConstraintMotor` | `0x141e8e1c0` |
| `hknpRagdollMotorController::hknpRagdollMotorController` | `0x141af6f60` |

The hkp motor constructors are minimal thunks that return or install vtable addresses. They do not initialize all tuning fields.

### Motor Constructor Behavior

Observed constructors:

| Constructor | Behavior |
| --- | --- |
| `0x141e726d0` | returns vtable `0x142e95fe8` for `hkpPositionConstraintMotor` |
| `0x141e7c470` | returns vtable `0x142e9a4f8` for `hkpSpringDamperConstraintMotor` |
| `0x141e827e0` | returns vtable `0x142eac8e0` for `hkpVelocityConstraintMotor` |
| `0x141e8e1c0` | returns vtable `0x142ecd078` for `hkpCallbackConstraintMotor` |

Implication:

- Allocating and calling these constructors is not enough.
- The caller must populate motor fields such as force limits, stiffness/tau/damping, velocity target, recovery velocities, or callback pointers.
- Correct class size and member offsets matter.

### Base Motor Metadata

Reflection function:

- `FUN_142b3ee20`

Observed class:

- `hkpConstraintMotor`
- size `0x18`
- base class reference through `DAT_145b677a0`
- enum member `MotorType`

Base motor objects therefore have at least the Havok object header and motor type metadata before derived-class fields.

### Limited Force Motor Metadata

Observed strings:

- `hkpLimitedForceConstraintMotor`
- `minForce`
- `maxForce`

These field strings are referenced by runtime/reflection data around `0x143871450`, `0x143871538`, and `0x143871570`.

Implication:

- Force limits are part of the motor hierarchy.
- Any motor behavior that needs stable grab behavior through hkp motors must define force bounds, not only target values.

### hkpPositionConstraintMotor

Reflection functions:

- `FUN_142b2b140`
- `FUN_142b2b1b0`

Observed metadata:

- class size `0x30`
- base class `DAT_1465c80e0`
- member table `PTR_DAT_142e95ee0`
- 4 reflected members

Observed field strings:

- `tau`
- `damping`
- `proportionalRecoveryVelocity`
- `constantRecoveryVelocity`

Short Havok description strings near the member metadata say:

- `tau` controls stiffness/convergence speed.
- `damping` damps the motor.
- proportional recovery velocity scales with displacement from the target.
- constant recovery velocity is a displacement-independent recovery term.

Practical meaning:

- Position motors are target recovery motors.
- They need displacement information from a constraint axis or angle.
- They rely on solver integration through a constraint atom that knows the target angle/position and solver result offsets.

### hkpVelocityConstraintMotor

Reflection functions:

- `FUN_142b31620`
- `FUN_142b31690`

Observed metadata:

- class size `0x30`
- base class `DAT_1465c80e0`
- member table `PTR_DAT_142eac860`
- 3 reflected members

Observed field strings:

- `tau`
- `useVelocityTargetFromConstraintTargets`
- `velocityTarget`

Practical meaning:

- Velocity motors can drive toward a target velocity.
- They may use an explicit velocity target or derive a velocity target from constraint target changes.
- This is still constraint-context data, not a standalone body drive.

### hkpSpringDamperConstraintMotor

Reflection functions:

- `FUN_142b2c8e0`
- `FUN_142b2c950`

Observed metadata:

- class size `0x28`
- base class `DAT_1465c80e0`
- member table `PTR_s_springConstant_142e9a480`
- 2 reflected members

Observed field strings:

- `springConstant`
- `springDamping`

Practical meaning:

- Spring damper motors encode spring stiffness and damping directly.
- They still require a constraint atom and target context to convert spring state into solver rows.

### hkpCallbackConstraintMotor

Reflection function:

- `FUN_142b3a830`

Observed metadata:

- class size `0x48`
- base class `DAT_1465c80e0`
- enum `CallbackType`
- member table includes `callbackFunc`
- 5 reflected members

Practical meaning:

- Callback motors can defer behavior to a callback function.
- They require a correct callback signature and lifecycle. The mapped constructor alone does not reveal safe calling conventions for custom callbacks.

## Part 3 - hkp Linear And Angular Motor Atoms

The older "custom linear and angular motors" concept maps more closely to hkp constraint atoms than to the derived motor classes alone.

### hkpLinMotorConstraintAtom

Reflection function:

- `FUN_142b3c950`

Observed metadata:

- class `hkpLinMotorConstraintAtom`
- size `0x18`
- base class `DAT_1465d4c70`
- member table `PTR_s_isEnabled_142ed3f30`
- 6 reflected members

Observed related strings:

- `TYPE_LIN_MOTOR`
- `isEnabled`
- `motor`
- `targetPosition`
- `previousTargetPositionOffset`

Inferred role:

- Linear motor atom drives a linear constraint coordinate.
- It needs an enabled flag, a target position, previous target position/runtime offset, and a motor pointer.
- It depends on the surrounding constraint data's axis/frame atoms for coordinate definition.

### hkpAngMotorConstraintAtom

Reflection function:

- `FUN_142b3c4c0`

Observed metadata:

- class `hkpAngMotorConstraintAtom`
- size `0x28`
- base class `DAT_1465d4c70`
- member table `PTR_s_isEnabled_142ed4720`
- 8 reflected members

Observed related strings:

- `TYPE_ANG_MOTOR`
- `isEnabled`
- `motorAxis`
- `targetAngle`
- `initializedOffset`
- `previousTargetAngleOffset`
- `correspondingAngLimitSolverResultOffset`
- `motor`

Inferred role:

- Angular motor atom drives an angular coordinate along a motor axis.
- It needs a target angle, previous target angle offset, initialization offset, limit solver-result relationship, enabled flag, motor axis, and a motor pointer.
- It is tightly coupled to angular limit atoms if the corresponding limit solver result offset is used.

### Ragdoll Motor Atom

Observed enum string:

- `TYPE_RAGDOLL_MOTOR`

Ragdoll constraint data contains three motor pointer slots and associated angular limit state. The utility functions described below expose ragdoll motor assignment as three motor slots, not as one scalar target.

## Part 4 - Constraint Data That Owns Motors

### Limited Hinge Constraint Data

Constructor:

- `hkpLimitedHingeConstraintData::hkpLimitedHingeConstraintData` at `0x1419aca30`

Observed key writes:

| Offset | Observed value | Inferred role |
| --- | --- | --- |
| `this + 0xc8` | word `0x20` | atom/type or angular limit atom marker |
| `this + 0xfb` | `0` | boolean flag |
| `this + 0xfc` | `0xc0490fdb` | minimum angle, about `-pi` |
| `this + 0x100` | `0x40490fdb` | maximum angle, about `+pi` |
| `this + 0x104` | `0x3f800000` | limit/motor scalar default |
| `this + 0xeb` | word `0x100` | flag/offset data |
| `this + 0xc4` | `0x540050` | packed solver/runtime offsets |
| `this + 0xc2` | `0` | motor enabled flag |
| `this + 0xcc` | `0` | runtime/solver field |
| `this + 0xd0` | `0` | motor pointer |

Helper:

- `FUN_1419ac9f0`

Observed helper behavior:

- Writes solver/result offsets at `param + 0xa4`, `param + 0xa6`, and `param + 0xa8`.
- Initializes them to `0xffff` when disabled/unresolved.
- Writes packed values including `0x540050` and `0x20` when enabled/configured.

Limited hinge is important because hkp motor utilities treat constraint type `2` as having one motor pointer at `this + 0xd0`.

### Ragdoll Constraint Data

Constructor:

- `hkpRagdollConstraintData::hkpRagdollConstraintData` at `0x1419b1d50`

Observed key writes:

| Offset | Observed value | Inferred role |
| --- | --- | --- |
| `this + 0xc4` | `0x940090` | packed solver/runtime offsets |
| `this + 0xc2` | `0` | motor enabled flag |
| `this + 0x100` | `0` | motor slot 0 |
| `this + 0x108` | `0` | motor slot 1 |
| `this + 0x110` | `0` | motor slot 2 |
| `this + 0x138` | `0xbf060aa6` | angular limit default |
| `this + 0x13c` | `0x3f060aa6` | angular limit default |
| `this + 0x140` | `0x3f4ccccd` | angular limit softness/scalar default |
| `this + 0x158` | `0xc2c80000` | angular/friction default |
| `this + 0x15c` | `0x3f800000` | scalar default |
| `this + 0x160` | `0x3f4ccccd` | scalar default |
| `this + 0x156` | `0x38` after helper call | runtime offset/size flag |
| `this + 0x173` | word `0x100` | flag/offset data |
| `this + 0x175` | `1` | boolean flag |
| `this + 0x178` | `0xbccccccd` | limit/motor scalar |
| `this + 0x17c` | `0x3ccccccd` | limit/motor scalar |
| `this + 0x180` | `0x3f4ccccd` | limit/motor scalar |

Helper:

- `FUN_1419b2430`

Observed behavior:

- Writes `this + 0x156` to `0x38` when enabled, otherwise `0`.

Ragdoll is important because hkp motor utilities treat constraint type `7` as a three-motor-slot constraint.

### Prismatic Constraint Data

Constructor:

- `hkpPrismaticConstraintData::hkpPrismaticConstraintData` at `0x1419b1350`

Observed key writes:

| Offset | Observed value | Inferred role |
| --- | --- | --- |
| `this + 0x20` | word `2` | atom/type marker |
| `this + 0xb0` | word `0x0b` | linear motor-related atom type |
| `this + 0xb2` | `1` then later packed data | enabled/runtime data |
| `this + 0xb4` / `0xb6` | `0xffff` initially | solver/runtime offsets |
| `this + 0xc8` | word `0x0a` | related linear atom |
| `this + 0xd8` | word `0x0d` | related atom |
| `this + 0xe8` | word `0x07` | related atom |
| `this + 0xf8` | word `0x07` | related atom |
| `this + 0x108` | word `0x09` | related atom |
| `this + 0x10c` | `0xff7fffee` | linear limit/min default |
| `this + 0x110` | `0x7f7fffee` | linear limit/max default |

Helper:

- `FUN_1419b1320`

Observed helper behavior:

- Writes linear motor solver result offsets at `param + 0x94` and `param + 0x96`.
- Initializes them to `0xffff`.
- When enabled/configured, writes packed offset data including `0x540050`.

Prismatic is the clearest mapped owner of a linear motor atom. However, the generic motor utility mapped below does not expose prismatic as a supported type in its type switch. That means prismatic linear motors likely need direct constraint-specific setup or a different code path.

## Part 5 - Motor Set/Get Utilities And Runtime Use

### Constraint Data Utils - Get Motors

Function:

- `FUN_1418429d0`

Evidence:

- References source string `Data\hkpConstraintDataUtils.cpp`.
- References error string meaning the constraint type has no motors.

Observed behavior:

1. Calls virtual function at constraint data vtable `+0x20` to get a type id.
2. If type id is `2`, returns the one motor pointer at `param_2[0x1a]`, equivalent to object offset `+0xd0`.
3. If type id is `7`, calls ragdoll getter functions:
   - `FUN_1419b25d0`
   - `FUN_1419b2630`
   - `FUN_1419b2600`
4. For other types, logs an error and returns failure status.

Type interpretation from surrounding constructors:

- Type `2` maps to limited hinge style motor ownership.
- Type `7` maps to ragdoll style motor ownership.

### Constraint Data Utils - Set Motors

Function:

- `FUN_141842470`

Evidence:

- References source string `Data\hkpConstraintDataUtils.cpp`.
- Error line constant observed at `0x42`.

Observed behavior:

1. Calls virtual function at constraint data vtable `+0x20` to get type id.
2. If type id is `2`:
   - calls `FUN_1419acfd0` to set the limited hinge motor pointer.
   - calls `FUN_1419ad060` to enable/disable motor state and clear runtime buffer if provided.
3. If type id is `7`:
   - calls `FUN_1419b25c0`, `FUN_1419b25e0`, and `FUN_1419b2610` to set all three ragdoll motor slots.
   - calls `FUN_1419b2640` to enable/disable motor state and clear runtime buffer if provided.
4. Otherwise logs that the constraint type has no motors.

This utility is not a general "attach motor to anything" API. It supports specific constraint data type ids.

### Limited Hinge Motor Setter

Function:

- `FUN_1419acfd0`

Observed behavior:

- Increments the new motor reference count at motor `+0x08` when allowed by the short at `motor + 0x0a`.
- Decrements the old motor pointer at constraint `+0xd0`.
- If old reference count reaches zero, calls old motor virtual destructor through vtable `+0x18`.
- Stores the new motor pointer at constraint `+0xd0`.

This confirms motor pointer ownership is refcounted and cannot be replaced as a raw pointer without lifecycle handling.

### Limited Hinge Enable/Disable

Function:

- `FUN_1419ad060`

Observed behavior:

- Writes enabled state at object `+0xc2`.
- Writes another state byte at object `+0xea` as inverse of enabled.
- Clears four qwords in the provided runtime buffer when one is supplied.

Disabling a motor is not the same as clearing the pointer. The atom/constraint state is also toggled and runtime data is reset.

### Ragdoll Motor Slot Setter

Function:

- `FUN_1419b2520`

Wrappers:

- `FUN_1419b25c0` sets slot 0.
- `FUN_1419b25e0` sets slot 1.
- `FUN_1419b2610` sets slot 2.

Observed behavior:

- New motor pointer refcount is incremented.
- Old slot pointer refcount is decremented.
- Old pointer destructor is called if the count reaches zero.
- New pointer is stored at `constraint + 0x100 + slotIndex * 8`.

Getter functions:

- `FUN_1419b25d0` returns motor pointer at `+0x100`.
- `FUN_1419b2600` returns motor pointer at `+0x108`.
- `FUN_1419b2630` returns motor pointer at `+0x110`.

### Ragdoll Motor Enable/Disable

Function:

- `FUN_1419b2640`

Observed behavior:

- Writes enabled state at object `+0xc2`.
- Writes another state byte at object `+0x122` as inverse of enabled.
- Clears twelve qwords in the provided runtime buffer when one is supplied.

Ragdoll motor enable touches broader runtime state than limited hinge, matching its three-axis motor setup.

### hknpRagdollMotorController

Constructor:

- `hknpRagdollMotorController::hknpRagdollMotorController` at `0x141af6f60`

Runtime function:

- `FUN_141af7260`

Observed behavior in `FUN_141af7260`:

- Iterates through constraints/bones controlled by the controller.
- Checks weight/activity values before enabling motors.
- For constraint type `2`, if motor pointer at `+0xd0` exists:
  - calls `FUN_1419ad060` to enable/disable limited hinge motor state.
  - clears object `+0xcc` when enabling.
- For constraint type `7`:
  - calls `FUN_1419b2640` to enable/disable ragdoll motor state.
  - calls `FUN_1419b26c0` with a global vector at `0x143864460` when enabling.
- Tracks whether the controller remains active in a byte at controller `+0x40`.

This is evidence that hkp motor state is enabled and consumed as part of constraint/controller runtime, not as a free body action.

## Part 6 - hkp Motor Data Requirements

A complete hkp motor setup needs more data than mouse spring.

### Shared Requirements

| Requirement | Reason |
| --- | --- |
| Correct hkp constraint data type | motor utilities only mapped for type `2` and type `7`; prismatic uses a different path |
| Correct local frames | linear/angular coordinates depend on constraint frames |
| Correct pivot/axis data | motor axes and target positions/angles are local to the constraint |
| Motor object of correct derived type | position, velocity, spring-damper, or callback motor |
| Motor fields populated | constructors do not populate tuning |
| Force limits | base/limited force metadata exposes min/max force |
| Target angle or target position | required by angular/linear motor atoms |
| Previous target offset | atom metadata includes previous target position/angle runtime offsets |
| Solver result offsets | initialized as `0xffff` or packed runtime offsets; wrong offsets prevent proper solving |
| Enabled flags | motor atoms and constraint data have explicit enable state |
| Runtime buffer | enable/disable functions clear runtime qwords when supplied |
| Refcounted motor pointer ownership | setter functions increment/decrement and call destructors |
| Constraint instance/world registration | motors operate through constraints, not standalone body ids |
| Type-specific activation/update path | limited hinge, ragdoll, and prismatic differ |

### Linear Motor Requirements

From `hkpLinMotorConstraintAtom` and prismatic constructor evidence:

- linear motor atom present and enabled.
- motor pointer assigned.
- target position set.
- previous target position runtime offset set.
- corresponding solver result offsets set.
- local axis/frame atoms configured by the surrounding constraint.
- limits/friction atoms coordinated if present.

Linear motor state alone does not define where the body should go in world space. The constraint data around it defines the coordinate system.

### Angular Motor Requirements

From `hkpAngMotorConstraintAtom`, limited hinge, and ragdoll evidence:

- angular motor atom present and enabled.
- motor pointer assigned.
- motor axis set.
- target angle set.
- previous target angle runtime offset set.
- initialized offset set.
- corresponding angular limit solver result offset set when paired to limits.
- local frame basis configured.
- angular limits configured when using limited hinge/ragdoll.

Angular motor behavior is axis-specific. It is not equivalent to setting a world-space quaternion target unless the constraint frames and target-angle extraction are also correct.

## Part 7 - hkp Motor Limitations And Failure Modes

Observed and inferred limitations:

- Motor constructors do not fully initialize behavior. Missing fields produce undefined or inert tuning.
- Generic set/get utilities only mapped support constraint types `2` and `7`.
- Prismatic linear motor atoms exist, but the mapped generic utility does not set them through the same switch.
- Motor pointers are refcounted. Raw replacement risks leaks, dangling pointers, or premature destruction.
- Solver/runtime offsets are mandatory. Values like `0xffff`, `0x540050`, and `0x940090` are not optional decoration; they control runtime solver state placement.
- Enable flags exist at both atom/constraint levels. Pointer presence alone is not enough.
- Runtime buffers must be cleared on state changes to avoid stale solver history.
- Angular motors are local-axis/angle systems; world-space hand orientation must be converted into constraint-local target angles.
- Linear motors are local-coordinate systems; world-space hand position must be converted into constraint-local target position.
- Motor force limits must be tuned or the solver can be weak, overly aggressive, or unstable.
- hkp motor infrastructure is constraint-pair oriented; it is heavier than per-body hknp action driving.
- Mixing hkp-style motors with hknp body/action paths requires careful bridge knowledge. The mapped code shows both systems in the binary, but they do not share the same direct object model.

## Part 8 - Direct Comparison For Grab Architecture

| Topic | hknp Mouse Spring | hkp Linear/Angular Motors |
| --- | --- | --- |
| Core object | `hknpBSMouseSpringAction` | hkp constraint motor plus constraint atom/data |
| Main address | constructor `0x141e4a850`, update helper `0x141e4aa30` | motor constructors around `0x141e726d0`, `0x141e7c470`, `0x141e827e0`; utilities `0x141842470`, `0x1418429d0` |
| Integration style | per-body hknp action | solver-side constraint data |
| Body reference | hknp body id at action `+0x18` | constraint instance and hkp constraint data |
| Target data | target point, local anchor, orientation data | target position/angle/velocity inside atoms |
| Tuning | action scalars at `+0xc0`, `+0xc4`, `+0xc8`, `+0xcc` | motor fields: tau, damping, force limits, recovery velocities, spring constants, velocity targets |
| Limits | cap/deadband style in action helper | native linear/angular limits through constraint atoms |
| Solver coupling | writes through hknp body/world apply calls | integrated into hkp constraint solver rows |
| Data burden | moderate | high |
| Pair constraint | no direct pair semantics in mapped helper | yes, by constraint type |
| Lifecycle risk | action registration and world lock | constraint data, motor refcounts, runtime offsets, enable state |
| Best fit from observed code | grabbed object follows a moving hand target | constrained mechanical/joint behavior with defined axes and limits |

## Part 9 - Practical Interpretation For ROCK

Based only on the mapped binary behavior:

1. The mouse spring path is the engine-native hknp action-style way to pull a dynamic body toward a moving target.
2. It is comparatively direct for object grab because the data model starts from `world + body id + target`.
3. It has built-in damping, correction, cap, deadband, previous-state tracking, and body activation behavior.
4. It does not encode rich pairwise constraints, per-axis limits, or hard angular constraints.
5. The hkp motor path provides richer solver semantics but requires a complete constraint architecture around it.
6. Older custom linear/angular motors cannot be recreated safely by creating motor objects alone. They require matching hkp constraint data, atoms, offsets, target conversion, runtime buffers, and pointer ownership.
7. For hand grab, hkp motors would require a designed local constraint frame between grabbed object and controller/target proxy. Ghidra evidence does not show the motor classes accepting a world-space hand target directly.
8. The hkp path is more suitable when ROCK needs a true constrained relationship with local limits, axes, and solver row behavior. It is not a drop-in replacement for mouse spring's hknp action.

## Part 10 - Unknowns That Remain After This Pass

The following were not proven by this mapping and should not be treated as known facts:

- Exact official field names for every `hknpBSMouseSpringAction` member.
- Exact semantic names of `FUN_141539f30`, `FUN_141539d30`, `FUN_14153a250`, and `FUN_14153a6a0`.
- Whether Bethesda ships additional wrapper APIs that construct prismatic motors outside the mapped generic utility.
- Whether ROCK's current grab setup matches Bethesda's source layout byte-for-byte.
- Whether FO4VR exposes safe public F4SE/CommonLibF4VR declarations for these hknp action methods.
- Exact unit value represented by `DAT_145b29178` and `DAT_1437cea5c`, though usage strongly indicates scale conversion.

These unknowns do not invalidate the main conclusion: mouse spring and hkp motors are different integration architectures with different required data and failure surfaces.

## Address Appendix

### Mouse Spring

| Address | Meaning |
| --- | --- |
| `0x141e4a850` | `hknpBSMouseSpringAction::hknpBSMouseSpringAction` |
| `0x142e88f28` | hknpBSMouseSpringAction vtable |
| `0x141e4afc0` | destructor/free virtual |
| `0x141e4a920` | action update virtual, calls helper |
| `0x141e4aa30` | main mouse spring apply/helper function |
| `0x140f19250` | creation path using mouse spring action |
| `0x140f19f51` | constructor call inside `FUN_140f19250` |
| `0x140f1bf70` | second creation path using mouse spring action |
| `0x140f1ca4a` | constructor call inside `FUN_140f1bf70` |
| `0x1438b1988` | class string `.?AVhknpBSMouseSpringAction@@` |
| `0x1438644b0` | vector copied to action `+0xb0` on construction |
| `0x143864450` | position/error threshold vector used by helper |
| `0x1438644c0` | velocity/correction threshold vector used by helper |

### hkp Motor Classes

| Address | Meaning |
| --- | --- |
| `0x141e726d0` | `hkpPositionConstraintMotor::hkpPositionConstraintMotor` |
| `0x142e95fe8` | hkpPositionConstraintMotor vtable |
| `0x141e7c470` | `hkpSpringDamperConstraintMotor::hkpSpringDamperConstraintMotor` |
| `0x142e9a4f8` | hkpSpringDamperConstraintMotor vtable |
| `0x141e827e0` | `hkpVelocityConstraintMotor::hkpVelocityConstraintMotor` |
| `0x142eac8e0` | hkpVelocityConstraintMotor vtable |
| `0x141e8e1c0` | `hkpCallbackConstraintMotor::hkpCallbackConstraintMotor` |
| `0x142ecd078` | hkpCallbackConstraintMotor vtable |
| `0x142b3ee20` | reflection creation for `hkpConstraintMotor` |
| `0x142b2b140` | reflection creation for `hkpPositionConstraintMotor` |
| `0x142b31620` | reflection creation for `hkpVelocityConstraintMotor` |
| `0x142b2c8e0` | reflection creation for `hkpSpringDamperConstraintMotor` |
| `0x142b3a830` | reflection creation for `hkpCallbackConstraintMotor` |

### hkp Atoms And Constraint Data

| Address | Meaning |
| --- | --- |
| `0x142b3c950` | reflection creation for `hkpLinMotorConstraintAtom` |
| `0x142b3c4c0` | reflection creation for `hkpAngMotorConstraintAtom` |
| `0x1419aca30` | `hkpLimitedHingeConstraintData::hkpLimitedHingeConstraintData` |
| `0x1419ac9f0` | limited hinge solver offset helper |
| `0x1419b1d50` | `hkpRagdollConstraintData::hkpRagdollConstraintData` |
| `0x1419b2430` | ragdoll runtime offset helper |
| `0x1419b1350` | `hkpPrismaticConstraintData::hkpPrismaticConstraintData` |
| `0x1419b1320` | prismatic linear motor offset helper |
| `0x1418429d0` | hkpConstraintDataUtils get motor(s) |
| `0x141842470` | hkpConstraintDataUtils set motor(s) |
| `0x1419acfd0` | limited hinge motor pointer setter |
| `0x1419ad060` | limited hinge motor enable/disable |
| `0x1419b2520` | ragdoll motor slot setter |
| `0x1419b25c0` | ragdoll slot 0 setter wrapper |
| `0x1419b25e0` | ragdoll slot 1 setter wrapper |
| `0x1419b2610` | ragdoll slot 2 setter wrapper |
| `0x1419b2640` | ragdoll motor enable/disable |
| `0x141af6f60` | `hknpRagdollMotorController::hknpRagdollMotorController` |
| `0x141af7260` | ragdoll motor controller runtime function |

### Important Strings

| Address | String / Meaning |
| --- | --- |
| `0x142e1c450` | `hkpLimitedForceConstraintMotor` |
| `0x142e1be98` | `minForce` |
| `0x142e1c2f0` | `maxForce` |
| `0x142e1bf58` | `tau` |
| `0x142e1c178` | `damping` |
| `0x142e1da38` | `proportionalRecoveryVelocity` |
| `0x142e1c370` | `constantRecoveryVelocity` |
| `0x142e1ab18` | `useVelocityTargetFromConstraintTargets` |
| `0x142e1ada8` | `velocityTarget` |
| `0x142e9a4d0` | `springConstant` |
| `0x142e9a4e0` | `springDamping` |
| `0x142ed4c40` | `TYPE_LIN_MOTOR` |
| `0x142ed4cc0` | `TYPE_ANG_MOTOR` |
| `0x142ed4cd0` | `TYPE_RAGDOLL_MOTOR` |
| `0x142ed5400` | `hkpLinMotorConstraintAtom` |
| `0x142e1c208` | `hkpAngMotorConstraintAtom` |
| `0x142e147d0` | `isEnabled` |
| `0x142e1c878` | `motorAxis` |
| `0x142e1c850` | `targetAngle` |
| `0x142ea0128` | `targetPosition` |
| `0x142ed5010` | `previousTargetPositionOffset` |
| `0x142e1c5f8` | `previousTargetAngleOffset` |
| `0x142e1c978` | `initializedOffset` |
| `0x142e1bfa0` | `correspondingAngLimitSolverResultOffset` |
| `0x142e1d304` | `motor` |
| `0x142e1c580` | `angMotor` |

## Final Mapping Conclusion

Ghidra shows mouse spring grab as an hknp action that operates on one body id and a moving target, using position error, target-derived velocity, velocity damping, correction caps, thresholds, world locking, action registration, and body activation.

Ghidra shows hkp linear/angular motors as constraint-solver components. They need motor data, atoms, local constraint frames, target offsets, solver result offsets, enabled flags, runtime buffers, and refcount-safe pointer ownership. They offer richer constraint semantics but require a complete hkp constraint architecture around them.

For ROCK grab behavior, mouse spring is the lower-burden body-follow mechanism visible in FO4VR's binary. hkp custom linear/angular motors are viable only as part of a deliberately built constraint system with correct local axes, limits, target conversion, solver offsets, and lifecycle handling.
