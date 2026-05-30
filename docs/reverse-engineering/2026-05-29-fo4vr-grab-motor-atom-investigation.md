# FO4VR Grab Motor Atom Investigation

Date: 2026-05-29

Project: ROCK

Branch: feature/ghidra-grab-motor-mapping

Scope: Dynamic proxy grab custom constraint, hkp ragdoll motor atom, hkp linear motor atom, hkpPositionConstraintMotor layout, and remaining frame-feed mismatch risks.

Source authority:
- Current local ROCK source.
- Ghidra MCP on the loaded FO4VR executable image.
- Byte-level cross-checks against `E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe`.

External web: not used.

Confidence:
- High for atom sizes, solver-result counts, runtime offset convention, stock vtable slot hazards, and hkpPositionConstraintMotor field layout.
- Medium for `target_bRca` row/column semantics because the low-level solver schema consumer was not fully decompiled here. Layout evidence verifies where the data lives, not every matrix interpretation inside the solver.

## Question

The runtime symptom is a bad dynamic grab where the object can appear to lose rotational authority or enter a limp/360 state. The investigation question was whether ROCK is feeding Havok motors incorrectly, missing required motor setup steps, or corrupting the custom constraint/motor atom contract.

## Current ROCK Feed Path

Relevant source files:

- `src/physics-interaction/grab/GrabConstraint.h`
- `src/physics-interaction/grab/GrabConstraint.cpp`
- `src/physics-interaction/grab/GrabConstraintMath.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabCore.h`

ROCK builds a custom constraint data object:

```text
object +0x000: custom hkpRagdollConstraintData-derived vtable
object +0x008: hkReferencedObject header dword 0xffff0001
object +0x020: atom chain start

atom +0x000 / object +0x020: type 2  set-local-transforms, size 0x90
atom +0x090 / object +0x0b0: type 23 setup-stabilization, size 0x10
atom +0x0a0 / object +0x0c0: type 19 ragdoll-motor, size 0x60
atom +0x100 / object +0x120: type 11 linear-motor X, size 0x18
atom +0x118 / object +0x138: type 11 linear-motor Y, size 0x18
atom +0x130 / object +0x150: type 11 linear-motor Z, size 0x18
```

The custom atom block size is `0x148`, from object `+0x20` through object `+0x167`.

The custom runtime currently reports:

```text
solver results: 12
solver result bytes: 12 * 8 = 0x60
ragdoll init offset: 0x60
ragdoll previous angular offset: 0x64
linear init bytes: 0x70, 0x71, 0x72
linear previous target floats: 0x74, 0x78, 0x7c
used private runtime end: 0x80
reported runtime size: 0x100
```

## Binary Evidence

### Executable Image

The unpacked executable has the same segment layout as the Ghidra image:

```text
.text   0x140001000 - 0x142c40bff
.rdata  0x142c4c000 - 0x1436f05ff
.data   0x1436f1000 - 0x14689e1af
```

The address base is `0x140000000`.

### getConstraintInfo Atom Table

`FUN_141a4ad20` is the FO4VR `GetConstraintInfoUtil` used by ROCK through `offsets::kFunc_GetConstraintInfoUtil`.

The switch table was byte-read from:

```text
E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe
VA  0x141a4aee8
RVA 0x01a4aee8
```

Relevant atom entries:

```text
type  2 -> 0x141a4aed1: advance atom pointer by 0x90, no solver results
type 11 -> 0x141a4ae16: schema +0x50, solver results +2, advance by 0x18
type 19 -> 0x141a4ae8d: schema +0xc0, solver results +6, advance by 0x60
type 23 -> 0x141a4adef: advance atom pointer by 0x10, no solver results
```

This proves ROCK's 12-result custom runtime count is coherent:

```text
type 19 ragdoll motor: 6 results
3 * type 11 linear motor: 3 * 2 = 6 results
total: 12 results
```

It also corrects an earlier working assumption: the set-local-transforms atom does not contribute 3 solver results in this FO4VR helper. The ragdoll motor atom contributes 6.

### Stock hkpRagdollConstraintData

Stock vtable:

```text
hkpRagdollConstraintData vtable = 0x142e18298
```

Stock constructor:

```text
hkpRagdollConstraintData::hkpRagdollConstraintData = 0x1419b1d50
```

Important constructor facts:

- Writes referenced-object header `0xffff0001` at object `+0x08`.
- Uses atom base `this + 0x20`.
- Calls `FUN_1419b2910(this + 0x20)` to initialize the stock atom chain.
- Writes set-local transform A columns/position at object `+0x30..+0x60`.
- Writes set-local transform B columns/position at object `+0x70..+0xa0`.
- Writes ragdoll target rows/columns area at object `+0xd0..+0xf0`.
- Writes ragdoll motor pointers at object `+0x100`, `+0x108`, `+0x110`.
- Writes through object `+0x180`, so the stock object is larger than ROCK's compact custom object.

Stock `FUN_1419b2910` initializes:

```text
atom base +0x000: type 2
atom base +0x090: type 23, disabled, real max/high clamps initialized
atom base +0x0a0: type 19, active flag initially 1, private runtime offsets initially -1
atom base +0x100 and beyond: extra stock ragdoll limit/friction atoms
```

The stock constructor then disables the ragdoll motor flag at object `+0xc2` and writes stock ragdoll private runtime offsets through `FUN_1419b1d20`.

`FUN_1419b1d20(atomBase, enabled)` writes:

```text
atomBase +0xa4: 0x0090
atomBase +0xa6: 0x0094
```

when enabled. Stock `getRuntimeInfo` for ragdoll returns:

```text
runtime size: 0xb0
solver results: 0x12
```

So stock uses `0x90` as the start of ragdoll private runtime because `0x12 * 8 = 0x90`. ROCK's custom chain has 12 solver results, so its ragdoll private runtime start at `0x60` is the same convention applied to a smaller atom chain.

### Stock hkpLinearClearanceConstraintData

Stock linear clearance is useful because it contains a type 11 linear motor atom.

Constructor:

```text
hkpLinearClearanceConstraintData::hkpLinearClearanceConstraintData = 0x142020620
```

Stock type 11 atom starts at object `+0xb0`:

```text
object +0xb0: type 11
object +0xb2: active/motor flag
object +0xb3: axis
object +0xb4: init runtime offset
object +0xb6: previous target runtime offset
object +0xb8: target float
object +0xc0: motor pointer
```

The constructor writes offsets `0x70` and `0x74` for the type 11 atom.

Stock linear clearance `getRuntimeInfo` returns:

```text
runtime size: 0x78
solver results: 0x0e
```

Again, `0x0e * 8 = 0x70`, so linear motor private state starts immediately after all solver result slots. This supports ROCK's linear offsets:

```text
init bytes: 0x70, 0x71, 0x72
previous target floats: 0x74, 0x78, 0x7c
```

### hkpPositionConstraintMotor

Position motor vtable:

```text
hkpPositionConstraintMotor vtable = 0x142e95fe8
```

Constructor:

```text
0x141e726d0:
LEA RAX, [0x142e95fe8]
RET
```

The constructor only returns the vtable address. It does not initialize the referenced-object header or tuning fields by itself in this code path.

The clone function at `0x141f610f0` allocates `0x30` bytes and copies these fields:

```text
+0x08: hkReferencedObject header dword 0xffff0001
+0x10: motor type byte
+0x18: minForce
+0x1c: maxForce
+0x20: tau
+0x24: damping
+0x28: proportionalRecoveryVelocity
+0x2c: constantRecoveryVelocity
```

This matches ROCK's `HkPositionMotor` struct and `createPositionMotor` writes.

ROCK currently sets:

```text
vtable = base + (0x142e95fe8 - 0x140000000)
referenceCount = 1
memSizeAndFlags = 0xffff
pad0C = 0
type = 1
minForce / maxForce / tau / damping / recovery fields
```

No missing motor-constructor step was found.

## Vtable Safety

ROCK copies stock ragdoll vtable slots and overrides the dangerous ones. Ghidra confirms those overrides are required.

Stock slot hazards:

```text
slot 0 destructor 0x1419b2aa0:
  calls stock destructor, releases motors at object +0x100..+0x110,
  and frees size 0x1a0 unless mem flags override it.

slot 5 getConstraintInfo 0x1419b27f0:
  calls GetConstraintInfoUtil(this + 0x20, 0x180, info).

slot 6 isValid 0x1419b2710:
  reads stock-only fields through about object +0x18a and limit fields.

slot 9 setSolvingMethod 0x1419b2810:
  writes object +0x190.

slot 13/14 display flags:
  read/write object +0x18b.

slot 15 setMotorMode 0x1419b2860:
  toggles stock fields, including object +0xc2 and +0x18a.

slot 16/17 max friction:
  read/write object +0x194.

slot 18 stock getRuntimeInfo 0x1419b2900:
  returns runtime size 0xb0 and 18 solver results.
```

ROCK overrides these paths with custom callbacks:

```text
destructor
getType
getConstraintInfo
isValid
setSolvingMethod
getInertiaStabilization
setDisplayFlags
getDisplayFlags
setMotorMode
setMaxFriction
getMaxFriction
getRuntimeInfo
addInstance
```

The inherited add-instance implementation also only zeroes runtime memory, but ROCK overrides it to log and zero the provided runtime explicitly.

Conclusion: the compact object is unsafe with unmodified stock vtable behavior, but the current override set covers the stock methods proven to read/write outside the compact layout or return stock runtime metadata.

## What This Proves

The current custom atom and motor object feed is internally coherent at the binary layout level:

- Atom base `this + 0x20` is correct.
- Type 2, type 23, type 19, and type 11 IDs are correct for FO4VR.
- Type 19 size and solver-result count are correct.
- Type 11 size and solver-result count are correct.
- Custom runtime solver-result count of 12 is correct for 1 ragdoll motor plus 3 linear motors.
- Private runtime offsets at `0x60`, `0x64`, `0x70..0x7c` follow the same convention used by stock ragdoll and stock linear clearance constraints.
- hkpPositionConstraintMotor is 0x30 bytes and ROCK writes the fields the stock clone function copies.
- No separate required motor activation API was found beyond atom active flags, valid motor pointers, valid runtime offsets, and valid runtime info.

This makes a missing motor-constructor step or wrong motor object size unlikely.

## What This Does Not Prove

The binary layout does not fully settle the row/column convention of `target_bRca`. The layout proves:

```text
ragdoll target_bRca lives at type19 atom +0x10
ragdoll motors live at type19 atom +0x40
```

It does not by itself prove whether the generated solver schema interprets that target as the row view, column view, inverse relation, or forward relation in every orientation. Current source is back to the row-view convention after reverting the column target commit.

Runtime telemetry remains the authority for that convention:

- `RAGDOLL ANGULAR PROBE beforeErr/afterErr/reduce`
- `targetRowsInv`
- `targetColsToTransformB`
- `bRcaRowsErr`
- `bRcaColsErr`
- `transformBDelta`

## Remaining Suspect: Data Fed Into Correct Atoms

The strongest remaining risk is not motor object setup. It is feeding a mathematically coherent but physically wrong frame/pivot pair into the correct atoms.

Current freeze captures the selected BODY-local grip point:

```cpp
frozen.pivotBConstraintLocalGame =
    transform_math::worldPointToLocal(input.constraintBodyWorld, input.gripPointWorld);
```

But proxy constraint creation then computes the solver transform-B pivot from the frozen BODY relation and pivot A:

```cpp
solverPivotBConstraintLocalGame =
    computeDynamicTransformBTranslationGame(
        desiredBodyTransformProxySpace,
        _grabFrame.pivotAHandBodyLocalGame);
```

That computed value is what gets written to transform B and stored as:

```cpp
_grabAuthorityPivotBConstraintLocalGame = solverPivotBConstraintLocalGame;
```

Held updates then keep using `activeProxyConstraintPivotBLocalGame()`, which returns `_grabAuthorityPivotBConstraintLocalGame` while the proxy constraint is active.

This means there are two possible BODY-local pivot-B values:

```text
1. selected pivot: selected grip point in constraint/body local space
2. relation-implied pivot: inverse(proxyAuthorityBodyHandSpace) * pivotAHandBodyLocalGame
```

Telemetry already names the difference:

```text
transformBErrPred =
    distance(frozenAuthorityFrame.pivotBConstraintLocalGame,
             predictedTransformBLocal)
```

If `transformBErrPred` is meaningfully nonzero on broken grabs, the solver is not using the same BODY-local grip point selected by the acquisition/freeze path. That is the same class of bug as the earlier palm-pocket/collider local-space issue: data is valid in its own space, but the solver receives a value converted through a different authority frame.

Why this can look like low rotational authority:

- The linear pivot may be moved toward a relation-implied point that is not the physical grip point.
- The lever arm from COM can shrink or change direction.
- The angular target and linear pivot can be internally coherent but not coherent with the visible selected grip.
- The effect is hand-independent and can appear only on some object orientations or grab points.

This is a better explanation than "missing motor setup" if logs show nonzero angular force and the same atom/motor contract is used for both good and bad grabs.

## Relation To The Reverted Column Target Commit

The reverted column-target commit made the broken behavior change from limp/360 toward a strong wrist-twist style failure. That is useful evidence:

- The ragdoll motor atom can produce strong angular authority when fed a target it consumes forcefully.
- The column target path was not the correct target relation for the current hand/object frame, because it twisted the hand/object relation.
- The original limp/360 symptom is therefore less likely to be caused by an uninitialized motor object or missing motor activation.
- The symptom is more likely caused by target frame/pivot data being wrong or inconsistent on bad grabs, or by the remaining unresolved `target_bRca` convention for some orientations.

## Recommendations

Do not change these based on this investigation:

- `HkPositionMotor` size or fields.
- hkpPositionConstraintMotor vtable address.
- type 19 and type 11 atom sizes.
- custom runtime solver-result count of 12.
- private runtime offsets `0x60`, `0x64`, `0x70..0x7c`.
- setup-stabilization defaults.
- current vtable override set.

Next code-level work should focus on the frame/pivot feed invariant:

1. Treat `transformBErrPred` as a hard diagnostic for freeze coherence.
2. If the selected BODY-local pivot and relation-implied transform-B pivot differ, do not silently let both meanings exist.
3. Pick one authority model:
   - BODY-selected pivot authority: keep `pivotBConstraintLocalGame` from freeze and make the desired BODY relation align pivot A to that selected point.
   - Relation-implied pivot authority: explicitly mark the selected mesh pivot as visual evidence only and log the substitution as solver pivot rebinding.
4. Add a source-boundary or policy test that prevents hidden replacement of selected BODY-local pivot B without a named policy and telemetry.
5. Keep using runtime telemetry to decide `target_bRca` storage convention; Ghidra layout evidence alone is not enough to change that again.

Bottom line: the motors themselves look correctly allocated, initialized, attached, activated, and budgeted at the Havok data-contract level. The most likely remaining "solver sometimes gets right but not correct" class is a frame/pivot feed mismatch before data reaches the motor atoms.

## Runtime Follow-Up: Constraint-Frame Target

Source: ROCK runtime log from the deployed `907f373` logging-only build.

Verification method: dynamic proxy grab of a Nuka-Cola bottle with new `targetRowsDesiredBRca`, `targetColsDesiredBRca`, and `desiredBRcaIdentity` telemetry.

Finding:

- `ragdoll=yes`, `forceA=2000`, `forceL=2000`, `tau=0.030`, and `damp=0.80`, so the angular motor was active and budgeted.
- The hidden proxy body tracked its physical target: `proxyErr` stayed near `0.00gu/0.0deg`.
- The selected BODY-local pivot remained coherent: `transformBDelta=0.00gu`.
- `targetRowsInv=0.0deg` showed the old row target matched the saved proxy BODY relation, but `targetRowsDesiredBRca` and `targetColsDesiredBRca` were both high against the actual constraint-frame target.
- On one bad sample `desiredBRcaIdentity=4.7deg` while `targetRowsDesiredBRca=97.6deg` and angular error worsened from `164.9deg` to `170.5deg`.

Implication:

`target_bRca` must be derived from the actual body-A/body-B constraint frames: physical proxy body A, desired BODY pose B, transform-A, and transform-B. It must not be written directly from the saved generated/proxy BODY relation, because transform-B already carries that relation into the B constraint frame.
