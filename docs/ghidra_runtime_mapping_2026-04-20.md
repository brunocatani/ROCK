# FO4VR Runtime Mapping Audit — 2026-04-20

## Scope

This note maps the current ROCK hand-collider / grab path against the loaded FO4VR binary in Ghidra.
It ignores the deleted old plan/docs state and does not use `libraries_and_tools/havok/`.

Goal: identify whether the current failures come from bad wrappers/addresses, bad matrix/quaternion
conventions, or bad constraint-side body-space math.

## Current ROCK Runtime Path

### Hand collider update

`PhysicsInteraction::getInteractionHandTransform()`
-> `computeHandCollisionTransformFromHandBasis()`
-> `Hand::updateCollisionTransform()`
-> `computeHardKeyFrame`
-> `BethesdaPhysicsBody::setTransform()`
-> `BethesdaPhysicsBody::setVelocity()`

### Grab creation

`PhysicsInteraction::updateGrabInput()`
-> `Hand::grabSelectedObject()`
-> capture `_grabHandSpace`
-> capture `_grabBodyLocalTransform`
-> `createGrabConstraint()`

### Held update

`Hand::updateHeldObject()`
-> recompute `desiredBodyTransformHandSpace = _grabHandSpace * _grabBodyLocalTransform`
-> rewrite `m_target_bRca`
-> rewrite `transformB.rotation`
-> rewrite `transformB.translation`

## Binary-Verified Wrapper Map

### Confirmed valid wrappers / addresses

- `0x141E08A70` — `bhkNPCollisionObject::SetTransform(this, hkTransformf*)`
  - Resolves body ID from the wrapper and forwards to `0x141DF55F0`
- `0x141DF55F0` — deferred world transform setter
  - Uses TLS `+0x1528` to decide immediate vs queued path
- `0x141E082A0` — `bhkNPCollisionObject::SetVelocity(this, linVel, angVel)`
  - Resolves body ID and forwards to `0x141DF56F0`
- `0x141DF56F0` — deferred combined velocity setter
- `0x14153A6A0` — hard-keyframe velocity solver
  - Signature matches current ROCK usage:
    `world, bodyId, targetPos[4], targetQuat[4], dt, outLinVel[4], outAngVel[4]`
- `0x141E07300` — `bhkNPCollisionObject::SetMotionType(this, motionType)`
  - Real full transition function, not a hallucinated wrapper
- `0x14153B2F0` — raw motion-properties setter
  - Only writes `motion+0x38`
  - Does not perform a full keyframed/dynamic transition
- `0x141A4AD20` — constraint-info utility used by `GrabConstraint`
  - Real function, signature and behavior are consistent with atom-walk / size accumulation

### Important validated behavioral fact

`SetBodyMotionProperties` is only a fallback. The current ROCK preference for
`CollisionObject_SetMotionType(DYNAMIC)` is correct, because the raw setter does not clear body flags
or rebuild the full motion state.

## High-Confidence Matrix / Body-Space Findings

### 1. Havok transform memory is being misinterpreted in the current source comments

HIGGS reference:

- `NiMatrixToHkMatrix()` writes Havok columns:
  - col0 = `(m00, m10, m20)`
  - col1 = `(m01, m11, m21)`
  - col2 = `(m02, m12, m22)`

FO4VR binary:

- `0x1415395E0` (`SetBodyTransform` inner path) copies the incoming transform as 4-float blocks,
  preserving the existing W lanes in body memory
- It does not reinterpret or transpose those blocks on write
- `0x141722C10` (matrix -> quaternion helper) consumes those same blocks as a Havok matrix

Conclusion:

- The numeric body layout `0,1,2 / 4,5,6 / 8,9,10` is not proof of “row-major semantics”
- It is consistent with Havok column blocks stored as:
  - block0 = basis X axis
  - block1 = basis Y axis
  - block2 = basis Z axis

The current ROCK comments that label this as “row-major body storage” are not reliable.

### 2. `getBodyWorldTransform()` is reconstructing body rotation with the wrong semantic mapping

Current code in `HandGrab.cpp` reads:

- row0 = `body[0], body[1], body[2]`
- row1 = `body[4], body[5], body[6]`
- row2 = `body[8], body[9], body[10]`

That treats each 4-float Havok block as a Ni row.

But the Havok-side meaning of those blocks is column-oriented. The correct Ni reconstruction is the
HIGGS/Havok inverse mapping:

- `ni[0][0] = body[0]`, `ni[1][0] = body[1]`, `ni[2][0] = body[2]`
- `ni[0][1] = body[4]`, `ni[1][1] = body[5]`, `ni[2][1] = body[6]`
- `ni[0][2] = body[8]`, `ni[1][2] = body[9]`, `ni[2][2] = body[10]`

Impact:

- `_grabBodyLocalTransform` is being captured from a transposed live body rotation
- All later held-update math that multiplies by `_grabBodyLocalTransform` inherits that error

### 3. The current pivotA / pivotB body-local conversion is also based on the same wrong interpretation

Current code in `GrabConstraint.cpp` computes local axes with:

- X = `[0,4,8] dot delta`
- Y = `[1,5,9] dot delta`
- Z = `[2,6,10] dot delta`

That only makes sense if the body array is being interpreted as Ni rows.

Under Havok’s actual block layout, the body basis vectors are the contiguous blocks:

- X axis = `[0,1,2]`
- Y axis = `[4,5,6]`
- Z axis = `[8,9,10]`

Impact:

- `transformA.translation` and `transformB.translation` can be wrong in a way that depends on body
  orientation
- That directly matches the reported “grab is all fucked up” symptom because the motors are trying to
  bring the wrong local anchors together

### 4. The current quaternion helper is under direct suspicion

Current ROCK `niRotToHkQuat()` was changed to a conjugated sign convention based on interpreting the
 FO4VR matrix helper as if its input were row-major Ni storage.

What the binary + HIGGS combination says instead:

- HIGGS sends Havok rotations through `NiMatrixToHkMatrix()` / `NiQuatToHkQuat()` with normal Havok
  column semantics
- FO4VR `0x141722C10` is operating on Havok matrix memory blocks, not on a Ni row-major scratch matrix

Result:

- The current `niRotToHkQuat()` sign choice is no longer trustworthy
- It is likely producing a conjugated target quaternion relative to what `computeHardKeyFrame`
  expects

This needs direct correction or a focused follow-up blind check before code changes are committed.

## Symptom Correlation

### Collider shift with player orientation

Consistent with one or both of:

- hand/body basis being reconstructed from the live body with the wrong matrix semantic mapping
- target quaternion convention not matching the transform convention used in the same frame

### Broken grab behavior

Consistent with:

- `_grabBodyLocalTransform` captured from a transposed body basis
- `pivotA` / `pivotB` local conversion using wrong basis vectors
- held-frame `transformB.rotation` / `transformB.translation` updates built on that bad local frame

## Constraint-Side Status

What is currently supported by the binary verification:

- the shared constraint-info utility address is real
- the custom constraint is not obviously calling a fake function
- the raw motion-properties fallback is correctly identified as insufficient

What is not yet re-audited here:

- every individual custom atom offset used by `GrabConstraint.cpp`
- every runtime offset written into the custom constraint runtime area

Those still need a dedicated solver-side pass if grab remains unstable after the transform/body-space
math is corrected.

## Current Verdict

The present failures do not look like “all wrappers are fake”.

The wrapper layer for the critical hand/grab path is mostly real:

- `SetTransform`
- `SetVelocity`
- `SetMotionType`
- `computeHardKeyFrame`
- `SetBodyMotionProperties` fallback
- `GetConstraintInfoUtil`

The strongest failure mode is a matrix/quaternion interpretation drift that infected multiple source
paths:

1. body transform reconstruction
2. body-local pivot conversion
3. quaternion target generation

This is sufficient by itself to explain:

- orientation-dependent collider drift
- incorrect grab anchor behavior
- inconsistent held-object motor targets
