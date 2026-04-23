# FO4VR Quaternion / HardKeyFrame Audit

**Date:** 2026-04-21
**Binary:** FO4VR executable loaded in Ghidra MCP
**Scope:** Blind-audit the quaternion contract behind `computeHardKeyFrame`, `DriveToKeyFrame`, and ROCK's manual `computeHardKeyFrame + setTransform` path.

## Why this audit exists

Phase 1 confirmed the current dominant bug class is transform-convention drift. The remaining open question was whether ROCK's current `niRotToHkQuat()` comment and sign choice match the real FO4VR runtime contract or whether they only matched an older mistaken interpretation.

This matters directly to:

- `Hand.cpp` manual hand-body drive path
- `WeaponCollision.cpp` manual weapon-body drive path
- any future code that mixes `computeHardKeyFrame()` with `SetTransform()`

## Audit targets

1. `FUN_14153a6a0` at `0x14153A6A0`
2. `FUN_141722c10` at `0x141722C10`
3. `FUN_141e086e0` at `0x141E086E0`
4. HIGGS reference paths that build packed Havok rotations and packed Havok quaternions

## Key verdict

`computeHardKeyFrame()` itself uses the normal body quaternion convention for the live `hknpBody` basis. The apparent sign inversion only appears when `FUN_141722c10` is fed a raw `NiMatrix3` row-major block. That is exactly what Bethesda's `DriveToKeyFrame()` wrapper does with a raw `NiTransform`, and it remains internally consistent only because its teleport fallback uses that same raw transform layout.

ROCK's manual path is different:

- it computes angular velocity from `computeHardKeyFrame()`
- then teleports with a packed `hkTransformf` built by `niRotToHkTransformRotation()`

For that manual path, the target quaternion must match the packed/live Havok orientation, not the raw `NiTransform` row layout. That means the correct target quaternion is the textbook quaternion of the original Ni rotation matrix, not its conjugate.

## Blind findings

### 1. `FUN_141722c10` is a matrix-to-quaternion helper

Ghidra decompile at `0x141722C10` reads matrix elements at `[0]`, `[5]`, `[10]` for the trace and emits four floats in memory order:

- `x = (m21 - m12) * s`
- `y = (m02 - m20) * s`
- `z = (m10 - m01) * s`
- `w = sqrt(trace + 1) * 0.5`

The disassembly matches the same accesses:

- `[RDX + 0x18] - [RDX + 0x24]`
- `[RDX + 0x20] - [RDX + 0x08]`
- `[RDX + 0x04] - [RDX + 0x10]`

This is the textbook quaternion formula when the input matrix is laid out as Havok columns:

- col0 = `[0,1,2]`
- col1 = `[4,5,6]`
- col2 = `[8,9,10]`

### 2. `computeHardKeyFrame` reads the live body quaternion from the live `hknpBody`

Ghidra decompile at `0x14153A6A0`:

- calls `FUN_141722c10((...),(float*)(bodyBase + bodyId * 0x90))`
- therefore converts the live `hknpBody` rotation block to quaternion
- then computes the orientation delta as `q_target * conj(q_body)`

The multiply order is visible in the decompile:

- body quaternion from `FUN_141722c10` lands in `local_c8/fStack_c4/fStack_c0/fStack_bc`
- target quaternion comes from `param_4`
- the resulting delta quaternion matches `target * conj(body)`

So `computeHardKeyFrame()` expects `param_4` to describe the same orientation convention as the live body basis.

### 3. `DriveToKeyFrame` takes a raw `NiTransform*`, not a packed `hkTransformf*`

Ghidra decompile at `0x141E086E0` shows:

- `FUN_141722c10((undefined8 *)local_58, param_2);`
- `FUN_14153a6a0(world, bodyId, pfVar11 + 0xc, local_58, dt, ...)`
- teleport fallback uses `FUN_141df55f0(world, bodyId, pfVar11, 1);`

`pfVar11 + 0xc` means the translation starts at float index 12, which matches `NiTransform`:

- `NiMatrix3` = 12 floats
- translation = floats 12, 13, 14
- scale = float 15

This matches the local wrapper too:

- [BethesdaPhysicsBody.cpp](E:/fo4dev/PROJECT_ROCK_V2/ROCK/src/physics-interaction/BethesdaPhysicsBody.cpp:720) passes `const RE::NiTransform&`
- [HavokOffsets.h](E:/fo4dev/PROJECT_ROCK_V2/ROCK/src/physics-interaction/HavokOffsets.h:200) already documents `DriveToKeyFrame(this*, NiTransform*, float dt)`

### 4. Why `DriveToKeyFrame` can use a conjugated quaternion and still be internally consistent

Raw `NiTransform.rotate` is stored as Ni rows:

- row0 = `[m00,m01,m02]`
- row1 = `[m10,m11,m12]`
- row2 = `[m20,m21,m22]`

If `FUN_141722c10` is fed that raw row-major block directly, it computes:

- `x = (m12 - m21) * s`
- `y = (m20 - m02) * s`
- `z = (m01 - m10) * s`

That is the conjugate of the textbook quaternion of `M`.

`DriveToKeyFrame` stays internally consistent because its teleport fallback also feeds the same raw transform block to `SetTransform`-side code. Raw Ni rows copied into the live body blocks produce a live basis equivalent to `M^T`, and the quaternion of `M^T` is the conjugate of the quaternion of `M`.

So Bethesda's wrapper is self-consistent for the raw `NiTransform` contract even though that contract is not the same one ROCK uses in the manual packed-Havok path.

## Contract table

### A. Raw `NiTransform.rotate` memory

Memory layout:

- `[0,1,2] = row0 = [m00,m01,m02]`
- `[4,5,6] = row1 = [m10,m11,m12]`
- `[8,9,10] = row2 = [m20,m21,m22]`

`FUN_141722c10(rawNiRows)` returns:

- `conjugate(quat(M))`

### B. ROCK packed `hkTransformf.rotation`

`niRotToHkTransformRotation(M)` packs:

- row0 = Ni column 0
- row1 = Ni column 1
- row2 = Ni column 2

Memory layout becomes:

- `[0,1,2] = [m00,m10,m20]`
- `[4,5,6] = [m01,m11,m21]`
- `[8,9,10] = [m02,m12,m22]`

`FUN_141722c10(packedHkTransform)` returns:

- `quat(M)` textbook

### C. Live `hknpBody` rotation block

Phase 1 already verified the live body stores contiguous Havok basis columns:

- `[0,1,2] = X basis`
- `[4,5,6] = Y basis`
- `[8,9,10] = Z basis`

`FUN_141722c10(liveBody)` returns:

- `quat(liveBodyOrientation)` textbook

## HIGGS comparison

HIGGS uses packed Havok rotations and packed Havok quaternions, not the raw `NiTransform` wrapper contract:

- [math_utils.cpp](E:/fo4dev/skirymvr_mods/source_codes/higgs/src/math_utils.cpp:288) packs Havok columns from Ni columns with `NiMatrixToHkMatrix`
- [hand.cpp](E:/fo4dev/skirymvr_mods/source_codes/higgs/src/hand.cpp:698) builds `hkQuaternion desiredQuat; desiredQuat.setFromRotationSimd(transform.m_rotation);`
- [utils.cpp](E:/fo4dev/skirymvr_mods/source_codes/higgs/src/utils.cpp:313) drives hard keyframe velocity with `NiQuatToHkQuat(rot)` after `MatrixToQuaternion`

That matches ROCK's manual packed-Havok path, not Bethesda's raw `DriveToKeyFrame(NiTransform*)` wrapper contract.

## Current ROCK claims vs blind audit

### Claim: `FUN_141722c10` uses the opposite sign convention

**Verdict:** PARTIAL / MISLEADING

It only appears opposite when the input is raw Ni row-major memory. For live `hknpBody` or a packed `hkTransformf`, it is the normal textbook quaternion formula.

### Claim: manual `computeHardKeyFrame + setTransform` should use the conjugated quaternion

**Verdict:** DISPUTE

That would only match the raw `DriveToKeyFrame(NiTransform*)` contract. ROCK's manual path teleports with a packed `hkTransformf`, so its target quaternion must match the packed/live Havok orientation instead.

### Claim: the current `niRotToHkQuat()` implementation in `PhysicsUtils.h` matches the manual path

**Verdict:** DISPUTE

The current implementation computes the conjugate signs:

- `qx = m12 - m21`
- `qy = m20 - m02`
- `qz = m01 - m10`

For the manual hand and weapon paths, the correct target quaternion should instead match `FUN_141722c10(niRotToHkTransformRotation(M))`, which yields:

- `qx = m21 - m12`
- `qy = m02 - m20`
- `qz = m10 - m01`

## What this means for ROCK

These callers are affected:

- [Hand.cpp](E:/fo4dev/PROJECT_ROCK_V2/ROCK/src/physics-interaction/Hand.cpp:675)
- [WeaponCollision.cpp](E:/fo4dev/PROJECT_ROCK_V2/ROCK/src/physics-interaction/WeaponCollision.cpp:477)

They currently pair:

- `computeHardKeyFrame(..., tgtQuat, ...)` from `niRotToHkQuat()`
- `SetTransform()` using `niRotToHkTransformRotation()`

If `tgtQuat` is conjugated while `SetTransform()` targets the packed/live orientation, the angular velocity solver and the teleport/live body basis disagree on the desired orientation.

That mismatch is a plausible direct cause of:

- orientation-dependent collider drift
- unstable angular behavior in the keyframed hand body
- matching instability in the weapon collision body when enabled

## Implementation consequence for the next phase

For ROCK's manual packed-Havok path, `niRotToHkQuat()` should be changed to the textbook signs so it matches:

- the live body quaternion extracted inside `computeHardKeyFrame()`
- the packed `hkTransformf` orientation sent to `SetTransform()`
- the HIGGS packed-Havok reference path

This change should be applied together to:

- `Hand.cpp`
- `WeaponCollision.cpp`
- `PhysicsUtils.h`

and then verified with live witness logging, because the old session comments in `PhysicsUtils.h` were based on conflating two different contracts:

1. raw `DriveToKeyFrame(NiTransform*)`
2. manual packed `computeHardKeyFrame + SetTransform(hkTransformf)`
