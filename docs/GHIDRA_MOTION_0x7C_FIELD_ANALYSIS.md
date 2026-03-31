# hknpMotion +0x7C Field Analysis (Ghidra, 2026-03-31)

## Question
Is the field at hknpMotion+0x7C an independent scalar "integrationFactor" (TYPE_REAL)
or the W component of a 16-byte vector "previousStepAngularVelocity" at +0x70?

## Answer: It is the W component of previousStepAngularVelocity (hkVector4f at +0x70)

### Evidence

#### 1. FUN_1417d1d60 — writes previousStep velocities into motion
Called from `hknpMotion__setFromMassProperties` (0x1417d13a0) at the end.
```asm
; Writes previousStepAngularVelocity as full 16-byte SIMD vector
1417d1dd3: MOVAPS xmmword ptr [RCX + 0x70], XMM3   ; full 16 bytes at +0x70

; Writes previousStepLinearVelocity as full 16-byte SIMD vector
1417d1dfd: MOVAPS xmmword ptr [RCX + 0x60], XMM2   ; full 16 bytes at +0x60
```
Both are computed via quaternion rotation (identical pattern to current velocity computation).

#### 2. FUN_1417d1cb0 — the PARALLEL function writing current velocities
Structurally identical to FUN_1417d1d60 but writes to +0x40/+0x50:
```asm
1417d1d23: MOVAPS xmmword ptr [RCX + 0x50], XMM3   ; angularVelocity
1417d1d4d: MOVAPS xmmword ptr [RCX + 0x40], XMM2   ; linearVelocity
```
This proves the pattern: +0x40/+0x50 = current velocities, +0x60/+0x70 = previous step velocities.

#### 3. hknpMotion__setFromMassProperties (0x1417d13a0) — reads +0x70 as vector
```asm
1417d13db: MOVAPS XMM4, xmmword ptr [RBX + 0x70]   ; loads ALL 16 bytes as one vector
```
Then uses XMM4 in quaternion multiplication (SHUFPS, MULPS, SUBPS, ADDPS pattern).
+0x7C (the W component) participates in the quaternion math as part of the 4-component vector.

#### 4. hknpMotionManager__initializeMotion (0x1417e1e40) — zeroes +0x70 as vector
```asm
1417e1f55: MOVAPS xmmword ptr [RBX + 0x70], XMM0   ; XMM0 = zero from XORPS
```
Zeroes all 16 bytes at +0x70 (including +0x7C) as a single SIMD operation.

#### 5. FUN_1417e1fc0 (motion reset) — zeroes +0x70 as vector
```asm
1417e1ffa: MOVAPS xmmword ptr [R8 + 0x70], XMM0    ; zeros all of +0x70..+0x7F
```

### SDK Confirmation
Havok SDK `hknpPatches.cxx` line 1335:
```cpp
HK_PATCH_MEMBER_ADDED("previousStepAngularVelocity", TYPE_VEC_4, HK_NULL, 0)
```
TYPE_VEC_4 = 16-byte vector (4 floats), confirming the binary evidence.

### Where is integrationFactor?
The SDK `hknpPatches.cxx` line 1326 lists `integrationFactor` as TYPE_REAL (single float).
The patch file lists it between `orientation` and `maxRotationToPreventTunneling`,
but patch ordering != memory ordering. In the compiled FO4VR binary, `integrationFactor`
is NOT at +0x7C. It may be packed into the scalar fields at +0x38..+0x3F, or it may
have been removed/inlined in this version of the Havok runtime. No MOVSS access to
motion+0x7C was found in any function — it is always part of a 16-byte MOVAPS.

### Functions examined
| Address | Name | Access to +0x7C |
|---------|------|-----------------|
| 0x1417d13a0 | hknpMotion__setFromMassProperties | MOVAPS read [+0x70] as vector |
| 0x1417d1d60 | (unnamed, prev step velocity writer) | MOVAPS write [+0x70] as vector |
| 0x1417d1cb0 | (unnamed, current velocity writer) | Writes +0x40/+0x50 only (parallel structure) |
| 0x1417e1e40 | hknpMotionManager__initializeMotion | MOVAPS zero [+0x70] as vector |
| 0x1417e1fc0 | (unnamed, motion reset) | MOVAPS zero [+0x70] as vector |
| 0x1417e2200 | (unnamed, motion-to-solver copy) | Does NOT access +0x70 at all |

### Conclusion
**+0x7C is definitively the W component of previousStepAngularVelocity (hkVector4f at +0x70).**
No function in the binary accesses +0x7C independently as a scalar float.
Every access to this memory is a 16-byte SIMD operation (MOVAPS) at +0x70.
