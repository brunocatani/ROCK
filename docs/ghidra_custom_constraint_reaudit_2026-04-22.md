# FO4VR Custom Constraint Re-Audit - 2026-04-22

## Why this note exists

The chat was auto-compacted and the remaining fault domain is now narrow: the linear/pivot side is
behaving, while the custom angular path still blows out. This note restarts the binary audit from
raw FO4VR evidence plus HIGGS source, instead of trusting the compacted conversation.

## Scope

Re-verify the custom grab constraint facts that directly affect the angular path:

- `transformB.rotation` offsets inside `hkpSetLocalTransformsConstraintAtom`
- `m_target_bRca` placement inside the ragdoll motor atom
- `getRuntimeInfo` slot and output-field order
- runtime-offset facts that are actually supported by the current binary pass

Binary under audit: FO4VR executable loaded in Ghidra MCP.

## Verified binary facts

### 1. Stock ragdoll atoms start at `constraintData + 0x20`

`hkpRagdollConstraintData::vfunction6` at `0x1419B27F0` is the stock `getConstraintInfo` stub.

Disassembly:

```asm
1419b27f0: MOV R8,RDX
1419b27f3: ADD RCX,0x20
1419b27f7: MOV EDX,0x180
1419b27fc: JMP 0x141a4ad20
```

Verdict:

- atoms start at `this + 0x20`
- stock ragdoll atom span is `0x180`
- ROCK `ATOMS_START = 0x20` is supported by the binary

### 2. Slot `+0x90` is the runtime-info virtual, and the output order is `size` then `numSolverResults`

`hknpConstraint::init` at `0x1417E39C0` calls `[*vtable + 0x90]` with `R8 = &localRuntimeInfo`.
After the call it uses:

- `[rsp+0x60]` as the runtime allocation size
- `[rsp+0x64]` as the stored solver-result count byte

Stock ragdoll `getRuntimeInfo` is `hkpRagdollConstraintData::vfunction19` at `0x1419B2900`:

```asm
1419b2900: MOV dword ptr [R8 + 0x4],0x12
1419b2908: MOV dword ptr [R8],0xb0
1419b290f: RET
```

Verdict:

- FO4VR slot `18` / `+0x90` is confirmed for `getRuntimeInfo`
- field order is:
  - `out[0] = sizeOfExternalRuntime`
  - `out[1] = numSolverResults`
- ROCK callback write order matches the engine contract

### 3. Stock ragdoll motor runtime-offset fields are at atom `+0x04/+0x06`

`hkpRagdollConstraintData` constructor at `0x141E8A5E0` does:

```asm
141e8a600: LEA RCX,[RBX + 0x20]
141e8a604: MOV EDX,EDI
141e8a609: CALL 0x1419b1d20
```

Helper `FUN_1419B1D20` writes:

```asm
1419b1d23: MOV word ptr [RCX + 0xa4],AX
1419b1d2a: MOV word ptr [RCX + 0xa6],AX
1419b1d38: MOV dword ptr [RCX + 0xa4],0x940090
```

Because `RCX = this + 0x20`, those writes land at:

- `this + 0xC4` = ragdoll atom `+0x04`
- `this + 0xC6` = ragdoll atom `+0x06`

Verdict:

- stock FO4VR ragdoll motor atom starts at `constraintData + 0xC0`
- `m_initializedOffset` and `m_previousTargetAnglesOffset` really are the words at atom `+0x04/+0x06`
- stock ragdoll writes `0x90/0x94` there, which is consistent with `18 * 8 = 0x90` bytes of stock
  solver results before the angular motor state

### 4. Stock ragdoll motor pointers are at atom `+0x40`

Destructor `FUN_1419B1EE0` starts releasing three pointers from `this + 0x100`:

```asm
1419b1ef9: LEA RBX,[RCX + 0x100]
...
1419b1f43: ADD RBX,0x8
...
```

With ragdoll atom base already verified at `this + 0xC0`, this places the 3 motor pointers at:

- `ragdollAtom + 0x40`

Verdict:

- FO4VR object layout matches the expected ragdoll motor tail pointer array at `+0x40`
- this makes a `m_target_bRca` block at atom `+0x10..+0x3F` structurally consistent
- absolute ROCK target location `constraintData + 0xD0` remains supported by stock layout

This is strong structural evidence, but not yet a direct decompile of a stock setter that names the
target field explicitly.

### 5. `transformB.rotation` absolute offsets used by ROCK are supported by the solver path

Inside `FUN_141A55550`, the stock `hkpSetLocalTransformsConstraintAtom` case begins at
`0x141A556DA` and consumes `0x90` bytes.

Key pattern:

```asm
141a556e7: LEA RAX,[R14 + 0x40]
...
141a5573f: LEA RAX,[R14 + 0x80]
...
141a557a6: ADD R14,0x90
```

The first loop walks `R14 + 0x20/+0x30/+0x40`.
The second loop walks `R14 + 0x60/+0x70/+0x80`.

With atom base at absolute `constraintData + 0x20`, that means the second transform rotation is
read from absolute:

- `constraintData + 0x70`
- `constraintData + 0x80`
- `constraintData + 0x90`

Verdict:

- ROCK `kTransformB_Col0 = 0x70`
- ROCK `kTransformB_Col1 = 0x80`
- ROCK `kTransformB_Col2 = 0x90`

are supported by the binary.

This lowers the probability that the current angular failure is caused by a simple
`transformB.rotation` absolute-offset mismatch.

## HIGGS cross-check

HIGGS held-update path:

- `constraintData->setTargetRelativeOrientationOfBodies(...)`
- `constraintData->m_atoms.m_transforms.m_transformB.m_translation = ...`

Reference:

- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/hand.cpp:3921-3926`

Important difference:

- HIGGS does **not** rewrite `transformB.rotation` every frame during hold update
- ROCK currently rewrites both:
  - `m_target_bRca`
  - `transformB.rotation`

Because the FO4VR SetLocalTransforms atom is consumed in the solver path each step, rewriting
`transformB.rotation` changes the live constraint frame itself, not only the motor target.

## Corrections to prior assumptions

### 1. The previously cited mid-function PCs for runtime-pointer advancement were off

The old notes cited:

- `0x141A57162`
- `0x141A5873C`

Fresh inspection shows those exact PCs are later `CMP` instructions inside `FUN_141A55550`, not the
`ADD [RSI+...]` instructions that were being referenced.

The nearby adds are at:

- `0x141A57153`
- `0x141A5872D`

But this re-audit did **not** re-establish those case tails as clean proofs of ragdoll/linear motor
runtime layout. The earlier claim that they directly proved the custom runtime pointer increments is
not yet re-confirmed.

## Current verdict

### What is now lower-probability

- a simple absolute-offset bug in `transformB.rotation`
- a bad `getRuntimeInfo` slot number
- a bad `getRuntimeInfo` field order

### What remains plausible

- `m_target_bRca` is still worth a dedicated direct-consumption check, but the stock object layout
  supports ROCK's current absolute placement more than it contradicts it
- the stronger remaining semantic difference from HIGGS is that ROCK mutates both the constraint
  frame (`transformB.rotation`) and the angular target (`m_target_bRca`) every frame

### Narrowest remaining fault domain after this pass

1. solver-side meaning of the matrix written into `m_target_bRca`
2. whether `transformB.rotation` should remain the grab-time body-space frame instead of being
   updated per-frame
3. runtime comparison of live held-body rotation vs `_heldNode->world.rotate`

The fresh binary evidence shifts the main suspicion away from a raw `transformB.rotation` absolute
offset mismatch and toward the angular-frame contract itself.
