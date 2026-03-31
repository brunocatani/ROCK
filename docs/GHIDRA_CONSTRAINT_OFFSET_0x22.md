# Constraint Entry +0x22 — Ghidra Blind Analysis (2026-03-31)

## Question
What lives at offset +0x22 (int16) within the 0x38-byte constraint entry struct at world+0x128?

Two competing names:
- "atomSize" — total size of constraint atom chain
- "numSolverResults" — count of solver result entries

## Verdict: **sizeOfSchemas** (total Jacobian schema size in bytes)

Neither competing name is precisely correct.

- "atomSize" is close but imprecise — it's not the size of the atoms, but the size of the Jacobian schemas generated FROM the atoms.
- "numSolverResults" is incorrect — that comes from a different field in the getConstraintInfo output.

## Evidence

### hknpConstraint::init (0x1417e39c0)

The key line:
```c
*(short *)(constraint + 0x22) = local_34 + (short)local_1c;
```

Disassembly:
```asm
0x1417e3ad4: ADD AX, word ptr [RSP + 0x3c]   ; AX = sizeOfSchemas + extraSchemaSize
0x1417e3ad9: MOV word ptr [R15 + 0x22], AX    ; store to constraint+0x22
```

Where the values come from:
1. `local_34` (at RSP+0x24) = output of `constraintData->getConstraintInfo()` vtable call (slot 5, vtable+0x28)
2. `local_1c` (at RSP+0x3C) = initialized to 0 before the call, matches `ConstraintInfo() { m_extraSchemaSize = 0; }`

### getConstraintInfo output struct mapping

The vtable call fills a struct starting at RSP+0x20. Cross-referencing with hkpConstraintInfo (Havok 2013 SDK):

| Offset in output | SDK field | How used in init |
|---|---|---|
| +0x00 (RSP+0x20) | m_maxSizeOfSchema | stored to constraint+0x20 (via local_20 at RSP+0x38 -- note: hknp may repack) |
| +0x04 (RSP+0x24) | **m_sizeOfSchemas** | **stored to constraint+0x22** (the field in question) |
| +0x08 (RSP+0x28) | m_numSolverResults + m_numSolverElemTemps? | stored to constraint+0x18 |
| +0x0C (RSP+0x2C) | (byte field) | stored to constraint+0x25 |

**Critical:** offset +0x04 in hkpConstraintInfo is `m_sizeOfSchemas`, NOT `m_numSolverResults` (which is at +0x08).

### Constraint destruction (0x141546b40) and cleanup (0x141555ac0)

Neither function reads +0x22. The cleanup function uses:
- +0x26 (runtimeSize) for memory deallocation
- +0x28 (runtimeData pointer) to find the allocation
- +0x08 (constraintData pointer) for refcount decrement

This confirms +0x22 is a solver-side value, not a memory management value.

## Updated Constraint Entry Layout (0x38 bytes)

| Offset | Size | Field | Notes |
|---|---|---|---|
| +0x00 | 4 | bodyIdA | uint32 |
| +0x04 | 4 | bodyIdB | uint32 |
| +0x08 | 8 | constraintData* | hkReferencedObject pointer |
| +0x10 | 4 | constraintId | uint32 |
| +0x14 | 2 | (unknown) | set to 0xFFFF during init |
| +0x16 | 1 | flags | uint8, bit2=disabled, bit0+bit7=contact constraint |
| +0x17 | 1 | constraintType | uint8, from getType() vtable call |
| +0x18 | 8 | (constraintInfo data) | from getConstraintInfo output |
| +0x20 | 2 | maxSizeOfSchema | int16, from getConstraintInfo |
| **+0x22** | **2** | **sizeOfSchemas** | **int16, total Jacobian schema size = m_sizeOfSchemas + m_extraSchemaSize** |
| +0x24 | 1 | (solver temp info) | from getRuntimeInfo output |
| +0x25 | 1 | (numSolverElemTemps?) | byte from getConstraintInfo |
| +0x26 | 2 | runtimeSize | int16, size of external runtime allocation |
| +0x28 | 8 | runtimeData* | pointer to allocated solver runtime |
| +0x30 | 8 | (padding/unused) | zeroed during init |
