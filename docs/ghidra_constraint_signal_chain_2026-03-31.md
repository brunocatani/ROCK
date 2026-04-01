# Ghidra RE: Constraint Creation Signal Chain (2026-03-31)

## Summary

**YES** - `hknpWorld::createConstraint` at `0x1415469b0` triggers `hknpBSWorld::addConstraintBodyMapEntry` at `0x141df66d0` via a Havok signal dispatch mechanism. The call is indirect through a linked-list signal at hknpWorld offset `+0x550`.

## Complete Call Chain

```
hknpWorld::createConstraint (0x1415469b0)
  |
  +-- [always] FUN_14155a2f0(world+0x550, world, constraintCinfo, constraintId)
  |     This is a SIGNAL DISPATCH function - iterates a linked list of listeners
  |     registered on the signal slot at world+0x550
  |       |
  |       +-- [registered listener] hknpBSWorld::addConstraintBodyMapEntry (0x141df66d0)
  |             Reads bodyIdA from cinfo+0x08, bodyIdB from cinfo+0x0C
  |             Inserts BOTH directions into hash map at world+0x718:
  |               map[bodyIdA] = bodyIdB
  |               map[bodyIdB] = bodyIdA
  |
  +-- [conditional: bodyIdA != bodyIdB AND constraint type != 3]
        hknpWorldEx::handleConstraintActivationMode (0x14154a810)
        FUN_14155a050(world+0x560, world, constraintId)  -- another signal dispatch
```

## Signal Registration

The registration happens in `FUN_141df6030` (called from `FUN_141df99e0` which is `bhkWorld::createHavokWorld` or similar):

```
FUN_141df6030(hknpWorld* world):
    // Register 4 signal listeners:
    register(world+0x580, activatePendingBodiesPostCollide)   // FUN_141dfea40 - 4-arg signal
    register(world+0x4d8, addBodyManageConstrainedActivation) // FUN_141dfebf0 - different signal type
    register(world+0x550, addConstraintBodyMapEntry)          // FUN_141dfead0 - 3-arg signal
    register(world+0x558, removeConstraintBodyMapEntry)       // FUN_141dfeb60 - 3-arg signal
```

This is called during world construction at `0x141df99e0` (line: `FUN_141df6030(plVar14)`), which constructs an `hknpBSWorld` (Bethesda's subclass of hknpWorld).

## Signal Dispatch Mechanism (FUN_14155a2f0)

Address: `0x14155a2f0`
This is a generic signal dispatcher. It:
1. Sets dispatch-in-progress flag (bit 0 of signal slot)
2. Walks a linked list of listener nodes (each node has: vtable ptr, next ptr, callback ptr)
3. For each active listener (bits 0-1 of next ptr == 0), calls `node->vtable[2](node, ...args)`
4. If listener marks itself for removal during dispatch, calls destructor
5. Clears dispatch flag when done

The linked list head is stored at the signal slot address (world+0x550 for constraint add).

## Item Details

### Item 1: 0x1415469b0 - hknpWorld::createConstraint

**What it is:** Creates a physics constraint between two bodies. Steps:
1. Acquires world lock (world+0x690)
2. If constraint buffer is at capacity (0x7FFFFFFF), triggers buffer expansion signal (world+0x540)
3. Allocates constraint ID from free list (world+0x120)
4. Initializes constraint entry at `constraintArray + id * 0x38` (stride confirmed 0x38)
5. **Fires "constraint added" signal** at world+0x550 with (world, constraintCinfo, constraintId) -- THIS IS THE KEY DISPATCH
6. Checks constraint type via vtable call (slot 0x20): if type == 3, logs error "Unsupported constraint type" and disables
7. If bodyIdA != bodyIdB: calls handleConstraintActivationMode, then fires signal at world+0x560
8. If bodyIdA == bodyIdB: disables constraint (sets bit 2 at entry+0x16)
9. Releases world lock

**Proposed name:** hknpWorld::createConstraint (already named by Ghidra)
**Key evidence:** String "Dynamics\\World\\hknpWorld.cpp" at line 0x4BF, constraint stride 0x38, signal dispatch pattern

### Item 2: 0x141df66d0 - hknpBSWorld::addConstraintBodyMapEntry

**What it is:** Bethesda extension that maintains a bidirectional adjacency map of constrained body pairs.
1. Reads bodyIdA from constraintCinfo+0x08
2. Reads bodyIdB from constraintCinfo+0x0C
3. Calls `FUN_141e06140(world+0x718, bodyIdA, bodyIdB)` - inserts A->B mapping
4. Calls `FUN_141e06140(world+0x718, bodyIdB, bodyIdA)` - inserts B->A mapping

The map at world+0x718 is a hash map (FUN_141e06140 uses hash: `(id >> 4) * 0x9E3779B1` which is the golden ratio hash).

**Proposed name:** hknpBSWorld::addConstraintBodyMapEntry (already named)
**Counterpoint:** Could be any body-pair tracking (collision pairs, etc.), but the signal name string in FUN_141df6030 explicitly says "addConstraintBodyMapEntry"
**Key evidence:** String "hknpBSWorld::addConstraintBodyMapEntry" in registration, bidirectional insertion, reads from cinfo offsets +0x08/+0x0C which match hknpConstraintCinfo bodyIdA/bodyIdB

### Item 3: 0x14046c700 - setMotionTypeIfUnconstrained

**What it is:** Conditionally sets a body's motion type, but ONLY if the body has no constraints.
1. Calls `FUN_141e09170(body)` to check if body has any constraints
2. If result is false (no constraints), calls `bhkNPCollisionObject::SetMotionType(body, 2)` (2 = DYNAMIC)
3. Returns 0

**Proposed name:** `bhkNPCollisionObject::setMotionTypeDynamicIfUnconstrained` or `tryRestoreMotionType`
**Counterpoint:** Could be a generic "make dynamic" function that happens to check constraints. But the guard pattern strongly suggests it's protecting against changing motion type on constrained bodies.
**Key evidence:** Direct call to FUN_141e09170 as a guard, only proceeds if unconstrained, hardcoded motion type 2

### Item 4: 0x141e09170 - hasConstraint / isBodyConstrained

**What it is:** Checks if a body has any constraints by iterating a constraint list.
1. Checks if pointer at param_1+0x20 is null (no constraint data) -> returns false
2. Gets count from `FUN_141e0c7a0()` which reads `*(*(param+0x10)+0x58)` -- constraint count
3. Iterates through constraint entries via `FUN_141e0c760(*(param+0x20), index)` which computes `base + index * 0x18`
4. For each entry, compares `entry+0x08` against `*(param+0x28)` (the body's own ID)
5. Returns true if any match found

**Proposed name:** `hknpBSWorldUtil::isBodyConstrained` or `hasConstraintForBody`
**Counterpoint:** Could be checking for a specific constraint (not just any), but the early-exit-on-first-match pattern and boolean return strongly suggest "has any constraint"
**Key evidence:** Iterates array with stride 0x18, compares body ID field, returns bool, constraint entry count from +0x58

## Constraint Body Map Structure (world+0x718)

The map is a Havok hash map:
- `+0x00`: data pointer (array of 8-byte entries: 4-byte key + 4-byte value)
- `+0x08`: count (number of entries)
- `+0x0C`: capacity mask (used in hash: `hash & mask`)

Hash function: `(bodyId >> 4) * 0x9E3779B1` (Knuth multiplicative hash with golden ratio constant)

Linear probing for collision resolution (increments index until finding -1 sentinel).

## Key Offsets Summary

| Offset (from hknpWorld) | What | Notes |
|---|---|---|
| +0x020 | Body array base | stride 0x90 |
| +0x120 | Constraint ID allocator | free list |
| +0x128 | Constraint array pointer | stride 0x38 per entry |
| +0x13C | Constraint capacity marker | 0x7FFFFFFF = full |
| +0x498 | Activation mode enabled flag | checked by handleConstraintActivationMode |
| +0x4A0 | Simulation islands pointer | used for activation propagation |
| +0x540 | Signal: constraint buffer full | |
| +0x550 | Signal: constraint added | **addConstraintBodyMapEntry registered here** |
| +0x558 | Signal: constraint removed | removeConstraintBodyMapEntry registered here |
| +0x560 | Signal: constraint activation changed | |
| +0x580 | Signal: pending bodies post-collide | |
| +0x690 | World mutex/lock | |
| +0x718 | Constraint body adjacency map | Bethesda extension, hash map |

## Does createConstraint Filter by Constraint Type for the Signal?

**NO.** The signal dispatch at world+0x550 (FUN_14155a2f0) fires UNCONDITIONALLY for ALL constraint types. Looking at the assembly:

```asm
141546a63: MOV R9D,dword ptr [R15]       ; constraintId
141546a66: LEA RCX,[RDI + 0x550]         ; signal slot
141546a6d: MOV R8,R14                    ; constraintCinfo
141546a70: MOV RDX,RDI                   ; world
141546a73: CALL 0x14155a2f0              ; DISPATCH -- no type check before this
141546a78: TEST byte ptr [RSI + 0x16],0x4 ; AFTER dispatch, check disable flag
```

The constraint type check (vtable[0x20] == 3) happens AFTER the signal dispatch. So:
- addConstraintBodyMapEntry fires for ALL constraint types including unsupported ones
- The type-3 disable and the bodyIdA==bodyIdB disable happen after the map entry is already added
- This means the body map may contain entries for disabled constraints

## Implications for ROCK

1. When ROCK creates a constraint via `hknpWorld::createConstraint`, the engine AUTOMATICALLY registers both bodies in the adjacency map at world+0x718. No manual registration needed.

2. The `isBodyConstrained` check at `0x141e09170` queries a DIFFERENT structure (not the hash map) -- it iterates what appears to be a direct constraint list on the body/collision object itself (param+0x20). This might be the bhkNPCollisionObject's own constraint tracking.

3. The `setMotionTypeIfUnconstrained` pattern at `0x14046c700` shows Bethesda's own code guards against changing motion type on constrained bodies. ROCK should do the same -- never change a body's motion type to DYNAMIC while it has active constraints, or the constraint solver will fight the motion type change.

4. The removeConstraintBodyMapEntry signal at world+0x558 is the mirror -- it fires from `hknpWorld::destroyConstraints` (confirmed by xrefs to FUN_14155a050 which dispatches on +0x560, and the registration shows +0x558 for remove).
