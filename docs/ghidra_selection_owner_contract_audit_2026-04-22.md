# Ghidra Audit — Selection / Owner Contract — 2026-04-22

## Why this audit exists

The source review found a stale contract split in the selection path:

- `resolveBodyToRef()` resolves through `bhkNPCollisionObject::Getbhk(...)->sceneObject`
- `isGrabbable()` still requires `ref->Get3D()->collisionObject`

The key question is whether FO4VR actually guarantees that the root 3D node owns the collision object for a grabbable ref, or whether the authoritative runtime owner can be a child node. This audit was done against the FO4VR binary to settle that contract before changing source.

## Wave items and verdicts

| Item | Raw identifier | Existing interpretation | Blind verdict |
|---|---|---|---|
| 1 | `0x142996CB0` | `NiCollisionObject::vfunction43` owner-link routine | **CONFIRMED** |
| 2 | `0x141E07AC0` | `bhkNPCollisionObject::vfunction48` create/instance seeding path | **CONFIRMED** |
| 3 | `0x141E07A10` | `bhkNPCollisionObject::Getbhk(bhkWorld*, hknpBodyId&)` | **CONFIRMED** |
| 4 | `0x141E07E30` | collision-object -> live-body resolver | **CONFIRMED** |
| 5 | `0x140408580` / surrounding block | `TESObjectREFR::FindReferenceFor3D(NiAVObject*)` | **PARTIAL** |

## Detailed findings

### Item 1 — `0x142996CB0`

Blind read:

- writes `this + 0x8 = param_1`
- if `param_1 != 0`, reads `*(param_1 + 0x100)`
- writes `*(param_1 + 0x100) = this`
- decrements/releases the old collision object if one was already attached
- if the newly linked owner node now points back inconsistently, calls the owner vfunc at `vtable + 0x150`

Conclusion:

This is the bidirectional collision-object ↔ owner-node link routine.

Confirmed runtime contract:

- `collisionObject + 0x8` = owner `NiAVObject`
- `ownerNiAVObject + 0x100` = collision object

Important implication:

The owner is **a specific `NiAVObject`**, not necessarily the root 3D node.

### Item 2 — `0x141E07AC0`

Blind read:

- reads the owner node from `this->NiRefObject_data.offset_0x8`
- if owner exists, copies world transform data from owner world fields around `+0x70`
- if owner does not exist, uses identity/default transform data
- calls another helper with that transform
- resolves the live body via `FUN_141e07e30(this)`
- writes `body + 0x88 = this`

Conclusion:

This is the collision-object/body instancing path that seeds the body from the owner node world transform and then installs the body back-pointer.

Confirmed runtime contract:

- body creation/instancing is driven from the linked owner node world transform
- the live body stores `body + 0x88 = bhkNPCollisionObject*`

Important implication:

Again, the authoritative runtime owner is the linked node on the collision object, not a special “root-only” field.

### Item 3 — `0x141E07A10`

Blind read:

- validates `param_1 != 0`
- reads a world-like pointer from `param_1 + 0x60`
- rejects sentinel body ID `0x7fffffff`
- locks the world at `world + 0x690`
- reads the body array base from `world + 0x20`
- returns `*(bodyBase + bodyId * 0x90 + 0x88)`

Conclusion:

This is the runtime collision-object lookup from `(bhkWorld, bodyId)`.

Confirmed runtime contract:

- body → collision object lookup is `body + 0x88`
- the engine helper uses world locking and the authoritative world/body array path

Important implication:

For owner lookup, the engine-backed path is the authoritative contract. A hand-written “root node owns collision” assumption is weaker than the actual runtime path.

### Item 4 — `0x141E07E30`

Blind read:

- reads the collision object's physics-system-like pointer from `param_1 + 0x20`
- resolves a body ID through helper calls
- resolves the world
- locks the world
- returns `worldBodyArray + bodyId * 0x90`

Conclusion:

This is the inverse helper: collision object -> live body pointer.

Confirmed runtime contract:

- the wrapper family is built around a real body ↔ collision object ↔ owner-node chain

### Item 5 — `0x140408580` / surrounding block

Address-library evidence:

- local VR address data maps `REL::ID(766937)` to `0x140408580` for `TESObjectREFR::FindReferenceFor3D(NiAVObject*)`

Ghidra evidence:

- no function starts exactly at `0x140408580`
- the surrounding function block begins at `0x140408530`
- that block delegates to helper routines that search/maintain sets of 3D nodes associated with references

Verdict:

**PARTIAL**

I did not rely on the historic label alone, but the exact decompilation slice around the address-library location is not clean enough to make a stronger blind identity claim in this pass. What is clear is that the resolution path operates on a node-membership/search mechanism, not on a hardcoded “root node only” assumption.

This is enough for the selection contract question in this wave, but the exact `FindReferenceFor3D` implementation can be re-audited separately if needed.

## Questions answered

### 1. Is `sceneObject` the authoritative collision owner?

**Yes, for the collision-object ownership contract.**

The binary shows that:

- the collision object stores an owner `NiAVObject`
- the owner node stores the collision object back
- the body back-pointer returns to the collision object
- `Getbhk(world, bodyId)` returns the collision object from the live body

So the direct runtime path from a live hit body is:

- `bodyId -> Getbhk(...) -> collisionObject -> sceneObject`

That is the authoritative owner-node chain.

### 2. Under what cases does `ref->Get3D()->collisionObject` differ from the actual hit owner?

The binary evidence does not enumerate every gameplay case, but it does prove the owner is any linked `NiAVObject`, not a special root-only node. Therefore any ref whose collision object is attached to a child `NiAVObject` is a valid runtime case, and the root-node-only assumption is not guaranteed by the engine contract.

### 3. Does the binary support a required root-node filter in `isGrabbable()`?

**No.**

Nothing in the audited owner-link or body-lookup path requires the root `Get3D()` node to own the collision object. The current source filter in `isGrabbable()` is therefore stricter than the proven runtime contract.

### 4. What should the source treat as authoritative for selection ownership?

For the hit/physics path:

- the collision owner node reached through `Getbhk(...)->sceneObject`

For ref resolution:

- continue using `TESObjectREFR::FindReferenceFor3D(...)` from the actual hit owner node or hit node, not from a forced root-only assumption

## Repair implication

The current source split is stale:

- `resolveBodyToRef()` already uses the engine-backed owner-node path
- `isGrabbable()` still enforces a root-node collision-object requirement

That root-node requirement should not be treated as authoritative in the repair phases.

## Net result

**The source review finding is confirmed.** The current selection filter is using a stronger and older assumption than the binary-backed ownership contract actually guarantees.
