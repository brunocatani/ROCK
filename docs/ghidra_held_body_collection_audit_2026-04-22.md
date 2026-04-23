# Ghidra Audit — Held Body Collection Contract — 2026-04-22

## Why this audit exists

`Hand.cpp::collectBodyIdsRecursive()` still walks scene-node collision objects through a heuristic contract:

- read raw `collisionObject + 0x20`
- treat it as a `bhkPhysicsSystem*` if the value is "pointer-like"
- read `system->instance`
- enumerate body ids

The source review flagged that as stale because it was not clear whether:

1. raw `bhkNPCollisionObject + 0x20` is really the physics-system pointer in FO4VR
2. the paired body/system-local index lives beside it
3. sibling collision-object subclasses share the same slots
4. the current "pointer above 64KB" check is a real type discriminator or only a heuristic

This audit was done to settle the field layout and subclass contract before changing held-body ownership collection.

## Wave items and verdicts

| Item | Raw identifier | Existing interpretation | Blind verdict |
|---|---|---|---|
| 1 | `0x141E07710` | `bhkNPCollisionObject` value ctor | **CONFIRMED** |
| 2 | `0x141E0B0E0` | `bhkNPCollisionObject` default ctor | **CONFIRMED** |
| 3 | `0x141E0A050` | `bhkNPCollisionObjectUnlinked` value ctor | **CONFIRMED** |
| 4 | `0x141E2A160` | `bhkNPCollisionObjectUnlinked` default ctor | **CONFIRMED** |
| 5 | `0x141E07AC0` | `bhkNPCollisionObject::vfunction48` create/link path | **CONFIRMED** |
| 6 | `0x141E07E30` | collision object -> live body helper | **CONFIRMED** |
| 7 | `0x141E205A0` | `bhkCharacterController` ctor | **CONFIRMED** |
| 8 | `0x141E0C460` | physics-system body-id lookup helper | **CONFIRMED** |

## Important offset note

Ghidra renders several fields in these functions as members of `this->NiRefObject_data`, which begins **after** the vtable pointer.

That means:

- `NiRefObject_data.offset_0x18` in the decompiler corresponds to raw `this + 0x20`
- `NiRefObject_data.offset_0x20` in the decompiler corresponds to raw `this + 0x28`

This matters because the blind read initially looked like "`+0x18` is the physics-system pointer", but once the vtable offset is accounted for, the raw object layout remains:

- raw `bhkNPCollisionObject + 0x20` = physics-system pointer
- raw `bhkNPCollisionObject + 0x28` = body/system-local index

## Detailed findings

### Item 1 — `0x141E07710`

Blind read:

- calls `NiCollisionObject::NiCollisionObject((NiCollisionObject*)this, param_3)`
- writes `(this->NiRefObject_data).offset_0x18 = param_2`
- writes `*(uint*)&(this->NiRefObject_data).offset_0x20 = param_1`

After converting from the decompiler-relative offsets to raw object offsets:

- raw `this + 0x20 = param_2`
- raw `this + 0x28 = param_1`

Conclusion:

The non-default `bhkNPCollisionObject` constructor stores a pointer/value pair in the exact slots currently assumed by ROCK:

- raw `+0x20` pointer field
- raw `+0x28` integer index field

### Item 2 — `0x141E0B0E0`

Blind read:

- constructs `NiCollisionObject`
- sets the same family vtable
- writes `(this->NiRefObject_data).offset_0x18 = 0`
- writes `*(uint*)&(this->NiRefObject_data).offset_0x20 = 0xffffffff`

Converted raw offsets:

- raw `+0x20 = 0`
- raw `+0x28 = -1`

Conclusion:

The default NPCollisionObject state is explicitly:

- no physics-system pointer
- invalid body/system-local index

### Item 3 — `0x141E0A050`

Blind read:

- inlines the NPCollisionObject setup
- then swaps to the `bhkNPCollisionObjectUnlinked` vtable
- writes the same value pair as Item 1

Converted raw offsets:

- raw `+0x20 = param_2`
- raw `+0x28 = param_1`

Conclusion:

`bhkNPCollisionObjectUnlinked` shares the same pointer/index storage contract as `bhkNPCollisionObject`.

### Item 4 — `0x141E2A160`

Blind read:

- default-constructs the unlinked variant
- leaves raw `+0x20 = 0`
- leaves raw `+0x28 = -1`

Conclusion:

The unlinked default state matches the linked family's default semantics:

- null physics-system pointer
- invalid local index

### Item 5 — `0x141E07AC0`

Blind read:

- reads `(this->NiRefObject_data).offset_0x18`
- passes that pointer into the physics-system helpers
- calls `FUN_141E0C320((longlong*)(this->NiRefObject_data).offset_0x18, ...)`
- resolves the live body through `FUN_141E07E30(this)`
- writes `body + 0x88 = this`

Converted raw offsets:

- the helper input is raw `this + 0x20`

Conclusion:

The create/link path itself uses raw `bhkNPCollisionObject + 0x20` as the physics-system-bearing field. This is the strongest blind witness in this wave that the raw offset used by ROCK is structurally correct.

### Item 6 — `0x141E07E30`

Blind read:

- if `*(this + 0x20) == 0`, uses sentinel body id
- otherwise calls `FUN_141E0C460(*(this + 0x20), ..., *(int*)(this + 0x28))`
- resolves the world from that same `*(this + 0x20)` path
- returns `world->bodyArray + bodyId * 0x90`

Conclusion:

This helper fully settles the raw pair:

- raw `collisionObject + 0x20` is the physics-system pointer
- raw `collisionObject + 0x28` is the per-system body index used to recover the live body id

This also means the current review concern is **not** "ROCK uses the wrong raw offset". The offset is correct.

### Item 7 — `0x141E205A0`

Blind read:

- builds the `bhkCharacterController` object through the NPCollisionObject family path
- leaves `(this->NiRefObject_data).offset_0x18 = 0`
- leaves `*(uint*)&(this->NiRefObject_data).offset_0x20 = 0xffffffff`
- later installs the controller vtables and controller-specific fields

Converted raw offsets:

- raw `+0x20 = 0`
- raw `+0x28 = -1`

Conclusion:

`bhkCharacterController` shares the same family slots, but its default controller state is not an active grabbable-object physics-system binding.

Important implication:

The current source heuristic:

- "read raw `+0x20` and accept if it looks pointer-like"

is **not** a real type discriminator. It is only a semantic state probe. It can reject controller-family instances in their default state, but it does not prove the object is the intended grabbable-object collision subtype.

### Item 8 — `0x141E0C460`

Blind read:

- resolves through the physics-system instance
- reads the body-id array from `instance + 0x20`
- uses caller-provided element index
- returns sentinel `0x7fffffff` when the instance is missing

Conclusion:

The authoritative held-body enumeration contract remains:

- collision object raw `+0x20` -> `bhkPhysicsSystem*`
- `bhkPhysicsSystem` -> instance
- instance `+0x20` -> body-id array
- instance `+0x28` -> body count
- collision object raw `+0x28` -> local index into that body-id array

## Questions answered

### 1. How should `bhkNPCollisionObject` be distinguished safely from proxy/object variants?

The blind audit does **not** support "pointer above 64KB" as an authoritative type discriminator.

What is proven:

- the NPCollisionObject family (`bhkNPCollisionObject`, `bhkNPCollisionObjectUnlinked`, `bhkCharacterController`) shares the raw `+0x20/+0x28` slot pair
- controller-family objects can legally have:
  - raw `+0x20 = 0`
  - raw `+0x28 = -1`

Practical conclusion:

The safer contract is semantic, not threshold-based:

1. collision object exists
2. raw `+0x20` is non-null
3. raw `+0x28` is not `-1`
4. the physics-system instance exists
5. the local index is in range of `instance + 0x28`

If a stronger exact subclass discriminator is wanted, it should be vtable/RTTI-backed, not a pointer-threshold heuristic.

### 2. What is the authoritative way to reach `bhkPhysicsSystem::instance` and body IDs from a collision object?

Confirmed path:

1. raw `collisionObject + 0x20` -> `bhkPhysicsSystem*`
2. `bhkPhysicsSystem` -> instance (already confirmed in earlier audits)
3. instance `+0x20` -> body-id array
4. instance `+0x28` -> body count
5. raw `collisionObject + 0x28` -> system-local index into that array

This path is exactly what `0x141E07E30` and `0x141E0C460` use together.

### 3. Are there collision-object subclasses in FO4VR that share the field shape but not the same ownership contract?

Yes, partially.

What is proven:

- `bhkCharacterController` shares the same family storage slots
- it also inherits the NPCollisionObject create/link family path
- but its default state is not the same as a live clutter/body-owned collision object

What is **not** proven in this wave:

- a sibling subclass with a non-null raw `+0x20` that points to something other than a `bhkPhysicsSystem`

Practical conclusion:

The current held-body recursion is still too heuristic, but the problem is narrower than "wrong offset". The raw field layout is right; the weak point is the untyped acceptance rule.

## Repair implication

Wave 4 corrects and narrows the source review finding:

1. ROCK's raw `kCollisionObject_PhysSystemPtr = 0x20` assumption is **correct** for the NPCollisionObject family.
2. The stale part is the **heuristic discriminator**, not the offset.
3. `collectBodyIdsRecursive()` should stop treating "pointer above 64KB" as the contract.
4. The safer recursion path is:
   - validate raw `+0x20` / raw `+0x28`
   - resolve instance through the verified physics-system contract
   - bounds-check against `instance + 0x28`
   - then enumerate body ids from `instance + 0x20`

## Net result

**Wave 4 is complete.**

The FO4VR binary confirms that held-body collection in ROCK is built on the correct raw field pair:

- raw `bhkNPCollisionObject + 0x20` = physics-system pointer
- raw `bhkNPCollisionObject + 0x28` = local body index

The remaining problem is not an incorrect offset assumption; it is that the current source uses an untyped state heuristic where the binary-backed contract should be:

- family slot validity
- live instance presence
- local-index bounds validation
