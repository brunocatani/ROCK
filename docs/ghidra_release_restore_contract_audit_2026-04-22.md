# Ghidra Audit — Release / Restore Contract — 2026-04-22

## Why this audit exists

The current release path in `HandGrab.cpp` still mixes three different assumptions:

1. long-term motion restoration is handled by native FO4VR lifecycle code
2. temporary release-time restore may need manual motion-properties writes
3. collision-object recovery can fall back through `refr->Get3D()->collisionObject`

This audit was done to settle the native FO4VR contract for release/restoration before changing source:

- whether release/deactivation resolves through the live body/collision-object chain or a root-3D path
- whether motion reset is driven by `SetMotionType`, world motion helpers, or explicit motion-properties restoration
- whether collision filter restoration belongs to a collision-object wrapper or the world/body setter

## Wave items and verdicts

| Item | Raw identifier | Existing interpretation | Blind verdict |
|---|---|---|---|
| 1 | `0x141E07300` | `bhkNPCollisionObject::SetMotionType` | **CONFIRMED** |
| 2 | `0x141DF5B80` / `0x14153AF00` | world/body collision-filter setter wrapper | **CONFIRMED** |
| 3 | `0x141E07990` | body -> collision object back-pointer helper | **CONFIRMED** |
| 4 | `0x14061D040` | post-load / deactivation-side restore helper | **PARTIAL** |
| 5 | `0x14061CC80` | body-activated callback with release/deactivation implications | **PARTIAL** |
| 6 | `0x14061CE90` | activation/reattachment-side helper | **PARTIAL** |
| 7 | `0x1401CD900` | collision-object helper that combines filter + motion-type change | **CONFIRMED** |
| 8 | `0x14153B2F0` | world/body motion-properties setter | **CONFIRMED** |

## Detailed findings

### Item 1 — `0x141E07300`

Blind read:

- resolves body id through the collision object's physics-system instance
- branches on requested motion type:
  - `param_2 == 2` path
  - `param_2 == 1` path
  - `param_2 == 0` path
- for the non-static paths, routes into world/body helpers rather than mutating only local wrapper state

Conclusion:

This is the authoritative collision-object motion-type wrapper. It is not a thin flag flip; it is the native transition path.

Important implication:

When FO4VR wants to force a body back into a specific motion-type state from the object side, it uses `SetMotionType(collisionObject, type)`, not a root-node lookup followed by ad hoc raw writes.

### Item 2 — `0x141DF5B80` / `0x14153AF00`

Blind read:

- `0x141DF5B80` is a thin wrapper into `0x14153AF00`
- `0x14153AF00`:
  - locks the world
  - indexes the live body row
  - writes `body+0x44` when the filter actually changes
  - if `rebuildMode == 0` and `body+0x6C != -1`, calls `0x14153C5A0`
  - emits the body-changed signal at `world+0x538`

Conclusion:

The authoritative primitive for collision-filter restoration is the world/body setter.

Important implication:

Any higher-level object-side collision-filter helper bottoms out here. This is the actual write path ROCK should treat as authoritative.

### Item 3 — `0x141E07990`

Blind read:

- returns `*(body + 0x88)`

Conclusion:

This is the direct body -> collision object back-pointer helper.

Important implication:

Native restore/deactivation helpers can resolve the collision object from the live body without ever going through `refr->Get3D()`.

### Item 4 — `0x14061D040`

Blind read:

- begins by resolving a collision-object-like pointer from `param_2` through `0x141E07990`
- performs several object/reference cleanup steps
- if a fixed-object check succeeds, calls the 5-parameter helper `0x141DF95B0`
- on the later narrow branch:
  - checks `((body+0x44) & 0x7F) == 0x14`
  - skips actors
  - resolves collision object from the live body again
  - calls `0x141E07300(collisionObject, 2)`

Verdict reasoning:

The body is clear enough to prove behavior, but the exact historical symbolic name is not required for this wave and remains stronger than necessary.

Important implication:

This helper does **not** treat `refr->Get3D()->collisionObject` as authoritative. It works from the live body/collision-object chain and only forces `SetMotionType(..., 2)` on a narrow non-actor layer-`0x14` branch.

### Item 5 — `0x14061CC80`

Blind read:

- resolves the live body row from `world + 0x20 + bodyId * 0x90`
- resolves collision object from the body via `0x141E07990`
- resolves the reference from the collision object owner node
- if `(bodyFlags & 8) == 0`, returns
- if `(bodyFlags & 5) == 0`, routes dynamic tracked refs into `0x14061CE90(ref, 1)`
- otherwise, for a narrower branch:
  - requires layer `0x14`
  - resolves the collision object again
  - if not actor, calls `0x141E07300(collisionObject, 2)`

Conclusion:

This callback again proves that the native restore/reset path is body/collision-object driven, not root-node driven.

### Item 6 — `0x14061CE90`

Blind read:

- performs reference-side activation/reattachment work
- writes tracked-object byte state through `0x140E7E620(..., 1)` on one path
- can reattach the reference to another parent/container path
- does not itself restore arbitrary motion-properties presets

Conclusion:

The native activation side appears to be gameplay/reference bookkeeping plus object reattachment, not a general “restore previous physics preset” helper.

### Item 7 — `0x1401CD900`

Blind read:

- calls `collisionObject->vtable+0x178` with a filter value
- calls another object-side virtual with the second property
- then calls `0x141E07300(collisionObject, 2)`

Conclusion:

FO4VR does have an object-side helper that combines:

- collision filter change
- another collision-object-side property write
- motion-type change to `2`

Important implication:

This is a special combined transition helper, not proof that general filter restoration should avoid the world/body setter.

### Item 8 — `0x14153B2F0`

Blind read:

- world/body-side motion-properties setter
- direct body row lookup
- reads `body+0x68` motion index
- writes through the motion array / motion-properties machinery

Additional caller spot-checks:

- `0x141E21CF0` resolves a body id through an owning wrapper, computes/selects a preset, then calls `0x14153B2F0`
- `0x14154B960` calls `SetTransform`-side body update, then forces motion-properties preset `3`

Conclusion:

The authoritative primitive for arbitrary motion-properties restoration is world/body-side, not a clean collision-object wrapper.

## Questions answered

### 1. Is there a collision-object-backed release path we should be using instead of root-only fallback?

Yes.

The native restore/deactivation helpers resolve from:

- live body
- `body+0x88` collision object
- owner-node/reference

They do **not** require `refr->Get3D()->collisionObject` as the authoritative path.

Repair implication:

The current release code in `HandGrab.cpp` that tries:

- `_savedObjectState.refr->Get3D()`
- then `root->collisionObject`

is stale as the primary restore path. If release needs the collision object, it should prefer the live body -> collision object chain first.

### 2. What does FO4VR expect for restoring motion properties on a held object after constraint destruction?

What the binary proves:

- native long-term lifecycle reset is motion-type driven, not “restore previous motionPropertiesId on release” driven
- `0x14061D040` and `0x14061CC80` force `SetMotionType(..., 2)` only on a narrow non-actor layer-`0x14` path
- fixed-object handling routes through the separate 5-parameter helper `0x141DF95B0`
- arbitrary motion-properties preset changes are done through `0x14153B2F0` on the world/body side

Practical conclusion:

FO4VR does **not** expose a proven native “release this dynamic object and restore its previous preset immediately through the collision object” contract. The native reset behavior is lifecycle/motion-type based. If ROCK wants to restore a specific original dynamic preset while keeping the object dynamic, the only audited primitive for that is the world/body motion-properties setter.

### 3. Is collision filter restoration better done through a wrapper object or the world/body function only?

For the primitive authoritative write:

- world/body function only: `0x14153AF00`

What also exists:

- a higher-level collision-object helper `0x1401CD900` that combines filter change + extra property + `SetMotionType(..., 2)` for a specific transition pattern

Practical conclusion:

General release-time collision-filter restoration should treat the world/body setter as authoritative. The object-side combined helper is relevant only when ROCK wants to deliberately mirror that exact native transition pattern, not as the generic restore primitive.

## Repair implication

Wave 3 sharpens the earlier source finding:

1. `HandGrab.cpp` root-only collision-object fallback is stale.
2. Release-time “motion preset restore” should not be described as something the native lifecycle already does generically.
3. If ROCK chooses to preserve the object's original dynamic preset on release, that is a deliberate ROCK-side policy and should use the world/body motion-properties setter, not a guessed collision-object restore path.
4. Collision-filter restoration should remain on the world/body setter unless we intentionally model the native combined helper used for special object-state transitions.

## Net result

**Wave 3 is complete.**

The native FO4VR release/reset path is body/collision-object driven, not root-3D driven. Motion-type restoration is native and object-side; arbitrary motion-properties restoration is world/body-side. Collision-filter restoration is authoritative at the world/body setter, with only a narrower combined collision-object helper for special transitions.
