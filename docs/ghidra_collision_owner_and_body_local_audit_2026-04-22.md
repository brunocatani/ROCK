# FO4VR Collision Owner And Body-Local Audit — 2026-04-22

## Why this note exists

This audit answers three concrete questions that were still being discussed from headers,
parser guesses, and HIGGS comparison:

1. Does FO4VR have a real collidable -> owner-node helper path like HIGGS's
   `GetNodeFromCollidable()`?
2. Does FO4VR expose a clean equivalent of HIGGS's `GetRigidBodyTLocalTransform()`?
3. What binary facts already exist for the body -> collisionObject -> owner-node chain?

This pass uses the loaded FO4VR executable in Ghidra as ground truth.

## HIGGS reference point

HIGGS on Skyrim VR uses:

- `GetRigidBodyTLocalTransform(bhkRigidBody*)` in `higgs/src/utils.cpp:251`
- a real collidable-node path during grab math
- older `bhkRigidBodyT` / `hkp` contracts that expose local rigid-body transforms more directly

FO4VR is different:

- `hknp` backend
- `bhkNPCollisionObject` wrapper layer
- fewer useful surviving names
- more wrapper/data-chain reconstruction

## Verified functions and contracts

### 1. `NiCollisionObject::vfunction43` is the real link routine

- Address: `0x142996CB0`
- Ghidra label: `NiCollisionObject::vfunction43`

Decompiled behavior:

- writes `this + 0x8 = ownerNiAVObject`
- writes `ownerNiAVObject + 0x100 = this`
- releases the previous collision object if the owner already had one
- if the linked collision object still disagrees about its owner, calls the virtual at `vtable + 0x150`

Binary consequence:

- `NiCollisionObject + 0x8` is the owning `NiAVObject*`
- `NiAVObject + 0x100` is the attached `NiCollisionObject*`

This is the FO4VR collidable <-> owner-node contract. It is not a guess.

### 2. `bhkNPCollisionObject::vfunction48` is the real world instantiation path

- Address: `0x141E07AC0`
- Ghidra label: `bhkNPCollisionObject::vfunction48`

Decompiled behavior:

- ensures the wrapped `bhkPhysicsSystem` is instantiated for the target world
- if an owner node exists at `this + 0x8`, copies the owner node world transform from:
  - rotation/translation block at `owner + 0x70 .. 0xAC`
- passes that transform into the physics-system instantiation helper
- resolves the created body pointer
- writes `body + 0x88 = this`

Binary consequence:

- FO4VR seeds the physics body from the owning NiAVObject world transform
- body back-pointer to the collision object is real and written by the engine

### 3. `FUN_141E07E30` resolves the live body pointer from a collision object

- Address: `0x141E07E30`

Decompiled behavior:

- uses the collision object's wrapped physics-system pointer at `this + 0x20`
- uses stored body index at `this + 0x28`
- resolves the body ID with `FUN_141E0C460`
- resolves the world with `FUN_141E0C4E0`
- returns `world->bodyArrayBase + bodyId * 0x90`

Binary consequence:

- the runtime chain is:
  - `body -> body+0x88 -> bhkNPCollisionObject`
  - `bhkNPCollisionObject + 0x8 -> owner NiAVObject`

That is enough to write typed helpers without guessing.

### 4. Supporting helper meanings

- `0x141E0C460`
  - reads the body ID from the wrapped `bhkPhysicsSystem`
- `0x141E0C4E0`
  - returns the world pointer from the wrapped `bhkPhysicsSystem`
- `0x141E0C430`
  - checks whether the wrapped system is already instantiated for the requested world
- `0x141E0C320`
  - instantiates the wrapped system into the world when needed

## What this proves about `GetNodeFromCollidable`

We do not need to keep treating owner-node recovery as speculative.

FO4VR already exposes the equivalent contract, even though the symbol did not survive as a neat
named `GetSceneObject()` helper:

- from a `NiCollisionObject*`, the authoritative owner node is at `collisionObject + 0x8`
- from a body, the authoritative collision object is at `body + 0x88`

So the typed FO4VR helper is effectively:

- `GetCollisionObjectFromBody(body) -> *(body + 0x88)`
- `GetOwnerNodeFromCollisionObject(collObj) -> *(collObj + 0x8)`
- `GetOwnerNodeFromBody(body) -> *( *(body + 0x88) + 0x8 )`

That part is now binary-verified.

## What this does NOT prove about visual ownership

This audit proves the collision owner node contract.
It does **not** yet prove that the collision owner node is always the same node we should compare
against in held-object visual diagnostics.

Those can still differ:

- collision owner node
- hit/collidable node selected by mesh grab
- `refr->Get3D()` root
- final render geometry node

So the binary has already answered "how do we get the owner node from the collidable/body?",
but it has not yet answered "which of those scene nodes is the authoritative visual node during hold?"

## What this proves about `GetRigidBodyTLocalTransform`

This pass did **not** find a clean FO4VR equivalent of HIGGS's
`GetRigidBodyTLocalTransform(bhkRigidBody*)` in the audited `NiCollisionObject`,
`bhkNPCollisionObject`, and `bhkPhysicsSystem` family.

What the binary does show instead:

- FO4VR stores the owner node pointer on the collision object
- FO4VR seeds the body transform from the owner node world transform during instantiation
- FO4VR stores a body -> collisionObject back-pointer

What it does **not** show in this family:

- an exposed wrapper that returns a typed "rigidBodyT local transform"
- an obvious stored body-local transform field analogous to Skyrim VR's `bhkRigidBodyT`

Current verdict:

- HIGGS has a real helper because Skyrim VR's older `bhkRigidBodyT` path exposes that concept
- FO4VR likely does not expose the same concept as a surviving wrapper
- for FO4VR, reconstructing body-local transform from:
  - live body world transform
  - owner/collidable node world transform
  is not inherently suspicious

So the existence of `computeRuntimeBodyLocalTransform()` is not the problem by itself.
The real remaining questions are:

- are we using the correct node when we compute it?
- is our matrix/quaternion convention correct when we compute it?
- is the visual node we compare against actually the collision owner node?

## Practical engineering consequences for ROCK

Binary-backed helpers can now be treated as settled:

- `GetCollisionObjectFromBody`
- `GetOwnerNodeFromCollisionObject`
- `GetOwnerNodeFromBody`

Still unresolved after this audit:

- a direct FO4VR wrapper equivalent to HIGGS `GetRigidBodyTLocalTransform`
- the exact held-object visual-sync contract for root vs collision owner vs selected hit node

## Bottom line

The "missing HIGGS information" is now narrower than before:

- We **do** have the FO4VR binary contract for collidable/body -> owner node.
- We do **not** yet have evidence of a clean FO4VR wrapper equivalent to
  `GetRigidBodyTLocalTransform()`.
- The remaining high-value unknown is the visual-sync contract, not whether
  `sceneObject` recovery exists at all.
