# FO4VR Owner-Sync And Writeback Audit — 2026-04-22

## Why this note exists

The previous audit proved the body -> collisionObject -> owner-node contract.
This pass answers the next question:

- does FO4VR perform owner-node sync inside the `NiCollisionObject` / `bhkNPCollisionObject`
  wrapper family, or does the visible-node writeback live elsewhere?

This matters because ROCK's remaining held-object bug is no longer "can we recover the owner node?"
It is "which node does FO4VR actually keep visually aligned with the body during runtime?"

## Scope

Audited functions:

- `0x142996CB0` — `NiCollisionObject::vfunction43`
- `0x141E07AC0` — `bhkNPCollisionObject::vfunction48`
- `0x141E07E30` — live body resolver from collision object
- `0x141E08A70` — collision-object transform wrapper
- `0x141E082A0` — collision-object velocity wrapper
- `0x141DF55F0` — deferred body transform setter
- `0x141DF56F0` — deferred body velocity setter
- `0x141DFC2E0` — higher-level external caller that drives a body from an owner node transform

## Findings

### 1. `vtable + 0x150` is the normal owner-link implementation, not a hidden visual-sync method

The constructor/link path in `NiCollisionObject` calls the virtual at `vtable + 0x150` when the
linked collision object disagrees about its owner.

That target is:

- `0x142996CB0` — `NiCollisionObject::vfunction43`

Behavior:

- writes `collisionObject + 0x8 = owner NiAVObject`
- writes `owner NiAVObject + 0x100 = collisionObject`
- handles reference ownership / replacement

There is also a trivial thunk at:

- `0x141C601F0` — `vfunction43`

which resolves back to the same base implementation.

Current conclusion:

- the `+0x150` virtual is part of the ownership-link contract
- this is **not** evidence of a dedicated body -> visible-node writeback routine

### 2. `bhkNPCollisionObject::vfunction48` seeds body state from the owner node once

`0x141E07AC0`:

- checks or instantiates the wrapped `bhkPhysicsSystem` for the requested world
- if `this + 0x8` is non-null, reads the owner node world transform from `owner + 0x70 .. 0xAC`
- passes that world transform into the instantiation helper
- resolves the live body pointer
- writes `body + 0x88 = this`

This proves:

- owner node world -> body is a real initialization path
- body back-pointer -> collision object is real

It does **not** prove a per-frame visual writeback path in the same family.

### 3. `SetTransform` and `SetVelocity` wrappers do not touch the owner node

`0x141E08A70`:

- resolves wrapped body ID and world
- forwards to `0x141DF55F0`

`0x141E082A0`:

- resolves wrapped body ID and world
- forwards to `0x141DF56F0`

`0x141DF55F0`:

- immediately calls `0x1415395E0` or queues a deferred transform job

`0x141DF56F0`:

- immediately calls `0x141539F30` or queues a deferred velocity job

Important negative result:

- none of these audited wrappers read or write `collisionObject + 0x8`
- none of them read or write `ownerNiAVObject + 0x70`
- none of them appear to push the live body transform back into the owner node

So the collision-object wrapper layer writes the body, not the visible node.

### 4. There are higher-level callers outside the wrapper family that can drive body state from a node

`0x141DFC2E0` is a strong example.

Observed behavior:

- reads a node world transform from `param_2[2] + 0x70 .. 0xAC`
- resolves the wrapped body ID
- calls the body transform setter path (`0x1415395E0` or deferred equivalent)
- may create / attach a `bhkNPCollisionProxyObject`

What this means:

- FO4VR has external systems above the raw collision-object wrappers that use node world transforms
  to drive bodies
- ownership and sync behavior can exist outside `bhkNPCollisionObject` itself

This is the strongest indication in this pass that the visible sync path is likely a higher-level
scene/physics service, not a method on the wrapper family we have been auditing.

### 5. No per-frame body -> owner-node writeback routine was isolated in the audited wrapper family

Within the functions audited here, the pattern is consistent:

- owner node world is used to initialize or drive body state
- body back-pointers are maintained
- wrapper setters affect the body only

What was **not** found:

- a clean `bhkNPCollisionObject` method that reads the live body and writes back to
  `ownerNiAVObject->world`
- a clean FO4VR analog of "update owner node from held body" inside the wrapper family

## Current interpretation

The binary now separates two things that were previously being conflated:

1. **Ownership contract**
   - settled
   - body -> collision object -> owner NiAVObject is real and typed enough to use

2. **Visual sync contract**
   - still unresolved
   - not obviously implemented in the collision-object wrapper methods we audited

That means the remaining bug is less likely to be "we still need one more hidden wrapper" and more
likely to be one of:

- we are comparing against the wrong scene node
- the real visual owner is not the same as the collision owner
- the higher-level scene sync system expects a different node/root contract than the one ROCK is using

## Engineering consequence for ROCK

What can now be treated as binary-backed:

- `GetCollisionObjectFromBody(body) -> *(body + 0x88)`
- `GetOwnerNodeFromCollisionObject(collObj) -> *(collObj + 0x8)`
- `GetOwnerNodeFromBody(body) -> *( *(body + 0x88) + 0x8 )`

What should be treated as the next unknown to measure:

- `collision owner node` vs `selected hit/collidable node` vs `refr->Get3D()` root

That is the correct focus for the next runtime diagnostic/code phase.

## Bottom line

This pass did **not** reveal a missing HIGGS-style wrapper that would solve the held visual bug by
itself.

It did reveal something more useful:

- the owner-link contract is real
- the body-writing wrappers do not perform visual writeback
- the remaining visual-sync logic likely lives outside the wrapper family

So the next technical question is no longer "what address are we missing?"
It is "which FO4VR scene node is the one the higher-level sync path actually treats as authoritative?"
