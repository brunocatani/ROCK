# Contact Pipeline Refactor Notes

This refactor separates ROCK contact handling into three decisions: what Havok reported, what the two bodies semantically are, and which ROCK route should consume the contact. The goal is to preserve the narrow FO4VR native boundary while making provider publication, support-grip contact, dynamic push, and surface evidence explicit instead of relying on repeated ad hoc body-pair checks.

## Verified Native Boundary

- `0x1403b9e50` is the FO4VR contact signal subscription helper shape used by ROCK. Ghidra shows it allocates a signal slot and stores the member-slot callback payload.
- `0x14175c650` is the FO4VR contact-signal point extraction wrapper used by ROCK. Ghidra shows it resolves the two body indices through the event/manifold, reads hknp body and motion arrays, and dispatches to the manifold-specific extractor.
- `0x14061b5c0` confirms the aggregate contact consumer reads body ids at event `+0x08/+0x0C` and uses the same extraction wrapper before handling contact evidence.

## Current Contract

- Layer `43` is ROCK hand/tool collision.
- Layer `44` is ROCK weapon/tool collision.
- `43 <-> 44` is an explicit `HandWeapon` route.
- Hand/weapon contacts against registered external bodies remain provider contacts.
- Generic `43 <-> 44` contacts do not become provider spam.
- World static/animstatic layers are classified as `WorldSurface` evidence when they appear in contact/query data.
- The production hand physical collision mask still excludes static and animstatic layers, so wall/surface detection must come from evidence probes or verified contact/query paths, not from broad physical blocking.

## HIGGS Reference Principle

HIGGS keeps contact listener capture separate from hand collision registration, haptics, ignored-contact handling, and post-simulation contact state. ROCK follows the same architecture here, but does not copy Skyrim hkp listener structures or layer values. FO4VR hknp contact/event layouts remain behind ROCK's `HavokRuntime` and contact policy boundary.

## 2026-05-02 Native CreateInstance Regression Follow-Up

Runtime testing after the refactor showed `bhkNPCollisionObject::CreateInstance` could return apparently valid hknp body IDs for ROCK-generated hand/weapon collider bodies, but those bodies did not participate in solver contact and grab constraints attached to them did not hold dynamic objects. The deploy at `2026-05-02 23:29:43 -03:00` restored the pre-refactor direct `hknpWorld::CreateBody` ownership path while keeping the Bethesda allocator/TLS fixes, manual `body+0x88` collision-object back-pointer, and minimal `bhkPhysicsSystem` instance.

Open investigation: determine why the native `CreateInstance` path creates non-contacting generated bodies in FO4VR. Do not assume the path is invalid globally; verify the real call contract later with code plus Ghidra. Likely areas to verify are owner NiNode/link timing, `bhkWorld` attachment expectations, internal body flags/quality setup, physics-system instance population, and whether `CreateInstance` expects Bethesda scene ownership that ROCK's generated collider wrappers do not have.

Grab note: the same deploy also changed ROCK's custom grab constraint motor activation so motor atoms are enabled before FO4VR `hknpWorld::CreateConstraint`, because hknp inserts the constraint into the world during that call. This follows the HIGGS timing principle where motors are active before world insertion.

## 2026-05-02 Ghidra Follow-Up: Why vfunc48 Alone Did Not Collide

Ghidra verification after runtime testing confirmed the failing wrapper path stopped one native lifecycle phase too early.

Relevant FO4VR functions:

- `0x141E07AC0` — `bhkNPCollisionObject::vfunction48`: creates or changes the `bhkPhysicsSystem` runtime instance, using the collision object's owner node transform when available, then writes `hknpBody+0x88 = collisionObject`.
- `0x141E07BE0` — `bhkNPCollisionObject::vfunction49`: ensures vfunc48 has created an instance, checks whether the body is already present in the `bhkWorld`, calls `bhkPhysicsSystem::AddToWorld` if not, then writes `hknpBody+0x88 = collisionObject`.
- `0x141E0C580` — `bhkPhysicsSystem::AddToWorld`: calls `bhkWorld::AddPhysicsSystem`.
- `0x141DFAC30` — `bhkWorld::AddPhysicsSystem`: locks the hknp world and calls the hknp physics-system add pass.
- `0x141565770` / `0x1415441F0` — hknp add-bodies path. Ghidra shows timed sections named `LtAddBodies`, `StAddToBroadPhase`, and `StFireCallbacks`; this is the broadphase/solver registration pass missing from the failed ROCK wrapper path.

Root cause of the wrapper regression: ROCK called `0x141E07AC0` directly. That produced a valid `hknpPhysicsSystem` instance and valid body IDs, but it did not run the vfunc49/AddToWorld path that registers the bodies with the world broadphase. This matches the observed symptom exactly: valid body IDs and logs, but no physical contact and no grab constraint authority from the hand body.

Secondary findings:

- vfunc48 reads `collisionObject+0x10` as the owner NiNode and builds the initial system transform from `NiAVObject+0x70..0xAC`. If no owner is linked, it falls back to identity. The failed ROCK wrapper path linked the debug/generated NiNode after vfunc48, so even a future wrapper implementation should link owner before native instance creation.
- The hknp physics-system constructor remaps serialized `hknpPhysicsSystemData` materials, constraints, and body cinfos before body creation. It stores body IDs in the runtime instance at `instance+0x20`, with count at `instance+0x28`.
- `hknpBodyCinfo+0x0C` is the motion slot; the native constructor remaps `0x7FFFFFFF` to motion `0` for serialized systems. Direct ROCK `CreateBody` explicitly allocates a motion for generated keyframed bodies, which remains the known-good path.

## 2026-05-02 Native Wrapper Reimplementation

ROCK now uses the verified native wrapper path again, but through the full lifecycle instead of the failed vfunc48-only path:

- Create generated `hknpPhysicsSystemData`, `bhkPhysicsSystem`, and `bhkNPCollisionObject` with the verified Bethesda allocator/TLS context.
- Link the generated owner `NiNode` before world insertion, because vfunc48 reads `collisionObject+0x10` while constructing the runtime instance.
- Call `bhkNPCollisionObject::vfunction49` at `0x141E07BE0`, which runs the add-to-world/broadphase phase.
- Resolve the body ID from `bhkPhysicsSystem::GetBodyId` rather than fabricating a runtime instance.
- Promote non-static generated bodies to a real motion slot if the serialized-system constructor left them on static motion `0`, then apply the native keyframed body state.
- Remove the wrapper-owned physics system through `bhkWorld::RemovePhysicsSystem` before releasing refs, passing the runtime `hknpPhysicsSystemInstance*` stored at `bhkPhysicsSystem+0x18`, not the `bhkPhysicsSystem*` wrapper object.

Additional Ghidra checks done during implementation:

- `0x141DFAD00` is the matching `bhkWorld` remove-physics-system wrapper. It locks the hknp world path and calls `0x141565A00`, which gathers live body IDs from the system instance and removes them from the world.
- `0x142996CB0` is `NiCollisionObject::SetSceneGraphObject`; it increments the collision object's refcount when storing it in `NiAVObject+0x100`, so ROCK must release that node-held ref when tearing down the generated NiNode.
- `0x141E07780` (`bhkNPCollisionObject` destructor) clears the hknp body collision-object back-pointer and releases its physics system, but it is not the broadphase/world removal pass. This is why explicit physics-system removal is part of `BethesdaPhysicsBody::destroy`.

## 2026-05-03 Motion Wrapper Correction

Follow-up review found a real wrapper mismatch in the generated-body motion promotion:

- `0x141DF95B0` is `bhkWorld::SetMotion(NiAVObject*, preset, recursive, force, activate)`, the recursive Bethesda scene wrapper. It must not be called with `(hknpWorld*, bodyId, motionId)`.
- `0x14153BAE0` is the low-level `hknpWorld::setBodyMotion(hknpBodyId, hknpMotionId, RebuildCachesMode)`. Generated collider promotion now uses this address and passes rebuild mode `1`.
- `0x141DFA2B0` confirms the recursive SetMotion command checks command `+0x38` to decide whether to walk child nodes. ROCK now exposes that bool as `recursive` and passes native args in verified order: `recursive, force, activate`.

## 2026-05-03 Shutdown CTD Fix

Runtime crash logs showed teardown entering `bhkWorld::RemovePhysicsSystem`, then `hknpPhysicsSystem::removeFromWorld`, then crashing in `hknpThreadSafetyCheck::acquireWriteAccess` because the native world pointer was null. The broken boundary was in `BethesdaPhysicsBody::destroy`: ROCK passed the ref-counted `bhkPhysicsSystem*` wrapper to the remove wrapper, but the verified removal path reads the runtime physics-system instance layout directly.

The destroy path now unwraps `bhkPhysicsSystem+0x18` and passes that runtime `hknpPhysicsSystemInstance*` to `0x141DFAD00`. The same helper is also used by per-body filter/activation helpers that need the native world pointer from the physics-system instance. This preserves Bethesda ownership while keeping native hknp calls on the pointer type their layouts expect.
