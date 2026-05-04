# Generated Body Native Boundary Fixes — 2026-05-03

This note records the Ghidra-backed reason for the generated body patch. The production code and tests remain the source of truth.

## Verified Functions

- `0x141C17DD0` is `~NiNode`, not a constructor.
- `0x141C17D30` is the usable `NiNode` constructor and takes `(this, initialChildCapacity)`.
- `0x141E08D60` is `bhkNPCollisionObject::GetFilterInfo(this, uint32_t* out)`.
- `0x140F1AB90` takes `(player, handIndex, flag)`; the native game caller passes `flag = 0`.
- `0x14153AFC0` is `hknpWorld::setBodyMaterial(world, bodyId, materialId, rebuildMode)`. Ghidra shows `rebuildMode = 0` runs the immediate cache-update helper before world notification.
- `0x141564DE0` remaps `hknpPhysicsSystemData` material IDs through the local system-data material array before adding bodies to the world. Body-cinfo `+0x12` is a local material index during this phase, not a global world material ID.
- `0x141802670` reads surface velocity data from the material structures passed into contact solving. A bad material ID can feed invalid material data into this path and matches the bottle-release crash stack.
- `0x141E07AC0` / `0x141E0C320` / `0x141564DE0` prove wrapper-created bodies do not consume `bodyCinfo+0x0C` as a direct world motion ID. In `hknpPhysicsSystemData`, that field is a local motion-cinfo index. If it is `0x7FFFFFFF`, the wrapper converts it to static motion `0` before calling `hknpWorld::createBody`.
- `0x1417A2FC0` is the `hknpMotionCinfo` constructor used by the wrapper path. Non-static generated ROCK bodies must append a local `0x70`-byte motion cinfo to system-data array `+0x30` and set body-cinfo `+0x0C` to that local index before `bhkNPCollisionObject::AddToWorld`.
- `0x141561DD0` initializes `bodyCinfo+0x50` to `0`; `0x1415616F0` copies that byte into `body+0x7E` as a shape/body specialization seed. It is not the HIGGS/hkp collidable quality field and must not be set to `2` for keyframed bodies.

## Code Direction

Generated ROCK bodies now use local material index `0` in temporary `hknpPhysicsSystemData`, then assign the desired global material on the live `hknpWorld` body after native AddToWorld returns a body ID.

This avoids crossing the local system-data material namespace with the global world material namespace, which was the likely source of invalid material data during restored hand contact.

Generated ROCK keyframed colliders now also provide a local motion cinfo before native AddToWorld instead of creating a static wrapper body and calling `hknpWorld::setBodyMotion` afterward. That matches the wrapper contract and avoids stale static-body initialization, bad motion ownership, and incorrect shape-specialization seeding.
