# Weapon Collision Visualizer Review - 2026-05-03

## Context

The current mode `2` compound semantic generation was added to keep semantic parts such as `WeaponMagazine`, `WeaponBolt`, `Stock`, and related daughter meshes together for PAPER reload authoring. The user reported that after this change no weapon collision visualizers appear.

This review did not use Ghidra. It is based on source inspection and the deployed ROCK log under `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE`.

## Findings

### 1. Mode 2 creates static compound Havok shapes that the debug overlay cannot draw

`WeaponCollision::createGeneratedWeaponBodiesInBank()` builds a static compound shape whenever a generated source has more than one child local point cloud:

- `WeaponCollision.cpp:1591` chooses convex only when `childLocalPointCloudsGame.size() <= 1`.
- `WeaponCollision.cpp:1628` calls `havok_compound_shape_builder::buildStaticCompoundShape(children)`.

The debug overlay shape generator only handles shape type `0`, `1`, `4` as convex, `2` as sphere, `3` as capsule, and `11` as scaled/wrapped shape. All other shape types return an invalid mesh:

- `DebugBodyOverlay.cpp:612` `generateShape()`.
- `DebugBodyOverlay.cpp:621-650` supported shape switch.
- `DebugBodyOverlay.cpp:763-784` caches an empty GPU shape when generation fails.

Expected effect: generated compound weapon bodies can exist physically, but the overlay draws nothing for them. This directly matches the reported "no visualizers" symptom for weapons whose mode-2 parts are now static compounds.

### 2. The overlay has no generated-geometry fallback path

ROCK stores generated mesh evidence on `WeaponBodyInstance`:

- `WeaponCollision.cpp:1660-1663` stores local bounds and local points.
- `WeaponCollision.cpp:671-676` exposes the stored geometry as provider evidence.

The debug overlay does not consume this stored generated geometry. It receives only body ids from `PhysicsInteraction::publishDebugBodyOverlay()`:

- `PhysicsInteraction.cpp:2697-2703` publishes weapon body ids.
- `DebugBodyOverlay.cpp:680+` extracts a live body shape and then calls `generateShape()`.

Expected effect: even though ROCK already has enough point/bounds data to draw a proxy for generated weapon bodies, visual rendering depends entirely on decoding the live Havok shape. Static compound bodies therefore have no fallback visualization.

### 3. Revolver `WeaponMagazine` parents can still emit false `MagazineBody` evidence

The compound traversal splits `44MagnumCylinder` from a `WeaponMagazine` parent, and excludes bullets/moonclips. However, remaining child tri-shapes under the same parent that classify as `Other` are still appended into the active parent part:

- `WeaponCollision.cpp:1309` starts `traverseSemantic()`.
- `WeaponCollision.cpp:1336-1338` appends a child tri-shape into `activePart` when it does not split from parent.
- `WeaponCollision.cpp:1381-1382` emits every accumulated part cloud.

For the `.44` shape pattern, child pieces like gate/ejector can leave a `WeaponMagazine` parent cloud behind even after cylinder and ammo visuals are handled separately. That can create a false `MagazineBody` for PAPER.

### 4. Compound mag aliasing can misclassify magwell names

`classifyWeaponPartName()` correctly handles `magwell` as `WeaponPartKind::Magwell`, but compound mode then overrides any source starting with `Mag` as a magazine alias unless it is `Magnum` or ammo-ish:

- `WeaponPartClassifier.h:159-160` recognizes `magwell`.
- `WeaponCollisionGroupingPolicy.h:124-137` `isMagazineMeshAlias()`.
- `WeaponCollisionGroupingPolicy.h:140-155` returns magazine if `isMagazineMeshAlias()` is true.

Expected effect: names like `Magwell` or `MagazineWell` can become `MagazineBody` in compound mode instead of socket/support evidence.

### 5. Provider v3 evidence count is still body-bound

The new provider detail API reports the weapon body count, then copies descriptors from active bodies:

- `PhysicsInteraction.cpp:726-728` returns `_weaponCollision.getWeaponBodyCount()`.
- `PhysicsInteraction.cpp:739` gets `_weaponCollision.getProfileEvidenceDescriptors()`.
- `WeaponCollision.cpp:642` builds descriptors from body instances.

Expected effect: evidence is separate from the old fixed snapshot array, but it is still lost if a generated source fails body creation. It is not a complete generated-source evidence channel.

### 6. PAPER bridge defaults to fewer evidence rows than ROCK can create

ROCK can create up to 100 weapon collision bodies:

- `WeaponCollisionLimits.h:14` `MAX_WEAPON_COLLISION_BODIES = 100`.

PAPER's provider bridge defaults to copying only 64 details:

- `PaperProviderBridge.h:156-166` `maxDetails = 64`.

Expected effect: valid ROCK evidence rows 64-99 can be silently absent from PAPER authoring.

## Log Notes

The latest local `ROCK.log` found during review was last written at `2026-05-02 22:54:25`. The entries in that log are old-style generated convex bodies with `children=0`, including fragmented `WeaponMagazine#0`, `WeaponMagazine#1`, etc. No current mode-2 compound entries were present in that log.

## Repair Direction

The direct runtime repair is to make generated compound bodies visible to the debug overlay. Two viable approaches:

1. Add static compound traversal to `DebugBodyOverlay::generateShape()` and merge child render meshes using each child transform. This needs verified hknp static compound layout information before implementation.
2. Add a generated weapon debug proxy path: publish ROCK-owned generated local point/bounds geometry through `BodyOverlayFrame` for weapon bodies and draw that when shape decoding fails. This avoids depending on static compound internals and uses data ROCK already owns.

The proxy path is the safer source-driven repair for visualizers. Static compound traversal can still be added later for general overlay completeness after verification.

## Test Gaps To Close

- A generated compound body must produce visible debug overlay geometry or a proxy.
- `Magwell`/`MagazineWell` must remain magwell/socket evidence in compound mode.
- A revolver `WeaponMagazine` parent must not emit a false `MagazineBody` when useful children are cylinder/gate/ejector/ammo display.
- Provider v3/PAPER bridge should cover the full ROCK weapon body budget and point-copy truncation behavior.
