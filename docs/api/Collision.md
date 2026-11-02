# Collision (interface 3, major 2)

Header: `include/ROCK/Collision.h`. Hand and body contacts, player collider
descriptors, hand collision availability, world raycasts, physics scale and
each weapon slot's generated bodies, magazine ejection clearance and object
rest watches.

Interface `Collision`, major 2, minor 0; major 1 is refused. The former
`ROCK_EjectionContacts_Query*` and `ROCK_ObjectRest_QueryV1` exports are
removed; their functions are here. External body registration is removed
with no replacement. Permissions: `Read` (1), `Write` (2).

## Contacts and colliders (game thread, Read)

- `copySemanticHandContacts(owner, hand, maxFramesSinceContact, contacts,
  capacity, count)` copies up to `kMaxSemanticContacts` contacts of one
  hand's colliders that are no older than `maxFramesSinceContact`. `role`,
  `finger` and `segment` name the part of the hand.
- `copyPlayerColliderDescriptors(owner, descriptors, capacity, count)`
  copies up to `kMaxColliderDescriptors` hand and body colliders with
  their transforms and shapes.
- `copyBodyContacts(owner, contacts, capacity, count)` copies up to
  `kMaxBodyContacts` contacts of the player's body colliders.
- `getHandCollisionAvailability(owner, hand, state)` reports whether a
  hand's collision bodies exist and may collide, with body counts, contact
  slots and the dynamic collision layer.

Every record passed in must have `size` and `version` set; capacity above
the limit returns `CapacityFull`.

## World raycast (game thread, Read)

`queryWorldRaycast(owner, request, result)` casts against the world up to
`kMaxRaycastDistanceGame` game units, at most
`kMaxRaycastsPerOwnerPerFrame` casts per owner per frame. Nonzero
generation fields in the request must match ROCK's current ones. The
result carries the frame and generations at the time of the cast.

`request.filter` chooses what the cast can hit. `Selection` (default) is the
item-picking ray: it stops at the first body, actors included. `Surfaces`
sweeps a 1-unit sphere and returns the closest visible solid surface (world
geometry, terrain, trees, clutter, props, debris, glass), passing through
actors, weapons, projectiles, triggers and invisible collision; the result's
`hitLayer` is then the Fallout 4 collision layer of that surface.

## Environment (any thread, Read)

`getEnvironment(owner, environment)` returns the game-to-Havok scale, its
revision, and per weapon slot (`weapons[slot]`) the generation key and
generated body ids, for the publication named by `sample`. Both slots are
published; an empty slot has `bodyCount` 0.

## Ejection clearance (game thread)

`requestEjectionClearance(owner, request)` (Write) stops a dropped
reference, normally an ejected magazine, from colliding with the weapon in
`weaponSlot` for one second after its bodies become available (up to 1.5 s
waiting for them). `IncludeHands` also excludes both physical hands. The
world and other objects still collide; ray queries and grabbing are
unaffected. `reference`, `expectedBase`, `weaponFormId` and
`weaponGenerationKey` must match the live object and the slot's weapon.
A weapon slot is the native equipment index (see `Weapon.md`); a slot at
or above `kMaxWeaponSlots` returns `InvalidArgument`, and slot 1 returns
`TargetUnavailable`.
Repeating the same request observes it without extending it:
`RequestQueued` while waiting, `Ok` once active or finished. At most
`kMaxEjectionClearances` references; another owner gets `OwnerConflict`,
a changed request `GenerationMismatch`. `cancelEjectionClearance(owner,
referenceFormId)` cancels one clearance, or all of the owner's with 0.

## Object rest (game thread)

`watchObjectRest(owner, request, state)` (Write) watches a dropped MISC
reference until it rests. `RequestQueued` admits a watch (at most
`kMaxObjectRestWatches`); poll with the same request: `Ok` reports
`Watching` or `Resting`. `Resting` completes and frees the watch. Two
seconds of supported rest freezes the bodies and saves their pose;
grabbing wakes them. Held items cannot rest. Owner, lifecycle or scene loss
cancels unfinished watches. `cancelObjectRest(owner, referenceFormId)`
cancels one watch, or all with 0.

## Sample (any thread)

`getSample` returns the publication identity. Follow hand touches through the
semantic hand contacts and their contact state.

## Records and enums

| Record | Used by | Fields |
| --- | --- | --- |
| `SemanticHandContact` | `copySemanticHandContacts` | `frameIndex`, `hand`, `role`, `finger`, `segment` (the part of the hand), `handBodyId`, `targetBodyId`, `targetFormId`, `flags` (`SemanticHandContactFlag`), `contactState` (`SemanticContactState`), `framesSinceContact`, `contactSequence`, `contactPointGame`, `contactNormalGame`, `collisionGeneration` |
| `BodyContact` | `copyBodyContacts` | `frameIndex`, `bodyId`, `targetBodyId`, `bodyLayer`, `targetLayer`, `zone` (`BodyZoneKind`), `side` (`BodyZoneSide`), `role`, `descriptorIndex`, `targetKind` (`BodyContactTargetKind`), `inPowerArmor`, `hasContactPointGame`, `contactPointGame` |
| `PlayerColliderDescriptor` | `copyPlayerColliderDescriptors` | `frameIndex`, `kind` (`PlayerColliderKind`: `Hand`, `Body`), `hand`, `bodyId`, `role`, `zone`, `side`, `descriptorIndex`, `flags` (`PlayerColliderFlag`), `lengthGameUnits`, `radiusGameUnits`, `transform`, `collisionGeneration` |
| `HandCollisionAvailability` | `getHandCollisionAvailability` | `frameIndex`, `hand`, `flags` (`HandCollisionAvailabilityFlag`), `collisionSequence`, `collisionGeneration`, `handBodyCount`, `dynamicTwinCount`, the three generations, `otherHandContactSlots`, `weaponContactSlots`, `dynamicCollisionLayer`, `pairSuppressionLeaseCount`, `collisionEnabledBodyCount`, `filterKnownBodyCount` |
| `WorldRaycastRequest` | `queryWorldRaycast` | `startGame`, `directionGame`, `maxDistanceGame`, the three guard generations, `filter` (`WorldRaycastFilter`: `Selection` 0, `Surfaces` 1) |
| `WorldRaycastResult` | `queryWorldRaycast` | `hit`, `flags` (`WorldRaycastResultFlag`: `Hit`, `NormalValid`), `hitFraction`, `hitDistanceGame`, `hitPointGame`, `hitNormalGame`, `frameIndex`, the three generations, `hitLayer` |
| `Environment` | `getEnvironment` | `sample`, `gameToHavokScale`, `havokToGameScale`, `physicsScaleRevision`, `weapons[kMaxWeaponSlots]` |
| `WeaponBodies` | inside `Environment` | `weaponGenerationKey`, `bodyCount`, `bodyIds[8]` |
| `EjectionClearanceRequest` | `requestEjectionClearance` | `reference` (`ReferenceKey`), `expectedBase`, `weaponGenerationKey`, `weaponSlot`, `weaponFormId`, `flags` (`EjectionClearanceFlag::IncludeHands`) |
| `ObjectRestRequest` | `watchObjectRest` | `reference` (`ReferenceKey`), `expectedBase`; the state out-parameter is `ObjectRestState`: `Waiting`, `Watching`, `Resting` |

| Enum | Values |
| --- | --- |
| `SemanticHandContactFlag` | `ContactPointValid`, `ContactNormalValid`, `TargetFormResolved`, `HeldObjectRelation`, `CollisionAvailable`, `TransitionSuppressed` |
| `SemanticContactState` | `Begin` 1, `Continued` 2, `End` 3 |
| `BodyZoneKind` | `Unknown`, `Pelvis`, `SpineLower`, `SpineUpper`, `Chest`, `NeckHead`, then `Shoulder`, `UpperArm`, `ForearmUpper`, `ForearmLower`, `Hand` for left (6-10) and right (11-15) |
| `BodyZoneSide` | `Center`, `Left`, `Right` |
| `BodyContactTargetKind` | `Unknown` 0, `Weapon` 2, `HeldObject` 3, `WorldSurface` 6, `DynamicProp` 7, `Actor` 8, `QueryOnly` 9 |
| `PlayerColliderFlag` | `Valid`, `Enabled`, `PrimaryPalmAnchor`, `TransformValid`, `InPowerArmor`, `LifecycleAllowed`, `FilterKnown`, `CollisionSuppressed` |
| `HandCollisionAvailabilityFlag` | `BodiesReady`, `DynamicTwinsReady`, `PhysicsWritesAllowed`, `CollisionAvailable`, `TransitionSuppressed`, `MenuSuppressed`, `HandDisabled`, `DynamicInteractionsEnabled`, `DynamicOtherHandContact`, `DynamicWeaponContact`, `DynamicWeaponPairSuppressed`, `DynamicPairFilterReady`, `FilterKnown`, `CollisionSuppressed` |

`kNoBody` (`0x7FFFFFFF`) marks a missing body id. Points and normals are in game
space; distances in game units.

## Retired values

`BodyZoneKind` 16-23 (leg and foot zones) and `BodyContactTargetKind` 1
(`Hand`), 4 (`Body`) and 5 (`External`) are retired and never reused.
