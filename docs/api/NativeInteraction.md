# NativeInteraction (interface 14, major 2)

Header: `include/ROCK/NativeInteraction.h`. A native extension of ROCK's
physics step: callbacks in ROCK's physics frame, Havok substep and world
retirement; driving ROCK's weapon bodies by weapon slot; and extension
handling of grab selection, extraction, grenade proximity and impact.

Interface `NativeInteraction`, major 2, minor 0; major 1 and 1.1 are refused.
Permissions: `Read` (1), `Write` (2); registration needs both.

Extensions create, read and retire their own bodies and constraints and read
Havok directly, using the `world` and `bhkWorld` of the current `Frame`.
Native addresses are borrowed only inside the callback that supplied them.

## Registration

`registerExtension(owner, callbacks)` on the game thread, outside callbacks
(a Core callback may register before the first frame). Up to
`kMaxExtensions`. `unregisterExtension(owner)` runs `retire` and removes the
extension. A faulting callback removes its extension.

## Callbacks

Every callback is optional and receives `context`.

| Callback | When |
| --- | --- |
| `update(frame)` | once per ROCK physics frame |
| `preCollide(frame)` | each Havok substep, before collision |
| `retire(frame)` | the world goes away or the extension is removed; remove everything held in that world |
| `weaponUpdate(weaponFrame, command)` | each frame per driven weapon slot |
| `weaponPreCollide(frame, slot)` | each substep; nonzero keeps driving |
| `weaponContact(contact)` | a contact of the slot's weapon body (any physics thread; publish through a bounded thread-safe channel) |
| `handContact(contact)` | a contact of a dynamic hand body (right 0, left 1; palm, finger or forearm) with a body on a layer enabled by `setDynamicObstacleLayers`, whether or not the hand holds anything; every registered extension receives it (any physics thread, same rules as `weaponContact`) |
| `weaponRetire(frame, slot)` | before ROCK retires the slot's weapon body; remove your constraints on it |
| `classifySelection`, `validateSelection`, `extractSelection` | grab selection and item extraction |
| `proximityTarget`, `impactEffect` | grenade proximity and impact |

## Weapon driver

One extension at a time drives weapons by registering `weaponUpdate` (then
`weaponPreCollide` and `weaponRetire` are required; another driver gets
`OwnerConflict`). In `weaponUpdate`, `WeaponFrame` gives the slot, its
dynamic body, the generation, the pose ROCK is about to drive to and the
primary grip. Returning `Controlled` in `WeaponCommand` replaces the drive
target and the visual weapon pose for that frame.

A `Controlled` command may also set `lodged` with `lodgeTarget`, the body
that holds the weapon (a blade stuck in a limb). When the last hand lets go
while that mark is from the current or previous frame, and Weapon Equip Mode
drops the weapon in place (Auto Drop), ROCK places the loose weapon without
the hand's throw and keeps it lodged: it rides that body every physics step
and takes the body's collision group, so it never collides with that actor
while hands, grab selection and the world still meet it. If the body goes
away, the lodge follows the same bone's new body (an actor's limb changes
bodies when his ragdoll enters or leaves the world). It ends, and the weapon
is an ordinary loose weapon again, when a hand takes or pulls it, when its
reference or bodies go away, when it is pushed 8 game units off its seat for
eight physics steps, or when the bone has no body. Nothing is constrained to
the target body. A save keeps only the loose weapon, so after a load it is
an ordinary loose weapon.

Weapon calls take `weaponSlot` below `kMaxWeaponSlots`. Slot 0 is the primary
weapon and slot 1 the second equipped (akimbo) weapon; each slot has its own
weapon body, callbacks (`weaponSlot` in the frame, contact and retire),
drive command and contact exclusion. For slot 1, `primaryGripLocal` is the
holding hand on that weapon. A slot without a weapon body returns
`TargetUnavailable` (clearing it is a no-op):

- `activateWeapon(owner, slot)`: wake the weapon body (Update, WeaponUpdate
  or PreCollide).
- `setWeaponContactExclusion(owner, slot, identity)` /
  `clearWeaponContactExclusion` / `hasWeaponContactExclusion`: one target body
  the weapon body stops colliding with (Update or WeaponUpdate). The
  `BodyIdentity` must match the live body field for field.
- `copyWeaponSources(owner, slot, sources, capacity, count)`: the generated
  weapon bodies and their source names; `count` is the total and
  `min(capacity, count)` are copied (WeaponUpdate).
- `getSourceGeometry(owner, slot, body, direction, geometry)`: a body's
  source in weapon space, and with a direction its bounds and support point
  (WeaponUpdate).

## Obstacles

`setDynamicObstacleLayers(owner, mask)` (Update or WeaponUpdate): which actor
layers ROCK's dynamic hand and weapon bodies collide with, as a layer bit mask:
8 (Biped), 32 (DeadBip), 33 (BipedNoCC) and `kPassiveObstacleLayer` (55). Any
other bit returns `InvalidArgument`. The masks of all extensions combine.

Bodies on 8 and 32 are dynamic (active ragdolls and corpses). Once one of those
layers is admitted, ROCK's keyframed hand and weapon bodies stop touching it, so
those limbs are pushed only by the dynamic bodies, with real mass, and never
shoved with a keyframed body's infinite mass. A dynamic hand's contact with a
corpse also counts as a touch for touch grabs, as the keyframed hand's did.

Extension bodies on `kPassiveObstacleLayer` should use collision group
`kPassiveObstacleGroup` (`0x0010`): ROCK's dynamic proxies never use it, and the
engine filter drops pairs within one group, so those bodies do not collide with
each other.

`setStaticContactPass(owner, bodies, count)` (Update): these bodies stop
contacting static bodies (walls, fixed furniture, terrain) and keep every other
contact, moving and keyframed bodies, hands and weapons included. The call
replaces the owner's list; zero bodies clears it and Retire drops it. ROCK
rebuilds the contact caches of every body that joins or leaves the list, so a
body that leaves collides again at once. The lists of all extensions combine, up
to `kMaxStaticContactPassBodies` bodies (`CapacityFull` beyond that). An owner
keeps its list current: a stale id applies to whichever body takes that id.

## Selection and gameplay

ROCK asks extensions in registration order; the first that handles a
selection (`SelectionKind` other than `Unhandled`), proximity or impact query
answers it. A selection's `ticket` holds extension-owned values only;
re-resolve on action. `extractSelection` returns the extracted reference.

`SelectionKind`:

| Value | Meaning |
| --- | --- |
| `Unhandled` 0 | Not this extension's body; ROCK asks the next one. |
| `Blocked` 1 | The body may not be grabbed. |
| `DirectConnectedBody` 2 | ROCK holds the body as part of its connected body set (a ragdoll limb, for example), driving with the set's mass and moving the set onto the hand. |
| `DirectDetachedBody` 3 | ROCK holds the body on its own, as a detached body. |
| `ExtractableItem` 4 | ROCK calls `extractSelection` and holds the extracted reference. |
| `DirectLiveLimb` 5 | A limb of a living actor whose ragdoll the extension drives toward its animation. ROCK holds it like `DirectConnectedBody` but drives with the limb's own mass and never moves the actor's body set onto the hand. |

ROCK accepts kinds up to `kLastSelectionKind` (`DirectLiveLimb`).
`SelectionQuery::stage` (`SelectionStage`) is `BeforeCoreTypes` when ROCK
asks before its own object types and `AfterCoreTypes` after them.

## Records

| Record | Used in | Fields |
| --- | --- | --- |
| `Frame` | every callback | `phase` (`Phase`), `sample`, `world`, `bhkWorld`, `hmdPosition`, `deltaSeconds`, `gameToHavok`, `havokToGame`, `substepSeconds`, `worldReady`, `menuBlocked`, `hmdValid`, `timingValid`, `timingFallback` |
| `BodyIdentity` | contact exclusion | `body`, `motion`, `collisionObject`, `ownerNode`, `shape`, `group` (filter >> 16); compared field for field with the live body |
| `WeaponFrame` | `weaponUpdate` | `weaponSlot`, `body`, `frame`, `generation`, `form`, `supportOwnsPose`, `requestedWeapon`, `centerWeaponLocal`, `primaryGripLocal`, `primaryGripValid`, `recoveryDistanceGame` |
| `WeaponCommand` | `weaponUpdate` output | `result` (`DriveResult`: `Unchanged`, `Controlled`), `requestedWeapon`, `presentation`, `lodged`, `lodgeTarget` (`BodyIdentity`) |
| `Contact` | `weaponContact` | `weaponSlot`, `weapon`, `frame`, `target`, `targetLayer`, `pointWorld`, `normalValid`, `normalWorld` (unit, target surface toward the weapon), `pointCount`, `separationValid`, `separationGame` |
| `HandContact` | `handContact` | `hand` (0 right, 1 left), `part` (`HandPart`: `Palm`, `Finger`, `Forearm`), `frame`, `handBody`, `target`, `targetLayer`, `pointWorld`, `normalValid`, `normalWorld` (toward the hand), `pointCount`, `separationValid`, `separationGame` |

Havok creates a contact manifold before the shapes meet, ahead of fast
motion, so a published contact does not mean the bodies touch.
`separationGame` is the signed distance of the manifold's closest point in
game units: positive while the shapes are still apart, negative while they
overlap. Judge touch by it, never by the contact's existence alone.
| `WeaponSource` | `copyWeaponSources` | `body`, `generation`, `name[64]`; up to `kMaxWeaponSources` (128) |
| `SourceGeometry` | `getSourceGeometry` | `valid`, `triangleCount`, `generation`, `sourceInWeapon`, `boundsLocal`, `supportPointLocal` |
| `SelectionQuery` | selection callbacks | `body`, `reference`, `world`, `hitNode`, `hitPoint`, `hasHitPoint`, `farSelection`, `stage` |
| `SelectionReply` | selection callbacks | `kind`, `owner`, `visualNode`, `ticket[8]`, `reason[64]` |
| `ExtractedReference` | `extractSelection` output | `nativeHandle`, `referenceForm`, `itemForm` |
| `ProximityQuery`, `ProximityResult` | `proximityTarget` | query: `radiusGame`, `reference`; result: `handled`, `targetForm`, `scanned`, `truncated` |
| `ImpactQuery`, `ImpactResult` | `impactEffect` | query: `authoredDamage`, `source`, `target`; result: `handled`, `targetForm` |

`Phase`: `Update` 1, `PreCollide` 2, `Retire` 4, `WeaponUpdate` 5,
`WeaponContact` 6, `WeaponRetire` 7, `Selection` 8, `Extraction` 9,
`Gameplay` 10, `HandContact` 11. `kInvalidBody` (`0x7FFFFFFF`) marks a missing
body.

`packTransform` and `unpackTransform` convert between `Transform` and a native
`NiTransform` value by value. Native `NiMatrix3` rows carry a fourth padding
lane, so never copy nine floats directly.

## Retired values

`Phase` 3 (`Render`) and `DriveResult` 2 (`RetireWeapon`) are retired and
never reused.
