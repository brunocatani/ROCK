# Grab (interface 4, major 2)

Header: `include/ROCK/Grab.h`. Grab and release commands, hand interaction
state, hand reservations, held placement, default grab poses, guided held
objects, reference and power-armor queries, and grab events.

Interface `Grab`, major 2, minor 0; major 1 and its minors are refused. The
References interface (10) is retired; its queries are here. Permissions:
`Read` (1), `Write` (2).

## Commands (game thread, Write)

`requestForceGrab`, `requestForceRelease`, `requestThrownDrop`,
`requestPowerArmorGrab` and `requestInventoryGrab` queue a command and return
its id (up to `kMaxQueuedCommands` queued). Poll the outcome with
`getInteractionCommandResult` (Read; the latest `kMaxCommandResults` results
are kept); `cancelInteractionCommand` cancels a queued command. Nonzero generation fields in a request must match ROCK's current
ones. `requestInventoryGrab` puts an item of `baseFormId` from the player's
inventory into the hand. Power armor grabs reach at most
`kMaxPowerArmorGrabDistanceGame`.

## Hand state (Read)

- `getHandInteractionState(owner, hand, state)` works on any thread,
  including F4SE tasks: a value copy of the latest publication. Check
  `HandInteractionFlag::Valid`. `Primary` and `Offhand` describe the hand's
  role for the primary weapon. `FiringGrip`, `PartGrip`, `PartCarry` and
  `NativeWeaponCarry` describe the equipped weapon the hand holds, of either
  slot, and `targetFormId` names it; the second weapon's hand reports
  `FiringGrip | NativeWeaponCarry`. The state, target, grip and release
  sequences advance only when that part of the hand changes.
- `getHandTargetDetails(owner, hand, details)` (game thread) adds the
  touched reference, collision layer, anchor, normal, mesh part and power
  armor point.

## Hand reservation (game thread)

`acquireHandReservation`, `renewHandReservation` (Write) reserve one hand
for 1 to `kMaxReservationLeaseFrames` frames; `ReloadReserved` stops that hand
from taking a support grip on the weapon in `weaponSlot`. One owner at a time
(`OwnerConflict`). The hand must be that weapon's offhand (`HandUnavailable`
otherwise); the reservation applies only while that hand is the offhand.
A weapon slot is the native equipment index (see `Weapon.md`); a slot at
or above `kMaxWeaponSlots` returns `InvalidArgument`, and slot 1 returns
`TargetUnavailable`. Expiry, generation
changes and owner loss end it; poll `getHandReservationState` (Read) to see
that (`active` 0). `releaseHandReservation` (Write) ends it.

## Held placement (game thread)

`getHeldPlacementState` (Read) reports what each hand holds, the equipped
weapon and each hand's physical stick click (`rightButton`, `leftButton`). A
hand's click acts only for that hand: with an object in each hand, the right
click places the right one and the left click the left one; an object held in
both hands answers either click. Submit the state's unchanged `frameToken` and
`sample` with `submitHeldPlacementIntent` (Write) in the same phase (Animation
`BeforeRock` preferred). `ReserveRightClick` and `ReserveLeftClick` each cover
that hand's whole press and release while it holds an eligible object; leave
the other hand's click unreserved so it keeps its own actions. `Anchor` (with a
`formId` and the reservation of a hand holding it) asks ROCK to place the
object as a native fixed placement; only the hands holding it let go.
Acceptance is not completion: `lastRequestToken` and `lastResult` report it.
`clearHeldPlacementIntent` withdraws the intent; it never thaws a completed
placement. ROCK hides a press that begins on a reserved click from the game.

`equippedWeapon` reports the primary equipped weapon while it can be placed,
and `equippedWeaponHand` the hand whose click is the bipod's: the firing hand
while both hands hold the weapon, the hand holding it otherwise, and the last
such hand while none does. Never reserve that click for the weapon. Set `TimeEquippedWeaponHold` while you time
a hold on it: a press still latches the bipod at once, but a latched weapon
unlatches when the button comes up instead of when it goes down, so a hold can
place the pose it rests in. `AnchorEquippedWeapon` (with
`TimeEquippedWeaponHold`, `equippedWeapon.formId` and `FixedSurfaceContact`,
without `Anchor` or a reservation of `equippedWeaponHand`'s click) has ROCK let
go of the weapon and move it from the player to a native fixed placement at its
current pose. `lastResult` reports it; once the weapon has left the player the
placement no longer cancels.

## Default grab poses (game thread, Write)

`setDefaultGrabPoses` stores right and left hand poses for a base form (one
owner per form, 64 forms in all); they fill missing saved offsets for loose
non-weapons and throwables on later grabs, ahead of ROCK's own built-in poses.
`clearDefaultGrabPoses` removes
them; clearing an absent entry is `Ok`, another owner's is `OwnerConflict`.

## Held object target (game thread)

`setHeldObjectTarget` (Write) guides the held loose reference toward
`rootWorld` through ROCK's grab constraint for 1 to 2 frames; it stays dynamic
and collidable. Supply the current sample and the hand's `gripSequence`.
`connectedWeaponFormId` and `connectedWeaponGeneration`, when set, suppress
only the collision of the weapon in `connectedWeaponSlot` with the object
(socket insertion); a connected slot 1 returns `TargetUnavailable`. One owner
per hand; two-handed, fixed and loose-firearm holds are refused.
`clearHeldObjectTarget` (Write) ends it; `getHeldObjectPose` (Read) returns
the solved body pose at the reference root.

## Released object drive (game thread, Write)

`driveReleasedObject(owner, drive, progress)` glides a loose reference that
no ROCK hand holds onto `drive.rootWorld` (game units, Ni basis rows, the
reference's own scale). Each call sets this step's velocities of its dynamic
bodies so the root closes the remaining gap with time constant
`approachSeconds` (0.05 to 2); the object stays dynamic and collidable. Call
it every frame while gliding and stop calling to let the object go.
`progress` (optional) reports the root's remaining `distanceGame` and
`angleRadians`. `Busy` while a hand holds the reference; `TargetInvalid` for an actor or
when it has no dynamic body.

## Reference queries (game thread, Read)

`queryReferenceInteraction(owner, query, interaction)` describes a
reference: base form, activation block, open state and furniture use for
`furnitureMarkerIndex`. `queryPowerArmorTarget(owner, query, target)` adds
the power armor frame, wearer and the two armor hand points.

## Events

`setEventCallback(owner, callback, userData)` (game thread, Read) calls
`callback` synchronously on the producing thread; calling ROCK inside it is
forbidden and returns `Busy`. A faulting callback revokes its owner.
`clearEventCallback` removes it. The callback carries one-off grab events
(pull, commit, impact, release); follow lasting hand state through
`getHandInteractionState` and its sequences. `getSample` returns the
publication identity.

## Records

| Record | Used by | Fields |
| --- | --- | --- |
| `ForceGrabRequest` | `requestForceGrab` | `hand`, `flags` (`ForceGrabFlag::UsePreferredGrabPointGame`), `targetFormId`, `targetBodyId`, the three guard generations, `maxDistanceGame`, `preferredGrabPointGame[3]` |
| `ForceReleaseRequest`, `ThrownDropRequest` | `requestForceRelease`, `requestThrownDrop` | `hand`, `flags` (`ImmediateCollisionRestore`, `RequireMatchingTarget`, `UseVelocityHavok`), `targetFormId`, `targetBodyId`, the three guard generations, `linearVelocityHavok[3]`, `angularVelocityRadiansPerSecond[3]` |
| `PowerArmorGrabRequest` | `requestPowerArmorGrab` | `target` (`ReferenceKey`), `hand`, `point` (`PowerArmorPoint`), `maxDistanceGame` |
| `InventoryGrabRequest` | `requestInventoryGrab` | `hand`, `baseFormId`, the three guard generations |
| `InteractionCommandResult` | `getInteractionCommandResult` | `ownerToken`, `commandId`, `kind` (`InteractionCommandKind`), `state` (`InteractionCommandState`: `Queued`, `Succeeded`, `Rejected`, `Cancelled`), `failure` and `failureStage` (`InteractionFailure`), `hand`, `targetFormId`, `targetBodyId`, `frameIndex`, the three generations, `stage` (`CommandStage`: `Accepted`, `Queued`, `Committed`, `Applied`, `Terminal`), `acceptedFrame`, `committedFrame`, `appliedFrame` |
| `ReleasedObjectDrive`, `ReleasedObjectProgress` | `driveReleasedObject` | `reference` (`ReferenceKey`), `expectedBase`, `approachSeconds`, `rootWorld`; `distanceGame`, `angleRadians` |
| `HandInteractionState` | `getHandInteractionState` | `frameIndex`, `hand`, `phase` (`HandInteractionPhase`), `targetKind` (`BodyContactTargetKind`), `flags` (`HandInteractionFlag`), `reservedTargetIdentity`, `targetFormId`, `primaryBodyId`, `heldBodyCount`, `heldBodyIds[8]`, `stateSequence`, `targetSequence`, `gripSequence`, `releaseSequence`, the four generations, `surfaceAnchorGame`, `surfaceGripMode` |
| `HandTargetDetails` | `getHandTargetDetails` | `handState`, `referenceFormId`, `referenceNativeHandle`, `flags` (`TargetDetailFlag`), `collisionLayer`, `anchorGame`, `normalGame`, `powerArmorPoint`, `sourceTriangleIndex`, `collisionNodeName[64]`, `meshPartName[64]` |
| `HandReservationRequest` | acquire, renew | `weaponSlot`, `hand`, `reservation` (`HandReservationKind`: `Normal`, `ReloadReserved`, `ReloadPoseOverride`), `leaseFrames`, the three guard generations |
| `HandReservationState` | `getHandReservationState` | `hand`, `reservation`, `ownerToken`, `expiresAfterFrame`, `active`, `remainingFrames`, the three generations, `weaponSlot` |
| `HeldPlacementState` | `getHeldPlacementState` | `frameToken`, `sample`, `right` and `left` (`HeldPlacementHand`: `formId`, `flags` `LooseObject`/`FixedSurfaceContact`/`Weapon`), `rightButton` and `leftButton` (`HeldPlacementButton`: `available`, `held`, `pressed`, `ageMilliseconds`), `lastRequestToken`, `lastResult` (`PlacementResult`: `None`, `Pending`, `Anchored`, `Rejected`), `lastRequestFormId`, `equippedWeapon` (`HeldPlacementHand`), `equippedWeaponHand` |
| `HeldPlacementIntent` | `submitHeldPlacementIntent` | `frameToken`, `sample`, `formId`, `flags` (`PlacementIntentFlag`: `Anchor`, `TimeEquippedWeaponHold`, `AnchorEquippedWeapon`, `ReserveRightClick`, `ReserveLeftClick`) |
| `DefaultGrabPoses` | `setDefaultGrabPoses` | `baseFormId`, `right`, `left` (`DefaultHandPose`: `flags` `Present`/`FingerValues`/`FingerJoints`, `translateGame[3]`, `rotate[9]`, `fingerValues[5]`, `fingerJointValues[15]`) |
| `HeldObjectTarget` | `setHeldObjectTarget` | `hand`, `referenceFormId`, `leaseFrames`, `connectedWeaponSlot`, `connectedWeaponFormId`, `rootWorld`, `sample`, `gripSequence`, `connectedWeaponGeneration` |
| `HeldObjectPose` | `getHeldObjectPose` | `hand`, `referenceFormId`, `bodyId`, `sample`, `gripSequence`, `rootWorld` |
| `ReferenceQuery` | both reference queries | `referenceFormId`, `referenceNativeHandle`, the three guard generations, `furnitureMarkerIndex` |
| `ReferenceInteraction` | `queryReferenceInteraction` | `flags`, `referenceFormId`, `referenceNativeHandle`, `baseFormId`, `baseFormType`, `activationBlocked`, `openState` (`NativeOpenState`: `NotApplicable`, `Open`, `Opening`, `Closed`, `Closing`), `furnitureInUse`, `furnitureInUseIncludingReservations`, `furnitureMarkerIndex`, `worldGeneration`, `frameIndex` |
| `PowerArmorTarget` | `queryPowerArmorTarget` | `touchedReference`, `frameReference`, `flags`, `actorFormId`, `points[kPowerArmorPointCount]` (`PowerArmorPointPose`: `point`, `valid`, `world`, `frameLocal`) |
| `Event` | event callback | `kind` (`EventKind`), `frameIndex`, the three generations, `hand`, `formId`, `primaryBodyId`, `secondaryBodyId`, `collisionLayer`, `flags`, `sourceKind`, `positionGame[3]`, `velocityGame[3]`, `mass`, `speedGameUnitsPerSecond`, `intensityHint` |

| Enum | Values |
| --- | --- |
| `HandInteractionPhase` | `Idle`, `Touching`, `Selecting`, `Pulling`, `Catching`, `Holding`, `Releasing`, `StashCandidate`, `ConsumeCandidate` |
| `HandInteractionFlag` | `Valid`, `Primary`, `Offhand`, `LooseObject`, `LooseWeapon`, `FiringGrip`, `PartGrip`, `PartCarry`, `InputSuppressed`, `CollisionAvailable`, `TransitionSuppressed`, `HeldBodyListTruncated`, `TouchGrab`, `FixedSurfaceLatch`, `GlobalSurfaceLatch`, `SurfaceAnchorValid`, `MeshSurfaceAnchor`, `MeshFingerPose`, `MeshCollisionFallback`, `DynamicOtherHandContact`, `DynamicWeaponContact`, `DynamicWeaponPairSuppressed`, `NativeWeaponCarry`, `RockGripActive`, `AttachOnly` |
| `InteractionFailure` | `None`, `ProviderNotReady`, `PhysicsWritesBlocked`, `OwnerNotRegistered`, `InvalidRequest`, `StaleWorldGeneration`, `StaleSkeletonGeneration`, `StaleProviderGeneration`, `TargetMissing`, `TargetUnavailable`, `TargetBodyMissing`, `TargetAlreadyOwned`, `HandInvalid`, `HandDisabled`, `HandBusy`, `HandNotHolding`, `HeldObjectMismatch` |
| `TargetDetailFlag` | `Reference`, `Body`, `Anchor`, `Normal`, `MeshPart`, `ActivationBlocked`, `OpenState`, `FurnitureUse`, `PowerArmorClassification`, `PowerArmorFrame`, `PowerArmorActor` |
| `EventKind` | `SelectionLocked` 1, `PullStarted` 2, `PullArrived` 3, `PullCatchAttempt` 4, `PullCatchSucceeded` 5, `GrabCommitted` 6, `HeldImpact` 7, `Released` 8, `TwoHandStarted` 9, `TwoHandStopped` 10, `StashCandidate` 11, `ConsumeCandidate` 12, `Stashed` 13, `Consumed` 14, `LootStarted` 15, `LootCompleted` 16, `SelectionUnlocked` 17 |

`kNoBody` (`0x7FFFFFFF`) marks a missing body id.

`Released` means a reference has left that hand for the world. Besides a held object being let go, it fires when an
equipped weapon is thrown: `formId` is then the dropped reference and `primaryBodyId` its first body, once its bodies
carry the throw's velocity. An equipped weapon that leaves a still hand sends no event. With two hands on one
reference, each hand reports its own `GrabCommitted` and `Released`, so the object is free only when both have let go.

## Retired values

`EventKind` 100-104 (the removed event streams' `HandStateChanged`,
`CommandCompleted`, `GrabNotification`, `ReleaseNotification`,
`ReservationLost`), `BodyContactTargetKind` 5 (`External`) and
`PlacementIntentFlag` 1 (one shared click reservation) are retired and never
reused.
