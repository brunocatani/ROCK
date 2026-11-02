# WeaponParts (interface 7, major 2)

Header: `include/ROCK/WeaponParts.h`. The parts of the equipped weapons by
weapon slot: contacts, classification evidence, part targets and drives,
grip state, part poses, the model's sources, collision suppression, grab
points, and authored state for loose weapon items.

Interface `WeaponParts`, major 2, minor 0; major 1 and every 1.x minor are
refused. Permissions: `Read` (1), `Write` (2). Calls run on the game thread
unless noted.

## Weapon slots

Per-weapon calls take a weapon slot, the native equipment index
(`kMaxWeaponSlots` = 2; see `Weapon.md`). A slot at or above
`kMaxWeaponSlots` returns `InvalidArgument`; a slot ROCK does not publish yet
(currently slot 1) returns `TargetUnavailable`, and clearing it is a no-op.

## Keys

Sources are the weapon model's nodes. ROCK names them with opaque
`sourceKey` values that hold only for one `weaponGenerationKey`; a key of
another generation returns `GenerationMismatch`. Keys come from
`copySources`, contacts, evidence and poses. `copySources` reports each
node's `parentKey` (0 is the weapon root); `querySourcePath` reports the
parent and child index of one key; `querySourcePose` reports its parent-local,
weapon-root-local and world transforms with the publication `Sample`.

## Copy calls

`copyEvidenceDetails`, `copyEvidencePoints`, `copyPartPoses`,
`copyDriveResults` and `copySources` take a capacity (at most the matching
`kMax...` constant, else `CapacityFull`) and report the total in `outCount`;
`min(capacity, total)` records are copied. Capacity 0 just counts.

## Reads (Read)

- `queryContact(owner, slot, query, result)`: the part nearest a world point
  within `radiusGame`.
- `copyEvidenceDetails` and `copyEvidencePoints(owner, slot, bodyId, ...)`:
  each generated collider, how ROCK classified it (`PartKind`, roles, form
  and attach point) and its points.
- `queryTargetResolution`: which owner's target wins for a part, and its grab
  mode.
- `copyPartPoses` and `copyDriveResults`: the current part poses and what
  happened to each drive this frame (`DriveApplication`).
- `getGripState(owner, hand, state)` (any thread): the latest published grip
  of a hand: kind, part, weapon slot, the target owner and group it went
  through, and the hand in part space.

## Part targets and drives (Write)

`setPartTargets(owner, slot, targets, count)` replaces the owner's whole
selection: which parts (by body, source key, source name, part kind or role)
the owner's group may grab, with `FullTwoHandAuthority` or `AttachOnly`. A
name selector resolves to its source now (`Ambiguous` if it does not).
Without `NonExclusive`, an active selection rejects grips on other parts.

`setDriveTargets` poses part sources in weapon-root or source-parent space for
`leaseFrames` (1 to `kMaxDriveLeaseFrames`); renew to keep them.

`clearPartTargets(owner, slot)` and `clearDriveTargets(owner, slot)` remove the
owner's selection and drives on the slot. A lost target or drive shows up as a
failed renew.

## Collision suppression (Write)

`setCollisionSuppression(owner, slot, request)` stops the weapon's generated
bodies whose source is one of `sourceKeys`, or a descendant, from colliding.
Visibility, transforms and body existence are unchanged. One owner per
weapon (`OwnerConflict` otherwise); 1 to `kMaxCollisionSources` sources, lease
1 or 2 frames, `sample` at most one frame old. Clear
(`clearCollisionSuppression(owner, slot)`), expiry, owner, generation or
weapon loss restores the bodies. Renew each frame.

## Grab points (Write)

`setGrabPoints(owner, slot, points, count)` places 1 to `kMaxGrabPoints`
points in part sources of the owner's `AttachOnly` groups; register the
targets first (`TargetUnavailable` otherwise). Ids are unique within a group;
`handMask` is left 1, right 2; radius is in (0, 64] game units. The palm, the
finger or the pinky probe reaches the nearest point within both its radius
and the hand-probe radius. The hand seats on the nearest collider surface of
the group's parts within the radius, in the currently driven pose, keeping
its approach rotation, and follows the moving part. No surface means no
grab. The groups' parts are never grabbed through ordinary surfaces. A
replaced set affects new grabs only. `clearGrabPoints` removes the set.

## Loose item state (Write, any thread)

`setItemState(owner, key, state)` describes a loose weapon item by native
identity (`ItemKey`: weapon form, plus inventory owner and ExtraUniqueID, or
the dropped reference and 0): up to `kMaxItemParts` node paths relative to
the model's Weapon node, each with a parent-local transform and a hidden
flag. `moveItemState` follows the item between identities; moving to owner 0
removes it. `clearItemStates` removes the owner's states. States persist
until removed or the owner is lost; serialize your own and republish after a
load. Calls may come from native serialization callbacks.

## Sample (any thread)

`getSample` returns the publication identity. Withdrawn part targets or
drives show up as a failed renew.

## Records

| Record | Used by | Fields |
| --- | --- | --- |
| `ContactQuery`, `ContactResult` | `queryContact` | query: `pointGame`, `radiusGame`, `flags`; result: `weaponSlot`, `valid`, `bodyId`, `partKind`, the four roles (`reloadRole`, `supportRole`, `socketRole`, `actionRole`), `interactionKey`, `sourceKey`, `weaponGenerationKey`, `probeDistanceGame` |
| `EvidenceDetail` | `copyEvidenceDetails` (up to `kMaxEvidenceDetails`) | `bodyId`, `partKind`, the four roles, `fallbackGripPose`, `pointCount`, `interactionKey`, `sourceKey`, `weaponGenerationKey`, `localBoundsGame`, `omodFormId`, `attachPointFormId`, `classificationSource` (`ClassificationSource`), `sourceName[64]`; `copyEvidencePoints` copies up to `kMaxEvidencePoints` `Point3` |
| `Target` | `setPartTargets` (up to `kMaxTargets`) | `flags` (`TargetFlag`), `grabMode` (`GrabMode`: `None`, `FullTwoHandAuthority`, `AttachOnly`), `weaponGenerationKey`, `sourceKey`, `bodyId`, `partKind`, the four roles, `groupId`, `priority`, `sourceName[64]` |
| `DriveTarget` | `setDriveTargets` (up to `kMaxDrives`) | `flags`, `driveSpace` (`DriveSpace`: `WeaponRootLocal`, `SourceParentLocal`), `weaponGenerationKey`, `sourceKey`, `bodyId`, `groupId`, `priority`, `leaseFrames`, `targetTransform`, `sourceName[64]` |
| `ResolutionQuery`, `ResolutionResult` | `queryTargetResolution` | query: `weaponGenerationKey`, `bodyId`, `partKind`, the four roles, `sourceKey`, `sourceName[64]`; result: `winningOwner`, `weaponGenerationKey`, `frameIndex`, `whitelistActive`, `matched`, `grabMode`, `groupId`, `priority` |
| `DriveResult` | `copyDriveResults` | `frameIndex`, `owner`, `weaponGenerationKey`, `bodyId`, `groupId`, `priority`, `result` (`DriveApplication`), `appliedSourceParentLocal`, `sourceName[64]` |
| `GripState` | `getGripState` | `hand`, `gripKind` (`GripKind`), `active`, `attachOnly`, `gripSequence`, `weaponGenerationKey`, `weaponSlot`, `bodyId`, `partKind`, the four roles, `omodFormId`, `sourceKey`, `targetOwner`, `targetGroupId`, `targetGrabMode`, `attachPointFormId`, `classificationSource`, `authoredSupportGrip`, `hasHandPartLocal`, `handPartLocalSpace` (`GripLocalSpace`: `WeaponRootLocal`, `PartSourceLocal`), `handPartLocal`, `sourceName[64]` |
| `PartPose` | `copyPartPoses` (up to `kMaxPoses`) | `frameIndex`, `weaponGenerationKey`, `sourceKey`, `bodyId`, `partKind`, `omodFormId`, `attachPointFormId`, `flags` (`PoseFlag`: `Valid`, `SourceParentLocalValid`, `WeaponRootLocalValid`), `actionRole`, `sourceParentLocal`, `weaponRootLocal`, `sourceName[64]` |
| `Source` | `copySources` (up to `kMaxSources`) | `bodyId`, `weaponGenerationKey`, `sourceKey`, `parentKey`, `sourceParentLocal`, `name[64]` |
| `SourcePose` | `querySourcePose` | `sample`, `valid`, `weaponSlot`, `weaponGenerationKey`, `sourceKey`, `sourceParentLocal`, `weaponRootLocal`, `world` |
| `CollisionSuppression` | `setCollisionSuppression` | `leaseFrames`, `sourceCount`, `sample`, `weaponGenerationKey`, `sourceKeys[kMaxCollisionSources]` |
| `GrabPoint` | `setGrabPoints` | `pointId`, `groupId`, `weaponGenerationKey`, `sourceKey`, `handMask`, `radiusGame`, `pointInSource` |
| `ItemKey`, `ItemState` | item state calls | key: `weapon`, `owner`, `unique`; state: `count`, `parts[kMaxItemParts]` (`ItemPart`: `path[256]`, `local`, `hidden`) |

| Enum | Values |
| --- | --- |
| `PartKind` | `Receiver`, `Barrel`, `Handguard`, `Foregrip`, `Pump`, `Stock`, `Grip`, `Magazine`, `Magwell`, `Bolt`, `Slide`, `ChargingHandle`, `BreakAction`, `Cylinder`, `Chamber`, `Shell`, `Round`, `LaserCell`, `Lever`, `Sight`, `Accessory`, `CosmeticAmmo`, `Other`, `LaserSight`, `Flashlight`, `LaserFlashlightCombo`, `Scope`, `MuzzleDevice`, `Bipod` (0-28) |
| `ActionRole` | `None`, `Bolt`, `Slide`, `ChargingHandle`, `Pump`, `BreakAction`, `Cylinder`, `Lever`, `Latch` |
| `ClassificationSource` | `NameToken`, `SlotAnchor`, `RigAnchor`, `AttachmentEvidence` |
| `TargetFlag` | `MatchBodyId`, `MatchSourceKey`, `MatchSourceName`, `MatchPartKind`, `MatchReloadRole`, `MatchSupportRole`, `MatchSocketRole`, `MatchActionRole`, `NonExclusive` |
| `DriveApplication` | `Unknown`, `Applied`, `Unresolved`, `StaleGeneration`, `MissingParent`, `LostPriority`, `InvalidTransform`, `CapacityRejected`, `Restored` |
| `GripKind` | `None`, `FiringGrip`, `SupportFullAuthority`, `SupportVisualOnly`, `PartCarry`, `AttachOnly` |

`kNoBody` (`0x7FFFFFFF`) marks a missing body id.
