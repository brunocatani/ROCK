# Weapon (interface 6, major 2)

Header: `include/ROCK/Weapon.h`. Equipped weapons by weapon slot: state and
hand assignment, grip, classification, emitters, composition, authored grip,
scope sight, the handling authority lease and inventory equips.

Interface `Weapon`, major 2, minor 0; major 1 and 1.1 are refused. The
former `ROCK_EquipPickup_QueryV1` export is removed; `isEquipPickup` is here.
Permissions: `Read` (1), `Write` (2). Calls run on the game thread unless
noted.

## Weapon slots

A weapon slot is the native equipment index (`kMaxWeaponSlots` = 2): 0 is
the first equipped weapon, 1 the second. A slot never implies a hand: which
hand fires or supports a weapon is in its `WeaponState`. Per-weapon calls
return `InvalidArgument` for a slot at or above `kMaxWeaponSlots`, and
`TargetUnavailable` for a read ROCK does not publish for that slot yet. Both
slots publish `getWeaponState`, `getGrip`, `getClassification` and
`copyEmitters`; the other per-weapon reads currently serve slot 0 only.

## State (Read)

- `getWeaponState(owner, slot, state)`: the slot's weapon, its lifecycle
  flags and equip transition, `firingHand` (the hand with the firing role;
  `FiringGripHeld` says whether it holds the grip now), `supportHand` (the
  hand on the support grip, or None) and handling flags. Both hands are None
  without a weapon. With `AmmoKnown`, `loadedRounds` and `reserveRounds` are
  the slot's ammo counter: the native HUD rule for that slot's equipment
  index, except that reserve excludes rounds loaded in the other slot's
  weapon when both use the same uncharged ammo. `UserWeaponOffset` marks a
  weapon the player resized or re-gripped with their own FRIK weapon offset
  file (FRIK's compiled defaults never count): ROCK seats and scales it from
  that file and gives it dynamic support grabs only, so animation authored
  for ROCK's grip of the weapon does not fit its hands.
  `SecondaryReloadActive` (slot 1 only) says ROCK's private graph of that
  weapon is in its reload; slot 0's reload is the player's own graph.
- `getGrip(owner, slot, grip)`: the solved weapon world transform, each
  hand's grip in weapon space and the projectile origin and direction. Slot
  1 publishes its placed weapon's world transform and its own projectile
  origin and direction, without hand grips, and returns `TargetUnavailable`
  while no second weapon is placed in a hand. `copyEmitters` transforms are
  in the frame of that slot's `weaponWorld`.
- `getClassification`: size class, keyword flags and their provenance.
- `copyEmitters(owner, slot, emitters, capacity, count)`: flashlights,
  lasers and reticles; `count` is the total and `min(capacity, count)`
  records are copied (capacity 0 just counts).
- `getCompositionState` and `copyCompositionEntries`: installed mods and
  their attach points.
- `getAuthoredGripPose`: the authored hand and finger poses ROCK selected.
- `getScopeSightState`: scope availability, activation source, anchor and
  sight bounds in weapon space.

## Handling authority

`setHandlingAuthority(owner, request)` (Write) lets one owner override ROCK's
weapon handling tuning for 1 to `kMaxHandlingLeaseFrames` frames; renew it to
keep it. Another owner gets `OwnerConflict`. `clearHandlingAuthority` (Write)
ends it; `getHandlingAuthority` (Read) reports the active lease. A lost lease
shows up as a failed renew.

ROCK's firing-grip handoff and reattach zone is a quarter sector on each side
of the grip, swept from straight out to straight down the grip, with
`firingGripReattachRadiusGameUnits` as its radius. `FiringGripCylinderZone`
replaces it with one cylinder per side along the weapon's lateral axis, the
same reach long, for as long as the lease holds. The zone thickness stays
ROCK's setting under either shape.

## Inventory equip

1. `captureInventoryWeapon(owner, baseFormId, stackIndex, item)` (Read)
   immediately before the request. Its keys are comparison witnesses only.
2. `requestInventoryEquip(owner, request, commandId)` (Write) equips into
   the requested hand. The weapon lands in the slot ROCK's equip rule picks:
   the slot whose weapon that hand fires or carries is replaced (the old
   weapon returns to the inventory and `outgoingFormId` names it), else the
   first free slot, else the slot whose weapon no hand fires. A weapon the
   other hand holds stays equipped. A hand busy with anything else (a held
   object, a pending grab) is refused with `HandBusy`. One switch may be
   outstanding; `RequestQueued` is admission, not success.
3. `getInventoryEquipResult(owner, commandId, result)` (Read); the latest 64
   terminal results are kept. `weaponSlot` is the slot the weapon went to.
   Inspect `flags` even when the switch fails.
4. `cancelInventoryEquip(owner, commandId)` (Write) before the native equip;
   afterwards `AlreadyCommitted`.

## Unequip (Write)

`unequipWeapon(owner, slot, expectedFormId)` returns the weapon in `slot`
to the inventory from its own hand, synchronously on the game thread. The
other slot's weapon stays in its hand; when slot 0 empties, a slot-1 weapon
moves to slot 0 in the same hand on the next frame. Read the hand's slot from
Hands `getRoles` (or a slot's `firingHand`) first. `expectedFormId` must name
the slot's weapon (`TargetInvalid` otherwise); an empty slot returns
`TargetUnavailable`, an equip switch in progress `Busy` and a blocking menu
`NotReady`. `InternalError` means the native unequip did not empty the slot.

## Equip pickup (Read, any thread)

`isEquipPickup(owner, referenceFormId, count, matches)` is for a native
pickup interceptor: `matches` is 1 only while ROCK acquires exactly that
reference and count for an equip, on the acquiring thread. It is intent, not
a promise that the equip succeeds.

## Sample (any thread)

`getSample` returns the publication identity. A finished equip transition
advances the weapon state's `terminalSequence`; lost handling authority shows
up as a failed renew.

## Firing aim (Read)

`getFiringAimWorld(owner, hand, outWorld)` returns where a firearm held by
`hand` sits this frame: ROCK's fixed controller-to-weapon aim basis applied to
that hand's filtered controller (left: mirrored, with the left aim trim). It
is the same for every firearm and does not depend on what is equipped, so a
consumer that shows a weapon in the hand before ROCK reveals the real one (a
holster draw) aims it here. Scale is 1. Game thread. `TargetUnavailable`
while the controller frame is unknown.

## Warm inventory weapons (Write)

`setWarmInventoryWeapons(owner, items, count)` replaces this owner's set of
inventory weapons kept ready to equip (`captureInventoryWeapon` records, at
most `kMaxWarmInventoryWeapons` across owners; count 0 clears). ROCK keeps each
firearm's native animation subgraph loaded in the background, so its first
`requestInventoryEquip` after a load does not wait about half a second on the
engine. Other weapon types need no warming. The set is dropped when the owner
unregisters or physics shuts down: send it again when the Sample's world,
skeleton or provider generation changes. `TargetUnavailable` (nothing changed)
when an item no longer matches its stack; `CapacityFull` above the limit.

## Automatic reload (Write)

`setAutomaticReloadBlocked(owner, blocked)`: while an owner holds the block, an
empty slot-1 weapon dry-fires on its trigger instead of starting a reload; its
reload button still reloads. One owner at a time (`OwnerConflict` otherwise); it
ends with `blocked` 0 or when the owner goes away. ROCK starts that reload
itself, so hooks on the game's own automatic-reload call sites never see it;
those call sites, which serve slot 0, are not ROCK's and stay untouched.

## Interaction key (Read)

`WeaponState.weaponGenerationKey` names the weapon's colliders, so it is 0 until
they are built, about a second after an equip. `getInteractionKey(owner, slot,
key)` works from the moment the weapon is drawn, 0 with no weapon, and is what a
mod that needs the weapon rather than its colliders passes as the generation to
WeaponParts sources, drives, targets and grab points and to `setReloadGesture`.
When the colliders arrive it changes to the generation key, once: source keys
stay valid and what you registered follows, and new records name the new key.
On slot 1 it is the generation key.

## Chamber marker (Write)

`setChamberMarker(owner, slot, marker, leaseFrames)` draws a cartridge beside the
slot's ammo counter, in every counter style: `Empty` hollow and dim, `Chambered`
solid. ROCK does not track chambers; the marker says what the owner says. Lease 1
to `kMaxChamberMarkerLeaseFrames` frames, renewed each frame, so the marker goes
away by itself when the owner stops; `None` clears it. One owner per slot
(`OwnerConflict` until the holder clears it or its lease runs out).

## Draw (Write)

`requestInventoryDraw(owner, request, outCommandId)` is `requestInventoryEquip`
for a weapon the consumer shows in the hand at `getFiringAimWorld` until ROCK
reveals it (a holster draw). When ROCK seats the firearm in that hand, the
weapon continues from that aim pose and the hand from its tracked pose, and
both ease into the grip together instead of snapping. Results and polling are
those of `requestInventoryEquip`.

## Records

| Record | Used by | Fields |
| --- | --- | --- |
| `WeaponState` | `getWeaponState` | `sample`, `weaponSlot`, `flags` (`WeaponStateFlag`), `weaponFormId`, `firingHand`, `supportHand`, `handlingFlags` (`HandlingStateFlag`), `weaponGenerationKey`, `transitionSequence`, `terminalSequence`, `transitionSource` and `terminalSource` (`TransitionSource`), `terminalResult` (`TransitionResult`), `transitionWeaponFormId`, `terminalWeaponFormId`, `loadedRounds`, `reserveRounds` |
| `Grip` | `getGrip` | `weaponSlot`, `flags` (`GripFlag`), `weaponGenerationKey`, `weaponFormId`, `weaponWorld`, `rightHandInWeapon`, `leftHandInWeapon`, `muzzleOriginGame`, `muzzleDirectionGame` |
| `Classification` | `getClassification` | `weaponSlot`, `valid`, `keywordFlags` (`KeywordFlag`, 64-bit), `sizeClass` (`SizeClass`: `Melee`, `Pistol`, `Rifle`, `Heavy`), `source` (`ClassificationSource`: `None`, `Keyword`, `WeaponData`, `EquipSlot`), `weaponFormId`, `confidence`, `weaponGenerationKey`, `provenanceFlags` (`ClassificationProvenanceFlag`) |
| `Emitter` | `copyEmitters` (up to `kMaxEmitters`) | `kind` (`EmitterKind`: `Flashlight`, `Laser`, `Reticle`), `source` (`EmitterSource`: `EffectGeometry`, `AddOnNode`), `flags` (`EmitterFlag`), `active`, `visible`, `addOnNodeValue`, `omodFormId`, `attachPointFormId`, `weaponGenerationKey`, `weaponLocalTransform`, `forwardWeaponLocal`, `sourceName[64]` |
| `CompositionState` | `getCompositionState` | `weaponSlot`, `weaponFormId`, `weaponGenerationKey`, `compositionSignature`, `semanticCoverageMask`, `missingCoverageMask`, `publicationSequence`, `entryCount` |
| `CompositionEntry` | `copyCompositionEntries` (up to `kMaxCompositionEntries`) | `omodFormId`, `attachPointFormId`, `stableIndex`, `flags` (`CompositionFlag`: `Active`, `Disabled`, `AttachPointResolved`, `SemanticEvidenceMatched`), `semanticCoverageMask` |
| `AuthoredGripPose` | `getAuthoredGripPose` | `weaponSlot`, `weaponFormId`, `weaponGenerationKey`, `variantKey`, `captureSequence`, `source` (`AuthoredGripSource`: `LiveEquippedGraph`, `NativeIdlePreharvest`, `RuntimeCanonical`), `flags` (`AuthoredGripPoseFlag`), finger masks and `right`/`left` `HandInWeapon` and `FingerLocalTransforms[15]`, the three generations |
| `ScopeSightState` | `getScopeSightState` | `sample`, `weaponSlot`, `flags` (`ScopeSightFlag`), `weaponGenerationKey`, `weaponFormId`, `activationSource` (`ScopeActivationSource`: `NativeGeometry`, `RockGeometry`, `ManualInput`), `nativeScopeOverlayIndex`, `anchorWeaponLocal`, `sightBoundsWeaponLocal`, `sightBodyCount`, `sightBodyId`, `omodFormId`, `attachPointFormId` |
| `HandlingRequest` | `setHandlingAuthority` | `features` (`HandlingFeature`), `leaseFrames`, the grip-zone, reattach, proximity-support and haptic tuning values, the left firing aim trim (`leftFiringAimYawDegrees`, `leftFiringAimPitchDegrees`, `leftFiringAimOffsetGameUnits[3]`), `equipVisualBridgeTimeoutSeconds`, `equipVisualBridgeBlendSeconds`, the three guard generations |
| `HandlingAuthority` | `getHandlingAuthority` | `active`, `features`, `owner`, `expiresAfterFrame` |
| `InventoryWeapon` | capture, equip, draw, warm set, Storage displays | `baseFormId`, `stackIndex`, `count`, `stackKey`, `instanceKey` (comparison witnesses only), `frameIndex`, the three generations |
| `EquipRequest` | `requestInventoryEquip`, `requestInventoryDraw` | `hand`, `item` |
| `EquipResult` | `getInventoryEquipResult` | `commandId`, `transferSequence`, `state` (`EquipState`: `Queued`, `Switching`, `Succeeded`, `Failed`, `Cancelled`), `failure` (`EquipFailure`), `flags` (`EquipResultFlag`: `IncomingEquipAccepted`, `IncomingGripAcquired`, `IncomingPresented`), `hand`, `incomingFormId`, `outgoingFormId`, `weaponSlot`, the three generations |

| Enum | Values |
| --- | --- |
| `WeaponStateFlag` | `Valid`, `IdentityPending`, `DrawPending`, `BridgePresented`, `NativeRenderable`, `HandPoseHandoffComplete`, `RecoveryExhausted`, `TransitionActive`, `WeaponPresent`, `WeaponDrawn`, `PresentationKnown`, `WeaponEquipped`, `BipodInputReserved`, `AmmoKnown`, `UserWeaponOffset`, `SecondaryReloadActive` |
| `HandlingStateFlag` | `FiringGripHeld`, `ManualOwnership`, `PartCarry`, `LeftFiringAvailable` |
| `TransitionSource` | `Unknown`, `ObservedEquip`, `HeldTriggerEquip`, `HeldGripZoneEquip`, `MenuExit`, `WorkbenchExit` |
| `TransitionResult` | `None`, `Completed`, `WeaponUnequipped`, `IdentityLost`, `ExpectedIdentityTimeout`, `NativeAnimationHandoff`, `WeaponNoLongerDrawn`, `RecoveryExhausted`, `ProviderLost`, `Shutdown` |
| `GripFlag` | `Valid`, `WeaponTransformOwned`, `WeaponWorldValid`, `RightHandInWeaponValid`, `LeftHandInWeaponValid`, `MuzzleWorldValid` |
| `HandlingFeature` | `FiringGripOwnership`, `PrimaryDetach`, `AmbidextrousHandoff`, `GripZoneEquip`, `GripZoneHoverHaptics`, `FiringGripProximitySupport`, `FiringGripCylinderZone` |
| `EquipFailure` | `None`, `LifecycleLost`, `OwnerLost`, `HandUnavailable`, `HandBusy`, `InventoryChanged`, `InfrastructureUnavailable`, `NativeEquipFailed` 8, `GripOrPresentationFailed`, `CancelledBeforeCommit`, `EquippedWeaponChanged`, `MenuBlocked` |

## Retired values

`TransitionResult` 10 (shoulder sheath), `GripFlag` bits 1 and 2 (two-hand grip
and left firing hand; the hands are in `WeaponState`), `EquipFailure` 7
(`OutgoingTransferFailed`) and `EquipResultFlag` bits 0 and 1 are retired and
never reused.
