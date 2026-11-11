# Migrating a mod from ROCK 0.92 to ROCK 1.0

ROCK 0.92 published provider API **1** through the RPS SDK. ROCK 1.0 publishes
provider API **2** from this repository's `include/ROCK/` and serves nothing
else. A mod built against the old headers asks for major 1, gets
`UnsupportedMajor`, and ROCK stops the game at the main menu with an error box
naming that mod. There is no compatibility layer: the mod has to be rebuilt
against provider API 2 with the changes on this page.

You need this page if your plugin calls `ROCKAPI_QueryInterfaceV1`, directly or
through `rock::api::Client`. A mod that only checks whether `ROCK.dll` is loaded
needs nothing.

Work through the sections in order. The [Overview](Overview.md) and each
interface's page hold the full 2.0 contract; this page lists what changed from
0.92 and where each old call went.

## 1. Build

- Remove the RPS SDK from your build: `find_package(RPS_SDK)`, the
  `RPS::ROCK*` targets, include paths ending in `SDK/ROCK/include`, and any copy
  of ROCK headers in your repository.
- Add this repository's `include/` directory through a CMake cache variable,
  for example `ROCK_INCLUDE_DIR`, and include `<ROCK/...>`. Nothing is linked.
  Take the headers from the ROCK release you ship against.
- Delete includes of the minor-extension headers. Their calls are in the main
  tables now: `GrabV1_1.h`, `InputV1_1.h` and `WeaponV1_1.h` (and from
  pre-release SDK builds `GrabV1_2.h`, `GrabV1_3.h`, `InputV1_2.h` to
  `InputV1_5.h`, `TouchV1_1.h`, `WeaponPartsV1_2.h` to `WeaponPartsV1_7.h`,
  `NativeInteractionV1_1.h`).
- Delete includes of headers that no longer exist: `References.h`,
  `Diagnostics.h`, and from pre-release SDK builds `EjectionContacts.h`,
  `EjectionContactsV2.h`, `ObjectRest.h`, `EquipPickup.h`, `StorageDisplay.h`,
  `StorageFeedback.h`, `StorageInteraction.h`, `StorageInteractionV1_1.h`,
  `InventoryDisplay.h`, `InventoryDisplayV1_1.h`, `StashConversion.h`. The
  sections below say where their calls went.
- `ROCKApi.h`, `ROCKProviderApi.h` and `ROCKConfigurationApi.h` (the
  `ROCKAPI_GetApi`, `ROCKAPI_GetProviderApi`, `ROCKAPI_GetDescriptorV1` and
  `GetROCKConfigurationApi` exports) were not served by 0.92 either. Code still
  using them has to move to the interfaces below.
- The headers need C++20 and a 64-bit build.

## 2. Discovery and connection

- The export and the handshake keep their names: `ROCKAPI_QueryInterfaceV1`,
  `QueryInterfaceV1`, `InterfaceDescriptorV1`, `kQueryExportName`.
- Request major 2, minor 0, with your own `sizeof(Api)`. `rock::api::Client`
  does this: `connect(query, name)` registers your owner through Core 2, and
  `acquire(permissions, table)` binds each interface. If you wrote discovery by
  hand with literal `1`s, use each table's `interfaceId`, `majorVersion` and
  `minorVersion` instead.
- Every interface is major 2, minor 0, with one table. There are no minors to
  negotiate.
- Never probe with trial queries, for example "ask for 2, then 1". Every
  refused query stops the game.
- When `ROCK.dll` is loaded but refuses your request, do the same as ROCK: show
  an error box naming your mod and stop at the main menu, so the player knows
  which mod needs an update. RPS Framework's `RPS/Runtime/RequiredUpdate.h`
  provides the box. When `ROCK.dll` is not loaded, turn your ROCK features off
  and keep running.
- Permission bits are unchanged, except that Animation gains `Callbacks` (4)
  for the animation phase callbacks, which moved there from Core.

## 3. Names

- `V1` is gone from every function, record and enum: `ApiV1` is `Api`,
  `HandFrameV1` is `HandFrame`, `getHandFrameV1` is `getHandFrame`. Values of
  surviving enums are unchanged.
- `ForOwner` is gone too: `registerFrameCallbackForOwnerV1` is
  `registerFrameCallback`.
- `Abi.h`: `SampleV1` is `Sample`, `PowerArmorPointV1` is `PowerArmorPoint`.
  `StreamV1` is removed with the event streams.
- Only the discovery names above keep `V1`.

## 4. Records

- Every record that starts with `size` and `version` must have
  `size == sizeof(record)` and `version == 1`, for input and output alike.
  Value-initialize records (`Record r{};`) and the defaults are right. A wrong
  size or version returns `InvalidSize` or `UnsupportedVersion` and reads or
  writes nothing. Some records gained `version` in 2.0; check each record you
  fill.
- Every `reserved` field is gone, so most layouts changed. Delete code that
  reads or writes `reserved*`, and never reuse a binary built against the old
  headers.
- A record's loose `frameIndex`, generation fields and `stateSequence` are now
  one `Sample sample`. `sample.publicationSequence` is the former
  `stateSequence`, and `sample.collisionGeneration` is new. This applies to
  `HandFrame`, Core's `Snapshot`, `PlayerControllerState`,
  `LogicalInputActionState`, `HandInputSuppressionState` (whose lease clock is
  now `leaseFrameIndex`), Weapon's `ScopeSightState` and others.
- Count-then-copy pairs are one copy call: it reports the total in `outCount`
  and copies `min(capacity, total)` records. Capacity 0 only counts.

## 5. Interface by interface

### Core (1)

| 0.92 | 1.0 |
| --- | --- |
| `getFrameSnapshot` | `getSnapshot`; `Snapshot` carries `sample` |
| `registerConsumerV1`, `unregisterConsumerV1` | `registerConsumer`, `unregisterConsumer`; `Registration` and `Owner` lose `reserved` |
| `registerFrameCallbackForOwnerV1`, `unregisterFrameCallbackForOwnerV1` | `registerFrameCallback`, `unregisterFrameCallback` |
| `registerAnimationPhaseCallbackV1`, `unregisterAnimationPhaseCallbackV1`, `AnimationPhaseV1` and its records | Moved to Animation (below) |
| `copyEvents` | Removed (section 7) |

### Hands (2)

| 0.92 | 1.0 |
| --- | --- |
| `getHandFrameV1` | `getHandFrame`; `HandFrame` carries `sample`; flag bits 2, 3 and 4 are retired |
| `getPresentedHandFrameV1` | `getPresentedHandFrame`; game thread only, `WrongThread` elsewhere |
| `getPresentedHandPoseV1` | `getPresentedHandPose`; fields reordered, `fingerLocalTransformMask` is `uint32` |
| `getHeadPose` | Unchanged name; `HeadPose.valid` is `flags` (`HeadPoseFlag::TransformValid` 1, `ForwardValid` 2) |
| `getRoles`: `Roles.primary`, `Roles.offhand` | `Roles.right` and `Roles.left`, each a `HandWeaponRole` with `weaponSlot` (`kNoWeaponSlot` when empty) and `role` (`None`, `Firing`, `Support`) |

The old primary hand is the hand whose role is `Firing` for the weapon slot
you care about; the offhand is the hand whose role is `Support`, or the other
hand. Two weapons can be equipped, so a single primary/offhand pair no longer
exists.

### Collision (3)

| 0.92 | 1.0 |
| --- | --- |
| `getBodyContactSnapshotV1` | `copyBodyContacts` (status and count). `BodyContact` loses `targetZone`, `targetSide`, `targetRole`, `targetDescriptorIndex` and `targetInPowerArmor`; target kinds `Hand` (1), `Body` (4) and `External` (5) are retired |
| `copySemanticHandContactsV1` | `copySemanticHandContacts` |
| `copyPlayerColliderDescriptorsV1` | `copyPlayerColliderDescriptors`; the leg and foot colliders are gone, `BodyZoneKind` 16 to 23 are retired |
| `getHandCollisionAvailabilityV1` | `getHandCollisionAvailability` |
| `queryWorldRaycastV1` | `queryWorldRaycast`. `WorldRaycastRequest` gains `version` and `filter`: `Selection` (0) is the 0.92 ray, `Surfaces` (1) skips actors. The result gains `hitLayer` |
| `getEnvironment` | `Environment` gains `version` and loses `externalBodyCount`; the weapon body fields are `weapons[kMaxWeaponSlots]` |
| `clearExternalBodies`, `registerExternalBodiesForScopeV1`, `clearExternalBodiesForScopeV1`, `copyExternalContactsSinceV1` | Removed, no replacement |
| `ExternalBodyContactPolicy::SuppressRockDynamicPush` | Retired: ROCK no longer pushes objects with extra impulses |
| `copyEvents` | Removed |

New: `requestEjectionClearance`, `cancelEjectionClearance`, `watchObjectRest`,
`cancelObjectRest`.

### Grab (4)

| 0.92 | 1.0 |
| --- | --- |
| `requestForceGrabV1`, `requestForceReleaseV1`, `requestThrownDropV1`, `requestPowerArmorGrabV1` | Same names without `V1`; their records lose `reserved` |
| `requestInventoryGrab` | Unchanged name; `InventoryGrabRequest` gains `version` |
| `getInteractionCommandResultV1`, `cancelInteractionCommandV1` | `getInteractionCommandResult`, `cancelInteractionCommand` |
| `getHandInteractionStateV1`, `getHandTargetDetailsV1` | `getHandInteractionState`, `getHandTargetDetails`; layouts kept, `surfaceGripMode` is `touch::SurfaceGripMode` |
| `acquireOffhandReservationV1`, `renewOffhandReservationV1`, `releaseOffhandReservationV1`, `getOffhandReservationStateV1` | `acquireHandReservation`, `renewHandReservation`, `releaseHandReservation`, `getHandReservationState`. `HandReservationRequest` and `HandReservationState` carry `hand` and `weaponSlot`; the reservation applies only while that hand is the weapon's support hand (`HandUnavailable` otherwise) |
| Grab 1.1 `getHeldPlacementState`, `submitHeldPlacementIntent`, `clearHeldPlacementIntent` | In the main table. `HeldPlacementState` reports each hand's stick click (`rightButton`, `leftButton`) and the placeable equipped weapon (`equippedWeapon`, `equippedWeaponHand`). `PlacementIntentFlag::ReserveClick` (1) is retired: reserve `ReserveRightClick` (16) or `ReserveLeftClick` (32) for the hand holding the object. `formId` names only the reference to anchor |
| References `queryReferenceInteractionV1`, `queryPowerArmorTargetV1` | Grab `queryReferenceInteraction`, `queryPowerArmorTarget` (Grab Read) |
| `setEventCallback`, `clearEventCallback` | Kept. `Event` keeps what the callback delivers and loses `sequence`, `ownerToken`, `subjectSequence`, `commandKind`, `commandState`, `failure` and `phase`. `EventKind` 100 to 104 are retired |
| `copyEvents` | Removed |

`Released` now also fires when an equipped weapon is thrown from the hand.
New: `setDefaultGrabPoses`, `clearDefaultGrabPoses`, `setHeldObjectTarget`,
`clearHeldObjectTarget`, `getHeldObjectPose`, `driveReleasedObject`.

### Touch (5)

| 0.92 | 1.0 |
| --- | --- |
| `setTouchGrabTargetsForScopeV1`, `clearTouchGrabTargetsForScopeV1`, `copyTouchGrabStatesForScopeV1`, `requestTouchGrabYieldV1` | Same names without `V1`; `TouchGrabTarget` and `TouchGrabState` lose `reserved` |
| `copyEvents` | Removed; follow `TouchGrabState.sequence` |

New: `setAuthoredHand`, `clearAuthoredHand`, and the target flag
`AcquireWhileGripHeld` for wildcard `FixedAnchor` targets.

### Weapon (6)

Every per-weapon call takes a weapon slot (section 6).

| 0.92 | 1.0 |
| --- | --- |
| `getEquippedWeaponStateV1`, `getEquippedWeaponHandlingStateV1`, `getPrimaryHandV1`, `getOffhandHandV1` | `getWeaponState(owner, slot, state)`. One `WeaponState` holds the state, the handling flags (`HandlingStateFlag`), `firingHand`, `supportHand` and the slot's ammo counter. It is a game-thread read now |
| `getEquippedWeaponGripStateV1` | `getGrip`; `Grip` flag bits 1 (`TwoHandGripActive`) and 2 (`FiringHandLeft`) are retired |
| `queryEquippedWeaponClassificationV1` | `getClassification` |
| `getWeaponEmitterCountV1` + `copyWeaponEmittersV1` | `copyEmitters` |
| `getWeaponCompositionStateV1`, `copyWeaponCompositionEntriesV1` | `getCompositionState`, `copyCompositionEntries` |
| `getSelectedAuthoredGripPoseV1` | `getAuthoredGripPose` |
| `getScopeSightStateV1` | `getScopeSightState`; the record carries `sample` |
| `setEquippedWeaponHandlingAuthorityV1`, `clearEquippedWeaponHandlingAuthorityV1` | `setHandlingAuthority`, `clearHandlingAuthority`; new `getHandlingAuthority`. In `HandlingRequest`, `flags` is `features` and `firingGripPromotionRadiusGameUnits` is removed |
| Weapon 1.1 `captureInventoryWeapon`, `requestInventoryEquip`, `getInventoryEquipResult`, `cancelInventoryEquip` | In the main table. `InventoryWeapon` gains `version`. `EquipResult.weaponSlot` says which slot the weapon went into; `OutgoingTransferFailed` (7) and `EquipResultFlag` bits 0 and 1 are retired |
| `copyEvents` | Removed; follow `WeaponState::terminalSequence` |

An inventory equip into a hand that holds a weapon replaces that weapon and
names it in `EquipResult.outgoingFormId`; it never drops the other hand's
weapon. ROCK's firing-grip handoff zone is a quarter sector beside and below
the grip; set `HandlingFeature::FiringGripCylinderZone` in your handling
request to keep the 0.92 cylinders. New: `unequipWeapon`, `getFiringAimWorld`,
`setWarmInventoryWeapons`, `requestInventoryDraw`, `setInventoryDrawPose`, `setAutomaticReloadBlocked`,
`setChamberMarker`, `getInteractionKey`, `isEquipPickup`.

### WeaponParts (7)

Every per-weapon call takes a weapon slot.

| 0.92 | 1.0 |
| --- | --- |
| `queryWeaponContactAtPoint` | `queryContact` |
| `getWeaponEvidenceDetailCountV1` + `copyWeaponEvidenceDetailsV1` | `copyEvidenceDetails` |
| `getWeaponEvidenceDetailPointCountV1` + `copyWeaponEvidenceDetailPointsV1` | `copyEvidencePoints` |
| `setWeaponPartTargetsV1`, `clearWeaponPartTargetsV1` | `setPartTargets`, `clearPartTargets` |
| `setWeaponPartDriveTargetsV1`, `clearWeaponPartDriveTargetsV1` | `setDriveTargets`, `clearDriveTargets` |
| `getWeaponPartGripStateV1` | `getGripState`. In `GripState`, `providerOwnerToken`, `providerGroupId` and `providerGrabMode` are `targetOwner`, `targetGroupId` and `targetGrabMode` |
| `queryWeaponPartTargetResolutionV1` | `queryTargetResolution`; `ResolutionResult.winningOwnerToken` is `winningOwner` |
| `copyWeaponPartPoseSnapshotV1` | `copyPartPoses` |
| `copyWeaponPartDriveApplicationResultsV1` | `copyDriveResults`; `DriveResult.ownerToken` is `owner` |
| `copySources`, `querySourcePose`, `querySourcePath` | Same names; `SourcePose.flags` is `valid` |
| `copyEvents` | Removed |

Records lose `WeaponPart` and `V1` from their names (`TargetFlag`, `GrabMode`,
`DriveSpace`, `GripKind`, ...) with unchanged values, and carry `size` and
`version`. New: `setCollisionSuppression`, `clearCollisionSuppression`,
`setGrabPoints`, `clearGrabPoints`, `setItemState`, `moveItemState`,
`clearItemStates`. Targets, drives and grab points accept the weapon's
interaction key (Weapon `getInteractionKey`) as their generation, so they work
from the moment the weapon is drawn.

### Animation (8)

| 0.92 | 1.0 |
| --- | --- |
| `setNativeAnimationAuthorityV1`, `clearNativeAnimationAuthorityV1`, `getNativeAnimationAuthorityStateV1` | Same names without `V1`. `NativeAnimationAuthorityRequest` gains `weaponSlot`; the state reports `weaponSlotMask` |
| `setHandVisualAuthorityV1`, `clearHandVisualAuthorityV1` | Same names without `V1`; `HandVisualAuthorityRequest.fingerLocalTransformMask` is `uint32` |
| `publishNativeAnimationRuntimeV1`, `clearNativeAnimationRuntimeV1` | Same names without `V1` |
| Core `registerAnimationPhaseCallbackV1(owner, callback, userData, token)` | Animation `registerAnimationPhaseCallback(owner, registration, token)` with a `PhaseCallbackRegistration`: a nonzero `phaseMask` built with `phaseBit(phase)`, the callback and `userData`. Bind Animation with `Callbacks`. ROCK calls you only for the phases in the mask. Registering at startup still works |
| `copyEvents` | Removed |

### Input (9)

| 0.92 | 1.0 |
| --- | --- |
| `setHandInputSuppressionV1`, `clearHandInputSuppressionV1`, `getHandInputSuppressionStateV1` | Same names without `V1`. The request's `chordButtonsLow[2]` and `chordButtonsHigh[2]` are one `uint64 chordButtons[2]` (index 0 the left wand, 1 the right). The state's `frameIndex` is `leaseFrameIndex`, its generations are `sample` |
| `getRawWandButtonStateV1`, `getRawWandThumbstickV1`, `isNativePipboyInputSuppressedV1`, `getNativeInputContextV1`, `getLogicalInputActionStateV1` | Same names without `V1`; `LogicalInputActionState` carries `sample` |
| Input 1.1 `getPlacementClickState` | In the main table. `PlacementClickState` gains `version` and loses `objectFormId`; `InputReserved` (2) is retired for `RightInputReserved` (4) and `LeftInputReserved` (8) |
| `copyEvents` | Removed; a lost suppression lease fails its renew and records `lastInvalidationReason` |

New: `setReloadGesture`, `clearReloadGesture`, `getReloadGesture`,
`setSupportTrigger`, `clearSupportTrigger`, `getSupportTrigger`, and the
logical action `PalmWheel`.

### References (10): retired

Interface 10 is refused. `queryReferenceInteraction` and
`queryPowerArmorTarget` are in Grab with the same records, without `V1`.

### PlayerController (11)

`getPlayerControllerStateV1` and `requestPlayerControllerJumpV1` lose `V1`.
`PlayerControllerState` carries `sample`, `PlayerControllerJumpRequest` loses
`reserved`, and state flag bits 7 and 8 are retired.

### Diagnostics (12): retired

Interface 12 is refused. ROCK no longer draws anything for other mods:
`publishDebugOverlayV1`, `clearDebugOverlayV1` and the collider visualization
calls have no replacement in ROCK. Draw lines, spheres, text and meshes through
RPS UI Framework's WorldDraw interface, and keep working when RPS UI is absent.

### Configuration (13)

- `ApiV1`, `SettingV1`, `VisitorV1` and `PermissionV1` lose `V1`. The table is
  `revision`, `visit`, `setValue` and the new `openSetupWizard`.
- ROCK's settings files are `ROCK.toml` and `ROCK_Developer.toml`.
  `Setting::section` is now a TOML table path and keys lose their type letter:
  the 0.92 `[PhysicsInteraction] iFarGrabMode` is table `Grabbing`, key
  `FarGrabStyle`. Table and key match exactly and case-sensitively; the old INI
  spellings are not accepted. Read `visit` for the current catalog instead of
  hard-coding names.
- `ValueType::Float3` is new: one setting holds `[x, y, z]` (or
  `[pitch, yaw, roll]`) where 0.92 had three keys. Float values always carry a
  `.` or an exponent.
- `setValue` is refused while the target file has a syntax error.
- Many settings were moved, renamed or removed; ROCK's `CHANGELOG.md` lists
  them.

### New interfaces

NativeInteraction (14) and Storage (15) did not exist in 0.92. See their
pages if you need them.

## 6. Weapon slots

A player can equip two weapons. Everything about a weapon takes a weapon slot,
the native equipment index: 0 is the first equipped weapon, 1 the second
(`kMaxWeaponSlots` = 2). Pass slot 0 where 0.92 meant "the equipped weapon" or
"the primary weapon". A slot never implies a hand: read hands from
`WeaponState.firingHand` and `supportHand`, or from Hands `getRoles`. The
[Overview](Overview.md#weapon-slots) lists what slot 1 serves. A hand holding
the slot-1 weapon reports `Holding` in Grab's hand state, so check that state
before treating a hand as free.

## 7. Event streams

`copyEvents` is gone from every interface. Follow published state instead:

| You read from a stream | Read instead |
| --- | --- |
| Grab, release and hand state changes | Grab `getHandInteractionState` and its sequences |
| Command completion | Grab `getInteractionCommandResult` |
| A lost reservation | Grab `getHandReservationState` |
| Hand touches | Collision `copySemanticHandContacts` and their contact state |
| Touch-grab state | `TouchGrabState` and its `sequence` |
| Lifecycle, physics start and stop | Core `Snapshot` lifecycle flags and `lastLifecycleReason` |
| Equip transitions | `WeaponState::terminalSequence` |
| A lost lease | The renew call fails; Input suppression also sets `lastInvalidationReason` |
| Pull, catch, impact, release with velocity | The Grab event callback, unchanged |

When ROCK revokes your owner after one of your callbacks faulted, every call
returns `OwnerRevoked`.

## 8. Behaviour that changed without a compile error

- Two weapons: slot 1 is real data, and the hand holding it is busy.
- An inventory equip replaces the weapon of the hand you equip into.
- Grab `Released` also fires for a thrown equipped weapon.
- Animation phase callbacks run only for the phases you select.
- Weapon `getWeaponState` and Hands `getPresentedHandFrame` are game-thread
  calls; elsewhere they return `WrongThread`.
- Stick clicks are per hand: a placement or the bipod reserves only one hand's
  click.
- The leg and foot body colliders are gone.
- ROCK draws nothing for other mods.
- Every version refusal stops the game, including a refusal your own fallback
  query caused.

## 9. Check the result

1. Build against `include/ROCK/` with no RPS SDK path left. Fix every compile
   error with the tables above.
2. Search your source for leftovers: `V1` outside the discovery names,
   `reserved`, `copyEvents`, `Offhand`, `primary`, `ForOwner`, and literal
   majors or table sizes.
3. Start the game with ROCK 1.0. No error box appears at the main menu,
   `ROCK.log` names no refusal for your DLL, and your own log shows `connect`
   and every `acquire` returning `Ok`.
4. If your mod touches weapons, test with a weapon in each hand.

Retired ids, slots, enum values and flag bits are listed on each interface's
page and are never reused.
