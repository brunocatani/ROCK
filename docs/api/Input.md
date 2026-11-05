# Input (interface 9, major 2)

Header: `include/ROCK/Input.h`. Hand input suppression, raw controller reads,
native input context, and leases on the reload button and support trigger of
the equipped weapon.

Interface `Input`, major 2, minor 0; major 1 and its minors are refused.
Permissions: `Read` (1), `Write` (2). All functions except `getSample` run on
the game thread.

## Hand input suppression

`setHandInputSuppression(owner, request)` (Write) suppresses ROCK and native
input on one hand for `leaseFrames` (up to `kMaxSuppressionLeaseFrames`); renew
it each frame you need it. `flags` are `HandInputSuppressionFlag` bits.
`ReserveButtonChord` reserves the OpenVR buttons in `chordButtons[0]` (left
wand) and `chordButtons[1]` (right wand); allowed bits are 1, 2, 7, 32 and 33,
and the requesting hand's entry must be nonzero. Nonzero generation fields
must match ROCK's current ones. `clearHandInputSuppression(owner, hand)`
clears one hand, or both with `Hand::None`.

`getHandInputSuppressionState(owner, hand, state)` (Read) reports your lease
(`callerFlags`, `callerLeaseActive`, `callerExpiresAfterFrame`,
`callerRemainingFrames`, `lastInvalidationReason`) and the flags in effect
from every owner (`effectiveFlags`). `leaseFrameIndex` is the lease clock the
expiry counts against; `sample` identifies the publication.

## Raw reads (Read)

- `getRawWandButtonState(owner, hand, buttonId, state)`: level state
  (`available`, `held`) of one physical button, with the controller sample's
  sequence and age. Edges are not exposed.
- `getRawWandThumbstick(owner, hand, x, y)`: physical stick axes.
- `isNativePipboyInputSuppressed(owner, value)`: 1 while ROCK suppresses
  native Pip-Boy input.
- `getNativeInputContext(owner, value)`: `NativeInputContextFlag` bits
  (`Available`, `MenuActive`, `PrimaryActivationTarget`); 0 when the input
  hook is unavailable.
- `getLogicalInputActionState(owner, LogicalInputAction::Jump, state)`: the
  jump action's level, press sequence and availability.
- `getPlacementClickState(owner, state)`: `PlacementClickFlag::InputReserved`
  while an add-on's held placement action owns the right-stick click,
  including its release; `objectFormId` is the submitted candidate.

## Reload gesture

`setReloadGesture(owner, request)` (Write) leases the Accept button (X left,
A right) of the firing hand of the weapon in `weaponSlot` for 1 to
`kMaxWeaponInputLeaseFrames` frames. The request names the weapon
(`weaponSlot`, with `weaponFormId` and `weaponGenerationKey` as witnesses)
and a sample from the current or previous frame. A weapon slot is the native equipment index (see `Weapon.md`); a slot at
or above `kMaxWeaponSlots` returns `InvalidArgument`, and slot 1 returns
`TargetUnavailable`. One owner holds it at a time (`Busy` otherwise); the wrong
hand returns `HandUnavailable`.

ROCK classifies the press with `fManualScopeHoldSeconds`: a short release adds
one to `tapSequence`; a hold stays ROCK scope input (`scopeHeld`) and never
becomes a tap. Flags:

- `BlockFire`: suppress that hand's native trigger while set; firing resumes
  after a physical trigger release.
- `GameplayPriority`: a workbench, use or take target of the primary wand
  owns the whole press and release. Without it, Accept is masked from the
  game for the lease.
- `KeepAcceptNative`: Accept stays entirely with the game: no mask, tap or
  scope hold, so its native reload and activation keep working and the lease
  only carries `BlockFire`. Not valid with `GameplayPriority`.

Menus, input loss, hand changes and lease changes (including flags) cancel an
unfinished gesture and need a release before rearming. Expiry, clearing and
owner loss keep a held button drained from the game until it is released.
`clearReloadGesture` releases the lease; `getReloadGesture` (Read) reads it.

## Support trigger

`setSupportTrigger(owner, request)` (Write) leases the physical trigger of the
supporting hand of an active two-hand grip on the weapon in `weaponSlot` (1 to
`kMaxWeaponInputLeaseFrames` frames, current or previous sample; slot 1
returns `TargetUnavailable`, as for the reload gesture). It suppresses only that trigger's game
button, axis and native action. `pressSequence` is monotonic across leases;
read a baseline after acquiring. A new claim or unavailable input needs a
release before a fresh press; clearing, expiry, hand changes and owner loss
drain a held trigger. `weaponGenerationKey` is the weapon's interaction key
(Weapon `getInteractionKey`), so the lease works without colliders and stays
held when they arrive and that key changes. `clearSupportTrigger` releases it;
`getSupportTrigger` (Read) reads it.

## Sample

`getSample` returns the publication identity. A lost suppression shows up as
a failed renew and in the state's `lastInvalidationReason`.

## Records and enums

| Record | Used by | Fields |
| --- | --- | --- |
| `HandInputSuppressionRequest` | `setHandInputSuppression` | `hand`, `flags`, `leaseFrames`, the three guard generations, `chordButtons[2]` |
| `HandInputSuppressionState` | `getHandInputSuppressionState` | `sample`, `leaseFrameIndex`, `callerExpiresAfterFrame`, `hand`, `callerFlags`, `effectiveFlags`, `callerLeaseActive`, `callerRemainingFrames`, `lastInvalidationReason` |
| `RawWandButtonState` | `getRawWandButtonState` | `available`, `held`, `sampleSequence`, `sampleAgeMilliseconds`, `availabilityReason` |
| `LogicalInputActionState` | `getLogicalInputActionState` | `sample`, `sampleSequence`, `pressSequence`, `action`, `available`, `held`, `availabilityReason`, `sampleAgeMilliseconds` |
| `PlacementClickState` | `getPlacementClickState` | `flags`, `objectFormId`, `sample` |
| `ReloadGestureRequest` | `setReloadGesture` | `weaponSlot`, `weaponFormId`, `weaponGenerationKey`, `sample`, `hand`, `leaseFrames`, `flags` (`ReloadGestureFlag`) |
| `ReloadGestureState` | `getReloadGesture` | `owner`, `tapSequence`, `expiresAfterFrame`, `sample`, `active`, `hand`, `held`, `scopeHeld`, `weaponSlot` |
| `SupportTriggerRequest` | `setSupportTrigger` | `weaponSlot`, `weaponFormId`, `weaponGenerationKey`, `sample`, `hand`, `leaseFrames` |
| `SupportTriggerState` | `getSupportTrigger` | `owner`, `pressSequence`, `expiresAfterFrame`, `sample`, `active`, `hand`, `held`, `weaponSlot` |

| Enum | Values |
| --- | --- |
| `HandInputSuppressionFlag` | `SuppressNormalGrabPress`, `SuppressGrabRelease`, `SuppressHeldWeaponTriggerEquip`, `SuppressGameplayCandidates`, `SuppressOpenVrGameInput`, `SuppressNativeVats`, `SuppressNativeVans`, `SuppressGrenadeQuickDraw`, `ReserveTriggerGripChord`, `ReserveButtonChord`; `SuppressConfigModeChord` is the first four together |
| `InputAvailabilityReason` | `Available`, `HookNotSampled`, `BlockingMenu`, `ReleaseToRearm`, `InvalidButton` |
| `SuppressionInvalidationReason` | `None`, `Expired`, `GenerationChanged`, `OwnerUnregistered`, `ProviderLost`, `ExplicitClear`, `CallbackFault` |
| `LogicalInputAction` | `Jump` |

Up to `kMaxSuppressions` (32) suppression leases are held across owners.

## Retired values

`PlacementClickFlag` bit 0 is retired and never reused.
