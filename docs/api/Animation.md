# Animation (interface 8, major 2)

Header: `include/ROCK/Animation.h`. Animation-phase callbacks, native
animation authority leases, hand visual authority, and the native animation
runtime status published by the mod that executes it.

Interface `Animation`, major 2, minor 0; major 1 is refused. Permissions:
`Read` (1), `Write` (2), `Callbacks` (4).

## Animation phases

`registerAnimationPhaseCallback(owner, registration*, token*)` (game thread,
Callbacks; up to `kMaxPhaseCallbacks`) takes a `PhaseCallbackRegistration`
(`phaseMask`, `callback`, `userData`) and calls `callback(context, userData)`
only at the phases selected in `phaseMask`. Build the mask with
`phaseBit(phase)`; it must be a nonzero subset of `kAllPhases`, otherwise the
call returns `InvalidArgument`. Select only the phases the callback handles:
unselected phases cost nothing. The phases of an animation frame:

| Phase | When | Allowed |
| --- | --- | --- |
| `NativeGraphOutput` | at the player graph-output entry, before native scene and FRIK writers | capture only |
| `BeforeRock` | before ROCK writes weapon and hand presentation | writes |
| `AfterRock` | after ROCK's interaction update | writes |
| `Complete` | after all visual writers; closes the control frame | writes |
| `Presented` | after FRIK's world final, once per skeleton frame | observation only |

`AnimationPhaseContext`: `phase`, `flags` (`AnimationPhaseContextFlag`:
`RockEnabled`, `ProviderReady`, `SkeletonReady`, `MenuBlocking`,
`ConfigBlocking`, `VisualWritesAllowed`), `frameIndex`, `deltaSeconds` and
the generations. The context is borrowed for the call; a faulting callback
revokes its owner. `unregisterAnimationPhaseCallback` stops future calls.

## Native animation authority

`setNativeAnimationAuthority(owner, request)` takes a lease (1 to
`kMaxNativeAuthorityLeaseFrames` frames) on native arm, hand and weapon
animation (`NativeAnimationAuthorityFlag`; `ReloadPose` = all three) for the
request's generations; `weaponSlot` names the weapon the `Weapon` flag
covers. A weapon slot is the native equipment index (see `Weapon.md`); a slot at
or above `kMaxWeaponSlots` returns `InvalidArgument`, and slot 1 returns
`TargetUnavailable`. `clearNativeAnimationAuthority` releases it.
`getNativeAnimationAuthorityState` reports the active flags, the weapon slots
under `Weapon` authority (`weaponSlotMask`, bit per slot), status flags
(`NativeAnimationAuthorityStatusFlag`), owner count and capture counters.

The mod that executes the native animation publishes its status with
`publishNativeAnimationRuntime` (lease up to
`kMaxRuntimePublicationLeaseFrames`) and withdraws it with
`clearNativeAnimationRuntime`.

## Hand visual authority

`setHandVisualAuthority(owner, request)` overrides one hand's rendered world
transform and/or finger joints (`HandVisualAuthorityFlag`) with a priority in
[-10000, 10000] and a lease up to `kMaxHandVisualLeaseFrames`. Finger joints
follow the order thumb, index, middle, ring, pinky (three each, parent-local);
bit N of `fingerLocalTransformMask` enables joint N. Not accepted during the
`Presented` phase. `clearHandVisualAuthority(owner, hand)` releases it.

## Sample

`getSample` returns the publication identity. A lost authority shows up as a
failed renew.

## Records

| Record | Used by | Fields |
| --- | --- | --- |
| `PhaseCallbackRegistration` | `registerAnimationPhaseCallback` | `phaseMask`, `callback`, `userData` |
| `AnimationPhaseContext` | phase callbacks | `phase`, `flags`, `frameIndex`, `deltaSeconds`, the three generations |
| `NativeAnimationAuthorityRequest` | `setNativeAnimationAuthority` | `flags` (`NativeAnimationAuthorityFlag`: `Arms`, `Hands`, `Weapon`, `ReloadPose`), `weaponSlot`, `leaseFrames`, the three guard generations |
| `NativeAnimationAuthorityState` | `getNativeAnimationAuthorityState` | `activeFlags`, `weaponSlotMask`, `statusFlags`, `activeOwnerCount` (up to `kMaxNativeAuthorityOwners`), `capturedTransformCount`, `captureSequence` |
| `HandVisualAuthorityRequest` | `setHandVisualAuthority` | `hand`, `flags` (`HandVisualAuthorityFlag`: `WorldTransform`, `FingerLocalTransforms`), `priority`, `fingerLocalTransformMask`, `worldTransform`, `fingerLocalTransforms[kFingerLocalTransformCount]` (15), `leaseFrames`, the three guard generations |
| `NativeAnimationRuntimePublication` | `publishNativeAnimationRuntime` | `statusFlags`, `capturedTransformCount`, `captureSequence`, `leaseFrames`, the three generations |

`NativeAnimationAuthorityStatusFlag`: `HookInstalled`, `RuntimeEnabled`,
`CaptureValid`, `LocalReloadTestLeaseActive`, `HookInstallFailed`,
`ThreadMismatch`, `CaptureFault`, `RuntimeProviderAvailable`.
