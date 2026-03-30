# ROCK Shoulder Stash Plan and Tracker

Date: 2026-05-11
Status: implemented, local Release build passed, in-game validation pending
Scope: ROCK FO4VR shoulder stash for held loose objects

## Decision Summary

This plan chooses a hybrid detector because ROCK already has generated full-body
colliders with body-zone metadata, while an HMD-relative shoulder sphere remains
useful only as an availability fallback. This solves the main precision problem
in the HIGGS-style approach: the headset is not the torso, especially when FRIK
body pose, leaning, power armor scale, or player calibration moves the real
shoulder away from a fixed HMD offset. A pure HMD gate was rejected because it is
less precise than the actual ROCK/FRIK shoulder bodies. A pure collider-only
gate was also rejected because collider rebuilds, skeleton readiness, or body
provider warmup can temporarily remove shoulder-zone evidence. Object
eligibility follows the user-approved HIGGS rule: the currently held object can
be stashed when it is not actor-owned body content, its base form is playable,
and untakeable books are excluded after the FO4VR book flag is verified.

## User Decisions Already Made

- Detection authority: body colliders primary, HMD fallback only when body
  collider data is unavailable or stale.
- Commit behavior: candidate feedback while held near the shoulder, actual stash
  on grip release in the confirmed zone.
- Object eligibility: use the HIGGS-style loose playable object rule instead of
  a strict form-type whitelist.

## Local Architecture Mapped

ROCK already has most of the structural scaffolding needed:

- Body zones already include stable shoulder enum values:
  - `BodyZoneKind::LeftShoulder = 6`
  - `BodyZoneKind::RightShoulder = 11`
  - API mirror values are asserted in `ROCK/tests/ProviderBoundaryTests.cpp`.
- Full-body generated colliders already carry semantic metadata:
  - `ROCK/src/physics-interaction/body/BodyBoneColliderSet.h`
  - `BodyColliderMetadata` includes zone, side, role, body id, descriptor index,
    and power-armor state.
- The standard and power-armor shoulder collider descriptors already exist:
  - standard left shoulder: `LArm_Collarbone` to `LArm_UpperArm`, radius `2.5`
  - standard right shoulder: `RArm_Collarbone` to `RArm_UpperArm`, radius `2.5`
  - power armor shoulders use radius `4.2`
  - source: `ROCK/src/physics-interaction/debug/SkeletonBoneDebugMath.h`
- `BodyContactRuntime` already states that body-zone contacts are intended for
  stash, holsters, backpack zones, and provider queries.
- Grab event ABI already reserves:
  - `StashCandidate = 11`
  - `Stashed = 13`
- Hand lifecycle already has:
  - `HandState::StashCandidate`
  - `BeginStashCandidate`
  - `CommitStash`
  - `CancelGameplayCandidate`
  - candidate states still count as holding states.
- Current normal release path lives in
  `ROCK/src/physics-interaction/core/PhysicsInteraction.cpp::updateGrabInput`.
  The stash check must run before the normal release/drop path.
- Current release code in `Hand::releaseGrabbedObject` always applies throw
  velocity on final object release. Stash needs an explicit transfer disposition
  so a released-to-inventory object does not get a throw impulse.

## HIGGS Behavior Used As Approved Reference

HIGGS is used here only for the object-eligibility behavior the user approved,
not as the overall design authority.

Mapped source:

- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/hand.cpp:1908`
  - `Hand::IsObjectDepositable`
- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/hand.cpp:3108`
  - release path that consumes or stashes while held
- `E:/fo4dev/skirymvr_mods/source_codes/higgs/src/config.cpp`
  - shoulder velocity, radius, haptic, and book-skip config

HIGGS deposit eligibility:

- Reject selected object if it is an actor.
- Resolve `refr->baseForm`.
- Reject null base form.
- Reject base forms that are not playable.
- If the base form is a book, reject books with the cant-be-taken flag.
- Use low hand/controller speed plus shoulder-zone position to decide whether
  the held object is depositable.

HIGGS transfer behavior:

- Count is read from the held reference extra data.
- Stash callbacks are triggered before native activation.
- Books can skip activation and use native pickup instead.
- Other items use native reference activation with the player as action ref.

FO4VR differences that matter:

- ROCK can hold target categories HIGGS did not model, especially actor
  equipment and detached body content. These must be blocked unless they have
  already materialized as ordinary loose references.
- ROCK has real generated body colliders and should not use HMD offsets as the
  primary shoulder location.
- FO4VR CommonLib/F4VR helpers must be verified before using offsets or raw
  extra-data layout.

## Ghidra Audit Notes

User approval: granted for this task.

Binary loaded in Ghidra:

- Image base: `0x140000000`
- `.text`: `0x140001000` to `0x142C40BFF`

Verified calls:

- `TESObjectREFR::ActivateRef`
  - CommonLibF4VR declaration: `REL::ID(753531)`, offset `0x3F4A60`
  - Ghidra VA: `0x1403F4A60`
  - Function found: `FUN_1403f4a60`
  - Decompile shape matches a 7-argument native activation wrapper returning
    `bool`.
  - Plan: use the CommonLibF4VR wrapper, not a direct handwritten call.

- `TESObjectREFR::AddInventoryItem`
  - CommonLibF4VR declaration: `REL::ID(78185)`, offset `0x3E3DD0`
  - Ghidra VA: `0x1403E3DD0`
  - Function found: `FUN_1403e3dd0`
  - Decompile shape matches a native add-inventory path with container ref,
    base object, extra-data pointer, count, and owner/source parameters.
  - Plan: keep this as a document/book/holotape bypass option only if native
    `ActivateRef` opens UI or has unwanted activation behavior.

- Extra count setter correction:
  - `F4VR-CommonFramework` labels `ExtraDataList_setCount` at offset `0x88FE0`.
  - Ghidra shows `0x140088FE0` uses extra type `0x0B` and constructs
    `ExtraPersistentCell`, not `ExtraCount`.
  - `F4VR-CommonFramework` labels `ExtraDataListSetPersistentCell` at offset
    `0x87BC0`.
  - Ghidra shows `0x140087BC0` constructs `ExtraCount`, uses extra type `0x24`,
    and stores a `uint16_t` count at extra-data offset `+0x18`.
  - Ghidra also resolves `ExtraCount::ExtraCount` at `0x1400B92A0`.
  - Plan: do not call the misnamed framework helper for stack count. Add a
    ROCK-owned `ExtraCount` utility with the verified `kCount` type and guarded
    `uint16_t` read, or call the verified function only through a correctly
    named local wrapper.

- `ExtraDataList_setAmmoCount`
  - Offset `0x980D0`, VA `0x1400980D0`
  - Ghidra shows extra type `0x6A` and a 32-bit value at `+0x18`.
  - Not directly needed for shoulder stash.

Open verification before code:

- FO4VR `TESObjectBOOK::data.flags` cant-be-taken bit is not named in the local
  CommonLibF4VR header. It was verified before implementation from the approved
  local FO4/F4SE source reference:
  `E:/fo4dev/fallout4vr_mods/source_codes/Neanka-mods-repo/f4seee/f4seeeEvents.h`
  uses `kType_CantBeTaken = 1 << 1`, and Ghidra confirmed the FO4VR
  `TESObjectBOOK` constructor stores book data at `+0x170`. ROCK now uses
  `1u << 1` for the untakeable-book exclusion.

## Implementation Status

Implemented files:

- `src/physics-interaction/stash/ShoulderStashMath.h`
- `src/physics-interaction/stash/ShoulderStashDetector.h`
- `src/physics-interaction/stash/ShoulderStashDetector.cpp`
- `src/physics-interaction/stash/ShoulderStashEligibility.h`
- `src/physics-interaction/stash/ShoulderStashEligibility.cpp`
- `src/physics-interaction/stash/ShoulderStashTransfer.h`
- `src/physics-interaction/stash/ShoulderStashTransfer.cpp`

Integration points:

- `BodyBoneColliderSet` now publishes generated collider length/radius metadata
  so the shoulder detector can evaluate the real shoulder capsule, not a fixed
  HMD sphere.
- `Hand::GrabReleaseContext` now carries `GrabReleaseDisposition`.
- `Hand::releaseGrabbedObject` applies throw velocity only for
  `PhysicalDrop`; stash transfer uses `TransferToInventory` and commits the
  `StashCandidate -> Idle` path through `CommitStash`.
- `PhysicsInteraction::updateGrabInput` evaluates stash eligibility and body
  evidence before ordinary release. A confirmed release transfers to player
  inventory through `TESObjectREFR::ActivateRef`; failed transfer falls back to
  the ordinary released event instead of deleting anything.
- `StashCandidate` and `Stashed` grab events now produce configured haptics.
- Packaged and active production `ROCK.ini` were updated in place.

Verification completed:

- `powershell -NoProfile -ExecutionPolicy Bypass -File ROCK/tests/ShoulderStashSourceTests.ps1 -Root ROCK`
- `ctest -C Release -R "ShoulderStashSourceTests|ROCKHandStateMachineTests" --output-on-failure`
- `powershell -NoProfile -ExecutionPolicy Bypass -File ROCK/tests/ConfigDefaultParitySourceTests.ps1 -Root ROCK`
- `VCPKG_ROOT=C:/vcpkg cmake --build build --config Release -- /m:1`

Build result:

- `ROCK/build/Release/ROCK.dll` built successfully.
- Existing post-build copy deployed `ROCK.dll` and `ROCK.pdb` to
  `D:/FO4/mods/ROCK/F4SE/Plugins/`.

## Target User Behavior

While the player holds an eligible loose item:

1. The hand or held-object stash probe enters a shoulder zone.
2. ROCK detects the shoulder using actual generated body colliders when valid.
3. ROCK gives candidate feedback without transferring the item.
4. If the player releases grip while the candidate is still confirmed, ROCK
   transfers the object to the player inventory.
5. If the player moves away before releasing, the hand returns to normal held
   state and normal release/drop behavior remains unchanged.
6. If body collider data is unavailable, ROCK falls back to an HMD-relative zone
   and logs that the fallback was used.

Both hands may stash to either shoulder. Same-side shoulder evidence gets a
confidence bias, but cross-side stash should still work when the player
deliberately reaches across the chest.

## Detection Model

Primary source: body-zone collider query.

Add a shoulder-zone query layer over `BodyBoneColliderSet`:

- Enumerate active body collider metadata.
- Find colliders whose zone is `LeftShoulder` or `RightShoulder`.
- Read current body transforms through the existing Havok runtime body transform
  path.
- Produce a compact `BodyZoneVolume`:
  - zone kind
  - side
  - body id
  - center line or capsule endpoints
  - radius
  - in-power-armor flag
  - transform freshness/frame number if available

Candidate probe point:

- Use the held object's current grip/pivot when available.
- Fall back to the hand grab anchor if object transform is unavailable.
- Keep this explicit in the detector input so future backpack/holster systems
  can use different probe points without changing the detector contract.

Distance test:

- Treat the shoulder body as a capsule.
- Compute distance from stash probe to the capsule segment.
- Candidate enters when distance is less than `radius + enterPadding`.
- Candidate remains active until distance exceeds `radius + exitPadding`.
- `exitPadding` must be larger than `enterPadding` to avoid flicker.

Contact evidence:

- If `BodyContactRuntime` reports recent contact with the matching shoulder
  zone, use it as strong evidence.
- Contact evidence should not be the only path; geometric zone query is needed
  so release in the shoulder pocket works even if contact callbacks miss a
  frame.

Velocity and dwell:

- Use a low-speed gate to avoid accidental stashing during fast throws.
- Use ROCK's current held-object/hand velocity history instead of copying HIGGS
  units directly.
- Add a small dwell requirement before candidate confirmation.
- Candidate may still start immediately for feedback, but `confirmedForCommit`
  should require dwell or strong contact evidence.

Fallback source: HMD-relative zone.

Use only when:

- FRIK skeleton is not ready.
- Body collider set has no valid shoulder bodies.
- Shoulder body transform read fails.
- Body collider data is stale for too many frames.

Fallback should:

- Use explicit config offsets/radii.
- Mark the decision source as `HmdFallback`.
- Log sampled fallback reasons.
- Never override valid body-collider evidence.

## Detector API Shape

Proposed new module:

- `src/physics-interaction/stash/ShoulderStashMath.h`
- `src/physics-interaction/stash/ShoulderStashDetector.h`
- `src/physics-interaction/stash/ShoulderStashDetector.cpp`

Core data shapes:

```cpp
enum class ShoulderStashEvidenceSource : std::uint8_t
{
    None,
    BodyZoneCollider,
    BodyZoneContact,
    BodyZoneColliderAndContact,
    HmdFallback,
};

struct ShoulderStashProbe
{
    RE::NiPoint3 pointGame;
    RE::NiPoint3 velocityGamePerSecond;
    bool hasVelocity;
};

struct ShoulderStashDecision
{
    bool candidate;
    bool confirmedForCommit;
    body_zone::BodyZoneKind zone;
    std::uint32_t shoulderBodyId;
    ShoulderStashEvidenceSource source;
    float distanceGameUnits;
    float confidence;
};
```

Per-hand runtime state:

- current candidate zone
- current evidence source
- frames/seconds inside zone
- hysteresis active flag
- last shoulder body id
- last confidence
- last candidate event frame
- fallback reason for sampled logging

## Eligibility Model

Proposed new module:

- `src/physics-interaction/stash/ShoulderStashEligibility.h`
- `src/physics-interaction/stash/ShoulderStashEligibility.cpp`

Eligibility rules:

- Stash feature enabled in config.
- Held reference is non-null.
- Held reference is not deleted or disabled.
- Held reference is not the player.
- Held target is a loose inventory-compatible reference.
- Reject live actors, whole actor bodies, detached gore, and actor equipment
  still owned by an actor/equipment provider.
- Resolve `baseForm = heldRef->GetObjectReference()`.
- Reject null base form.
- Reject base form type `NPC_`.
- Require `baseForm->GetPlayable(baseForm->GetBaseInstanceData())`.
- If base form is `BOOK`, reject untakeable books after FO4VR book flag
  verification.
- Do not apply a strict MISC/WEAP/BOOK/etc. whitelist. This matches the
  user-approved HIGGS-style eligibility and lets native activation remain the
  final authority.

Reason strings should be returned for logs and tests:

- `disabled-by-config`
- `missing-held-ref`
- `deleted-or-disabled`
- `player-ref`
- `non-loose-rock-target`
- `actor-or-body-target`
- `missing-base-form`
- `non-playable-base`
- `untakeable-book`
- `eligible`

## Transfer Model

Proposed new module:

- `src/physics-interaction/stash/ShoulderStashTransfer.h`
- `src/physics-interaction/stash/ShoulderStashTransfer.cpp`

Stack count:

- Default to count `1`.
- If the held reference has an `ExtraDataList`, read `ExtraCount`.
- Verified by Ghidra:
  - extra type is `0x24`
  - value is `uint16_t` at extra-data offset `+0x18`
  - values below `2` should behave as count `1`
- Do not use the misnamed `F4VR-CommonFramework::ExtraDataList_setCount`
  helper for this.

Native transfer:

- Default path:
  - release ROCK physics authority with a transfer-to-inventory disposition
  - call `heldRef->ActivateRef(player, nullptr, count, false, false, false)`
  - dispatch `Stashed` only on native success
- Failure path:
  - if native transfer returns false, do a normal physical release without
    stash success events
  - log the eligibility result, native return, ref form id, base type, count,
    evidence source, and zone
  - never delete or disable the reference manually on failure

Book/document bypass:

- HIGGS skips activation for books to avoid opening the book UI.
- FO4VR has both `BOOK` and `NOTE`/holotape-like behavior.
- Add the bypass as a separately configured transfer mode only after verifying
  FO4VR behavior in-game.
- Candidate bypass implementation options:
  - `Actor::PickUpObject(ref, count, playPickUpSounds)`
  - `TESObjectREFR::AddInventoryItem`
  - `TESObjectREFR::AddObjectToContainer`
- Do not choose the bypass blindly. It must preserve stack count and avoid
  duplicating or destroying references.

Release disposition:

Extend `GrabReleaseContext` with an explicit disposition, not ad hoc booleans:

```cpp
enum class GrabReleaseDisposition : std::uint8_t
{
    PhysicalDrop,
    TransferToInventory,
    OwnershipHandoff,
};
```

For `TransferToInventory`:

- do not apply throw linear/angular velocity
- do restore or clear ROCK-owned grab constraints and native grab actions
- do clear hand pose, selection, body leases, and lifecycle state
- do not schedule delayed hand-collision restore if the held object disappears
  immediately, unless the same restore delay is still needed for hand/body
  penetration recovery
- keep logs distinct from normal release logs

## Per-Frame Flow

In `PhysicsInteraction::updateGrabInput`, while `hand.isHolding()`:

1. Build `ShoulderStashProbe` from the held object grip/pivot or hand anchor.
2. Evaluate stash eligibility for the held reference.
3. Evaluate shoulder stash detector.
4. If eligible and detector has candidate:
   - apply `BeginStashCandidate` when current state is `HeldBody`
   - dispatch `StashCandidate` only on candidate enter or meaningful source/zone
     change
   - drive candidate haptics
5. If candidate becomes invalid:
   - apply `CancelGameplayCandidate`
   - return to `HeldBody`
6. On grip release:
   - if `confirmedForCommit`, execute stash transfer path
   - otherwise run existing normal release/drop path unchanged

Same-frame release handling:

- Run stash candidate detection before the normal release block.
- If the hand enters a confirmed candidate on the same frame the grip is
  released, apply `BeginStashCandidate` and then `CommitStash` in order.
- Do not allow `CommitStash` from `HeldInit`; wait until held acquisition has
  completed or fall back to normal release.

## Events And Haptics

Events:

- `StashCandidate`
  - dispatch on candidate enter
  - `refr`: held reference
  - `formID`: held reference form id
  - `primaryBodyId`: held object body id when known
  - `secondaryBodyId`: shoulder body id when known
  - `positionGame`: stash probe position or shoulder hit/proximity point
  - `intensityHint`: confidence
- `Stashed`
  - dispatch only after native transfer success
  - same captured ref/form/body metadata where safe
  - external consumers should treat `formID` as authoritative because native
    activation may disable or detach the reference

Haptics:

- Add stash candidate and stash commit handling to
  `PhysicsInteraction::handleGrabEventHaptics`.
- Candidate feedback should be sustained but rate-limited, not emitted as an
  event every frame.
- Commit feedback should be a stronger short pulse.
- Add config:
  - `bShoulderStashHapticsEnabled`
  - `fShoulderStashCandidateHapticIntensity`
  - `fShoulderStashCandidateHapticDurationSeconds`
  - `fShoulderStashCandidateHapticIntervalSeconds`
  - `fShoulderStashCommitHapticIntensity`
  - `fShoulderStashCommitHapticDurationSeconds`

## Config Plan

Add config to `RockConfig.h`, `RockConfig.cpp`, `ROCK/data/config/ROCK.ini`,
and production `C:/Users/SENECA/Documents/My Games/Fallout4VR/ROCK_Config/ROCK.ini`
in place when implementation starts.

Candidate keys:

- `bShoulderStashEnabled = true`
- `bShoulderStashUseBodyZoneColliders = true`
- `bShoulderStashAllowHmdFallback = true`
- `fShoulderStashEnterPaddingGameUnits`
- `fShoulderStashExitPaddingGameUnits`
- `fShoulderStashMinDwellSeconds`
- `fShoulderStashMaxSpeedGameUnitsPerSecond`
- `fShoulderStashFallbackRightOffsetX/Y/ZGameUnits`
- `fShoulderStashFallbackLeftOffsetX/Y/ZGameUnits`
- `fShoulderStashFallbackRadiusGameUnits`
- `bShoulderStashSkipActivateBooks`
- `bShoulderStashSkipActivateNotes`
- haptic keys listed above

Defaults must be chosen from ROCK/FO4VR measurements, not copied directly from
HIGGS meters or Skyrim units.

## Test Plan

Unit tests:

- `ShoulderStashMathTests.cpp`
  - point-to-capsule distance
  - enter/exit hysteresis
  - dwell confirmation
  - same-side bias without cross-side rejection
  - velocity gate
- `ShoulderStashEligibilityTests.cpp`
  - null held ref
  - deleted/disabled ref
  - player ref
  - actor and body targets rejected
  - actor equipment rejected until materialized as loose ref
  - playable base accepted
  - non-playable base rejected
  - book cant-take flag once verified
- `ShoulderStashTransferTests.cpp`
  - default stack count
  - verified `ExtraCount` read at `+0x18`
  - count clamp/fallback behavior
  - native transfer failure preserves normal release behavior
- State machine tests:
  - held body can enter stash candidate
  - stash candidate cancels back to held body
  - stash candidate commits to idle
  - held init cannot commit stash
  - same-frame begin plus commit path is explicit and tested

Integration/build tests:

- CMake Release build for ROCK.
- Existing provider boundary tests.
- Existing hand state machine tests.
- Existing body contact tests.

In-game test matrix:

- Single loose MISC item.
- Stacked loose item.
- Weapon.
- Ammo.
- Aid item.
- Book.
- Note/holotape.
- Quest item or non-playable item.
- Power armor and non-power-armor body profile.
- Leaning torso/HMD offset mismatch.
- Body collider unavailable during startup/fallback path.
- Same-side and cross-side shoulder reach.
- Fast throw near shoulder should drop/throw, not stash.

## Risk Register

- Misnamed F4VR count helper:
  - mitigated by Ghidra finding above and a ROCK-owned wrapper.
- Book cant-take flag unknown in FO4VR:
  - must verify before implementing the book exclusion.
- Native activation may open UI for books/notes:
  - implement bypass only with separate config and verification.
- Native activation may fail for playable but scripted objects:
  - on failure, drop normally and log; do not delete.
- Ref pointer lifetime after activation:
  - capture form id and metadata before activation; external consumers should
    rely on form id after `Stashed`.
- Collider staleness:
  - query must expose source/freshness and fall back to HMD only when needed.
- Accidental stash while throwing:
  - velocity gate plus dwell plus release-in-zone commit.
- Candidate flicker:
  - hysteresis, dwell, and source/zone tracking.

## Implementation Tracker

Investigation:

- [x] Map current ROCK hand lifecycle stash scaffolding.
- [x] Map current ROCK body-zone and generated shoulder collider scaffolding.
- [x] Map current ROCK release path that must be intercepted.
- [x] Map HIGGS stash eligibility and release behavior.
- [x] Verify FO4VR `TESObjectREFR::ActivateRef` in Ghidra.
- [x] Verify FO4VR `TESObjectREFR::AddInventoryItem` in Ghidra.
- [x] Verify `ExtraCount` layout enough to avoid the misnamed framework helper.
- [x] Verify FO4VR `TESObjectBOOK::data.flags` cant-be-taken bit.

Architecture:

- [x] Add shoulder stash math module.
- [x] Add body-zone shoulder query over generated colliders.
- [x] Add per-hand shoulder stash detector runtime.
- [x] Add HMD fallback path with explicit evidence source.
- [x] Add stash eligibility module.
- [x] Add stash transfer module.
- [x] Extend grab release context with release disposition.
- [x] Wire stash candidate update before normal held release.
- [x] Wire commit path on grip release in confirmed candidate.
- [x] Add stash events and haptic handling.
- [x] Add config fields and INI entries.

Verification:

- [ ] Add unit tests for math.
- [ ] Add unit tests for eligibility.
- [ ] Add unit tests for stack count helper.
- [x] Add source-boundary test for stash release, eligibility, transfer, config, and prod INI.
- [x] Run existing hand state-machine test.
- [x] Build ROCK Release locally.
- [x] Update production INI in place after implementation.
- [x] Deploy DLL/PDB only after local build succeeds.
- [ ] Run in-game test matrix and record results here or in a follow-up note.
