# Forced First-Person Weapon Visual Refresh Plan

## Why This Approach

ROCK currently builds generated weapon collision from the first-person weapon visual tree, ultimately reached through `firstPersonSkeleton:Weapon`. That is the correct drive root for colliders, but in FO4VR the first-person weapon graph can stay one weapon-mod transaction behind: the equipped instance changes, while the biped weapon slot still looks valid to native attach code because the weapon form pointer, instance-data pointer, and existing slot node did not change. Calling `QueueAttachWeapon` by itself can therefore enter native attach and still do nothing. The chosen fix is to trigger Bethesda's own first-person weapon slot clear through `QueueRemoveWeapon`, then queue `QueueAttachWeapon` for the same live equipped weapon instance. This forces the native slot to rebuild without inventing alternate collider roots, without using OMOD data as geometry, and without broad equip or animation refresh calls.

## Problem Statement

Observed behavior:

- Some weapon parts are missing from `firstPersonSkeleton:Weapon` when ROCK scans the visual tree for collision geometry.
- If another weapon part is changed later in the workbench, the previously missing part appears and the newest part becomes missing.
- This behaves like the first-person biped weapon visual is one mutation behind.
- Enrichment cannot solve this because enrichment only finds late geometry after the first-person tree publishes it.
- The native graph does not catch up automatically in the tested cases.
- Current `QueueAttachWeapon` remap does not reliably fix the graph because native attach can decide the weapon is already attached.

Important distinction:

- The missing part is not merely absent from ROCK classification.
- The missing part is absent from the first-person graph ROCK is reading.
- The fix must make FO4VR republish the correct first-person weapon tree before ROCK generates or replaces colliders.

## Verified Native Findings

All addresses below were mapped against the local unpacked FO4VR binary and local address-library data. These are documentation findings for the later implementation pass; re-check in Ghidra before changing native boundaries if the binary, CommonLibF4VR, or address library changes.

### Current Read Path

`f4vr::getWeaponNode()` in `libraries_and_tools/F4VR-CommonFramework/src/f4vr/PlayerNodes.h` returns:

```cpp
return findNode(getPlayer()->firstPersonSkeleton, "Weapon");
```

This is a read-only graph lookup. It does not build, refresh, attach, or force missing weapon parts to publish. Calling it manually without CommonFramework does not change behavior because the issue is the native graph state behind the `"Weapon"` node, not the wrapper itself.

### QueueAttachWeapon

Address-library entry:

- `REL::ID(916430)`
- FO4VR address: `0x140DAB8F0`
- Label: `TaskQueueInterface::QueueAttachWeapon(Actor*, BGSObjectInstanceT<TESObjectWEAP>&, BGSEquipIndex)`

Native task behavior:

- Queues task id `0x12`.
- The task processor constructs a temporary `BGSObjectInstance`.
- It calls virtual `A5` / `AttachWeapon` at vtable offset `0x528`.

Important attach path:

- `TESObjectREFR::AttachWeapon` reaches native attach helper `FUN_1401C8150`.
- `FUN_1401C8150` checks the biped weapon slot cache before rebuilding.
- Rebuild can be skipped when:
  - cached form pointer still matches,
  - cached instance-data pointer still matches,
  - cached slot node exists.

This explains the failed remap. Workbench edits can mutate the already-equipped object-instance extra/mod stack in place while form and instance-data pointer identity remain unchanged. The native attach path can consider the slot already valid even though one visual part is stale.

### Internal Slot Clear

Native helper:

- `FUN_1401C5FB0`

Observed behavior:

- Clears/releases the current biped slot node and related references.
- Zeros the stored `BGSObjectInstance` fields for the slot.
- Is the lower-level operation that makes a later attach rebuild possible.

Rejected direct approach:

- Calling `FUN_1401C5FB0` directly from ROCK would be more surgical, but it would require ROCK to own a private native address, slot pointer layout, and thread/lifetime assumptions.
- It should remain a fallback research option only if the queued remove route is proven insufficient in-game.

### QueueRemoveWeapon

Address-library entry:

- `REL::ID(24792)`
- FO4VR address: `0x140DAB9F0`
- Label: `TaskQueueInterface::QueueRemoveWeapon(Actor*, TESObjectWEAP*, BGSEquipIndex)`

Native task behavior:

- Queues task id `0x13`.
- The task processor constructs a form-only `BGSObjectInstance`.
- It calls virtual `A6` / `RemoveWeapon` with `queue3DTasks = false`.

### PlayerCharacter::RemoveWeapon

Player virtual `A6` entry:

- Player vtable base from CommonLibF4VR: `VTABLE::PlayerCharacter[0] = 0x142D80F88`
- A6 entry address: `0x142D814B8`
- Dispatch target: `0x140F0B340`

Observed behavior:

- Calls `GetBiped(true)` for the first-person biped.
- Maps weapon/equip index through `FUN_1401CAFC0`.
- Calls `FUN_1401C84F0` on the first-person biped slot.
- Calls `Actor::RemoveWeapon` after the first-person clear.

### FUN_1401C84F0

Observed behavior:

- Locates the biped object slot by equip index.
- Calls `FUN_1401C5FB0` for matching/cached slot entries.
- Clears the exact cached state that allows `QueueAttachWeapon` to no-op.

Conclusion:

`QueueRemoveWeapon` followed by `QueueAttachWeapon` is the correct first implementation target because it reaches the verified slot clear through Bethesda's task queue and public task-style boundary, then rebuilds from the current equipped instance.

## Design Goals

- Keep generated collision driven only by the real first-person weapon visual tree.
- Never use OMOD/index data as direct geometry.
- Never use alternate roots as collider drive roots.
- Never keep duplicate collider banks alive for the same weapon.
- Avoid broad native calls:
  - no `HandleItemEquip`,
  - no `SetEquippedItem`,
  - no `RequestLoadAnimationsForWeaponChange`,
  - no `QueueUpdate3D`,
  - no `QueueShow1stPerson`,
  - no first-person arm refresh as a collider recovery path.
- Bound all native refresh attempts so a broken graph cannot produce per-frame task spam.
- Preserve deployed INI compatibility.
- Make the native boundary auditable by source-boundary tests.

## Non-Goals

- Do not implement visual-tree enrichment as the main fix.
- Do not read the weapon part list from OMOD/index data to synthesize colliders.
- Do not attach colliders to `primaryWeapontoWeaponNode`, offset nodes, or other fallback roots when `Weapon` is missing.
- Do not clear all actor 3D or reload animation state.
- Do not depend on HIGGS behavior for this fix.

## Proposed Architecture

### Native Boundary

Move the raw native task calls behind a named native helper under `src/physics-interaction/native/`, because this is a Bethesda native task boundary.

Suggested helper ownership:

- `src/physics-interaction/native/WeaponVisualRefreshNative.h`
- `src/physics-interaction/native/WeaponVisualRefreshNative.cpp`

Suggested domain-facing API:

```cpp
namespace rock::weapon_visual_refresh_native
{
    enum class RefreshTaskResult : std::uint8_t
    {
        Queued,
        MissingTaskQueue,
        InvalidActor,
        InvalidWeapon,
    };

    struct RefreshTaskInput
    {
        RE::Actor& actor;
        RE::TESObjectWEAP& weapon;
        RE::BGSObjectInstanceT<RE::TESObjectWEAP>& weaponInstance;
        RE::BGSEquipIndex equipIndex;
    };

    [[nodiscard]] RefreshTaskResult queueForcedFirstPersonWeaponRefresh(const RefreshTaskInput& input);
}
```

Implementation rule:

- `queueForcedFirstPersonWeaponRefresh` queues `QueueRemoveWeapon` first, then `QueueAttachWeapon`.
- `QueueRemoveWeapon` receives the weapon form pointer and equip index.
- `QueueAttachWeapon` receives the same live matched borrowed `BGSObjectInstanceT` currently used by the remap runtime.
- The helper does not inspect visual nodes and does not make collision decisions.

### Domain Runtime

Replace the current semantic meaning of `WeaponVisualRemapRuntime` with forced visual refresh semantics.

Two acceptable implementation options:

- Rename files/symbols to `WeaponVisualRefreshRuntime` if cleanup can be done coherently in the same commit.
- Keep file names temporarily only if source-boundary/test churn is too large, but update all user-facing logs/comments from "remap" to "refresh".

Preferred cleanup:

- Rename runtime namespace to `weapon_visual_refresh_runtime`.
- Rename request API to `requestCurrentFirstPersonWeaponVisualRefresh`.
- Rename result/log strings from `nativeRemap...` to `nativeRefresh...`.
- Keep the production INI key `bWeaponCollisionNativeVisualRemapEnabled` as a compatibility alias, but internally map it to refresh behavior.

The runtime keeps the current strict target matching:

- pending instance signature must be nonzero,
- current equipped weapon must exist,
- form ID/address must match if known,
- instance-data address must match if known,
- object-instance-extra address must match if known,
- equip index must be live and compatible.

The runtime then calls the native helper once a refresh is authorized.

### Witness Policy

Current witness sources:

- `GameLoadBaseline`
- `InventoryEquip`
- `AttachMod`
- `WorkbenchApplyChanges`

Refresh authorization should become more precise than the current workbench-only remap:

- `WorkbenchApplyChanges`: authorized after stale or missing visual evidence because this is the proven post-mutation stack writer path.
- `GameLoadBaseline`: authorized only for initial generated collider bootstrap when a weapon is equipped and the visual graph is missing/stale after grace.
- `InventoryEquip`: authorized only for initial equip/spawn/bootstrap, not for repeated normal Pip-Boy transition frames.
- `AttachMod`: not sufficient by itself; remains secondary evidence because it may not prove the final equipped stack has settled.

Policy should distinguish request reasons:

- `workbenchStaleVisibleSource`
- `workbenchMissingVisualAfterGrace`
- `initialEquipMissingVisualAfterGrace`
- `initialEquipStaleVisibleSource`

Do not let "missing visual root" immediately trigger refresh on every frame. Missing root must pass the existing grace/stability window first.

### Collision Lifecycle

Generated collision remains blocked until a settled valid visual source exists.

Expected behavior by state:

- No equipped weapon:
  - no native refresh,
  - destroy/disable generated weapon colliders according to existing no-weapon lifecycle.
- Equipped weapon but missing `Weapon` root during grace:
  - no collider creation,
  - no refresh until grace expires.
- Equipped weapon and missing `Weapon` root after grace:
  - queue bounded forced refresh if an authorized live equipped witness exists,
  - keep collider bodies disabled/unpublished because there is no drive root.
- Equipped weapon and stale visible source:
  - keep old valid colliders active if they have a safe drive root,
  - queue bounded forced refresh,
  - replace colliders only after the visual source changes and settles.
- Equipped weapon and new settled source:
  - build replacement collider bank,
  - publish atomic body IDs before enabling collision,
  - disable/destroy old bank through existing lifecycle,
  - clear pending witness and refresh attempt state.

Important invariant:

- A forced native refresh request never directly publishes colliders.
- It only asks FO4VR to republish the first-person visual tree.
- ROCK still requires the visual tree to change/settle before collider replacement.

## Implementation Stages

### Stage 1: Rename and Policy Cleanup

Purpose:

- Stop treating this as "remap" and model it as a native visual refresh task lifecycle.

Work:

- Introduce `WeaponVisualRefreshRuntime` names and request/result enums.
- Keep compatibility with `bWeaponCollisionNativeVisualRemapEnabled`.
- Rename attempt state from `NativeVisualRemapAttemptState` to `NativeVisualRefreshAttemptState`.
- Rename policy function from `evaluateNativeVisualRemap` to `evaluateNativeVisualRefresh`.
- Update logs to include:
  - pending signature,
  - refresh reason,
  - queued attempt count,
  - stale frames after queue,
  - form ID/address,
  - instance-data address,
  - object-instance-extra address,
  - equip index,
  - witness source,
  - result.

Cleanup:

- Remove stale "remap" docs/comments unless they are explicitly describing the legacy broken behavior in this plan.
- Do not leave both remap and refresh paths active.

### Stage 2: Native Helper

Purpose:

- Keep raw native task calls outside the weapon domain.

Work:

- Add native helper for forced first-person weapon refresh.
- Resolve:
  - `QueueRemoveWeapon` with `REL::ID(24792)`.
  - `QueueAttachWeapon` with `REL::ID(916430)`.
- Queue remove then attach for the same matched actor/weapon/equip index.
- Return structured result to the weapon runtime.

Source-boundary expectations:

- Raw `QueueRemoveWeapon` and `QueueAttachWeapon` symbols appear only in the native helper.
- Weapon domain calls only the named helper.

### Stage 3: Runtime Request Flow

Purpose:

- Use live equipped state and current witness evidence to queue the native refresh only when it can target the correct weapon.

Work:

- Reuse existing equipped snapshot collection:
  - `middleHigh->equippedItems`,
  - first-person equip data fallback,
  - actor equip data fallback,
  - authoritative equipped weapon witness fallback.
- Preserve strict target matching.
- Build the borrowed two-pointer weapon instance only after target match succeeds.
- Call native helper.
- Record result in bounded attempt state.

Failure handling:

- Missing player: no refresh.
- Missing current process/middle high: acquisition failure with backoff.
- Missing live equipped weapon: acquisition failure with backoff.
- Mismatched equipped weapon: no refresh for this frame; do not queue against the wrong target.
- Missing task queue: acquisition failure with backoff.
- Queued but visual still stale: observe for a small fixed frame window, then retry up to max attempts.

### Stage 4: Missing-Root Refresh Eligibility

Purpose:

- Fix weapons where `firstPersonSkeleton:Weapon` never appears or appears incomplete after the normal settle window.

Work:

- Add missing-root native refresh policy separate from stale-visible policy.
- Evaluate only after existing missing visual grace expires.
- Require:
  - equipped weapon witness exists,
  - live equipped candidate matches witness,
  - no current menu/equip transition guard blocks intervention,
  - refresh not exhausted for that pending witness.
- Do not create colliders while root is missing.
- Log missing-root refresh attempts distinctly from stale-visible refresh.

Suggested reasons:

- `missingVisualAfterGraceWorkbench`
- `missingVisualAfterGraceInitialEquip`
- `missingVisualRefreshAwaitingLiveEquip`
- `missingVisualRefreshExhaustedForInstance`

### Stage 5: Stale-Visible Refresh Eligibility

Purpose:

- Fix the "one part behind" case when the `Weapon` root exists but the source set still matches old geometry.

Work:

- Keep current stale-source detection:
  - equipped instance changed,
  - same-form remap witness changed,
  - owner identity changed,
  - visual source still matches cached source or is incomplete.
- Replace current `QueueAttachWeapon` request with forced remove+attach refresh.
- Keep old collider bank active only when it has a valid drive root.
- Never regenerate from unchanged stale geometry.

### Stage 6: Publication and Cleanup

Purpose:

- Prevent the duplicate-collider failure that happened before.

Work:

- Ensure only one active generated weapon body bank is published at a time.
- On missing root, call the explicit disable/unpublish lifecycle.
- On valid replacement, publish replacement metadata before enabling collision.
- On no equipped weapon, clear pending refresh state and unpublish generated weapon bodies.
- On settled current source, clear authoritative witness and refresh attempt state.

## Test Plan

### C++ Policy Tests

Add or update policy tests for:

- Workbench witness plus stale visible source requests forced refresh.
- Workbench witness plus missing root after grace requests forced refresh.
- Missing root during grace does not request refresh.
- No equipped weapon never requests refresh.
- Mismatched live equipped weapon never queues refresh.
- Same pending witness retries only after the observation interval.
- Refresh attempts stop after the configured max attempts.
- Initial equip/bootstrap can request one bounded refresh after grace.
- `AttachMod` alone does not authorize refresh.

### Source-Boundary Tests

Update `tests/WeaponCollisionLifecycleSourceTests.ps1` or add a focused native-boundary test:

- Reject `QueueRemoveWeapon` outside `src/physics-interaction/native/WeaponVisualRefreshNative.cpp`.
- Reject `QueueAttachWeapon` outside the same native helper.
- Reject broad native calls in weapon domain:
  - `HandleItemEquip`
  - `SetEquippedItem`
  - `RequestLoadAnimationsForWeaponChange`
  - `QueueUpdate3D`
  - `QueueShow1stPerson`
  - `Update1StPersonArm`
- Require weapon domain to call `queueForcedFirstPersonWeaponRefresh` rather than raw native task names.
- Require docs/log wording to use refresh semantics.

### Manual In-Game Scenarios

Use sampled logs and visual/collider behavior for:

- No weapon equipped:
  - no refresh queued,
  - no live generated weapon colliders.
- Fresh weapon equip:
  - no collider publication until `Weapon` root exists and settles.
- AMCAR workbench barrel/handguard change:
  - witness captured after workbench stack write,
  - stale source detected,
  - remove+attach refresh queued,
  - visual source changes,
  - exactly one collider bank remains active.
- Weapon with missing full `Weapon` root after equip/spawn:
  - missing root grace expires,
  - refresh queued once or bounded retry,
  - colliders stay unpublished until root returns.
- Repeated failed refresh:
  - no per-frame native task spam,
  - logs show exhausted state.

### Build/Test Commands

Run from `ROCK/`:

```powershell
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j $env:NUMBER_OF_PROCESSORS
ctest --test-dir build-tests -C Release --output-on-failure -j $env:NUMBER_OF_PROCESSORS
```

For fast deployed validation:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m
```

## Acceptance Criteria

- Changing a weapon part at the workbench no longer leaves the first-person collision source one mod behind.
- The missing AMCAR barrel/handguard case produces one correct generated collider set after refresh.
- No second collider set remains alive in the wrong orientation.
- With no weapon equipped, generated weapon colliders are not active.
- The system never builds colliders from OMOD/index data alone.
- The system never drives colliders from an alternate fallback root when `Weapon` is missing.
- Refresh attempts are bounded and visible in logs.
- Source-boundary tests enforce native helper ownership.

## Rollback

Rollback is the focused commit that introduces this refresh architecture. Do not keep the old `QueueAttachWeapon`-only remap path as a fallback in production code. If the queued remove+attach trigger fails in-game, revert the commit and map the direct biped slot clear path as a separate verified design.
