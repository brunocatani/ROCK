# 2026-06-11 - Weapon Collider Reload Rebuild Gate

## Purpose

ROCK must stop treating reload animation visual churn as a reason to walk the equipped weapon NiNode tree or recreate generated weapon collision bodies. A reload should keep the current weapon collider set alive. Collider recreation should happen only for real equipped-weapon lifecycle changes: draw/equip/unsheath initial creation, actual equipped identity or mod-instance changes after workbench/equip, missing bodies for a drawn weapon, or explicit safety invalidations such as config, scale, world, or drive-failure rebuilds.

## Source And Confidence

- Project: ROCK.
- Branch observed before planning: `feature/ghidra-grab-motor-mapping`.
- Source used: current local ROCK source and tests only.
- External sources: none.
- Ghidra / FO4 Mods MCP: not used and not needed for this change.
- Confidence: medium-high for the source-level failure mode. Runtime validation is still required because FO4VR reload visual timing can only be fully confirmed in game.

## Current Failure Surfaces

The initial fix idea, "make the gates identity-based," is directionally right but not enough unless all hot paths below are covered.

1. `WeaponCollision::update` calls `getEquippedWeaponKey(weaponNode, ...)` every frame before deciding whether rebuild is required.
   - `getEquippedWeaponKey` currently computes a visual composition key by calling `makeGeneratedWeaponMeshRootCandidates` and `accumulateWeaponVisualKey`.
   - That is a NiNode tree walk in the normal update path, even when no rebuild happens.
   - Reload animation can change visible child nodes or transient reload geometry and therefore change the key or force work before the gate has actually decided a rebuild is allowed.

2. `observedKey` currently mixes equipped identity with visual composition.
   - `keyChanged` becomes true for visual-only changes.
   - That makes reload-only tree churn look like a real weapon generation change.
   - Once `keyChanged` is true, the code enters visual stabilization, source scanning, and staged collider creation.

3. `WeaponCollision::update` destroys generated bodies immediately when `weaponNode == nullptr`.
   - If reload temporarily hides or detaches the first-person weapon node while the weapon is still drawn and the equipped identity is unchanged, this causes a destroy/recreate cycle.
   - That must become a retain/skip-frame case, not a clear-current-state case.

4. Pending creation is matched by `pendingGeneratedWeaponBuildMatches(observedKey, observedVisualKey)`.
   - If a real equip/workbench rebuild is already staged, a visual-only reload witness change could cancel the staged create.
   - Pending-create cancellation should be keyed to equipped identity and explicit invalidations, not reload visual witness churn.

5. `GeneratedSourceCache` currently matches on both equipped key and visual key.
   - For actual rebuilds, visual/source cache is still useful.
   - For unchanged identity, cache matching should not be consulted at all because source extraction should not be attempted.

6. `maybeDumpWeaponAnimNodeDiagnostics` is debug-only but currently observes generation changes.
   - After the lifecycle key becomes identity-only, generation-change dumps should no longer fire for reload visual-only changes.
   - Interval dumps may still walk debug roots when the debug option is enabled; that is acceptable because it is explicitly diagnostic.

## Required Design

The weapon collider lifecycle must be split into two layers:

1. Cheap equipped identity layer.
   - Read `equipData`, `instanceData`, and object-instance-extra/mod data.
   - Do not inspect weapon NiNode children.
   - This identity is the authority for whether the equipped weapon changed.

2. Expensive visual/source layer.
   - Walk weapon visual roots and extract generated hull sources only after an allowed rebuild reason is already established.
   - Allowed rebuild reasons are:
     - initial creation while weapon is drawn and bodies are missing;
     - equipped identity changed;
     - weapon collision settings changed;
     - generated body drive explicitly requested rebuild;
     - world/scale/config shutdown invalidations already handled by existing paths.

Reload-only visual changes are not allowed rebuild reasons.

## Implementation Plan

1. Refactor identity helpers.
   - Introduce an identity-only generation key helper, or change the existing collider lifecycle key so it no longer mixes `visualCompositionKey`.
   - Keep visual composition as a separate rebuild-time witness only.
   - Update policy tests so generation key changes with equipped instance/mod identity, but not with visual witness changes.

2. Change `WeaponCollision::update` signature and call site.
   - Pass `runtime.weaponDrawn` from `PhysicsInteraction::update`.
   - In `WeaponCollision::update`, read equipped identity first without requiring `weaponNode`.
   - If there is no drawn weapon or no equipped weapon identity, destroy/clear existing bodies as the holster/no-weapon path.

3. Add an unchanged-identity fast path.
   - If current bodies exist, settings did not change, drive rebuild is not requested, and identity is unchanged:
     - do not call visual-key traversal;
     - do not call `findGeneratedWeaponShapeSources`;
     - do not touch pending build state except to advance a matching pending build if one is already active for the same identity;
     - retain current bodies and sync dominant-hand collision state.

4. Handle transient missing visuals safely.
   - If `weaponDrawn == true`, identity is unchanged, bodies exist, and `weaponNode == nullptr`, keep current bodies and return after sync.
   - Do not destroy bodies or clear cached identity in this case.
   - Transform updates already require a live `weaponNode`, so the bodies simply keep their last queued target until the visual returns.

5. Gate visual witness and source extraction behind allowed rebuild.
   - Only once rebuild is allowed and `weaponNode` is non-null, compute visual witness stats.
   - Keep visual stabilization before full source extraction for real rebuilds.
   - Keep `findGeneratedWeaponShapeSources` only on this rebuild path.

6. Update staged build matching.
   - Pending build should be cancelled by identity/settings/drive/world invalidation, not by visual witness changes alone.
   - Visual witness may remain stored for logs/cache, but it must not cancel a valid same-identity staged creation.

7. Keep safety invalidations intact.
   - Do not weaken config disable, hknp world change, scale change, no equipped identity, no drawn weapon, or generated drive failure handling.
   - Do not add hidden fallback behavior or native visual remap hooks.

## Test Plan

Run these after implementation:

```bat
git status --short --branch
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
git status --short --branch
```

Add or update tests:

- `WeaponCollisionLifecyclePolicyTests.cpp`
  - identity-only generation key does not change when only visual witness changes;
  - identity key changes when equipped instance/mod data changes;
  - body-set publication key still reflects actual generated source set after an allowed rebuild.

- `WeaponCollisionLifecycleSourceTests.ps1`
  - normal unchanged-identity update path must not call visual tree traversal/source extraction;
  - `findGeneratedWeaponShapeSources` remains behind rebuild-required gates;
  - transient `weaponNode == nullptr` with drawn unchanged identity must not destroy bodies;
  - removed native remap/witness plumbing must remain absent.

## Runtime Validation Needed

After build/deploy, validate in game with logging/profiler:

- Draw/equip weapon: one collider creation is expected.
- Reload repeatedly: no `WeaponColliderBuild`, no `WeaponColliderCreate`, no staged create queued, no weapon body destroy/recreate.
- Enter and exit workbench without changing weapon mods: no recreate expected unless the game genuinely changes equipped instance data.
- Exit workbench after changing weapon mods: one rebuild is expected after identity changes and visual source stabilizes.
- Holster weapon: bodies may be destroyed/cleared through the no-drawn-weapon path.

## Risks

- If FO4VR workbench changes only visual nodes without updating equipped instance/object-extra identity, this design will intentionally not rebuild. That would require a separate approved source of workbench-exit truth or a local runtime event hook, not visual churn guessing.
- If reload temporarily changes `equipData` or object-instance-extra in a way that looks like a real mod change, identity may still rebuild. Runtime logs should confirm whether that happens.
- Retaining bodies during transient missing visuals means collision bodies may hold the last transform briefly. This is preferable to destroy/recreate churn, but runtime validation should check that no stale collider persists after actual holster or weapon removal.

## Decision

The implementation should not rely on visual composition to decide whether a weapon changed. Visual composition is evidence for building bodies after an allowed lifecycle transition, not the lifecycle authority itself. Equipped identity is the gate; visual source extraction is the work behind the gate.

## Implementation Result

Implemented on 2026-06-11.

- `WeaponCollision::update` now receives drawn/holstered state from `PhysicsInteraction`.
- Equipped identity is read before any visual witness work.
- Reload-time missing visuals with unchanged equipped identity retain the current generated bodies instead of destroying them.
- Visual composition and generated source extraction are now rebuild-time work only.
- Pending generated weapon creation is matched against equipped identity and creation settings, not reload visual witnesses.
- Policy/source tests were updated to lock the identity-only lifecycle gate and missing-visual retain behavior.

Validation run:

```bat
powershell -ExecutionPolicy Bypass -File tests\WeaponCollisionLifecycleSourceTests.ps1
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m
```

All tests passed locally. The `custom-fast` Release build copied `ROCK.dll` and `ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.

## Review Follow-Up

Implemented after code review on 2026-06-11:

- Equipped weapon lifecycle keys no longer mix raw runtime pointer addresses such as equip-data, instance-data, object-extra, or visual-object addresses. Those values can churn during reload/runtime wrapper updates and must not open the collider rebuild gate.
- `PhysicsInteraction` now treats retained generated weapon bodies as active right-hand weapon authority even when `resolveEquippedWeaponInteractionNode()` returns null. This keeps dominant-hand collision suppression, contact evidence ownership, and soft-contact ownership aligned with the retained weapon collider set during reload-null visual frames.
- `WeaponCollision` no longer acquires/releases `WeaponDominantHand` suppression directly. `PhysicsInteraction` owns suppression for the complete generated hand-collider set; keeping a second single-body owner path risks releasing the shared suppression owner while retained weapon bodies are still live.

Still blocked:

- A true workbench-exit rebuild gate is not implemented here because current local source does not expose an authoritative workbench-exit signal. PAPER should not own this ROCK collider lifecycle. Finding a reliable signal likely requires approved FO4VR binary investigation in Ghidra, scoped to workbench/menu exit or weapon-mod apply events.
