# FRIK Runtime Detach Tracking - 2026-05-23

Project: ROCK
Source authority: local ROCK source, local hFRIK source, local F4VR-CommonFramework source, local builds/tests.
Confidence: implemented and locally validated.

## Goal

Reduce ROCK's runtime dependence on hFRIK for state ROCK can own independently. Keep hFRIK as the visual body/hand pose publication bridge only.

## Keep On FRIK For Now

- Visible hand/body pose publication.
- External hand world transform tags.
- Finger pose and local-transform publication.
- FRIK hand-pose local-transform baseline until ROCK owns a full visual hand authority stack.
- Offhand weapon-grip suppression tags.

## Detach Checklist

- [x] ROCK-owned frame delta source instead of `FRIKApi::getFrameTime()`.
- [x] ROCK-owned weapon drawn state instead of `FRIKApi::isWeaponDrawn()`.
- [x] ROCK-owned player-space tracker instead of `FRIKApi::getSmoothedPlayerPosition()` / `isPlayerMoving()`.
- [x] ROCK-owned skeleton readiness gate using root node, attached parent, flattened tree, required hand/body bones, and stable physics world.
- [x] ROCK-owned menu/input gate, with FRIK config/Pipboy treated as compatibility-only when available.
- [x] Centralize remaining FRIK visual calls behind a named bridge/helper so runtime logic does not scatter direct FRIK access.
- [x] Add focused policy tests for the new ROCK-owned state logic.
- [x] Run `custom-tests`, full `ctest`, and `custom-fast` deploy build.

## Runtime Invariants

- FRIK lifecycle messages are hints, not authority for creating or continuing ROCK physics.
- ROCK physics creation and writes require ROCK-local readiness checks.
- Missing FRIK API must fail closed for visual publishing, but must not corrupt ROCK-owned runtime state.
- ROCK-owned frame/player/menu/weapon state must be sampled once per frame and copied into consumers; hot paths must not repeatedly query cross-plugin state.

## Notes While Implementing

- Current hard FRIK state queries were found in `ROCKMain.cpp`, `PhysicsInteraction.cpp`, `PhysicsInteractionFrame.inl`, and `PhysicsInteractionProvider.inl`.
- Existing root-flattened bone readers already give ROCK an independent skeleton source for hand/body collider readiness.
- F4VR-CommonFramework has local `f4vr::IsWeaponDrawn()` and `GameMenusHandler`; use those before adding new engine assumptions.

## Implementation Result

- Added `RockRuntimeState` and `RockRuntimeStatePolicy` as the per-frame ROCK-owned state sampler/policy layer.
- Added `FrikVisualAuthorityBridge` as the only direct FRIK visual authority boundary under `src/physics-interaction`.
- Updated creation/update gates to use ROCK-local skeleton, menu, frame-time, weapon, and player-space state.
- Left hard visual authority on FRIK: visible hand pose publication, local transform publication, external hand transforms, baseline local transforms, and offhand grip suppression.
- `PhysicsInteractionProvider` still exposes the legacy field name `frikSkeletonReady` for API compatibility, but now fills it from ROCK-local skeleton readiness.

## Validation

- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release --output-on-failure -j $env:NUMBER_OF_PROCESSORS` passed 49/49.
- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m` succeeded and auto-deployed `ROCK.dll` / `ROCK.pdb`.

## Remaining Runtime Check

- In-game cell-change and teleport validation still needs to confirm ROCK now fails closed through skeleton/provider resets without depending on stale FRIK readiness state.
- Current checkout also has unrelated weapon anim-node diagnostic changes in progress; keep those out of this runtime-detach commit unless deliberately reviewed as part of a separate task.

## Startup Crash Follow-Up

2026-05-23 startup CTD after the initial detach fix showed ROCK calling `f4cf::f4vr::getWorldRootNode()` before FO4VR had populated the player's root scene graph:

```text
EXCEPTION_ACCESS_VIOLATION at ROCK.dll f4cf::f4vr::getWorldRootNode
```

The runtime sampler now treats missing player/root graph as a closed skeleton-readiness state and does not call unsafe F4VR player-node helpers until the required root graph exists. `DirectSkeletonBoneReader` also resolves the root flattened tree through guarded local checks, and the native player collision scan no longer calls the unsafe `getRootNode()` helper directly.

## Startup Readiness Follow-Up

2026-05-23 runtime logs showed ROCK loading successfully but never creating physics after `kSkeletonReady`:

```text
Direct skeleton bone cache rebuilt: ... required=61/62 mode=CoreBodyAndFingers
Direct skeleton bone cache missing required bones:
```

The root cause was local ROCK code, not FRIK API readiness. `kRequiredCoreBoneNames` declared 32 entries but only provided 31 names, so the table contained one empty required `std::string_view`. That made local skeleton readiness fail on a non-existent empty bone.

The readiness gate now samples the root-flattened hands/fingers path for global skeleton readiness and leaves body-collider coverage to `BodyBoneColliderSet`, which already samples `AllFlattenedBones` and can skip missing body descriptors. Required skeleton-name tables have compile-time non-empty assertions and a focused policy test so this exact failure cannot silently return.

## Body Collider Lifecycle Isolation

Follow-up source review found one remaining over-coupling after the readiness fix: `generatedBodiesExistForConfig()` still treated body bone colliders as part of the core generated-body lifecycle when `bBodyBoneCollidersEnabled=true`, and lifecycle rebuild returned failure if body collider creation failed.

That meant a partial or unavailable body skeleton could still close ROCK's physics-write gate even when both hand collision bodies were valid. The core lifecycle now tracks hand-generated bodies only. Body bone colliders remain enabled, but creation failures only schedule the body retry path and do not invalidate hand physics.
