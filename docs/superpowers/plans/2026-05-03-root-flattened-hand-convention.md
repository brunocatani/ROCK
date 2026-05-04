# Root Flattened Hand Convention Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Normalize hand collision, grab frame, selection vectors, grab pivot fallback, and finger correction reads onto the same root flattened bone tree used by generated hand colliders.

**Architecture:** The root flattened tree is the single runtime hand-frame source. `DirectSkeletonBoneReader` remains the native boundary for reading `f4vr::getFlattenedBoneTree()` / `f4vr::getRootNode()`. `HandBoneCache` becomes a per-frame root-tree snapshot cache instead of a first-person `NiNode` pointer cache; callers consume stored world transforms and stop carrying first-person hand nodes as grab authority.

**Tech Stack:** C++23, CommonLibF4VR/F4VR-CommonFramework, FO4VR NetImmerse transforms, hknp physics bodies, PowerShell source-boundary tests, CMake Release build.

---

## Decision Note

This change is being done because generated hand colliders already use the root flattened bone tree, but grab frame/orientation and finger surface correction still read first-person skeleton nodes. That gives selection, pivot fallback, finger aim, and held-object target math a different source convention from the actual hand collider bodies. The alternatives were to move colliders back to first-person nodes or preserve the mixed model with more diagnostics. Both keep two authorities alive. The chosen approach makes the root flattened tree the single hand convention and keeps weapon colliders on their visual weapon root, because weapons are not hand bones and their generated hulls are authored from weapon mesh roots.

## File Map

- Modify: `src/physics-interaction/HandBoneCache.h`
  - Replace first-person skeleton pointer cache with per-frame root flattened hand transform cache backed by `DirectSkeletonBoneReader`.
- Modify: `src/physics-interaction/HandFrameResolver.h`
  - Resolve hand frames from cached root flattened transforms, with no `NiNode` authority.
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
  - Refresh hand frame cache every frame, return root flattened transforms for grab/selection/provider output, and make `handNode` diagnostic-only/null when the root tree is authoritative.
- Modify: `src/physics-interaction/PhysicsInteraction.h`
  - Expose a read-only root flattened hand transform accessor for public ROCK API calls.
- Rename/Modify: `src/physics-interaction/RootFlattenedFingerSkeletonRuntime.cpp`
  - Resolve finger landmark snapshots from the root flattened tree.
- Rename/Modify: `src/physics-interaction/RootFlattenedFingerSkeletonRuntime.h`
  - Carry root flattened naming for the finger snapshot and landmark helpers.
- Modify: `src/physics-interaction/GrabFingerLocalTransformRuntime.h`
  - Resolve live finger correction transforms from root flattened entries and their root-tree parent entries.
- Modify: `src/physics-interaction/TwoHandedGrip.cpp`
  - Resolve two-handed weapon hand frames and alternate thumb local-transform support from root flattened entries and parent entries.
- Modify: `src/physics-interaction/GrabFingerPoseRuntime.h`
  - Rename the internal solved-pose diagnostic flag so it does not claim first-person/FRIK finger-bone authority.
- Modify: `src/api/ROCKApi.cpp`
  - Publish palm position/forward helpers from the root flattened hand transform accessor.
- Modify: `src/RockConfig.cpp`, `src/RockConfig.h`, `data/config/ROCK.ini`
  - Rename root flattened finger debug settings while keeping old INI keys as fallback aliases.
- Create: `tests/RootFlattenedHandConventionSourceTests.ps1`
  - Source-boundary regression test that rejects first-person skeleton reads from the hand/grab/finger convention paths and requires root flattened reader usage.

## Tasks

### Task 1: Add Source-Boundary Test

**Files:**
- Create: `tests/RootFlattenedHandConventionSourceTests.ps1`

- [x] **Step 1: Write the failing source-boundary test**

The test requires `HandBoneCache`, `FrikFingerSkeletonRuntime`, and `GrabFingerLocalTransformRuntime` to use `DirectSkeletonBoneReader` and `DebugSkeletonBoneSource::FrikRootFlattenedBoneTree`, and rejects `getFirstPersonSkeleton()` / `getFirstPersonBoneTree()` in those convention paths.

- [x] **Step 2: Run the test and verify RED**

Run: `pwsh -NoProfile -ExecutionPolicy Bypass -File tests/RootFlattenedHandConventionSourceTests.ps1`

Observed before implementation: FAIL because current code still calls `f4vr::getFirstPersonSkeleton()` in `HandBoneCache.h`, `FrikFingerSkeletonRuntime.cpp`, and `GrabFingerLocalTransformRuntime.h`, and `PhysicsInteraction.cpp` still fetches `getNode(isLeft)`.

### Task 2: Convert Hand Frame Cache

**Files:**
- Modify: `src/physics-interaction/HandBoneCache.h`

- [x] **Step 1: Replace the pointer cache with a root-tree transform snapshot**

`resolve()` captures `HandsAndForearmsOnly` from `DebugSkeletonBoneSource::FrikRootFlattenedBoneTree`, finds `RArm_Hand` and `LArm_Hand`, stores their world transforms, source skeleton pointer, source bone tree pointer, and power-armor state, then logs `rootFlattenedTree`.

- [x] **Step 2: Make refresh per-frame**

Because transforms are copied out of the flattened tree instead of read live through cached `NiNode*`, `PhysicsInteraction::refreshHandBoneCache()` must call `resolve()` each frame.

### Task 3: Convert Grab Frame Resolution

**Files:**
- Modify: `src/physics-interaction/HandFrameResolver.h`
- Modify: `src/physics-interaction/PhysicsInteraction.cpp`
- Modify: `src/physics-interaction/PhysicsInteraction.h`
- Modify: `src/api/ROCKApi.cpp`
- Modify: `src/physics-interaction/PhysicsFrameContext.h` only if the node field must be removed

- [x] **Step 1: Resolve from cached transforms**

`getInteractionHandTransform()` returns the cached root flattened hand transform through `HandFrameResolver`.

- [x] **Step 2: Stop publishing first-person hand nodes as interaction authority**

`getInteractionHandNode()` returns `nullptr` when the root flattened cache is authoritative. The existing `HandFrameInput::handNode` field remains for API compatibility unless build errors show it should be removed.

- [x] **Step 3: Export provider hand transforms from the same cache**

`fillProviderFrameSnapshot()` now publishes `_handBoneCache.getWorldTransform(false/true)` instead of FRIK API hand transforms.

- [x] **Step 4: Disable hand frame input when the root cache is unavailable**

`buildFrameContext()` now marks a hand disabled if `_handBoneCache` does not have a valid root flattened snapshot, preventing identity fallback from entering selection/grab math.

- [x] **Step 5: Route public ROCK palm helpers through the same accessor**

`ROCKApi::getPalmPosition/getPalmForward` now read `PhysicsInteraction::tryGetRootFlattenedHandTransform()` instead of FRIK API hand transforms.

### Task 4: Convert Finger Runtime Reads

**Files:**
- Modify: `src/physics-interaction/RootFlattenedFingerSkeletonRuntime.cpp`
- Modify: `src/physics-interaction/RootFlattenedFingerSkeletonRuntime.h`
- Modify: `src/physics-interaction/GrabFingerLocalTransformRuntime.h`
- Modify: `src/physics-interaction/TwoHandedGrip.cpp`
- Modify: `src/physics-interaction/GrabFingerPoseRuntime.h`

- [x] **Step 1: Resolve finger landmarks from root flattened snapshot**

`resolveLiveFingerSkeletonSnapshot()` captures `HandsAndForearmsOnly` from `DirectSkeletonBoneReader` and reads `L/RArm_Hand` plus the 15 finger bones from snapshot entries.

- [x] **Step 2: Resolve local transform correction parents from root flattened parent indices**

`GrabFingerLocalTransformRuntime` builds a compact array of `{world, parentWorld}` transform pairs from snapshot `treeIndex` / `parentTreeIndex`, and uses those for world-to-local rotation conversion.

- [x] **Step 3: Resolve two-handed alternate thumb parents from root flattened parent indices**

`TwoHandedGrip` uses the same compact root flattened `{world, parentWorld, local}` thumb transforms for alternate-thumb support grip overrides.

- [x] **Step 4: Resolve two-handed weapon hand frames from root flattened hand bones**

`TwoHandedGrip::getHandBoneTransform()` now captures `L/RArm_Hand` from the root flattened tree instead of calling `FRIKApi::getHandWorldTransform()`.

- [x] **Step 5: Fail two-handed support when root flattened hand transforms are unavailable**

`TwoHandedGrip` now skips start or clears the active grip if the root flattened hand transforms cannot be sampled, instead of solving against an identity transform.

- [x] **Step 6: Rename finger skeleton runtime/debug labels to root flattened**

The finger snapshot runtime, namespace, debug marker roles, and shipped debug INI keys now use `RootFlattenedFingerSkeleton` naming. `RockConfig` still accepts the old `FrikFingerSkeleton` INI keys as fallback aliases.

### Task 5: Verify

**Files:**
- Build output: `build/Release/ROCK.dll`

- [x] **Step 1: Run source-boundary test GREEN**

Run: `pwsh -NoProfile -ExecutionPolicy Bypass -File tests/RootFlattenedHandConventionSourceTests.ps1`

Observed after implementation: PASS.

- [x] **Step 2: Run existing generated body source-boundary test**

Run: `pwsh -NoProfile -ExecutionPolicy Bypass -File tests/GeneratedBodyDriveSourceTests.ps1`

Observed: PASS.

- [x] **Step 3: Build Release**

Run: `$env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release`

Observed: build succeeded, emitted `build/Release/ROCK.dll`, and copied it to `D:/FO4/mods/ROCK/F4SE/Plugins/`.

- [x] **Step 4: Run focused compiled tests and source-boundary suite**

Observed: all Release policy executables built by `BUILD_ROCK_TESTS` exited cleanly, and every `tests/*SourceTests.ps1` source-boundary test passed.

### Task 6: Code Review

**Files:**
- All modified source files above

- [x] **Step 1: Review for remaining first-person skeleton convention leaks**

Run: `rg -n "getFirstPersonSkeleton|getFirstPersonBoneTree|first-person|firstPerson" src/physics-interaction -g "*.cpp" -g "*.h"`

Observed: no hand/grab/finger authority path uses first-person skeleton or FRIK API hand transforms. Remaining FRIK hand-transform reads are diagnostic parity/debug-overlay comparisons; weapon visual-root paths still use `f4vr::getWeaponNode()` because weapon generated bodies are mesh-rooted, not hand-bone-rooted.

- [x] **Step 2: Review transform-source consistency**

Check that `frame.rawHandWorld`, selection vectors, grab pivot fallback, dynamic pull, near grab, held-object update, and finger correction all consume root flattened hand/finger transforms.

Observed: `frame.rawHandWorld`, provider hand transforms, selection vectors, normal grab, dynamic pull, held-object update, finger landmark snapshots, finger local-transform correction, and two-handed support hand frames now all consume root flattened hand/finger transforms or disable/fail when the root flattened source is unavailable.
