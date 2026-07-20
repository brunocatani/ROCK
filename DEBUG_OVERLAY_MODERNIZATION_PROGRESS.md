# ROCK Debug Overlay Modernization Ledger

## Purpose

This file is the persistent implementation ledger for bringing the safe, productionized renderer and performance work from `F:\fo4dev\devtools\CollisionVisualizerF4VR` into ROCK's debug visualizer.

It exists so the work can resume correctly after context compaction. Update this file whenever a slice starts, materially changes, is validated, is committed, or is blocked. Do not rely on chat history as the source of progress state.

The goal is not to copy the standalone visualizer. The goal is to make ROCK's explicit diagnostic overlay safe, bounded, low-overhead, and maintainable without losing any existing feature, body role, transform convention, visual fidelity, or diagnostic coverage.

## Resume Protocol

After any context compaction or interrupted session:

1. Read this entire file.
2. Read `F:\fo4dev\PROJECT_ROCK_V2\AGENTS.md` and the applicable FO4VR skill instructions.
3. Run `git status --short --branch` in `ROCK\`.
4. Reconcile the live tree with the `Current Progress` and `Commit Ledger` sections below.
5. Preserve all unrelated user/concurrent changes; never stage them into overlay commits.
6. Continue the first unchecked implementation slice only after its prerequisites are complete.
7. Use the required `custom-fast` configure preset followed by the capped Release build for every normal plugin build:
   - `cmake --preset custom-fast`
   - `cmake --build build-fast --config Release --target ROCK -- /m:1 /p:CL_MPCount=2`
8. Cap every CTest run at `-j 4`.
9. Commit each complete, validated slice and run post-commit regression checks before advancing.

## Source Authority

1. The user's current instructions.
2. Current ROCK source and tests on the checked-out branch.
3. Current ROCK build output, runtime logs, and active configuration.
4. Current `hFRIK\` source for provider behavior when relevant.
5. Approved FO4VR binary verification when a layout, offset, or callsite requires it.
6. `CollisionVisualizerF4VR` only as a local implementation reference for renderer architecture and performance patterns.

Do not copy standalone raw Havok offsets or assume that a standalone layout is authoritative for ROCK. Do not use HIGGS. Do not use the web.

## Repository Baseline

- Target repo: `F:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Starting branch: `feature/ghidra-grab-motor-mapping`
- Audit source revision: ROCK `5215de3`, later advanced concurrently to `5634ab2` before implementation began.
- Standalone reference revision: `F:\fo4dev\devtools\CollisionVisualizerF4VR` at `596a1da` on clean `master`.
- The ROCK tree contained unrelated active weapon/scope work when implementation began. Overlay commits must contain only overlay files, tests, configuration changes required by the overlay, and this ledger.

## Non-Negotiable Preserved Invariants

- Keep ROCK's current-frame stereo matrices and guarded camera reads.
- Preserve two-eye rendering from the left-eye submitted texture.
- Preserve every existing body role, color, axis, marker, skeleton, and text diagnostic.
- Preserve the Target role's body-array transform convention.
- Preserve live-motion transforms for roles that currently use live motion when available.
- Capture transforms and live diagnostic values every game frame. Do not introduce distance LOD, movement-threshold refresh, or multi-frame transform intervals.
- Do not permanently replace detailed shapes with proxies. A captured body AABB may be used only while detail is pending or genuinely unsupported.
- Preserve deterministic cache invalidation when world identity, geometry identity, or shape-decoding settings change.
- Do not dereference Havok or mutable ROCK configuration from the OpenVR compositor callback after the snapshot-boundary slice.
- Do not allow exceptions to cross F4SE/OpenVR/runtime boundaries.
- Do not leave duplicate render paths, compatibility fallbacks, or temporary production code.
- Keep the ROCK provider API at V1.

## Baseline Audit

### Existing ROCK Strengths To Retain

- Current-frame stereo extraction uses the verified renderer offsets and guarded reads.
- Body geometry is already stereo-instanced with `DrawIndexedInstanced(..., 2)`.
- Lines already use one dynamic vertex-buffer mapping per line batch, with deduplication and a vertex budget.
- The overlay already has explicit body-role semantics and fixed logical entry caps.
- Shape caching protects against generated-shape address reuse with a geometry fingerprint.
- World and shape-decoding setting changes invalidate the shape cache.
- CPU profiler scopes already cover overlay publication and rendering.
- A submitted-texture RTV cache already exists.

These are foundations to preserve, not features to reimplement.

### Baseline Structural Costs

On a cache-cold rendered frame, current ROCK can perform up to 32 CPU shape generations and up to 64 D3D buffer creations from the OpenVR `Submit` callback. With `N` visible body entries it also performs approximately `N` model constant-buffer maps and `N` stereo draw calls. Text is built, uploaded, and drawn separately for each text entry.

No FPS or GPU-time claim is inferred from source inspection. Runtime timing must be measured after instrumentation is installed.

## Detailed Improvement Plan

### Slice 1: Render-Hook And D3D Hardening

- [x] Add once-per-published-game-frame rendering using the overlay frame serial.
- [x] Add an atomic reentrancy guard around the overlay draw.
- [x] Make the OpenVR Submit hook fail closed and prevent exceptions crossing the hook.
- [x] Replace manual pass cleanup with one RAII render-pass guard.
- [x] Save, clear, and restore vertex, pixel, geometry, hull, and domain shaders including class instances.
- [x] Preserve every D3D state that the overlay changes.
- [x] Preserve vertex-buffer slots 0 and 1 so later instancing cannot leak state.
- [x] Reject nested render passes.
- [x] Make camera/model upload functions return failure and skip affected draws rather than binding stale contents.
- [x] Convert long-lived D3D resources to deterministic COM ownership with all-or-nothing partial-initialization cleanup.
- [x] Keep D3D/hook ownership process-scoped because ROCK/F4SEVR exposes no supported hot-unload or device-reset callback; avoid unsafe destructor-time vtable restoration.
- [x] Remove the no-op hardcoded `0xD844BC` main-render trampoline.
- [x] Remove redundant nested RTV/viewport save-restore ownership.
- [x] Add source regressions for the single Submit path, frame guard, shader-state coverage, and fail-closed upload behavior.
- [x] Add/retain shader compilation coverage.

Acceptance:

- Exactly one overlay render attempt is admitted for each newly published frame.
- Recursive or duplicate Submit calls skip without touching D3D state.
- Every changed D3D state is restored on success, validation failure, map failure, and exception.
- The no-op RVA hook is absent.
- `custom-fast` Release build and relevant tests pass.
- Slice is committed and post-commit regression checks pass.

### Slice 2: Immutable Render-Ready Publication Boundary

- [x] Replace the mutex-protected full-frame copy with a pooled immutable snapshot or fixed triple-buffer design.
- [x] Add a monotonic publication serial used by the render guard.
- [x] Capture exact role-specific body transforms on ROCK's game/physics update path.
- [x] Capture stable body identity, shape identity/fingerprint, actual body AABB, role, color/decode metadata, and render flags.
- [x] Capture a narrow immutable overlay settings structure with the frame.
- [x] Remove `hknpWorld*` and transient engine pointers from the render-facing snapshot.
- [x] Remove all body/motion-array dereferences from the compositor callback.
- [x] Remove all mutable `g_rockConfig` reads from the compositor callback.
- [x] Ensure builder buffers are reused without clearing or copying unused fixed-capacity storage.
- [x] Preserve Target body-array transforms and all other current transform-source rules with explicit tests.

Acceptance:

- OpenVR rendering consumes immutable values only.
- No Havok object, world pointer, body pointer, motion pointer, or mutable config is read by the compositor path.
- The renderer does not take the publication mutex or copy the full logical frame.
- Existing stereo and semantic source/unit regressions pass unchanged or are strengthened.
- `custom-fast` Release build and relevant tests pass.
- Slice is committed and post-commit regression checks pass.

### Slice 3: Bounded Shape Recipe, Worker, Upload, And Cache Pipeline

- [x] Define stable CPU-only shape recipes captured on the ROCK update thread.
- [x] Ensure recipe capture is admitted against the per-frame and queue budgets before expensive decoding begins.
- [x] Add one owned low-priority worker that never dereferences Havok or calls D3D.
- [x] Build mesh data from immutable recipes only.
- [x] Add explicit worker initialization and deterministic shutdown/join ordering.
- [x] Add bounded pending-job and completed-upload queues.
- [x] Limit recipe captures per game frame.
- [x] Limit GPU uploads per rendered frame.
- [x] Add cache states `Pending`, `Ready`, and `Unsupported`.
- [x] Add generation tokens so stale jobs/uploads are discarded after world/settings invalidation.
- [x] Add LRU eviction and both entry-count and approximate GPU-byte budgets.
- [x] Keep generated-shape address-reuse protection; never reduce identity to a raw pointer.
- [x] Compute geometry identity once during safe capture, not on every render lookup.
- [x] Keep cache locks out of geometry building and D3D resource creation.
- [x] Publish a captured real body AABB proxy while detailed geometry is pending.
- [x] Keep detailed geometry as the final output whenever supported.
- [x] Add deterministic queue/cache policy tests and lifecycle regressions.

Acceptance:

- No convex triangulation, support-vertex call, recursive fingerprinting, or first-use CPU mesh build occurs in OpenVR Submit.
- No worker thread reads engine memory or creates D3D resources.
- All queues and caches have enforced bounds and observable deferrals/evictions.
- Cache invalidation cannot publish stale generation results.
- Pending detail remains visible through the body's actual captured AABB.
- `custom-fast` Release build and relevant tests pass.
- Slice is committed and post-commit regression checks pass.

### Slice 4: Ordered GPU Instancing And Allocation-Free Diagnostic Batches

- [ ] Add an 80-byte or equivalently aligned per-object instance record containing model matrix and color.
- [ ] Add an instanced input layout with step rate 2 so each object record serves both stereo instances.
- [ ] Upload one contiguous object-instance stream per overlay frame where practical.
- [ ] Draw ordered adjacent runs that share the same mesh.
- [ ] Preserve transparency/diagnostic ordering; do not globally reorder entries merely to enlarge batches.
- [ ] Canonicalize shared unit-cube proxy geometry.
- [ ] Canonicalize reusable sphere/capsule geometry when the required model transform preserves shape fidelity.
- [ ] Keep arbitrary detailed convex meshes cached and batch adjacent equal meshes.
- [ ] Add color to line vertices and upload/draw the complete ordered line stream without per-color model-buffer maps.
- [ ] Replace per-frame `unordered_set` line deduplication with reusable bounded scratch or an allocation-free equivalent.
- [ ] Aggregate all text entries into one reusable colored vertex upload/draw.
- [ ] Fix the unreachable text-truncation statistic with an explicit overflow flag/counter.
- [ ] Evaluate a glyph atlas or shader bitmask only after the single-upload text path is complete and measured.
- [ ] Add GPU-instancing, stereo-indexing, ordering, capacity, and shader compilation tests.

Acceptance:

- Visible bodies no longer require one model constant-buffer map per body.
- Shared proxy/detail runs produce one stereo-instanced draw per ordered mesh run.
- Lines use one upload and no per-color constant-buffer updates.
- Text uses reusable storage and a bounded aggregate upload.
- Truncation/overflow counters report actual rejected work.
- `custom-fast` Release build and relevant tests pass.
- Slice is committed and post-commit regression checks pass.

### Slice 5: Shape Fidelity, Configuration, Metrics, And Module Boundaries

- [ ] Add triangle shape recipe support when local layout/source authority is sufficient.
- [ ] Verify scaled-convex translation against FO4VR source/binary evidence before applying it.
- [ ] Verify static/dynamic compound layouts against FO4VR before implementing compound recipes.
- [ ] Add capped compound recursion and child-count policy.
- [ ] Use actual captured AABB fallback for compressed mesh, height-field, unknown, or safely unsupported geometry.
- [ ] Resolve configuration drift: generation value `100` versus effective cap `32`, C++ convex default `6` versus repository INI `8`, and hidden line/cache budgets.
- [ ] Add validated keys for captures/frame, queued jobs, uploads/frame, cache entries, cache bytes, instance capacity, line vertices, and text vertices.
- [ ] Update repository INIs and the active production INI in place for every added/renamed/removed key.
- [ ] Remove or correctly use stale settings-key helpers.
- [ ] Add counters for duplicate/reentrant skips, draw calls, mesh binds, buffer maps, instance counts, proxy/detail/cache state, queue depths, deferrals, evictions, and rejected vertices.
- [ ] Add a nonblocking D3D timestamp-query ring if it can be read several frames later without flushing or stalling.
- [ ] Split the current monolith into narrow modules for hook/lifecycle, D3D pass, shaders, published frame, shape recipe/cache, diagnostic batching, and statistics.
- [ ] Remove superseded helpers and duplicate paths after module extraction.

Acceptance:

- Every new binary/layout-dependent behavior is locally verified and recorded below before production use.
- Configuration defaults, effective limits, repo INIs, and active production INI agree.
- CPU and GPU cost can be measured without blocking the render path.
- Module ownership and shutdown order are explicit.
- All prior overlay features and fidelity remain present.
- `custom-fast` Release build and full capped regression suite pass.
- Slice is committed and post-commit regression checks pass.

## Deliberately Excluded Standalone Behavior

The following standalone behaviors must not be ported into ROCK's explicit diagnostic overlay:

- Whole-world body scanning.
- Generic collision-layer filtering.
- Distance-based visibility or detail LOD.
- Movement-threshold snapshot refresh.
- Multi-frame transform refresh intervals.
- Standalone-only character-controller, center-of-mass, or constraint diagnostics unless separately requested.
- Raw standalone Havok offsets or structures without ROCK-local verification.
- The standalone's superseded older render-finalization hook.
- Any change that replaces ROCK's verified current-frame stereo path.
- Any permanent fidelity reduction in exchange for performance.

Stable membership or geometry identity may be cached, but transforms and live diagnostic values remain current every game frame.

## Validation Contract

For every implementation slice:

1. Inspect `git diff --check` and the scoped diff.
2. Run targeted unit/source/shader tests.
3. Run the required auto-deploying preset in Release configuration:

   `cmake --preset custom-fast`

   `cmake --build build-fast --config Release --target ROCK -- /m:1 /p:CL_MPCount=2`

4. If C1060 occurs, retry with `/p:CL_MPCount=1`.
5. Run CTest with no more than `-j 4`.
6. Inspect deployed DLL/PDB metadata after auto-deploy.
7. Stage only files owned by the slice plus this ledger.
8. Commit with the required scoped commit-message format.
9. Re-run the relevant post-commit regressions and record the result.
10. Inspect final `git status` and record unrelated remaining dirt.

Runtime completion also requires, when the game can be exercised safely:

- `f4sevr.log` confirms ROCK loaded correctly.
- ROCK's runtime log confirms overlay hook installation and no repeated-state failures.
- Overlay diagnostics are visually checked in both eyes.
- Repeated enable/disable, config reload, world transition, and shape invalidation do not crash or leak stale geometry.
- CPU/GPU measurements are compared with the baseline without claiming gains from source structure alone.

## Current Progress

- [x] Completed source audit of ROCK versus CollisionVisualizerF4VR.
- [x] Classified direct transfers, ROCK-specific adaptations, and excluded standalone behavior.
- [x] Created persistent implementation ledger.
- [x] Slice 1 implementation, full regression suite, build, auto-deploy, commit, and post-commit regression complete.
- [x] Slice 2 implementation, build, auto-deploy, commit, and post-commit regression complete.
- [x] Slice 3 implementation, build, auto-deploy, and pre-commit full regression complete.
- [ ] Slice 4 not started.
- [ ] Slice 5 not started.
- [ ] Final runtime validation not started.

## Verification Evidence Ledger

Record any Ghidra/FO4VR source verification here before implementing a new offset/layout-dependent feature.

| Date | Claim | Authority/evidence | Result | Implemented in commit |
|---|---|---|---|---|
| 2026-07-20 | Existing ROCK stereo matrices and role-specific transform sources remain authoritative | Current ROCK source and existing stereo/semantic tests | Preserve unchanged | Pending |
| 2026-07-20 | `hknpWorld::GetBodyAabb` has the ABI and output layout required by ROCK | Modified and pristine CommonLibF4VR both declare `(hknpWorld*, hknpBodyId, void*)` at `REL::ID(249572)`; local address library/PDB map it to FO4VR `0x141539120`; read-only Ghidra disassembly/decompilation shows the third argument in `R8`, exactly eight float writes, min `[0..3]` then max `[4..7]`, after unsigned 16-bit decompression | **Confirmed.** Use the engine wrapper on the publisher thread, validate finite ordered bounds, and convert Havok units to game units | `07abc83` |
| 2026-07-20 | ROCK must not copy the standalone raw compressed-AABB decoder | Read-only Ghidra shows `PUNPCKLWD`/`PUNPCKHWD` against zero, while CollisionVisualizerF4VR reads the same body storage through `std::int16_t*` | **Standalone implementation disputed.** Its signed interpretation is wrong above `32767`; ROCK keeps the verified engine wrapper and rejects those raw-offset/signed patterns in source regression | `07abc83` |
| 2026-07-20 | Overlay `hknpShape` virtual calls have the correct slots/signatures | Modified and pristine `hknpShape.h` agree exactly: `GetType` slot `04`, `GetNumberOfSupportVertices` slot `08`, and `GetSupportVertices(hkcdVertex*, int32)` slot `09`; the standalone visualizer independently calls the same virtuals/slots | **Confirmed.** Preserve the CommonLib virtual calls and bounded/null-checked support-vertex handling | Existing behavior; guarded during Slice 3 extraction |
| 2026-07-20 | Renderer singleton and D3D device/context layout are stable across the two local CommonLib copies | Modified and pristine `BSGraphics.h` agree on `RendererData::GetSingleton` ID `1235449`, device `+0x48`, context `+0x50`, and size `0x25C0`; local relocation manifest maps the ID to FO4VR `.data` `0x1460F3CE8`; standalone uses the same interface | **Confirmed.** Retain null checks and non-owning casts to D3D11 interfaces | Existing behavior; Slice 1 hardened lifetime/state handling |

## Commit Ledger

| Slice | Commit | Build | Tests | Post-commit regression | Notes |
|---|---|---|---|---|---|
| Ledger setup | `a48c915` | Not applicable | Not applicable | `git show --check` passed | Explicitly authorized Markdown progress artifact |
| 1: Hook/D3D hardening | `da16718` | `custom-fast` Release build and auto-deploy passed | 117/117 full suite passed | 117/117 passed after commit | Deployed DLL/PDB hashes match build artifacts |
| 2: Immutable publication | `07abc83` | `custom-fast` Release build and auto-deploy passed | 119/119 full suite passed | 119/119 passed after commit; `git show --check` passed | CommonLib boundary independently cross-checked against pristine source, local address evidence, and FO4VR Ghidra |
| 3: Async shape/cache | Pending hash | `custom-fast` Release build and auto-deploy passed | 122/122 full suite passed | Pending commit | One low-priority worker; bounded recipe, CPU-completion, upload, LRU, and GPU-byte paths |
| 4: GPU/diagnostic batching | Pending | Pending | Pending | Pending | |
| 5: Fidelity/config/metrics/modules | Pending | Pending | Pending | Pending | |

## Progress Journal

### 2026-07-20 — Audit And Initialization

- Audited ROCK's overlay renderer, publisher, policy, configuration, tests, and relevant standalone history.
- Identified the compositor callback as the primary safety/performance boundary.
- Confirmed that ROCK already has correct current-frame stereo, stereo-per-body instancing, line batching, logical caps, cache invalidation, and CPU profiling that must be retained.
- Confirmed that standalone commit `806fef7` introduced a per-frame guard after repeated rendering caused an NVIDIA driver failure; ROCK currently lacks the equivalent guard.
- Confirmed that the standalone current renderer adds complete programmable-stage state handling, immutable snapshot publication, bounded shape recipe/build/upload work, stateful cache generations, AABB transitional proxies, multi-body stereo instancing, reusable render scratch, shader compile tests, and performance-policy tests.
- Recorded unrelated active weapon/scope changes in the ROCK worktree. They are not owned by this effort and must not be staged or reverted.

### 2026-07-20 — Slice 1 Hook/D3D Hardening

- Added a tested publication-serial `FrameAdmission` lease. It rejects duplicate and reentrant overlay draws while leaving a newer publication claimable after the active draw completes.
- Made the OpenVR Submit hook `noexcept`, catch overlay exceptions, log the first failure, and always forward to the original Submit function.
- Removed the no-op hardcoded main-render trampoline at RVA `0xD844BC`.
- Replaced partial raw D3D ownership with an all-or-nothing `D3DResources` COM aggregate.
- Added a single RAII render-pass guard that captures/restores VS, PS, GS, HS, DS, shader class instances, VS constant buffers, input layout, topology, raster/depth/blend state, render targets, viewports, vertex-buffer slots 0 and 1, and the index buffer.
- The overlay now explicitly disables GS/HS/DS while its shaders are active.
- Camera and model constant-buffer map failures now fail closed and skip the affected frame/draw instead of binding stale data.
- Extracted shader source into a testable header and added runtime shader-compilation tests.
- Added frame-admission unit tests and render-safety source-boundary tests.
- Focused tests passed: frame admission, shader compilation, render safety, and verified stereo layout.
- `custom-fast` Release compilation and link succeeded. Auto-deploy failed only when overwriting `D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll` because `Fallout4VR.exe` was running and held the DLL open. The game was not terminated.
- After the game exited naturally, the required retry compiled, linked, and auto-deployed successfully.
- Full capped regression suite passed: 52 policy tests and 65 source-boundary tests, 117/117 total.
- Deployed `ROCK.dll` is version `0.5.0.0`, size `5,479,424`, timestamp `2026-07-20 19:38:38`; build/deploy SHA-256 match `328CF566A0272E59B0641455A35E6437EDFDD4CA4A0EA918CEF6CE32FEFFB8F2`.
- Build/deploy `ROCK.pdb` SHA-256 match `6E818CEA09C6DECE0CE682F56EF63E7A0E8B5CFA5B657239600482CED27C32FD`.
- Committed Slice 1 as `da16718` (`fix/debug-overlay: harden compositor rendering lifecycle`).
- Post-commit regression rebuilt the registered targets and passed 117/117 tests. The worktree was clean before Slice 2 began.

### 2026-07-20 — Slice 2 Immutable Publication Boundary

- Replaced the mutex-protected `BodyOverlayFrame` copy with a four-buffer single-producer snapshot pool and `std::atomic<std::shared_ptr<const PublishedOverlayFrame>>` publication.
- Added focused pool tests covering reuse, retention while published/consumed, capacity exhaustion, and recovery.
- The publisher now resolves role-specific body and body-axis transforms before publication. Target bodies/axes retain BODY-array authority; other roles retain live-motion-when-available behavior.
- Captured stable body IDs, pointer-plus-geometry-fingerprint shape keys, real world body AABBs, roles, render flags, decode data, and the narrow overlay settings consumed by rendering.
- Real AABBs use the current local CommonLibF4VR `hknpWorld::GetBodyAabb` wrapper, validate finite ordered bounds, and convert Havok units to ROCK game units. No standalone raw AABB offsets were imported.
- Blind CommonLib verification was performed locally without assuming the modified headers were correct: every overlay-facing wrapper was compared with `original frik deps/CommonLibF4VR`, then relocations were checked against the local address database/PDB, and the new AABB call was independently verified in the FO4VR binary through read-only Ghidra analysis.
- `hknpBodyId` is byte-identical in the modified and pristine CommonLib copies (four-byte `hkHandle<uint32_t, 0x7fffffff, ...>`), and the two local address-ID maps are byte-identical. This rules out a hidden argument-width or map-drift mismatch for `GetBodyAabb`.
- Ghidra confirmed that FO4VR's AABB function writes exactly 32 bytes as minimum then maximum float vectors and zero-extends the stored 16-bit components. The standalone visualizer's raw `std::int16_t` decoder is therefore not safe to transfer; a source regression now rejects that decoder and its raw offsets.
- The existing `hknpShape` virtuals and `BSGraphics::RendererData` access were separately cross-checked between both CommonLib trees and against the standalone caller/address evidence; no overlay-side correction was required.
- The compositor now atomically acquires one immutable snapshot and no longer dereferences `hknpWorld`, body arrays, motion arrays, shape pointers, or mutable `g_rockConfig` state.
- Shape CPU decoding is temporarily performed during safe publication and passed as CPU-owned data. Slice 3 replaces this synchronous transitional step with bounded immutable recipes and a worker.
- GPU cache values are shared immutable handles so game-thread invalidation cannot destroy a buffer still used by the compositor.
- Only active marker, skeleton, axis, body, and text records cross the publication boundary; pooled vector capacity is retained between frames.
- Added a compositor-boundary source regression and strengthened Target transform-source regression patterns for the new publication location.
- Required `custom-fast` configure, capped Release build, auto-deploy, and the complete test suite passed.
- The initial full-suite invocation encountered stale `build-tests` regeneration without preset-provided `VCPKG_ROOT`; refreshing `custom-tests` corrected the test environment, after which 53 policy tests and 66 source-boundary tests passed, 119/119 total.
- Final audited Slice 2 build recompiled and auto-deployed successfully after the CommonLib verification comments/regression were added. The complete suite again passed: 53 policy tests and 66 source-boundary tests, 119/119 total.
- Deployed `ROCK.dll` is version `0.5.0.0`, size `5,357,056`, timestamp `2026-07-20 20:06:57`; build/deploy SHA-256 match `38AB73B97C75D38C5F31B582FD683C8B1390B4F4BB299B31A05D4BA5E9C3F0D2`.
- Build/deploy `ROCK.pdb` SHA-256 match `F4E13E7D3920CBAC38A0554EA12CEE5ECC02551E3D6B39BB821803CE4880B49A`.
- Committed Slice 2 as `07abc83` (`fix/debug-overlay: publish immutable render snapshots`).
- Post-commit regression passed 119/119 tests; `git show --check` passed and the worktree was clean before Slice 3 began.

### 2026-07-20 — Slice 3 Bounded Shape Pipeline

- Slice started from clean commit `07abc83` after its full post-commit regression.
- Added immutable CPU-only recipes for supported sphere, capsule, convex, and scale-only scaled-convex geometry. Engine reads occur only during guarded publisher capture; the worker consumes copied values only.
- Publisher admission now reserves a bounded cache/queue slot before recipe capture. Duplicate shapes coalesce, per-frame capture work is capped, and pointer-plus-geometry-fingerprint identity remains intact.
- Replaced synchronous compositor mesh generation with one owned below-normal-priority worker. The compositor processes a bounded number of completed CPU meshes and performs D3D uploads outside the pipeline mutex.
- Added explicit `Pending`, `Ready`, and `Unsupported` states; generation-token invalidation; independent pending/completed queue bounds; entry and approximate GPU-byte budgets; LRU eviction; stale-result accounting; and allocation-failure terminalization.
- Captured real body AABBs remain visible through one canonical unit-cube proxy while detail is pending and for genuinely unsupported geometry. Supported detailed geometry replaces the proxy as soon as its upload completes.
- Added deterministic shutdown before physics teardown, including worker notification, join, cache release, and restart coverage.
- Eliminated per-frame fingerprint heap allocation by replacing the temporary support-vertex vector with a fixed 256-entry stack array.
- Added pure geometry tests, pipeline lifecycle/bounds/LRU tests, and a source-boundary regression proving the worker has no CommonLib/Havok/D3D access and the compositor performs no recipe capture, fingerprinting, or CPU mesh construction.
- Re-audited every CommonLib-facing overlay function against `original frik deps/CommonLibF4VR`. Exact virtual-slot counts are now enforced for `GetType`, `GetNumberOfSupportVertices`, and `GetSupportVertices`; both fingerprint and recipe paths are enclosed by fail-closed SEH boundaries.
- Replaced the duplicated raw `+0x14` radius read with the identically laid-out CommonLib `hknpShape::convexRadius` member. Recursive scaled recipes use unique ownership attached before the deeper guarded read, so a structured access fault cannot orphan a temporary inner recipe.
- Required `custom-fast` configure and capped Release build passed and auto-deployed. The dedicated `custom-tests` tree was regenerated to avoid accepting the stale test registry left in `build-fast`; the current complete suite passed 55 policy tests and 67 source-boundary tests, 122/122 total.
- Deployed `ROCK.dll` is version `0.5.0.0`, size `5,373,440`, timestamp `2026-07-20 20:42:40`; build/deploy SHA-256 match `EC586103C9FBB5425465ED4FF699CA63141088659EA4E29FCD29C1F58729ED3E`.
- Build/deploy `ROCK.pdb` SHA-256 match `FBB3C5C9B45C35888D7903229D57544DC797566977C41B5C0EDA2CCC9AAFEE93`.

## Remaining Risks

- The ROCK worktree was changing concurrently when this effort began. Re-check overlapping core files before every edit and commit.
- A worker adds lifecycle complexity; shutdown, invalidation, and device ownership must be explicit and tested.
- Transparent diagnostic ordering constrains batching; correct order takes priority over draw-count reduction.
- Compound and scaled-convex details are layout-sensitive and cannot be copied from standalone without FO4VR-local verification.
- The auto-deploying build modifies deployed DLL/PDB as required by workspace policy. Runtime launch/log validation still depends on whether the game can be exercised during the task.
- Structural improvements are not measured performance gains until instrumentation and runtime comparison confirm them.
