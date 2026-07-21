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

- [x] Add an 80-byte or equivalently aligned per-object instance record containing model matrix and color.
- [x] Add an instanced input layout with step rate 2 so each object record serves both stereo instances.
- [x] Upload one contiguous object-instance stream per overlay frame where practical.
- [x] Draw ordered adjacent runs that share the same mesh.
- [x] Preserve transparency/diagnostic ordering; do not globally reorder entries merely to enlarge batches.
- [x] Canonicalize shared unit-cube proxy geometry.
- [x] Canonicalize reusable sphere/capsule geometry when the required model transform preserves shape fidelity.
- [x] Keep arbitrary detailed convex meshes cached and batch adjacent equal meshes.
- [x] Add color to line vertices and upload/draw the complete ordered line stream without per-color model-buffer maps.
- [x] Replace per-frame `unordered_set` line deduplication with reusable bounded scratch or an allocation-free equivalent.
- [x] Aggregate all text entries into one reusable colored vertex upload/draw.
- [x] Fix the unreachable text-truncation statistic with an explicit overflow flag/counter.
- [x] Evaluate a glyph atlas or shader bitmask only after the single-upload text path is complete and measured.
- [x] Add GPU-instancing, stereo-indexing, ordering, capacity, and shader compilation tests.

Acceptance:

- Visible bodies no longer require one model constant-buffer map per body.
- Shared proxy/detail runs produce one stereo-instanced draw per ordered mesh run.
- Lines use one upload and no per-color constant-buffer updates.
- Text uses reusable storage and a bounded aggregate upload.
- Truncation/overflow counters report actual rejected work.
- `custom-fast` Release build and relevant tests pass.
- Slice is committed and post-commit regression checks pass.

### Slice 5: Shape Fidelity, Configuration, Metrics, And Module Boundaries

- [x] Add triangle shape recipe support when local layout/source authority is sufficient.
- [x] Verify scaled-convex translation against FO4VR source/binary evidence before applying it.
- [x] Verify static/dynamic compound layouts against FO4VR before implementing compound recipes.
- [x] Add capped compound recursion and child-count policy.
- [x] Use actual captured AABB fallback for compressed mesh, height-field, unknown, or safely unsupported geometry.
- [x] Resolve configuration drift: generation value `100` versus effective cap `32`, C++ convex default `6` versus repository INI `8`, and hidden line/cache budgets.
- [x] Add validated keys for captures/frame, queued jobs, uploads/frame, cache entries, cache bytes, instance capacity, line vertices, and text vertices.
- [x] Update repository INIs and the active production INI in place for every added/renamed/removed key.
- [x] Remove or correctly use stale settings-key helpers.
- [x] Add counters for duplicate/reentrant skips, draw calls, mesh binds, buffer maps, instance counts, proxy/detail/cache state, queue depths, deferrals, evictions, and rejected vertices.
- [x] Add a nonblocking D3D timestamp-query ring that reads older frames without flushing or stalling.
- [x] Extract stable independently-owned domains into narrow shader, admission, snapshot-pool, settings, shape-geometry, shape-pipeline, diagnostic-batch, GPU-timing, and statistics modules. Keep the private hook/D3D-pass/published-frame coordinator in one translation unit so its process lifetime and render-pass ownership remain local rather than creating cyclic internal APIs.
- [x] Remove superseded helpers and duplicate paths after module extraction.

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
- [x] Slice 3 implementation, build, auto-deploy, commit, and post-commit regression complete.
- [x] Slice 4 implementation, build, auto-deploy, commit, and post-commit regression complete.
- [x] Slice 5 implementation, build, auto-deploy, commit, and post-commit regression complete.
- [ ] Final runtime/in-headset validation unavailable in this session: the game is not running and the final deployed artifact has not yet been loaded.

## Verification Evidence Ledger

Record any Ghidra/FO4VR source verification here before implementing a new offset/layout-dependent feature.

| Date | Claim | Authority/evidence | Result | Implemented in commit |
|---|---|---|---|---|
| 2026-07-20 | Existing ROCK stereo matrices and role-specific transform sources remain authoritative | Current ROCK source and existing stereo/semantic tests | Preserve unchanged | `07abc83` and retained through final regression |
| 2026-07-20 | `hknpWorld::GetBodyAabb` has the ABI and output layout required by ROCK | Modified and pristine CommonLibF4VR both declare `(hknpWorld*, hknpBodyId, void*)` at `REL::ID(249572)`; local address library/PDB map it to FO4VR `0x141539120`; read-only Ghidra disassembly/decompilation shows the third argument in `R8`, exactly eight float writes, min `[0..3]` then max `[4..7]`, after unsigned 16-bit decompression | **Confirmed.** Use the engine wrapper on the publisher thread, validate finite ordered bounds, and convert Havok units to game units | `07abc83` |
| 2026-07-20 | ROCK must not copy the standalone raw compressed-AABB decoder | Read-only Ghidra shows `PUNPCKLWD`/`PUNPCKHWD` against zero, while CollisionVisualizerF4VR reads the same body storage through `std::int16_t*` | **Standalone implementation disputed.** Its signed interpretation is wrong above `32767`; ROCK keeps the verified engine wrapper and rejects those raw-offset/signed patterns in source regression | `07abc83` |
| 2026-07-20 | Overlay `hknpShape` virtual calls have the correct slots/signatures | Modified and pristine `hknpShape.h` agree exactly: `GetType` slot `04`, `GetNumberOfSupportVertices` slot `08`, and `GetSupportVertices(hkcdVertex*, int32)` slot `09`; the standalone visualizer independently calls the same virtuals/slots | **Confirmed.** Preserve the CommonLib virtual calls and bounded/null-checked support-vertex handling | Existing behavior; guarded during Slice 3 extraction |
| 2026-07-20 | Renderer singleton and D3D device/context layout are stable across the two local CommonLib copies | Modified and pristine `BSGraphics.h` agree on `RendererData::GetSingleton` ID `1235449`, device `+0x48`, context `+0x50`, and size `0x25C0`; local relocation manifest maps the ID to FO4VR `.data` `0x1460F3CE8`; standalone uses the same interface | **Confirmed.** Retain null checks and non-owning casts to D3D11 interfaces | Existing behavior; Slice 1 hardened lifetime/state handling |
| 2026-07-20 | FO4VR triangle shapes can be captured without importing an unverified concrete layout | Pristine and modified CommonLib expose no concrete `hknpTriangleShape` fields, but both expose the verified support-vertex virtuals. Read-only Ghidra shows `hknpTriangleShape` constructors at `0x1415A5550` and `0x1415A5900` inheriting the convex-polytope path | **Confirmed.** Capture exactly the first three finite support vertices through CommonLib and emit one double-sided triangle; do not apply generic centroid/radius hull inflation | `833cc08` |
| 2026-07-20 | Scaled-convex inner shape, scale, and translation fields used by the standalone are correct for FO4VR | Read-only Ghidra of `hknpScaledConvexShapeBase` constructor `0x14175FC50` writes the child pointer at `+0x30`, zero/metadata at `+0x38`, the scale vector at `+0x40`, and helper-produced translation at `+0x50` | **Standalone and current ROCK disputed.** `+0x38` is not the scale vector. Read scale at `+0x40`, translation at `+0x50`, validate both, include both in the cache fingerprint, and apply `inner * scale + translation` with translation converted from Havok units | `833cc08` |
| 2026-07-20 | Static/dynamic compound root and child-slot layouts from the standalone are correct for FO4VR | Read-only Ghidra of `hknpCompoundShape` constructor `0x1416E2BE0`, allocation helper `0x1416E3F20`, copy helper `0x1416E3340`, static key-mask constructor `0x1415F68E0`, and independent consumers shows root slots at `+0x60`, high-water/size at `+0x68`, 0x80-byte slots, transform at slot `+0x00`, scale at `+0x40`, child shape pointer at `+0x50`, and active byte `+0x60 == 0`. Static `0x141E9CAC0` and dynamic `0x1416E4420` constructors share that base | **Standalone root offsets disputed; FO4VR layout confirmed.** Support types 7 and 8 using the verified fields, fail the whole recipe to the body AABB if any active child is invalid/unsupported, and enforce configured child/depth bounds | `833cc08` |
| 2026-07-20 | Final overlay CommonLib call surface remains valid after all modernization changes | Exhaustive current-source scan found only `hknpShape::{GetType, GetNumberOfSupportVertices, GetSupportVertices}`, `hknpWorld::GetBodyAabb`, `BSGraphics::RendererData::GetSingleton`, and `REL::Relocation(REL::Offset)::address`. Modified and pristine CommonLibF4VR agree on the relevant virtual slots/signatures, relocation IDs, renderer member offsets, and offset-relocation semantics; the runtime-sensitive shape/AABB behavior was independently checked in FO4VR Ghidra above | **Confirmed.** The GPU timer uses only Windows D3D11 SDK interfaces and adds no CommonLib or engine-memory dependency | `ffc7852` |

## Commit Ledger

| Slice | Commit | Build | Tests | Post-commit regression | Notes |
|---|---|---|---|---|---|
| Ledger setup | `a48c915` | Not applicable | Not applicable | `git show --check` passed | Explicitly authorized Markdown progress artifact |
| 1: Hook/D3D hardening | `da16718` | `custom-fast` Release build and auto-deploy passed | 117/117 full suite passed | 117/117 passed after commit | Deployed DLL/PDB hashes match build artifacts |
| 2: Immutable publication | `07abc83` | `custom-fast` Release build and auto-deploy passed | 119/119 full suite passed | 119/119 passed after commit; `git show --check` passed | CommonLib boundary independently cross-checked against pristine source, local address evidence, and FO4VR Ghidra |
| 3: Async shape/cache | `889347a` | `custom-fast` Release build and auto-deploy passed | 122/122 full suite passed | 122/122 passed after commit; `git show --check` passed | One low-priority worker; bounded recipe, CPU-completion, upload, LRU, and GPU-byte paths |
| 4: GPU/diagnostic batching | `2c0c365` | `custom-fast` Release build and auto-deploy passed | 124/124 full suite passed | 124/124 passed after commit; `git show --check` passed | One body map, one colored-line map/draw, one aggregate text map/draw; ordered adjacent mesh runs |
| 5a: Shape fidelity | `833cc08` | `custom-fast` Release build and auto-deploy passed | 124/124 full suite passed | 124/124 passed after commit; `git show --check` passed | Correct triangle/scaled-convex/compound behavior from verified FO4VR evidence |
| 5b: Bounded runtime settings | `212d9e9` | `custom-fast` Release build and auto-deploy passed | 126/126 full suite passed | 126/126 passed after commit; `git show --check` passed | Twelve sanitized limits wired at their actual enforcement points; repository, packaged, and active INIs updated in place |
| 5c: GPU timing/admission/modules | `ffc7852` | `custom-fast` Release build and auto-deploy passed | 127/127 full suite passed pre-commit | Release rebuild/auto-deploy and 127/127 passed after commit; `git show --check` passed | Fixed non-blocking D3D11 query ring, differentiated admission counters, and isolated runtime-statistics contract |

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
- Committed Slice 3 as `889347a` (`fix/debug-overlay: bound asynchronous shape pipeline`).
- Post-commit regression passed 122/122 tests; `git show --check` passed and the worktree was clean before Slice 4 began.

### 2026-07-20 — Slice 4 Ordered GPU And Diagnostic Batching

- Slice started from clean commit `889347a` after its full post-commit regression.
- Implementation target: one reusable per-object instance stream with ordered adjacent mesh runs, one colored line stream, one aggregate colored text stream, bounded reusable scratch, and explicit overflow statistics without changing stereo or transparency order.
- Replaced per-body model/color constant-buffer maps with one exactly bounded 80-byte instance stream. D3D advances each record after two instances, while the shader derives the current eye from `SV_InstanceID & 1`; one mapped stream therefore retains ROCK's existing two-eye rendering.
- Body draws now combine only adjacent entries whose resolved GPU mesh is identical. No global sort occurs, the original diagnostic/alpha order remains intact, and retained shared GPU owners prevent concurrent invalidation from releasing buffers during a render pass.
- The existing actual-AABB fallback continues to use one shared unit-cube mesh. Direct spheres now share one canonical unit-sphere cache entry and retain exact radius through a per-body uniform model scale. Capsules deliberately remain parameterized cached meshes: endpoint distance and cap radius are independent, so a single ordinary model transform cannot canonicalize all capsules without distorting them. Nested spheres also remain baked so scaled-convex composition is unchanged.
- Replaced line `unordered_set` storage with prepared generation-stamped open-address scratch. Reversed duplicates are coalesced, insertion order and per-line colors are retained, non-finite/degenerate input fails closed, and caller budgets cannot exceed the prepared hard capacity.
- Colored lines are written directly to one mapped dynamic vertex buffer and emitted in one stereo draw. Text entries are appended in order to one prepared colored-vertex vector, mapped once, and drawn once; no per-entry or per-color model upload remains.
- Text overflow reporting now records both affected entries and the exact number of rejected vertices. Body-instance and line-map failures are separately observable and continue to fail closed with once-only warnings.
- A glyph atlas/shader-bitmask conversion was evaluated after aggregation. It is not adopted without runtime evidence: the current bit-font has no texture/sampler dependency, is allocation-free after preparation, and now costs one bounded upload/draw; adding atlas state would increase pass-state ownership and visual-risk surface before the Slice 5 timing ring can establish that text vertex generation is material.
- Added line-order/dedup/non-finite/capacity unit coverage, canonical-sphere geometry coverage, explicit batching/source invariants, and compilation coverage for all three vertex shaders plus the pixel shader.
- Required `custom-fast` configure and capped Release build passed and auto-deployed with the final canonical-sphere implementation. The refreshed `custom-tests` tree built all 56 policy binaries, and the complete capped suite passed 56 policy plus 68 source-boundary tests, 124/124 total.
- Deployed `ROCK.dll` is version `0.5.0.0`, size `5,380,096`, timestamp `2026-07-20 21:00:51`; build/deploy SHA-256 match `0DDF552B345573A9DCA11B2074833EA851545C48C22330DB4DF7AE9003FBE25D`.
- Build/deploy `ROCK.pdb` SHA-256 match `AFED24D421596082AE908E3F9B298B947593E636ADC4AC338AA1263CA42A3B8C`.
- Committed Slice 4 as `2c0c365` (`fix/debug-overlay: batch ordered diagnostic rendering`).
- Post-commit regression passed 124/124 tests; `git show --check` passed and the worktree was clean before Slice 5 began.

### 2026-07-20 — Slice 5 Fidelity, Configuration, Metrics, And Module Boundaries

- Slice started from clean commit `2c0c365` after its full post-commit regression.
- Verification target: establish triangle, scaled-convex translation, and compound child layouts independently from pristine/modified CommonLib and FO4VR binary evidence before adding any new engine-memory reads.
- Neither local CommonLib tree defines concrete triangle, scaled-convex, or compound layouts, so CommonLib virtuals remain the authority for triangle vertices and read-only FO4VR Ghidra evidence is the authority for the two unavoidable concrete-layout reads.
- Ghidra confirmed that the existing ROCK/standalone scaled-convex `+0x38` scale read is incorrect: FO4VR stores scale at `+0x40` and translation at `+0x50`. The Slice 5 correction must update capture and fingerprinting together so cache identity cannot alias distinct translated shapes.
- Ghidra independently confirmed the compound root/slot layout through constructor, allocator, copier, key-mask, and consumer code. The standalone's `+0x58` root array and `+0x60` count are not valid for FO4VR; the verified root fields are `+0x60` and `+0x68` with 0x80-byte slots.
- Added a dedicated triangle recipe built from exactly three finite CommonLib support vertices and emitted double-sided. It no longer passes through generic convex centroid/radius inflation.
- Corrected scaled-convex capture and cache fingerprinting to use child `+0x30`, scale `+0x40`, and translation `+0x50`; translation is converted from Havok units and applied after component scale.
- Added static/dynamic compound capture from the verified root and slot layout. Inactive/free slots are skipped, but every active slot must have a finite transform/scale, a child pointer, and a completely capturable child recipe. Recursion, scanned slots, combined vertices, and 16-bit indices are bounded.
- Compound construction is intentionally all-or-nothing. Any invalid/unsupported child, non-finite transform result, or 16-bit combined-mesh overflow rejects the detailed cache entry so the renderer retains the full body's already-captured real AABB; it never displays a silently incomplete compound.
- Focused shape geometry and shape pipeline source-boundary checks passed. The required `custom-fast` Release build compiled, linked, and auto-deployed successfully; the complete refreshed suite passed 124/124 tests before the shape-fidelity commit.
- The deployed shape-fidelity artifact is `ROCK.dll` version `0.5.0.0`, size `5,390,848`, timestamp `2026-07-20 21:20:06`; DLL SHA-256 is `5C8F455B917B7B34283D6A6F2A065240B790A49056A87CF1405D40304EB12B5A` and PDB SHA-256 is `161E5AF12DE2F66D6041497B1FFEF3895C2461AD030696DF1579B398F4561CD1`.
- Committed the verified shape-fidelity correction as `833cc08` (`fix/debug-overlay: verify and expand shape fidelity`). Post-commit build/test regression passed 124/124 tests and `git show --check` passed from a clean worktree.
- Replaced the misleading `iDebugMaxShapeGenerationsPerFrame = 100` surface (which was silently capped to 32) with `iDebugMaxShapeCapturesPerFrame = 32`, matching the existing effective behavior and the actual publisher-side operation.
- Extracted `DebugOverlayRuntimeSettings.h` as the single pure contract for capture, convex, compound, queue, completion, upload, cache-entry, cache-byte, body-instance, line-vertex, and text-vertex defaults/hard caps. Config loading and immutable frame capture both sanitize through that contract.
- Wired every new cap to its real enforcement point: reservation/capture admission, aggregate pipeline backlog/completed queue, bounded compositor uploads, LRU entry/GPU-byte trimming, fixed body instance stream, reusable line-batch frame budget, and aggregate text generation.
- Removed the unused broad `makeOverlaySettingsKey` and stale generation clamp/name instead of retaining compatibility paths. Only geometry-affecting convex/compound settings participate in shape-cache invalidation; work/memory caps apply immediately without unnecessary geometry invalidation.
- Updated `data/config/ROCK.ini`, `data/mod/ROCK_Config/ROCK.ini`, and the active production `ROCK_Config/ROCK.ini` in place with identical keys/defaults and documented ranges. No unrelated production setting was replaced.
- Added pure runtime-limit tests and source-boundary coverage; strengthened line-batch tests so the allocation-free overload enforces each immutable frame's configured budget. The required `custom-fast` Release build compiled, linked, and auto-deployed, and the expanded complete suite passed 126/126 tests before the configuration commit.
- The deployed bounded-settings artifact is `ROCK.dll` version `0.5.0.0`, size `5,396,480`, timestamp `2026-07-20 21:35:52`; DLL SHA-256 is `D66920AC27286FF0DC326370732173FAE2B211AE09F067BD1985AA81E0D8B6A1` and PDB SHA-256 is `5A1557A47CAAF3D3359CC542346816C10B2A37DD66D2CCF1289D35AA60BB04EC`.
- Committed the bounded runtime settings as `212d9e9` (`feature/debug-overlay: expose bounded runtime budgets`). Post-commit regression passed 126/126 tests; `git show --check` passed and the worktree was clean before instrumentation began.
- Added a four-slot D3D11 timestamp/disjoint query ring owned by the D3D resource generation. It issues one RAII sample around body, line, and text draws, checks at most one older slot per admitted frame, uses `D3D11_ASYNC_GETDATA_DONOTFLUSH` for every result probe, never loops waiting, and never calls `Flush`.
- GPU query creation is optional and all-or-nothing. A failure logs once and leaves the renderer fully operational; query readiness is deliberately excluded from `D3DResources::ready()`.
- Added cumulative admission telemetry that distinguishes no-publication, duplicate-publication, active/reentrant, and serial-race rejection from successful acquisitions. The existing atomic admission invariants and lease ownership remain unchanged.
- Extracted the D3D query ring and per-frame runtime counter contract into `DebugOverlayGpuTimer` and `DebugOverlayStats`. Existing publication, shader, line-batch, shape-geometry, shape-pipeline, settings, and admission modules already own their narrow domains; the private render-pass coordinator remains together because splitting its D3D state and immutable-frame consumers would introduce cross-module lifetime coupling without removing hot-path work.
- Added source-boundary coverage requiring the fixed query ring, correct begin/end ordering, non-flushing bounded readback, optional initialization, full draw-span scope, pre-log scope closure, telemetry publication, and removal of the monolithic statistics definition. Extended admission unit coverage verifies every deterministic counter category.
- Re-scanned every CommonLib-facing call in the final overlay and compared it with both the active modified CommonLibF4VR and `original frik deps/CommonLibF4VR`. No new discrepancy was found beyond the scaled/compound layout defects already corrected in `833cc08`; the GPU timer is direct D3D11 and required no additional Ghidra claim.
- The required `custom-fast` Release plugin build compiled, linked, and auto-deployed. After refreshing `custom-tests`, the complete pre-commit suite passed 57 policy tests plus 70 source-boundary tests, 127/127 total.
- The final pre-commit instrumentation artifact is `ROCK.dll` version `0.5.0.0`, size `5,403,648`, timestamp `2026-07-20 21:52:33`; build/deploy DLL SHA-256 match `886A6FA3C840C6462DE8D3C5CD1A6BA4F9AB1912042A67DB12C615C5710A5537`, and build/deploy PDB SHA-256 match `329C385065D295F5191F8524D0EA90DCB95EAB6C3265D244A267C846EA8730E1`.
- Committed the instrumentation/module slice as `ffc7852` (`feature/debug-overlay: add nonblocking gpu instrumentation`). The required post-commit Release build auto-deployed successfully, all 127 tests passed again, and `git show --check` reported no whitespace errors.

### 2026-07-20 — Runtime Validation Boundary

- `Fallout4VR.exe` was not running during the final validation check.
- The final deployed DLL timestamp is `2026-07-20 21:52:33`, while the newest `f4sevr.log` and `ROCK.log` timestamps are `19:32:29` and `19:36:46`. Those logs predate the final artifact and therefore are not accepted as loader, hook, visual, or timing evidence for this build.
- Build, link, auto-deploy, artifact identity, source/unit/shader regressions, and CommonLib/Ghidra verification are complete. Loader smoke, both-eye visual inspection, repeated lifecycle exercises, and collection of the new GPU timing samples remain runtime checks that require a new game session.

## Remaining Risks

- The ROCK worktree was changing concurrently when this effort began. Re-check overlapping core files before every edit and commit.
- A worker adds lifecycle complexity; shutdown, invalidation, and device ownership must be explicit and tested.
- Transparent diagnostic ordering constrains batching; correct order takes priority over draw-count reduction.
- Compound and scaled-convex details are layout-sensitive and cannot be copied from standalone without FO4VR-local verification.
- The auto-deploying build modifies deployed DLL/PDB as required by workspace policy. Runtime launch/log validation still depends on whether the game can be exercised during the task.
- Structural improvements are not measured performance gains until instrumentation and runtime comparison confirm them.
