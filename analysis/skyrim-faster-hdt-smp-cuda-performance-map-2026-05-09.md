# Skyrim HDT-SMP vs Faster HDT-SMP CUDA/Performance Map

Date: 2026-05-09

Scope:
- Reference tree A: `E:\fo4dev\skirymvr_mods\source_codes\hdtSMP64`
- Reference tree B: `E:\fo4dev\skirymvr_mods\source_codes\Faster\hdtSMP64`
- ROCK target context: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- No ROCK code changes were made for this map.

## Source Checkpoints

Older/aers checkout:
- Path: `E:\fo4dev\skirymvr_mods\source_codes\hdtSMP64`
- Branch: `master`
- Commit: `5223ee3 Check for first person skeleton on reset -.-`
- Remote: `https://github.com/aers/hdtSMP64`
- Layout: old Visual Studio solution/project layout, source under `hdtSMP64\hdtSMP64`.

Faster/DaymareOn checkout:
- Path: `E:\fo4dev\skirymvr_mods\source_codes\Faster\hdtSMP64`
- Branch: `dev`
- Commit: `c668c09 fix: null-check dynamicData before using it as vertex source (#326)`
- Remote: `https://github.com/DaymareOn/hdtSMP64`
- Layout: CMake/CommonLibSSE-NG layout, source under `src`.

Historical CUDA-bearing Faster commits checked:
- `42b8e09 We're now able to build AE/SE, AVX/AVX2/AVX512/NoAVX, CUDA/NOCUDA from the same branch...`
- `00acef3 (tag: v3.1.2-dev) Merge pull request #218 from asdasdduck/perf-1`
- `9f19508 refactor: NUKE CUDA (#237)`

## Executive Finding

The Faster README is describing a real CUDA line of development, but the current `dev` checkout on disk is not that implementation anymore. The CUDA files existed historically (`hdtCudaCollision.cu`, `hdtCudaCollision.cuh`, `hdtCudaInterface.cpp/.h`, `hdtCudaPlanarStruct.cuh`) and were removed by `9f19508 refactor: NUKE CUDA (#237)`.

Current Faster `dev` still has stale CUDA wording in `README.md` and `configs/configs.xml`, but:
- there are no `.cu` or `.cuh` files in the working tree;
- root `CMakeLists.txt` is `LANGUAGES CXX`, not CUDA;
- `src/CMakeLists.txt` does not list CUDA sources or link `cudart`;
- current `src/config.cpp` does not parse `enableCuda`;
- current CUDA references are README/config comments plus two old commented-out `m_bonesGPU`/`m_verticesGPU` lines.

So this reference gives us two maps:
- current Faster: post-CUDA CPU/TBB/Bullet-MT optimized implementation;
- historical Faster CUDA: recoverable from git history, useful as an architecture reference but not active in the checked-out branch.

## README Comparison

Older/aers README says the mod is `hdtSMP for Skyrim Special Edition`. Its stated work is cleanup and reliability:
- works with unchanged Bullet source;
- accepts both `.` and `,` decimal separators in configs;
- removes tracked armors from physics if skeleton leaves active scene;
- writes transforms during pauses;
- resets on loading screens;
- improves pause detection through menu state;
- adds large-rotation clamp/reset options;
- hard-codes a BenthicLurker skeleton exception;
- adds a debug stats console command;
- removes dependency on `hdtSSEFramework`;
- renames plugin to `hdtSMP64` and supports VR.

Faster README says the mod is `hdtSMP with CUDA for Skyrim SE/AE/VR`. Its stated additions over aers are:
- CommonLibSSE-NG build instead of static SKSE linking;
- actor FOV/angle prioritization;
- CUDA collision acceleration with CPU fallback;
- distance checks to disable far NPC physics;
- dynamic timesteps to avoid slow-frame feedback loops;
- `can-collide-with-bone`;
- `external` sharing option for inter-NPC collision meshes;
- stricter armor handling to reduce gradual FPS loss;
- prefix mechanism changed from pointer-derived to incrementing IDs;
- same-cell/near-player activity rule;
- `smp list`;
- defaultBBP mesh-name remapping;
- facegen scanning support;
- facegen bones added to head instead of NPC root.

Historical `00acef3` README has the most detailed CUDA section. It claims GPU acceleration for:
- vertex position calculations for skinned mesh bodies;
- collider bounding boxes for per-vertex and per-triangle shapes;
- aggregate bounding boxes for collider-tree internal nodes;
- collider-list building for final checks;
- sphere-sphere and sphere-triangle collision checks;
- merging collision results.

It explicitly keeps CPU work for:
- converting collision results to Bullet manifolds;
- the Bullet solver itself.

It also warns that CUDA was experimental and often only helped on a low-end CPU/high-end GPU split. On a 6850K plus 1080Ti, the README says crowded-area framerate was a little worse than CPU-only; with the same machine restricted to two CPU cores, GPU collision was around 2-3x faster for collision time.

## What The Mod Does

Both trees are HDT-SMP-style skinned mesh physics plugins. At a high level:

1. It discovers actors, armor meshes, head parts, and XML physics definitions.
2. It builds Bullet rigid bodies and constraints for bones/mesh-driven collision shapes.
3. Each frame, it reads current skeleton/mesh transforms from the game.
4. It updates skinned collision geometry from bone transforms.
5. It performs broadphase, BVH/midphase, and narrowphase collision checks.
6. It feeds contacts into Bullet as manifolds.
7. Bullet solves the physics.
8. It writes resulting bone transforms back to the game scene.

The Faster version mainly improves scheduling, culling, shape update cost, collision cost, and actor/armor lifecycle hygiene.

## Current Faster CPU/TBB Implementation

### Build And Runtime

Current Faster uses:
- CommonLibSSE-NG;
- CMake presets for noavx/AVX/AVX2/AVX512;
- Bullet;
- TBB;
- xbyak;
- spdlog;
- Detours.

The current CMake path is CPU-only even though the README still says CUDA.

### Physics World Scheduling

`src/hdtSkyrimPhysicsWorld.cpp/.h` implements a two-phase frame model:
- `FrameEvent` calculates time, reads transforms, and schedules the simulation work into a `tbb::task_group`;
- `FrameSyncEvent` waits for pending work and writes transforms back;
- shutdown waits outstanding tasks before tearing systems down.

Important details:
- `_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON)` is used in the update paths to avoid denormal slowdowns.
- `m_averageInterval += (interval - m_averageInterval) * .125f` smooths frame interval.
- effective tick is `min(m_averageInterval, m_timeTick)`.
- accumulated time is clamped to `tick * m_maxSubSteps`.
- if accumulated time is too small (`m_accumulatedInterval * 2.0f <= tick`), it skips simulation.
- `readTransform(remainingTimeStep)` happens before the background simulation.
- `writeTransform()` happens after the background task finishes.

This is the dynamic timestep behavior from the README. It trades exact fixed-step consistency for avoiding a single slow frame causing a backlog feedback loop.

### Floating-Origin Style Offset

Before Bullet stepping, Faster computes an average rigid-body center and subtracts that offset from all bodies. After stepping, it restores the offset.

Purpose:
- reduce numerical precision problems when simulation objects are far from origin;
- keep Bullet math in a smaller coordinate range.

FO4VR/ROCK mapping:
- useful as a diagnostic/policy idea for world-origin and player-space discontinuities;
- not something to copy directly into hknp without proving where the engine expects world-space transforms.

### Actor Culling And Budgeting

`ActorManager` in Faster adds a performance culling layer:
- skeletons compute squared camera distance and camera-facing score;
- skeletons are sorted by angle/distance priority;
- skeletons below `minCullingDistance` are forced active;
- `maxActiveSkeletons` limits total active physics participants;
- `autoAdjustMaxSkeletons` adjusts the cap based on average SMP main-loop time vs `budgetMs`;
- inactive skeletons disable their physics systems.

This is not just distance culling. It is a budgeted actor-activation system:
- nearest actors stay active;
- front-of-camera actors are preferred;
- same-cell and active-scene tests prevent false inactive states;
- first-person physics can be disabled by config.

FO4VR/ROCK mapping:
- useful for body/hand/weapon generated collider budgets;
- do not cull the player hands/weapons by actor distance, but do budget optional body-zone or remote actor contact work.

### Bullet World And Solver Changes

Older/aers uses a custom single-threaded `btDiscreteDynamicsWorld` path plus custom `ConstraintGroup`, `GroupConstraintSolver`, `LCP`, and `SimulationIslandManager` code.

Current Faster moved to:
- `btDiscreteDynamicsWorldMt`;
- Bullet TBB scheduler through `btGetTBBTaskScheduler()`/`btSetTaskScheduler()`;
- `btConstraintSolverPoolMt`;
- no `btBatchedConstraints` MT solver;
- explicit island unification before parallel solve.

Important Faster solver guard:
- if two dynamic bones have active contact points, they must be in the same simulation island;
- constraints between two kinematic bodies are disabled before solving;
- HDT manifolds are cleared after solving to prevent stale contacts.

FO4VR/ROCK mapping:
- the island/ownership rule is highly relevant conceptually;
- do not let parallel workers write into native hknp state independently;
- group dependent contact/drive work before any parallel processing;
- clear transient contact evidence each frame.

### Broadphase/Midphase/Narrowphase Pipeline

Current Faster collision flow in `hdtDispatcher.cpp`:
- receives Bullet broadphase overlapping pairs;
- filters non-skinned pairs and kinematic-kinematic pairs;
- collects candidate `SkinnedMeshBody*` pairs;
- builds a deduped body update list;
- builds a deduped extra vertex-shape list for triangle-vs-triangle companion shapes;
- updates each unique body only once using `tbb::parallel_for_each`;
- runs `collapseCollideL` as a fast BVH early-out;
- runs `SkinnedMeshAlgorithm::processCollision` on remaining pairs in parallel;
- clears `m_pairs`.

This is a strong reusable pattern:
- collect candidates;
- dedup expensive update sources;
- do CPU-heavy read-only shape/candidate work in parallel;
- only then flush native contacts/outputs through a controlled path.

### Current CPU Skinning Optimizations

Current Faster `SkinnedMeshBody::internalUpdate()` uses:
- pointer aliases marked `__restrict`;
- prefetching for bones and vertices;
- AVX2/FMA path under `__AVX2__`;
- unrolled fallback SSE path;
- full four-weight computation in the AVX2 path, avoiding branchy weight checks;
- no heap allocation in the hot loop.

This replaces the older branchy SSE loop and the inactive OpenCL path.

### Current Shape Update Optimizations

Current Faster `hdtSkinnedMeshShape.cpp`:
- per-vertex shape AABB update uses aligned vector stores;
- per-triangle update stays simple because comments say the compiler already auto-vectorizes well and memory bandwidth is the limiting factor;
- triangle shape `addTriangle` uses stack-local fixed arrays for up to 12 unique bones instead of heap containers;
- bones are sorted by weight/key to make collider tree behavior stable.

### Current Collision Algorithm Optimizations

Current Faster `hdtSkinnedMeshAlgorithm.cpp/.h`:
- caps collision results at `MaxCollisionCount = 512`;
- has thread-local collision result buffers and merge buffers to avoid hundreds of heap allocations per frame;
- uses a sparse generation-marked merge buffer so it does not clear a full bone-pair matrix every frame;
- tracks active cells and iterates only touched cells;
- narrows candidate collider sets by a two-stage AABB pass before actual narrowphase;
- sorts collision results by depth before merging;
- pre-scales normals/positions outside inner loops;
- uses `tbb::this_task_arena::isolate` around nested collision parallelism.

FO4VR/ROCK mapping:
- very relevant for any ROCK CPU contact prepass or generated collider analysis;
- safer than CUDA as a first optimization target.

## Older/aers GPU/OpenCL State

The older/aers checkout contains old OpenCL-style code behind `#ifdef ENABLE_CL`:
- `hdtSkinnedMeshBody.cpp` has an OpenCL `updateVertices` kernel string;
- `hdtSkinnedMeshShape.cpp` has OpenCL `updateCollider` kernel strings for vertex and triangle shapes;
- `hdtSkinnedMesh\clkernel\*.cl` files exist;
- headers reference `hdtCL`, `hdtCLKernel`, and `cl::Buffer`.

But in this checkout:
- no project-level `ENABLE_CL` definition was found;
- no `hdtCL`/`hdtCLKernel` implementation was found in this repo;
- current call sites use `internalUpdate()`, not `internalUpdateCL()`;
- the README does not claim active OpenCL support.

Conclusion: older/aers is effectively CPU/PPL for this local source tree. The OpenCL path is a dormant or framework-leftover path, likely from earlier HDT-SMP lineage.

## Historical CUDA Implementation

### Build Gates

The historical Visual Studio project at commit `42b8e09` had real CUDA build configurations:
- `SE_CUDA_*` and `AE_CUDA_*` solution configurations;
- preprocessor definition `CUDA`;
- `hdtCudaCollision.cu` compiled through `<CudaCompile>`;
- `hdtCudaCollision.cuh`, `hdtCudaInterface.cpp/.h`, and `hdtCudaPlanarStruct.cuh`;
- CUDA 10.2 build customization imports;
- links `cudart_static.lib` and `cudadevrt.lib`;
- separate `NOCUDA` configs excluded the CUDA compile item.

The later CMake transition around `00acef3` had CUDA presets and CI toolkit install steps, but the README itself warned that CUDA was currently undefined / not correctly wired for CMake. This matters: the historically real CUDA path was strongest in the old VS project era, not the current CMake `dev` checkout.

### Runtime Gates

Historical runtime gates:
- compile-time `#ifdef CUDA`;
- config `enableCuda`;
- config `cudaDevice`;
- console command `smp gpu`;
- `CudaInterface::hasCuda()` returns `enableCuda && m_enabled`;
- `m_enabled` is initialized from `cuDeviceCount() > 0`;
- no-CUDA builds warn that CUDA is not built in when the config contains CUDA keys.

Historical timing gate:
- `FrameTimer` can alternate CPU/GPU frames for `smp timing`;
- dispatcher checks `FrameTimer::cudaFrame()` while timing.

### GPU Data Model

Historical CUDA wrapper classes:
- `CudaBody`: owns GPU body data, vertex input, vertex output buffer, bone map, bone weights, bone transforms, stream.
- `CudaPerVertexShape`: owns per-vertex collider input and GPU AABB output.
- `CudaPerTriangleShape`: owns per-triangle collider input and GPU AABB output.
- `CudaMergeBuffer`: owns temporary collision merge data and copies it back for manifold creation.
- `CudaCollisionPair<T>`: batches BVH leaf-pair collision setup and launches the collision kernels.
- `CudaBuffer`, `CudaDeviceBuffer`, `CudaPooledBuffer`, `CudaBufferPool`: manage pinned host/device buffers and per-frame temporary storage.

The CUDA data layout is explicitly GPU-friendly:
- planar arrays through `hdtCudaPlanarStruct.cuh`;
- fixed structs for vertices, bones, collider inputs, AABBs, collision setup, merge data;
- device/host wrappers to keep host and CUDA structs size-compatible.

### GPU Kernels

Historical `hdtCudaCollision.cu` kernels include:
- `kernelBodyUpdate`: computes skinned vertex positions from bone transforms and weights.
- `kernelPerVertexUpdate`: computes per-vertex collider AABBs.
- `kernelPerTriangleUpdate`: computes per-triangle collider AABBs.
- `kernelBoundingBoxReduce`: computes aggregate internal-node AABBs for collider trees.
- `kernelCollision`: runs sphere-sphere and sphere-triangle narrowphase and writes merged collision contributions.
- `fullInternalUpdate`: launches body, collider, and tree-AABB updates.

The collision kernel uses:
- shared memory;
- block-size templates;
- vertex-list thresholds;
- bounding-box prefilters;
- dynamic reduction into merge buffers;
- separate penetration modes for triangle handling.

### Historical CUDA Dispatch Flow

CUDA dispatcher flow:
1. Build candidate skinned body pairs from Bullet broadphase.
2. Build a map of unique bodies/shapes to update.
3. If CUDA is enabled and available:
   - lazily create CUDA objects for bodies/shapes;
   - ensure objects are on the selected device;
   - update CPU-side bone transforms;
   - launch GPU internal update for body vertices and shape/tree AABBs;
   - synchronize body streams before using updated tree AABBs.
4. For collision pairs:
   - run `collapseCollideL` as a tree-level early-out;
   - queue `CudaCollisionPair` kernels;
   - launch merge-buffer transfer;
   - apply copied merge data back into Bullet manifolds on CPU.
5. If CUDA is not enabled or available:
   - update bodies/shapes on CPU;
   - run CPU `SkinnedMeshAlgorithm::processCollision`.

The GPU never solved Bullet constraints. It accelerated geometry/collision preparation and narrowphase; Bullet manifold creation and solver stayed CPU-side.

### Delayed Collision Gate

Historical code had `CUDA_DELAYED_COLLISIONS`:
- some CUDA collision applications could be delayed until the next dispatch;
- triangle-vs-triangle cases were applied immediately;
- delayed closures captured weak pointers to body CUDA objects so stale bodies would not be applied after removal;
- code comments warned that device changes while tasks are pending were likely broken.

This is a major caution for ROCK: deferred GPU work needs explicit lifetime epochs, device epochs, and frame ownership. Weak pointers alone would not be enough in FO4VR hknp if native bodies can be destroyed/reused.

### Why CUDA Was Removed

Commit `9f19508 refactor: NUKE CUDA (#237)` removed:
- CUDA files;
- OpenCL `clkernel` remnants;
- CUDA interface code;
- CUDA branches in collision/body/shape/dispatcher code;
- frame timer CUDA path;
- CUDA config parsing;
- about 3967 lines.

The later/current branch appears to have pursued CPU/TBB/SIMD cleanup instead. That is consistent with the historical README warning that CUDA often did not outperform the optimized CPU path on normal high-end CPU systems.

## What Is Transferable To FO4VR/ROCK

Recommended transfer ideas:

1. Candidate prepass and dedup
- Build per-frame candidate sets.
- Dedup bodies/shapes/sources before expensive updates.
- Update each generated collider/weapon/body source once per frame.
- Parallelize only read-only or locally-owned CPU work.

2. Sparse generation-marked merge buffers
- Useful for contact evidence or bone-pair/contact-pair accumulation.
- Avoid clearing large matrices every frame.
- Track active cells and iterate only touched cells.

3. Thread-local hot buffers
- Avoid per-contact and per-pair heap allocation.
- Works well for narrowphase, shape sampling, support-point fit, debug geometry collection.

4. Budgeted activation
- Use a budget similar to Faster's `budgetMs` and `maxActiveSkeletons`.
- Apply it to optional remote actor/body-zone work, not the core player hands/weapons.
- Keep the player and immediate interaction objects always active.

5. Profiler split
- Faster separates setup, hidden/background work, wait time, and write/apply time.
- ROCK already has a performance profiler; it can benefit from explicitly separating hidden CPU time from actual frame wait/apply cost.

6. CPU SIMD/TBB before CUDA
- Current Faster ended up with AVX2/FMA, prefetching, TBB, and Bullet-MT rather than current CUDA.
- For ROCK, a CPU geometry/contact prepass is lower risk and easier to verify than a CUDA path.

7. Lifecycle guards
- Faster's actor/armor/head lifecycle strictness maps well to ROCK's FRIK skeleton lifecycle and generated body lifetimes.
- Every async or cached native body path should carry a world epoch/body-bank epoch/lifecycle state.

8. Floating-origin diagnostics
- The translation-offset idea is useful as a diagnostic for world/player-space shifts.
- For ROCK, prefer logging/compensation at transform conversion boundaries rather than shifting native hknp bodies behind the engine's back.

Not recommended to transfer directly:
- Background writes into native physics state.
- A second full physics stepping model inside ROCK.
- CUDA that writes or mutates hknp/Bethesda native bodies.
- Dynamic timestep changes to the engine's own physics step.
- Bullet-specific island solver behavior.

## ROCK CUDA Assessment

CUDA is not the next practical improvement for ROCK unless profiling proves CPU narrowphase/shape generation is the dominant bottleneck.

Reasons:
- ROCK uses FO4VR hknp/native Havok, not a self-contained Bullet world.
- GPU readback latency can erase gains unless work is very large and batchable.
- Player hands/weapon/body contacts are latency-sensitive.
- Native body lifetimes are engine-owned and can change across skeleton/world lifecycle events.
- Current Faster removed CUDA after years of carrying it, and its own README said CPU-only was often faster on balanced systems.

If we ever test GPU acceleration in ROCK, the safe boundary is:
- GPU only computes candidate contacts, shape samples, support points, or broadphase evidence;
- CPU owns all native Havok writes;
- results carry frame ID, world pointer/epoch, body bank epoch, and source identity;
- stale results are discarded silently;
- feature is optional and off by default.

## Practical Implementation Plan For ROCK From This Reference

Phase 1 - CPU prepass and profiler refinement:
- Add profiler buckets for candidate collection, shape/source update, hidden worker time, wait time, native apply time.
- Add a per-frame candidate collector for generated body/weapon/hand contact work.
- Dedup source objects before expensive geometry/body frame updates.
- Use fixed/thread-local scratch buffers for contact candidate arrays.

Phase 2 - Budget policy:
- Add optional budget gates for remote/non-player contact sources.
- Keep hands, held object, active weapon, and immediate body contacts exempt.
- Log budget decisions with sampled warnings, not every frame.

Phase 3 - Sparse contact accumulation:
- Implement a generation-marked sparse accumulator for repeated body/body or bone/body contact evidence.
- Sort/weight contact evidence before final native apply.

Phase 4 - SIMD-friendly geometry hot paths:
- Audit generated collider/source sampling for structure-of-array opportunities.
- Add AVX2/FMA only where the code is already proven hot and data alignment is controlled.
- Keep scalar fallback.

Phase 5 - Optional GPU experiment only after measurement:
- Prototype outside the native write path.
- Use a single feature flag and no hard dependency on CUDA.
- Compare against CPU/TBB with real frame captures before keeping it.

## Direct Answer: "What Does Faster Do Differently?"

Compared with the older aers tree, current Faster mainly:
- modernizes build/runtime integration;
- removes old framework/static SKSE assumptions;
- moves to TBB and Bullet MT;
- adds actor activation/culling and performance budgets;
- tightens armor/head/facegen lifecycle;
- adds more XML collision control (`can-collide-with-bone`, `external`);
- optimizes CPU skinning, collider update, and collision merge paths;
- adds stronger profiling/console inspection;
- historically had CUDA, then removed it from current `dev`.

The best parts for FO4VR/ROCK are the CPU/TBB pipeline structure, deduped candidate updates, budget gating, sparse merge buffers, and lifecycle discipline. The historical CUDA implementation is useful as a design caution and a possible future experiment, not as a direct port target.

