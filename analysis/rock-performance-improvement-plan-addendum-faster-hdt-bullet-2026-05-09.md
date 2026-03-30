# ROCK Performance Improvement Plan Addendum - Faster HDT-SMP and Bullet3

Date: 2026-05-09

Scope:
- ROCK target: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Faster HDT-SMP reference: `E:\fo4dev\skirymvr_mods\source_codes\Faster\hdtSMP64`
- Bullet3 reference: `E:\fo4dev\skirymvr_mods\source_codes\bullet3`
- Existing ROCK performance note: `analysis/rock-performance-scan-2026-05-08.md`
- Existing Faster map: `analysis/skyrim-faster-hdt-smp-cuda-performance-map-2026-05-09.md`

No ROCK code changes were made for this addendum.

## Executive Decision

The Faster HDT-SMP and Bullet3 references add useful architecture patterns to the ROCK improvement plan, but they do not justify adding Bullet as a second runtime physics engine inside FO4VR.

Use Bullet3 as a pattern source for:
- candidate broadphase caches;
- proxy/AABB update rules;
- simulation-island style ownership grouping;
- bounded parallel CPU precompute;
- persistent scratch buffers;
- stale pair/contact cleanup.

Do not use Bullet3 as:
- a second solver beside FO4VR's native hknp/Havok world;
- a runtime dependency for player hand, weapon, or body collision;
- a GPU path;
- a replacement for native body lifecycle and write gates.

The safe ROCK direction is a CPU-first prepass layer that reduces how much work reaches native Havok, not an alternate physics world.

## Bullet3 Checkpoint

Local Bullet3 tree:
- Path: `E:\fo4dev\skirymvr_mods\source_codes\bullet3`
- Remote: `https://github.com/bulletphysics/bullet3`
- Commit: `63c4d67e3 add pyproject.toml`

Relevant files inspected:
- `src\BulletDynamics\Dynamics\btDiscreteDynamicsWorldMt.cpp`
- `src\BulletDynamics\Dynamics\btDiscreteDynamicsWorldMt.h`
- `src\BulletDynamics\Dynamics\btSimulationIslandManagerMt.cpp`
- `src\BulletDynamics\Dynamics\btSimulationIslandManagerMt.h`
- `src\BulletDynamics\ConstraintSolver\btSequentialImpulseConstraintSolverMt.cpp`
- `src\BulletDynamics\ConstraintSolver\btSequentialImpulseConstraintSolverMt.h`
- `src\BulletCollision\BroadphaseCollision\btDbvtBroadphase.cpp`
- `src\BulletCollision\BroadphaseCollision\btDbvtBroadphase.h`
- `src\BulletCollision\BroadphaseCollision\btDbvt.cpp`
- `src\BulletCollision\BroadphaseCollision\btDbvt.h`
- `src\BulletCollision\CollisionDispatch\btSimulationIslandManager.cpp`
- `src\BulletCollision\CollisionDispatch\btSimulationIslandManager.h`
- `src\LinearMath\TaskScheduler\btTaskScheduler.cpp`
- `src\LinearMath\btThreads.h`

## What Bullet Adds To The ROCK Plan

### 1. Candidate Broadphase Cache

Bullet's DBVT broadphase keeps proxies for objects, updates AABBs only when a proxy moves, and maintains an overlapping-pair cache. ROCK can borrow that structure without borrowing Bullet.

ROCK mapping:
- create a generated-collider candidate layer before expensive hand/body/weapon contact work;
- store compact proxies for generated body bones, hand capsules/spheres, weapon collision primitives, and optional world/object contact candidates;
- update a proxy only when its source generation, transform, shape dimensions, or lifecycle epoch changes;
- keep a pair cache keyed by stable ROCK source identity, not raw native pointers alone;
- remove stale pairs when provider generation, skeleton generation, world epoch, or generated body bank epoch changes.

Why this matters:
- ROCK currently has many systems that can update/generatedrive colliders every frame;
- a candidate cache lets us skip expensive per-source work when nothing meaningful changed;
- stale pair cleanup aligns with the lifecycle fixes already completed.

Recommended scope:
- implement a simple broadphase first, not a full DBVT port;
- start with sphere/AABB proxy overlap tests and stable arrays;
- move to a dynamic BVH only if profiler data says the simple prefilter is the bottleneck.

### 2. Ownership Islands

Bullet's MT island manager groups bodies, manifolds, and constraints before solving. It solves independent islands in parallel while keeping dependent work together.

ROCK mapping:
- group generated work by native ownership boundary before parallel precompute;
- never let two worker tasks prepare conflicting writes for the same native body or same generated body drive;
- treat each hand, held object, weapon, body zone, and actor body-bank as ownership roots;
- large or ambiguous ownership groups should run serially on the main/native apply path;
- small independent groups can use worker precompute for read-only candidate generation.

Why this matters:
- FO4VR native hknp is not under ROCK's ownership;
- concurrency bugs here would show up as stale pointer writes, duplicate drives, contact evidence races, or frame-order dependent behavior;
- the lifecycle tracker already added gates, but a performance plan must preserve those gates when adding parallelism.

### 3. Bounded CPU Worker Precompute

Bullet's task scheduler avoids nested parallel work, falls back to the main thread when work is too small, and uses grain-size thresholds. Faster HDT also uses background work only where it can safely wait before writing back.

ROCK mapping:
- worker tasks may compute candidate pairs, sorted source lists, AABBs, shape samples, and contact evidence;
- workers must not mutate native hknp bodies, create native bodies, destroy native bodies, or write game scene transforms;
- every worker result must carry frame ID, world/lifecycle epoch, provider generation, and source identity;
- stale results are dropped at the native apply gate;
- nested worker entry should fall back to inline execution to avoid recursive task fan-out;
- small workloads should run inline to avoid synchronization overhead.

What this adds to the existing profiler:
- separate `candidate collect`, `proxy update`, `worker precompute`, `worker wait`, and `native apply` buckets;
- report both hidden worker time and the amount of time the frame actually waits.

### 4. Persistent Scratch Buffers

Bullet's ray and DBVT traversal code keeps reusable stacks and avoids repeated allocation in hot paths. Faster HDT's current CPU path also uses thread-local collision buffers and sparse generation-marked merge buffers.

ROCK mapping:
- add frame-local or thread-local scratch for candidate pairs, AABB overlaps, source update lists, shape samples, and contact evidence;
- avoid heap allocation inside per-bone, per-collider, per-contact, and per-selection loops;
- use generation-marked sparse buffers when the possible pair space is large but the touched set is small;
- iterate only active cells/pairs when resolving contact evidence.

Priority targets:
- body collider update and flush;
- soft-contact candidate collection;
- weapon collision candidate work;
- selection shape casts and repeated temporary containers;
- debug overlay shape extraction when diagnostics are enabled.

### 5. Pair And Contact Lifecycle Cleanup

Bullet has explicit overlapping-pair caches and simulation manifold cleanup. Faster HDT clears transient HDT manifolds after solving so old contacts do not survive into the wrong frame.

ROCK mapping:
- contact evidence should be frame-scoped unless explicitly promoted to a damped runtime state;
- pair caches must expire on source disappearance, world change, skeleton change, provider generation change, and body bank reset;
- candidate pairs should include generation numbers to prevent raw-pointer reuse bugs;
- final native apply should verify the same gates added by the lifecycle engine fix before using cached/precomputed output.

This should be treated as part of correctness, not just performance.

## What Faster HDT Adds To The ROCK Plan

The Faster reference and the Bullet reference point in the same direction: reduce the amount of expensive collision work through culling, deduplication, batching, and safe parallel CPU work.

Useful Faster patterns:
- collect candidate pairs first;
- dedup expensive source updates before doing shape/body work;
- update each unique skinned body/source once per frame;
- use background CPU work for read-only or locally-owned computation;
- wait before writing results back into the game;
- apply budget caps to optional work, not core player interactions;
- keep stale/lost actors and armor systems from continuing to consume runtime work;
- use sparse merge buffers instead of clearing large matrices.

Useful historical CUDA lesson:
- the GPU path only accelerated geometry/collision preparation and narrowphase;
- Bullet solver/manifold application stayed CPU-side;
- Faster eventually removed CUDA from current `dev`;
- for ROCK, GPU is a later experiment only if profiling proves a large, batchable CPU contact-prepass bottleneck.

## What Not To Add

Do not add Bullet as a linked runtime library for ROCK collision.

Reasons:
- FO4VR already has a native hknp/Havok world;
- a second solver would duplicate state, time steps, contact ownership, and body lifetime;
- translating between Bullet and hknp every frame would add latency and correctness risk;
- native objects can be destroyed or reused by the engine, so external solver state is dangerous unless it is purely advisory;
- current Faster itself moved away from CUDA and relies on optimized CPU/TBB/Bullet-MT within a self-owned Bullet world, which ROCK does not have.

Do not add Bullet OpenCL/CUDA primitives.

Reasons:
- Bullet3's GPU examples are not a drop-in path for FO4VR hknp;
- GPU readback would likely erase gains for player-proximal hand/weapon work;
- player interaction latency matters more than raw throughput;
- ROCK first needs profiler-backed CPU prepass improvements.

Do not port Bullet's solver/island code into ROCK.

What we want is the grouping rule:
- independent groups can be precomputed independently;
- dependent groups stay together;
- native writes remain gated and serialized.

## Additions To The Existing ROCK Improvement Plan

### Phase A - Profiler Split And Counters

Add profiler buckets:
- generated proxy refresh;
- candidate broadphase collect;
- candidate pair count;
- source dedup count;
- worker precompute time;
- worker wait time;
- stale worker result drops;
- native apply time;
- skipped unchanged proxies;
- expired pair-cache entries.

Success condition:
- one test run can show whether time is spent in candidate collection, shape/source update, worker wait, or native apply.

Likely files:
- `src/physics-interaction/performance/PerformanceProfiler.h`
- `src/physics-interaction/performance/PerformanceProfiler.cpp`
- `src/physics-interaction/core/PhysicsInteractionFrame.inl`
- `src/physics-interaction/core/PhysicsFrameContext.h`

### Phase B - Generated Collider Proxy Cache

Add a small ROCK-owned proxy cache for generated collider sources.

Initial proxy data:
- source kind;
- stable source identity;
- lifecycle epoch/generation;
- AABB or bounding sphere;
- last transform/version stamp;
- dirty flag;
- active flag;
- last touched frame.

Initial source coverage:
- body bone colliders;
- hand generated colliders;
- weapon generated collision primitives;
- soft-contact candidate sources.

Success condition:
- unchanged sources skip expensive refresh work;
- removed sources expire cleanly;
- generated proxy stats appear in the profiler.

Likely files:
- new `src/physics-interaction/collision/GeneratedColliderProxyCache.h`
- new `src/physics-interaction/collision/GeneratedColliderProxyCache.cpp`
- `src/physics-interaction/body/BodyBoneColliderSet.cpp`
- `src/physics-interaction/hand/HandBoneColliderSet.cpp`
- `src/physics-interaction/weapon/WeaponCollision.cpp`
- `src/physics-interaction/contact/SoftContactRuntime.cpp`

### Phase C - Candidate Pair Prefilter

Use the proxy cache to generate candidate pairs before doing expensive source updates or native interaction checks.

Start simple:
- linear or bucketed AABB/sphere checks for the small player-local set;
- stable arrays and frame scratch;
- no dynamic BVH until profiler data proves it is needed.

Candidate outputs:
- hand/body candidates;
- hand/object candidates;
- weapon/object candidates;
- body/world or body/static candidates where currently expensive;
- optional remote actor/body-zone candidates only if the system later supports them.

Success condition:
- expensive contact work sees fewer candidates;
- gameplay-critical interactions still happen without noticeable missed contacts;
- profiler shows candidate count and filtered count.

Likely files:
- new `src/physics-interaction/collision/ContactCandidateCache.h`
- new `src/physics-interaction/collision/ContactCandidateCache.cpp`
- `src/physics-interaction/contact/SoftContactRuntime.cpp`
- `src/physics-interaction/weapon/WeaponCollision.cpp`
- `src/physics-interaction/core/PhysicsInteractionContacts.inl`

### Phase D - Ownership Island Policy

Add a ROCK-specific grouping policy before any parallel precompute.

Group by:
- hand side;
- held object;
- weapon authority;
- generated body bank;
- actor/provider generation;
- native body identity where applicable.

Rules:
- two candidate jobs that can touch the same native body or generated drive belong in the same island;
- ambiguous ownership runs serial;
- read-only source sampling can split only after ownership has been proven independent;
- final native writes still pass the lifecycle/write gates.

Success condition:
- worker scheduling cannot create duplicate or conflicting native apply work;
- tests can assert that shared ownership collapses into one group.

Likely files:
- new `src/physics-interaction/collision/ContactIslandPolicy.h`
- new `src/physics-interaction/collision/ContactIslandPolicy.cpp`
- `src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp`
- `src/physics-interaction/core/PhysicsFrameContext.h`

### Phase E - Bounded Worker Precompute

Add an optional CPU worker lane for candidate/contact preparation after Phase B-D are stable.

Allowed worker work:
- collect and sort candidate pairs;
- compute local-space shape samples;
- compute AABBs/bounds;
- classify contact evidence;
- prepare immutable apply packets with generation stamps.

Forbidden worker work:
- hknp body creation;
- hknp body destruction;
- hknp body writes;
- game node writes;
- provider registration changes;
- long-lived native pointer ownership.

Required gates:
- feature flag default off until measured;
- inline fallback for small work;
- anti-nesting guard;
- frame ID and epoch validation on result apply;
- profiler split between hidden worker time and actual wait time.

Likely files:
- new `src/physics-interaction/performance/PhysicsPrecomputeScheduler.h`
- new `src/physics-interaction/performance/PhysicsPrecomputeScheduler.cpp`
- `src/physics-interaction/core/PhysicsInteractionFrame.inl`
- `src/physics-interaction/core/PhysicsFrameContext.h`
- `src/RockConfig.h`
- `src/RockConfig.cpp`

### Phase F - Budgeted Optional Work

Borrow Faster's budget idea, but apply it only to optional work.

Always active:
- player hands;
- currently held object;
- active weapon collision;
- immediate body contacts that affect the player;
- lifecycle/write-gate cleanup.

Budget candidates:
- remote actor body-zone work;
- expensive debug shape extraction;
- optional static-world body contact checks;
- noncritical soft-contact refinement;
- low-priority selection extras if profiler shows pressure.

Budget inputs:
- profiler average over a window;
- candidate count;
- last player interaction time;
- distance to player/HMD where relevant;
- in-front-of-HMD score only for optional remote/actor work.

Success condition:
- the system can reduce optional work when over budget without disabling core VR interaction.

Likely files:
- new `src/physics-interaction/performance/InteractionBudgetPolicy.h`
- new `src/physics-interaction/performance/InteractionBudgetPolicy.cpp`
- `src/RockConfig.h`
- `src/RockConfig.cpp`
- `src/physics-interaction/core/PhysicsInteractionFrame.inl`

### Phase G - Sparse Contact Evidence Accumulator

Add a generation-marked accumulator for repeated contact evidence.

Use when:
- the possible pair space is larger than the touched pair set;
- contacts need damping or sorting before native apply;
- pair evidence currently uses containers that allocate or clear too much.

Data model:
- compact key for source pair;
- generation value;
- active-cell list;
- evidence score/depth/normal/position summary;
- last observed frame;
- lifecycle generation stamps.

Success condition:
- no full matrix clearing per frame;
- stale evidence expires deterministically;
- final apply can sort/weight evidence before use.

Likely files:
- new `src/physics-interaction/contact/ContactEvidenceAccumulator.h`
- new `src/physics-interaction/contact/ContactEvidenceAccumulator.cpp`
- `src/physics-interaction/contact/SoftContactRuntime.cpp`
- `src/physics-interaction/collision/ContactActivityTracker.h`

### Phase H - Optional Dynamic BVH Experiment

Only consider this after Phase B-C profiler output proves the simple prefilter is still too expensive.

Experiment shape:
- ROCK-owned dynamic AABB tree for generated proxies;
- update only dirty proxies;
- query candidate overlaps with persistent traversal stacks;
- no Bullet runtime dependency;
- benchmark against the simpler broadphase.

Success condition:
- measurable improvement over the simple cache on real FO4VR captures;
- no lifecycle or stale-pair regressions.

If not proven, discard it.

## Test And Review Additions

Add source/unit tests for:
- proxy cache expires sources on generation/epoch change;
- unchanged proxy skips refresh;
- pair cache removes stale entries;
- candidate prefilter keeps player-critical pairs;
- ownership island policy merges shared native-body work;
- worker apply packets drop stale frame/generation results;
- worker lane cannot call native write APIs;
- budget policy never disables hands, held object, active weapon, or lifecycle cleanup;
- sparse accumulator iterates only active cells and expires stale evidence.

Add runtime verification:
- profiler run with all performance features off as baseline;
- profiler run with proxy cache only;
- profiler run with candidate prefilter;
- profiler run with worker precompute, if enabled;
- compare candidate counts, filtered counts, native apply time, and frame wait time;
- inspect logs for stale result drops and pair-cache expirations.

## Concrete Plan Delta

Add these items to the ROCK improvement roadmap:

1. Keep current lifecycle engine fix as the correctness foundation.
2. Extend profiler buckets before changing behavior.
3. Add generated collider proxy cache.
4. Add simple candidate pair prefilter.
5. Add ownership island grouping.
6. Add frame/thread scratch buffers for hot paths.
7. Add sparse contact evidence accumulator where profiler shows repeated allocation or clearing.
8. Add optional bounded worker precompute after grouping exists.
9. Add optional interaction budget for noncritical work.
10. Defer dynamic BVH until simple prefilter is proven insufficient.
11. Defer CUDA/GPU until CPU prepass profiling proves a large, batchable bottleneck.
12. Do not link Bullet into ROCK as runtime physics.

## Final Recommendation

The Bullet tree is useful, but not as a library to drop into FO4VR.

The best immediate additions are:
- broadphase-style candidate/proxy caching;
- ownership-island grouping;
- persistent scratch buffers;
- sparse generation-marked contact evidence;
- bounded worker precompute with strict native-write gates.

These fit the ROCK lifecycle work already completed and attack performance without weakening FO4VR native physics safety.
