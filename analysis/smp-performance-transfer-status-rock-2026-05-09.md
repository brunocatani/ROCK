# SMP Performance Transfer Status For ROCK

Date: 2026-05-09

Scope:
- ROCK target: `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- SMP/Faster references:
  - `analysis/skyrim-hdt-smp-engine-fix-map-2026-05-09.md`
  - `analysis/skyrim-faster-hdt-smp-cuda-performance-map-2026-05-09.md`
  - `analysis/rock-performance-improvement-plan-addendum-faster-hdt-bullet-2026-05-09.md`

No ROCK code changes were made for this status note.

## Short Answer

SMP/Faster improves physics by reducing unnecessary work, batching collision work, keeping lifecycle state clean, and moving only safe CPU work off the main path. ROCK already has the correctness foundation and first instrumentation layer, but not the deeper collision acceleration layer yet.

Already implemented in ROCK:
- diagnostics profiler for hot physics/update paths;
- production debug-cost reduction;
- lifecycle generations and physics write gates;
- generated hknp body creation/drive for hands, body colliders, and weapon hulls;
- physics-substep flushing for generated keyframed bodies;
- stale source checks and rebuild-on-drive-failure behavior;
- fixed-size contact activity tracking for hand contact continuity;
- body contact ring buffer;
- shape caching for selection sphere casts;
- weapon generated-source dedup/settling and generated body banks.

Not implemented yet:
- global generated collider proxy cache;
- broadphase-style candidate pair prefilter;
- sparse generation-marked contact accumulator;
- ownership-island grouping before parallel precompute;
- bounded worker precompute lane;
- optional budget policy for noncritical work;
- dynamic BVH/DBVT-style structure;
- CUDA/GPU acceleration.

## Implemented From The SMP/Faster Idea Set

| SMP/Faster idea | ROCK status | Notes |
|---|---|---|
| Measure before optimizing | Implemented | `PerformanceProfiler` covers frame, hand/body collider update, generated flush, weapon collision, selection casts, soft contact, debug overlay, contact resolve, and native callback paths. |
| Keep timing diagnostics out of gameplay | Implemented | Profiler is opt-in and returns before high-resolution clock sampling when disabled. |
| Disable expensive debug work in production | Implemented | Packaged config defaults turn off expensive collider/skeleton/debug rendering and verbose identity logging. |
| Explicit lifecycle state | Implemented | ROCK tracks world, skeleton, provider generations, generated-body validity, stable frame count, and lifecycle reasons. |
| Block native writes during unsafe lifecycle windows | Implemented | Physics writes require matching hknp world and `PhysicsWriteAllowed`. |
| Rebuild generated bodies on world/skeleton/provider changes | Implemented | Generated bodies are invalidated and rebuilt against the current lifecycle epoch. |
| Drive generated bodies at physics-step boundary | Implemented | Hand, body, and weapon generated colliders queue transforms during frame update and flush to hknp during native physics callbacks. |
| Avoid stale generated-body motion | Implemented | Generated keyframed drive tracks source age, skips stale targets, and requests rebuilds after native drive failures. |
| Handle physics substeps | Implemented | Generated body drive can interpolate queued targets during Havok substeps and only promotes placed targets at the correct boundary. |
| Fixed-size hot contact continuity state | Implemented | `ContactActivityTracker` uses fixed slot banks, atomic pair keys, and stale pruning. |
| Recent body contact evidence cache | Implemented | `BodyContactRuntime` keeps a fixed-size ring buffer of body contact records. |
| Avoid per-frame query shape creation | Implemented | Selection sphere casts cache process-lifetime hknp sphere shapes by radius. |
| Dedup/settle generated weapon sources | Partially implemented | Weapon collision dedups TriShape source groups and waits for stable visual/generated source composition before rebuilding body banks. |
| Reusable soft-contact storage | Partially implemented | Soft contact reserves vectors each update, but still constructs per-frame vectors instead of reusing frame/thread scratch. |
| Broadphase/candidate cache | Not implemented | No global AABB/proxy cache yet. |
| Sparse merge/contact accumulator | Not implemented | Existing trackers are fixed-slot/ring-buffer, not generation-marked sparse merge buffers. |
| Worker precompute | Not implemented | ROCK currently keeps native interaction work synchronous. |
| Dynamic budget adjustment | Not implemented | Current config disables debug overhead, but there is no runtime budget policy for optional work. |
| SIMD/SoA hot geometry loops | Not implemented as a dedicated pass | Some code is allocation-conscious, but there is no Faster-style AVX2/SIMD pass. |
| CUDA/GPU acceleration | Not implemented and not recommended next | Faster's current branch removed CUDA; ROCK should only revisit GPU after CPU prepass profiling proves a large batchable bottleneck. |

## What We Can Implement Next

### 1. Profiler Split For Candidate Work

Before changing behavior, add profiler buckets and counters for:
- generated proxy refresh;
- skipped unchanged proxies;
- candidate broadphase collect;
- candidate pair count;
- filtered pair count;
- source dedup count;
- native apply time;
- stale cached result drops.

Reason:
- ROCK has broad hot-path scopes already, but not enough detail to prove which SMP-style optimization pays off.

### 2. Generated Collider Proxy Cache

Add a ROCK-owned cache of compact generated-collider proxies:
- source kind;
- stable source identity;
- lifecycle/world/skeleton/provider generation;
- AABB or bounding sphere;
- last transform/version stamp;
- dirty/active flags;
- last touched frame.

Initial sources:
- hand generated colliders;
- body bone colliders;
- weapon generated hull bodies;
- soft-contact runtime shapes.

Reason:
- this is the safest Bullet/Faster-inspired optimization and does not change native solver behavior.

### 3. Candidate Pair Prefilter

Use the proxy cache to produce candidate pairs before expensive contact work.

Start simple:
- sphere/AABB checks over small player-local arrays;
- no full DBVT/BVH yet;
- stable arrays and frame scratch;
- exact native apply gates unchanged.

Reason:
- this mirrors SMP/Faster's "collect candidates, dedup sources, update each expensive source once" model.

### 4. Persistent Scratch Buffers

Replace repeated hot-path vector construction with reusable frame/thread scratch where profiler points to pressure.

Priority targets:
- soft-contact runtime shapes;
- body/hand skeleton lookup maps;
- weapon generated source and point-cloud temporary vectors;
- selection/contact temporary arrays.

Reason:
- this is low risk and matches both Bullet and Faster's allocation-avoidance approach.

### 5. Sparse Contact Evidence Accumulator

Generalize the fixed-slot idea into a generation-marked sparse accumulator for body/hand/weapon contact evidence.

Use it for:
- touched pair lists;
- contact quality/depth/normal summaries;
- active-cell iteration;
- stale evidence expiry by frame and lifecycle generation.

Reason:
- this gives ROCK the Faster-style sparse merge-buffer benefit without introducing a second solver.

### 6. Ownership Island Policy

Group work before any parallel precompute:
- right hand;
- left hand;
- held object;
- weapon body bank;
- body collider bank;
- provider/external body owner;
- native body identity where available.

Rule:
- if two jobs may write or prepare writes for the same native body or generated drive, they belong in one island and must apply serially.

Reason:
- this borrows Bullet's island principle while preserving FO4VR native ownership.

### 7. Bounded CPU Precompute

After proxy cache and ownership grouping exist, add optional worker precompute.

Allowed worker tasks:
- compute candidate pairs;
- compute bounds;
- classify immutable contact evidence;
- sort/merge contact packets.

Forbidden worker tasks:
- create hknp bodies;
- destroy hknp bodies;
- drive hknp bodies;
- write scene nodes;
- mutate provider state.

Reason:
- this is the closest ROCK-safe equivalent to Faster's background CPU work.

### 8. Optional Budget Policy

Add dynamic budget only for noncritical work.

Never budget away:
- player hands;
- current held object;
- active weapon collision;
- lifecycle cleanup;
- write gates.

Budget candidates:
- expensive debug extraction;
- optional body/static-world checks;
- remote actor/body-zone work if added later;
- low-priority selection extras if measured hot.

Reason:
- Faster's actor budget maps to optional ROCK work, not core VR interaction.

### 9. Optional Dynamic BVH

Only build a DBVT/BVH-like structure if the simple candidate prefilter becomes the bottleneck.

Reason:
- the player-local collision set is probably small enough that a simple broadphase wins on complexity and verification cost.

### 10. CUDA/GPU Later, If Ever

Only consider GPU for read-only evidence generation after:
- CPU proxy/candidate/precompute path exists;
- profiler shows a large batchable CPU bottleneck;
- stale result drop logic is proven.

Reason:
- Faster removed CUDA in current `dev`, and FO4VR hknp native writes cannot safely move to GPU.

## Priority Order

1. Extend profiler buckets.
2. Add generated collider proxy cache.
3. Add candidate pair prefilter.
4. Convert the worst hot-path temporaries to persistent scratch.
5. Add sparse contact evidence accumulator.
6. Add ownership island policy.
7. Add bounded CPU precompute behind a default-off flag.
8. Add optional budget policy.
9. Revisit dynamic BVH only with profiler proof.
10. Revisit GPU only after all CPU-side work is measured.

## Bottom Line

ROCK already has the SMP-style safety foundation: lifecycle, write gates, generated body drives, contact routing, and profiling. The next improvement wave is not a new solver or CUDA. It is a CPU collision-prepass layer: proxy cache, candidate filtering, sparse evidence, scratch buffers, and eventually safe worker precompute.
