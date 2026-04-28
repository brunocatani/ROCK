# FO4VR Shape-Cast Selection Mapping

This note exists because ROCK selection/grab must be rebuilt on verified physics-query behavior instead of visual-mesh inference. HIGGS proves the desired behavior at the system level: close selection is a small swept sphere from the hand, far selection is a ray-limited swept sphere, and the accepted physics hit drives the later grab/pull path. The FO4VR implementation cannot copy HIGGS directly because FO4VR uses Havok hknp and Bethesda wrapper code, so the native shape-cast ABI below was checked in Ghidra before any implementation work.

## Verdict

FO4VR shape cast is viable for ROCK selection.

Use the direct `hknpWorld::CastShape` path at `0x1415A6C00` with the runtime `0x80` shape-cast query record. Do not use `0x141E47740` as the direct query layout; that function builds Bethesda's compact queued request record, which is later expanded by the engine into the direct runtime query.

## Source Evidence

- HIGGS source: `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
- ROCK current source: `src/physics-interaction/ObjectDetection.cpp`, `Hand.cpp`, `ObjectDetection.h`
- FO4VR binary: `E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe`
- Local docs checked: `libraries_and_tools/havok/functions_parsed/collision_queries.md`, `libraries_and_tools/havok/GHIDRA_CONFIRMED_ADDRESSES.md`
- Ghidra functions checked this pass:
  - `0x1415A6C00` world shape-cast wrapper
  - `0x1415A6DF0` world closest-points wrapper
  - `0x141DF8D60` `bhkWorld::PickObject`
  - `0x1415FF4E0` sphere shape creation
  - `0x141E47740` compact queued shape-cast request builder
  - `0x141E45930` queued collision-query processor
  - direct `CastShape` callers `0x1401A2260` and `0x141E21030`

## HIGGS Behavior To Preserve

HIGGS close selection uses a sphere linear cast, not an AABB plus mesh-distance gate.

- The query sphere is placed at the palm/grab start point.
- Its filter is set to the pickup query layer.
- Its radius is `NearCastRadius`.
- It is swept to `start + palmDirection * NearCastDistance`.
- All hits are collected.
- The chosen hit is the selectable hit with the smallest lateral distance from the cast ray.
- HIGGS passes the same collector as the cast collector and start collector, so start-overlap behavior is intentional.

HIGGS far selection first raycasts to clip the distance, then sphere-casts along the surviving distance with `FarCastRadius`. This makes far selection a fat physics query, not just a closest-hit ray.

## Current ROCK Gap

Current `findCloseObject()` does not match that behavior. It runs `hknpWorld::QueryAabb`, then accepts or rejects candidates through render mesh triangle extraction and `findClosestGrabPoint()`. That makes close grab depend on visual mesh availability and mesh metrics instead of collision-query truth.

Current `findFarObject()` uses `bhkWorld::PickObject`, which is a closest-hit ray path. It does not perform HIGGS' ray-limited sphere cast.

`HandState::SelectedFar` exists in `Hand.h`, but `Hand::updateSelection()` currently stores both close and far selections as `SelectedClose`. Future implementation must use the state split, not only `SelectedObject::isFarSelection`.

## Verified Native Functions

### `0x1415A6C00` - `hknpWorld::CastShape`

Confirmed wrapper behavior:

- Timer string: `TtWorldCastShape`.
- Acquires the hknp world read lock at `world + 0x690`.
- Dispatches through the query interface at `*(world + 0x178)` vtable slot `+0x68`.
- Passes the body array at `world + 0x20`, world pointer, broadphase data at `world + 0x180`, hit collector, and optional start-point collector.
- Releases the read lock internally.

ROCK callers should not add their own world lock around this wrapper.

### `0x1415A6DF0` - `hknpWorld::GetClosestPoints`

Confirmed self-locking wrapper over query-interface slot `+0x78`. This can be used later for overlap/proximity refinement, but it is not required to prove the swept-sphere selection path.

### `0x141DF8D60` - `bhkWorld::PickObject`

Confirmed ray-pick wrapper. This is valid for clipping far selection distance, but by itself it is not HIGGS-style fat selection.

### `0x1415FF4E0` - Sphere Shape Creation

Confirmed creation of an `hknpSphereShape`:

- Allocates `0x80` bytes.
- Sets the `hknpSphereShape` vtable.
- Writes radius at shape offset `+0x14`.
- Initializes convex/sphere support data.
- Returns a ref-counted shape pointer.

Implementation must define shape ownership clearly. Prefer an internal selection probe object that owns/caches the query sphere rather than leaking per-frame shapes.

## Direct Runtime Shape-Cast Query Layout

This is the layout to use for direct calls to `hknpWorld::CastShape`.

| Offset | Field | Evidence | Confidence |
|---:|---|---|---|
| `0x00` | collision filter pointer from `*(world+0x150)+0x5E8`, or custom filter | direct callers and queued expansion | High |
| `0x08` | material id, normally `0xFFFF` | direct callers | High |
| `0x0A` | padding/reserved | direct callers zero or packed with material | Medium |
| `0x0C` | collision filter info | direct callers and AABB query parity | High |
| `0x10` | reserved / collector output pointer in some paths, observed zero for direct casts | direct callers | Medium |
| `0x18` | reserved byte/flags, observed zero | direct callers | Medium |
| `0x20` | query shape pointer | direct callers | High |
| `0x28` | reserved/padding | direct callers | Medium |
| `0x30` | start position vector in Havok space | direct callers and queued expansion | High |
| `0x40` | displacement vector in Havok space, not normalized; W lane set to `1.0` | direct callers and queued expansion | High |
| `0x50` | precomputed inverse displacement plus sign mask | direct callers and queued expansion | High |
| `0x60` | tolerance/epsilon, observed constant `0.001` (`0x3A83126F`) | direct callers and queued expansion | High |
| `0x64..0x7F` | reserved/zero | direct callers and queued stride | Medium |

Direct runtime stride is `0x80`, confirmed by `0x141E45930`, which allocates queued shape-cast runtime records as `count * 0x80` and calls `0x1415A6C00` with `base + index * 0x80`.

The displacement inverse/sign field is computed the same way in multiple engine call sites: zero-safe components, `rcpps`, one Newton refinement, and `movmskps` sign extraction. ROCK should implement this once in a small wrapper, not inline it in selection logic.

## Compact Queued Request Is Not The Direct ABI

`0x141E47740` builds a compact `0x50` queued request record:

- `+0x00..0x0F`: start vector copied from source `+0x30`
- `+0x10..0x1F`: end vector computed from start plus source direction/scalar
- `+0x20..0x2F`: compact transform/orientation data
- `+0x38`: shape pointer with ref-count increment
- `+0x40`: collision filter info
- `+0x44`: request flags/type
- `+0x48`: `0xFFFFFFFF`

That compact record is later expanded by `0x141E45930` into the runtime `0x80` query plus a `0x30` query-shape-info/transform record. ROCK should not pass this compact record directly to `CastShape`.

## Result And Ref Resolution

`hknpCollisionResult` is `0x60` bytes. The accepted hit body id is at `hitBodyInfo.m_bodyId`, offset `0x40` in the result. Current CommonLibF4VR headers match the Ghidra-observed all-hit collector usage.

Resolution path:

1. Read `hit.hitBodyInfo.m_bodyId`.
2. Resolve `bhkNPCollisionObject::Getbhk(bhkWorld, bodyId)`.
3. Use `collisionObject->sceneObject`.
4. Resolve the reference through `TESObjectREFR::FindReferenceFor3D`.
5. Do not reject only because the selected reference root lacks a direct collision object; child collision bodies are valid.

## Filter Policy Gate

The final collision filter must be deliberate and logged. Current evidence:

- Current ROCK near AABB uses `(0x000B << 16) | 45`.
- FO4VR engine interactable range scan documentation shows `0x0006002D`.
- Current ROCK far ray uses `0x02420028` through `bhkPickData`.
- HIGGS uses a pickup/custom-pick layer for its sphere casts.

Future implementation should use one explicit internal selection-query policy for close and far sphere casts, then log hit/reject layer data under verbose diagnostics. Do not silently mix AABB layer 45 and ray layer 40 unless the behavior is intentional.

## Implementation Gates

Before code changes are considered complete:

- Add one internal wrapper for direct runtime shape casts.
- Own/cached sphere shape lifetime must be explicit.
- Use `hknpAllHitsCollector` for close and far sphere casts.
- Decide start-point collector behavior. HIGGS passes the same collector for start overlaps; FO4VR supports the parameter, but the observed direct FO4VR callers often pass null.
- Close selection should choose by physics hit lateral distance to the cast path, not render triangle distance.
- Far selection should ray-clip first, then sphere-cast up to the clipped distance.
- `SelectedClose` and `SelectedFar` state routing must be real. Far must not create a close grab constraint directly.
- Active object-tree preparation and body-set fallback remain the grab preparation path after selection chooses a ref/body.
- Diagnostics must report query counts, accepted/rejected refs, layers, body ids, selected state, and primary body fallback reason.

## Planned Config Surface

No public ROCK API change is required.

Internal/INI tuning planned:

- `fNearCastRadiusGameUnits = 6.0`
- `fNearCastDistanceGameUnits = 25.0`
- `fFarCastRadiusGameUnits = 21.0`

Existing `fNearDetectionRange` should remain a compatibility alias/default source for near cast distance, and `fFarDetectionRange` should remain max far reach.

## Test Plan

Pure tests:

- close selection beats far selection for the same object.
- far selection never enters close-grab directly.
- `SelectedFar -> SelectionLocked -> Pulled` transition policy.
- rejection counters are deterministic.
- fake node-chain resolution documents the parent-walk assumption.
- direct shape-cast query wrapper fills the `0x80` runtime layout with the verified offsets.

Runtime tests:

- a bottle touching the hand selects as near, not far.
- loose weapon/object touching the hand can be grabbed.
- far pointing highlights/pulls but does not create a close constraint immediately.
- complex child-collision weapon/object still promotes the full object tree during active grab prep.
- start-overlap behavior is checked explicitly with the start-point collector enabled and disabled.

## Open Questions

- Whether ROCK should pass the same collector as `a_startPointCollector` for close selection. HIGGS does this, and it probably matters for objects already intersecting the hand sphere, but it should be runtime-verified.
- Final filter info for selection sphere casts. The wrapper can be built now, but selection policy should be chosen with layer diagnostics visible.
- Exact semantic names for the reserved direct query fields at `0x10`, `0x18`, `0x28`, and `0x64..0x7F`. Observed engine callers zero them for the direct path ROCK needs.
