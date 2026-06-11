# ROCK Grab-Start Stutter Elimination Plan

Date: 2026-06-11

Affected project: ROCK

Source authority: current local ROCK source, current local tests, and approved local HIGGS strategy review. No web, Ghidra, or FO4 Mods MCP was used for this plan.

Confidence: high for the grab-start hitch diagnosis; medium for exact runtime cost distribution until profiler sampling is captured in game.

## Summary

ROCK stutters on grab start for dead/body grabs and loose weapon grabs because acquisition performs multiple expensive synchronous passes on the grab-press frame.

The target architecture is to remove full object-tree discovery from the grab edge. ROCK should prewarm and cache acquisition evidence while selection is stable, then start grab from a ready mechanical/visual authority snapshot. If the snapshot is not ready, grab should enter a bounded pending state and finish discovery over later frames instead of blocking one frame.

This plan targets:

- dead actor body and ragdoll grabs;
- dynamic movable static and articulated body-like grabs;
- loose weapon grabs as loose-object and multipart refs.

It does not target equipped/generated weapon collider rebuild stutters.

## Findings

Current ROCK grab startup does too much synchronous work:

- `ObjectPhysicsBodySet::scanObjectPhysicsBodySet` recursively walks the selected ref tree.
- Each collision object is expanded through native physics-system body IDs.
- Each body is resolved back to a ref and classified.
- `Hand::grabSelectedObject` scans before active prep, runs recursive motion/collision prep, then scans again.
- Close grab also extracts mesh/surface triangles for visual grip evidence.
- Nearby grab damping can scan additional nearby refs at grab begin.

HIGGS’s useful clue is strategic rather than a direct FO4VR implementation target:

- selection owns a concrete rigid body/collidable;
- physics grab starts from that body;
- connected/mechanical scope grows outward from the selected body;
- full object-tree visual proof is not the primary authority for the grab-start moment.

ROCK already has enough selection-time seed data:

- selected ref;
- selected body ID;
- hit node;
- hit point and normal;
- shape key;
- target kind;
- visual node.

## Implementation Slice Landed

The first production slice keeps the existing classifier and lifecycle policy intact while removing repeated grab-edge tree enumeration:

- Added reusable `ObjectPhysicsBodyScanCache`.
- Added `captureObjectPhysicsBodyScanCache` to prewarm selected-tree collision-object body IDs during selection.
- Added `buildObjectPhysicsBodySetFromScanCache` to rebuild live `ObjectPhysicsBodyRecord` data from cached body IDs after recursive active prep.
- Added a hand-owned `GrabAcquisitionCache` keyed by selected ref, form ID, root node, hit node, selected body ID, target kind, `bhkWorld`, `hknpWorld`, hand body IDs, source body ID, and scan depth.
- Selection update prewarms the cache for physical active-grab/dynamic-pull targets.
- `startDynamicPull` and `grabSelectedObject` try cached before-prep and prepared body sets before falling back to direct full scans.
- Cache invalidates on selection clear, selection anchor loss, release, reset, and world loss.
- Added profiler scopes/counters for acquisition body scan, active prep, mesh extraction, nearby damping begin, cache hits/misses, visited nodes, collision objects, body IDs, mesh triangles, and nearby damping motions.

This preserves multipart loose weapon ownership because it does not collapse acquisition to one selected body. Cached body IDs are still classified through the existing active-grab body policy, and prepared records are read from the live Havok world after motion/collision prep.

## Remaining Work

This slice does not yet implement the full pending-grab state. If the cache is absent or stale, ROCK currently falls back to the old direct scan so the grab remains functional. A later slice should replace that fallback for normal close grabs with a bounded pending acquisition state.

This slice does not yet move mesh/surface extraction out of the grab edge. Mesh extraction is now profiled, but the next optimization should prewarm mesh evidence or downgrade strict visual authority for mechanically valid body/articulated targets without blocking grab start.

Nearby damping is still synchronous at grab begin. It is now profiled; a later slice should stage/cap nearby ref scans or use cached nearby candidate data.

## Target Final Architecture

- Mechanical authority starts from the selected or resolved primary body.
- Mechanical scope owns held body IDs, restore, contact filtering, inertia normalization, mass scope, release velocity scope, and player-space compensation.
- Visual/grip authority uses mesh, pocket, support, and finger evidence when ready.
- For dead bodies and articulated targets, missing strict visual proof should downgrade grip quality instead of forcing a new full tree scan or rejecting a mechanically valid grab.
- Loose weapons must keep multipart body ownership and restore safety.
- Do not lower `iObjectPhysicsTreeMaxDepth` as the main fix; that risks missing multipart ownership and restore bodies.

## Validation Strategy

- Source tests must prove `grabSelectedObject` no longer has an unconditional double full-tree scan on the grab edge.
- Cache identity tests must cover ref, root node, selected body, world, target kind, and disabled/deleted ref invalidation.
- Mechanical scope tests should cover dead actor bodies, multipart loose weapons, dynamic movable statics, fixed-attached/incomplete discovery, and peer-hand adoption.
- Pending-grab tests should cover ready cache immediate grab, cache-not-ready staged completion, stale selection cancellation, and timeout fail-closed.
- Runtime validation should sample profiler output while grabbing complex loose weapons and dead bodies. Expected result for this slice: body scan cache hits during grab/pull and no repeated grab-edge tree enumeration when selection was prewarmed.

## Assumptions

- No web, Ghidra, or FO4 Mods MCP is needed for this implementation slice.
- Correctness and restore safety are more important than shaving cost by dropping bodies from ownership scope.
- If exact FO4VR native constraint adjacency becomes required, stop and request scoped Ghidra approval before reverse engineering.
