# Dynamic Worn-Body Box Sizing For ROCK Body Colliders

## Decision Note

This approach keeps ROCK's current bone-driven body collider lifecycle because those bodies already move correctly with the root-flattened FRIK skeleton and are integrated with ROCK's collision layers, metadata, debug overlays, and rebuild flow. The alternative of full mesh-accurate body collision would add much more runtime cost and instability than this feature needs. The chosen design uses worn body, clothing, and power-armor geometry only to estimate each zone's length, width, and height, then feeds those dimensions into ROCK's existing generated collider system.

## Summary

- Keep ROCK's current bone-driven body collider lifecycle and per-frame keyframed body driving.
- Change body collider shape generation from fixed capsule-like dimensions to oriented box-like convex hulls sized from worn body, clothing, and power-armor geometry.
- Use worn visual nodes as the sizing source: player body/root geometry plus equipped armor/clothing slot nodes, even if FRIK later hides some nodes for first-person comfort.
- Do not build mesh-accurate collision. Meshes only provide approximate length, width, and height per body zone.

## Key Changes

- Add a body sizing runtime that captures the current root-flattened skeleton, extracts worn mesh vertices using the existing ROCK mesh extraction path, and projects vertices into each body-zone frame.
- For each existing body descriptor, compute dynamic `length`, `width`, `height`, and optional local center offset from projected mesh bounds.
- Use existing `makeRoundedBoxHullPoints()` style geometry for body colliders instead of capsule-like hull points.
- Preserve existing body zones, power-armor profiles, collision layer behavior, metadata publication, and per-frame transform driving.
- Rebuild body collider shapes only when the skeleton/equipment/geometry/config signature changes, not every frame. Per-frame updates only move the generated bodies.

## Interfaces And Config

- Add body collider dimension data beside the existing frame result: dynamic box extents, source confidence, vertex count, and fallback reason.
- Add INI/config toggles with defaults:
  - `bBodyBoneColliderDynamicWornSizingEnabled = true`
  - `fBodyBoneColliderDynamicPaddingGameUnits = 1.5`
  - `iBodyBoneColliderDynamicMinZoneVertices = 12`
- Existing scale settings remain active and apply after dynamic sizing, so manual zone overrides can still correct edge cases.
- Add a short implementation note in the new body sizing code explaining why this is mesh-derived box sizing, not full mesh collision.

## Classification Rules

- Prefer skinned vertex influence when available: dominant bone influence maps directly to the closest matching body zone.
- For unskinned armor/equipment geometry, fall back to nearest body segment projection using the descriptor's oriented frame.
- Reject implausible dimensions through the existing role/profile limits, expanded for width/height instead of radius only.
- If a zone has too few usable vertices, keep the current descriptor default for that zone and log the fallback reason.

## Test Plan

- Add unit tests for projecting mesh points into a body-zone frame and computing stable box dimensions.
- Add tests for skinned influence mapping to torso, arm, leg, hand, and foot zones.
- Add fallback tests for unskinned equipment points and low-vertex/invalid geometry.
- Add lifecycle tests proving collider shape rebuild occurs on equipment/power-armor/config signature changes and not on ordinary per-frame motion.
- Build ROCK locally with the existing Release build command and run the ROCK test suite.

## Assumptions

- No web, no HIGGS baseline, and no Ghidra are needed for this pass.
- Existing ROCK mesh extraction offsets and skinned triangle extraction are treated as the local implementation authority.
- Production INI changes, if needed during implementation, must update the real active INI in place rather than replacing it.
