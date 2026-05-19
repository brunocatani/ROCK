# Grab Mesh Detection Improvement Notes

Date: 2026-05-19
Project: ROCK
Branch: feature/ghidra-grab-motor-mapping
Source authority: current local ROCK source, tests, and local git history only
Confidence: working findings; implementation design not approved yet

## Goal

Improve grab mesh detection and grip-point quality for thin/long objects such as brooms and small objects such as soap.

The user clarified that a previous mesh simplification was made while investigating grab inversions, but the inversion root cause was elsewhere. That means we can reconsider richer mesh evidence, but we still need to preserve the newer grab-frame invariant: mesh/contact evidence chooses a BODY-local point, and normals/orientation must not become a second authority path unless explicitly trusted.

## Current Pipeline Map

- Object selection still starts from hknp selection evidence. Collision identifies the reference/body and supplies a hit point/normal/shape key when available.
- Visual mesh extraction happens in `src/physics-interaction/grab/MeshGrab.h` and is consumed in `src/physics-interaction/hand/HandGrab.cpp`.
- `MeshGrab.h` extracts static, dynamic, and skinned `BSTriShape` triangles. It skips hidden nodes, skips blacklisted geometry shapes only, guards native memory reads, and has a very high `kMaxMeshExtractionTriangles = 1'000'000`.
- Dynamic skinned tri-shapes are intentionally skipped because that combined path is not verified.
- The active grab path calls `extractAllSurfaceTriangles(meshSourceNode, ..., maxDepth = 10, ...)`.
- Close grabs currently prefer palm-pocket position-only mesh authority before generic selection-hit mesh snap or palm-ray mesh fallback.
- Contact patches and multi-finger contact are mostly validation/pose evidence. They are deliberately restricted from replacing pivot B except through tightly gated mesh-backed paths, after prior inversion debugging.
- Captured local mesh is reused for long-object lever length and later seated-pivot reacquire. Runtime held finger pose does not re-solve from live body-derived triangles after the object starts settling.

## Current Relevant Defaults

- `bGrabMeshContactOnly = true`
- `bGrabRequireMeshContact = true`
- `fGrabMaxTriangleDistance = 100.0`
- `iGrabContactQualityMode = 1` (hybrid evidence)
- `iGrabMinFingerContactGroups = 3`
- `fGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0`
- `fGrabSurfaceBehindPalmToleranceGameUnits = 1.5`
- `iGrabContactPatchProbeCount = 5`
- `fGrabContactPatchProbeSpacingGameUnits = 3.0`
- `fGrabContactPatchProbeRadiusGameUnits = 2.0`
- `fGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0`
- `fGrabContactPatchMaxNormalAngleDegrees = 35.0`
- `fGrabAlignmentMaxSelectionToMeshDistance = 8.0`

## Previous Simplification Found

Commit `e92773c` (`fix/grab: simplify close mesh pivot authority`) added `findClosestGrabSurfaceHitToPointPositionOnly()` and changed close grabs so the palm-pocket mesh point is selected before generic mesh fallback. It also made contact patches and multi-finger contacts validation/pose evidence rather than broad pivot replacement sources.

That change was not a raw triangle-count optimization. It was an authority simplification: fewer evidence paths can own the frozen BODY-local pivot B. This matters because improving thin/small-object grabs should probably expand sampling and ranking while keeping the single-pivot invariant, not resurrect the old multi-authority surface-frame behavior.

## Source And Test Constraints

- `tests/HandGrabNativeBoundarySourceTests.ps1` currently requires palm-pocket mesh authority before generic mesh fallback.
- The same source-boundary test rejects `usePatchPivot = true`, so restoring patch pivot authority directly would intentionally fail current tests.
- The test also rejects `rebuildTrianglesInWorldSpace`, which protects held-object behavior from re-solving live body-derived triangles while a dynamic object is settling.
- Any richer mesh solution must preserve the explicit source-node visual evidence path and BODY-local solver pivot path. Source-node mesh evidence can choose a world-space point, but it must not inject mesh-local axes into solver authority.
- Existing policy tests already cover contact evidence and contact-patch clustering, but they do not currently cover a larger probe pattern or thin/small object probe coverage.

## Detailed Diagnosis

The weak spot is not currently mesh extraction volume. ROCK already walks visual geometry broadly and stores triangle evidence when readable. The weaker area is how much surface evidence is sampled around the palm before pivot authority is chosen.

The current contact patch pattern is:

- center
- positive palm tangent
- negative palm tangent
- positive palm bitangent
- negative palm bitangent

This works well for broad surfaces. It is much easier to miss:

- a broom handle between cardinal samples;
- a small soap-sized prop where one or two samples hit but cannot produce confident orientation;
- a low-poly/thin prop where triangle normals disagree with the selection normal;
- a render mesh whose surface is offset from the hknp hit by slightly more than the current snap/alignment gates.

The current resolver also has a tension for long objects. It uses long-lever distance in the pivot score, which is useful for stability, but that can demote an otherwise correct palm-pocket point on a broom-like object. Long-lever distance is still valuable for angular scaling after the pivot is chosen; it may be too strong as a pivot-selection penalty.

## Likely Weak Points

- Raw triangle cap is probably not the limiting factor. The extraction cap is already one million triangles, and all static/dynamic/skinned triangles are collected when readable.
- The contact patch footprint is small and fixed: center plus four cardinal palm-plane offsets. Thin objects can fall between samples, and small objects can produce only one hit, which becomes low-confidence or position-only evidence.
- The probe-count config is clamped to `1..5`, so simply increasing `iGrabContactPatchProbeCount` in the INI cannot add diagonal/ring samples without code changes.
- `fGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0` and `fGrabAlignmentMaxSelectionToMeshDistance = 8.0` may be tight for small objects whose hknp collision hit is offset from the rendered triangle surface.
- `fGrabContactPatchMaxNormalAngleDegrees = 35.0` can make patch evidence position-only on thin rods, handles, or low-poly small props where normals are noisy or only one side is hit.
- Long-object scoring includes a lever penalty. That improves stability for long props, but it may demote the correct broom-handle palm-pocket point when the selected local mesh has a very long lever.

## Improvement Options

### Option A: Conservative Sampling-Only Pass

Raise `kMaxGrabContactPatchSamples` and replace the fixed five-sample pattern with a bounded nine- or thirteen-sample pattern. Keep contact patches as validation/pose evidence only. Update config clamps and tests.

Pros:

- Lowest risk of reintroducing pivot inversions.
- Directly helps thin objects by covering diagonal gaps around the palm.
- Directly helps small objects by making it more likely to collect two or three same-surface samples.
- Easy to validate with pure policy/source tests.

Cons:

- If the bad grab point comes from pivot ranking rather than missing samples, this may improve diagnostics and pose but not fully fix grip placement.

### Option B: Sampling Plus Guarded Mesh-Backed Pivot Ranking

Do Option A, then allow a mesh-snapped contact patch to compete for position authority only when it is owner-matched, selection-coherent, close to the palm pocket, and better-scored than the canonical point by a meaningful margin. Keep normals as evidence unless explicitly trusted.

Pros:

- Best chance to improve broom and soap grab placement in one coherent change.
- Restores the useful part of richer mesh detection without making patch normals a second orientation authority.
- Can keep the source-boundary invariant by replacing the broad `usePatchPivot` path with a named, narrow gate.

Cons:

- Higher risk than sampling-only.
- Requires updating the existing source-boundary test so it rejects broad patch authority but allows the new narrow gate.
- Needs careful tests around position-only patches, normal mismatch, and selection coherence.

### Option C: Diagnostic Runtime Pass First

Add focused debug counters/log fields for contact-patch sample coverage, mesh-snap rejection reason, selection-to-mesh delta, normal-angle rejection, and long-lever score penalty.

Pros:

- Gives stronger runtime evidence before changing behavior.
- Useful if we need to tune values against real brooms/soap in game.

Cons:

- Does not directly improve grabbing yet.
- Adds logging/debug surface that must stay rate-limited and not clutter hot paths.

## Recommended First Implementation

Use Option B, but build it in two testable layers:

1. Add the larger bounded probe pattern and config clamp first.
2. Add a narrow `meshBackedPatchPivot` style gate that can replace the selected position only when all safety checks pass.

The reason to include the guarded pivot-ranking change is that the user specifically wants better grabs on thin/long and small objects, not just better diagnostics. Sampling alone is safe, but it may leave the chosen BODY-local pivot unchanged. The guarded pivot path should be position-only unless normal trust is independently proven, so it can improve where the object seats in the hand without reviving the old orientation-authority fight.

## Test Plan For Implementation

- Add a policy/source test proving the max contact patch sample count is greater than five and the config clamp accepts the new max.
- Add a policy test for the generated probe pattern: center, cardinal samples, and diagonal/ring samples should be stable and bounded.
- Add a contact-patch cluster test for a thin rod-like sample layout where cardinal-only sampling would under-sample but the expanded pattern gives enough same-surface hits.
- Add a source-boundary test that rejects broad `usePatchPivot = true` while allowing the new named narrow mesh-backed gate.
- Keep the existing tests that reject live triangle rebuilds while held and protect BODY-local pivot authority.
- Run the ROCK test build and full `ctest` suite after implementation.
