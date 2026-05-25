# Grab Surface Patch Authority Plan - 2026-05-25

Project: ROCK

Branch observed: `feature/ghidra-grab-motor-mapping`

Source authority: current local ROCK source, current tests, and existing local grab notes only.

Confidence: high for current source map, medium for runtime behavior until in-game validation is performed on bad-rotation grabs.

Status: implementation in progress for the guarded `SurfacePatchPositionPivot` path. Runtime validation is still required on bad-rotation grabs.

## Problem Statement

ROCK currently freezes one object-side BODY-local pivot B during grab creation. That pivot is derived from a selected world-space grip point. For mesh-backed grabs, the selected grip point usually comes from one visual mesh triangle or from an existing contact-patch candidate that is still constrained by the same single-pivot invariant.

The failure pattern under discussion is:

- the selected triangle is tiny, low-quality, or near an edge;
- the selected point is offset from the real palm/finger contact area;
- the selected point lands on the wrong visual child or nearby surface;
- the frozen BODY-local pivot B ends up with poor leverage or bad seating;
- rotation then feels weak or unstable even if the solver is following the captured pivot correctly.

The goal is not to make multiple physics grabs. The goal is to use a small, deduped, owner-coherent surface patch as better evidence for one final frozen BODY-local pivot.

## Current Source Map

Relevant files:

- `src/physics-interaction/grab/MeshGrab.h`
- `src/physics-interaction/grab/GrabContact.h`
- `src/physics-interaction/grab/GrabCore.h`
- `src/physics-interaction/grab/GrabMotionController.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl`
- `tests/GrabContactPatchClusterPolicyTests.cpp`
- `tests/GrabPivotAuthorityPolicyTests.cpp`
- `tests/GrabMotionControllerPolicyTests.cpp`
- `tests/HandGrabNativeBoundarySourceTests.ps1`

Current behavior summary:

- `MeshGrab.h` extracts `TriangleData { v0, v1, v2 }` from static, dynamic, and skinned `BSTriShape` geometry.
- Grab commit chooses a visual surface point from the mesh using palm-pocket, selection-hit, or palm-ray logic.
- The chosen point becomes `grabGripPoint`.
- `grabGripPoint` is converted to object-local and BODY-local data during grab-frame freeze.
- The debug overlay can draw the selected triangle, but the active solver authority is the frozen BODY-local pivot B.
- `GrabContact.h` already defines a bounded nine-point contact patch probe pattern through `kContactPatchProbePatternSampleCount = 9`.
- `HandGrab.cpp` already builds runtime contact patches with sphere casts, optional mesh recovery, same-surface clustering, and mesh snapping.
- `chooseMeshBackedPatchPivotAuthority()` now allows `SurfacePatchPositionPivot` only after mesh snap, owner/body coherence, deduped broad surface support, selection coherence, distance bounds, and meaningful pocket/score improvement.
- `GripSupportModel` exists as a separate stage for opposed pinch, long-handle, and palm-wrap evidence.

Important current invariant:

```text
mesh/contact evidence may choose a world-space point
world-space point is frozen as BODY-local pivot B
BODY-local pivot B is the runtime solver reference
mesh triangles and patch normals must not become hidden orientation authority
```

## Desired Design

Use more than one surface sample around the palm pocket, dedupe and cluster those samples, then produce one canonical pivot candidate.

The final authority still remains:

```text
one hand/proxy-side pivot A
one object-side BODY-local pivot B
one frozen hand/object rotation relation
one native grab constraint
```

The patch is allowed to improve the position of pivot B only if it passes strict gates. It must not create a second rotation basis, a second constraint, or a held-time mesh solver.

## Non-Goals

- Do not add multiple simultaneous grab constraints for one held object.
- Do not keep querying visual triangles every held frame.
- Do not use mesh triangle axes as object rotation authority.
- Do not use contact patch normals as angular authority without a separate design, tests, and runtime validation.
- Do not make broad patch pivot replacement the default fallback.
- Do not accept owner-mismatched visual triangles.
- Do not use HIGGS, web sources, Ghidra, or binary reverse engineering for this task unless separately approved.

## Proposed Runtime Model

### Step 1: Build Probe Origins

Use the existing palm-pocket frame:

- center at the active palm-pocket acquisition point;
- palm normal as probe direction basis;
- finger-forward and thumb-side vectors as tangent axes.

Use the existing bounded nine-point pattern:

```text
center
+tangent
-tangent
+bitangent
-bitangent
+tangent +bitangent
+tangent -bitangent
-tangent +bitangent
-tangent -bitangent
```

Keep the pattern bounded. Do not make probe count unbounded or proportional to mesh size.

### Step 2: Collect Surface Candidates

Each probe should collect one best local surface candidate.

Candidate sources:

- exact hknp hit on the resolved body;
- mesh-recovered hit when the cast first hits another body but the nearest visual mesh point resolves back to the selected object;
- direct mesh snap from the probe point when the hknp cast is inconclusive, if allowed by existing selection and mesh-contact policy.

Each accepted sample must record enough identity to dedupe and debug:

- world point;
- oriented normal;
- resolved body id;
- whether it was an exact body hit or mesh recovered;
- source shape pointer as a transient runtime identity, if available;
- source node pointer as a transient runtime identity, if available;
- triangle index, if available;
- source kind, if available;
- selection delta;
- palm depth and lateral distance;
- rejection reason for failed candidates.

Keep pointer identities runtime-only. Do not serialize them or rely on them across frames.

### Step 3: Dedupe Samples

Dedupe before confidence scoring so repeated probes on the same tiny triangle cannot masquerade as a broad patch.

Preferred exact key:

```text
sourceShape pointer + triangleIndex + sourceKind
```

Fallback key when triangle identity is missing:

```text
resolved body id + quantized world point + oriented normal bucket
```

Suggested first-pass thresholds:

- point epsilon: `0.5` to `1.0` game units;
- normal duplicate threshold: dot product `>= 0.98`;
- keep the sample with the best combined score:
  - closer to palm-pocket center;
  - lower cast fraction;
  - smaller selection delta;
  - exact body hit preferred over mesh recovery when positions are equivalent.

Do not dedupe across different source shapes unless the fallback point/normal epsilon proves they are the same surface. Multi-part objects can legitimately have adjacent child shapes.

### Step 4: Cluster Same-Surface Samples

Use the current same-surface clustering concept, but apply it after dedupe.

Accept a cluster only if:

- every sample resolves to the selected body or accepted same-family ownerless visual mesh;
- depth spread around the palm anchor is small;
- lateral spread remains inside the palm-pocket footprint;
- normals are coherent after orienting toward the palm;
- the cluster does not span opposite faces of a box, tray, weapon, or multi-part object.

Reject or downgrade:

- one sample only on a large or long object;
- many duplicate samples on one triangle;
- samples split across two faces;
- samples with normals that only become coherent after arbitrary flipping unrelated to palm direction;
- samples too far from the original selection hit unless the selection hit itself is known to be collision-only evidence.

### Step 5: Compute Patch Quality

Patch confidence should be explicit. Recommended metrics:

- `uniqueTriangleCount`
- `uniqueShapeCount`
- `acceptedProbeCount`
- `dedupedSampleCount`
- `clusterSampleCount`
- `rawRejectedCount`
- `duplicateRejectedCount`
- `ownerRejectedCount`
- `normalRejectedCount`
- `depthRejectedCount`
- `selectionRejectedCount`
- `patchSpanGameUnits`
- `patchDepthSpreadGameUnits`
- `patchToPocketDistanceGameUnits`
- `patchToSelectionDistanceGameUnits`
- `patchToCanonicalPivotDistanceGameUnits`
- `pivotToBodyOriginDistanceGameUnits`
- `pivotToBodyComDistanceGameUnits`, if COM is available through already-safe body/motion reads

Confidence categories:

- `None`: no valid surface samples.
- `SinglePoint`: one unique sample. Valid evidence, not broad support.
- `Line`: two or more samples mostly along one axis. Useful for handle/edge evidence, weak for roll authority.
- `Patch`: three or more non-collinear samples on one surface. Good position evidence.
- `Opposed`: samples support both sides of a pinch/wrap. Candidate for grip support model.

### Step 6: Choose One Pivot Candidate

The patch should output one candidate world point.

Candidate options:

- nearest mesh-snapped point to the fitted patch center;
- weighted centroid of deduped same-surface samples;
- clamped projection of palm pocket center onto the patch span;
- grip-support-model pivot when opposed/long-handle support is strong enough.

Initial recommendation:

1. Prefer a mesh-snapped patch center when it is owner-coherent and close to the palm pocket.
2. If the patch is only a line on a long handle, let `GripSupportModel` decide whether it can author the pivot.
3. If only one unique sample exists, keep it as evidence but do not let it replace an already valid palm-pocket mesh point unless the baseline is invalid or obviously collision fallback.
4. If the patch candidate improves seating but lacks normal confidence, allow position-only pivot replacement only through a named, tested gate.

The patch point can replace `grabGripPoint` only before the grab frame freezes. After freeze, no held-time patch updates should move pivot B.

### Step 7: Freeze BODY-Local Authority

Once selected, the candidate follows the existing authority path:

```text
selected patch point world
  -> grabGripPoint
  -> grip source evidence stored separately
  -> pivotBBodyLocalGame = worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint)
  -> freezeGrabAuthorityFrame
  -> native constraint creation
```

The selected triangle or patch must remain evidence. The solver receives the BODY-local pivot, not the source node's local triangle basis.

## Rotation Authority Implications

The surface patch should help rotation indirectly by improving pivot placement and leverage.

It should not directly define rotation axes.

Rotation can still be poor for non-pivot reasons:

- angular motor force or tau too low for the object's inertia;
- mass/inertia distribution makes rotation slow even with a good pivot;
- frozen raw hand/object angular relation is wrong;
- proxy frame basis is mismatched;
- pivot is correct but too close to COM for that object type;
- collision contacts are resisting the object after grab;
- hand/object collision suppression or restore state is wrong;
- loose weapon or long-object scaling is misclassified.

Therefore the implementation must log enough to separate:

```text
bad source point
bad BODY-local pivot
bad lever arm
bad angular motor authority
bad frame/basis relation
external collision resistance
```

Do not assume every poor rotation case is a mesh triangle problem.

## Authority Gates

A patch may be eligible to replace pivot B only when all required gates pass:

- target is not an actor-driven grab path;
- target is not hand-pocket-only unless that path explicitly opts in;
- target is not far actor equipment handoff;
- no authored grab node is present, unless authored nodes explicitly yield;
- baseline pivot is valid or baseline is collision fallback;
- patch has at least two unique samples, or one sample with a clear small-object exception;
- patch is mesh-snapped to the selected object or exact-body matched;
- source ownership matches resolved body, or accepted same-family ownerless visual mesh rule passes;
- patch point is close to palm-pocket center;
- patch point is coherent with selection hit, when selection hit is present and trusted;
- patch does not cross multiple faces or unrelated child nodes;
- patch improves seating by a meaningful margin;
- patch does not move pivot farther than the bounded authority delta;
- patch replacement is position-only unless a separate normal/axis policy is added later.

Recommended named gate:

```text
SurfacePatchPositionPivot
```

This name should appear in policy tests, source-boundary tests, logs, and telemetry if implemented.

## Dedupe And Multi-Triangle Issues

### Same Triangle Hit By Multiple Probes

If several probes snap to the same triangle, that proves the triangle is reachable, not that the surface patch is broad.

Expected handling:

- keep one representative;
- count duplicates separately;
- classify confidence as `SinglePoint` unless other unique triangles survive.

### Adjacent Triangles On One Surface

This is the ideal case.

Expected handling:

- dedupe keeps separate triangle ids;
- normal/depth cluster accepts them;
- fitted patch or mesh-snapped center can become a strong position candidate.

### Tiny Triangle Next To Large Triangle

Tiny triangles can pull nearest-point queries to an edge.

Expected handling:

- prefer the cluster center over the first triangle hit;
- include triangle area in scoring if cheap to compute from stored vertices;
- reject degenerate triangles;
- avoid using normal from tiny/near-degenerate triangles as trusted orientation.

### Low-Poly Handles And Rods

A rod may produce a line of samples instead of a planar patch.

Expected handling:

- classify as `Line`;
- allow position evidence;
- do not claim roll authority from same-side line samples;
- let `GripSupportModel` decide if long-handle axis support can author a pivot.

### Boxes, Trays, And Corners

Probes can wrap around an edge and hit two faces.

Expected handling:

- same-surface cluster rejects split-face samples;
- if one face wins, use only that face;
- if no face wins, keep baseline pivot.

### Small Objects

Small props may not produce three unique samples even with nine probes.

Expected handling:

- scale spacing/radius using existing object lever logic;
- allow a two-sample patch to improve position if selection and palm-pocket coherence are strong;
- allow one-sample replacement only when baseline is invalid/collision fallback and object extent is below a small-object threshold;
- never require large patch span for tiny props.

### Multi-Body Or Ownerless Visual Meshes

Rendered mesh evidence can live under child nodes that do not map one-to-one to the hknp body.

Expected handling:

- continue using existing accepted body set and ownerless same-family rules;
- do not accept a concrete owner mismatch;
- record whether each sample was exact body, mesh recovered, or ownerless visual-family accepted;
- if sample ownership is mixed, reject pivot replacement but keep diagnostics.

## Data Model Plan

Keep pure math decoupled from RE types.

Preferred approach:

1. Add a small runtime surface identity wrapper in `HandGrab.cpp` or a grab-local helper, not directly in pure math if it needs RE pointers.
2. If pure policy tests need identity, use numeric fields only:
   - `surfaceKey`;
   - `triangleIndex`;
   - `sourceKindId`;
   - `sampleRole`.
3. Keep `GrabContactPatchSample<Vector>` usable by pure math tests.
4. Convert runtime samples to policy samples after dedupe.

Possible structs:

```cpp
struct RuntimeSurfacePatchSample
{
    grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3> sample;
    std::uintptr_t sourceShape = 0;
    std::uintptr_t sourceNode = 0;
    std::uint32_t triangleIndex = 0xFFFF'FFFF;
    std::uint32_t surfaceKey = 0;
    const char* sourceKind = "none";
    const char* acceptanceReason = "none";
    bool hasTriangle = false;
    bool exactBodyHit = false;
    bool meshRecovered = false;
    bool ownerlessVisualFamily = false;
};
```

The exact shape of this struct should follow existing style and avoid heap churn in the grab path.

## Scoring Plan

A patch candidate score should not be one magic distance.

Inputs:

- distance from palm-pocket center;
- distance from original selected mesh/collision point;
- distance from current canonical pivot;
- owner confidence;
- unique triangle count;
- patch confidence category;
- duplicate ratio;
- body-origin or COM lever quality;
- long-object extent;
- small-object extent;
- normal trust, only as evidence quality;
- grip support model confidence.

Suggested scoring shape:

```text
score =
    pocketDistance
  + selectionDeltaPenalty
  + authorityDeltaPenalty
  + duplicatePenalty
  + ownerRecoveryPenalty
  - confidenceBonus
  - supportModelBonus
```

Use clear gates before score. Score should choose between plausible candidates, not rescue unsafe candidates.

Review current `longLeverGameUnits` use before changing behavior. It estimates mesh extent from the candidate point, but poor rotation may require a more direct pivot-to-body-origin or pivot-to-COM metric. Do not assume long mesh extent equals good rotational leverage.

## Debug And Telemetry Plan

Add or extend grab-start logs only where rate-limited and gated by existing debug settings.

Recommended fields:

- `probeCount`
- `rawAccepted`
- `deduped`
- `duplicates`
- `clusterSamples`
- `uniqueTriangles`
- `uniqueShapes`
- `exactBodySamples`
- `meshRecoveredSamples`
- `ownerlessVisualSamples`
- `clusterReason`
- `pivotAuthoritySource`
- `pivotAuthorityReason`
- `baselinePoint`
- `patchPoint`
- `pivotShift`
- `pocketDistance`
- `selectionDelta`
- `patchSpan`
- `depthSpread`
- `pivotToBodyOrigin`
- `pivotToCom`
- `longLever`
- `angularErrorAtGrab`
- `angularErrorAfterSettle`, if already available from existing held telemetry

Overlay additions:

- draw deduped patch samples with a different role from raw probe hits;
- draw rejected duplicates only when verbose overlay is enabled;
- draw selected patch center and baseline pivot together;
- keep the current selected source triangle overlay.

Do not make overlay text or lines unbounded.

## Implementation Phases

### Phase 0: Baseline Verification

Map current behavior on at least three representative runtime cases:

- good broad-surface grab;
- bad small/tiny-triangle grab;
- bad long/rod/handle grab.

Capture:

- selected triangle;
- current pivot B;
- contact patch sample count;
- patch reason;
- pivot authority reason;
- angular error and solver behavior.

No behavior changes in this phase.

### Phase 1: Explicit Patch Sample Identity And Dedupe

Add runtime metadata for contact patch samples and dedupe them before clustering.

Expected code touch points:

- `RuntimeGrabContactPatch` in `HandGrab.cpp`;
- sample collection inside `buildRuntimeGrabContactPatch()`;
- pure dedupe helper if it can be expressed without RE pointers;
- debug snapshot/log fields.

Tests:

- duplicate triangle samples count as one unique sample;
- same-point fallback dedupe works without triangle identity;
- adjacent triangles on same surface remain distinct;
- owner mismatch is rejected before dedupe can hide it;
- dedupe output is bounded to the configured max sample count.

### Phase 2: Patch Quality Classification

Add explicit confidence classification from deduped samples.

Expected code touch points:

- `GrabContact.h` pure policy;
- `GrabContactPatchClusterPolicyTests.cpp`;
- `GrabMotionControllerPolicyTests.cpp` if the classification affects motion scaling;
- source-boundary tests for stored telemetry fields.

Tests:

- one unique triangle is `SinglePoint`;
- two same-surface points are `Line`;
- three non-collinear same-surface points are `Patch`;
- split-face samples are rejected or downgraded;
- line samples cannot claim roll/orientation authority.

### Phase 3: Guarded Position Pivot Candidate

Introduce a named `SurfacePatchPositionPivot` candidate that can compete with the current canonical pivot before freeze.

Important: this phase changes behavior and updates the former evidence-only policy deliberately. Weak, tiny, line-only, unsnapped, owner-mismatched, or selection-incoherent patches remain evidence only.

Expected code touch points:

- `chooseMeshBackedPatchPivotAuthority()`;
- `resolveMeshBackedGrabPivotAuthority()`;
- `inferGrabPivotAuthoritySource()`;
- `_grabFrame.pivotAuthoritySource` telemetry;
- source-boundary tests that previously required evidence-only behavior.

Tests:

- patch can replace collision fallback when all gates pass;
- patch can replace a worse mesh point only with meaningful pocket and score improvement;
- patch cannot replace authored grab node;
- patch cannot replace pinch pocket unless explicitly allowed;
- patch cannot replace palm-pocket point without a clear improvement;
- patch cannot replace pivot when owner mismatch exists;
- patch cannot replace pivot when selection delta is too large;
- patch cannot replace pivot when all probes hit one tiny triangle on a large object;
- accepted patch remains position-only.

### Phase 4: Grip Support Integration

Route line/opposed/wrap evidence through `GripSupportModel` rather than overloading same-surface patch logic.

Expected behavior:

- same-surface patch improves seating position;
- opposed pinch/palm wrap can author a grip-support pivot;
- long-handle evidence can move pivot along the handle only when support span and object long axis agree;
- no patch normal becomes rotation authority.

Tests:

- long same-face line cannot author roll authority;
- opposed contacts can author a grip-support pivot within max shift;
- support model keeps pivot shift bounded;
- small object support does not over-shift outside the object.

### Phase 5: Runtime Validation And Tuning

Run the validated build in-game on known problem objects.

Required runtime checks:

- no immediate snap/inversion on grab start;
- no worse behavior on good broad-surface grabs;
- improved pivot seating on bad small/tiny-triangle grabs;
- improved lever/rotation feel on handles where the original pivot was wrong;
- no cross-face pivot on boxes/trays;
- no owner mismatch on multipart objects;
- no sustained debug log spam.

Only after runtime validation should config defaults be changed. Prefer source constants and existing config first.

## Test Plan

Minimum for any code implementation:

```bat
cd ROCK && cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

For production plugin validation when code changes are ready:

```bat
cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

`custom-fast` auto-deploys to the local ROCK mod path. Run it only when the implementation is ready for deployed testing.

Focused test categories:

- pure policy tests for dedupe, clustering, classification, and authority gates;
- source-boundary tests for no held-time triangle rebuild;
- source-boundary tests for BODY-local pivot authority;
- source-boundary tests for position-only patch replacement;
- source-boundary tests that patch normals do not become angular authority;
- build/test validation before any commit.

## Rollback And Safety

Rollback should be a clean Git commit revert. Do not keep a hidden old/new runtime switch unless explicitly requested.

If behavior is risky, stage the implementation as separate commits:

1. docs and diagnostics;
2. dedupe/classification with no pivot replacement;
3. guarded position-only pivot replacement;
4. runtime tuning after in-game validation.

If Phase 3 causes regressions, revert only the pivot replacement while keeping dedupe diagnostics if they remain useful and tested.

## Open Questions

- Are the worst bad-rotation cases caused by bad pivot location, bad angular motor authority, or both?
- Does current `longLeverGameUnits` scoring demote the correct handle/palm point on long objects?
- Should pivot-to-COM be measured directly, and is there already a safe local source for COM in this path?
- Should single-sample small-object patch replacement be allowed, or should small props stay with palm-pocket mesh point plus grip support?
- Should authored grab nodes always override patch evidence, or should a future authored-node quality flag allow yielding?
- Does the existing seated-pivot reacquire path need to consume deduped patch confidence, or should it stay independent?

## Definition Of Done For Implementation

- The implementation keeps one frozen BODY-local pivot B.
- Multi-sample evidence is deduped before confidence scoring.
- Duplicate hits on one triangle cannot claim broad patch confidence.
- Adjacent same-surface triangles can produce a stronger position candidate.
- Split-face and owner-mismatched samples are rejected.
- Patch replacement, if enabled, is named, position-only, bounded, and tested.
- Rotation basis remains the frozen hand/object relation, not mesh axes.
- Held-time code does not rescan/rebuild live mesh triangles.
- Debug output explains why a patch was accepted or rejected.
- Policy tests and source-boundary tests cover the new invariants.
- ROCK build/tests are run before committing code.
