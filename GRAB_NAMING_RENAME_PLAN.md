# ROCK Grab Naming Rename Plan

Date: 2026-05-27
Repo: `ROCK`
Branch: `feature/ghidra-grab-motor-mapping`
Scope: Current dynamic loose-object grab path only

## Execution Tracking

Status: Completed
Last updated: 2026-05-27

Phase checklist:

- [x] Phase 1: Lock vocabulary and add one short rationale comment
- [x] Phase 2: Rename low-level constraint API and math helpers
- [x] Phase 3: Rename core grab-frame data structures
- [x] Phase 4: Rename runtime telemetry and helper APIs
- [x] Phase 5: Rename node/object terminology and seat terminology
- [x] Phase 6: Rename locals, debug labels, and log text
- [x] Validation: fast build
- [x] Validation: source-boundary test build
- [x] Validation: source-boundary tests
- [x] Final grep for retired stems
- [x] Git commit

Working notes:

1. Added explicit in-file tracking so the rename can resume safely if interrupted.
2. Initial scope mapping confirmed the work is concentrated in `GrabConstraint*`, `GrabCore.h`, `GrabTelemetry.h`, `Hand.h`, `Hand.cpp`, and `HandGrab.cpp`.
3. Legitimate real-hand-body/contact uses still exist and must remain untouched in semantic-contact and `_handBody` code.
4. Runtime/overlay/source-boundary consumers also needed updates because the rename crosses telemetry structs, debug logging, and test-enforced source boundaries.
5. Validation completed successfully with `custom-fast`, `ROCKPolicyTestBinaries`, and `ctest -L source-boundary`.
6. Rename implementation completed and committed on this branch.
7. Post-review consistency cleanup renamed the last low-risk leftovers: `hasPalmSeatPoint` -> `hasSeatPoint` and debug telemetry state `previous*HeldNode/Object*` -> captured/held-visual equivalents.
8. Active production INI audited in place at `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`; no stale grab-naming terms were present, so no production INI keys or comments needed edits.

## Purpose

This plan defines the rename work needed to make ROCK's current grab path readable without changing behavior.

The main problem is vocabulary drift. The live runtime model is now:

1. A hidden no-contact proxy is constraint body A.
2. The held object's rigid BODY is constraint body B.
3. Runtime angular authority comes from raw hand rotation.
4. Runtime translation authority comes from the proxy/palm anchor frame.
5. The object-side grip point is frozen in held BODY-local space.

Large parts of the code still use older names like `handBody`, `HandSpace`, `palm`, and `objectWorld` even when the value now means `authority body`, `authority frame`, `seat point`, or `captured node world`.

## Naming Goals

1. Make the body A / body B contract obvious everywhere.
2. Separate real palm-anchor data from proxy authority data.
3. Separate raw-hand frame data from hybrid authority-frame data.
4. Separate captured node space from held BODY space.
5. Keep low-level solver names readable without losing the B-relative-to-A meaning.
6. Make debug/telemetry names match production runtime semantics.

## Non-Goals

1. No behavior changes.
2. No authority-model redesign.
3. No release/deploy changes.
4. No broad cleanup outside current grab naming.

## Canonical Vocabulary To Adopt

Use this vocabulary consistently before renaming anything.

| Concept | Canonical meaning |
| --- | --- |
| `authority body` | Constraint body A. Currently the hidden grab-authority proxy. |
| `held body` | Constraint body B. The primary held object's rigid BODY. |
| `raw hand` | Root-flattened controller/hand frame used for angular intent. |
| `authority frame` | Hybrid runtime frame: raw-hand rotation plus proxy/palm translation. |
| `captured node` | The node/world transform used to freeze object-side relation data. |
| `seat point` | Hand-side contact seat used for capture or reacquire. Palm pocket or pinch pocket. |
| `pivot A` | Live hand-side authority pivot. |
| `pivot B body-local` | Frozen grip point in held BODY-local space. |
| `pivot B constraint-local` | Transform-B local pivot consumed by the solver representation. |

## Names That Should Not Be Renamed Blindly

These use `handBody` or similar terms legitimately and should not be swept up by a global replace.

1. Semantic-contact and hand-collider body IDs that really are generated hand collider bodies.
Location: `src/physics-interaction/hand/HandGrab.cpp:1619-1728`
Notes: `handBodyId` here refers to actual hand collider bodies contributing contact evidence.

2. The `_handBody` member and helpers that really refer to the live palm-anchor physics body.
Locations: `src/physics-interaction/hand/Hand.h`, `src/physics-interaction/hand/Hand.cpp:892-929`
Notes: This is still a real hand body, not the proxy authority body.

3. `tryResolveLivePalmAnchorReference(...)` and `palmAnchor*` names.
Locations: `src/physics-interaction/hand/Hand.cpp:892-1005`, `src/physics-interaction/grab/GrabTelemetry.h:87-88`
Notes: These names still match reality.

## Rename Inventory

### 1. Constraint API body-A vocabulary

Current names:
`handBodyId`, `handBodyWorld`, `palmWorldGame`, `pivotBBodyLocalHk`, `desiredBodyTransformHandSpace`

Proposed names:
`authorityBodyId`, `authorityBodyWorld`, `pivotAWorldGame`, `pivotBConstraintLocalHk`, `desiredBodyTransformAuthoritySpace`

Declaration sites:
`src/physics-interaction/grab/GrabConstraint.h:236-242`

Primary use sites:
`src/physics-interaction/grab/GrabConstraint.cpp:316-318, 432, 438, 519-523, 539, 553-565`
`src/physics-interaction/hand/HandGrab.cpp:4643-4650`

Why it is misleading:
The live caller passes proxy body A data, not literal hand-body data. `pivotBBodyLocalHk` is especially wrong because the caller passes the solver transform-B local pivot, not the BODY-local evidence pivot.

Affected files:
`src/physics-interaction/grab/GrabConstraint.h`
`src/physics-interaction/grab/GrabConstraint.cpp`
`src/physics-interaction/hand/HandGrab.cpp`

### 2. Constraint math authority-space vocabulary

Current names:
`desiredBodyToHandRotation`, `desiredBodyTransformHandSpace`, `pivotAHandBodyLocalGame`

Proposed names:
`desiredBodyToAuthorityRotation`, `desiredBodyTransformAuthoritySpace`, `pivotAAuthorityBodyLocalGame`

Declaration sites:
`src/physics-interaction/grab/GrabConstraintMath.h:45-47, 97-110, 137-152`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4461, 4601, 7590-7591, 9682`
`src/physics-interaction/grab/GrabConstraint.cpp:438`

Why it is misleading:
These helpers operate in authority/body-A space, not generic hand space.

Affected files:
`src/physics-interaction/grab/GrabConstraintMath.h`
`src/physics-interaction/grab/GrabConstraint.cpp`
`src/physics-interaction/hand/HandGrab.cpp`

### 3. Ragdoll target `BRca` / `ARcb` terminology

Current names:
`RAGDOLL_MOTOR_TARGET_BRCA`, `targetBRca`, `targetBRcaRaw`, `currentBRca`, `currentARcb`, `ragdollBRcaRowsErrorDegrees`, `ragdollARcbRowsInverseErrorDegrees`

Proposed names:
`RAGDOLL_MOTOR_TARGET_B_RELATIVE_TO_A`, `targetBRelativeToA`, `targetBRelativeToARaw`, `currentBRelativeToA`, `currentARelativeToB`, `ragdollBRelativeToARowsErrorDegrees`, `ragdollARelativeToBRowsInverseErrorDegrees`

Declaration sites:
`src/physics-interaction/grab/GrabConstraint.h:76`
`src/physics-interaction/hand/Hand.h:720, 733-736`

Primary use sites:
`src/physics-interaction/grab/GrabConstraint.cpp:355, 436, 451-452`
`src/physics-interaction/hand/HandGrab.cpp:4441-4452, 4766-4767, 9409-9496, 9841, 9905-9908`

Why it is misleading:
The current names reflect reverse-engineering shorthand, not readable solver relation names.

Affected files:
`src/physics-interaction/grab/GrabConstraint.h`
`src/physics-interaction/grab/GrabConstraint.cpp`
`src/physics-interaction/hand/Hand.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 4. Captured desired-frame family in `CanonicalGrabFrame`

Current names:
`rawHandSpace`, `rawRotationProxyHandSpace`, `rawRotationProxyBodyHandSpace`

Proposed names:
`desiredNodeInRawHandSpace`, `desiredNodeInAuthorityFrameSpace`, `desiredBodyInAuthorityFrameSpace`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:527-529`

Primary construction sites:
`src/physics-interaction/grab/GrabCore.h:1141-1144`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4729-4735, 8300-8305, 4379-4391, 8481, 9060`

Why it is misleading:
These are not hand transforms. They are captured desired node/body relations expressed in different authority frames.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 5. Captured body-A family in `CanonicalGrabFrame`, `SplitGrabFrame`, and `FrozenGrabAuthorityFrame`

Current names:
`handBodyToRawHandAtGrab`, `handBodyWorldAtGrab`, `pivotAHandBodyLocalGame`, `pivotAHandBodyLocal`

Proposed names:
`authorityBodyToRawHandAtGrab`, `authorityBodyWorldAtGrab`, `pivotAAuthorityBodyLocalGame`, `pivotAAuthorityBodyLocal`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:530, 535, 539, 863-865, 1077-1079`

Primary construction sites:
`src/physics-interaction/grab/GrabCore.h:903-904, 1137-1140`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:2558-2568, 4376-4380, 4461, 4601, 7571, 7734-7735, 8885`

Why it is misleading:
In the current live path these values describe proxy/body-A authority data, not a real hand body.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 6. Captured raw-hand-at-grab family

Current names:
`liveHandWorld`, `liveHandWorldAtGrab`

Proposed names:
`rawHandWorldAtGrab`, `rawHandWorldAtGrab`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:461, 534`

Primary use sites:
`src/physics-interaction/grab/GrabCore.h:630, 631`
`src/physics-interaction/hand/HandGrab.cpp:7570, 4375`

Why it is misleading:
These values are the raw/root-flattened hand transform captured at grab time, not a generic "live hand" snapshot distinct from raw hand.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 7. Seat-point family

Current names:
`palmSeatPointWorldAtGrab`, `palmSeatPointMode`

Proposed names:
`seatPointWorldAtGrab`, `seatPointMode`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:542, 586`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:5716, 6637, 6798, 6823, 6955, 6965, 7356, 7360, 7747, 7943, 8811, 8815`

Why it is misleading:
The field stores palm-pocket data in some grabs and pinch-pocket data in others.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 8. Captured node vs object terminology

Current names:
`heldNode`, `objectNodeWorldAtGrab`, `desiredObjectWorldAtGrab`, `desiredObjectWorld`, `objectWorldTransform`

Proposed names:
`capturedNode`, `capturedNodeWorldAtGrab`, `desiredCapturedNodeWorldAtGrab`, `desiredCapturedNodeWorld`, `capturedNodeWorldTransform`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:536-537, 603`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:6914, 7572, 4137-4171, 4359-4368, 4400-4421, 5677-5683, 6258, 6990, 7404-7422, 8317-8318, 8459-8467, 8825, 9102`

Why it is misleading:
These values are usually the captured collidable or grip-frame node, not necessarily the whole held object root. `heldNode` also reads like a persistent visual-root pointer even though capture stores `collidableNode`.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 9. Direction-ambiguous local frame names

Current names:
`bodyLocal`, `rootBodyLocal`, `ownerBodyLocal`

Proposed names:
`bodyInCapturedNodeSpace`, `bodyInRootNodeSpace`, `bodyInOwnerNodeSpace`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:466, 531-533, 1047-1048, 1068-1070`

Primary use sites:
`src/physics-interaction/grab/GrabCore.h:1110-1127`
`src/physics-interaction/hand/HandGrab.cpp:6915, 6923-6926, 7422, 9060-9102`
`src/physics-interaction/grab/GrabThreePhase.h:440-442`

Why it is misleading:
The current names hide the transform direction. These are BODY-in-node-space transforms, not arbitrary local frames.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/grab/GrabThreePhase.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 10. Grip-point local-frame names

Current names:
`gripPointLocal`, `gripEvidenceLocal`, `gripNormalLocal`

Proposed names:
`gripPointCapturedNodeLocal`, `gripEvidenceCapturedNodeLocal`, `gripNormalCapturedNodeLocal`

Declaration sites:
`src/physics-interaction/grab/GrabCore.h:470-472, 546-548`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:6927-6937, 7336-7345, 4144-4155`

Why it is misleading:
These are local to the captured node frame, not to the held BODY and not to a generic unnamed local frame.

Affected files:
`src/physics-interaction/grab/GrabCore.h`
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 11. Live palm-anchor telemetry family

Current names:
`handBodyId`, `handMotionIndex`, `handBodySource`, `handBodyWorld`, `handBodyBasis`, `rawToHandBody`, `handBodyFingerBaseLineWorld`

Proposed names:
`livePalmAnchorBodyId`, `livePalmAnchorMotionIndex`, `livePalmAnchorSource`, `livePalmAnchorWorld`, `livePalmAnchorBasis`, `rawToLivePalmAnchor`, `livePalmAnchorFingerBaseLineWorld`

Declaration sites:
`src/physics-interaction/grab/GrabTelemetry.h:72, 74, 76, 80, 101, 135, 140`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4225, 4235-4240, 4288-4290`

Why it is misleading:
These values do refer to a real palm-anchor body, but they currently share the `handBody` stem with fields that mean proxy/body-A authority data. Renaming this family avoids two different meanings sharing one stem.

Affected files:
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 12. `nativeFlattenedHandWorld` alias

Current name:
`nativeFlattenedHandWorld`

Proposed action:
Either remove it and use `rawHandWorld` directly, or rename it to `rawHandWorldTelemetryAlias` if the separate field is still needed for overlay formatting.

Declaration site:
`src/physics-interaction/grab/GrabTelemetry.h:79`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4219-4226, 4248-4261, 4315-4317`

Why it is misleading:
In current code it is assigned directly from `rawHandWorld` and no longer represents a distinct source.

Affected files:
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 13. Legacy pivot telemetry family

Current names:
`legacyConfiguredPivotAWorld`, `legacyConfiguredPivotAToPalmAnchor`, `legacyConfiguredPivotAToGrabAuthority`, `legacyConfiguredPivotAToProxyReadback`, `legacyConfiguredPivotAToRuntimePivotA`

Proposed names:
`legacyHandBasisPivotAWorld`, `legacyHandBasisPivotAToPalmAnchor`, `legacyHandBasisPivotAToGrabAuthority`, `legacyHandBasisPivotAToProxyReadback`, `legacyHandBasisPivotAToRuntimePivotA`

Declaration sites:
`src/physics-interaction/grab/GrabTelemetry.h:123, 127-130`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4226, 4250, 4261, 4317, 4436`

Why it is misleading:
The current name does not say this is a comparison-only legacy hand-basis pivot, not current runtime authority.

Affected files:
`src/physics-interaction/grab/GrabTelemetry.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 14. Pivot-A helper split

Current names:
`computeGrabPivotAWorld(...)`, `computeGrabStartupCapturePivotAWorld(...)`

Proposed names:
`computeRuntimeAuthorityPivotAWorld(...)`, `computeStartupCaptureSeatPivotAWorld(...)`

Declaration sites:
`src/physics-interaction/hand/Hand.h:398, 403`

Implementation sites:
`src/physics-interaction/hand/Hand.cpp:931-976`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4394, 5126, 5190, 5290, 5602, 7903, 8100`

Why it is misleading:
The two helpers deliberately compute different conventions, but their current names are too similar and encourage misuse.

Affected files:
`src/physics-interaction/hand/Hand.h`
`src/physics-interaction/hand/Hand.cpp`
`src/physics-interaction/hand/HandGrab.cpp`

### 15. Pivot debug snapshot naming

Current names:
`GrabPivotDebugSnapshot.handBodyWorld`, `GrabPivotDebugSnapshot.objectBodyWorld`

Proposed names:
`authorityBodyWorldPosition`, `heldBodyWorldPosition`

Declaration site:
`src/physics-interaction/hand/Hand.h:50-56`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:3985-3988, 4428-4436`

Why it is misleading:
These fields store positions, not full transforms, and the hand-side value is proxy/body-A in the active constraint path.

Affected files:
`src/physics-interaction/hand/Hand.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 16. Low-level debug-probe cleanup

Current names:
`targetBRcaRaw`, `requiredAxisProxyLocal`, `bodyAWorldBefore`, `bodyWorldBefore`

Proposed names:
`targetBRelativeToARaw`, `requiredAxisAuthorityBodyLocal`, `authorityBodyWorldBeforeSolve`, `heldBodyWorldBeforeSolve`

Declaration site:
`src/physics-interaction/hand/Hand.h:712-739`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:9418-9496, 9841-9853`

Why it is misleading:
This is debug-only data, but it still teaches the reader the wrong runtime vocabulary if left unchanged.

Affected files:
`src/physics-interaction/hand/Hand.h`
`src/physics-interaction/hand/HandGrab.cpp`

### 17. Local-variable and log-label cleanup

Current names and labels:
`desiredBodyTransformHandSpace`, `desiredBodyToHandSpace`, `currentPalmAnchorWorld`, `kGrabObjectRotationReferenceName = "rawRotationPalmTranslation"`, log labels like `handBody`, `object`, `palm`, and `target_bRca`

Proposed names and labels:
`desiredBodyTransformAuthoritySpace`, `desiredBodyToAuthoritySpace`, `currentRuntimePivotAWorld`, `kGrabObjectRotationReferenceName = "rawRotationAuthorityTranslation"` or `"rawRotationProxyTranslation"`, and log labels updated to `authorityBody`, `heldBody`, `pivotA`, `pivotBConstraintLocal`, `targetBRelativeToA`

Primary use sites:
`src/physics-interaction/hand/HandGrab.cpp:4445-4467, 4595-4601, 4394, 8009, 9841`
`src/physics-interaction/grab/GrabConstraint.cpp:445-452, 539`

Why it matters:
Even if core symbol names are fixed, stale locals and logs will keep the mental model confusing.

Affected files:
`src/physics-interaction/hand/HandGrab.cpp`
`src/physics-interaction/grab/GrabConstraint.cpp`

## Execution Order

### Phase 1: Lock vocabulary and add one short rationale comment

1. Add or update one short comment near the core frame definitions explaining `authority body`, `held body`, `authority frame`, and `captured node`.
2. Do not rename behavior yet.

### Phase 2: Rename low-level constraint API and math helpers

1. Rename the `GrabConstraint.h` function parameter family.
2. Rename the `GrabConstraintMath.h` authority-space helpers.
3. Rename `BRca` / `ARcb` identifiers in low-level solver code and debug structs.

### Phase 3: Rename core grab-frame data structures

1. Rename `CanonicalGrabFrame` authority-body fields.
2. Rename `SplitGrabFrame` and `FrozenGrabAuthorityFrame` fields to match.
3. Rename `ImmutableGrabCaptureTelemetry` fields that mirror those values.
4. Rename `bodyLocal` and `grip*Local` fields if we decide to fix direction/frame ambiguity in the same pass.

### Phase 4: Rename runtime telemetry and helper APIs

1. Rename the live palm-anchor telemetry family.
2. Rename or remove `nativeFlattenedHandWorld`.
3. Rename legacy pivot comparison fields.
4. Rename pivot-A helper functions in `Hand.h` and `Hand.cpp`.

### Phase 5: Rename node/object terminology and seat terminology

1. Rename `heldNode` and `objectNodeWorldAtGrab` families.
2. Rename `desiredObjectWorldAtGrab` and related helper names.
3. Rename `palmSeatPoint*` to `seatPoint*`.

### Phase 6: Rename locals, debug labels, and log text

1. Sweep `HandGrab.cpp` and `GrabConstraint.cpp` locals.
2. Update telemetry field labels and debug messages so they teach the new model.
3. Run a final grep for old stems.

## Search Checklist For The Actual Rename Pass

Run these searches after each phase and before the final build.

1. `handBodyToRawHandAtGrab`
2. `handBodyWorldAtGrab`
3. `pivotAHandBodyLocalGame`
4. `desiredBodyTransformHandSpace`
5. `desiredBodyToHand`
6. `pivotBBodyLocalHk`
7. `rawHandSpace`
8. `rawRotationProxyHandSpace`
9. `rawRotationProxyBodyHandSpace`
10. `palmSeatPoint`
11. `heldNode`
12. `objectNodeWorldAtGrab`
13. `desiredObjectWorldAtGrab`
14. `nativeFlattenedHandWorld`
15. `legacyConfiguredPivotA`
16. `BRca`
17. `ARcb`
18. `target_bRca`

## Validation Plan For The Future Rename Implementation

1. Build ROCK with the normal fast preset.
Command: `cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m`

2. Build source-boundary policy tests.
Command: `cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`

3. Run at least source-boundary tests.
Command: `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`

4. Final grep should show only legitimate `handBody` references in hand-collider/contact code and live palm-anchor helpers.

## Risks

1. The `handBody` stem currently means two different things in different files: real palm-anchor body and proxy/body-A authority data. The plan must rename both families deliberately, not globally.
2. `objectWorldTransform` and `desiredObjectWorld*` are widely used. Rename those only after the authority-body vocabulary is already stable, or the pass will be much harder to review.
3. Telemetry overlays and debug code may compile against these field names even if tests do not. Expect compile-driven cleanup in any consumer that includes `GrabTelemetry.h` or `GrabCore.h`.
4. Log strings should be updated in the same coherent change; otherwise the code and the logs will teach different mental models.

## Recommended Implementation Shape

1. Do the rename as one coherent branch task, but in compiler-safe phases.
2. Keep each phase behavior-preserving.
3. Prefer semantic renames over abbreviations.
4. Favor `authorityBody`, `heldBody`, `capturedNode`, and `seatPoint` over `proxy`, `object`, or `handBody` when the value is not literally those things.
5. Do not rename legitimate real-hand-body semantic contact variables unless they are part of the live palm-anchor telemetry disambiguation.
