# Raw-Hand Constraint Normalization Plan

Date: 2026-05-26  
Repo: `ROCK`  
Branch at planning time: `feature/ghidra-grab-motor-mapping`

## Status

Implemented in the working tree; awaiting final full-suite validation and commit.

Progress log:

- 2026-05-26: Runtime logs showed `feedErrProxyAngle=0.0deg` while
  `bRcaErrProxyAngle` stayed large, proving the target write was internally
  consistent but the live A/B solver frames were authored under a different
  contract.
- 2026-05-26: Authority origin locked to the active pivot A world point. The
  authority frame uses raw hand rotation plus active pivot A translation. This
  preserves the physical grab pivot while allowing body A and body B local
  frames to reconstruct the same raw-authority frame.
- 2026-05-26: Runtime conversion implemented: capture now stores
  `rawAuthorityObjectSpace` and `rawAuthorityBodySpace`; creation and held
  updates write `transformA = proxyBody^-1 * rawAuthorityWorld`,
  `transformB = desiredBody^-1 * rawAuthorityWorld`, and identity `target_bRca`.
- 2026-05-26: Debug/telemetry paths updated to report the new identity-target
  invariant and proxy-local signed axis decomposition from the live constraint
  frames.
- 2026-05-26: Validation so far: `custom-fast` Release build passed and
  auto-deployed; `custom-tests` policy binaries built; source-boundary ctest
  passed.

## Goal

Track the implementation of a **full raw-authority constraint-space normalization** of ROCK dynamic grab, without repeating the partial conversions that were already tried and reverted.

This plan is specifically for the "proper raw-hand conversion" contract, not for another one-off tweak to proxy rotation, transform A, or target bytes in isolation.

## Why this note exists

History already shows several partial attempts:

- `db8a05f` — `fix/grab: drive proxy rotation from raw hand authority`
- `889f5a6` — `fix/grab: revert raw-hand proxy rotation change`
- `8191665` — `fix/grab: convert raw authority target to proxy solver frame`
- `a4d305e` — `fix/grab: revert proxy-space angular target conversion`
- `025b2e9` — `fix/grab: adapt constraint A frame to raw hand`
- `4c8f5bb` — revert of `025b2e9`
- `7d9c933` — `fix/grab: adapt proxy constraint frame to raw hand`
- `c7def76` — `fix/grab: revert raw-hand constraint frame adapter`

Those commits show that parts of the problem were explored, but **the full frame contract was never converted coherently end to end**.

## Core diagnosis

The important thing to normalize is not "raw hand rotation" in the abstract. The thing that must be normalized is the **entire constraint frame contract**.

The required pieces are:

1. Body A world frame
2. `transformA` local frame
3. Body B world frame
4. `transformB` local frame
5. `target_bRca` ragdoll motor target
6. `desiredBodyWorld` telemetry / drive target
7. Pivot A / B local translations

The implementation is only correct if those seven items agree on one contract.

## Previous production contract (baseline)

The previous production behavior was effectively the coherent **proxy-A contract**:

- Body A physical body = hidden proxy body
- Body A world frame = generated palm/proxy frame with runtime seat offset
- `transformA` rotation = identity
- `desiredBodyWorld` = `makeRawRotationPalmTranslationFrame(rawHand, proxyWorld) * frozenBodyRelation`
- `transformB` / `target_bRca` = written from the current frozen body-in-hybrid-frame relation
- Pivot A = explicit local point on proxy body A
- Pivot B = explicit local point on body B using current solver-local convention

This is internally consistent enough to run, but it leaves a persistent raw-vs-proxy basis mismatch when the live proxy body A orientation differs from the raw angular target frame.

## Implemented target contract

The implemented path is the full **Option B** conversion:

- Body A physical body:
  - still the hidden proxy body
  - still physically driven from generated palm/proxy motion
- Raw authority frame:
  - raw hand rotation
  - plus the chosen proxy/pivot translation policy
- `transformA`:
  - `proxyBodyWorld^-1 * rawAuthorityFrame`
- `transformB`:
  - `desiredBodyWorld^-1 * rawAuthorityFrame`
- `target_bRca`:
  - identity-equivalent zero-error target in FO4VR's actual solver byte convention
- `desiredBodyWorld`:
  - `rawAuthorityFrame * frozenBodyInRawAuthorityFrame`
- Pivot A:
  - recomputed local to body A / frame A consistently
- Pivot B:
  - recomputed local to body B / frame B consistently

## Required invariant

At the desired pose:

- `world(transformA on body A) == rawAuthorityFrame`
- `world(transformB on body B) == rawAuthorityFrame`
- `target_bRca` encodes zero angular error between those two frames in FO4VR's real ragdoll motor convention

That is the invariant the implementation must satisfy.

## Important consequence

If we choose full Option B, then **raw-to-proxy body rotation mismatch is no longer itself the success metric**, because body A may intentionally remain a generated palm/proxy body in world space.

Under Option B, the success metrics become:

- constraint frame A world vs raw authority world
- constraint frame B world vs raw authority world
- zero-error encoding in `target_bRca`
- stable grip / pivot behavior
- no persistent long-path angular correction on left hand

## What must not happen again

This implementation must not do any of these partial states:

### Bad partial state A

- `transformA = proxyBody^-1 * rawHand`
- `transformB = old frozen body relation`
- `target_bRca = old body-to-hand relation`

This moves A to a new contract while B / target remain in the old one.

### Bad partial state B

- proxy body world rotation changed to raw-hand-like behavior
- but `transformA` / `transformB` / `target_bRca` still assume the old solver relation

This moves body A world behavior without converting the solver contract.

### Bad partial state C

- `transformB` / `target_bRca` converted to a new proxy solver space
- but frozen body relation, telemetry target, and pivot math still use the old raw-authority capture assumptions

This converts B bytes without converting the capture/telemetry contract that produced them.

## Locked architectural decision

The **raw authority frame origin** is the active pivot A world point.

The raw authority frame is:

```text
rawAuthorityWorld.rotate    = rawHandWorld.rotate
rawAuthorityWorld.translate = activePivotAWorld
rawAuthorityWorld.scale     = rawHandWorld.scale
```

This choice matters because it determines:

- whether current `rawRotationProxyBodyHandSpace` can still be reused
- whether current `pivotAHandBodyLocalGame` remains valid under the new frame contract
- how `transformB.translation` must be derived
- whether current telemetry names remain truthful

The old `rawRotationProxyBodyHandSpace` field cannot be the runtime solver
contract anymore because its origin is the proxy/seat origin. It may remain as
legacy telemetry, but runtime constraint frames must use the new
`rawAuthorityObjectSpace` and `rawAuthorityBodySpace` relations.

### Runtime equations

At capture:

```text
rawAuthorityAtGrab = raw hand rotation + pivotAWorld translation
rawAuthorityObjectSpace = inverse(rawAuthorityAtGrab) * desiredObjectWorld
rawAuthorityBodySpace   = inverse(rawAuthorityAtGrab) * desiredBodyWorld
```

At creation and held update:

```text
rawAuthorityWorld = raw hand rotation + activePivotAWorld translation
desiredObjectWorld = rawAuthorityWorld * rawAuthorityObjectSpace
desiredBodyWorld   = rawAuthorityWorld * rawAuthorityBodySpace

transformA = inverse(proxyBodyWorld) * rawAuthorityWorld
transformB = inverse(desiredBodyWorld) * rawAuthorityWorld
target_bRca = identity-equivalent zero-error target
```

Required reconstruction invariant:

```text
proxyBodyWorld * transformA == rawAuthorityWorld
desiredBodyWorld * transformB == rawAuthorityWorld
```

The physical proxy body remains driven from the generated palm/proxy frame. The
constraint frame inside body A is what adapts that physical body into raw hand
rotation space.

## Expected impact by subsystem

### `src/physics-interaction/hand/HandFrame.h`

Relevant because it defines startup capture and runtime proxy offset helpers.

Implemented role:

- keep startup capture and runtime offset helpers explicit
- do **not** silently blur generated proxy seat math with raw-authority constraint-frame math
- if a new raw-authority frame helper is needed, name it separately from the physical proxy body frame helper

### `src/physics-interaction/hand/Hand.cpp`

Relevant because it defines:

- `computeGrabPivotAWorld(...)`
- `tryComputeGrabRawRollPalmPocketPivotAWorld(...)`
- `computeGrabStartupCapturePivotAWorld(...)`

Implemented role:

- startup capture pivot logic must remain coherent with whichever translation policy is chosen for raw authority
- do not let held-update constraint-frame math quietly diverge from startup capture pivot authority

### `src/physics-interaction/hand/HandGrab.cpp`

Primary implementation site.

Critical areas:

- freeze/capture path
- `createProxyConstraintGrabDrive(...)`
- `updateProxyConstraintGrabDriveTarget(...)`
- `resolveGrabAuthorityProxyFrame(...)`
- after-solve telemetry / anomaly logging

Implemented role:

- separate these concepts explicitly:
  - physical proxy body world frame
  - raw authority world frame
  - frozen body-in-raw-authority relation
  - solver frame A local transform
  - solver frame B local transform
- avoid reusing old names if they no longer describe the new contract truthfully

### `src/physics-interaction/grab/GrabConstraint.cpp`

Relevant because this is where constraint bytes are authored.

Implemented role:

- `transformA` and `transformB` must both be authored from the same raw-authority frame contract
- `target_bRca` must be derived from the same exact contract, not from a separate shortcut path
- creation-time writes and per-frame writes must share the same math

### `src/physics-interaction/grab/GrabConstraintMath.h`

This is the single source of truth for the raw-authority contract math.

Implemented role:

- pure helpers only
- no ambiguous "hand space" names if the real meaning is "raw authority frame" or "solver frame"
- add explicit helpers for:
  - frame A derivation
  - frame B derivation
  - identity-equivalent `target_bRca` encoding
  - pivot translation derivation under the new contract

## Data contract changes

The frozen frame now carries explicit raw-authority relations while keeping the
old raw-rotation proxy fields only as legacy telemetry/capture context.

Legacy fields no longer driving the runtime solver:

- `rawRotationProxyBodyHandSpace`
- `rawRotationProxyHandSpace`
- `pivotAHandBodyLocalGame`
- `pivotBConstraintLocalGame`

New explicit runtime fields:

- `rawAuthorityObjectSpace`
- `rawAuthorityBodySpace`
- `constraintAInBodyASpace`
- `constraintBInBodyBSpace`

## Implementation phases

## Phase 0 — Lock the contract on paper

Before code:

1. Define the raw authority frame origin exactly.
2. Define whether current frozen relation fields are reusable or must be replaced.
3. Write the intended world/local equations for A, B, target, and pivots.
4. Decide the expected identity-equivalent form of `target_bRca` for FO4VR row/column conventions.

Deliverable:

- one short equations note committed before runtime changes

Status:

- Complete in this document. Authority origin is active pivot A world point.

## Phase 1 — Add pure math tests first

Add policy tests only, no runtime wiring yet.

Tests must prove:

1. Given a proxy body world frame and a raw authority world frame, computed frame A local transform reconstructs the raw authority world frame.
2. Given a desired body world frame and the same raw authority world frame, computed frame B local transform reconstructs the raw authority world frame.
3. At the desired pose, A and B world-space constraint frames match.
4. `target_bRca` written from that contract is the identity-equivalent zero-error target in the verified FO4VR encoding.
5. Pivot A and pivot B translations remain consistent with the same frame contract.

### Requirement

Do not wire runtime code until these tests exist and pass.

Status:

- Complete. `GrabAuthorityFramePolicyTests.cpp` and
  `GrabAuthorityProxyPolicyTests.cpp` cover raw-authority recomposition, body
  A/B local frame reconstruction, local-transform serialization, and identity
  `target_bRca`.

## Phase 2 — Add explicit telemetry for the raw-authority invariant

Before changing production behavior, add telemetry that can evaluate the full Option B contract.

Needed metrics:

- body A world vs raw authority world
- constraint A world vs raw authority world
- body B world vs desired body world
- constraint B world vs raw authority world
- `target_bRca` vs identity-equivalent zero-error target
- pivot A local reconstruction error
- pivot B local reconstruction error

### Important note

Existing `rawToProxy` telemetry is not sufficient for Option B, because the proxy body's own world basis may intentionally remain different from raw authority.

Status:

- Complete. Ragdoll probe telemetry treats `target_bRca` as identity and keeps
  the proxy-local signed axis dumps for body correction, live BRca correction,
  and feed mismatch.

## Phase 3 — Convert capture/freeze data model

If current frozen fields are not exact matches for the chosen authority frame, update the capture model first.

Tasks:

1. Freeze body relation in the exact raw authority frame.
2. Freeze object relation in the same frame.
3. Freeze pivot data in forms that can rebuild both A and B consistently.
4. Remove or rename ambiguous old fields only when the new fields fully replace them.

### Rule

Do not patch runtime writes around stale capture semantics.

Status:

- Complete. Freeze stores raw-authority object/BODY relations from
  `rawHandWorld.rotate + pivotAWorld.translate`.

## Phase 4 — Convert constraint creation path

Convert creation in one coherent change.

Creation must compute from one contract:

- physical proxy body world
- raw authority world
- frame A local transform
- desired body world
- frame B local transform
- pivot A translation
- pivot B translation
- `target_bRca`

### Requirement

Creation-time math must not have one-off formulas that differ from held-update math.

Status:

- Complete. Creation uses the same `computeConstraintFrameInBodySpace(...)`
  and identity-target writer as held updates.

## Phase 5 — Convert held-update path

Update path must recompute the same contract per frame.

That means:

- frame A update from live proxy body world + live raw authority world
- frame B update from live desired body world + same raw authority world
- `target_bRca` update from the same contract
- pivot translation update from the same contract

### Failure condition

If creation and update do not use the same equations, the attempt is not complete.

Status:

- Complete. Held updates recompute raw authority, desired object/BODY, frame A,
  frame B, active pivot B, and identity `target_bRca` from the same equations.

## Phase 6 — Convert telemetry and debug overlays to the new truth

Once runtime math is converted:

- update telemetry names so they match the real contract
- stop presenting proxy-body-world mismatch as the primary error metric for Option B
- add explicit constraint-frame-world comparisons instead

Status:

- Mostly complete. Existing overlay/log fields were repointed to the identity
  target invariant and raw-authority desired frames. Runtime log labels now use
  `targetRowsIdentity` / `targetColsIdentity`; field names in
  `GrabTelemetry.h` remain backward-compatible.

## Phase 7 — Remove superseded partial-contract helpers

After the new contract is stable:

- remove helpers that only exist for partial A-only or B-only adaptations
- remove stale comments describing the old contract
- remove any duplicated telemetry paths that describe the old model

Status:

- Complete for runtime helpers. The old body-to-hand target writer and dynamic
  transform-B helper were removed from `GrabConstraintMath.h`. The
  `rawRotationProxy*` frozen fields remain as legacy telemetry/capture context,
  not solver inputs.

## Validation plan

### Local build/test

Build:

```bat
cd ROCK && cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
```

Run tests:

```bat
cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

Source-boundary tests if needed:

```bat
cd ROCK && ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

Fast plugin build used for runtime testing/deploy:

```bat
cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

## Runtime validation goals

Use one left grab and one right grab of the same object.

### For Option B, success should be judged by:

1. `constraintAWorld -> rawAuthorityWorld` approaches zero rotational error
2. `constraintBWorld -> rawAuthorityWorld` approaches zero rotational error at desired pose
3. `target_bRca` logs as identity-equivalent zero-error target in the verified solver convention
4. pivot A/B reconstruction errors stay near zero
5. persistent left-hand long-path / 360-style correction disappears
6. `RAGDOLL FRAME ERROR` no longer shows the same steady mismatch pattern

### For Option B, this is **not** a required success metric:

- `rawToProxy` collapsing to near zero

That metric only belongs to a body-A-world-rotation strategy, not to a full frame-A adapter strategy.

## Risk list

### Risk 1 — Authority origin ambiguity

If translation origin is not locked first, the project will accidentally mix:

- proxy seat origin
- active pivot A origin
- startup capture pivot origin

That will invalidate A/B equality even if the rotation math looks correct.

### Risk 2 — Reusing old frozen names with new meaning

This makes debugging much harder and risks carrying stale assumptions into telemetry and update code.

### Risk 3 — `target_bRca` identity assumption may be byte-convention sensitive

The conceptual target is zero angular error. Identity rows and columns are the
stored representation used by this implementation, which keeps the byte
convention explicit and easy to verify.

### Risk 4 — Pivot B translation is easy to fake incorrectly

Old helper behavior was tied to the previous transform-B convention. This
conversion re-derives pivot B from the new frame equality.

### Risk 5 — Left-hand fix can appear solved while the contract is still wrong

A partial change may reduce the visible symptom while leaving A/B contract disagreement in place. That is not acceptable.

## Execution discipline used

1. Locked math and authority origin in this document.
2. Added/updated math policy tests.
3. Converted capture model.
4. Converted creation and held update together.
5. Repointed telemetry and source-boundary tests.
6. Removed superseded body-to-hand target helpers.
7. Runtime left/right object validation remains the next in-game check after
   deployment.

## Practical stop conditions

Stop and reassess if any of these happen:

- frame A world matches raw authority, but frame B world does not
- frame B world matches raw authority, but `target_bRca` is still non-zero-error in solver terms
- grip point looks stable but angular error remains persistent in telemetry
- left-hand symptom improves only because body A world was changed, while A/B equality is still broken

## Final summary

This work should be treated as a **single contract conversion**, not a rotation tweak.

The real definition of success is:

- one explicit raw authority frame
- one coherent A/B frame contract
- one coherent pivot contract
- one coherent `target_bRca` encoding
- one coherent `desiredBodyWorld` target

Anything less is just another partial experiment and is likely to repeat the earlier revert cycle.
