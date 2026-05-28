# Grab Authority INI Hot-Swap Plan

Date: 2026-05-28

Project: ROCK

Scope: INI-gated test plan for every grab authority fix family in `POTENTIAL_FIXES_PLAN.md`.

Source authority: user request, current local ROCK source, `POTENTIAL_FIXES_PLAN.md`.

Verification used for this plan: local source inspection only. No web, Ghidra, FO4 Mods MCP, HIGGS, or external references were used.

## Goal

Make every proposed grab authority fix testable from `ROCK.ini` while preserving current behavior as the baseline.

The intended workflow is:

1. Release any held object.
2. Remove headset.
3. Edit and save `ROCK.ini`.
4. Put headset back on.
5. Test the next grab under the selected mode.

This does not require safe mid-hold mode switching. A held object should keep the mode captured at grab start until release.

## Current Baseline

Current production behavior is mode `0`.

Mode `0` means:

| Area | Current Behavior |
| --- | --- |
| Authority translation | Palm/proxy authority body translation |
| Authority rotation | Raw root-flattened hand rotation |
| Authority frame | Hybrid raw-rotation plus proxy-translation frame |
| Pivot A freeze | Current split frame/proxy-side contract |
| Pivot B solver translation | Current dynamic transform-B relation derived from desired body relation plus pivot A |
| Existing comments | Current comments say pure proxy rotation was worse and current hybrid fixed wrist/world-direction dependency |

The implementation anchor is `makeRawRotationPalmTranslationFrame()` in `src/physics-interaction/hand/HandGrab.cpp`.

## Hot-Swap Contract

INI reload already exists in ROCK. The file watcher marks reload pending, and the frame hook calls `g_rockConfig.processPendingConfigReload()`.

The experiment contract should be:

| Rule | Reason |
| --- | --- |
| Read current INI values on frame-thread reload | Avoid reading or parsing INI from physics or grab hot paths |
| Snapshot the chosen experiment mode into `_grabFrame` at grab commit | A grab has a frozen solver contract and should not change modes mid-hold |
| Seated pivot promotion/re-freeze uses the captured mode from the same held object | Avoid mixing baseline capture with experimental re-seat math |
| New INI values affect the next grab only | Matches headset-off testing workflow |
| Mode `0` remains byte-for-byte/current-behavior equivalent where possible | Gives a clean baseline and rollback point |
| Log selected mode once per grab when debug logging is enabled | Makes captures diagnosable without spam |

## Proposed INI Keys

Add keys under `[PhysicsInteraction]` in the source INI template `data/config/ROCK.ini`. Do not edit the production INI unless deployment/testing explicitly requires it.

```ini
; Grab authority experiment mode.
; 0=current baseline. Other values are temporary experiment modes.
iGrabAuthorityExperimentMode=0

; Angle, in degrees, where raw/proxy mismatch blend starts.
fGrabAuthorityMismatchBlendStartDegrees=15.0

; Angle, in degrees, where raw/proxy mismatch blend reaches max weight.
fGrabAuthorityMismatchBlendFullDegrees=45.0

; Maximum blend weight toward proxy rotation for blend modes.
fGrabAuthorityMismatchBlendMaxWeight=1.0

; Optional cap for startup-only blend modes, in seconds from grab commit.
fGrabAuthorityStartupBlendMaxSeconds=0.35

; Extra one-shot diagnostics for selected mode and raw/proxy mismatch.
bGrabAuthorityExperimentDebugLogging=false
```

Mode-specific booleans should be avoided unless a mode truly needs them. The mode number should select one coherent behavior path.

## Full Mode Matrix

| Mode | Name | Fix Family | Summary | Expected Use |
| --- | --- | --- | --- | --- |
| 0 | CurrentHybridBaseline | Baseline | Current raw rotation plus proxy translation, current pivot/solver contract | Control test |
| 10 | RawRawAuthorityFrame | Family 1 | Use raw hand translation and raw hand rotation for authority frame | Tests whether hybrid translation is the bad part |
| 11 | ProxyProxyAuthorityFrame | Family 1 | Use proxy translation and proxy rotation for authority frame | Tests whether mixed rotation is the bad part |
| 20 | HybridHardSwitchOnMismatch | Family 2 | Use current hybrid normally, switch fully to proxy rotation when raw/proxy mismatch exceeds threshold | Fast binary test for mismatch hypothesis |
| 21 | HybridSmoothBlendOnMismatch | Family 2 | Use current hybrid normally, slerp raw rotation toward proxy rotation as mismatch grows | Safest first behavioral fix candidate |
| 22 | HybridStartupHardSwitchOnMismatch | Family 2 | Same as mode 20, but only during initial grab startup window | Tests whether only acquisition needs protection |
| 23 | HybridStartupSmoothBlendOnMismatch | Family 2 | Same as mode 21, but only during initial grab startup window | Best low-risk startup-only candidate |
| 24 | HybridBlendUntilTouchHeld | Family 2 | Blend while acquisition is not `TouchHeld`, then return to current hybrid | Tests whether bad grabs are only pre-seat/acquisition defects |
| 30 | BodyLocalPivotBTruth | Family 3 | Keep selected held-body grip point as solver-side B truth instead of deriving B from cross-contract A relation | Tests clean B-side contract fix |
| 31 | BodyLocalPivotBTruthWithHybridBlend | Family 3 plus Family 2 | Mode 30 plus smooth mismatch blend from mode 21 | Tests whether B-side fix still needs rotation protection |
| 40 | AuthorityLocalPivotAFreeze | Family 4 | Freeze pivot A in authority-frame local space, convert to proxy-local only when writing transform A | Tests clean A-side freeze fix |
| 41 | AuthorityLocalPivotAFreezeWithHybridBlend | Family 4 plus Family 2 | Mode 40 plus smooth mismatch blend from mode 21 | Tests whether A-side fix still needs rotation protection |
| 50 | UnifiedAuthorityLocalAAndBodyLocalB | Families 3 and 4 | Freeze A in authority-local space and use body-local B as solver truth | Cleanest full contract-unification experiment |
| 51 | UnifiedAuthorityLocalAAndBodyLocalBWithBlend | Families 2, 3, and 4 | Mode 50 plus smooth mismatch blend from mode 21 | Last-resort combined experiment |

## Mode Details

### Mode 0 - CurrentHybridBaseline

Purpose: Preserve current behavior exactly.

Authority frame:

```text
translation = proxy translation
rotation    = raw hand rotation
```

Pivot/solver behavior:

```text
pivot A = current proxy/split-frame contract
pivot B = current dynamic transform-B relation
```

Acceptance requirement: Debug logs should show mode `0`, and gameplay should match the current build.

### Mode 10 - RawRawAuthorityFrame

Purpose: Test whether the hybrid translation source is causing the intermittent bad grabs.

Authority frame:

```text
translation = raw hand translation
rotation    = raw hand rotation
```

Expected tradeoff: May lose the nicer palm/proxy physical anchor seat.

Primary question: Do bad rotational grabs disappear when position and rotation use one raw-hand contract?

### Mode 11 - ProxyProxyAuthorityFrame

Purpose: Test whether the raw/proxy rotation split is causing the intermittent bad grabs.

Authority frame:

```text
translation = proxy translation
rotation    = proxy rotation
```

Expected tradeoff: Likely worse object orientation/directness based on current comments and history.

Primary question: Do bad rotational grabs disappear when position and rotation use one proxy contract?

### Mode 20 - HybridHardSwitchOnMismatch

Purpose: Prove or disprove the raw/proxy mismatch hypothesis quickly.

Authority frame:

```text
translation = proxy translation
rotation    = raw hand rotation when mismatch < threshold
rotation    = proxy rotation when mismatch >= threshold
```

Threshold source: `fGrabAuthorityMismatchBlendStartDegrees`.

Expected tradeoff: Can produce a noticeable mode boundary if the threshold is crossed during grab startup.

Primary question: Are the bad grabs correlated with high raw/proxy angular mismatch?

### Mode 21 - HybridSmoothBlendOnMismatch

Purpose: Safest first fix candidate from Family 2.

Authority frame:

```text
translation = proxy translation
rotation    = slerp(raw rotation, proxy rotation, blendWeight)
```

Blend weight:

```text
0.0 at fGrabAuthorityMismatchBlendStartDegrees
fGrabAuthorityMismatchBlendMaxWeight at fGrabAuthorityMismatchBlendFullDegrees
clamped between 0.0 and fGrabAuthorityMismatchBlendMaxWeight
```

Expected tradeoff: If blend weight is too high, hand feel can become less direct.

Primary question: Can intermittent bad grabs be corrected without affecting normal grabs?

### Mode 22 - HybridStartupHardSwitchOnMismatch

Purpose: Test hard mismatch protection only during startup.

Authority frame:

```text
startup window: mode 20 behavior
after startup: mode 0 behavior
```

Startup window source: `fGrabAuthorityStartupBlendMaxSeconds`.

Expected tradeoff: Less runtime disturbance than mode 20, but can still produce a startup snap if threshold behavior is too abrupt.

Primary question: Is the defect purely a grab-start/freeze problem?

### Mode 23 - HybridStartupSmoothBlendOnMismatch

Purpose: Best low-risk startup-only blend candidate.

Authority frame:

```text
startup window: mode 21 behavior
after startup: mode 0 behavior
```

Startup window source: `fGrabAuthorityStartupBlendMaxSeconds`.

Expected tradeoff: Normal held behavior should stay closest to baseline after initial acquisition.

Primary question: Can acquisition be protected without changing settled hold feel?

### Mode 24 - HybridBlendUntilTouchHeld

Purpose: Blend only while the grab is not fully seated/touch-held.

Authority frame:

```text
before TouchHeld: mode 21 behavior
at/after TouchHeld: mode 0 behavior
```

Expected tradeoff: Depends on acquisition phase correctness. If phase transitions are late or noisy, the blend can last longer than desired.

Primary question: Are bad grabs tied to pre-seat acquisition rather than settled hold behavior?

### Mode 30 - BodyLocalPivotBTruth

Purpose: Test the clean B-side contract fix from Family 3.

Authority frame:

```text
translation = proxy translation
rotation    = raw hand rotation
```

Pivot/solver behavior:

```text
pivot B solver translation = frozen held-body grip point contract
avoid deriving transform-B translation from cross-contract pivot A relation
```

Expected tradeoff: More invasive than blend modes because it changes the solver-side constraint contract.

Primary question: Is the bad grab caused by the A-local proxy point feeding B-side solver math under a different basis?

### Mode 31 - BodyLocalPivotBTruthWithHybridBlend

Purpose: Test whether Family 3 needs mismatch blend protection.

Authority frame:

```text
translation = proxy translation
rotation    = mode 21 blended rotation
```

Pivot/solver behavior:

```text
pivot B solver translation = mode 30 B-side truth
```

Expected tradeoff: This combines two fixes, so it should not be tested until modes 21 and 30 have been tested independently.

Primary question: Does B-side contract cleanup solve the issue alone, or does high raw/proxy mismatch still need rotation blending?

### Mode 40 - AuthorityLocalPivotAFreeze

Purpose: Test the clean A-side freeze fix from Family 4.

Authority frame:

```text
translation = proxy translation
rotation    = raw hand rotation
```

Pivot/solver behavior:

```text
freeze pivot A in authority-frame local space
convert authority-local pivot A to proxy-local only when writing transform A to the actual constraint
```

Expected tradeoff: More math churn than Family 2 because it changes the frozen A-side relation.

Primary question: Is the bad grab caused by freezing a key A-side point in proxy-local space while angular authority uses raw hand space?

### Mode 41 - AuthorityLocalPivotAFreezeWithHybridBlend

Purpose: Test whether Family 4 needs mismatch blend protection.

Authority frame:

```text
translation = proxy translation
rotation    = mode 21 blended rotation
```

Pivot/solver behavior:

```text
pivot A freeze = mode 40 authority-local contract
```

Expected tradeoff: This combines two fixes, so it should not be tested until modes 21 and 40 have been tested independently.

Primary question: Does A-side freeze cleanup solve the issue alone, or does high raw/proxy mismatch still need rotation blending?

### Mode 50 - UnifiedAuthorityLocalAAndBodyLocalB

Purpose: Test full contract unification without rotation blending.

Authority frame:

```text
translation = proxy translation
rotation    = raw hand rotation
```

Pivot/solver behavior:

```text
pivot A freeze = authority-local contract
pivot B solver translation = held-body local grip truth
```

Expected tradeoff: Most invasive non-blend mode. This should be treated as a clean-design experiment, not a first test.

Primary question: Do both A-side and B-side contract cleanups together remove the intermittent failure while keeping baseline feel?

### Mode 51 - UnifiedAuthorityLocalAAndBodyLocalBWithBlend

Purpose: Last-resort combined mode.

Authority frame:

```text
translation = proxy translation
rotation    = mode 21 blended rotation
```

Pivot/solver behavior:

```text
pivot A freeze = authority-local contract
pivot B solver translation = held-body local grip truth
```

Expected tradeoff: Hardest to attribute because it combines every fix family.

Primary question: If no isolated mode solves the issue, does a fully unified contract plus mismatch blend solve it?

## Implementation Plan

### Step 1 - Add Config Fields

Files likely touched:

```text
src/RockConfig.h
src/RockConfig.cpp
data/config/ROCK.ini
```

Add fields with defaults matching baseline:

```cpp
int rockGrabAuthorityExperimentMode = 0;
float rockGrabAuthorityMismatchBlendStartDegrees = 15.0f;
float rockGrabAuthorityMismatchBlendFullDegrees = 45.0f;
float rockGrabAuthorityMismatchBlendMaxWeight = 1.0f;
float rockGrabAuthorityStartupBlendMaxSeconds = 0.35f;
bool rockGrabAuthorityExperimentDebugLogging = false;
```

Clamp rules:

| Key | Clamp |
| --- | --- |
| `iGrabAuthorityExperimentMode` | Allow only known mode IDs, otherwise force `0` |
| `fGrabAuthorityMismatchBlendStartDegrees` | `0.0` to `180.0` |
| `fGrabAuthorityMismatchBlendFullDegrees` | `start` to `180.0` |
| `fGrabAuthorityMismatchBlendMaxWeight` | `0.0` to `1.0` |
| `fGrabAuthorityStartupBlendMaxSeconds` | `0.0` to `5.0` |

### Step 2 - Add Policy Type

Add a narrow policy type near grab frame math, either in `GrabCore.h` or a new small grab policy header.

Suggested shape:

```cpp
enum class GrabAuthorityExperimentMode : std::uint8_t
{
    CurrentHybridBaseline,
    RawRawAuthorityFrame,
    ProxyProxyAuthorityFrame,
    HybridHardSwitchOnMismatch,
    HybridSmoothBlendOnMismatch,
    HybridStartupHardSwitchOnMismatch,
    HybridStartupSmoothBlendOnMismatch,
    HybridBlendUntilTouchHeld,
    BodyLocalPivotBTruth,
    BodyLocalPivotBTruthWithHybridBlend,
    AuthorityLocalPivotAFreeze,
    AuthorityLocalPivotAFreezeWithHybridBlend,
    UnifiedAuthorityLocalAAndBodyLocalB,
    UnifiedAuthorityLocalAAndBodyLocalBWithBlend,
};
```

Keep mode parsing separate from behavior code so unknown INI values fail closed to baseline.

### Step 3 - Snapshot Mode Into Grab Frame

Add captured experiment policy fields to `CanonicalGrabFrame`.

Suggested fields:

```cpp
GrabAuthorityExperimentMode authorityExperimentMode = GrabAuthorityExperimentMode::CurrentHybridBaseline;
float authorityMismatchBlendStartDegrees = 15.0f;
float authorityMismatchBlendFullDegrees = 45.0f;
float authorityMismatchBlendMaxWeight = 1.0f;
float authorityStartupBlendMaxSeconds = 0.35f;
```

Capture these once when the grab is committed.

Do not read `g_rockConfig.rockGrabAuthorityExperimentMode` directly inside active held-object solver updates except for debug logging toggles.

### Step 4 - Centralize Authority Frame Creation

Replace direct calls to `makeRawRotationPalmTranslationFrame()` in grab acquisition, seated promotion, held updates, and diagnostics with a single policy-aware function.

Suggested function responsibility:

```text
Input: raw hand world, proxy world, captured policy, acquisition phase, grab age seconds
Output: authority frame and diagnostic blend weight
```

Mode `0` must return exactly the current hybrid frame.

### Step 5 - Implement Family 1 Modes

Modes: `10`, `11`.

Implementation scope:

```text
Only authority frame source changes.
Pivot A and pivot B contracts remain current.
```

Validation target:

```text
Mode 10 and mode 11 compile, log their mode, and affect new grabs only.
```

### Step 6 - Implement Family 2 Modes

Modes: `20`, `21`, `22`, `23`, `24`.

Implementation scope:

```text
Authority translation remains proxy translation.
Authority rotation is selected or blended based on raw/proxy angle.
Pivot A and pivot B contracts remain current.
```

Required math:

```text
rawProxyAngleDegrees = rotationDeltaDegrees(raw.rotate, proxy.rotate)
blendWeight = saturate((rawProxyAngleDegrees - startDegrees) / (fullDegrees - startDegrees))
blendWeight *= maxWeight
```

Hard switch modes:

```text
blendWeight = 0.0 below threshold
blendWeight = maxWeight at or above threshold
```

Startup modes:

```text
apply only while grabAgeSeconds <= fGrabAuthorityStartupBlendMaxSeconds
```

Until-touch-held mode:

```text
apply only while acquisition phase is not TouchHeld
```

### Step 7 - Implement Family 3 Modes

Modes: `30`, `31`.

Implementation scope:

```text
Change solver transform-B source of truth.
Keep current mode 0 untouched.
```

Mode `30` should test direct held-body-local B truth:

```text
solverPivotB = frozen pivotBBodyLocalGame or the equivalent constraint-body-local point from one B-side contract
do not derive solverPivotB from desiredBodyInAuthorityFrameSpace plus pivotAAuthorityBodyLocalGame
```

Mode `31` should use the same B-side truth plus mode `21` rotation blending.

Important invariant:

```text
The visual grip point, debug pivot marker, release reconstruction, and solver transform-B should all refer to the same B-side grip point contract.
```

### Step 8 - Implement Family 4 Modes

Modes: `40`, `41`.

Implementation scope:

```text
Change how pivot A is frozen.
Keep current mode 0 untouched.
```

Mode `40` should freeze pivot A in authority-frame local space:

```text
pivotAAuthorityLocalAtGrab = worldPointToLocal(authorityFrameAtGrab, pivotAWorld)
active pivot A world = localPointToWorld(currentAuthorityFrame, pivotAAuthorityLocalAtGrab)
transform-A proxy local = worldPointToLocal(currentProxyFrame, active pivot A world)
```

Mode `41` should use the same A-side freeze plus mode `21` rotation blending.

Important invariant:

```text
The authority-local pivot A remains stable under the same authority frame contract that owns rotation.
The proxy-local transform A is only the final solver representation.
```

### Step 9 - Implement Unified Modes

Modes: `50`, `51`.

Implementation scope:

```text
Combine Family 3 and Family 4 contract cleanup.
Mode 51 also adds mode 21 rotation blending.
```

These modes should be implemented after isolated Family 3 and Family 4 modes so failures remain attributable.

## Test Order

Recommended order for headset-off INI testing:

| Order | Mode | Reason |
| --- | --- | --- |
| 1 | 0 | Establish same-session baseline |
| 2 | 21 | Safest likely fix with minimal normal-grab impact |
| 3 | 23 | Startup-only version of mode 21 |
| 4 | 24 | Acquisition-phase-limited version of mode 21 |
| 5 | 20 | Hard proof of mismatch hypothesis |
| 6 | 22 | Startup-only hard proof |
| 7 | 10 | Full raw/raw contract test |
| 8 | 11 | Full proxy/proxy contract test |
| 9 | 30 | Clean B-side contract test |
| 10 | 40 | Clean A-side contract test |
| 11 | 31 | B-side cleanup plus blend |
| 12 | 41 | A-side cleanup plus blend |
| 13 | 50 | Unified A/B contract cleanup |
| 14 | 51 | Unified cleanup plus blend |

## Manual Test Protocol

Use the same object set and the same attempted hand poses for each mode.

Suggested test objects:

| Object Type | Why |
| --- | --- |
| Small clutter item | Exposes overcorrection and jitter |
| Long object or weapon | Exposes torque/lever-arm failures |
| Thin/rod-like object | Exposes pinch/pocket and rotation authority issues |
| Heavy object | Exposes motor force and lag changes |
| Object grabbed from awkward angle | Exposes raw/proxy mismatch failures |

For each mode:

1. Set `iGrabAuthorityExperimentMode`.
2. Save `ROCK.ini`.
3. Put headset back on.
4. Wait a moment for config reload.
5. Perform repeated new grabs, not held-object mode switches.
6. Note whether the bad intermittent grab appears, improves, or gets worse.
7. If debug logging is enabled, capture mode name, raw/proxy mismatch angle, blend weight, pivot source, and active grab point mode.

## Acceptance Criteria

A mode is a candidate fix only if:

| Requirement | Meaning |
| --- | --- |
| Bad intermittent grabs are reduced or eliminated | The targeted defect improves |
| Normal grabs remain close to baseline feel | The fix does not solve rare failures by degrading common grabs |
| Long objects do not gain new torque instability | Lever-arm behavior remains safe |
| Release/throw behavior remains sane | Frozen grip relation is not corrupted |
| No persistent constraint, proxy, or collision cleanup regression appears | Runtime cleanup remains safe |
| Logs identify the selected mode and key diagnostics | Test results are attributable |

Reject a mode if it:

| Rejection Condition | Meaning |
| --- | --- |
| Fixes one object class but breaks baseline feel broadly | Not a production candidate |
| Requires high blend weight for all grabs | It is probably masking a deeper contract issue |
| Produces visible snap on acquisition | Not acceptable unless isolated to a debug-only proof mode |
| Causes held object drift from the visible grip point | Solver and visual contracts disagree |
| Needs mode mixing to be usable before isolated modes are understood | Attribution is too weak |

## Build And Validation Commands For Implementation

Docs-only planning does not require a build. When this plan is implemented, use the standard ROCK validation path.

Fast build and auto-deploy:

```bat
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m
```

Test build:

```bat
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
```

Full tests:

```bat
ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

## Cleanup Plan

These modes are for controlled experimentation, not permanent parallel production behavior.

After a winning mode is identified:

| Action | Reason |
| --- | --- |
| Promote the winning behavior into the normal production path | Avoid a hidden experimental branch becoming production dependency |
| Remove losing modes unless explicitly kept for diagnostics | Avoid stale fallback paths |
| Remove obsolete INI keys from source template | Keep config surface clean |
| Keep only necessary debug logging | Avoid hot-path logging noise |
| Update durable docs with the result and validation | Preserve why the final behavior was chosen |

Git commits are the rollback mechanism. Do not keep old behavior as an untested runtime fallback unless explicitly requested.
