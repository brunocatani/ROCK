# Dynamic Grab Current Log Inference - 2026-05-14

## Scope

This note tracks the current log evidence only. It is not an implementation plan
and it does not authorize a code change.

Current symptom from in-game testing:

- dynamic grab still has wrong rotation relation after grab;
- controller/hand rotation feels world-direction dependent;
- latest hard-keyframe angular-velocity change did not fix the visible issue.

## Fresh Runtime Evidence

Log inspected:

- `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log`
- last write: `2026-05-14 18:33:01`

Deployed DLL inspected:

- `D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll`
- last write: `2026-05-13 22:37:51`

The log contains:

- `diag=bodyFrameConstraint+hardKeyframeAngularVelocity`

Meaning:

- the tested build includes the hard-keyframe angular velocity path;
- that path is active at runtime;
- the fix was tested and is insufficient.

## Main Session Analyzed

Grab session:

- `session=4`
- `hand=R`
- `formID=000B14E2`
- held body: `1157`
- hand body: `2492`
- proxy body: `661`
- constraint: `625`

Important frame range:

- frame 1 around `2026-05-14 18:32:53.960`
- held-state tail frames 373-402 around `2026-05-14 18:32:58`

## Confirmed Facts

### The visual object/node starts correct

At frame 1:

- `objectAtGrab` and `desiredAtGrab` rotations match.
- `heldNode`, `bodyDerivedNode`, and `visualNode` match.
- `bodyNodeToVisual=0.000gu/0.000deg`
- `heldToConDesired=2.326gu/0.000deg`
- `rotationPreservedDeg=0.000`

Meaning:

- at capture/start, the visual node/object relation is not inherently wrong;
- the object visual frame is preserved correctly at the node level.

### The body-frame target is already 90 degrees wrong at frame 1

At frame 1:

- `nativeToHeld=8.766gu/90.000deg`
- `bodyToConDesired=9.054gu/90.000deg`
- `heldBodyToConDesiredBody.max=119.99deg`
- `nativeBodyToHeldBody.max=119.99deg`

Meaning:

- the problem exists immediately in the body-frame side of the authority path;
- this is not only late drift or a post-settle issue.

### The proxy/authority hand frame is close to the native flattened hand frame

At frame 1:

- capture `rawHandToProxyPalm.max=0.41deg`
- telemetry `rawToConRev=5.147gu/0.408deg`
- framechain `nativeToAuthority=5.139gu/0.408deg`
- framechain `authorityToProxy=0.000gu/0.000deg`

Meaning:

- the generated authority/proxy frame is not the source of the 90-degree body
  mismatch in this session;
- the proxy frame is tracking the intended root-flattened hand orientation very
  closely.

### The generated palm debug frame is not the same frame as the authority frame

At frame 1:

- `generatedPalm` differs from `grabAuthority` by about `150.973deg`
- `nativeToPalm=5.139gu/150.578deg`
- `nativeToAuthority=5.139gu/0.408deg`

Meaning:

- the visual/debug generated-palm frame is a different representation than the
  authority/proxy frame;
- using generated-palm axes as proof of authority axes would be misleading.

### The active pivot is still the legacy INI-configured hand-space pivot

At frame 1:

- `legacyActive=yes`
- `activeSource=iniConfiguredHandspace`
- `legacyToRuntime=0.000gu`
- `legacyToPalm=2.149gu`
- `legacyToAuthority=2.149gu`
- `legacyToProxy=2.149gu`

Meaning:

- the active hand-side pivot A is still the old INI-configured hand-space pivot;
- this may affect pivot position and force lever behavior;
- it does not by itself explain the 90-degree rotation mismatch, because
  `rawToConRev` and `nativeToAuthority` are near zero in rotation.

### Constraint transform-B local storage is internally stable

Repeated through session 4:

- `transformBLocal=(-8.04,2.54,3.68)`
- `desiredTransformBLocal=(-8.04,2.54,3.67)`
- `transformBErr=0.000gu`
- `targetErr(colsInv=0.000deg rowsInv=172.396deg colsForward=172.396deg colsTransformB=0.000deg)`

Meaning:

- the transform-B bytes are internally consistent under the current diagnostic
  interpretation;
- pivot B is not obviously changing or drifting in local storage.

Important limitation:

- this proves internal consistency, not that the selected body-frame convention
  is the correct convention for the solver/visual relation.

### Proxy follows its target reasonably

Examples:

- frame 373 after solve: `proxyTargetErr=1.248gu/5.82deg`
- frame 397 after solve: `proxyTargetErr=0.297gu/6.93deg`
- frame 402 after solve: `proxyTargetErr=0.271gu/6.02deg`

Meaning:

- proxy body A is not grossly detached from the target;
- the major wrongness is not simply "proxy is not moving".

### Object remains offset from target/pivot

Examples:

- frame 373: `objectTargetErr=12.49gu/17.8deg`, `gripTargetErr=11.59gu`
- frame 397: `objectTargetErr=10.54gu/12.6deg`, `gripTargetErr=9.54gu`
- frame 402: `objectTargetErr=11.39gu/14.1deg`, `gripTargetErr=10.35gu`

Meaning:

- the object is not converging cleanly to the physical authority target;
- this may be a force/constraint issue, but the first-frame 90-degree body-frame
  mismatch shows there is a frame-chain problem before force tuning matters.

## Strongest Current Inference

The current problem is most likely in the body-frame/object-frame relationship
used by the custom authority path.

More specifically:

- the object visual/node frame is preserved correctly at capture;
- the proxy/authority hand frame is close to the native flattened hand frame;
- but the physics body frame used for the constraint target is already about
  90 degrees away at frame 1;
- later hard-keyframe angular velocity then drives toward that body target
  correctly, but the target itself is already the wrong body-frame relation.

This means the previous angular velocity convention hypothesis was either wrong
or incomplete. The current logs do not support continuing to treat angular
velocity vector convention as the root cause.

## What I Do Not Know Yet

The logs do not yet prove which exact frame relation is wrong. Current possible
causes still include:

- wrong composition order when deriving desired body transform from desired node
  transform;
- using BODY frame where the constraint/solver should consume a different local
  body/motion/shape frame;
- missing fixed body-to-node transform when converting visual object target to
  physics body target;
- diagnostic `colsInv` convention matching the stored bytes while still being
  the wrong conceptual relation;
- visual hand/object relation being computed from node space while the constraint
  authority is computed from native body space with a fixed 90-degree basis
  offset.

I cannot honestly pick the final code change from this log alone.

## What The Logs Rule Out Or Weaken

These are weaker explanations now:

- "the hard-keyframe angular velocity path was not active" - false, it is active;
- "proxy body is wildly off target" - not supported by proxy error values;
- "visual node is already wrong at frame 1" - false for session 4;
- "pivot B local storage is drifting" - not supported by transformB telemetry;
- "raw hand frame vs authority hand frame is rotated 90 degrees" - not supported
  by `nativeToAuthority` and `rawToConRev`.

## Data Needed Before Another Fix

The next diagnostic must compare candidate frame chains in the same frame, not
change behavior.

For each grab start and held frame, log candidate desired body transforms:

1. current body target:
   - `desiredNodeWorld * capturedNodeToBody`
2. inverse/current-order candidate:
   - alternate multiplication order for `node/body` relation
3. motion-frame candidate:
   - `desiredNodeWorld * capturedNodeToMotion`
4. live visual reconciliation candidate:
   - `liveVisualNodeWorld * capturedNodeToBody`
5. solver-readback reconciliation candidate:
   - `livePhysicsBodyWorld * capturedBodyToNode`

For each candidate, log:

- delta to `nativeBody[BODY]`;
- delta to `heldBody[MOTION]`;
- delta to `visualNode`;
- delta to `bodyDerivedNode`;
- delta to `conDesiredBody`;
- per-axis dot/degrees, not only a single quaternion angle.

The correct chain should be the one that is near zero against the intended
visual/node relation while preserving the same body frame consumed by the
constraint.

## Diagnostic Added After This Finding

Implementation-only note, no behavior change:

- added `GRAB FRAMECHAIN CANDIDATES` and `GRAB FRAMECHAIN CANDIDATE` log rows;
- the active grab drive still uses the same current target path;
- no candidate is fed back into motor, constraint, pivot, proxy, or hand pose
  authority.

The diagnostic logs these candidate body targets from the same frame:

- `currentConBody`: the current body target used by the custom authority path;
- `conNode*bodyLocal`: current desired visual/node target composed with the
  captured BODY-local relation;
- `conNode*constraintBodyLocal`: current desired visual/node target composed
  with the captured constraint-body-local relation;
- `rawNode*bodyLocal`: raw hand desired node target composed with BODY-local;
- `rawNode*constraintBodyLocal`: raw hand desired node target composed with
  constraint-body-local;
- `heldNode*bodyLocal`: live visual node composed with BODY-local;
- `heldNode*constraintBodyLocal`: live visual node composed with
  constraint-body-local;
- `conNode*invBodyLocal`: intentionally suspicious inverse candidate, included
  only to prove or disprove an inverse/order error.

Each candidate logs:

- body delta to native BODY readback;
- body delta to held/MOTION readback;
- body delta to current constraint desired body;
- node-from-BODY-local delta to live visual node;
- node-from-BODY-local delta to current desired node;
- node-from-constraint-body-local delta to live visual node;
- node-from-constraint-body-local delta to current desired node;
- per-axis basis deltas for candidate body and derived node frames.

Expected interpretation:

- If `heldNode*bodyLocal` or `conNode*bodyLocal` is near zero against the visual
  node but far from `currentConBody`, the current body target formula is wrong.
- If `conNode*constraintBodyLocal` is near zero and `conNode*bodyLocal` is not,
  the wrong local body relation is being used.
- If candidate correctness changes when the player turns, the bug is in
  world/local composition or player-space compensation.
- If every candidate is wrong, the captured source relation is incomplete and we
  need one deeper FO4VR body readback point before changing behavior.

## Current Honest Conclusion

The logs show enough to say the current fix is insufficient and the root issue
is still a frame-chain mismatch between visual object/node space and the physics
body frame used by the custom constraint.

The logs do not yet show enough to safely choose the replacement transform
formula. Another code fix without candidate frame-chain telemetry would still be
guessing.

## Fresh Right/Left Frame-Chain Log Read - 2026-05-14 18:49

Runtime evidence:

- deployed diagnostic DLL timestamp: `2026-05-14 18:44:57`;
- tested log timestamp: `2026-05-14 18:49:04`;
- fresh log contains the new `GRAB FRAMECHAIN CANDIDATES`,
  `GRAB FRAMECHAIN CANDIDATE`, and `GRAB FRAMECHAIN AXES` rows;
- recorded candidate groups:
  - `R, session 1`: 97 candidate frames, frame `571..667`;
  - `L, session 2`: 395 candidate frames, frame `1..395`.

Important limitation:

- the right-hand session starts in this log at held frame `571`, not frame `1`;
- therefore this log can compare right-hand held behavior, but cannot prove the
  right-hand attach/start-frame behavior;
- the left-hand session does include frame `1`, so it can be used for grab-start
  evidence.

### Session 2 Left Hand Start Frame

At `session=2 frame=1 hand=L phase=start`:

- visual node and body-derived node agree:
  - `bodyNodeToVisual=0.000gu/0.000deg`;
- held node and desired node agree:
  - `heldToConDesired=1.942gu/0.000deg`;
  - `heldNodeToConDesiredObj.max=0.00deg`;
- current desired BODY equals native BODY orientation:
  - `currentConBody bodyToNative=1.942gu/0.000deg`;
- current desired BODY is 180 degrees away from `heldBody[MOTION]`:
  - `currentConBody bodyToHeld=2.773gu/180.000deg`;
- `bodyTargetNodeErr=0.000gu/0.000deg`, so the current code is internally
  composing the desired node/body relation consistently.

Honest interpretation:

- the left-hand grab starts from a coherent visual/node relation;
- the diagnostic does not show an immediate visual target formula error at frame
  1;
- the 180-degree BODY-vs-MOTION readback is present from the start, but at frame
  1 it is not yet causing the visual node to disagree with the desired node.

### Session 2 Left Hand Held Drift

By `session=2 frame=190 hand=L`:

- current desired BODY still equals the current constraint target by definition:
  - `currentConBody bodyToCon=0.000gu/0.000deg`;
  - `nodeBodyLocalToConNode=0.000gu/0.000deg`;
- current target is no longer close to live visual node:
  - `currentConBody nodeBodyLocalToHeld=5.922gu/33.200deg`;
- raw-hand candidate is closer to the live visual node:
  - `rawNode*bodyLocal nodeBodyLocalToHeld=3.115gu/9.026deg`;
- live-visual candidate trivially reconstructs the visual node:
  - `heldNode*bodyLocal nodeBodyLocalToHeld=0.000gu/0.000deg`;
  - but that candidate is not safe authority because it is derived from the
    already-moved visual node.

By `session=2 frame=395 hand=L`:

- current target remains internally self-consistent:
  - `currentConBody bodyToCon=0.000gu/0.000deg`;
  - `nodeBodyLocalToConNode=0.000gu/0.000deg`;
- current target is still offset from live visual node:
  - `currentConBody nodeBodyLocalToHeld=6.329gu/31.778deg`;
- raw-hand candidate is closer to native BODY orientation:
  - `rawNode*bodyLocal bodyToNative=6.671gu/2.358deg`;
  - current target body-to-native is worse:
    `currentConBody bodyToNative=6.805gu/10.718deg`;
- live-visual candidate still reconstructs visual node exactly but does not
  explain the intended authority chain:
  - `heldNode*bodyLocal nodeBodyLocalToHeld=0.000gu/0.000deg`;
  - `heldNode*bodyLocal bodyToCon=6.237gu/31.778deg`.

### Session 1 Right Hand Held Behavior

Right-hand candidate rows begin at `frame=571`, already in held state.

At `session=1 frame=571 hand=R`:

- current target is internally self-consistent:
  - `currentConBody bodyToCon=0.000gu/0.000deg`;
  - `nodeBodyLocalToConNode=0.000gu/0.000deg`;
- current target is not close to live visual node:
  - `currentConBody nodeBodyLocalToHeld=10.809gu/31.920deg`;
- raw-hand candidate is closer to live visual node:
  - `rawNode*bodyLocal nodeBodyLocalToHeld=8.758gu/16.994deg`;
- live-visual candidate reconstructs visual node:
  - `heldNode*bodyLocal nodeBodyLocalToHeld=0.000gu/0.000deg`.

At `session=1 frame=667 hand=R`:

- current target remains internally self-consistent:
  - `currentConBody bodyToCon=0.000gu/0.000deg`;
- current target is still not the live visual relation:
  - `currentConBody nodeBodyLocalToHeld=6.258gu/23.447deg`;
- raw-hand candidate is again closer to native BODY orientation:
  - `rawNode*bodyLocal bodyToNative=6.936gu/1.054deg`;
  - current target body-to-native is worse:
    `currentConBody bodyToNative=6.184gu/16.791deg`.

### Aggregate Candidate Read

Average rotation deltas from the fresh log:

| Hand/session | Candidate | Frames | Avg body-to-native | Avg node-to-held | Avg body-to-current-target |
| --- | --- | ---: | ---: | ---: | ---: |
| L/2 | `currentConBody` | 395 | 22.45 deg | 65.82 deg | 0.00 deg |
| L/2 | `rawNode*bodyLocal` | 395 | 10.20 deg | 62.22 deg | 21.47 deg |
| L/2 | `heldNode*bodyLocal` | 395 | 61.37 deg | 0.00 deg | 65.82 deg |
| L/2 | `conNode*invBodyLocal` | 395 | 23.82 deg | 68.45 deg | 12.32 deg |
| R/1 | `currentConBody` | 97 | 24.18 deg | 31.00 deg | 0.00 deg |
| R/1 | `rawNode*bodyLocal` | 97 | 9.36 deg | 21.11 deg | 22.43 deg |
| R/1 | `heldNode*bodyLocal` | 97 | 19.41 deg | 0.00 deg | 31.00 deg |
| R/1 | `conNode*invBodyLocal` | 97 | 24.94 deg | 32.04 deg | 3.82 deg |

What this rules out:

- `conNode*bodyLocal` and `conNode*constraintBodyLocal` are not alternative
  fixes here; they collapse to the current target in this run;
- the inverse candidate is not the missing solution;
- BODY-local vs `constraintBodyLocal` is not the observed split in this run
  because they are identical for both hands;
- the current target math is internally consistent but is not staying aligned
  with the live visual/object relation during held motion.

What this supports:

- the bug is not a single obvious matrix transpose in the logged candidate set;
- the custom authority path has a self-consistent target frame, but that frame
  diverges from the visual hand/object relation during held motion;
- raw-hand-derived target often tracks native BODY orientation better than the
  authority/proxy-derived target, but it still does not solve the visual/object
  relation by itself;
- both hands show the same class of problem, with the left-hand session showing
  more severe drift later in the held state.

Honest uncertainty:

- these logs do not prove the final replacement formula;
- these logs do not directly show the user's exact physical controller rotation
  delta versus object angular delta, so they cannot fully explain the perceived
  N/S/E/W dependency by themselves;
- the next useful diagnostic is not another candidate body formula alone, but a
  per-frame controller/hand angular delta compared against object angular delta
  in both controller-local and player/world bases.

Concrete next diagnostic needed:

- log the per-frame raw controller/hand delta rotation;
- log the per-frame authority/proxy hand delta rotation;
- log the per-frame desired object delta and actual visual object delta;
- express those deltas in:
  - hand-local basis;
  - player/HMD-local basis;
  - world basis;
- include left/right separately in the row key.

That would directly answer whether wrist pitch is being applied as object yaw,
whether roll is inverted, and whether the mapping changes when the player turns.

## Follow-Up Change: Remove INI Pivot From Dynamic Authority

The next diagnostic build removes the old configured INI pivot from dynamic
grab authority:

- `Hand::computeGrabPivotAWorld` now resolves pivot A from the latest
  root-flattened generated palm target when available;
- if that generated palm target is unavailable during startup or world rebuild,
  the fallback is the raw tracked hand origin, not the INI-configured handspace
  offset;
- `GrabThreePhase` now has `buildGrabPocketFrameWithPalmCenter`, and dynamic
  grab capture passes the already-resolved generated palm pivot into the pocket
  frame;
- the old INI pivot remains logged as `legacyPivotA` only, with
  `legacyActive=no`;
- equipped weapon and two-hand weapon palm helpers still use their existing
  paths and are not part of this dynamic-loose-grab diagnostic change.

Reason:

- leaving the INI pivot as runtime authority made every later log ambiguous:
  the diagnostics could not prove whether the frame-chain error came from
  dynamic grab math or from an old manually tuned offset;
- removing it from dynamic authority gives cleaner data without changing object
  pivot B, COM policy, collision layers, or equipped weapon behavior.

## Follow-Up Telemetry: Angular Delta Mapping

The next diagnostic build adds `GRAB ANGULAR_DELTA` rows. These compare the
per-frame angular delta of:

- raw tracked hand;
- generated palm grab-authority frame;
- live proxy readback;
- raw desired object;
- constraint desired object;
- live held visual node;
- held hknp body;
- native BODY readback.

Each delta is logged as:

- world-axis degree vector;
- current hand-local degree vector;
- HMD/player-planar degree vector.

Purpose:

- show directly whether wrist pitch, yaw, or roll is being applied to the wrong
  object axis;
- show whether the mapping changes when the player rotates;
- separate raw controller motion from authority/proxy motion and from actual
  object motion.

## Fresh Runtime Read: 2026-05-14 19:10 Grabs

Source log:

- `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log`
- deployed DLL timestamp checked against the run: `2026-05-14 19:03:18`
- log timestamp after user test: `2026-05-14 19:10:07`

Sessions observed:

| Hand/session | Angular rows | First frame | Last frame |
| --- | ---: | ---: | ---: |
| R/6 | 227 | 16 | 242 |
| L/7 | 158 | 1 | 158 |

### Confirmed: INI Pivot Is Not Runtime Authority

`GRAB BASIS LEGACY_PIVOT` rows:

| Hand/session | Rows | `legacyPresent` | `legacyActive` | `activeSource` |
| --- | ---: | --- | --- | --- |
| R/6 | 227 | yes | no | `rootFlattenedPalmAnchorTarget` |
| L/7 | 158 | yes | no | `rootFlattenedPalmAnchorTarget` |

No `legacyActive=yes` rows were found in this run.

Conclusion:

- the old configured INI pivot still exists as a logged comparison point;
- it is not driving dynamic grab pivot A in these sessions;
- the observed rotation bug is not explained by old INI pivot authority.

### Confirmed: Raw Hand And Generated Authority Move Together

Aggregate angular delta:

| Hand/session | Raw avg | Raw max | Proxy avg | Proxy max | Constraint desired avg | Constraint desired max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| R/6 | 2.60 deg | 10.86 deg | 27.93 deg | 76.37 deg | 29.20 deg | 79.77 deg |
| L/7 | 1.46 deg | 2.31 deg | 12.43 deg | 47.89 deg | 12.33 deg | 46.30 deg |

In every sampled bad row, raw and generated-authority deltas are nearly
identical. Examples:

- R/6 frame 49: `raw=1.164`, `authority=1.165`,
  `proxy=65.823`, `conDesired=72.226`;
- R/6 frame 50: `raw=0.865`, `authority=0.865`,
  `proxy=64.543`, `conDesired=70.517`;
- L/7 frame 156: `raw=0.329`, `authority=0.327`,
  `proxy=46.341`, `conDesired=43.433`;
- L/7 frame 157: `raw=0.468`, `authority=0.467`,
  `proxy=45.557`, `conDesired=42.523`.

Conclusion:

- the root-flattened palm authority frame is not showing the huge rotation;
- the huge rotation appears after the proxy/constraint authority frame;
- `conDesired` follows the proxy jump, not the raw hand delta.

### Confirmed: Grab Start Can Be Clean, Then Held Loop Corrupts

Left session 7 frame 1:

- phase: `start`;
- `objectAtGrabToDesiredAtGrab.max=0.00`;
- `heldNodeToConDesiredObj.max=0.00`;
- `rawHandToHeldRelativeHand.max=0.00`;
- `proxyReadback` equals `grabAuthority`;
- `authorityToProxy=0.000gu/0.000deg`;
- `legacyActive=no`;
- `targetErr(colsInv=0.000deg ... colsTransformB=0.000deg)`.

Conclusion:

- the attach/start frame in this left-hand grab preserved the desired object
  rotation;
- the corruption is not proven at initial pivot capture in this session;
- the corruption grows after held update/solve starts using the live proxy path.

Right session 6 does not contain frame 1 in the angular-delta rows. Its first
angular-delta row is frame 16, so this log cannot prove the right-hand attach
frame itself. It does prove the right-hand held loop has the same proxy-driven
rotation amplification.

### Confirmed: Proxy Is Exact Before Solve But Wrong After Solve

Sparse `PROXY GRAB AUTHORITY` rows show the proxy on target before solve:

| Side | Rows | Max pre-solve proxy angular error |
| --- | ---: | ---: |
| Left | 17 | 0.03 deg |
| Right | 10 | 0.00 deg |

`PROXY GRAB AFTER_SOLVE` rows show large post-solve proxy angular error:

| Side | Rows | Avg post-solve proxy angular error | Max post-solve proxy angular error |
| --- | ---: | ---: | ---: |
| Left | 105 | 9.71 deg | 24.45 deg |
| Right | 152 | 22.22 deg | 40.90 deg |

Examples:

- R frame 45: `proxyTargetErr=0.991gu/40.90deg`;
- R frame 42: `proxyTargetErr=1.609gu/40.31deg`;
- L frame 152: `proxyTargetErr=0.732gu/24.45deg`;
- L frame 155: `proxyTargetErr=0.528gu/23.72deg`.

Conclusion:

- the pre-solve write/readback path can place the proxy exactly on target;
- after solve, the live proxy orientation is often tens of degrees away;
- the held desired frame then uses or reflects that corrupted proxy orientation;
- this is strong evidence that the bug lives in the proxy body-A authority /
  constraint solve integration, not in the raw controller/palm frame.

### Confirmed: Alternating/Stale Proxy Pattern

The angular rows frequently alternate between:

- a large proxy/conDesired angular delta;
- then a zero proxy/conDesired angular delta;
- then another large proxy/conDesired angular delta.

Examples:

- R frames 40, 41, 42, 43, 44, 45:
  - proxy: `64.30`, `46.55`, `0.00`, `76.37`, `56.25`, `0.00`;
  - conDesired: `63.06`, `45.36`, `0.00`, `74.88`, `55.04`, `0.00`.
- L frames 153, 154, 155, 156, 157, 158:
  - proxy: `47.70`, `45.45`, `0.00`, `46.34`, `45.56`, `0.00`;
  - conDesired: `44.66`, `43.05`, `0.00`, `43.43`, `42.52`, `0.00`.

Conclusion:

- the proxy readback/desired frame is not a smooth per-frame copy of the
  root-flattened palm authority;
- the pattern is consistent with stepwise physics update cadence, stale
  readback, or solve-side rotation being fed back into the next desired frame;
- it is not consistent with a continuous raw controller axis mapping bug alone.

### Current Inference

What the fresh log proves:

- legacy INI pivot is not active;
- raw hand and generated root-flattened authority agree closely;
- initial left attach can preserve object rotation exactly;
- proxy readback diverges after solve;
- the constraint desired object frame follows the proxy divergence;
- both hands are affected.

What the fresh log does not prove:

- the final production formula for replacing the proxy path;
- whether the proxy is being rotated by constraint feedback, stale body
  readback, wrong motion/body write target, or hard-keyframe velocity
  integration order;
- the exact controller-axis semantic mapping from the user's physical hand,
  because the logs do not include a named physical action marker like
  "user pitched wrist up now".

Most likely next code investigation target:

- inspect the body-A/proxy authority update path and remove the feedback
  dependency where held-target computation uses live proxy readback as the
  conceptual hand frame;
- the desired object frame should be computed from the root-flattened palm
  authority target and the captured contact/palm relationship, while the proxy
  should only be a solver body that is driven toward that target;
- if the proxy remains, after-solve proxy orientation must not become the next
  frame's hand/object reference authority.

## Source Pass After 19:10 Log

This section supersedes the broad "transform convention" suspicion for the
current runtime symptom. The collider convention was restored and the fresh log
still shows the dynamic grab problem. The active evidence now points at the
custom grab authority/rotation writers, not at old INI pivot authority or the
general generated-collider transform convention.

### Runtime Writer Order In Current Source

Relevant source paths:

- `src/physics-interaction/hand/HandGrab.cpp`
  - `Hand::resolveGrabAuthorityProxyFrame`
  - `Hand::createProxyConstraintGrabDrive`
  - `Hand::updateProxyConstraintGrabDriveTarget`
  - `Hand::applyProxyConstraintAngularVelocityDrive`
  - `Hand::flushPendingCustomGrabAuthority`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
  - `PhysicsInteraction::applyHeldPlayerSpaceVelocity`
  - `PhysicsInteraction::driveGrabAuthorityPhase0ProbeFromBetweenStep`
- `src/physics-interaction/grab/HeldPlayerSpaceRegistry.cpp`
  - `held_player_space_registry::applyCentralPlayerSpaceVelocity`
- `src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp`
  - `driveGeneratedKeyframedBody`
  - `placeGeneratedKeyframedBodyImmediately`

Current flow:

1. Held update resolves the root-flattened palm authority frame with
   `resolveGrabAuthorityProxyFrame`.
2. For proxy-constraint grab, held update computes:
   - `desiredObjectWorld = proxyAuthorityWorld * constraintHandSpace`;
   - `desiredBodyWorld = proxyAuthorityWorld * constraintBodyHandSpace`;
   - `activePivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame()`.
3. Held update queues only the proxy authority target through
   `queueProxyGrabAuthorityTarget`.
4. Between collide and solve, `flushPendingCustomGrabAuthority`:
   - copies the queued target;
   - computes linear velocity from previous proxy target to current proxy
     target;
   - computes angular velocity with FO4VR's hard-keyframe helper;
   - calls `_grabAuthorityProxy.setTransform(targetHavok)`;
   - immediately calls `_grabAuthorityProxy.setVelocity(linearVelocity,
     angularVelocity)`;
   - updates linear constraint target data;
   - applies object angular velocity directly through
     `world->SetBodyAngularVelocity`.

This means the active custom grab currently has two separate movement
authorities:

- body-A/proxy linear constraint authority;
- direct body-B angular velocity authority.

The ragdoll angular atom is disabled in `createGrabConstraint`:

- `setGrabMotorAtomsActive(header, true, false)`;
- log label: `directAngularDrive=yes ragdollAtom=disabled`.

So object rotation is not coming from a hkp/HIGGS-style angular constraint
solver. It is coming from ROCK's direct FO4VR hknp angular-velocity write.

### Player-Space Compensation Is Not The Primary 19:10 Cause

Fresh log rows:

- `Held player-space: ... warp=no distWarp=no rotWarp=no rotDelta=0.00deg
  delta=(0.00,0.00,0.00) velHk=(0.000,0.000,0.000)`;
- `Held player-space central writer: ... runtimeWarp=no distWarp=no
  rotWarp=no bodies=2 registered=1 motionsWritten=1 transformsWarped=0`.

Conclusion:

- player-space compensation is active as a velocity writer, but in this run it
  is not applying transform warps and is not producing rotation deltas;
- it remains worth keeping in the map because it can preserve/damp angular
  velocity, but it does not explain the immediate proxy post-solve rotation
  error seen in this log.

### Confirmed: Proxy Post-Solve Rotation Equals Proxy Angular Velocity Integration

The strongest concrete finding from this pass:

- before solve, `PROXY GRAB AUTHORITY` reports `proxyErr=0.000gu/0.00deg`
  or near zero;
- after solve, `PROXY GRAB AFTER_SOLVE` reports a proxy angular error that
  closely equals `proxyAngVel * dt`.

Examples:

| Hand | Seq | `proxyAngVel` | `dt` | Expected integrated angle | Logged after-solve proxy angle |
| --- | ---: | ---: | ---: | ---: | ---: |
| R | 61 | 29.575 rad/s | 0.016667 | 28.24 deg | 27.69 deg |
| R | 106 | 28.453 rad/s | 0.016667 | 27.17 deg | 26.68 deg |
| R | 151 | 20.511 rad/s | 0.016667 | 19.59 deg | 19.40 deg |
| R | 16 | 10.536 rad/s | 0.016667 | 10.06 deg | 10.04 deg |
| L | 61 | 7.964 rad/s | 0.016667 | 7.60 deg | 7.59 deg |

Conclusion:

- the hidden proxy is being put exactly at the target and then told to keep
  spinning during the same solver step;
- the post-solve proxy rotation error is not random and not primarily an INI
  pivot issue;
- this is a direct consequence of exact `setTransform(target)` plus nonzero
  angular `setVelocity(...)` on the proxy.

This is different from the generated hand/body collider drive:

- generated colliders use `DriveToKeyFrame(target, dt)` for normal drive;
- when generated colliders are placed exactly, `placeGeneratedKeyframedBodyImmediately`
  also zeroes velocity immediately;
- generated colliders do not normally use exact transform placement plus
  nonzero angular velocity on the same body in the same mode.

That distinction explains how the generated colliders can be visually correct
while the grab proxy, which uses a different drive policy, rotates away after
solve.

### Telemetry Nuance: `conDesired` Is A Live-Proxy Diagnostic, Not Always The Command

`getGrabTransformTelemetrySnapshot` uses the live proxy readback as
`constraintAnchorWorld` when the proxy exists. It then derives:

- `currentConstraintDesiredObjectWorld =
  constraintAnchorWorld * constraintHandSpace`;
- `currentConstraintDesiredBodyWorld =
  constraintAnchorWorld * constraintBodyHandSpace`.

But the actual between-step command in `flushPendingCustomGrabAuthority` uses
the queued `pending.proxyWorld`, not the after-solve live proxy readback.

Conclusion:

- the large `conDesired` jumps prove the live proxy readback is corrupted after
  solve;
- they do not by themselves prove the queued command target is corrupted;
- however, the solver does see the proxy body during the step, so letting the
  proxy integrate angular velocity is still a real physics problem, not only a
  log problem.

### Current High-Confidence Cause

The custom proxy drive violates a clean keyframed-body policy:

- it sets the proxy transform to the authority target;
- then it sets a nonzero angular velocity computed as if the proxy still needed
  to travel toward that target;
- after solve, the proxy has rotated away by approximately that angular
  velocity times the physics delta.

That can affect grab quality in two ways:

- the linear constraint body-A is not a stable palm anchor during solve;
- telemetry and any future code using live proxy readback can inherit the
  after-solve proxy rotation instead of the root-flattened authority frame.

The active object rotation still has a second unresolved branch:

- direct `SetBodyAngularVelocity` on body B may still have an axis-convention or
  target-frame issue;
- the 19:10 log proves the proxy body-A error, but it does not fully prove the
  direct body-B angular vector maps every physical wrist axis correctly.

### What Is Still Uncertain

Still not proven from the current log alone:

- whether removing proxy angular velocity is sufficient to fix the visible
  N/S/E/W object-axis issue;
- whether the direct body-B angular velocity vector from FO4VR's
  hard-keyframe helper has the correct sign/basis for all hand/controller
  rotations;
- whether a pure linear constraint plus direct angular body write is the final
  correct architecture, or whether a FO4VR-native angular constraint/motor path
  must be reintroduced after proper binary mapping;
- whether the held visual hand path is following the object error, the proxy
  error, or both during the user's specific physical wrist actions.

### Investigation Direction Before Any Behavior Patch

The next useful investigation should isolate two questions:

1. Body-A/proxy policy:
   - should the proxy be an exact hidden body with zero velocity every
     between-step;
   - or should it be driven only by velocity without exact transform snapping;
   - current mixed policy is internally inconsistent and confirmed by logs.
2. Body-B angular authority:
   - compare the requested object angular velocity vector, the target-frame
     rotation delta, and the actual next-frame body delta in world, hand-local,
     and HMD-local coordinates;
   - determine whether the direct `SetBodyAngularVelocity` vector is correct
     but fighting the proxy, or whether it is independently world-axis bound.

No implementation conclusion should ignore both branches. The proxy bug is
confirmed; the direct angular writer remains a separate suspect for the
world-direction/N/S/E/W symptom.

### Angular-Delta Object Motion Evidence

Aggregate `GRAB ANGULAR_DELTA` rows from the same run:

| Hand/session | Rows | Raw avg/max | Proxy avg/max | Held node avg/max | Held body avg/max |
| --- | ---: | ---: | ---: | ---: | ---: |
| R/6 | 227 | 2.60 / 10.86 deg | 27.93 / 76.37 deg | 5.05 / 15.84 deg | 2.81 / 15.13 deg |
| L/7 | 156 | 1.47 / 2.31 deg | 12.59 / 47.89 deg | 4.57 / 24.93 deg | 2.16 / 12.50 deg |

This separates two facts:

- the live proxy is much more wrong than the object body on average;
- the held object and visible node still rotate more than the raw hand in many
  rows, so the user-visible symptom is real and not only bad proxy telemetry.

Representative rows where raw hand motion is small but object/proxy motion is
larger:

- R/6 frame 235:
  - raw `0.447 deg`;
  - proxy `36.138 deg`;
  - held node `10.725 deg`;
  - held body `3.913 deg`.
- R/6 frame 238:
  - raw `1.368 deg`;
  - proxy `37.641 deg`;
  - held node `8.403 deg`;
  - held body `3.287 deg`.
- L/7 frame 111:
  - raw `1.979 deg`;
  - proxy `19.356 deg`;
  - held node `15.769 deg`;
  - held body `5.836 deg`.
- L/7 frame 69:
  - raw `1.845 deg`;
  - proxy `19.950 deg`;
  - held node `8.046 deg`;
  - held body `6.157 deg`.

Interpretation:

- the proxy angular-velocity integration bug is proven;
- the object/visual hand are not simply copying the full proxy error, but they
  are being disturbed while the proxy is wrong;
- because the angular atom is disabled, any remaining object-axis/N/S/E/W issue
  must be in one of these places:
  - the direct body-B angular velocity vector passed to
    `SetBodyAngularVelocity`;
  - the target body frame passed into the FO4VR hard-keyframe helper;
  - body-A proxy velocity feeding the linear constraint with a moving/rotating
    anchor during solve;
  - visual hand/object relation being reconstructed from a live frame that is
    already solver-disturbed.

Current confidence:

- high confidence: exact proxy transform plus nonzero proxy angular velocity is
  wrong and must be removed or redesigned;
- medium confidence: this explains a large part of stutter/axis confusion
  because body A is not a stable palm anchor during solve;
- not yet proven: direct body-B angular vector convention is correct. The log
  only records its magnitude (`rawAng`, `appliedAng`, `capAng`), not the vector
  basis/sign, so it cannot fully prove pitch/yaw/roll correctness.

## Implementation Pass: Exact Proxy, Zero Velocity

Reason for the implementation shape:

- The confirmed bug is not "all transforms are wrong"; it is the hidden proxy
  being snapped exactly to the palm authority target and then told to continue
  rotating with nonzero angular velocity in the same solver step.
- The generated collider path already proves the cleaner policy: either drive a
  keyframed body with native keyframe motion, or place it exactly and zero its
  velocity. Do not combine exact placement with velocity toward the same target.
- This pass therefore keeps the current custom grab architecture intact and
  changes only body-A/proxy policy: exact transform target plus zero linear and
  angular velocity.
- Body-B/object angular authority is left active but now logs the actual raw and
  applied angular velocity vectors, not only magnitudes, so any remaining
  pitch/yaw/roll/N/S/E/W issue can be isolated from the proxy bug.

Code changes:

- `Hand::flushPendingCustomGrabAuthority`
  - still computes proxy target velocity for telemetry;
  - no longer applies that velocity to the proxy;
  - writes `setTransform(target)` followed by `setVelocity(0, 0)`;
  - log label changed to
    `bodyFrameConstraint+exactProxyZeroVelocity+hardKeyframeAngularVelocity`;
  - logs `proxyDrive=exactZeroVelocity`, zero applied proxy velocity, and
    `proxyCalcVel/proxyCalcAngVel` as diagnostics only.
- `Hand::applyProxyConstraintAngularVelocityDrive`
  - now returns raw and applied body-B angular velocity vectors for telemetry;
  - direct object angular authority remains unchanged except for logging.

Expected next runtime evidence:

- `PROXY GRAB AUTHORITY` should show:
  - `proxyDrive=exactZeroVelocity`;
  - `proxyVel=0.000hk`;
  - `proxyAngVel=0.000rad/s`;
  - nonzero `proxyCalcAngVel` only as diagnostic.
- `PROXY GRAB AFTER_SOLVE` should no longer show proxy angular error matching
  `proxyCalcAngVel * dt`. If it still does, something outside the explicit
  proxy velocity write is rotating the proxy.
- If visible object rotation remains world-direction dependent while proxy
  after-solve error is fixed, the next suspect is the logged `rawAngVec` /
  `appliedAngVec` body-B direct angular path.

## Runtime Pass: 19:32 Exact-Proxy Build Still Feels Wrong

Fresh test artifacts:

- deployed DLL: `D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll`
  - timestamp `2026-05-14 19:30:04`;
- log: `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log`
  - timestamp `2026-05-14 19:32:31`;
- the tested DLL therefore includes the exact-proxy/zero-velocity change.

### Confirmed From The Fresh Log

The proxy drift bug is fixed in the fresh run:

- `PROXY GRAB AUTHORITY` reports
  `diag=bodyFrameConstraint+exactProxyZeroVelocity+hardKeyframeAngularVelocity`;
- sampled proxy drive rows show:
  - `proxyDrive=exactZeroVelocity`;
  - `proxyVel=0.000hk`;
  - `proxyAngVel=0.000rad/s`;
  - nonzero `proxyCalcAngVel` only as diagnostic;
- `PROXY GRAB AFTER_SOLVE` aggregate:
  - rows: `203`;
  - proxy position max: `0.00gu`;
  - proxy rotation average: `0.00deg`;
  - proxy rotation max: `0.04deg`.

So the current user-visible failure is not the previous "proxy integrated its
own angular velocity" bug.

### Remaining Object Error

The held object still fails to reach the desired body/grip target by large
amounts after solve:

| Hand | Rows | Proxy rot max | Object pos avg/max | Object rot avg/max | Grip avg/max |
| --- | ---: | ---: | ---: | ---: | ---: |
| L | 203 | 0.04 deg | 17.13 / 31.90 gu | 6.49 / 36.40 deg | 17.00 / 31.88 gu |

Largest object-rotation errors:

| Seq/frame | Proxy rot | Object pos | Object rot | Grip |
| --- | ---: | ---: | ---: | ---: |
| `296/444` | 0.00 deg | 20.26 gu | 36.40 deg | 13.87 gu |
| `226/339` | 0.00 deg | 27.38 gu | 28.30 deg | 29.87 gu |
| `227/340` | 0.00 deg | 24.68 gu | 26.00 deg | 27.80 gu |
| `224/336` | 0.00 deg | 31.12 gu | 25.20 deg | 31.66 gu |
| `228/342` | 0.00 deg | 21.14 gu | 25.20 deg | 25.16 gu |

Largest grip errors:

| Seq/frame | Proxy rot | Object pos | Object rot | Grip |
| --- | ---: | ---: | ---: | ---: |
| `223/334` | 0.00 deg | 31.75 gu | 22.50 deg | 31.88 gu |
| `222/333` | 0.00 deg | 31.90 gu | 20.10 deg | 31.76 gu |
| `224/336` | 0.00 deg | 31.12 gu | 25.20 deg | 31.66 gu |
| `221/331` | 0.00 deg | 31.29 gu | 17.30 deg | 31.29 gu |
| `225/337` | 0.00 deg | 29.87 gu | 24.90 deg | 30.98 gu |

Interpretation:

- body-A/proxy is now exact;
- body-B/object is the layer failing to track;
- the visible hand still appears wrong because the visual hand path is
  `heldObjectWorld * inverse(frozenObjectHandSpace)`. It follows the held
  object relation. If the object is late or rotating wrongly, the visual hand
  looks late/wrong too.

### Direct Angular Writer Evidence

Sampled `PROXY GRAB AUTHORITY` rows prove body-B direct angular velocity is
being written, but they do not prove it is sufficient or conceptually correct:

| Seq | Target rot err | Applied angular speed | Approx one-step rotation | After-solve object rot |
| --- | ---: | ---: | ---: | ---: |
| `106` | 69.99 deg | 10.923 rad/s | 10.43 deg | 5.60 deg |
| `151` | 56.06 deg | 2.097 rad/s | 2.00 deg | 1.50 deg |
| `196` | 120.35 deg | 3.777 rad/s | 3.61 deg | 0.90 deg |
| `241` | 39.91 deg | 4.416 rad/s | 4.22 deg | 1.90 deg |
| `286` | 41.27 deg | 1.722 rad/s | 1.64 deg | 1.40 deg |

This suggests:

- the direct angular writer is active;
- the object often moves less than the requested one-step angular correction;
- the remaining failure is not "the object receives no angular command";
- the remaining failure is likely that direct body velocity is an inadequate
  replacement for a real angular solver constraint, or that the target frame
  given to the hard-keyframe helper still does not represent the body-B frame
  the solver actually integrates.

### Visual Hand Relationship

Relevant source path:

- `Hand::updateHeldGrabbedObject`
  - computes `targetVisualHandWorld =
    buildHeldObjectRelativeHandWorld(heldVisualNodeWorld,
    _grabFrame.rawHandSpace)`;
- `buildHeldObjectRelativeHandWorld`
  - returns `heldObjectWorld * inverse(frozenObjectHandSpace)`;
- `applyGrabExternalHandWorldTransform`
  - applies that visual-only external hand target.

That means the visual hand is intentionally a passenger of the held object
relation during `TouchHeld`.

Fresh log examples:

- `GRAB VISUAL HAND` rows show low translation deviation, but only translation
  is logged there;
- `GRAB FRAME HOLD` rows show huge visual/body relation errors in the same
  run, including:
  - owner/root/held errors around `174deg` at one sampled point;
  - `HELD dynamic` object rotation errors around `170deg` in the same window.

So the visual hand symptom is downstream of object/body frame error; current
logs do not prove a separate visual-hand authority bug.

### Legacy INI Pivot Status

Fresh telemetry still reports:

- `legacyPresent=yes`;
- `legacyActive=no`;
- `activeSource=rootFlattenedPalmAnchorTarget`.

The old INI pivot is still read and logged, but this run does not show it as
the active runtime pivot. It is not the current primary suspect.

The INI comments are stale, though: the active config still says ordinary
one-hand grabs use the native mouse spring, while the live log says
`drive=proxyConstraint`. That stale comment can confuse future debugging and
should be cleaned once the behavior is stable.

### Current Honest Inference

The exact-proxy fix was real but insufficient.

The remaining issue is in the object authority layer:

1. body-A/proxy reaches the root-flattened palm target exactly;
2. body-B/object remains far from the desired target after solve;
3. the visual hand follows body-B/object, so it looks wrong when body-B is
   wrong;
4. direct `SetBodyAngularVelocity` is active, but it is not giving the solver
   HIGGS-quality angular authority.

The next investigation should not keep changing axis conventions blindly.
It should prove one of these two branches:

- **Branch A:** the direct body-B target frame is still wrong. Evidence needed:
  every physics step should log current body axes, desired body axes, raw
  angular vector, applied angular vector, and actual next-frame body rotation
  delta in the same basis.
- **Branch B:** the target frame is correct, but velocity writes are not a
  strong/stable enough angular authority. Evidence needed: direct velocity
  command vs actual post-solve delta shows consistent under-response even when
  the axis is correct.

Until that is separated, tuning force caps or flipping rows/columns is not
grounded.

## Runtime Visual Pass: Hidden Proxy Convention Was Wrong

A proxy-only visual isolation switch was added and tested in game:

- all other debug visual flags were disabled in the active INI;
- only the hidden no-contact grab authority proxy body/axes were drawn;
- the real palm/finger colliders had already been visually verified as correct.

User observation:

- the hidden proxy follows the hand positionally;
- its rotation does not behave like the working palm/finger colliders;
- it appears to use wrong/random rotation conventions.

Conclusion:

- the flattened root-bone tree is not the bad layer;
- the generated palm/finger collider frame is not the bad layer;
- the bad layer is the explicit grab-authority adapter
  `generatedColliderFrameToGrabAuthorityFrame`;
- the older transpose assumption was wrong for the physical body-A proxy.

Correction:

- body-A/proxy now consumes the generated palm-anchor frame directly;
- the adapter boundary remains as a named function, but it is identity for the
  physical proxy frame;
- any relative/inverse transform needed by grab math must live in the captured
  object-to-proxy relation, not in a physical proxy frame transpose.

Expected next runtime result:

- with `bDebugDrawGrabAuthorityProxy=true`, the hidden proxy axes should rotate
  with the same convention as the visually correct palm/finger colliders;
- if object rotation remains wrong after that, the next suspect returns to the
  object-side saved relationship or body-B angular authority, not body-A.
