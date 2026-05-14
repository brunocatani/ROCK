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
