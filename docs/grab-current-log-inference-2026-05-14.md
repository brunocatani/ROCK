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
