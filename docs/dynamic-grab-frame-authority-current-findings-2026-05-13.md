# Dynamic Grab Frame Authority - Current Findings

Date: 2026-05-13

This file records the current dynamic-grab mapping pass so the frame-authority
reasoning survives compaction. It supersedes older notes where they conflict,
especially any older conclusion that the proxy/body-A frame was not involved in
the grab-start twist.

## Scope

- Dynamic grab only.
- Ordinary one-hand loose objects and loose non-equipped weapons are in scope.
- Equipped weapon grab, two-hand equipped weapon behavior, actor ragdoll grab,
  and keyframed grab are out of scope unless a shared dynamic-grab function is
  required to understand the current bug.
- COM is never grip pivot authority. COM is only valid for mass, inertia, lever
  length, and release/throw calculations.

## Current Conclusion

The current grab-start twist is a frame-authority mismatch:

- ROCK captures pivot, desired object transform, and visual hand reverse relation
  from the root-flattened raw hand frame.
- The custom proxy constraint drives body A from a generated palm/proxy frame.
- That generated palm/proxy frame is built through `matrixFromAxes(...)`.
- `matrixFromAxes(...)` writes the requested local axes in the opposite storage
  shape from the convention used by FO4VR composition and ROCK transform helpers.
- The solver therefore sees a different body-A rotation than the visual hand and
  desired object relation were captured against.

This explains the observed symptom:

- The object and visible hand rotate immediately when grab begins.
- After the bad initial seating, physical controller translation/rotation mostly
  follows correctly because the relation is then stable, just captured in the
  wrong authority frame.

## Argument Base

### FO4VR binary convention

Ghidra checked on `Fallout4VR.exe.unpacked.exe`:

- `0x1401A8D60` is the FO4VR transform compose helper used by many node/world
  update paths.
- Its translation composition applies a child-local vector through the parent
  basis as:
  - `out.x = child.x * parent[0] + child.y * parent[4] + child.z * parent[8] + parent[12]`
  - `out.y = child.x * parent[1] + child.y * parent[5] + child.z * parent[9] + parent[13]`
  - `out.z = child.x * parent[2] + child.y * parent[6] + child.z * parent[10] + parent[14]`
- In ROCK's `NiMatrix` view, local X world axis is stored as row 0:
  `(m00, m01, m02)`.

This matches ROCK's `transform_math::rotateLocalVectorToWorld(...)`:

- world X contribution uses `entry[0][0], entry[0][1], entry[0][2]`;
- world Y contribution uses `entry[1][0], entry[1][1], entry[1][2]`;
- world Z contribution uses `entry[2][0], entry[2][1], entry[2][2]`.

### ROCK source convention

These helpers agree with the FO4VR compose convention:

- `src/physics-interaction/TransformMath.h`
  - `rotateLocalVectorToWorld(...)`
  - `localPointToWorld(...)`
  - `composeTransforms(...)`
  - `niRowsToHavokColumns(...)`
  - `havokColumnsToNiRows(...)`
- `src/physics-interaction/hand/HandFrame.h`
  - `transformHandspaceLocalToWorld(...)`
  - comment explicitly references the FO4VR compose helper at `0x1401A8D60`.
- `src/physics-interaction/debug/DebugAxisMath.h`
  - `rotateNiLocalToWorld(...)`.
- `src/physics-interaction/grab/GrabTelemetry.h`
  - telemetry basis generation uses the same `TransformMath` convention.

### The inconsistent helper

`src/physics-interaction/hand/HandColliderTypes.h` currently has:

```cpp
matrix.entry[0][0] = xAxis.x;
matrix.entry[1][0] = xAxis.y;
matrix.entry[2][0] = xAxis.z;
matrix.entry[0][1] = yAxis.x;
matrix.entry[1][1] = yAxis.y;
matrix.entry[2][1] = yAxis.z;
matrix.entry[0][2] = zAxis.x;
matrix.entry[1][2] = zAxis.y;
matrix.entry[2][2] = zAxis.z;
```

That stores axes as columns. Under ROCK/FO4VR's verified local-vector convention,
the intended local axes must be stored as rows:

- row 0 = local X world axis;
- row 1 = local Y world axis;
- row 2 = local Z world axis.

### Why this reaches grab authority

Call chain:

- `buildPalmAnchorFrame(...)`
  - computes palm/finger/back axes;
  - calls `matrixFromAxes(...)`.
- `HandBoneColliderSet::makeRoleFrame(...)`
  - uses `buildPalmAnchorFrame(...)` for `PalmAnchor`, `PalmFace`, `PalmBack`.
- `HandBoneColliderSet::_latestPalmAnchorTarget`
  - stores the generated palm frame.
- `HandBoneColliderSet::tryGetPalmAnchorTarget(...)`
  - exposes the generated frame.
- `Hand::resolveGrabAuthorityProxyFrame(...)`
  - prefers `tryGetPalmAnchorTarget(...)`.
- `createProxyConstraintGrabDrive(...)`
  - creates the hidden proxy body-A authority from that proxy/palm frame.
- `updateProxyConstraintGrabDriveTarget(...)`
  - keeps the custom constraint using that proxy/body-A authority.

Therefore a transposed generated palm frame becomes the actual physics authority
for the custom dynamic grab.

## Fresh Runtime Log Evidence

The current runtime logs show the custom proxy constraint is active:

- `driveMode=proxyConstraint`
- `proxy constraint grab drive: ... proxyBody=... objBody=... reason=ordinary-dynamic-loose-object`

The logs also show the object-side target is internally coherent:

- `constraintObjectFrame=BODY`
- `objectAtGrabToDesiredObject.max=0.00`
- `grabBodyToDesiredBody.max=0.00`
- `transformBLocal == desiredTransformBLocal`
- `transformBErr=0.000..0.001gu`
- `targetErr(colsInv=0.000deg)`
- `rotationPreservedDeg=0.000` or near zero at capture/early held frames.

So the current failure is not:

- COM fallback;
- desired-object rotation at capture;
- transform-B translation storage;
- `target_bRca` low-level column storage;
- body-B pivot selection.

The failure is visible in the raw-hand versus proxy/palm authority split:

- Session 10 held frames show `rawToConRev` around `99..129deg` while
  `targetErr(colsInv=0.000deg)` remains clean.
- Session 12 held frames show `rawToConRev` around `121..125deg` while
  `targetErr(colsInv=0.000deg)` remains clean.
- Earlier capture frames showed `rawHandToProxyPalm.max` in the
  `95..172deg` range.
- Example axis pattern at capture:
  - raw hand axes and proxy palm axes are not a deliberate small palm offset;
  - they are transposed/permuted in the same way expected from the helper writing
    local axes into columns while the rest of ROCK reads them as rows.

This matches the player-visible symptom: the object/hand pair rotates at grab
start, then stabilizes in the wrong seated orientation.

## HIGGS Dynamic Grab Parity For This Case

HIGGS does not use a separate transposed palm-frame authority for dynamic grab.
It uses one hand-frame authority chain.

### Body-A hand authority

`Hand::ComputeHandCollisionTransform(...)`:

- Comment in HIGGS source: the hand transform must be where the hand is in real
  life because hand collision drives the grab constraint.
- Translation = real hand transform applied to hand collision box offset.
- Rotation = real hand transform rotation converted to Havok.

Meaning:

- HIGGS body A rotation is the real hand rotation.
- Palm/box offset changes the hand collision body's position/shape seat.
- It does not replace the hand rotation convention with a separately built palm
  frame.

### Pivot and desired object capture

`Hand::TransitionHeld(...)`:

- Starts with selected/contact point `closestPoint`.
- If graphics geometry is available, replaces that with the closest mesh point
  `triPos`.
- Stores that selected mesh/contact point as `ptPos`.
- Builds `desiredNodeTransform = adjustedTransform`.
- Applies only translation: `desiredNodeTransform.pos += palmPos - ptPos`.
- Object rotation is preserved unless an explicit authored/special path is used.

Dynamic constraint pivots:

- `hkPivotA = palmPos`.
- `hkPivotB = ptPos`.
- `pivotA = inverse(bodyA.transform) * hkPivotA`.
- `pivotB = inverse(bodyB.transform) * hkPivotB`.

### Constraint orientation and held update

`CreateGrabConstraint(...)`:

- body A = hand body.
- body B = grabbed body.
- transform A/B are set in body space.
- target relative orientation is initialized from transform B.

`GrabConstraintData::setTargetRelativeOrientationOfBodies(...)`:

- computes target from object-to-hand relation and transform A.
- FO4VR Ghidra helper `0x1419B26C0` has the same broad multiplication role:
  it updates target at `+0xD0` from supplied relative orientation and transform A.

Held update:

- `heldTransform = collidableNode->m_worldTransform`.
- `inverseDesired = InverseTransform(desiredNodeTransformHandSpace)`.
- visual hand = `heldTransform * inverseDesired`.
- constraint target and transform-B pivot are refreshed from the same
  `desiredNodeTransformHandSpace` relation.

Therefore HIGGS keeps:

- physical hand body frame;
- desired hand/object relation;
- visual hand reverse relation;
- angular target;
- object-side pivot update;

all tied to the same hand transform convention.

## HIGGS Mass, Force, Collision, And Deviation Behavior

These are still the correct polish targets after frame authority is coherent.
They must not be used to mask the current grab-start rotation bug.

HIGGS held update does:

- Registers selected/connected/contained body mass for held object weight.
- Registers player-space bodies for player movement compensation.
- Computes visual hand from the object relation, then measures deviation from
  real hand.
- Drops/ends hold if average deviation exceeds configured distance after ignore
  windows.
- Uses finite motors:
  - linear max force = generic object force or loose weapon force;
  - angular max force = linear force / angular-to-linear ratio;
  - mass cap = `min(linearMaxForce, mass * maxForceToMassRatio)`;
  - angular cap follows the capped linear force;
  - collision switches tau target to colliding tau;
  - tau is advanced/lerped, not snapped;
  - fade-in can temporarily alter angular-to-linear ratio.

Important separation:

- COM/mass affects force budget and inertia feel.
- COM/mass does not choose pivot, object orientation, hand relation, or initial
  seating frame.

## HIGGS Finger/Pose Behavior Relevant To Dynamic Grab

HIGGS hand posing is geometry-driven math, not special mesh marker dependence:

- It gathers nearby triangles around the chosen contact point.
- It transforms finger normals, zero-angle vectors, and start positions from hand
  space to world.
- It offsets finger starts by `palmToPoint = ptPos - palmPos`, as if the hand had
  already seated onto the object.
- It ray/intersection-tests per finger against nearby triangles.
- It maps intersections to curl values.
- Thumb can use an alternate curve if the normal thumb curve misses.
- Fingers clamp to a minimum curl/open value to avoid overcurl.
- Finger animation then blends local finger transforms over time.

This is separate from the current grab-start twist, but it reinforces the same
principle: HIGGS seats math around the selected contact point and palm relation,
not around COM.

## ROCK Current Dynamic Grab Map For This Bug

Current ROCK capture path already has the right object-side rules:

- `buildGrabPocketFrame(...)`
- `buildObjectGripArea(...)` with `.centerOfMassValid=false`
- `shiftObjectToAlignGripWithPocket(...)`
- explicit rejection of object-origin/COM fallback:
  - `object origin/COM fallback is not valid dynamic grab authority`

Current ROCK constraint path also has valid pieces:

- custom proxy constraint path is active;
- transform-B local target is stable;
- angular target byte storage now matches solver interpretation;
- finite force controller exists;
- mass cap and angular-to-linear force ratio exist;
- loose weapon force branch exists.

The current broken connection is the hand-side authority frame:

- capture/visual relation uses raw root-flattened hand;
- solver body-A uses generated palm/proxy frame;
- generated palm/proxy frame stores axes through `matrixFromAxes(...)`;
- that helper does not match the verified ROCK/FO4VR local-vector convention.

## What Previous Fixes Must Not Muddy

Do not chase these again until frame authority is fixed and retested:

- Do not move pivot to COM.
- Do not enable COM fallback.
- Do not tune mass, tau, damping, lead, or force to hide the initial snap.
- Do not revert transform-B/target column storage while logs show
  `targetErr(colsInv=0.000deg)`.
- Do not switch body-B authority back to MOTION/COM as a pivot model.
- Do not add mouse spring as a second authority on the same held body.

Older notes saying the proxy/body-A frame was not the source are superseded by
the current BODY-frame logs. Those older notes were from an earlier contaminated
state where the body-vs-motion split was still being investigated.

## Implementation Direction To Review Next

The next code change should fix frame coherence before any mass/feel tuning.

Two viable approaches:

1. Coherent shared fix:
   - Change the generated collider axis builder so local axes are stored in the
     same row-axis convention used by FO4VR compose and ROCK `TransformMath`.
   - This repairs palm anchor, generated hand colliders, and proxy body-A
     authority together.
   - It has broader blast radius but removes a real convention bug at the source.

2. Narrow grab-authority fix:
   - Add a dedicated correctly stored grab-authority proxy frame while leaving
     existing generated collider frames unchanged.
   - This isolates the grab-start twist but leaves possible semantic/contact
     collider orientation mismatch elsewhere.

Current evidence favors the coherent shared fix, with tests, because the same
helper constructs more than the proxy target. The generated hand/body colliders
and semantic contacts should not keep a different rotation convention from the
rest of ROCK.

## Required Tests / Acceptance Checks

Source-level tests:

- A matrix built from known local X/Y/Z axes must return those exact axes when
  passed through `transform_math::rotateLocalVectorToWorld(...)`.
- Palm-anchor frame construction must not transpose/permutate supplied axes.
- Left and right hand cases must be tested separately.
- Constraint angular target storage must keep the current passing interpretation:
  `targetErr(colsInv=0.000deg)` equivalent.

Runtime log gates:

- `objectAtGrabToDesiredObject.max` remains near zero for non-authored generic
  grabs.
- `grabBodyToDesiredBody.max` remains near zero at capture.
- `transformBLocal == desiredTransformBLocal`.
- `targetErr(colsInv=0.000deg)` remains clean.
- `rawHandToProxyPalm` no longer shows large 90/120/155/170 degree convention
  deltas at capture.
- First held frames no longer show the object/visual-hand initial rotation snap.
- Left and right hand telemetry remain separated.

Gameplay gates:

- Grab start does not rotate the object or visual hand into a wrong orientation.
- Once held, physical hand translation and rotation still map correctly.
- Loose non-equipped weapons follow the same dynamic frame authority.
- No COM fallback.
- Actor ragdoll grab remains unchanged.
- Equipped weapon behavior remains unchanged.

## Unresolved After Frame Fix

After the frame-authority bug is removed, continue HIGGS-style dynamic polish:

- mass/finite-force behavior for heavy objects and loose weapons;
- angular-to-linear force budget by mass and lever length;
- collision tau response;
- visual hand deviation/lag behavior;
- geometry-driven finger pose improvement;
- multipart loose weapon stability and cleanup.

Those are real quality gaps, but they should be evaluated only after the initial
body-A frame is coherent. Otherwise their telemetry will be contaminated by the
wrong hand authority frame.

## Continuation: Runtime Proof From Fresh Logs

This section records the latest findings before compaction. It supersedes any
older notes that treated transform-B, COM fallback, or angular target byte
storage as the active root cause of the current grab-start snap.

The current visible symptom is specific:

- immediately after grab starts, the object and visible hand rotate into a wrong
  orientation;
- after the object has seated into that wrong orientation, physical hand
  translation and rotation map mostly as expected;
- this means the per-frame relation becomes internally stable after capture, but
  the captured hand/object authority frame is wrong at grab start.

Fresh logs show the custom proxy constraint path is active:

- `drive=proxyConstraint`;
- `proxyFrame=rootFlattenedPalmAnchorTarget/ok`;
- `proxy constraint grab drive`;
- `constraintObjectFrame=BODY`.

The same fresh logs also show the object-side capture is not the failing piece:

- `objectAtGrabToDesiredObject.max=0.00` on the checked bad grabs;
- `grabBodyToDesiredBody.max=0.00` on the checked bad grabs;
- `transformBLocal == desiredTransformBLocal`;
- `targetErr(colsInv=0.000deg)`;
- `rotationPreservedDeg=0.000` or close enough to prove the desired object
  rotation was not recaptured from COM or object origin.

The failing value is the hand-side body-A/proxy frame:

- `rawHandToProxyPalm.max` is large at capture;
- raw hand axes and proxy palm axes are not merely offset by a palm translation;
- proxy palm orientation is exactly the transpose of the raw hand orientation in
  all sampled failed captures.

### Transpose Evidence

The following table was generated from the last twelve `GRAB BASIS CAPTURE`
records in `ROCK.log`. For each capture, it compares the proxy palm axes against
the raw hand axes directly, and against the transpose of the raw hand basis.

`Transpose*Deg = 0.00` means the proxy palm local axis exactly matches the
transposed raw hand axis. The direct axis columns show the actual runtime
misalignment the player sees as a 90/180 degree grab-start snap.

```text
Line   Side   TransposeXDeg  TransposeYDeg  TransposeZDeg  DirectXDeg  DirectYDeg  DirectZDeg
1482   right           0.00           0.00           0.00       90.00        0.00      180.00
1996   left            0.00           0.00           0.00      180.00      180.00        0.00
2833   right           0.00           0.00           0.00        0.00      180.00      180.00
3239   left            0.00           0.00           0.00       90.00       90.00       90.00
4128   left            0.00           0.00           0.00       90.00       90.00      180.00
5742   left            0.00           0.00           0.00        0.00      180.00      180.00
6574   left            0.00           0.00           0.00       90.00       90.00       90.00
7278   left            0.00           0.00           0.00       90.00       90.00      180.00
8254   right           0.00           0.00           0.00       90.00        0.00       90.00
8990   right           0.00           0.00           0.00        0.00       90.00      180.00
9755   left            0.00           0.00           0.00      180.00       90.00       90.00
10677  right           0.00           0.00           0.00        0.00      180.00       90.00
```

This is decisive because a random hand/object bug would not produce perfect
transpose matches across both hands and multiple object grabs. The generated
palm/proxy frame is being built in the opposite storage convention from the raw
hand frame and ROCK's transform helpers.

### Current Source Map For The Transpose

`src/physics-interaction/TransformMath.h` and the Ghidra-verified FO4VR compose
helper at `0x1401A8D60` agree on the local-vector convention:

- local X world axis is row 0 `(m00,m01,m02)` in ROCK's NiMatrix view;
- local Y world axis is row 1 `(m10,m11,m12)`;
- local Z world axis is row 2 `(m20,m21,m22)`;
- `rotateLocalVectorToWorld(matrix, vector)` follows that row-axis convention.

`src/physics-interaction/debug/DebugAxisMath.h` uses the same convention for
diagnostics, and `src/physics-interaction/grab/GrabTelemetry.h` uses
`transform_math::rotateLocalVectorToWorld(...)` for the stronger `GRAB BASIS`
orientation logs.

The generated hand/palm frame path is different:

- `src/physics-interaction/hand/HandColliderTypes.h`
- `matrixFromAxes(...)`
- `buildSegmentColliderFrame(...)`
- `buildPalmAnchorFrame(...)`

`matrixFromAxes(...)` currently writes the supplied axes as columns:

```cpp
matrix.entry[0][0] = xAxis.x;
matrix.entry[1][0] = xAxis.y;
matrix.entry[2][0] = xAxis.z;
matrix.entry[0][1] = yAxis.x;
matrix.entry[1][1] = yAxis.y;
matrix.entry[2][1] = yAxis.z;
matrix.entry[0][2] = zAxis.x;
matrix.entry[1][2] = zAxis.y;
matrix.entry[2][2] = zAxis.z;
```

Under the verified ROCK/FO4VR convention, if a helper promises "local X points
at `xAxis`, local Y points at `yAxis`, local Z points at `zAxis`", it must store
those axes as rows:

```cpp
row 0 = xAxis.x, xAxis.y, xAxis.z
row 1 = yAxis.x, yAxis.y, yAxis.z
row 2 = zAxis.x, zAxis.y, zAxis.z
```

This is exactly the transpose shown in runtime logs.

### Origin Collider / Proxy Source

The hand body used by grab authority is the generated palm anchor body, not the
raw root hand node:

- `Hand::createCollision(...)` calls `_boneColliders.create(..., _handBody)`;
- `HandBoneColliderSet::create(...)` creates `ROCK_LeftPalmAnchor` /
  `ROCK_RightPalmAnchor`;
- `_handBody` is set to that palm-anchor generated body;
- `Hand::updateCollisionTransform(...)` updates the same body through
  `_boneColliders.update(...)`;
- `Hand::flushPendingCollisionPhysicsDrive(...)` flushes the generated
  keyframed drive for that body;
- `Hand::resolveGrabAuthorityProxyFrame(...)` first resolves
  `_boneColliders.tryGetPalmAnchorTarget(outProxyWorld)`;
- only after that does it fall back to reading `_handBody`, and only later to raw
  hand.

So the hidden body-A proxy used by the custom grab constraint inherits the
generated palm-anchor frame. If that generated frame is transposed, the
constraint's body-A authority is transposed from the first solver frame.

### Palm Anchor Axis Meaning

`buildPalmAnchorFrame(...)` is not using arbitrary axes:

- `xAxis` is the finger/front direction from the root-flattened hand bone toward
  the average finger base position;
- `zAxis` is the back-of-hand direction from root-flattened hand local `+Z`;
- palm face is therefore `-zAxis`, consistent with `PalmFace` moving by
  `-lookup.backDirection`;
- `yAxis` is the cross-palm/lateral direction from `cross(backAxis, xAxis)`;
- translation is the average hand/finger palm center, projected off the current
  back offset and then moved slightly toward the palm face.

Those axis meanings are valid. The current problem is storage convention, not
the semantic choice of finger/front, lateral, and back-of-hand directions.

`RootFlattenedFingerSkeletonRuntime` separately computes palm normal as hand
local `(0,0,-1)`, while `HandBoneColliderSet` stores back direction as hand local
`(0,0,+1)`. This is consistent if code treats palm face as negative
back-direction. It is not currently evidence of the grab-start rotation bug.

### Capture Sequence That Explains The Visual Bug

The current failing sequence is:

1. Raw root-flattened hand frame is valid.
2. Three-phase/pocket object alignment preserves object rotation and only shifts
   translation so the selected grip point seats into the palm.
3. `resolveGrabAuthorityProxyFrame(...)` supplies the generated palm-anchor
   frame as the constraint/proxy authority frame.
4. That generated palm-anchor orientation is the transpose of raw hand.
5. `_grabFrame.rawHandSpace` is captured from raw hand to desired object.
6. `_grabFrame.constraintHandSpace` and `_grabFrame.constraintBodyHandSpace` are
   captured from transposed proxy palm to desired object/body.
7. Hidden proxy body-A is created from the transposed frame.
8. The custom constraint pulls body B to satisfy the transposed body-A relation.
9. The visible hand follows the held object from the raw relation, so both object
   and visible hand appear to rotate at the grab moment.
10. After seating into the wrong captured relation, subsequent physical hand
    motion maps mostly correctly because the relation is stable, just wrong.

This exactly matches the user's in-game report.

### HIGGS Parity For This Specific Case

HIGGS dynamic grab uses a hand collision transform whose rotation is the real
hand transform:

- selected/contact point chooses object-side pivot;
- palm point chooses hand-side pivot;
- object rotation is preserved at grab transition;
- body-A/hand frame drives the constraint from the real hand relation;
- COM is only mass/inertia/throw data, not pivot or frame authority.

The production lesson is not "copy HIGGS 1:1". FO4VR has hknp, generated
root-flattened bodies, and richer hand/body systems. The parity rule for this
bug is narrower:

- body-A rotation must be coherent with the raw physical hand authority frame;
- palm anchor may provide a better palm contact translation;
- palm anchor must not introduce a separate transposed rotation authority;
- COM must not be used as a fallback grip frame or fallback pivot.

### Current Non-Implementation Conclusion

The current root cause is not mass, damping, tau, lead, COM, transform-B, or
`target_bRca` storage. Those can still matter for later polish, but they are not
the reason the object and visible hand rotate immediately at grab start.

The current root cause is:

```text
generated palm/proxy body-A orientation == transpose(raw hand orientation)
```

That transposed body-A frame then becomes the custom constraint authority frame.

### Research Still Needed Before Code Change

Before changing implementation, the next evidence to collect or preserve is:

- confirm whether fixing `matrixFromAxes(...)` makes generated palm-anchor basis
  match raw hand basis within expected anatomical offsets;
- confirm whether segment colliders that use the same helper were visually or
  collision-wise depending on the old transposed storage;
- decide whether grab proxy authority should use:
  - palm-anchor translation plus raw hand rotation, or
  - fully corrected palm-anchor transform after shared axis storage fix;
- add a source test proving `matrixFromAxes(...)` and
  `rotateLocalVectorToWorld(...)` agree on local X/Y/Z;
- add separate left/right runtime acceptance logs for direct raw-to-proxy axis
  deltas and transpose-match deltas.

The most likely durable fix is a coherent generated-frame convention correction,
not a mass/force tune and not another COM fallback.

## Continuation: Origin Collider And Body-A Frame Chain

This pass inspected the source chain from root-flattened hand bones to generated
origin collider to hidden grab proxy. The goal was to separate three questions
that were getting mixed together:

- what is the origin collider used for reference;
- what does "finger front" mean;
- where exactly the after-grab rotation bug enters the hand/object relation.

### Origin Collider Definition

In current ROCK, the "origin collider" for hand physics is not the old legacy
box and not the raw controller/root hand node. It is the generated palm-anchor
body:

```text
Hand::createCollision
  -> HandBoneColliderSet::create(world, bhkWorld, isLeft, _handBody)
      -> makeRoleFrame(..., PalmAnchor, anchorFrame)
      -> palmAnchorBody.create(..., "ROCK_LeftPalmAnchor"/"ROCK_RightPalmAnchor")
      -> placeGeneratedKeyframedBodyImmediately(palmAnchorBody, anchorFrame.transform)
      -> _latestPalmAnchorTarget = anchorFrame.transform
      -> publishAtomicBodyIds(palmAnchorBody, isLeft)
```

`_handBody` is therefore the palm anchor. The non-anchor generated bodies
(`PalmFace`, `PalmBack`, finger segments, etc.) are sibling semantic/contact
bodies, not the body-A authority used directly by the dynamic grab proxy.

During update:

```text
Hand::updateCollisionTransform
  -> HandBoneColliderSet::update(world, isLeft, _handBody, deltaTime)
      -> captureBoneLookup(GameRootFlattenedBoneTree)
      -> makeRoleFrame(..., PalmAnchor, anchorFrame)
      -> _latestPalmAnchorTarget = anchorFrame.transform
      -> queueBodyTarget(_handBody, anchorFrame.transform, ...)

Hand::flushPendingCollisionPhysicsDrive
  -> HandBoneColliderSet::flushPendingPhysicsDrive(...)
      -> driveGeneratedKeyframedBody(_handBody, _palmAnchorDriveState, ...)
```

Dynamic grab does not use the live palm-anchor hknp readback first. It uses the
sampled target:

```text
Hand::resolveGrabAuthorityProxyFrame
  -> _boneColliders.tryGetPalmAnchorTarget(outProxyWorld)
      source = "rootFlattenedPalmAnchorTarget"
  -> fallbackPalmAnchorWorld only if no sampled target exists
  -> live _handBody readback only after that
  -> rawHandFallback last
```

That means the custom hidden body-A proxy is initialized and driven from the
generated palm-anchor target frame.

### Body Reference Map

The current one-hand dynamic grab has four relevant frames:

```text
raw hand frame
  Source: root-flattened hand/controller handWorldTransform.
  Role: visual hand relation and real physical hand authority.

generated palm-anchor frame
  Source: HandBoneColliderSet::buildPalmAnchorFrame.
  Role: _handBody, semantic hand origin, grab proxy target source.

hidden grab authority proxy body A
  Source: created from proxyFrameWorldAtGrab, then driven from proxy target.
  Role: keyframed no-contact body A for custom finite-force constraint.

held object/body B
  Source: selected object's hknp BODY frame, not MOTION/COM.
  Role: dynamic constrained object. Pivot B is frozen selected contact point.
```

Current capture stores two object relations:

```text
_grabFrame.rawHandSpace
  = inverse(rawHandWorld) * desiredObjectWorld
  Used to derive the visible hand relation from the held object.

_grabFrame.constraintHandSpace
  = inverse(proxyFrameWorldAtGrab) * desiredObjectWorld
  Used to drive body B from body A/proxy.
```

This split is valid only if `rawHandWorld` and `proxyFrameWorldAtGrab` are
coherent hand frames. They may have a palm translation offset. They must not
disagree by a matrix transpose.

### Finger Front Definition

`finger front` is not a magic authored marker and not a HIGGS/ROCK mesh node. In
current ROCK it is derived from the root-flattened live hand skeleton:

```text
fingerCenter = average(fingerBases[thumb,index,middle,ring,pinky])
palmCenter   = average(hand.translate + all fingerBases)
xAxis        = normalize(fingerCenter - hand.translate)
```

So `xAxis` means "from hand bone toward the finger bases". It is the palm/finger
front direction for generated hand collider geometry.

`backDirection` is also live skeleton data:

```text
backDirection = rotateNiLocalToWorld(hand.rotate, (0,0,+1))
zAxis         = normalize(backDirection)
palm face     = -zAxis
```

`yAxis` is the lateral/cross-palm direction:

```text
yAxis = normalize(cross(backDirection, xAxis))
xAxis = normalize(cross(yAxis, backDirection))
zAxis = backDirection
```

The semantic axis selection is reasonable. The bug is not "finger front is the
wrong idea"; the bug is that the axes are stored into the matrix in the opposite
shape from the rest of ROCK's transform math.

### Exact Transpose Mechanism

`TransformMath::rotateLocalVectorToWorld(matrix, vector)` uses the verified
ROCK/FO4VR local-vector convention:

```text
local X world axis = (m00, m01, m02)
local Y world axis = (m10, m11, m12)
local Z world axis = (m20, m21, m22)
```

`GeneratedKeyframedBodyDrive::makeHavokTransform` and
`grab_authority_proxy::makeHavokTransform` both convert through
`niRotToHkTransformRotation(...)`, which calls
`transform_math::niRowsToHavokColumns(...)`. That conversion is correct only
when the incoming Ni matrix already follows the row-axis convention above.

`matrixFromAxes(...)` creates the opposite:

```text
entry[0][0] = x.x    entry[0][1] = y.x    entry[0][2] = z.x
entry[1][0] = x.y    entry[1][1] = y.y    entry[1][2] = z.y
entry[2][0] = x.z    entry[2][1] = y.z    entry[2][2] = z.z
```

When that matrix is later interpreted by `rotateLocalVectorToWorld(...)`:

```text
local X reads (entry[0][0], entry[0][1], entry[0][2])
             = (x.x, y.x, z.x)

local Y reads (entry[1][0], entry[1][1], entry[1][2])
             = (x.y, y.y, z.y)

local Z reads (entry[2][0], entry[2][1], entry[2][2])
             = (x.z, y.z, z.z)
```

That is the transpose of the intended basis.

The latest log line `10677` shows this directly:

```text
rawHand.X=(0.556,0.352,0.753)
rawHand.Y=(-0.608,-0.446,0.657)
rawHand.Z=(0.567,-0.823,-0.034)

proxyPalm.X=(0.560,-0.604,0.567)
proxyPalm.Y=(0.355,-0.444,-0.823)
proxyPalm.Z=(0.749,0.662,-0.034)
```

Those proxy axes equal the raw hand matrix read by columns:

```text
proxyPalm.X ~= (rawX.x, rawY.x, rawZ.x)
proxyPalm.Y ~= (rawX.y, rawY.y, rawZ.y)
proxyPalm.Z ~= (rawX.z, rawY.z, rawZ.z)
```

That is why every sampled capture produced a perfect transpose match, while
direct raw-to-proxy axis comparisons produced 90/180-degree-looking errors.

### Why The Bug Appears Only At Grab Start

The runtime behavior now has a coherent explanation:

```text
grab capture:
  object rotation is preserved
  object translation is shifted so selected contact seats at palm
  raw visual relation is captured from raw hand
  constraint relation is captured from transposed proxy palm

first solver frames:
  hidden body-A proxy follows transposed generated palm frame
  finite constraint rotates body B into the relation captured against that frame
  visible hand is then solved from body B using the raw visual relation
  result: both object and visible hand appear to rotate at attachment

after stabilization:
  the object is now seated in the wrong captured relation
  body A and body B continue to move coherently
  physical hand translation/rotation appears to match again
```

This matches the user-observed bug: the wrong rotation happens at the grab
moment, not as a continuous drift after the held state settles.

### HIGGS Comparison For The Same Relation

HIGGS dynamic grab confirms the design rule but not a 1:1 FO4VR implementation:

- `ComputeHandCollisionTransform(...)` uses the real hand transform rotation for
  hand collision/body-A. The configurable hand collision box offset changes
  translation, not rotation authority.
- `TransitionHeld(...)` computes `desiredNodeTransform = adjustedTransform` and
  then only translates it by `palmPos - ptPos`, preserving object rotation.
- `desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform` uses the
  real hand frame as the hand/object relation.
- Pivot A is the palm point in body-A space.
- Pivot B is the selected object contact point in body-B space.
- Held update derives the visible hand from the held object and the same captured
  desired relation, then refreshes constraint orientation/pivot from that
  relation.

So the parity rule is:

```text
hand/body-A rotation authority must be the same real hand rotation used for the
captured visual relation, while palm/contact offsets may alter position.
```

FO4VR can do this with richer generated bodies and hknp proxies, but the proxy's
rotation cannot be an independently transposed generated-collider frame.

### Current Implementation Implication

The most likely durable implementation direction is one of these two, to be
decided after review:

1. Shared generated-frame correction:
   - fix `matrixFromAxes(...)` to store local axes in ROCK/FO4VR row-axis form;
   - generated palm anchor, finger colliders, and proxy source all become
     convention-coherent;
   - requires validating generated collider contact behavior because this has a
     broader blast radius.

2. Grab-authority frame correction:
   - keep palm-anchor translation as the better palm reference;
   - use raw hand rotation for hidden proxy/body-A authority;
   - prevents the grab-start snap even if generated colliders need separate
     migration/testing.

The source evidence favors correcting the shared convention because the same
helper also orients finger and palm collider shapes. However, the safest runtime
validation should explicitly compare both possibilities in telemetry before
shipping a broader collider-orientation change.

### Telemetry To Add If More Data Is Needed

If implementation or another diagnostic pass is allowed later, the next logging
should avoid old column-based ambiguity and record the exact direct/transpose
relation:

```text
side=L/R
proxySource=rootFlattenedPalmAnchorTarget/rawHandFallback/...
rawToProxyDirectDeg=(x,y,z,max)
rawToProxyTransposeDeg=(x,y,z,max)
palmAnchorUsesRawRotation=yes/no
palmAnchorTranslationDelta=(x,y,z,len)
constraintBodyAToRawHandDeg=(x,y,z,max)
objectRotationPreservedDeg=...
bodyBTargetErr=...
```

Acceptance gate for this bug:

```text
objectRotationPreservedDeg ~= 0
rawToProxyTransposeDeg is not the best match anymore
rawToProxyDirectDeg is near zero, or intentionally small if palm-frame
  anatomical rotation is deliberately different
first-frame visual hand/object snap gone
COM fallback remains disabled
```

## Continuation: Visible Hand Attachment Path

This pass mapped the visual hand side of the same bug. The visual symptom is
that the hand appears to attach to the object and rotate with it at grab start.
The current code confirms that the visual hand layer is not an independent
physics authority; it follows the held object using the frozen raw-hand relation.

The external hand target is sent through FRIK:

```text
GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual"
GRAB_EXTERNAL_HAND_PRIORITY = 90

applyGrabExternalHandWorldTransform(...)
  -> FRIKApi::applyExternalHandWorldTransform(
         "ROCK_GrabVisual",
         left/right,
         adjustedHandTransform,
         90)
```

During held update, the target is computed as:

```text
heldVisualNodeWorld = _grabFrame.heldNode->world
  or deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.bodyLocal)

targetVisualHandWorld =
  hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(
      heldVisualNodeWorld,
      _grabFrame.rawHandSpace)

buildHeldObjectRelativeHandWorld(heldObjectWorld, frozenObjectHandSpace)
  = heldObjectWorld * inverse(frozenObjectHandSpace)
```

So the visual hand is driven from the held object and the raw-hand capture
relation:

```text
_grabFrame.rawHandSpace = inverse(rawHandWorldAtGrab) * desiredObjectWorldAtGrab
visual hand target      = currentHeldObjectWorld * inverse(rawHandSpace)
```

This mirrors the HIGGS held update pattern:

```text
HIGGS:
  inverseDesired = InverseTransform(desiredNodeTransformHandSpace)
  m_adjustedHandTransform = heldTransform * inverseDesired
```

The visual hand bug is therefore downstream of the object/body-A bug:

1. `rawHandSpace` is captured correctly from raw hand to desired object.
2. `constraintHandSpace` is captured against the transposed proxy palm.
3. The custom constraint rotates the object into the transposed proxy relation.
4. The visual hand follows the now-rotated object via `rawHandSpace`.
5. The player sees both object and visible hand rotate at attachment.

This means the visual hand layer should not be fixed by changing the visual
formula. The formula is the correct held-object-relative formula. The fix must
make the object stop rotating into the wrong body-A/proxy frame in the first
place.

Visual-hand interpolation does not hide this bug because its target is already
wrong:

- if lerp is disabled, the hand snaps directly to the wrong target;
- if lerp is enabled, the hand moves toward the wrong target over time;
- either way, the object has already been given a transposed body-A authority
  target, so visual smoothing cannot restore the original object rotation.

### Visual Path Boundary

Do not treat these as root causes for the current after-grab rotation:

- FRIK external transform API;
- `ROCK_GrabVisual` priority;
- `hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(...)`;
- `rawHandSpace` visual formula;
- visual hand lerp speed or angular speed.

They may need polish later, but changing them now would mask the symptom while
leaving the object driven by the wrong constraint authority frame.

## 2026-05-13 Implementation Contract: Generated Hand/Proxy Frame Convention

### Problem To Fully Fix

The remaining attach-time rotation bug is not a COM pivot bug and not a visual
hand formula bug. The capture logs showed the object-side state was sane:

- the selected contact/object-side pivot stayed local to the grabbed object;
- the desired object rotation was preserved at grab start;
- COM was not required to explain the immediate rotation;
- the visual hand followed the held object through the frozen raw-hand relation.

The bad relation was between the raw/root-flattened palm frame and the generated
physics palm/proxy frame used as body A. At grab start, the generated frame axes
were the transpose of the frame convention consumed by ROCK's transform helpers.
That made the custom authority path solve the held object toward a different
palm orientation than the one used by pivot capture and visual-hand reverse
relation. The visible hand then looked broken because it was following an object
that had already been rotated by the wrong body-A frame.

### Required Fix

Correction after collider-scope review: generated body/hand colliders were
already verified correct in-game with the existing column-stored collider
convention. The grab bug must therefore be fixed at the grab-authority boundary,
not by changing shared collider placement.

`hand_bone_collider_geometry_math::matrixFromAxes(...)` must keep the generated
collider convention:

Required storage:

```text
column 0 = generated local X axis in world space
column 1 = generated local Y axis in world space
column 2 = generated local Z axis in world space
```

Dynamic grab must convert only the palm-anchor/proxy transform into the
grab-frame convention before storing `constraintHandSpace` and
`constraintBodyHandSpace`:

```text
grab proxy frame = generatedColliderFrameToGrabAuthorityFrame(palm anchor frame)
                 = same translation and scale, transposed rotation
```

This preserves all generated hand/body collider placement while preventing the
custom grab constraint from using the physical collider storage convention as
the body-A grip authority convention.

### Files In The Grab-Only Adapter Build

- `src/physics-interaction/hand/HandColliderTypes.h`
  - owns `matrixFromAxes(...)`;
  - owns generated palm/finger collider frame construction;
  - originally kept the verified generated collider column convention;
  - owns `generatedColliderFrameToGrabAuthorityFrame(...)`, the explicit
    grab-only adapter.
- `src/physics-interaction/hand/HandGrab.cpp`
  - adapts `_boneColliders.tryGetPalmAnchorTarget(...)` output before using it
    as the custom grab proxy/body-A authority frame;
  - adapts palm-anchor fallback/readback frames the same way;
  - leaves raw hand fallback unadapted because it is not a generated collider
    frame.
- `tests/HandColliderFramePolicyTests.cpp`
  - originally locked `matrixFromAxes(...)` to column-stored generated collider
    axes;
  - originally verified the grab-only adapter converted palm and segment
    generated frames into the local-vector convention expected by grab math.
- `CMakeLists.txt`
  - registers `ROCKHandColliderFramePolicyTests`;
  - includes it in the CommonLib-backed policy test target list.

### What The Grab-Only Adapter Build Did Not Change

- It does not use COM as pivot authority.
- It does not change selected contact pivot capture.
- It does not change the visual-hand formula.
- It does not retune mouse spring, motors, mass, tau, damping, or lead.
- It does not change actor ragdoll grab, equipped weapon grab, or two-hand
  weapon handling.
- It does not change generated hand/body collider placement convention.
- It does not remove the diagnostic/proxy scaffolding already present in the
  working tree.

### Runtime Proof Needed After Build

A fresh in-game grab log and overlay should show the generated palm, converted
grab-authority palm, and proxy/body-A readback as separate frames:

```text
GRAB BASIS FRAMECHAIN ...
  generatedPalm=yes
  grabAuthority=yes
  proxyReadback=yes
  nativeToPalm=...
  palmToAuthority=...deg
  authorityToProxy=...deg
```

With `bDebugGrabTransformTelemetry`, `bDebugGrabTransformTelemetryAxes`, and
optionally `bDebugGrabTransformTelemetryText` enabled, the overlay draws:

- short palm triad offset to one side: generated palm collider frame consumed
  directly by grab math;
- longer center palm triad: generated palm converted to grab-authority frame;
- longest palm triad offset to the other side: live proxy/body-A readback;
- existing object triads: desired object and actual held node/body frames.

The visual reference is the hand, not the world and not the grabbed object:

- red/local X should run from the wrist/hand root toward finger bases;
- blue/local Z should point out the back of the hand;
- negative blue should point into the palm/object;
- green/local Y should run across the palm.

The object should no longer rotate immediately at attachment. If an attach-time
rotation remains after this axis fix, the next data to collect is not more COM
data. The next target is the active body-A transform source at the exact
between-collide-and-solve write/readback boundary:

- queued proxy target world transform;
- proxy body transform immediately after setter;
- proxy body transform after solve;
- raw hand/root-flattened hand transform for the same frame;
- object desired transform and actual body transform for the same frame;
- separated left/right session ids.

### Completion Criteria For This Frame Bug

- Release build succeeds.
- Full CTest suite passes.
- `ROCK.dll` and `ROCK.pdb` are deployed to the production ROCK plugin folder.
- In-game attach test no longer rotates the object/visual hand at grab start.
- If the in-game test still fails, the failure is treated as a new boundary
  mismatch after the generated-frame transpose has been eliminated, not as a
  reason to reintroduce COM fallback or visual-hand hacks.

## 2026-05-13 Full Generated Collider Convention Diagnostic

The grab-only adapter build preserved generated hand/body collider placement and
converted only at the grab-authority boundary. The next diagnostic build tests
the other side of that fork explicitly: change the generated collider frame
policy itself and make the grab-boundary adapter identity.

### What Changed

- `matrixFromAxes(...)` now stores generated X/Y/Z axes in ROCK's Ni
  local-vector convention:
  - matrix row 0 = generated local X in world;
  - matrix row 1 = generated local Y in world;
  - matrix row 2 = generated local Z in world.
- `generatedColliderFrameToGrabAuthorityFrame(...)` remains as an explicit
  boundary function, but now returns the input frame unchanged.
- Hand, palm, finger, and body generated colliders built through
  `hand_bone_collider_geometry_math::buildSegmentColliderFrame(...)` and
  `buildPalmAnchorFrame(...)` all participate in this diagnostic convention.
- The visual grab telemetry still draws generated palm, grab authority, and live
  proxy/readback triads, but generated direct-to-authority rotation should now be
  near zero. If it is not, the mismatch is outside this helper.

### What This Proves

This build answers whether the previous in-game collider behavior was hiding a
global basis convention mismatch. If generated hand/body colliders visually
rotate or translate incorrectly, then the old column-stored generated collider
policy was required for native body placement and the grab problem is elsewhere.
If colliders remain correct and grab attach rotation improves, then the previous
grab-only adapter was too narrow and the generated collider convention itself was
part of the grab-frame mismatch.

This remains a frame-convention diagnostic. It does not change COM, pivot
selection, mouse-spring tuning, mass force, angular authority, or hand pose
logic.

## 2026-05-13 Clean Data Pass

The full-convention build needs cleaner proof than visual impression alone.
Two diagnostics were tightened:

- Palm debug overlay wording now states the truth: generated-direct and
  grab-authority palm triads are drawn at separate origins for readability. They
  should match in basis, not physically overlap on screen. The numeric
  `directToAuthority` rotation is the proof.
- Generated keyframed body drive telemetry now records target-vs-live body
  rotation:
  - `bodyRotErr` = total target-to-readback rotation error;
  - `axisDeg=(x,y,z)` = per local-axis target/readback angular mismatch;
  - `targetX/Y/Z` and `bodyX/Y/Z` = the actual world basis vectors being
    compared.

With `bDebugGrabFrameLogging=true`, the log should now include sampled lines:

```text
Generated body frame compare owner=... bodyIndex=... bodyId=...
  bodyDeltaGame=...
  bodyRotErr=...
  axisDeg=(...)
  targetX=(...) targetY=(...) targetZ=(...)
  bodyX=(...) bodyY=(...) bodyZ=(...)
```

Interpretation:

- `directToAuthority ~= 0deg` and generated hand/body `bodyRotErr` large:
  the shared generated frame is internally consistent, but the native generated
  body placement/readback path is consuming rotation differently.
- generated hand/body `bodyRotErr ~= 0deg` while in-game colliders look rotated:
  the debug visual/reference expectations are wrong or the rendered overlay is
  not attached to the same body being tested.
- generated hand/body `bodyRotErr ~= 0deg` and grab proxy/readback diverges:
  the next fault is after palm target capture, inside proxy body-A drive or
  constraint update timing.
- `owner=weapon-collision` body telemetry is for equipped/generated weapon
  collision bodies only. It is not evidence for loose dynamic weapon grab,
  because loose grabbed weapons use the object's existing/native collision in
  this path.

## 2026-05-13 Current Stage Checkpoint

### Current Build Stage

Active diagnostic build:

- `d1ffe81 fix/grab-frame: test unified generated collider convention`
- `1a802aa fix/grab-frame: add generated body rotation telemetry`

The deployed build intentionally tests the full generated-collider convention
flip:

- generated hand colliders use the new shared `matrixFromAxes(...)` convention;
- generated body colliders that reuse `buildSegmentColliderFrame(...)` use the
  same convention;
- the generated palm anchor target uses the same convention;
- the hidden grab-authority proxy uses the same palm frame;
- `generatedColliderFrameToGrabAuthorityFrame(...)` is now identity;
- equipped weapon generated/managed collision is not part of this loose dynamic
  grab diagnostic. Loose non-equipped grabbed weapons do not get ROCK-generated
  weapon colliders here; they use their existing/native object collision as the
  grabbed physics body.

This is a diagnostic build, not a final physics-quality solution. It does not
change COM, pivot selection, mass, hand pose, mouse spring response, or angular
authority.

### Current Runtime Issue Being Investigated

The visible symptom before this checkpoint:

- at the grab moment, the grabbed object and visible hand rotate unexpectedly;
- after the initial snap/rotation, physical controller translation and rotation
  mostly match expected movement;
- this points to attach-time frame capture / body-A proxy / generated-frame
  convention, not to COM-based pivoting or post-hold input tracking.

The current question:

- Are generated hand/body colliders and the hidden grab-authority proxy using a
  consistent transform convention?
- If they are consistent in code, does FO4VR's native generated-body placement
  consume that convention differently when the body is actually driven?

### Test Setup Expected In Game

The active production INI should have:

```ini
bDebugGrabFrameLogging = true
bDebugGrabTransformTelemetry = true
bDebugGrabTransformTelemetryText = true
bDebugGrabTransformTelemetryAxes = true
iDebugGrabTransformTelemetryLogIntervalFrames = 1
```

In game, test a normal loose one-hand dynamic object grab and, if possible, one
loose non-equipped weapon grab. Use the right and left hands separately if time
allows; logs are side-separated.

Visual overlay expectations:

- generated palm direct triad and palm authority triad are drawn at different
  positions for readability, so do not judge by screen overlap;
- judge their orientation/basis by the axes and by the numeric
  `directToAuthority` log value;
- proxy/readback triad shows what the hidden authority body actually has after
  native body placement/readback;
- red/local X should point toward finger bases;
- blue/local Z should point out the back of the hand;
- negative blue should point into palm/object;
- green/local Y should run across the palm.

### Log Lines To Capture

The important log families:

```text
GRAB BASIS FRAMECHAIN ...
```

Use this for:

- `nativeToPalm`
- `nativeToAuthority`
- `nativeToProxy`
- `palmToAuthority`
- `authorityToProxy`
- generated palm basis
- grab-authority palm basis
- proxy/readback basis

```text
GRAB BASIS LEGACY_PIVOT ...
```

Use this only for contamination proof:

- `legacyToRuntime`
- `legacyToPalm`
- `legacyToAuthority`
- `legacyToProxy`

```text
Generated body frame compare owner=...
```

Use this for generated collider native-drive proof:

- `owner=hand-bone-collider`
- `owner=body-bone-collider`
- `bodyDeltaGame`
- `bodyRotErr`
- `axisDeg=(x,y,z)`
- `targetX/Y/Z`
- `bodyX/Y/Z`

Do not use `owner=weapon-collision` as evidence for loose dynamic grabbed
weapons. That owner belongs to equipped/generated weapon collision work, which
is separate from loose object/weapon dynamic grab.

```text
PROXY GRAB AFTER_SOLVE ...
PROXY GRAB AUTHORITY ...
```

Use this for body-A proxy timing/readback if the palm boundary is clean but the
grab still snaps.

### Result Matrix

#### Result A: Hand/body colliders visually break

Meaning:

- the old column-stored generated collider convention was required for native
  generated body placement;
- the unified convention flip is not production-safe.

Effect on next work:

- revert only the full generated-collider convention flip;
- keep the generated-body rotation telemetry;
- return to a grab-only adapter or a more explicit native-placement adapter;
- do not treat COM or mass as related to this failure.

#### Result B: Hand/body colliders look correct, `directToAuthority ~= 0deg`,
but generated hand/body `bodyRotErr` is large

Meaning:

- sampled generated frames are internally consistent;
- the native generated-body placement/readback path is consuming or returning
  rotation in a different convention.

Effect on next work:

- keep investigating `niRotToHkTransformRotation(...)`,
  `driveGeneratedKeyframedBody(...)`, `setTransform(...)`, and readback source
  convention;
- likely fix belongs at the native generated-body placement/readback boundary,
  not in grab pivot logic.

#### Result C: Hand/body colliders look correct and generated hand/body
`bodyRotErr ~= 0deg`, but `authorityToProxy` is large

Meaning:

- generated collider and native generated-body drive are likely correct;
- hidden proxy/body-A authority is diverging after palm target capture.

Effect on next work:

- inspect proxy target queue/flush, between-collide-and-solve timing, and
  after-solve readback;
- compare queued proxy target, immediate setter readback, and after-solve
  readback for the same session/frame.

#### Result D: `directToAuthority ~= 0deg`, generated body `bodyRotErr ~= 0deg`,
`authorityToProxy ~= 0deg`, but object/visual hand still snaps at grab start

Meaning:

- the palm/proxy generated-frame convention is probably not the attach snap
  source;
- the next suspect is object-side capture: `objectToBodyAtGrab`,
  `objectToConstraintBodyAtGrab`, transform-B, angular target, or desired
  object/held-node relation.

Effect on next work:

- map grab-start object frame capture and constraint target construction;
- do not keep changing hand collider conventions.

#### Result E: Loose grabbed weapons behave differently from generic objects

Meaning:

- do not jump to equipped weapon generated-collider conclusions;
- loose non-equipped weapon grab uses the grabbed weapon's existing/native
  collision body, the same dynamic-grab authority path as other loose objects;
- equipped weapon collisions, equipped weapon handling, and two-hand equipped
  weapon logic are separate systems and out of scope for this test.

Effect on next work:

- compare loose weapon sessions through `GRAB BASIS FRAMECHAIN`,
  proxy/readback, desired object, held body/object transform, and release logs;
- do not use `owner=weapon-collision` logs to explain loose grabbed weapon
  behavior unless a later implementation explicitly adds generated weapon
  colliders to loose dynamic grabs;
- if loose weapons differ while generic objects are clean, inspect object-side
  frame capture, multipart body selection, native collision shape/body relation,
  and weapon form/type-specific tuning in the dynamic grab path.

### What This Test Decides

This test decides whether to keep or reject the full generated-collider
convention flip.

It does not decide final grab quality, mass feel, HIGGS-style weight, or motor
architecture. Those remain later work after the attach-time frame snap is
resolved with clean evidence.

## 2026-05-13 Checkpoint: Legacy Pivot A Is Still Active

### Confirmed code path

The manually configured grab pivot was not removed.

- Config keys still exist:
  - `fRightGrabPivotAHandspaceX/Y/Z`
  - `fLeftGrabPivotAHandspaceX/Y/Z`
- Runtime config fields still exist:
  - `g_rockConfig.rockRightGrabPivotAHandspace`
  - `g_rockConfig.rockLeftGrabPivotAHandspace`
- `computeGrabPivotAHandspacePosition(bool isLeft)` returns those config
  values.
- `computeGrabPivotAPositionFromHandBasis(...)` transforms that configured
  point through the live hand transform.
- `Hand::computeGrabPivotAWorld(...)` returns that transformed configured
  point.

This means the runtime `pivotA` currently means:

```text
legacy configured handspace point
    -> transformed by the root-flattened live hand frame
    -> used as the hand-side grab anchor
```

It does not currently mean:

```text
bone-derived generated palm center
```

### Why this matters

The generated palm collider and the root-flattened finger/palm evidence are now
available, but they have not fully replaced the old configurable pivot as active
grab authority. The generated palm path is currently used for contact evidence,
debug visualization, proxy/authority diagnostics, and collider placement. The
legacy configured pivot still feeds the main grab anchor path through
`computeGrabPivotAWorld(...)`.

That legacy point is not a trustworthy diagnostic reference because its original
calibration basis predates the flattened bone-tree palm data. Using it as the
expected palm would contaminate the logs. The correct use during this diagnostic
stage is only to expose whether the old pivot is still affecting runtime and how
far it is from the bone-derived palm/proxy chain.

There is no intended production reason to keep the old INI pivot as dynamic-grab
authority once the flattened-bone palm/grip anchor replaces it. The source map
above already proves that it is still wired today. The runtime logs are not
needed to prove that fact; they are only needed to measure how far the legacy
anchor is from the bone-derived palm in the exact in-game attach pose.

### New distinction diagnostics

The telemetry now distinguishes these frames explicitly:

- `nativeFlattenedHand`: the root-flattened hand bone transform passed into the
  grab frame.
- `generatedPalm`: the bone-collider palm anchor target derived from the
  flattened bone/collider pipeline.
- `grabAuthority`: the generated-palm frame after ROCK's grab-authority adapter.
- `proxyReadback`: the actual hknp hidden proxy/body-A transform read back from
  the world.
- `legacyConfiguredPivotA`: the old INI-configured handspace pivot transformed
  through `nativeFlattenedHand`.
- `runtimePivotA`: the current hand-side pivot reported by the runtime grab
  pivot snapshot.

New log rows:

```text
GRAB BASIS FRAMECHAIN ...
GRAB BASIS LEGACY_PIVOT ...
```

New overlay lines:

```text
NATIVE flat->palm ...
AUTH palm->auth ... auth->proxy ...
LEGACY cfg->runtime ... cfg->proxy ...
```

### How to interpret the next test

If `legacyToRuntime ~= 0`, then the active runtime pivot is still the old INI
pivot.

If `legacyToPalm` is non-trivial while `legacyToRuntime ~= 0`, the grab is
still seating around a legacy point that is different from the bone-derived
palm. That can muddy attach-time rotation/translation results.

If `nativeToPalm`, `palmToAuthority`, and `authorityToProxy` are clean but
attach still snaps, the next suspect is not the generated palm convention. The
next suspect becomes object-side grab-frame capture and angular target
construction.

If `authorityToProxy` is large, the hidden proxy/body-A authority is not
actually following the generated palm frame seen by the diagnostic target.

## 2026-05-13 Checkpoint: Full Generated-Collider Convention Rejected

### In-game result

The full generated-collider convention test was rejected by in-game visual
inspection. Hand/body generated colliders had been correct before this test, and
the full-convention build made them rotate in the wrong directions.

The production-safe conclusion is:

- generated hand/body colliders must keep the old native placement convention;
- `matrixFromAxes(...)` stores generated axes as columns;
- this convention belongs to generated physical hull placement, not to generic
  grab-frame math;
- grab/proxy code must adapt the generated palm frame at the explicit
  `generatedColliderFrameToGrabAuthorityFrame(...)` boundary;
- the full-convention identity adapter must not remain.

### Code result

Restored policy:

```text
generated collider matrix:
  column 0 = generated local X in world
  column 1 = generated local Y in world
  column 2 = generated local Z in world

grab authority adapter:
  transpose generated collider rotation before handing it to grab/proxy math
```

This keeps the generated colliders on the convention that already worked in
game while preserving an explicit comparison point for grab-authority telemetry.

### What the fresh logs say about the attach rotation

The same log that rejected the full-convention collider test also points away
from generated-collider orientation as the attach-rotation root cause:

- `rawHandToProxyPalm` is near zero at capture;
- `objectAtGrabToDesiredObject` is zero at capture, so object visual rotation is
  preserved initially;
- `relationPivotErr` stays near zero;
- `transformBErr` is zero;
- `targetErr(colsInv)` is zero.

The suspicious mismatch is object-side BODY/MOTION authority:

```text
frame=1:
  conDesiredBody ~= nativeBody[BODY]
  heldBody[MOTION] is approximately 180 deg away from conDesiredBody/nativeBody
```

That means the custom proxy constraint appears to be targeting a BODY-authored
frame while the hknp solver/readback path being driven is MOTION-oriented for
this object. That mismatch explains the visible first-frame angular correction
better than the generated hand collider convention does.

Next investigation should stay on object-side frame ownership:

- which frame hknp constraints consume for body B;
- whether `constraintBodyWorldAtGrab` is actually BODY or MOTION for the solver;
- whether `desiredConstraintBodyWorld` should be built from the solver body-B
  frame instead of native BODY;
- why `heldBodyToConDesiredBody` and `nativeBodyToHeldBody` can report
  150-180 degree deltas immediately after capture even though visual object
  rotation was preserved.

## 2026-05-13 Checkpoint: Object-Side Solver Frame Fix

### Confirmed failure in source

The attach-rotation logs were backed by a hardcoded source mismatch:

```cpp
constexpr bool constraintUsesMotionBodyAtGrab = false;
const RE::NiTransform constraintBodyWorldAtGrab = grabBodyWorldAtGrab;
```

That forced all proxy-constraint body-B data to be captured in the native BODY
frame even when `tryResolveLiveBodyWorldTransform(...)` reported the live hknp
frame as `MotionCenterOfMass`. The bad runtime signature was:

```text
conDesiredBody ~= nativeBody[BODY]
heldBody[MOTION] ~= 150-180 deg away from conDesiredBody
```

So transform-B and angular target storage could be internally consistent while
still being authored for the wrong body-B frame.

### Fix policy

This does not make COM a grip fallback or pivot authority.

The selected object-side grip point remains:

```text
mesh/contact/authored grip world point
```

The hand-side anchor remains:

```text
root-flattened palm/proxy frame
```

The change is only the coordinate system used to store custom constraint
body-B data:

```text
if live body-B source == MOTION:
    constraint body-B frame = motionBodyWorldAtGrab
else:
    constraint body-B frame = grabBodyWorldAtGrab
```

ROCK now keeps both local copies:

- `_grabFrame.pivotBBodyLocalGame`: selected grip point in BODY space for
  native/visual/release reconstruction;
- `_grabFrame.pivotBConstraintLocalGame`: the same selected grip point in the
  solver body-B frame for transform-B;
- `_grabFrame.bodyLocal`: visible object to native BODY relation;
- `_grabFrame.constraintBodyLocal`: visible object to solver body-B relation;
- `_grabFrame.constraintBodyHandSpace`: desired solver body-B frame relative to
  the proxy/palm authority frame.

Runtime proxy error and pivot reads now use `tryGetGrabDriveObjectWorldTransform`.
For proxy constraint drive that prefers live `MotionCenterOfMass` readback when
available, then falls back to BODY if no motion frame exists. Native mouse-spring
and native visual/body reconstruction keep using the BODY helper.

### What the next in-game test should decide

Expected healthy attach signature:

```text
constraintObjectFrame=MOTION
desiredConstraintBody ~= heldBody[MOTION]
heldBodyToConDesiredBody near zero at frame 1
nativeBodyToHeldBody may remain nonzero as a diagnostic BODY/MOTION delta
objectAtGrabToDesiredObject remains zero for generic point-to-palm grabs
relationPivotErr remains near zero
```

If the object still snaps while `desiredConstraintBody` matches
`heldBody[MOTION]`, then the next suspect is not body-B frame selection. The
remaining candidates would be constraint angular target interpretation,
body-A/proxy after-solve readback, or visual hand publication timing.
