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

`hand_bone_collider_geometry_math::matrixFromAxes(...)` must store generated
collider axes in the Ni row convention used by:

- `transform_math::rotateLocalVectorToWorld(...)`;
- ROCK grab-frame math;
- FO4VR `NiTransform` composition as used by the existing transform helpers.

Required storage:

```text
row 0 = local X axis in world space
row 1 = local Y axis in world space
row 2 = local Z axis in world space
```

Column storage is forbidden for generated hand collider frames because the same
function builds the palm anchor frame later used as the dynamic-grab body-A
proxy authority. Column storage transposes body A and recreates the attach-time
object/visual-hand rotation.

### Files In This Fix

- `src/physics-interaction/hand/HandColliderTypes.h`
  - owns `matrixFromAxes(...)`;
  - owns generated palm/finger collider frame construction;
  - now documents why root-flattened hand collider axes must use the row
    convention.
- `tests/HandColliderFramePolicyTests.cpp`
  - locks `matrixFromAxes(...)` against `rotateLocalVectorToWorld(...)`;
  - verifies `buildPalmAnchorFrame(...)` local axes read back as the generated
    palm axes;
  - verifies `buildSegmentColliderFrame(...)` local axes read back as the
    generated segment axes.
- `CMakeLists.txt`
  - registers `ROCKHandColliderFramePolicyTests`;
  - includes it in the CommonLib-backed policy test target list.

### What This Fix Does Not Change

- It does not use COM as pivot authority.
- It does not change selected contact pivot capture.
- It does not change the visual-hand formula.
- It does not retune mouse spring, motors, mass, tau, damping, or lead.
- It does not change actor ragdoll grab, equipped weapon grab, or two-hand
  weapon handling.
- It does not remove the diagnostic/proxy scaffolding already present in the
  working tree.

### Runtime Proof Needed After Build

A fresh in-game grab log should show the generated proxy/palm basis matching the
raw/root-flattened hand basis in the same convention:

```text
GRAB BASIS CAPTURE ... convention=niLocalVectorToWorld ...
  rawHandToProxyPalm max ~= 0deg
```

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
