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
