# Dynamic Grab Orientation Basis Map - 2026-05-12

## Question Being Answered

This note answers the grab-basis clarification:

- What is the normal/orientation basis of the grab?
- Where is the palm normal?
- How does the system know what is parallel with the palm?
- Is the grab just a random point in space with guessed orientation?
- What is the HIGGS step-by-step logic from pivot-B capture to pivot-A seating to pull/converge/held state and hand posing?

Scope is dynamic grab only. Keyframed HIGGS grab is a boundary/exclusion here.

## Short Answer

HIGGS dynamic grab is not a random point plus guessed rotation.

HIGGS has two separate concepts:

1. Palm/contact basis:
   - `palmPos` is a hand-space palm point transformed by the live hand transform.
   - `palmDirection` / `palmVector` is a configured hand-space palm-facing direction transformed by the live hand rotation.
   - This direction is used for selection, line/shape casts, geometry closest-point queries, pulled-grab adjustment, and finger probe setup.

2. Held-object orientation basis:
   - Generic dynamic grab does not rotate the object to match the palm normal or triangle normal.
   - HIGGS preserves the object's current rotation.
   - HIGGS translates the object so the selected object/contact point sits at the palm point.
   - The final hand-object relationship is captured as `desiredNodeTransformHandSpace`.
   - The physics constraint angular target is created from the captured body/object-to-hand relation.

Therefore, "parallel with the palm" is known from the live hand transform plus configured palm vector/finger basis, but generic object orientation is not guessed from that normal. The object seats into the palm by translation, and rotation is preserved unless an explicit authored grab node / special equipped-weapon path says otherwise.

## HIGGS Confirmed Source Evidence

| Source | Lines | Confirmed role |
| --- | ---: | --- |
| `higgs/src/hand.cpp` | 2274-2279 | `GetPalmVectorWS` transforms configured `palmVector` from hand space to world space and mirrors X for the left hand. |
| `higgs/include/hand.h` | 245-246 | `GetPalmPositionWS` transforms `palmPosHandspace` by the current hand transform. |
| `higgs/include/config.h` | 331-333 | Default palm vector and palm position: `palmVector={-0.018,-0.965,0.261}`, `palmPosition={0,-2.4,6}`. |
| `higgs/src/hand.cpp` | 2312-2313 | Update computes `palmVector` and pointing vector from the live hand rotation each frame. |
| `higgs/src/hand.cpp` | 320-421 | Close selection casts a sphere from palm position along `palmVector`, then chooses the hit closest to the cast line. |
| `higgs/src/hand.cpp` | 425-518 | Far selection ray/linear-casts along pointing direction and filters by HMD cone. |
| `higgs/src/hand.cpp` | 1160-1217 | Geometry body selection finds the closest graphics point to the palm line. |
| `higgs/src/hand.cpp` | 1432-1448 | `TransitionHeld` recomputes closest geometry point and computes `palmToPoint = ptPos - palmPos`. |
| `higgs/src/hand.cpp` | 1454-1532 | Finger pose is solved from per-finger hand-space normals/open vectors transformed by hand rotation, then tested against nearby triangles. |
| `higgs/src/hand.cpp` | 1543-1547 | Desired object transform preserves current object rotation and translates by `palmPos - ptPos`; relation is captured in hand space. |
| `higgs/src/hand.cpp` | 1592-1605 | Pivot A is palm point in hand-body local space; pivot B is selected point in object-body local space; transform B is derived from captured desired body-to-hand relation. |
| `higgs/src/RE/havok.cpp` | 126-146 | `CreateGrabConstraint` sets body-space transforms and target relative orientation from transform B rotation. |
| `higgs/src/constraint.cpp` | 180-183 | `setTargetRelativeOrientationOfBodies` stores the relative orientation target for ragdoll motors. |
| `higgs/src/hand.cpp` | 3883-3926 | Held-body update makes the visual hand follow the actual object via captured relation, refreshes angular target, and refreshes pivot B. |
| `higgs/src/finger_animator.cpp` | 55-160 | Finger values are animated into local finger transforms over time, not snapped instantly. |

## HIGGS Dynamic Grab Step-by-Step

### 1. Live hand basis is established

Every update, HIGGS reads the live first-person hand node transform and stores it as `m_handTransform`.

From that live hand transform:

- `palmPos = GetPalmPositionWS(handNode->m_worldTransform)` uses `palmPosHandspace`;
- `palmVector = GetPalmVectorWS(handNode->m_worldTransform.rot)` uses configured `palmVector`;
- `pointingVector = GetPointingVectorWS(handNode->m_worldTransform.rot)` uses configured pointing vector.

Important distinction:

- `palmVector` is the palm-facing normal/direction for selection and geometry queries.
- It is not the generic held-object rotation authority.

### 2. Candidate selection uses palm/pointing basis

Close selection:

- starts at the Havok palm position;
- casts a sphere along the palm vector;
- uses contact hit positions;
- chooses the candidate whose hit point is closest to the cast line.

Far selection:

- casts/raycasts along pointing direction;
- clips by ray obstruction;
- linear-casts a sphere to the hit/target point;
- filters by HMD cone.

The selected point can initially be a collision hit point. It is not final pivot authority until `TransitionHeld` can remap it onto graphics geometry.

### 3. Optional pulled object path keeps the same basis

In `SelectionLocked`, HIGGS stores `pulledPointOffset = selectedObject.point - hkObjPos`.

When pull starts:

- HIGGS computes a target near the palm;
- applies predicted velocity to the connected body set for a short window;
- keeps collision ignore/listener state around the pulled object;
- if the object gets close enough, `FindCloseObject` can reacquire it and `TransitionHeld` is called with the close hit point.

Pull does not change the final grip convention. It is a convergence/acquisition phase before the same dynamic held logic.

### 4. HIGGS chooses the actual grabbed body from graphics/contact geometry

Before committing close grab, HIGGS can call `GetRigidBodyToGrabBasedOnGeometry`.

That function:

- gathers skinned and static graphics triangles;
- applies the pending adjusted object transform if needed;
- finds the closest point on graphics geometry to the palm line (`palmPos`, `palmDirection`);
- if the triangle belongs to a static node, resolves the closest parent with movable collision;
- if the triangle belongs to a skinned partition, weights candidate bones by vertex weight and distance to the selected point, then resolves the nearest movable collision parent.

This is why HIGGS can grab the correct physical body on multipart/skinned items. The point/body choice is geometry-driven, not COM-driven.

### 5. HIGGS recomputes pivot B from graphics geometry at commit

Inside `TransitionHeld`:

- `closestPoint` enters as the current selected/collision/pull point;
- `ptPos = closestPoint / havokWorldScale`;
- triangles are gathered/refreshed unless reused;
- HIGGS calls `GetClosestPointOnGraphicsGeometryToLine(triangles, palmPos, palmDirection, triPos, triNormal, ...)`;
- if successful, `ptPos = triPos`.

`triNormal` is found, but generic dynamic object rotation is not set from `triNormal`.

The selected object-side pivot B is effectively `ptPos`: the graphics/contact point on the object that should seat into the palm.

### 6. HIGGS solves finger pose from hand-space finger math and local geometry

If graphics point solving succeeded:

- HIGGS filters nearby triangles around `triPos`;
- computes `palmToPoint = ptPos - palmPos`;
- for each finger, loads configured/generated finger data:
  - finger normal in hand space;
  - zero-angle/open vector in hand space;
  - finger start position in hand space;
- mirrors left-hand X and normal axes;
- transforms those vectors/points by the live hand transform;
- shifts each finger start by `palmToPoint`, meaning "pretend the hand has moved so the palm is already seated on the object";
- runs triangle intersection against the finger curl curve plane;
- chooses the curve value where the finger would touch the mesh;
- if no intersection, closes the finger;
- clamps to a minimum open value to avoid overcurl;
- tries an alternate thumb curve if the primary thumb curve misses.

This answers the "parallel with palm" part for hand pose:

- the palm/finger basis comes from hand-space curves transformed by the live hand rotation;
- nearby object triangles supply collision/intersection evidence;
- the object does not need special `HIGGS_R/L` grab nodes for this math path.

### 7. HIGGS preserves object rotation and translates the selected point to palm

This is the core generic orientation rule:

```text
desiredNodeTransform = adjustedTransform
desiredNodeTransform.pos += palmPos - ptPos
desiredNodeTransformHandSpace = inverseHand * desiredNodeTransform
```

Meaning:

- start with the object's current or adjusted transform;
- keep its rotation;
- translate it so the selected point `ptPos` moves to `palmPos`;
- capture the result in hand space.

No COM. No generic surface-normal alignment. No guessed object orientation.

The "correct orientation" is the orientation the object already had at grab time, expressed as a stable hand-relative transform after seating the selected point into the palm.

### 8. HIGGS creates pivot A and pivot B in physics-local spaces

For dynamic held state:

- `hkPivotA = palmPos`;
- `hkPivotB = ptPos`;
- `pivotA = inverse(bodyA transform) * hkPivotA`;
- `pivotB = inverse(bodyB transform) * hkPivotB`.

So:

- pivot A is the palm point in the hand body local frame;
- pivot B is the selected object point in the grabbed body local frame.

Then HIGGS builds:

```text
handTransformHandSpace.pos = pivotA
desiredHavokTransformHandSpace = desiredNodeTransformHandSpace * rigidBodyTLocalTransform
handTransformObjSpace = inverse(desiredHavokTransformHandSpace)
handTransformObjSpace.pos = pivotB
CreateGrabConstraint(bodyA, bodyB, handTransformHandSpace, handTransformObjSpace)
```

This captures both:

- where the point-to-point linear motor should connect;
- what relative angular orientation the body should preserve.

### 9. HIGGS initializes custom motor constraint authority

`CreateGrabConstraint`:

- creates `GrabConstraintData`;
- calls `setInBodySpace(transformA, transformB)`;
- sets target relative orientation from transform B rotation;
- enables linear and angular motors.

The dynamic grab constraint uses:

- transform A = palm/hand-side constraint frame;
- transform B = object/body-side constraint frame;
- linear motors to bring pivot B to pivot A;
- angular motors to preserve the captured body-to-hand relation.

This is why the object should not rotate at grab start: the angular target should equal the captured relation.

### 10. Held-body update moves visual hand from the actual object, then refreshes the constraint

During held dynamic state:

- HIGGS reads the actual held object/node transform;
- computes `m_adjustedHandTransform = heldTransform * inverse(desiredNodeTransformHandSpace)`;
- lerps that visual hand transform at the beginning of grab;
- computes hand deviation from real hand to adjusted visual hand;
- drops/releases if average deviation exceeds max distance after the grace period;
- otherwise calls `UpdateHandTransform(m_adjustedHandTransform)`.

Then HIGGS refreshes the constraint:

```text
desiredTransformHandSpace = desiredNodeTransformHandSpace * rigidBodyTLocalTransform
desiredHandTransformHavokObjSpace = inverse(desiredTransformHandSpace)
setTargetRelativeOrientationOfBodies(desiredHandTransformHavokObjSpace.rot)
newPivotB = desiredHandTransformHavokObjSpace * palmPosHandspace
transformB.translation = newPivotB
```

This is the loop that keeps the palm/object relation coherent while the real physics body can lag under finite force.

## ROCK Current Basis Map

ROCK already has a concrete palm basis. It is not random.

### Configured/raw hand basis

Current ROCK config defaults:

- `rockPalmNormalHandspace = (0,0,1)`;
- `rockReversePalmNormal = true`;
- effective palm normal is therefore hand-space `(0,0,-1)` after transform/reversal;
- `rockRightGrabPivotAHandspace = (6.0,0.2,-2.0)`;
- `rockLeftGrabPivotAHandspace = (6.0,-0.2,-2.0)`.

`HandFrame.h` maps those hand-space values to world:

- `transformHandspaceLocalToWorld` uses the verified FO4VR `NiTransform` convention;
- `computePalmPositionFromHandBasis` maps pivot A from hand space to world;
- `computePalmNormalFromHandBasis` maps palm normal from hand space to world;
- `computePointingVectorFromHandBasis` maps far selection direction.

### Root-flattened/generated palm basis

Generated palm body/anchor is built from the root-flattened hand bone tree:

- `HandBoneColliderSet.cpp` captures `LArm_Hand` / `RArm_Hand` and finger bases from `GameRootFlattenedBoneTree`;
- back-of-hand direction is local hand `+Z`;
- palm anchor frame uses:
  - X axis = hand to average finger-base direction;
  - Z/back axis = back-of-hand direction;
  - Y axis = cross(back axis, X);
  - palm center = average of hand + finger bases, shifted toward palm face by `-backAxis`;
- `RootFlattenedFingerSkeletonRuntime.cpp` exposes live palm normal as local hand `-Z`.

So ROCK has a stronger live anatomical basis than HIGGS' old configured VRIK hand-space offsets. The important rule is to keep this basis as hand authority, not object authority.

### ROCK contact patch basis

ROCK contact patch logic uses the palm basis to probe a local palm plane:

- `palmNormalWorld`;
- `palmTangentWorld`;
- `palmBitangentWorld`;
- sphere probes are offset along tangent/bitangent;
- casts move along palm normal;
- contact normals are oriented toward the palm;
- patch fit computes normal/tangent/bitangent from accepted hit samples;
- pivot decision chooses mesh snap, nearest patch sample, or selected hit.

This gives ROCK explicit understanding of "parallel with palm" for contact-patch probing and pose evidence.

Important: this patch basis should choose/validate contact and finger pose evidence. It should not generically rotate the held object unless the product design explicitly adds a surface-alignment mode. HIGGS generic dynamic grab does not do that.

### ROCK held-object basis

Current ROCK dynamic grab commit mirrors the HIGGS relation at a higher level:

- `grabPivotAWorld` = palm/pocket center;
- `grabGripPoint` = mesh/contact/authored/patch-selected object-side point;
- `shiftObjectToAlignGripWithPocket(objectWorldTransform, grabPivotAWorld, grabGripPoint)` preserves object rotation and translates the object so the selected point seats at the palm;
- `objectToBodyAtGrab = inverse(objectWorld) * bodyWorld` preserves visual object to hknp BODY relation;
- `desiredBodyWorld = desiredObjectWorld * objectToBodyAtGrab`;
- pivot B is frozen in BODY local space from `grabBodyWorldAtGrab` and `grabGripPoint`;
- custom proxy authority follows the root-flattened palm target.

This is conceptually correct. The remaining risk is not "no basis." The risk is whether every solver-facing transform/matrix convention matches FO4VR hknp's raw constraint memory layout.

## Direct Answer To The Current Clarification

### Where is the normal?

HIGGS:

- palm normal/direction = configured `palmVector` transformed by hand rotation;
- object surface normal = `triNormal` from closest graphics triangle, but not generic object rotation authority;
- finger curl normals = per-finger hand-space curve normals transformed by hand rotation.

ROCK:

- palm normal = configured hand-space palm normal transformed by FO4VR hand basis, currently reversed to effective local `-Z`;
- root-flattened palm normal = live hand local `-Z`;
- contact patch normal = fitted/oriented object contact normal from samples;
- finger pose normal = live root-flattened palm normal plus per-finger/target surface normals.

### How does it know what is parallel with the palm?

HIGGS:

- palm plane is implicit: plane perpendicular to `palmVector`;
- finger curl planes are explicit per-finger planes from hand-space finger normal/open vectors;
- the object is not aligned to this plane for generic grab.

ROCK:

- palm plane is explicit:
  - normal = `computePalmNormalFromHandBasis`;
  - tangent = hand-space X transformed to world;
  - bitangent = hand-space Y transformed to world;
- contact patch probes and patch tangents use this basis.

### Are we pulling the orientation out of nowhere?

No. The correct generic orientation is captured:

```text
desired object rotation = object rotation at grab
desired object translation = current object translation + (palm point - selected object point)
desired hand/object relation = inverse(hand) * desired object
constraint angular target = inverse(desired body in hand/proxy space).rotation
```

If the object rotates immediately at grab start, that is not because there is no basis. It means the captured basis was written incorrectly to the solver, the wrong frame was used for BODY vs visual object, or two authorities are fighting.

## Implementation Implications

1. Do not rotate generic loose objects/weapons to match palm normal.
   - HIGGS does not do that for generic dynamic grab.
   - Palm normal is contact/query/pose evidence, not generic object orientation authority.

2. Do not use COM as pivot authority.
   - HIGGS pivot B is selected contact/graphics point transformed to object/body local space.
   - COM is mass/inertia data only.

3. Preserve object rotation at capture.
   - `desiredObjectWorld.rotate` should remain object rotation unless an explicit authored grip frame is used.

4. Seat the selected point into the palm by translation.
   - This is the point-to-palm relation.

5. Store the relation in the frame that actually drives the solver.
   - In ROCK/FO4VR that means preserving visual-object to hknp BODY relation and using the root-flattened proxy/palm frame for body A.

6. Finger pose should use palm/finger basis plus object geometry.
   - HIGGS uses per-finger curve planes and nearby mesh triangles.
   - ROCK can exceed this with root-flattened live finger bases, contact patch, per-finger targets, and surface aim normals.

## Remaining Unknowns / Next Checks

Confirmed from source:

- HIGGS generic dynamic grab preserves object rotation.
- HIGGS uses palm normal/palm vector for selection and geometry lookup, not generic rotation alignment.
- HIGGS finger pose is math/geometry driven, not dependent on `HIGGS_R/L` nodes.
- ROCK has explicit palm normal/tangent/bitangent and root-flattened palm/finger basis.

Still needs runtime/binary validation when debugging the active snap:

- whether FO4VR hknp consumes custom transform-B rotation and target-bRca as row or column memory in every relevant solver path;
- whether first-frame transform A/proxy pose is already available to the solver at constraint creation time or only on the next between-collide-and-solve flush;
- whether visual hand should remain raw for one frame until the object has accepted the newly created angular target, or whether current object-relative visual hand update is already safe once the solver target convention is correct.

## Completion Audit

Objective deliverables:

- explain normal/orientation basis: covered in "Short Answer", "HIGGS Confirmed Source Evidence", and "Direct Answer";
- map how HIGGS knows what is parallel with palm: covered in "How does it know what is parallel with the palm?";
- map step-by-step HIGGS grab from pivot B to pivot A to hand pose: covered in "HIGGS Dynamic Grab Step-by-Step";
- compare to ROCK understanding: covered in "ROCK Current Basis Map";
- separate confirmed facts from unknowns: covered in "Remaining Unknowns / Next Checks".
