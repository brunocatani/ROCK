# Custom Dynamic Grab Rotation Failure Map - 2026-05-13

Purpose: preserve the corrected conclusion so future work does not repeat the
same mistake.

This is a mapping note only. It is not an implementation plan and it is not a
rollback note.

## Corrected Conclusion

The world-direction-dependent rotation bug was born with the first integration
of the new custom/proxy dynamic grab architecture.

It was not introduced by one later patch and it was not fixed by one later
patch. Later commits changed how the symptom appeared, sometimes making it
better or worse, but the underlying issue already existed from the beginning of
the new grab integration.

Do not keep searching old commits to find the single bad line. That will waste
time. The fix needs to be a novel correction to the custom grab architecture's
rotation-space contract.

## User-Visible Symptom

After grabbing, held-object angular response is dependent on world/player
direction. This is the N/S/E/W class of bug:

- the same wrist/controller motion maps differently depending on player facing;
- object pitch/yaw/roll can appear swapped or inverted;
- rotating the player can shift the relationship;
- translation can feel mostly correct while rotation is wrong;
- the bug is most visible after the object is already held, not only at grab
  start.

## Core Design Failure

The old native mouse-spring path gave FO4VR one coherent body target:

```cpp
desiredBodyWorld -> native mouse spring
```

The custom integration split that authority into several pieces:

```cpp
generated palm/proxy frame -> hidden keyframed proxy body
hidden proxy body -> custom linear constraint body A
held object BODY -> custom constraint body B
desired body transform -> constraint target
desired body transform -> direct angular velocity or ragdoll angular atom
```

That split is not automatically wrong, but it requires a proven rotation-space
contract. The integration never proved that contract end to end.

The missing contract is:

> A physical wrist rotation in the root-flattened palm/controller frame must
> become exactly the same local held-object rotation regardless of world/player
> yaw, and every layer must agree which frame owns the axis.

The current system does not guarantee that.

## Why Commit Archaeology Is The Wrong Path Now

The issue existed from the start of the new custom grab path. Later commits
changed:

- proxy frame source;
- generated collider frame conversion;
- BODY vs MOTION experiments;
- ragdoll angular atom usage;
- direct `SetBodyAngularVelocity` usage;
- transform-B / target_bRca writes;
- diagnostic logging.

Those changes can alter symptoms, but none of them are the original category of
the bug. The category is architectural: multiple rotation authorities and
transform conventions were integrated without a single verified owner for
angular space.

## Current High-Value Suspect Areas

These are still relevant, but they should be treated as parts of one system,
not isolated patches.

### Proxy Frame Contract

Files:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/hand/HandColliderTypes.h`

Functions:

- `resolveGrabAuthorityProxyFrame(...)`
- `generatedColliderFrameToGrabAuthorityFrame(...)`
- `_boneColliders.tryGetPalmAnchorTarget(...)`

Question:

Does the proxy frame used at capture and every held update represent the same
local wrist axes as the physical controller/root-flattened palm frame?

If not, every downstream target can become world-direction dependent even when
positions look correct.

### Captured Grab Relation

Files:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabCore.h`

Fields:

- `_grabFrame.rawHandSpace`
- `_grabFrame.constraintHandSpace`
- `_grabFrame.constraintBodyHandSpace`
- `_grabFrame.bodyLocal`
- `_grabFrame.constraintBodyLocal`
- `_grabFrame.pivotBBodyLocalGame`
- `_grabFrame.pivotBConstraintLocalGame`

Question:

Are these relations all expressed in one coherent frame family, or are raw
hand, generated palm/proxy, BODY, and constraint body-B conventions being mixed?

### Angular Authority

Files:

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabConstraint.cpp`
- `src/physics-interaction/grab/GrabConstraintMath.h`
- `src/physics-interaction/grab/GrabAuthorityProxyMotion.h`

Functions:

- `updateProxyConstraintGrabDriveTarget(...)`
- `applyProxyConstraintAngularVelocityDrive(...)`
- `angularVelocityFromRotationDelta(...)`
- `grab_authority_proxy_motion::computeAngularVelocityRadiansPerSecond(...)`
- `grab_constraint_math::writeInitialGrabAngularFrame(...)`

Question:

Is angular error computed in the same frame that the angular drive consumes?

If the error axis is extracted in world matrix columns and written as world
angular velocity, while the desired relation was authored in palm/proxy local
space, the held object can become compass-direction dependent.

### Constraint Angular Atom

The old custom path used the ragdoll angular atom. Later work disabled it and
used direct object angular velocity. Since the N/S/E/W bug existed before that
change, both approaches need to be judged against the same frame contract.

The question is not "ragdoll atom or direct angular velocity?"

The correct question is:

> Which FO4VR hknp angular authority can consume a hand-local desired rotation
> without leaking world/player yaw into the held-object local axes?

## What Is Not The Main Cause

### COM

COM/MOTION is not the grip authority problem here. COM must remain mass,
inertia, lever, and throw data only.

Do not "fix" this by moving pivots to COM.

### Force Tuning

Changing max force, tau, damping, or angular speed caps will not fix an axis
mapping bug. It can hide or slow the symptom, but the object will still rotate
around the wrong interpreted axis.

### Old INI Pivot

The old configured pivot can muddy diagnostics and should be audited later, but
a static hand-space pivot offset does not explain player-facing-dependent
rotation by itself.

## Future Fix Direction

The fix must establish a single angular-space contract before changing code.

The future implementation should answer these in order:

1. What exact frame is the physical hand/palm authority frame?
2. What exact frame is captured at grab start?
3. What exact frame is recomposed every held update?
4. What exact frame does the chosen angular drive consume?
5. How is a local wrist roll/pitch/yaw converted into that drive's expected
   angular error or angular velocity?
6. How do we prove that rotating the player yaw does not change that mapping?

Only after those answers are explicit should code change.

## Testing Requirement For The Future

The next validation must not rely only on "does it feel better". It needs a
specific axis isolation test:

- hold object facing north;
- wrist pitch, yaw, roll separately;
- rotate player 90 degrees;
- repeat the same wrist pitch, yaw, roll;
- the object-local response must be the same in both player facings.

If a proposed fix cannot explain why this test passes, it is not a real fix.

## Do Not Repeat

Do not:

- hunt for one old commit as the origin;
- tune forces to hide axis errors;
- flip axes blindly;
- transpose whole frames without proving each consumer;
- mix collider visual correctness with grab authority correctness;
- use COM as pivot authority;
- assume native mouse spring conventions are the target;
- assume HIGGS hkp angular atoms map 1:1 to FO4VR hknp;
- patch a single line and call the architecture fixed.

The next work has to be a coherent angular authority design for FO4VR custom
dynamic grab.

## 2026-05-13 Late Research Addendum - Angular Axis Contract

This addendum records the concrete evidence found after the initial note.

### Source fact: ROCK local axes are stored as rows

`src/physics-interaction/TransformMath.h` defines
`rotateLocalVectorToWorld(matrix, vector)` as:

```cpp
world.x = m00 * x + m10 * y + m20 * z;
world.y = m01 * x + m11 * y + m21 * z;
world.z = m02 * x + m12 * y + m22 * z;
```

Therefore, for a unit local axis:

- local X in world = `(m00, m01, m02)` = stored row 0;
- local Y in world = `(m10, m11, m12)` = stored row 1;
- local Z in world = `(m20, m21, m22)` = stored row 2.

For grab-frame math, the physical local axes are rows, not columns.

### Source fact: current custom grab angular velocity uses columns

Two current custom grab angular paths extract axes from columns:

- `HandGrab.cpp::angularVelocityFromRotationDelta(...)`
- `GrabAuthorityProxyMotion.h::computeAngularVelocityRadiansPerSecond(...)`

Both sum:

```cpp
cross(column(previous, i), column(current, i))
```

That is inconsistent with the local-axis convention used by
`TransformMath::rotateLocalVectorToWorld`.

### Why this matches the N/S/E/W symptom

A small convention test with ROCK's stored-row transform model gives this
result for the same local wrist pitch while changing player yaw:

```text
yaw 0   row-axis [ 1,  0, 0]   column-axis [-1, 0, 0]
yaw 90  row-axis [ 0,  1, 0]   column-axis [-1, 0, 0]
yaw 180 row-axis [-1,  0, 0]   column-axis [-1, 0, 0]
yaw 270 row-axis [ 0, -1, 0]   column-axis [-1, 0, 0]
```

The row-derived axis rotates with the player/hand as expected. The
column-derived axis stays pinned to a world direction. That is exactly the
N/S/E/W failure class: a local wrist motion becomes a different held-object
rotation depending on world/player facing.

This does not prove the final implementation should simply change every column
reader to rows. It proves the current angular contract is not coherent: at
least one angular producer is extracting the command from the wrong storage
shape for ROCK's grab transform math.

### Ghidra fact: hknp angular velocity setter consumes a public world vector

Ghidra on `Fallout4VR.exe.unpacked.exe`:

- `0x141539D30` - hknp world angular velocity setter
- `0x1415399F0` - hknp world angular velocity getter
- `0x141539F30` - hknp world linear+angular velocity setter
- `0x14153A6A0` - hard-keyframe velocity computer
- `0x14153ABD0` - hard-keyframe wrapper
- `0x141E4AA30` - native mouse-spring action update

The setter at `0x141539D30`:

1. finds the hknp body slot by body id;
2. follows body `+0x68` to the motion index;
3. reads the motion record by `motionIndex * 0x80`;
4. reads motion orientation from offsets `+0x10..+0x1c`;
5. compares the supplied vector against the current public angular velocity;
6. rotates the supplied vector through the motion orientation;
7. stores the result into motion angular velocity slots `+0x50..+0x5c`.

The getter at `0x1415399F0` performs the reverse-style read path: it reads
motion angular velocity from `+0x50..+0x5c` and transforms it through the motion
orientation to return public angular velocity.

Conclusion: `SetBodyAngularVelocity` is not a raw local-slot write. The vector
ROCK passes must already be the correct public world-space angular vector. If
ROCK extracts that vector from column axes while the desired transform relation
uses row-local axes, hknp will then transform the wrong world vector through the
motion orientation. That gives a concrete mechanism for player-facing-dependent
rotation.

### Mouse spring comparison

Current and backup `NativeMouseSpringGrab.cpp` use an isolated conversion:

```cpp
makeMouseSpringTargetRotation(targetBodyWorldRotation)
    -> transposeRotation(targetBodyWorldRotation)
    -> niRowsToHavokColumns(...)
    -> write target transform columns
```

That old one-hand path did not ask ROCK to compute and pass a public angular
velocity vector every frame. It sent one target body transform to FO4VR's native
mouse-spring action. Ghidra shows the native action calls the hard-keyframe
velocity computer and hknp velocity setters internally, so its angular
correction is computed inside the same native body/motion convention family
that consumes it.

This explains why the old mouse-spring era could be smoother and less
direction-dependent even when it was not HIGGS-quality: it avoided exposing
ROCK's row/column grab-frame mismatch at the `SetBodyAngularVelocity` boundary.

### Proxy path comparison

The custom proxy path now has two angular producers:

1. proxy body angular velocity:
   `GrabAuthorityProxyMotion::computeAngularVelocityRadiansPerSecond(...)`
   computes a column-derived angular vector and passes it to generated body
   velocity;
2. held object angular velocity:
   `HandGrab.cpp::applyProxyConstraintAngularVelocityDrive(...)` computes a
   column-derived angular vector and passes it to `SetBodyAngularVelocity`.

The proxy pose itself may read back close to the requested pose, and the linear
pivot may remain stable, while the angular authority is still wrong. That is
because translation and rotation packing can be independently correct or wrong:
the bug is the angular vector's frame, not necessarily the point pivot.

### Current high-confidence failure model

The custom dynamic grab integration is mixing:

- row-local ROCK grab transforms;
- column-stored generated collider frames;
- a transposed grab-authority adapter;
- hknp target transform packing;
- direct public hknp angular velocity writes;
- hknp's internal motion-orientation conversion.

The current logs showing stable `transformBLocal` and low relation pivot error
do not clear the angular path. They only show the linear/pivot side is
internally consistent. The N/S/E/W symptom requires proving the angular command
axis, and the current source still has column-based angular extraction in both
proxy and object velocity paths.

### Research conclusion

The next fix should be designed around an explicit angular-space contract, not
around another blind transpose:

1. Capture and recompose the desired held relation in ROCK's row-local
   grab-frame convention.
2. Convert live and desired rotations into the exact public hknp world angular
   convention before calling `SetBodyAngularVelocity`, or stop using direct
   angular velocity and express the angular target entirely in a verified
   solver-local constraint frame.
3. Proxy-body angular velocity and held-object angular velocity must use the
   same conversion rule.
4. Diagnostics must project the commanded angular vector onto:
   - physical/root-flattened palm local axes;
   - proxy local axes;
   - desired held body local axes;
   - public hknp world axes.
5. The same wrist pitch/yaw/roll must produce the same local-object response
   after rotating player yaw by 90 degrees.

The likely missing piece is not force tuning and not COM. It is the missing
conversion from ROCK's row-local desired rotation delta into the public hknp
world angular velocity consumed by `SetBodyAngularVelocity`.

### Ghidra fact: generated-body `setVelocity` reaches the same hknp boundary

The hidden proxy path does not escape this convention requirement.

Ghidra:

- `0x141E082A0` - generated collision object `setVelocity(...)`
- `0x141DF56F0` - BSWorld velocity wrapper
- `0x141539F30` - hknp world `setBodyVelocity(...)`

The generated body `setVelocity(...)` path resolves the collision object's hknp
body id, then calls the BSWorld velocity wrapper. The wrapper either calls
`hknpWorld::setBodyVelocity(...)` directly or queues the same command depending
on physics-thread state.

`hknpWorld::setBodyVelocity(...)` writes linear velocity directly, but angular
velocity goes through the same motion-orientation conversion as
`SetBodyAngularVelocity`.

Conclusion: the proxy-body angular velocity produced by
`GrabAuthorityProxyMotion::computeAngularVelocityRadiansPerSecond(...)` has the
same public-world-vector requirement as the held object direct angular velocity.
The proxy path and object path both need the same angular conversion contract.

### Ghidra fact: native hard keyframe is an angular-vector oracle

Ghidra:

- `0x14153A6A0` computes linear and angular velocities from a target position,
  target quaternion, and dt;
- `0x14153ABD0` is a small wrapper that calls `0x14153A6A0` and then calls
  `0x141539F30`.

That proves the output angular vector from `0x14153A6A0` is in the exact public
convention consumed by `hknpWorld::setBodyVelocity(...)`.

Future design implication:

- ROCK does not have to guess the public hknp angular convention from raw
  matrix columns;
- it can use `ComputeHardKeyFrame` as a verification oracle, or potentially as
  the angular velocity producer, while keeping ROCK's custom finite-force,
  contact-pivot, and mass cap policy outside that native helper;
- if ROCK keeps a custom angular-delta function, its output must match
  `ComputeHardKeyFrame` for the same live body, target quaternion, and dt after
  mass/force caps are removed from the comparison.

This does not make native Fallout grab the model. It uses a FO4VR-native
low-level angular conversion function as the boundary oracle for the custom
grab authority.
