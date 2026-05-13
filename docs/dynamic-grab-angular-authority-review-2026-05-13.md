# Dynamic Grab Angular Authority Review - 2026-05-13

## Scope

This note supersedes the late-session working memory around the current loose
dynamic grab rotation failure. It is not an implementation plan. It is a
review of the current evidence so the next code change is not based on an
assumption.

In scope:

- ordinary dynamic one-hand loose object grab;
- loose non-equipped weapon grab as the same dynamic object path;
- hidden proxy/body-A authority;
- selected contact/palm pivot;
- object-side BODY constraint frame;
- angular authority and motor choice.

Out of scope:

- equipped weapon visual authority;
- two-hand equipped weapon handling;
- actor ragdoll grab;
- generated hand/body collider convention changes.

## Current in-game result

User-tested result after the collider convention restore:

- generated hand/body colliders look correct again;
- dynamic grab rotation is still wrong;
- rotation now does not match hand/controller even after the grab settles;
- the visible problem is not only the first attach pop anymore.

The fresh runtime log is:

```text
C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log
LastWriteTime: 2026-05-13 17:54:33
```

The deployed DLL tested by that log is:

```text
D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll
LastWriteTime: 2026-05-13 17:50:13
```

Representative current log pattern from session 7:

```text
proxyTargetErr ~= 0.1-0.6 gu / 2-6 deg
objectTargetErr ~= 2-4 gu / 110-160 deg
gripTargetErr ~= 1-2 gu
targetErr(colsInv=0.000deg)
transformBErr=0.000gu
ragEnabled=yes
angForce=20.0
linForce=250.0
mass=0.500
```

Interpretation:

- body-A/proxy is following the queued root-flattened palm authority closely;
- transform-B translation and the selected grip point remain coherent;
- the object is not converging angularly to the desired body frame;
- telemetry proving `colsInv=0` only proves ROCK wrote the matrix it intended,
  not that FO4VR's solver interprets it as the intended target.

## Confirmed non-causes

### Generated collider convention

The full generated-collider convention change was rejected by in-game testing.
The current production-safe rule is:

```text
generated collider axes are stored in the convention that already worked
grab/proxy code adapts that convention at generatedColliderFrameToGrabAuthorityFrame
```

Current grab failure remains after collider restoration, so the live rotation
bug is not solved by changing generated hand/body collider conventions.

### COM as pivot authority

COM is still not acceptable as grip pivot or fallback authority.

The logs and source show the current failure happens while:

- selected grip/pivot B is frozen from contact/palm evidence;
- `pivotBConstraintLocalGame` remains stable;
- `transformBLocal == desiredTransformBLocal`;
- BODY/MOTION/COM is diagnostic/weight data only.

The failure is angular authority, not COM grip fallback.

### HIGGS angular-to-linear force ratio direction

HIGGS source confirms its ordinary object branch uses:

```cpp
angularMotor->m_maxForce = linearMotor->m_maxForce / angularToLinearForceRatio;
```

ROCK's current `grab_motion_controller::angularForceFromRatio(...)` does the
same division. Therefore the current angular-force ratio direction is not, by
itself, the parity bug.

This does not prove the value is good in FO4VR. It only proves the ratio was
not accidentally inverted relative to HIGGS.

## Current ROCK angular implementation

ROCK's custom grab constraint currently uses a HIGGS-shaped hkp atom stream in
FO4VR:

```text
type 0x02: set local transforms
type 0x17: setup stabilization
type 0x13: ragdoll motor
type 0x0B: linear motor x3
```

The angular part is:

- one allocated `HkPositionMotor`;
- the same motor pointer written into the three type-`0x13` ragdoll motor slots;
- `transformB.rotation` written from
  `inverse(desiredBodyTransformProxySpace.rotate)`;
- `target_bRca` written from the same inverse matrix;
- transform A rotation left identity, with proxy BODY/world rotation carrying
  the palm authority frame.

Current source points:

```text
src/physics-interaction/grab/GrabConstraint.cpp
src/physics-interaction/grab/GrabConstraintMath.h
src/physics-interaction/hand/HandGrab.cpp
```

## FO4VR binary evidence so far

### Position motor helper

Ghidra address:

```text
0x141AFD600
```

Blind finding:

- this helper branches on the motor type byte at `+0x10`;
- type `1` is a normal position-motor-style branch;
- it consumes the same fields ROCK writes:
  - min force around `+0x18`;
  - max force around `+0x1C`;
  - tau around `+0x20`;
  - damping around `+0x24`;
  - recovery fields around `+0x28/+0x2C`.

Verdict:

- ROCK's `HkPositionMotor` layout is not the first suspect;
- using a position motor object in the ragdoll motor slots is at least a
  recognizable FO4VR path.

### Type-19 ragdoll motor atom case

Confirmed atom interpreter case:

```text
0x141A55550: live atom-stream interpreter
0x141A5873C: type 0x13 ragdoll motor atom case
```

Focused disassembly confirms the type-`0x13` case reads:

```text
atom + 0x02: enabled byte
atom + 0x04: initialized runtime offset
atom + 0x06: previous target angles runtime offset
atom + 0x10: target matrix
atom + 0x40: angular motor pointer slots
context + 0x48: hknp body B pointer
context + 0x30/+0x38: motion/inertia solver data
```

The important first composition is:

```text
rcx = output at stack + 0x530
rdx = context + 0x48 body B transform pointer
r8  = atom + 0x10 target_bRca
call 0x1417D05F0
```

`0x1417D05F0` multiplies Havok column-block transforms as:

```text
out = bodyBTransform * target_bRca
```

This means FO4VR's angular atom is not a black box that simply consumes
ROCK's diagnostic inverse matrix. It first composes the atom target through
the live body-B transform, then builds angular rows against other frame data
from the type-2 local-transform stage and motion/inertia context.

Verdict:

- the current ROCK telemetry check `colsInv=0` is insufficient;
- the exact FO4VR type-19 solve equation is still the critical missing piece;
- a 1:1 HIGGS matrix writer is not proven correct in FO4VR.

## HIGGS comparison boundary

HIGGS confirms the intended high-level model:

- contact/palm grip point;
- COM is weight data only;
- finite linear and angular motors;
- mass cap and angular-to-linear force ratio;
- collision-aware tau/force changes;
- hand/object deviation instead of infinite-strength hand authority.

But FO4VR is not Skyrim's hkp runtime.

FO4VR has hknp-era paths that HIGGS did not use, including:

- `hknpWorld::SetBodyVelocity`;
- `hknpWorld::SetBodyAngularVelocity`;
- `hknpWorld::ComputeHardKeyFrame`;
- `hknpWorld::ApplyHardKeyFrame`;
- native hknp ragdoll motor controller code;
- different constraint row build and atom interpreter internals.

Therefore HIGGS parity is behavioral, not binary-mechanical. The correct FO4VR
implementation may need to surpass HIGGS by using FO4VR-native angular
authority instead of copying the old hkp ragdoll motor atom.

## Current code review findings

### Finding 1: Angular atom likely wrong or incomplete for this FO4VR grab

Severity: critical.

Evidence:

- proxy/body-A tracks correctly;
- pivot and transform-B translation track correctly;
- object rotation remains 90-180 degrees off;
- current logs show `ragEnabled=yes` but the object does not converge;
- binary evidence shows type-`0x13` composes target with body B internally;
- the exact solve-side equation has not been proven equal to HIGGS.

Conclusion:

The current angular authority should be treated as suspect. Either:

- the target equation is wrong;
- transform A/B rotations are incomplete for FO4VR;
- the type-19 atom is not the best rotation authority for a proxy-driven loose
  object in FO4VR;
- or the custom atom stream lacks native FO4VR ragdoll/controller pieces needed
  for robust angular behavior.

### Finding 2: Old INI pivot A still affects dynamic grab authority

Severity: high, but probably not the 110-degree angular root cause.

Source:

```text
src/physics-interaction/hand/Hand.cpp
Hand::computeGrabPivotAWorld(...)
```

This still returns:

```text
computeGrabPivotAPositionFromHandBasis(fallbackHandWorldTransform, _isLeft)
```

Runtime session 7 logs:

```text
legacyActive=yes
activeSource=iniConfiguredHandspace
legacyToRuntime ~= 0.04-0.25 gu
legacyToPalm ~= 2.149 gu
```

Interpretation:

- current runtime pivot A is still the legacy INI-configured handspace point;
- generated palm/proxy authority is nearby but not the same point;
- this can contaminate attach seating and diagnostics;
- it does not explain why angular error remains around 100+ degrees after
  settling, because that is far larger than the measured pivot offset.

### Finding 3: Equipped weapon path is not comparable

Severity: important boundary.

Equipped/two-hand weapon visual authority does not use this dynamic grab
constraint angular atom path. It solves a direct weapon visual transform from
hand geometry and applies it to the weapon node.

Therefore the current loose grab angular failure must be fixed in the dynamic
loose object path, not by mixing equipped weapon code or generated weapon
collider assumptions into it.

## Decision point

The user question was whether ROCK may be using the wrong motor for rotation.

Current answer:

```text
Yes, that is now a serious and evidence-backed possibility.
```

More precisely:

- the linear motor atom and selected grip pivot are doing useful work;
- the HIGGS-shaped type-19 ragdoll angular motor path is not proven correct in
  FO4VR and is failing in live tests;
- FO4VR has newer native angular velocity/keyframe/controller paths that may be
  better suited to the desired dynamic grab authority;
- 1:1 HIGGS atom parity should no longer be assumed.

## Candidate fix directions to evaluate

### Option A: Finish exact type-19 equation and correct matrix writer

Keep the current atom stream, but only if Ghidra confirms the exact equation.

Required proof:

- how `bodyA * transformA`, `bodyB * transformB`, and `bodyB * target_bRca`
  are compared;
- whether `transformA.rotation` should remain identity or encode a palm local
  frame;
- whether `transformB.rotation` should be frozen, refreshed, forward, inverse,
  or multiplied by another term.

Risk:

- may still be a fragile hkp-era path inside hknp FO4VR;
- previous row/column and BODY/MOTION changes show that partial corrections
  can make gameplay worse.

### Option B: Keep linear constraint, replace angular atom with FO4VR-native finite angular drive

Use the existing hidden proxy and selected contact pivot for linear authority,
but stop relying on the type-19 ragdoll angular atom for object orientation.

Possible FO4VR-native angular authority:

- compute desired body rotation from the proxy frame and captured object
  relation;
- compute world-space angular error between live BODY rotation and desired BODY
  rotation;
- convert that to a target angular velocity;
- cap the angular velocity delta by mass/inertia/force budget;
- write only angular velocity during the between-collide-and-solve phase;
- keep COM as mass/inertia/lever data only, not grip pivot authority;
- disable the ragdoll angular atom so there is only one angular writer.

This is not mouse spring and not keyframed snap. It is a finite angular motor
implemented through FO4VR's hknp velocity API.

Risk:

- must be designed carefully so it does not become infinite-strength visual
  locking;
- must not fight the linear constraint;
- needs clean release velocity behavior.

### Option C: Native hknp ragdoll/controller path

Map and possibly use FO4VR's native hknp ragdoll motor controller or native
ragdoll constraint data instead of ROCK's custom minimal atom stream.

Risk:

- native ragdoll controller is probably built for skeleton constraints, not
  arbitrary loose objects;
- higher mapping cost before implementation;
- may still be overkill for a loose object angular drive.

## Current recommended engineering conclusion

Do not flip another matrix.
Do not change generated collider conventions again.
Do not use COM as pivot fallback.
Do not tune angular force as the main fix.

The next implementation should either:

1. be based on a fully proven FO4VR type-19 angular equation; or
2. replace only the angular authority with a FO4VR-native finite angular drive
   while keeping the working contact pivot, proxy body-A, and linear constraint.

Given the live logs, option 2 is now the stronger FO4VR-native candidate, but
the type-19 equation should still be mapped enough to know exactly what is being
removed or disabled.

## 2026-05-13 implementation checkpoint

Chosen correction for the next build:

```text
linear authority: existing custom constraint linear motors stay enabled
angular authority: type-19 ragdoll angular atom is disabled
replacement angular authority: FO4VR hknp SetBodyAngularVelocity
```

Why this is the chosen boundary:

- live logs showed linear/pivot tracking was close while angular target stayed
  wrong;
- disabling only the angular atom avoids changing selected contact pivot,
  transform-B translation, proxy body-A, generated colliders, or collision
  conventions;
- FO4VR `SetBodyAngularVelocity` and `SetBodyVelocity` were verified in Ghidra
  at `0x141539D30` and `0x141539F30`;
- `ApplyHardKeyFrame` at `0x14153ABD0` is confirmed as
  `ComputeHardKeyFrame -> SetBodyVelocity`, but it is not used here because it
  would be too close to hard keyframe authority for the held object;
- COM remains weight/diagnostic data only.

The new angular drive is finite:

- desired rotation comes from the same desired BODY frame already used by the
  proxy/linear path;
- raw angular velocity is computed from live BODY rotation to desired BODY
  rotation for the current physics delta;
- applied angular velocity is clamped by the existing mass-capped angular motor
  budget;
- the budget scale uses `160` as the reference because that is the existing
  default HIGGS-style angular budget (`2000 / 12.5`);
- the final cap also respects the configured throw angular velocity cap and the
  native motion angular-velocity cap when readable.

Expected fresh-log changes:

```text
PROXY GRAB AUTHORITY diag=bodyFrameConstraint+directAngularVelocity
directAngular=ok
rawAng=...
appliedAng=...
capAng=...
ragEnabled=no
```

Code review correction included before build:

- the shared rotation-delta helper could previously output zero angular
  velocity at an exact 180-degree error because the cross-product axis sum is
  singular at a half turn;
- runtime logs repeatedly showed 180-degree BODY/MOTION or object/target
  angular errors, so the direct angular drive now falls back to the strongest
  `previousColumn + currentColumn` axis witness for the 180-degree case;
- this prevents the new angular drive from doing nothing in exactly the state
  it is meant to escape.

Expected gameplay result:

- object rotation should move toward the same desired BODY frame as the hand;
- it should no longer settle into the type-19 atom's wrong 90-180 degree frame;
- heavy or mass-capped objects should not snap instantly because applied angular
  velocity is budget-scaled.

If this still fails:

- the next suspect is the desired BODY frame itself, not the type-19 motor;
- the logs will show whether `directAngular=ok` is applying capped velocity but
  `objectTargetErr` is not decreasing.
