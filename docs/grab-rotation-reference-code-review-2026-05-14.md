# Dynamic Grab Rotation Reference Code Review - 2026-05-14

## Scope

Review only. No implementation change.

User-observed state:

- hand/body/generated colliders rotate in the correct direction;
- the held object still rotates from the wrong reference;
- the object rotation feels object/world-direction relative instead of
  hand/collider relative;
- the issue is visible after dynamic grab and is not a collider convention
  problem.

## Current Build And Log Evidence

Reviewed runtime log:

- `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK.log`
- timestamp: 2026-05-14 20:27:54

Reviewed deployed build:

- `D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll`
- timestamp: 2026-05-14 20:23:37

Representative latest session:

- `session=20`
- hand: left
- form: `FF0031A1`
- held body: `1013`
- proxy body: `661`

## Findings

### 1. Palm/proxy authority is not the failing side

The latest log shows the hidden proxy target tracks the driven palm authority:

```text
PROXY GRAB AFTER_SOLVE ... proxyRead=ok proxySrc=MOTION ...
proxyTargetErr=0.000gu/0.00deg
```

This matches the in-game observation that the visible generated colliders rotate
correctly. The palm/collider/proxy side is not the current rotation bug.

### 2. The object BODY target is internally consistent but not the visual object reference

The same lines show the object BODY target rotation is close after solve:

```text
objectTargetErr=64.88gu/7.5deg
objectLiveProxyErr=64.88gu/7.5deg
```

Position is still far away, but the angular BODY target is not off by the same
100-degree visual error. The wrongness appears when BODY is related back to the
visible node.

### 3. BODY-to-visible-node relation is wrong by about 90-126 degrees

The log repeatedly reports:

```text
nativeToHeld=7.032gu/92.729deg
bodyNodeToVisual=0.824gu/90.513deg
```

Periodic frame-hold diagnostics show the same class of failure:

```text
bodyVsConstraint=0.40deg
ownerErr=119.26deg/0.72gu
heldErr=119.26deg/0.72gu
rootErr=119.26deg/0.72gu
```

Meaning:

- the solver-side BODY target is close to its constraint target;
- deriving the visible node from BODY using `_grabFrame.bodyLocal` is wrong;
- owner/root/held all agree on the visual node side, so this is not a bad
  hit-node choice;
- the captured BODY-to-visual relationship is the bad reference.

### 4. Capture shows the BODY rotation is effectively the transpose of the visual object rotation

At capture:

```text
objectAtGrab.X=(0.245,0.016,0.969)
objectAtGrab.Y=(-0.955,-0.169,0.245)
objectAtGrab.Z=(0.168,-0.985,-0.026)

grabBody.X=(0.243,-0.955,0.168)
grabBody.Y=(0.016,-0.170,-0.985)
grabBody.Z=(0.970,0.242,-0.026)
```

`grabBody` is approximately the stored transpose of `objectAtGrab`. This is the
strongest explanation for the N/S/E/W-style behavior: the captured local relation
contains `object^-1 * object^T`, which changes with world/object orientation
instead of being a stable object-to-hand grip relation.

### 5. Current code hardwires BODY as both pivot frame and rotation authority frame

Code references:

- `HandGrab.cpp:1276`:
  `tryGetGrabAuthorityBodyWorldTransform` returns the native mouse-spring BODY
  readback.
- `HandGrab.cpp:1874`:
  `tryGetGrabDriveObjectWorldTransform` forwards all active drive reads to the
  same BODY helper.
- `HandGrab.cpp:4273`:
  `constraintUsesMotionBodyAtGrab = false`.
- `HandGrab.cpp:4298`:
  `objectToBodyAtGrab = computeRuntimeBodyLocalTransform(objectWorldTransform,
  grabBodyWorldAtGrab)`.
- `HandGrab.cpp:4309`:
  `_grabFrame.bodyLocal = objectToBodyAtGrab`.
- `HandGrab.cpp:4562`:
  `_grabFrame.constraintBodyHandSpace` is built from the BODY-authored desired
  constraint body.
- `HandGrab.cpp:2644-2647`:
  held update composes `desiredBodyWorld` from `_grabFrame.constraintBodyHandSpace`.
- `HandGrab.cpp:2816-2879`:
  direct angular velocity is computed toward `desiredBodyWorld` and written with
  `world->SetBodyAngularVelocity`.

That means BODY is not merely the linear pivot frame. It is also the active
angular target frame.

### 6. The active angular authority is not the ragdoll angular atom

`GrabConstraint.cpp:355-366` disables the ragdoll angular atom:

```cpp
setGrabMotorAtomsActive(header, true, false);
```

So fixes to `target_bRca` or transform-B angular storage cannot be the primary
runtime rotation fix anymore. The actual angular writer is:

```cpp
computeHardKeyframeAngularVelocityForTarget(... desiredBodyWorld ...)
world->SetBodyAngularVelocity(...)
```

This is important because old reviews kept chasing constraint angular matrix
packing. That path is currently disabled for angular authority.

### 7. Native mouse spring had a separate rotation boundary conversion that the custom angular path does not mirror

`NativeMouseSpringGrab.cpp` intentionally transposes the target rotation before
writing native target transform columns:

```cpp
makeMouseSpringTargetRotation(targetBodyWorldRotation)
    -> transposeRotation(targetBodyWorldRotation)
```

The custom angular velocity path calls `ComputeHardKeyFrame` through:

```cpp
transform_math::niRowsToHavokQuaternion(targetWorld.rotate, targetRotationHavok)
```

with no equivalent mouse-spring boundary conversion. The current custom path is
therefore using a BODY-authored target as if it were already in the same
convention consumed by FO4VR's angular velocity computation.

## What Is Confirmed

- The hand/collider/proxy side tracks correctly in the current log.
- The visible object does not match the BODY-derived node relation.
- The mismatch is mostly rotational, roughly 90-126 degrees depending on the
  frame.
- The BODY frame captured from native hknp readback is approximately transposed
  relative to the visual node at capture.
- Angular authority is currently direct `SetBodyAngularVelocity`, not the
  constraint ragdoll angular atom.
- Existing source-boundary tests currently encode the wrong assumption that
  dynamic grab rotation authority must use the native BODY frame as-is.

## What Is Not Confirmed Yet

- Whether the production fix should:
  - convert BODY readback into a conventional visual/object frame before
    capture;
  - keep BODY only for linear pivot and use a separate visual/motion target for
    angular velocity;
  - add a narrow native-boundary conversion equivalent to the old mouse-spring
    target rotation conversion before `ComputeHardKeyFrame`;
  - or replace direct angular velocity with another FO4VR hknp constraint path.

The logs prove where the mismatch is. They do not by themselves prove which
native angular boundary convention `ComputeHardKeyFrame` expects for all body
types.

## Review Conclusion

The current bug is not caused by the generated hand/body colliders. It is caused
by mixing two rotation conventions inside the grabbed-object authority path:

1. visual hand/palm/collider frames use ROCK's conventional root-flattened
   NiTransform frame;
2. held object angular authority uses native hknp BODY readback as if it were
   the same convention;
3. capture freezes a BODY-to-visual relation that already contains a transpose
   or native packing mismatch;
4. per-frame angular velocity then faithfully drives toward that wrong BODY
   relation.

The result is exactly the observed behavior: the hand/colliders rotate correctly,
but the object follows an object/world-direction-dependent angular reference.

## Candidate Fix Direction To Discuss Before Code

Do not change collider conventions again.

The fix should split object-side authority into explicit frames:

- linear contact/pivot frame:
  keep using BODY-local pivot data if runtime tests prove the linear anchor uses
  BODY correctly;
- angular/visual frame:
  stop feeding the direct angular velocity writer a BODY matrix that is
  transposed relative to the visible object.

The first implementation candidate should be a narrow angular-reference fix:

1. capture a conventional visual/object rotation reference separately from the
   BODY-local pivot;
2. compose the per-frame desired visual/object rotation from the live palm/proxy
   frame;
3. convert that visual target through the correct FO4VR native angular boundary
   before calling `ComputeHardKeyFrame`;
4. keep COM out of pivot, target frame, and grip authority;
5. update source-boundary tests that currently force BODY-as-rotation-authority.

## 2026-05-14 Implementation Follow-Up

Implemented the narrow angular-reference split as an A/B runtime switch. Linear
grab authority, BODY-local pivot capture, transform-B pivot storage, generated
palm collider movement, and proxy movement are unchanged.

New config:

- `iGrabObjectRotationReferenceMode = 0`: legacy BODY angular target. This is
  the old behavior and exists only as a baseline.
- `iGrabObjectRotationReferenceMode = 1`: converted BODY capture. The native
  BODY readback is converted into a conventional object-frame candidate during
  capture, cached as `constraintConventionalBodyHandSpace`, and used only for
  angular target selection.
- `iGrabObjectRotationReferenceMode = 2`: split visual/native-boundary angular
  target. BODY remains the linear/pivot target, while angular velocity is
  computed from the desired visual object frame after crossing the same
  transpose boundary used by the old native mouse-spring target rotation path.

Important implementation boundary:

- `updateProxyConstraintGrabDriveTarget` still owns the BODY-space linear target
  and selected grip pivot.
- `resolveProxyConstraintAngularDriveTargetWorld` now selects the angular-only
  target before `applyProxyConstraintAngularVelocityDrive`.
- `applyProxyConstraintAngularVelocityDrive` still calls FO4VR
  `ComputeHardKeyFrame` and applies the finite angular velocity budget. Only the
  target rotation reference changed.
- COM remains mass/diagnostic data only and is not used as pivot, grip frame, or
  angular reference.

Telemetry now reports:

- `rotRefMode` in grab capture logs;
- `angularRef` and `angularTarget` in proxy authority and after-solve logs;
- `desiredConventionalBody` and `desiredObjectToDesiredConventionalBody` in
  basis capture logs.

Validation performed:

- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m`
- `ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%`

Build output was copied by the `custom-fast` preset to
`D:\FO4\mods\ROCK\F4SE\Plugins`.

## 2026-05-14 Mode 1/2 Test Result And Mode 3 Follow-Up

Runtime testing showed that modes 1 and 2 were not enough. The mode switch was
being read correctly, and telemetry showed the angular reference changing, but
the visible behavior stayed wrong. That means the remaining bug was not only
the final BODY/native angular boundary variant.

The useful log pattern was:

- `rawToConRev` stayed large, often around 150 degrees;
- the object followed the `conDesired`/proxy relation rather than the raw
  controller/root-hand relation;
- `proxyTargetErr` was effectively zero while object target/rotation error
  still grew during wrist motion.

Inference from that evidence:

- the generated palm/proxy frame is a good physical anchor for translation and
  pivot seating;
- its axes are not a safe object-rotation owner for dynamic grab;
- the held object angular relation must use the root-flattened raw
  hand/controller rotation, while the linear pivot continues to use the
  palm/proxy translation.

Implemented mode 3:

- `iGrabObjectRotationReferenceMode = 3`;
- capture stores `rawRotationProxyHandSpace` and
  `rawRotationProxyBodyHandSpace`;
- per-frame proxy authority builds a hybrid frame:
  palm/proxy translation plus raw hand/controller rotation;
- mode 3 keeps the existing native angular-boundary conversion used by mode 2;
- generated palm/body collider conventions are unchanged;
- COM remains excluded from pivot, target frame, and grip authority.

Validation after the mode 3 change:

- all 21 CTest tests passed;
- `git diff --check` passed;
- `cmake --build build-fast --config Release --target ROCK -- /m` compiled
  `ROCK.dll`;
- the deploy copy step reported the deployed DLL was locked by the running
  game, but the build output and deployed DLL hashes matched exactly, so the
  deployed runtime is already the fresh mode 3 build.
