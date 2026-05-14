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
