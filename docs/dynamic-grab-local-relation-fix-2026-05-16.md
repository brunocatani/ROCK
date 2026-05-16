# Dynamic Grab Local Relation Fix - 2026-05-16

## Why this pass exists

The dynamic grab now gets position and rotation authority from the
root-flattened hand bone. That fixes the authority source, but the held object
can still behave world-relative if the solver receives PivotB and angular target
from different captured relations.

The current symptom is:

- controller/hand transform source is correct;
- proxy/hand-bone authority can be correct;
- PivotB/yellow debug point and angular response still look world/compass
  relative;
- object rotation can feel like it is correcting toward a moving inertial point
  instead of preserving a hand-to-object local relationship.

The fix must not be an axis flip. It must make PivotA, PivotB, and angular target
come from one captured local hand/object relation.

## HIGGS reference logic, in plain terms

HIGGS does not let COM or world direction choose the grip relation.

At grab start:

```text
hand bone world transform
    -> palm point = handWorld * palmPosHandspace

grabbed object contact point
    -> ptPos

desired object transform
    -> preserves current object rotation
    -> translates object so ptPos seats into palm point
```

Then HIGGS freezes the relation:

```text
desiredNodeTransformHandSpace =
    inverse(handWorld) * desiredObjectWorld
```

For the constraint:

```text
pivotA = inverse(bodyA hand body) * palmWorld
pivotB = inverse(bodyB object body) * grabbedPointWorld

desiredHavokTransformHandSpace =
    desiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(bodyB)

desiredHandTransformObjSpace =
    inverse(desiredHavokTransformHandSpace)

transformA.translation = pivotA
transformB.translation = pivotB
angular target = desiredHandTransformObjSpace.rotation
```

During held update HIGGS keeps the same relation:

```text
desiredTransformHandSpace =
    savedDesiredNodeTransformHandSpace * GetRigidBodyTLocalTransform(bodyB)

desiredHandTransformHavokObjSpace =
    inverse(desiredTransformHandSpace)

constraint.setTargetRelativeOrientationOfBodies(
    desiredHandTransformHavokObjSpace.rotation
)

transformB.translation =
    desiredHandTransformHavokObjSpace * palmPosHandspace
```

Important interpretation:

- PivotA is the palm point in hand/body-A local space.
- PivotB is the corresponding point in object/body-B local space.
- Angular target is the same hand/object local relation, not a separate world
  correction.
- COM can affect mass/weight/throw/inertia, not grip pivot authority.

## Current ROCK problem map

Current dynamic grab authority source is correct:

```text
computeGrabAuthorityFrameFromHandBasis(handTransform)
    -> returns root-flattened hand bone transform
```

But the active solver path still carries older split-frame fields:

```text
rawHandSpace
constraintHandSpace
rawRotationProxyHandSpace
constraintBodyHandSpace
rawRotationProxyBodyHandSpace
pivotBBodyLocalGame
pivotBConstraintLocalGame
```

Known current code shape:

```text
createProxyConstraintGrabDrive:
    pivotAProxyLocalGame = worldPointToLocal(proxyBodyWorld, grabPivotAWorld)
    desiredBodyTransformBodyASpace = _grabFrame.rawRotationProxyBodyHandSpace
    activePivotBConstraintLocalGame =
        computeDynamicTransformBTranslationGame(
            desiredBodyTransformBodyASpace,
            pivotAProxyLocalGame)

updateProxyConstraintGrabDriveTarget:
    desiredBodyTransformBodyASpace = _grabFrame.rawRotationProxyBodyHandSpace
    outActivePivotBBodyLocalGame =
        computeDynamicTransformBTranslationGame(
            desiredBodyTransformBodyASpace,
            _grabAuthorityPivotAProxyLocalGame)
    writeGrabRagdollAngularTarget(targetBRca, desiredBodyTransformBodyASpace, identity)
    transformB.translation = outActivePivotBBodyLocalGame
```

This means the yellow active PivotB is not necessarily the original grabbed
contact point. It is currently generated from the desired body relation plus
PivotA. If that relation direction is wrong or mixed with the wrong body frame,
the point can look world-relative.

## Risky leftovers to remove or demote

These are the fields/functions most likely to keep the old mixed model alive:

```text
constraintDrivePivotBBodyLocalGame()
computeDynamicTransformBTranslationGame()
rawRotationProxyBodyHandSpace
pivotBConstraintLocalGame
writeGrabRagdollAngularTarget()
writeInitialGrabTransformBRotation()
```

They are not all bad individually. The problem is that they allow linear PivotB,
angular target, debug PivotB, and visual desired body to be built from separate
concepts that only appear compatible.

## Correct ROCK target model

Create one explicit captured relation for dynamic grab:

```text
DynamicGrabRelation
{
    handAuthorityWorldAtGrab
    bodyBWorldAtGrab

    pivotAHandLocal          // palm/hand origin in body-A local
    contactPointBodyBLocal   // selected grip/contact point in body-B local

    desiredObjectHandLocal   // inverse(hand) * desired object
    desiredBodyHandLocal     // inverse(hand) * desired body
    desiredHandBodyBLocal    // inverse(desiredBodyHandLocal)
}
```

Then the solver gets one coherent relation:

```text
transformA.translation = pivotAHandLocal
transformA.rotation    = identity or the explicit body-A local frame

transformB.translation = desiredHandBodyBLocal * pivotAHandLocal
transformB.rotation    = desiredHandBodyBLocal.rotation

angular target         = desiredHandBodyBLocal.rotation
```

If we keep a separate contact-point debug marker, name it separately:

```text
contactPointBodyBLocal       // original selected grip point
activeConstraintPivotBLocal  // solver transformB local point
```

Do not let both pretend to be "PivotB".

## Implementation direction

1. Add explicit fields to `CanonicalGrabFrame` for the coherent relation:
   - `desiredObjectHandAuthorityLocal`;
   - `desiredBodyHandAuthorityLocal`;
   - `desiredHandBodyLocal`;
   - `contactPointConstraintBodyLocalGame`;
   - `activeConstraintPivotBLocalGame`.

2. Capture these once at grab start from:
   - root-flattened hand authority frame;
   - desired object transform after contact point seats into palm;
   - constraint body-B frame selected for the solver.

3. Make constraint creation consume only these relation fields.

4. Make held update refresh only:
   - body-A proxy world transform from the live hand bone;
   - transformA local pivot from the same local pivot;
   - transformB translation from the same saved relation;
   - angular target from the same saved relation.

5. Keep old split fields only as debug compatibility during the transition, or
   remove them if all references can be replaced cleanly.

6. Update telemetry labels:
   - original contact point;
   - active solver PivotB;
   - hand-bone authority;
   - object desired relation.

## First implementation slice

The first production fix is narrower than the full field rename above:

- keep using the current saved hand/body angular relation for the ragdoll motor;
- stop recomputing active solver PivotB from that angular relation;
- make active solver PivotB equal the frozen selected contact point in the
  solver-facing body-B frame: `pivotBConstraintLocalGame`;
- keep the dynamic recompute helper available only as diagnostic/reference code
  until the relation equivalence is proven safe in FO4VR hknp.

Reason:

HIGGS creates the constraint with transformB at the grabbed contact point in
body-B local space. Its held-update recompute should resolve back to the same
local point because the saved relation is coherent. In ROCK/FO4VR, BODY/MOTION
and earlier split-frame work mean the recompute can become a second active
target. Freezing active solver PivotB to the actual contact point removes that
extra target while preserving the saved angular relation.

## What this should fix

- PivotB/yellow marker should stop acting like a world-direction target.
- Object rotation should follow the saved hand/object relation instead of
  correcting through a mixed BODY/MOTION/world relation.
- The same local relation should drive both linear seating and angular motor
  target.

## What this does not fix by itself

- Weapon multipart activation/deactivation.
- Long-object mass/inertia realism.
- Motor force tuning.
- Visual hand pose/offset polish.

Those depend on the relation being correct first.
