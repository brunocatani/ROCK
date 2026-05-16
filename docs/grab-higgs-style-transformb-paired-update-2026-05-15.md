# Grab HIGGS-Style TransformB Paired Update - 2026-05-15

## Why This Exists

The pivot split overlay and the Yum Yum Deviled Eggs logs show that the object
side `transformB` translation bytes can look stable while the object still
behaves like the angular target is correcting around the wrong relationship.

The important HIGGS rule is not "put PivotB at COM" or "put PivotB at the far
end." HIGGS separates:

- Pivot A: palm point in hand/body-A space.
- Pivot B: selected material/contact point expressed through the same
  hand/object relation as the constraint.
- COM/mass: mass, inertia, force cap, torque/lever, haptics, release data only.

In HIGGS `Hand::TransitionHeld`, the object is translated so the selected
`ptPos` seats at the palm. During held update, HIGGS recomputes transformB from
the inverse desired hand/object relation:

```cpp
desiredHandTransformHavokObjSpace = InverseTransform(desiredTransformHandSpace);
newPivotB = desiredHandTransformHavokObjSpace * palmPosHandspace;
constraintData->m_atoms.m_transforms.m_transformB.m_translation = newPivotB;
```

That keeps the linear pivot and angular relation paired. ROCK already had the
equivalent helper, `constraintDrivePivotBBodyLocalGame(...)`, but the active
proxy update path was bypassing it and writing the frozen selected-point local
directly.

## Implementation Rule For This Pass

- Keep BODY as the visual object-side frame. Re-express the same contact pivot
  in the solver body-B frame when FO4VR exposes MOTION; do not let MOTION/COM
  select or move the visual grip point.
- Compute active constraint PivotB from:
  `computeDynamicTransformBTranslationGame(desiredBodyTransformBodyASpace,
  pivotAProxyLocalGame)`.
- Use that computed PivotB at create and per-frame update in the active solver
  frame.
- Keep visual/contact PivotB separately as `_grabFrame.pivotBBodyLocalGame` so
  debug can still show the selected material point.
- Keep COM/mass available for force, inertia, lever length, and release only.

## Expected Evidence

- `transformBLocal` may differ from the raw frozen contact local, but it should
  be paired with the body-A palm relation.
- `transformBErr` should remain near zero because the bytes and computed desired
  transformB use the same formula.
- If rotation still fails after this, the next target is the ragdoll angular
  target direction itself, not the linear PivotB placement.
