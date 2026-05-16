# Grab Authority Ladder Diagnostics - 2026-05-16

## Why this exists

The dynamic grab failure is no longer just "which motor value is wrong." The in-game
debug view showed the palm/proxy reference can look correct while the held object
still follows the wrong angular relation. HIGGS avoids that class of failure by
feeding pivot B and the angular target from the same saved body-B relation. ROCK
now needs a diagnostic that proves, frame by frame, whether our proxy authority
chain does the same thing.

This file started as a telemetry pass, but the same review exposed two frame
simplifications. First, the hidden proxy body and the angular authority frame
were two names for the same runtime frame after raw-hand rotation was applied.
Second, the generated palm collider and the old manual handspace PivotA were
still being allowed to provide the authority position. The current diagnostic
pass removes both from dynamic grab authority: ROCK now feeds the grab relation
from the root-flattened hand frame itself.

## Frame ladder to prove

Names:

- `H`: raw root-flattened hand/controller frame.
- `A`: active authority frame, equal to `H`.
- `D`: desired object node frame.
- `DB`: desired dynamic body-B frame.
- `R`: saved body-B relation in authority-frame space.

Required relationships:

```text
A = H
DB = A * R
pivotA_world = A * pivotA_local
pivotB_world = DB * pivotB_local
```

For a coherent grab:

```text
pivotA_world ~= pivotB_world
angular desired body ~= linear desired body
live body in authority space ~= saved R, except for finite-force lag
constraint transform-B rotation ~= expected body-to-A rotation
ragdoll angular target ~= expected target composed with transform-A
```

## What the new telemetry should expose

New log label:

```text
GRAB AUTHORITY LADDER
```

Fields:

- `rawAuthority`: translation offset between raw hand origin and the actual
  authority body, plus rotation agreement between raw hand and authority.
- `desiredSplit`: difference between angular desired body and linear desired
  body.
- `pivot`: difference between reconstructed pivot A world and reconstructed
  pivot B world.
- `liveRel`: current live body relation inside authority space compared against
  the saved body relation.
- `atom`: transform-A, transform-B, and target bytes compared against the
  expected rotations.
- `atomPivot`: constraint byte pivot A/B compared against the expected local
  pivot values.

## Interpretation

- If `rawAuthority` has non-zero rotation, the authority body is not using raw
  flattened hand rotation.
- If `desiredSplit` is large, linear and angular targets are being built from
  different frames.
- If `pivot` is large while `desiredSplit` is small, pivot B is not derived from
  the same relation as angular target.
- If `liveRel` is large but all expected/atom checks are small, the solver is
  lagging or the body is fighting collision/mass; that is force tuning, not frame
  math.
- If `atom` is large, the byte writer or expected target direction is wrong.
- If `atomPivot` is large, the code's saved pivots and the actual constraint
  bytes disagree.

## Current suspected ambiguity

Several ROCK variables use "proxy" language for two different concepts:

- palm/proxy placement frame;
- hybrid authority frame after raw-hand rotation is applied.

The ladder diagnostic is meant to reveal whether that ambiguity is harmless
identity composition or whether it is feeding one frame into a function expecting
the other.

## Simplification applied with the diagnostic

The active path should now treat the hidden proxy body as the authority body, and
that authority body is fed from the root-flattened hand frame rather than from a
generated palm-collider frame or the old configured handspace PivotA:

```text
authorityWorld = root-flattened hand bone transform
proxy body is driven to authorityWorld
transform A local rotation is identity
desired body = authorityWorld * saved body-in-authority relation
pivot B = inverse(saved body-in-authority relation) * pivot A
```

That removes the old collider/INI-authority shapes:

```text
generated palm collider -> converted proxy frame -> authority frame
root-flattened hand frame + configured handspace point = authority frame
configured fGrabPivotAHandspace point -> authority position
root-flattened hand frame = authority frame
```

The generated palm/finger colliders and `fGrabPivotAHandspace*` values remain
useful for comparison telemetry and non-dynamic callers. They are no longer the
source of the dynamic grab authority frame.
