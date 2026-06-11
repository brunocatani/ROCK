# IK-Coupled Weapon World Contact Plan

Date: 2026-06-11
Project: ROCK
Status: planned / deferred

## Summary

ROCK hand world soft contact is working through real generated hand colliders. The next weapon-contact pass should give equipped guns the same world-stop behavior, but as weapon/IK authority rather than as another hand soft-contact branch.

The target behavior is:

- equipped guns stop on world surfaces using the real generated layer-44 weapon colliders;
- the visible weapon, primary hand, and support hand remain coupled instead of the gun being clamped separately;
- barrel/muzzle-style contacts get a conservative pivot response;
- final muzzle/fire-node authority still runs after ROCK writes the corrected weapon transform.

No Ghidra is needed for the first implementation. Current ROCK source already has the necessary contact classification, generated weapon body metadata, two-handed weapon authority, FRIK external hand target bridge, and final muzzle authority path. If implementation uncovers an engine-order or binary-layout question that local source cannot answer, pause and request scoped Ghidra use before inspecting binaries.

## Key Changes

- Publish `WeaponWorldSurface` native evidence by extending the current world-surface evidence path beyond hands.
- Record enough weapon metadata in native evidence to reject stale contacts:
  - source body ID;
  - weapon generation key;
  - source part kind / semantic role;
  - target body/layer/filter information;
  - raw contact point and normal.
- Add a weapon-owned `WeaponWorldContactRuntime` instead of expanding `SoftContactRuntime`.
- Consume weapon-world native evidence after normal weapon/two-hand solving and before generated weapon bodies are driven from current source transforms.
- Keep `SoftContactRuntime` hand-world-only.
- Preserve current layer ownership:
  - hands remain on layer 43;
  - weapons remain on layer 44;
  - body remains on layer 47;
  - no side-specific hand layers are introduced.

## Correction Strategy

The first implementation should use a two-stage correction:

1. Bounded pivot around the primary/right grip for forward weapon parts.
2. Bounded residual translation along the world normal so the weapon cannot remain inside the surface.

Pivot eligibility should be conservative:

- allow pivot for forward/contact-leading parts such as `Barrel`, `Handguard`, `Foregrip`, and `Pump`;
- prefer translation for `Receiver`, `Grip`, `Stock`, reload pieces, and unknown/cosmetic parts;
- reject pivot if the primary grip frame, weapon generation, contact normal, or contact point is missing or non-finite.

The pivot must be clamped and compositional. It should never become a full physics solver or rotate freely based on a single noisy contact. Translation remains the fallback that guarantees visible clearance.

## Visual And IK Authority

Apply the corrected weapon frame through existing weapon authority paths, not by directly patching arbitrary child nodes.

- Full two-handed grip:
  - solve the normal two-handed weapon frame;
  - apply weapon world-contact correction;
  - publish locked primary and support hand targets from the corrected weapon frame.
- Visual-only support grip:
  - correct the equipped weapon frame;
  - keep support hand locked to the corrected weapon frame;
  - while weapon contact is active, also publish a right-hand target from the captured primary hand/weapon relation so the gun does not detach from the primary hand.
- One-handed equipped weapon:
  - correct the equipped weapon frame;
  - publish the right-hand target from the current captured weapon/right-hand relation while contact is active.

After the corrected weapon transform is applied, run final muzzle authority so the fire node follows the projectile/barrel node in the final frame.

## Config Defaults

Planned internal/user-facing defaults:

```ini
bWeaponWorldContactEnabled=true
fWeaponWorldContactMaxCorrectionGameUnits=18.0
fWeaponWorldContactMaxPivotDegrees=12.0
fWeaponWorldContactPivotResponse=0.65
fWeaponWorldContactSkinGameUnits=0.5
fWeaponWorldContactCachedPlaneMaxTangentDriftGameUnits=12.0
```

If these are exposed in `ROCK.ini`, update both default config copies and production INI only when explicitly requested.

## Tests And Validation

Add focused tests before runtime validation:

- weapon-world evidence is published for `WeaponWorldSurface`;
- hand-world soft contact remains hand-only and does not consume weapon evidence;
- stale weapon-generation contacts are rejected;
- pivot eligibility matches weapon part semantics;
- pivot correction clamps by degrees and preserves the primary grip pivot;
- residual translation clears remaining penetration;
- update ordering keeps weapon contact correction before generated weapon body drive and final muzzle authority last.

Expected validation commands:

```bat
cd ROCK && cmake --preset custom-tests
cd ROCK && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%
cd ROCK && cmake --preset custom-fast
cd ROCK && cmake --build build-fast --config Release --target ROCK -- /m
```

## Assumptions

- This is visual/IK authority, not a physical force or constraint solver.
- Actual generated weapon colliders are the source of precision; fallback/cached evidence only stabilizes brief missing native contact frames.
- Cached contact is retained only while the same weapon body/generation still penetrates the same world plane within drift limits.
- Git commit rollback is the fallback mechanism; do not keep a shadow fake weapon-contact path.
