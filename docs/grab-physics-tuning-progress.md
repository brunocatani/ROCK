# ROCK Held Object Grab Physics Tuning Progress

## Why This Path

Normal dynamic object grabs and equipped weapon support grips are now treated as separate systems. HIGGS solves normal object carrying by keeping grabbed bodies coherent in player space, damping only the residual body motion, and restoring a throw velocity from recent local motion on release. ROCK had a constraint that followed the hand, but its per-frame damping was applied to the full body velocity, so normal player locomotion could look like the object was being re-solved or teleported each frame. The chosen approach keeps the existing FO4VR hknp constraint and verified hand conventions, adds HIGGS-style player-space compensation around it, and does not change the weapon semantic/two-handed path.

## Reference Findings

- HIGGS maintains a player-space body set and adds/subtracts room/player-space velocity to carried bodies each frame.
- HIGGS warps player-space bodies for large room rotation changes rather than forcing the solver to catch up through a velocity spike.
- HIGGS release behavior uses the largest recent local hand/object velocity, then adds player-space velocity back in so throws are stable while walking.
- HIGGS grab constraint defaults use proportional recovery `2.0`, constant recovery `1.0`, and damping `0.8` for both linear and angular motors.
- FO4VR Ghidra verification already confirmed the hknp deferred velocity setter and hard-keyframe velocity helper; no new native offset is needed for this implementation pass.

## Implemented Scope

- Pure math coverage for player-space velocity conversion, residual damping, release velocity composition, lerp duration, tau stepping, and angular force ratio.
- Runtime config for player-space compensation, residual damping, distance-based hand lerp duration, and recurring mesh finger-pose refresh.
- Held-object update path receives a player-space frame from `PhysicsInteraction` and applies it only to normal grabbed bodies.
- Weapon two-handed grip remains routed through `TwoHandedGrip` and `WeaponCollision`; this file does not merge the two behaviors.

## 2026-04-27 HIGGS-Grade Grab Pass

ROCK now separates adaptive motor target computation into `GrabMotionController.h`. Normal grabbed objects measure live body error against the desired hand-relative body transform every frame, then raise linear/angular tau and max force only when the body is falling behind. The force still fades in at grab start, stays mass-capped, and drops to the collision tau while the held body reports contact.

Mesh finger posing now prefers FRIK v5 per-joint values when available. The solver still uses ROCK's verified active handspace convention, but it now adds an alternate thumb probe, expands five mesh curl values into fifteen joint values, and smooths joint movement while held so the hand molds toward the mesh instead of constantly reapplying a fixed fist.

## 2026-04-27 Grab Angular Frame Finding

The rotation lag was a frame initialization mismatch, not evidence that the ragdoll angular motor is unusable. HIGGS creates the grab constraint with transform B and `target_bRca` both in the inverse desired body-to-hand frame, then refreshes `target_bRca` and pivot B during the hold loop. ROCK already refreshed `target_bRca` with that inverse frame per frame, but initialized transform B from the forward desired body-in-hand rotation. The linear pivot path could still land exactly, while the angular atom started from a different local frame and had to solve against stale orientation state.

ROCK now centralizes the convention in `GrabConstraintMath.h`: constraint creation writes transform B and initial `target_bRca` from `inverse(desiredBodyTransformHandSpace)`, and runtime target updates reuse the same Havok-column writer. This leaves motor tau/force policy unchanged while making the angular frame match the HIGGS `handTransformObjSpace` path.

## 2026-04-27 Havok Parsed Function Status

Current confirmed addresses still support the custom 6-atom constraint path: `0x1419B1D50` constructs `hkpRagdollConstraintData`, `0x1419B2910` initializes the native ragdoll atom block, `0x141AFD600` is the three-argument motor data calculator used by linear/ragdoll motors, `0x1415469B0` is `hknpWorld::createConstraint`, and `0x14153A6A0` matches the hard-keyframe delta-time helper. `GHIDRA_CONFIRMED_ADDRESSES.md` remains authoritative for atom/runtime offsets; older parsed notes that describe the angular motor as schema `0x0A` should not override the confirmed hkp atom type `19` and the verified runtime offsets.

## Open Research

- Full room-rotation warp should use a verified FO4VR hknp transform setter or native reintegration path before being enabled. This pass detects large player-space deltas and avoids adding a huge velocity spike, but does not write arbitrary body transforms.
- FO4VR Ghidra review during this pass confirmed the existing hard-keyframe helper, deferred velocity wrapper, mass-property rebuild path, and constraint-info utility directionally match ROCK's current usage. A deeper blind audit of every custom constraint atom field is still required before changing atom layout or adding new low-level constraint types.
