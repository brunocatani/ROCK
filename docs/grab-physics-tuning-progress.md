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

## Open Research

- Full room-rotation warp should use a verified FO4VR hknp transform setter or native reintegration path before being enabled. This pass detects large player-space deltas and avoids adding a huge velocity spike, but does not write arbitrary body transforms.
- FRIK exposes custom finger values today. True per-knuckle joint matching needs a separate verified FRIK joint API pass rather than being folded into this physics update.
