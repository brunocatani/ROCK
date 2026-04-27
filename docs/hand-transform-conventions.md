# Hand Transform Conventions

ROCK hand collision uses one resolved hand frame for collision, palm selection, grab math, and debug axes. The default source is the FO4VR native wand/fist-helper frame, not the first-person `RArm_Hand` / `LArm_Hand` bone.

This is intentionally different from the previous hand-bone path. The debug body overlay proved the renderer was faithfully drawing the live hknp hand bodies; the collision itself was wrong because the upstream frame was wrong. The visible failure was world/body-yaw dependent: the hand collider axes followed the collider exactly, but both stayed aligned to the wrong reference as the player turned. That points to source-frame selection, not Havok drawing or a cosmetic axis overlay.

HIGGS still provides the hand-space model: use one real tracked hand frame directly, apply the local hand collision offset in that frame, and mirror left-hand local X. ROCK no longer invents an authored swizzle (`X=raw Y`, `Y=raw Z`, `Z=raw X`).

Current INI controls:

- `iHandFrameSource = 0`: fist helper under native wand, falling back to native wand node.
- `iHandFrameSource = 1`: native wand node only.
- `iHandFrameSource = 2`: first-person `RArm_Hand` / `LArm_Hand` bone.
- `iHandFrameSource = 3`: FRIK API hand transform.
- `bHandFrameSwapWands = false`: swap primary/secondary wand slots when diagnosing handedness setups.

Ghidra-verified FO4VR hand-frame details:

- `NiAVObject` parent pointer is at `+0x28`; local transform starts at `+0x30`; world transform starts at `+0x70`.
- `PlayerCharacter +0x6F0` is the primary wand node.
- `PlayerCharacter +0x768` is the secondary wand node.
- `FUN_140ef90c0` requires both `+0x6F0` and `+0x768` and binds the native VR view/controller systems to those nodes.
- Active FRIK places fist helper nodes under the primary/secondary wand nodes and uses the first-person hand bones only after its IK/body solve.

Ghidra-verified FO4VR Havok details:

- `0x141e08a70` forwards `bhkNPCollisionObject::SetTransform` to the hknp world transform path.
- `0x141df55f0` copies/dequeues the transform memory unchanged before calling the world path.
- `0x1415395e0` consumes `hkTransformf` rotation as column blocks: column 0 at floats `0..2`, column 1 at `4..6`, column 2 at `8..10`, translation at `12..14`.
- `0x141722c10` converts that matrix layout to quaternion order `x,y,z,w`.
- `0x14153a6a0` uses the same matrix-to-quaternion helper while computing hard-keyframe angular velocity.

Implementation rules:

- All hand systems must call the same resolved hand-frame path. Do not make collision use wand space while palm/grab uses hand-bone space.
- Convert Ni row-major matrices to Havok column blocks before writing `hkTransformf`.
- Convert Havok column blocks back to Ni row-major matrices before using body transforms in game-space math.
- Derive hard-keyframe target quaternions from the same Ni-to-Havok convention so angular velocity does not fight `SetTransform`.
- Keep weapon collision copying behavior unchanged at the behavioral level; it reads real FO4VR Havok bodies and writes them back through the same conversion helpers.
