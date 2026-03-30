# Player Controller And Native Biped Collision - 2026-05-08

## Why This Note Exists

CollisionVisualizer is only the observation tool here. The important finding is what native FO4VR Havok objects it exposed.

This note corrects an earlier bad hypothesis: the pill-shaped torso/arm/leg/foot bodies seen before ROCK existed are not ROCK `BodyBoneColliderSet` bodies. ROCK can now create similar-looking capsule bodies when body colliders are enabled, but the pre-ROCK visuals are native actor biped/ragdoll collision bodies.

## Corrected Identification

### Cross / Bubble Shape

The cross/bubble that jumps almost instantly to the target movement position, with the body following afterward, is the player character-controller proxy shape, not the actor body/ragdoll.

Relevant native functions verified in Ghidra:

- `Actor::GetCharController`: `0x140DC42E0`, address-library id `396460`
- `bhkCharacterController::SetPosition`: `0x141E20EE0`
- `bhkCharacterController::GetPosition`: `0x14080A9F0`
- `bhkCharacterController::Move`: `0x141E22130`
- `bhkCharacterController::CheckGoodPos`: `0x141E21030`
- `bhkCharacterController::CheckLayerForGoodPos`: `0x141E24130`
- `bhkCharacterControllerShapeManager::CreatePencilShape`: `0x141E2BA30`
- `bhkCharacterControllerShapeManager::GetPencilShape`: `0x141E2CDF0`
- `bhkCharacterControllerShapeManager::GetCapsuleShape`: `0x141E2C910`

The visible odd cross/bubble form is consistent with the controller's non-ragdoll proxy geometry, especially the native "pencil" shape path. `CreatePencilShape` constructs an `hknpConvexPolytopeShape`, not a skeletal biped body.

The controller setup path also references a character bumper layer wrapper:

- `GetCharacterBumperLayer`: `0x141E28360`
- `L_CHARBUMPER` string: `0x142E88370`
- `bhkCollisionFilter::FindLayerByName`: `0x141E11790`

Static Ghidra confirms the `L_CHARBUMPER` lookup, but not the final numeric runtime BGSCollisionLayer value in this pass. That does not change the movement result: the player hard-stop behavior is governed by the character-controller movement/filter path, whose collision layer is `CHARCONTROLLER = 30`.

### Pill-Shaped Torso / Arms / Legs / Feet

The old pill-shaped body forms are native actor biped/ragdoll collision bodies.

Relevant native functions verified in Ghidra:

- `BipedAnim::InitHavok`: `0x1401CB430`
- Recursive collision-object setup helper: `0x1401CF320`
- Collision filter application helper: `0x1401CD900`
- Actor Havok/controller setup path: `Actor::vfunction134`, `0x140DC6F41`

`BipedAnim::InitHavok` applies filter info with low collision-layer bits set to `8`:

```c
local_40 = uVar10 << 8 | 8 | uVar12 << 0x10;
FUN_1401CF320(param_3, &local_48);
```

Layer `8` is native `BIPED`. This setup walks the actor's collision scene tree and applies the biped filter to the Havok collision objects.

SCISSORS source and logs independently match this:

- Live ragdoll bodies are read through the native hkbnp ragdoll path.
- Typical actor ragdoll body counts are `18` or `24`.
- Runtime contact logs show NPC body contacts on layer `8`.

Example local runtime witnesses:

- `Left hand touched [NPC_] 'Dog' ... body=3052 layer=8`
- `Weapon dynamic push applied: 'Dog' ... targetBody=3057 layer=8 acceptedBodies=24`
- `Left hand touched [NPC_] 'Hell-Raiser Looter' ... body=3582 layer=8`
- `Weapon dynamic push applied: 'Diamond City Security' ... targetBody=5271 layer=8 acceptedBodies=18`

## Layers And Collision Matrix

The vanilla hknp layer matrix was verified through:

- `bhkCollisionFilter::InitFlags`: `0x141E11950`
- `bhkCollisionFilter::SetLayerCollision`: `0x141E13D70`
- `bhkCollisionFilter::SetMultiLayerCollision`: `0x141E13DF0`

The matrix lives at collision-filter offset `+0x1A0`. The multi-layer helper writes the row mask and mirrors that layer bit into all affected rows.

### Player Character Controller

- Layer: `30`
- Name: `CHARCONTROLLER`
- Mask47: `0x79CEFD737EFE`
- Collides with layers:
  - `1, 2, 3, 4, 5, 6, 7`
  - `9, 10, 11, 12, 13, 14`
  - `16, 17`
  - `20, 21, 22, 24`
  - `26, 27, 28, 29, 30, 31`
  - `33, 34, 35`
  - `38, 39, 40, 43, 44, 45, 46`
- Does not collide with normal `BIPED = 8`.
- Does not collide with `DEADBIP = 32`.
- Does collide with `BIPED_NO_CC = 33`.

### Native Biped / Ragdoll Body Pills

- Normal layer: `8`
- Name: `BIPED`
- Mask47: `0x5BC03C037B8F`
- Collides with layers:
  - `0, 1, 2, 3`
  - `7, 8, 9`
  - `11, 12, 13, 14`
  - `16, 17`
  - `26, 27, 28, 29`
  - `38, 39, 40, 41, 43, 44, 46`
- Does not collide with `CHARCONTROLLER = 30`.
- Does not collide with `DEADBIP = 32`.
- Does not collide with `BIPED_NO_CC = 33`.

### Dead Biped Context

- Layer: `32`
- Name: `DEADBIP`
- Mask47: `0x3BC13E5B7EFF`
- Collides with layers:
  - `0, 1, 2, 3, 4, 5, 6, 7`
  - `9, 10, 11, 12, 13, 14`
  - `16, 17, 19, 20, 22, 25`
  - `26, 27, 28, 29, 32`
  - `38, 39, 40, 41, 43, 44, 45`
- Does not collide with `CHARCONTROLLER = 30`.

### Biped No Character Controller Context

- Layer: `33`
- Name: `BIPED_NO_CC`
- Mask47: `0x0040400004F0`
- Collides with layers:
  - `4, 5, 6, 7, 10, 30, 38`
- This is the biped-family layer that does collide with the player character controller.

### ROCK Caveat

Current ROCK can create its own body capsules through `BodyBoneColliderSet`:

- Source: `ROCK/src/physics-interaction/body/BodyBoneColliderSet.cpp`
- Filter used by ROCK body colliders: `0x000B002B`
- Layer: `43`

Those bodies can look similar in CollisionVisualizer if ROCK is loaded and `bBodyBoneCollidersEnabled=true`, but they are not the pre-ROCK native pill bodies described above.

## How The Hard Stop Works

The hard stop is character-controller movement, not native biped body collision.

The movement path sends desired motion into `bhkCharacterController::Move`. The controller performs shape queries/casts and support checks through the hknp world, using the character-controller collision layer and native controller filtering. When the controller hits a blocking layer, the controller motion is clamped, slid, or stopped. The actor/body then follows the resolved controller position.

This explains the observed timing:

1. Controller proxy/cross moves almost immediately to the resolved target.
2. The visible body follows the controller.
3. Native biped/ragdoll pills remain actor body collision, but normal layer `8` bodies do not create the player hard stop because `8 <-> 30` is disabled in the vanilla matrix.

Relevant pair checks from the verified matrix:

- `BIPED 8` vs `CHARCONTROLLER 30`: disabled both ways
- `DEADBIP 32` vs `CHARCONTROLLER 30`: disabled both ways
- `BIPED_NO_CC 33` vs `CHARCONTROLLER 30`: enabled both ways
- `BIPED 8` vs `BIPED_NO_CC 33`: disabled both ways
- `BIPED 8` vs `DEADBIP 32`: disabled both ways

## Open Verification

The exact numeric runtime value behind `L_CHARBUMPER` was not resolved from static binary analysis alone in this pass. It is looked up through BGSCollisionLayer data by name. Resolve it later with a runtime dump or FO4 data extraction if the exact numeric bumper layer becomes necessary.

The important corrected conclusion is already solid: the old torso/arm/leg/foot pill shapes are native layer-8 biped/ragdoll bodies, while the snap-to-target cross/bubble behavior is the player character-controller/proxy path using the layer-30 movement collision rules.
