# Native Melee Direct Collision Gap

Date: 2026-05-09

## Why This Exists

The previous native-melee suppression layer blocked the verified FO4VR
animation-event vtable handlers, the `AttackBlockHandler` RightStick gate, and
the live `VRInput` velocity thresholds. Gameplay still shows native melee during
fast turns and weapon swings, so the remaining source is not SCISSORS applying
replacement damage. The local active-ragdoll reference shows two additional
native layers that ROCK has not implemented yet: direct swing-handler entry
hooks that can bypass the vtable and a per-physics-step disable of the engine's
own VR melee collision body.

## Local Evidence

- ROCK log `2026-05-09 16:56:34` confirms the current build installed
  `WeaponSwingHandler`, `HitFrameHandler`, and
  `AttackBlockHandler::ShouldHandleEvent` hooks.
- The same ROCK log confirms the live FO4VR settings
  `bMeleeVelocityCheck:VRInput`,
  `fMeleeLinearVelocityThreshold:VRInput`, and
  `fMeleeAngularVelocityThreshold:VRInput` were controlled by ROCK.
- SCISSORS log samples show contacts being processed, but the sampled damage
  totals are `damage(applied=0, ...)`. There is no sampled
  `SCISSORS: native physics hit applied` line in the latest run, so SCISSORS is
  not currently the source of native-looking melee hits.
- Active-ragdoll/Planck source disables native melee with more than animation
  hooks:
  - It installs direct right/left swing handler entry hooks because SKSE can
    call the native swing functions directly instead of through the vtable.
  - It disables the native `VRMeleeData` collision body after creation by
    setting the collision-filter disable bit and updating the body filter.

## Required FO4VR Verification Before Code

The active-ragdoll offsets and `VRMeleeData` layout are Skyrim VR data and must
not be copied into ROCK. The FO4VR implementation needs Ghidra verification of:

- FO4VR direct right/left swing entrypoints, signatures, prologue bytes, and
  whether direct calls bypass the vtable hook.
- FO4VR storage and layout for native VR melee collision data, including the
  collision node/body pointer and the fields that toggle collision/impulse.
- The safe hknp equivalent of the active-ragdoll filter update for the native
  melee collision body.

## FO4VR Ghidra Verification Added

- `WeaponSwingHandler::Handle` is at `0x140FEF820`; its vtable slot is
  `0x142D8CA00`. The handler resolves the equipped side and calls the actor
  virtual weapon-swing callback.
- `PlayerCharacter::WeaponSwingCallBack(BGSEquipIndex)` is at `0x140F23E00`;
  its PlayerCharacter vtable slot is `0x142D817A8`. The callback calls the base
  actor swing callback and then dispatches player weapon-swing side effects.
- `get_xrefs_to` for both functions showed only data/vtable references, not a
  stable direct callsite. The verified production layer is therefore a
  PlayerCharacter vtable hook at `0x2D817A8`, not an unverified branch patch.
- `VRMeleeImpact` setup is at `0x140F38630` and is reached through recursive
  node collision attachment from `0x140F4B760`. It registers the impact
  callback at `0x140EFF000`.
- `VRMeleeImpact` callback `0x140EFF000` resolves the equipped item, rejects
  invalid targets, dispatches action data, applies native hit side effects, and
  writes the player melee cooldown/impact state around offsets `0x908`,
  `0x90C`, and `0x9A0` through `0x9C0`.
- The verified production layer is a validated entry trampoline on
  `0x140EFF000` that returns early only while ROCK full native melee
  suppression is active for the player. This disables the native contact-to-hit
  path without mutating hknp bodies or first-person node ownership.
- Follow-up review fixed two install-safety details:
  - The `VRMeleeImpact` entry bytes are pre-validated with the vtable targets
    before any native melee hook is patched.
  - If any hook install fails after patching begins, ROCK rolls back the whole
    native melee hook set and does not arm the VRInput threshold watchdog.
- The impact callback now has its own explicit policy: full player native melee
  suppression disables native impacts independently of the HitFrame toggle,
  while partial suppression and NPC impacts continue to call native.

## Current Conclusion

The remaining native melee is most likely ROCK-side suppression coverage, not
SCISSORS emitting the replacement hit. ROCK now needs the verified
PlayerCharacter weapon-swing callback suppression layer and the verified
VRMeleeImpact callback suppression layer. A lower-level hknp body filter watchdog
can still be studied later, but the current safe fix is to stop the native impact
callback that owns hit/damage dispatch rather than broadly mutating native body
filters.
