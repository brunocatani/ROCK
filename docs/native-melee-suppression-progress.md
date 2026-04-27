# Native Melee Suppression Progress

## Why This Approach

ROCK needs PLANCK-style player melee ownership, not a blanket combat disable. HIGGS keeps native VR melee data alive and mainly fixes/clones collision bodies. PLANCK suppresses the player's animation-driven melee timing and later invokes native hit/damage side effects from physical swing/contact logic. ROCK therefore suppresses only the FO4VR player animation handlers that create native swing/hitframe timing, leaves NPC/native non-player behavior alone, and keeps the decision in a tested policy so the future physical-hit pipeline can plug into it without replacing working hook code.

## References Checked

- HIGGS: `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hooks.cpp`
  - `UpdateVRMeleeDataRigidBodyCtorHook` fixes native VR melee collision body construction.
- HIGGS: `E:\fo4dev\skirymvr_mods\source_codes\higgs\src\hand.cpp`
  - `CreateWeaponCollision` clones native melee collision into a HIGGS body and toggles that cloned body.
- PLANCK / ActiveRagdoll: `E:\fo4dev\skirymvr_mods\source_codes\activeragdoll\src\main.cpp`
  - `WeaponRightSwingHandler_Handle_Hook` and `WeaponLeftSwingHandler_Handle_Hook` skip player native swing callback/sound handling.
  - `HitFrameHandler_Handle_Hook` suppresses player animation hitframes unless a recent physical swing is active.

## Ghidra Findings

- FO4VR registers animation handlers through `FUN_140febb50`.
- `WeaponSwingHandler` is an `IHandlerFunctor<Actor, BSFixedString>` object, not the exact split right/left Skyrim handler shape.
- Factory constructor `0x14100f1a0` allocates `WeaponSwingHandler` and assigns vtable `0x142d8c9f8`.
- `WeaponSwingHandler::Handle` is vtable entry `0x142d8ca00`, function `0x140fef820`, RVA `0x0fef820`.
- Factory constructor `0x14100d7c0` allocates `HitFrameHandler` and assigns vtable `0x142d8cb90`.
- `HitFrameHandler::Handle` is vtable entry `0x142d8cb98`, function `0x140feffb0`, RVA `0x0feffb0`.
- Both handlers receive the actor in `RDX` and the `BSFixedString` event/side in `R8`, and return a boolean in `AL`.

## Implemented

- Added `NativeMeleeSuppressionPolicy.h`.
- Added tested policy cases:
  - NPCs pass through to native handlers.
  - Disabled suppression passes through to native handlers.
  - Player weapon swing returns unhandled when suppression is enabled.
  - Player hitframe returns unhandled unless a ROCK physical swing is active.
  - Future physical swing can mark the hitframe handled without calling native animation-driven hitframe code.
- Added config:
  - `bNativeMeleeSuppressionEnabled=true`
  - `bNativeMeleeSuppressWeaponSwing=true`
  - `bNativeMeleeSuppressHitFrame=true`
  - `bNativeMeleeDebugLogging=false`
- Added FO4VR vtable hooks for the verified handler slots.
- Added `setNativeMeleePhysicalSwingActive()` / `isNativeMeleePhysicalSwingActive()` scaffolding for the later physical swing implementation.

## Not Implemented Yet

- Physical swing detection.
- Native `PlayerControls_SendAction`, attack-state, and `Character_HitTarget` invocation.
- Owner-aware melee hit filtering.
- Attack window start/stop tracking.

Those pieces require separate Ghidra verification of FO4VR attack state layout and hit/damage call signatures before use.
