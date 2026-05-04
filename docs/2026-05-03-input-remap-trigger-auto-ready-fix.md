# Input Remap Trigger Auto-Ready Fix

Date: 2026-05-03

## Why This Change Exists

The refactor left the native ReadyWeapon action suppression in place, but the game can still auto-ready from the right trigger path if the game-facing OpenVR controller state contains trigger input. FO4VR exposes trigger as both button 33 and analog Axis1.x, so suppressing only the ReadyWeapon action or only button bits is not a complete boundary. ROCK now treats holstered right-trigger suppression as part of the same input-remap policy that owns right-grip and Favorites suppression.

## Evidence

- HIGGS blocks trigger/grip at the OpenVR controller-state boundary when it owns interaction input.
- FO4VR/OpenVR headers define `k_EButton_SteamVR_Trigger` as Axis1, and OpenVR documents trigger analog data in Axis1.x.
- Ghidra verification of FO4VR `0x140FCE650` showed the ReadyWeapon handler is a side-effect-free matcher. Suppressing it is necessary, but it does not cover attack/trigger auto-ready.

## Implementation

- `InputRemapPolicy` now emits `filteredAxisMask` for game-facing analog axes that must be zeroed.
- While the right weapon is holstered and `bSuppressNativeReadyWeaponAutoReady=true`, the policy removes:
  - right trigger button bit 33 from `ulButtonPressed`
  - right trigger touch bit 33 from `ulButtonTouched`
  - right trigger analog Axis1 from `VRControllerState_t::rAxis`
- Runtime filtering applies that policy output only to game-facing OpenVR samples, preserving ROCK raw input tracking.

## Verification

- `ROCKInputRemapPolicyTests.exe`
- `InputRemapRuntimeSourceTests.ps1`
- Full Release build
- All current ROCK `*Tests.exe`
- Native/source boundary scripts

