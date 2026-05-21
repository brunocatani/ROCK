# Grab Pinch Pocket Implementation Notes - 2026-05-21

Project: ROCK

Source authority: current local ROCK source and user-approved design direction. No web, Ghidra, FO4 Mods MCP, or external reference source used.

Confidence: implemented and locally validated.

## Intent

Very small compact objects and short thin objects need a distinct close-grab seat. Palm-pocket grab is correct for normal objects, but tiny items like coins, balls, pocket watches, and cigarette-like rods should seat between thumb and index instead of trying to become palm-held objects.

## Runtime Invariants

- Pinch and palm are mutually exclusive grab seat modes chosen at grab commit.
- Pinch is only considered for close grabs with coherent local mesh geometry and a plausible thumb/index gap.
- The pinch pocket is captured from the root-flattened thumb/index snapshot at button press.
- Held updates must not recompute the pinch pocket from animated finger colliders or live finger bones.
- The held object remains dynamic. The frozen value is the BODY-local object point plus the hand/proxy-local relation.
- Pinch disables seated palm-pocket reacquire and held support refresh for the lifetime of that grab.
- Thumb/index finger targets are stored object-local; middle/ring/pinky are closed by policy.
- Palm close selection remains first priority. Pinch close selection is a fallback cast from the live thumb-index pocket only when the palm close cast misses.
- A pinch-direction fallback selection must qualify for pinch pocket at grab commit; it does not fall through to palm-pocket authority.
- Pinch-direction selection hysteresis and selected-close speed tracking use the live pinch origin rather than the palm origin.
- Runtime pinch selection resolves the thumb-index pocket lazily after the palm close cast misses; the per-frame frame-context capture is used only for pocket debug drawing.
- Pinch mesh search uses a configurable authored hand-space direction blended with the live thumb-index axis.

## Planned Files

- `src/physics-interaction/grab/GrabPinchPocket.h`: classifier/math policy for compact and short-thin objects.
- `src/physics-interaction/grab/GrabCore.h`: seat-mode storage on the canonical grab frame.
- `src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl`: per-hand palm/pinch pocket visual markers.
- `src/physics-interaction/hand/HandGrab.cpp`: commit-time arbitration and pinch pose publication.
- `src/RockConfig.h`, `src/RockConfig.cpp`, `data/config/ROCK.ini`: tuning defaults.
- `tests/GrabPinchPocketPolicyTests.cpp` and source-boundary tests: classifier and no-feedback guarantees.

## Initial Defaults

- Compact max extent: 10 game units.
- Thin rod max length: 18 game units.
- Thin rod max cross-section: 4 game units.
- Max pocket-to-object distance: 8 game units.
- Thumb/index gap: 1 to 12 game units.
- Thumb/index max-open curl: 0.45.
- Other finger curl: 0.20.
- Pinch detection direction: hand-space X/finger-forward.
- Pinch detection axis blend: 0.65 live thumb-index axis, 0.35 authored pinch direction.

## Current Status

- Implemented close-grab pinch-pocket arbitration as a commit-time seat mode.
- Added `GrabPinchPocket.h` classifier/math policy for compact and short thin objects.
- Added canonical grab-frame seat state so palm pocket and pinch pocket are mutually exclusive.
- Pinch captures the thumb/index distal-pad pocket from the root-flattened finger snapshot at grab commit, then stores object-local thumb/index pose targets.
- Pinch bypasses held palm-pocket support refresh and seated palm-pocket reacquire for the lifetime of the grab.
- Pinch bypasses the generic thumb curve solver and post-processes thumb/index plus middle/ring/pinky finger values from config.
- Added a per-hand debug overlay toggle for palm pocket center/radius/depth and pinch center/thumb-index axis/detection direction.
- Added fallback pinch close selection from the live thumb-index pocket using the pinch direction only after palm close selection fails.
- Tagged pinch-direction close selections so non-pinchable hits fail closed instead of becoming accidental palm grabs.
- Kept pinch fallback selection distance and selected-close speed tied to the pinch origin.
- Added INI-backed tuning defaults and tests.

## Validation

- `cmake --preset custom-tests` passed.
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m` passed.
- `ctest --test-dir build-tests -C Release --output-on-failure -j $env:NUMBER_OF_PROCESSORS` passed: 44/44.
- `cmake --preset custom-fast` passed.
- `cmake --build build-fast --config Release --target ROCK -- /m` passed and auto-deployed `ROCK.dll`/`ROCK.pdb` to `D:/FO4/mods/ROCK/F4SE/Plugins/`.

## Code Review Notes

- Pinch is close-grab only and rejects authored grab nodes, hand-pocket-only targets, loose weapons, multi-body assemblies, owner mismatch, missing mesh, missing finger snapshot, bad thumb/index gap, and far pocket-to-surface distance.
- The frozen relation is the object/body-local point and hand/proxy relation. Runtime updates do not recompute the pocket from animated finger colliders.
- Generic palm grabs keep the existing `rockPointToPalm` relation and authored grab nodes remain the only generic rotation override.
