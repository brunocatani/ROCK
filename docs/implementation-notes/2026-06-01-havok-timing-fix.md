# 2026-06-01 ROCK Havok Timing Fix

## Objective

Add a ROCK-controlled Fallout 4 VR Havok timing fix equivalent in intent to HIGGS's Skyrim VR Havok fix: keep the engine physics step cadence aligned with VR frame cadence so generated hand colliders, grab authority proxies, and held objects do not run at a visibly lower physics clock than the game-frame skeleton sampling path.

## Source Authority

- Project: ROCK.
- Local binary verification: Ghidra MCP against `E:\fo4dev\reverse_engineering\Fallout4VR.exe.unpacked.exe`.
- Local source: ROCK timing, generated collider drive, config, and hook code.
- External web: not used.
- Confidence: high for the patched FO4VR function/call-site relationship; runtime effect still requires in-game log validation.

## Verified FO4VR Timing Findings

- `bhkWorld::SetDeltaTime` is at VA `0x141DF7120`, executable offset `0x1DF7120`.
- The main update path calls it at VA `0x140D84BD0`, executable offset `0x0D84BD0`.
- `SetDeltaTime` writes the existing ROCK timing globals:
  - raw delta: `0x1465A3D70`
  - current substep delta: `0x1465A3D74`
  - remainder delta: `0x1465A3D7C`
  - accumulated delta: `0x1465A3D84`
  - substep count: `0x1465A3D8C`
- `bhkWorld::Update` at VA `0x141DF73FE` consumes the current substep delta and substep count and passes the substep delta through the step-listener callbacks ROCK already uses.
- The `SetDeltaTime` entry prologue includes a RIP-relative instruction inside the bytes needed for an absolute entry jump, so ROCK must not use its current relocated-entry trampoline helper there.

## Design

ROCK hooks the verified main call site instead of the `SetDeltaTime` function entry. The hook calls the original function first, then optionally overwrites only the current substep delta and substep count globals. It does not overwrite the engine's base substep global, so disabling the config allows the original `SetDeltaTime` result to take over again on the next frame.

The policy is intentionally FO4VR-native. Ghidra did not show the HIGGS-style `fMaxTime` / `fMaxTimeComplex` global pattern in the verified FO4VR hknp path, so ROCK normalizes the current FO4VR `SetDeltaTime` sample directly.

Config keys:

- `bHavokTimingFixEnabled`
- `fHavokTimingFixMinPhysicsFrameRate`
- `iHavokTimingFixMaxSubsteps`

Policy:

```text
maxPhysicsFrameTime = 1 / fHavokTimingFixMinPhysicsFrameRate
substepDelta = rawDelta

for i = 2..iHavokTimingFixMaxSubsteps:
    if substepDelta <= maxPhysicsFrameTime:
        break
    substepDelta = rawDelta / i

substepCount = floor(accumulatedDelta / substepDelta), clamped to 1..maxSubsteps
```

Expected examples with `fHavokTimingFixMinPhysicsFrameRate = 70` and `iHavokTimingFixMaxSubsteps = 3`:

- 90 Hz frame: one substep at about `0.0111s`.
- 60 Hz frame: two substeps at about `0.0083s`.
- 30 Hz frame: three substeps at about `0.0111s`.

## Implementation Constraints

- Fail closed if the call-site validation does not match the verified FO4VR bytes.
- Keep diagnostics behind existing `bDebugVerboseLogging` / `bDebugGrabFrameLogging` gates.
- Do not modify active production INIs.
- Do not write the base substep global; only override the current sample after the original function runs.
- Preserve ROCK's existing generated collider timing diagnostics.

## Validation Plan

- Add a pure policy test for 90 Hz, 60 Hz, 30 Hz, clamping, and invalid deltas.
- Add a source-boundary test for verified offsets, call-site validation, original-first ordering, config exposure, diagnostics gating, and no base-global writes.
- Run ROCK test configure/build plus source-boundary tests.
- Run the `custom-fast` Release build, which auto-deploys `ROCK.dll` and `ROCK.pdb`.
- Runtime follow-up: enable timing/grab diagnostics and compare `PALM_CLOCK` plus generated drive `rawDt`, `subDt`, `driveDt`, and `substeps`.

## Progress Log

- 2026-06-01: User approved Ghidra MCP and implementation.
- 2026-06-01: Verified `bhkWorld::SetDeltaTime`, the main call site, and the consumed timing globals in `Fallout4VR.exe.unpacked.exe`.
- 2026-06-01: Implemented call-site hook, config keys, pure policy, packaged INI documentation, source-boundary test, and policy test.
- 2026-06-01: `cmake --preset custom-tests`, `ROCKPolicyTestBinaries`, and full `ctest` passed: 60/60 tests.
- 2026-06-01: `custom-fast` Release build passed and auto-deployed `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
- 2026-06-01: Review follow-up aligned the timing-fix minimum delta threshold with existing Havok timing sampling and added accumulated-delta policy coverage.

## Final-Stage Follow-Up: Stick-Locomotion Hand Authority Bridge

Runtime testing after the Havok timing fix showed the remaining visible shimmer only during controller stick locomotion while holding an object. Disabling `bGrabPlayerSpaceCompensation` did not remove it, physical room movement did not trigger it, and stick rotation was not involved. That points away from the central held-object player-space velocity writer and toward the root-translation component of the root-flattened hand authority.

Design chosen:

- Add `GrabLocomotionAuthorityBridge` as a small `PhysicsInteraction` state machine.
- Drive it from `runtime_state::currentFrame().playerSpace.valid/moving` only while either hand is holding.
- Estimate player-root velocity from `playerSpace.deltaGameUnits / frame.deltaSeconds`.
- Smooth velocity at `45 Hz`, project a bounded `0.012s` translation lead, and clamp to `4.0` game units.
- Reset on invalid player space, no held object, world/menu reset, invalid delta, or root jumps over `35.0` game units.
- Apply only translation. Do not smooth rotation, snap turn, controller-local hand motion, or finger pose.
- Apply the same offset to `HandFrameInput.rawHandWorld` and to generated hand-collider bone lookup transforms before palm/finger role frames are derived.

Config keys:

- `bGrabLocomotionAuthorityBridgeEnabled`
- `fGrabLocomotionAuthorityMaxLeadSeconds`
- `fGrabLocomotionAuthoritySmoothingHz`
- `fGrabLocomotionAuthorityMaxOffsetGameUnits`
- `fGrabLocomotionAuthorityResetDistanceGameUnits`

Validation plan:

- Policy test for zero offset on invalid player space, no held object, and not moving.
- Policy test for bounded predictive offset during steady stick locomotion.
- Policy test for root-jump reset.
- Source-boundary test proving the same bridge offset reaches both held hand-frame authority and generated hand-collider bone lookup.
- Runtime check: stick-walk while holding an object at arm's length; physical room movement and stick rotation should remain unchanged.

Progress:

- 2026-06-01: Implementing bridge as a final-stage follow-up to the timing/grab authority work.
- 2026-06-01: Added bridge policy/config, runtime frame authority offset, generated hand-collider lookup offset, diagnostics, source-boundary test, and policy test.
- 2026-06-01: `cmake --preset custom-tests`, `ROCKPolicyTestBinaries`, full `ctest`, and `custom-fast` Release plugin build passed. `custom-fast` auto-deployed `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
