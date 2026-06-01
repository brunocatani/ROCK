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
