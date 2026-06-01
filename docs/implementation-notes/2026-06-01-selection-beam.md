# 2026-06-01 - Selection Beam Implementation Notes

Project: ROCK
Branch: feature/ghidra-grab-motor-mapping
Source authority: local ROCK source, local CommonLibF4VR/F4VR-CommonFramework headers, approved Ghidra inspection of Fallout4VR.exe.unpacked.exe.
Confidence: medium-high for architecture, high for avoiding vanilla grenade global state.

## Goal

Add a real in-game far-selection beam that can be disabled live from ROCK.ini.

## Renderer Decision

Ghidra inspection confirms the vanilla grenade trajectory path builds spline geometry through opaque curve/render buffers and grenade-specific global temporary reference state. Directly calling the vanilla update path would tie ROCK selection feedback to throwable weapon/perk state and global trajectory handles.

ROCK will instead own a small scenegraph effect per hand:

- fixed pool of NIF segment nodes cloned once per hand;
- no debug overlay renderer;
- no vanilla grenade trajectory update calls;
- no per-frame allocation after the first successful segment creation;
- endpoints come from ROCK's current far-selection state.

The pure beam math and defaults live in `SelectionBeamPolicy.h` so config parsing and policy tests do not include the scenegraph owner or heavy CommonLib renderer headers.

## Implementation Checklist

- [x] Add `SelectionBeamEffect` with pure policy helpers and runtime scenegraph ownership.
- [x] Add INI keys:
  - `bSelectionBeamEnabled`
  - `fSelectionBeamSegmentSizeGameUnits`
  - `fSelectionBeamCurveLiftGameUnits`
  - `fSelectionBeamAlpha`
- [x] Wire updates after normal hand selection refresh and hide immediately when selection clears, hand is disabled, grab starts, or config is turned off.
- [x] Package `Meshes/ROCK/selection_beam_segment.nif` as a ROCK-owned asset.
- [x] Add source-boundary and policy tests.

## Validation

- `cmake --preset custom-tests`: passed.
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`: passed.
- `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`: passed, 33/33.
- `ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%`: passed, 62/62.
- `cmake --preset custom-fast`: passed.
- `cmake --build build-fast --config Release --target ROCK -- /m`: passed and auto-deployed `ROCK.dll`, `ROCK.pdb`, and `Meshes/ROCK/selection_beam_segment.nif`.
