# Ghidra Preflight — Runtime Repair Verification — 2026-04-22

## Why this file exists

Before relying on any new reverse-engineering work, the active Ghidra program needed to be sanity-checked against the expected FO4VR runtime address map. This note records the pre-flight checks used for the runtime review repair plan.

## Binary sanity checks performed

### 1. Segment layout check

Ghidra reported the following image layout:

- image base `0x140000000`
- `.text` at `0x140001000 - 0x142c40bff`
- `.rdata` at `0x142c4c000 - 0x1436f05ff`
- `.data` at `0x1436f1000 - 0x14689e1af`

This is consistent with the expected FO4VR executable mapping and with previously documented FO4VR addresses in the project.

### 2. Known wrapper function check

Raw address checked:

- `0x141E08A70`

Ghidra result:

- function exists as `FUN_141e08a70`
- body: `0x141e08a70 - 0x141e08af3`

Blind behavioral read:

- reads `param_1 + 0x20` as a physics-system-like object
- resolves a body ID through `FUN_141e0c460`
- resolves a world through `FUN_141e0c4e0`
- calls `FUN_141df55f0(world, bodyId, param_2, 1)`

This is consistent with the already documented `bhkNPCollisionObject::SetTransform` wrapper contract and confirms the loaded program matches the expected FO4VR runtime map in this area.

### 3. Known data/global check

Raw data address checked:

- `0x143718110`

Ghidra xrefs:

- reads from `0x1404dff01`
- reads from `0x1404f270c`
- reads from `0x1404f27d7`
- reads from `0x1429e58c8`
- reads from `0x1429e5a90`

This confirms the expected global exists in the loaded program and is referenced from live code, which is consistent with the known FO4VR address map used elsewhere in the project.

## Preflight verdict

**CONFIRMED**

The currently loaded Ghidra program is consistent with the expected FO4VR runtime address map used by ROCK. It is suitable for the runtime repair verification waves.
