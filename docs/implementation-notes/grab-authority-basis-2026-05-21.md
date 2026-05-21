# Grab Authority Basis - 2026-05-21

Project: ROCK

Source authority: current ROCK source, current production/preset INI evidence, and live user validation. No web, Ghidra, FO4 Mods MCP, or external reference source used.

Confidence: source-validated; runtime live test still required for final feel.

## Finding

Dynamic grab pivot placement was already correct when both hands used the same generated palm proxy offset, including `Y=-2.0` on left and right. Palm-pocket and pinch code were still partly using the legacy authored handspace helper where `Y` and `Z` are swapped relative to the generated palm proxy frame.

That created a split authority model: the hidden proxy/pivot moved through generated palm proxy local space, while the palm pocket and pinch detection could resolve their palm/cross-palm axes from legacy authored handspace.

## Decision

Dynamic grab now treats generated grab authority proxy space as the grab basis:

- `X`: fingers.
- `Y`: palm depth.
- `-Y`: palm face for both hands.
- `+Y`: back of hand for both hands.
- `Z`: across palm.

Palm pocket construction, close selection palm normal, seated support refresh, seated reacquire, pinch detection, and grab-pocket debug markers use this same generated proxy basis. The legacy authored handspace conversion remains only for non-grab/fallback callers that still explicitly use it.

## Validation

- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release --output-on-failure -j $env:NUMBER_OF_PROCESSORS`

All 44 tests passed.
