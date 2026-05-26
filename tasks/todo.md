# Dynamic Grab PivotA Local Fix

- [x] Confirm current hidden-proxy offset source and the exact pivotA-zero runtime assumptions.
- [x] Change dynamic grab so the hidden proxy keeps its configured body-space offset instead of rebinding its origin to pivot A.
- [x] Compute and write transform-A pivot local from proxy body world to active pivotA world at grab create and held updates.
- [x] Update source-policy tests and authority-frame tests that currently require proxy-origin rebinding.
- [x] Build and run the relevant ROCK tests.

## Rationale

Current ROCK uses the hidden grab-authority proxy offset during startup capture, but once the grab goes live it moves the proxy body origin onto pivot A and keeps transform-A local pivot at zero. That defeats the configured seat offset and likely reintroduces the palm-origin dissatisfaction that the offset was meant to solve. The fix should keep the hidden proxy body at its offset frame and represent pivot A explicitly as a local point on body A.

## Review

- Runtime and policy/source-test edits are in place.
- Windows validation completed on 2026-05-26:
  - `cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m`
  - `cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
  - `ctest --test-dir build-tests -C Release --output-on-failure -R "GrabAuthority|HandGrabNativeBoundarySourceTests"`
  - `ctest --test-dir build-tests -C Release --output-on-failure -R "GrabPivotAuthority"`
  - `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- All of the above passed.
- Remaining validation is game/runtime verification from the checklist in `tasks/pivotA-local-fix-handoff-2026-05-26.md`.
