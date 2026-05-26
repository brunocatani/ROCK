# PivotA Local Fix Handoff

## Summary

Implemented the minimal hidden-proxy fix for dynamic grab:

- the hidden proxy keeps its configured runtime seat offset
- the proxy body origin is no longer rebound to pivot A at grab commit or held runtime
- transform A now carries pivot A as an explicit local point on body A

This is the narrow fix you asked for instead of the larger `_handBody` Body-A conversion.

## Files Changed

- `src/physics-interaction/hand/HandGrab.cpp`
- `tests/GrabAuthorityFramePolicyTests.cpp`
- `tests/GrabAuthorityProxyFrameSourceTests.ps1`
- `tests/HandGrabNativeBoundarySourceTests.ps1`
- `tasks/todo.md`
- `tasks/lessons.md`

## What Changed

### Runtime

`src/physics-interaction/hand/HandGrab.cpp`

- Removed proxy-origin rebinding helper usage.
- Constraint creation now uses:
  - `constraintPivotAWorld = grabPivotAWorld`
  - `pivotAProxyLocalGame = computeConstraintPivotLocalGame(proxyWorldTransform, grabPivotAWorld)`
- Held updates now recompute transform-A local pivot from the current proxy body world and current pivot-A world.
- Grab freeze no longer rewrites `proxyFrameWorldAtGrab.translate` to the selected pivot.
- Held runtime no longer rewrites `proxyAuthorityWorld` to the selected pivot.
- Between-step flush now keeps `pending.proxyWorld = applyRuntimeGrabAuthorityProxyOffsetToFrame(...)` instead of overwriting translation with pivot A.
- Seated pivot repromotion now stores `frozenSeatAuthorityFrame.pivotAHandBodyLocalGame` instead of resetting A-local to zero.

### Tests

- `tests/GrabAuthorityFramePolicyTests.cpp`
  - updated the authority-frame test to use a non-origin proxy frame
  - added an assertion that `pivotAHandBodyLocalGame` stays nonzero

- `tests/GrabAuthorityProxyFrameSourceTests.ps1`
  - updated regex expectations from proxy-origin rebinding to explicit A-local pivot math

- `tests/HandGrabNativeBoundarySourceTests.ps1`
  - updated regex expectations for separate proxy frame + pivot A freeze and nonzero promoted A-local pivot

## Validation Blocked On Linux

Both preset-based builds fail on Linux because CMake cannot create the Windows-only generator:

```text
Visual Studio 17 2022
```

## Next Steps On Windows

Run from `ROCK/`:

### 1. Runtime build

```bat
cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
```

### 2. Test build

```bat
cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
```

### 3. Relevant tests

First list matching tests if needed:

```bat
ctest --test-dir build-tests -C Release -N
```

Then run the authority/pivot-related ones:

```bat
ctest --test-dir build-tests -C Release --output-on-failure -R "GrabAuthority|HandGrabNativeBoundarySourceTests"
```

If you want broader coverage after that:

```bat
ctest --test-dir build-tests -C Release --output-on-failure -L source-boundary
```

## What To Watch For

### Compile/runtime hotspots

- `HandGrab.cpp`
  - transform-A local pivot write path around `updateProxyConstraintGrabDriveTarget(...)`
  - grab creation path around `createProxyConstraintGrabDrive(...)`
  - freeze path around `freezeGrabAuthorityFrame(...)` call site
  - held proxy flush path around `pending.proxyWorld = applyRuntimeGrabAuthorityProxyOffsetToFrame(...)`

### Behavioral checks in game

- confirm the hidden proxy keeps the `-2.0 Y` seat offset during held grab
- confirm the selected grip point still lands on the intended hand point
- confirm no return of the palm-pocket orbit bug
- confirm no on-grab snap/top-grab angular regression
- confirm left-hand runtime offset still behaves correctly
- confirm seated palm-pocket promotion still works

## Notes

- `tests/GrabAuthorityFramePolicyTests.cpp` shows a large diff on Linux because `apply_patch` likely normalized line endings while changing one test case. If you want a smaller final diff before commit, re-save that file with the repo's expected Windows line endings while keeping the new non-origin proxy assertion.
- No commit was made.

## Windows Continuation

Validation was completed on Windows on 2026-05-26.

### Commands Run

```bat
cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m
cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release --output-on-failure -R "GrabAuthority|HandGrabNativeBoundarySourceTests"
ctest --test-dir build-tests -C Release --output-on-failure -R "GrabPivotAuthority"
ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%
```

### Results

- `custom-fast` build passed and auto-deployed via the preset.
- `custom-tests` build passed.
- Targeted authority/native-boundary tests passed.
- `ROCKGrabPivotAuthorityPolicyTests` passed.
- Full `source-boundary` label run passed.

### Remaining Gaps

- The optional follow-up policy coverage suggested for `tests/GrabAuthorityProxyPolicyTests.cpp` was not added.
- The in-game behavioral checks listed above have not been run from this workspace session.
