# ROCK Grab Quality Adaptive Held Response - 2026-05-09

This pass improves held-object feel without writing into unverified native spring internals. The native mouse spring remains the authority for the held body, while ROCK now shapes the target fed into that spring and composes release velocity from controller motion. That keeps the change FO4VR-native, testable in pure math, and reversible through INI keys if tuning exposes object-class-specific problems.

## Implemented

- Added an adaptive held lead layer before native spring target submission.
  - Uses controller-relative hand velocity to lead the native target by a bounded amount.
  - Adds bounded angular lead from controller angular velocity.
  - Scales the lead down when grab/contact error is already high so collision pressure does not turn into extra target overshoot.
  - Leaves native spring damping, tau, and constraint state untouched.

- Added controller-derived release velocity.
  - Tracks local hand velocity during held state.
  - Tracks hand angular velocity from controller rotation deltas.
  - Converts angular swing into tangential velocity around the held body's center of mass.
  - Blends object/body velocity history with hand velocity so throws follow real arm motion instead of only the lagging held rigid body.
  - Preserves player-space velocity compensation on release.

- Added thumb surface safety for alternate thumb local transforms.
  - Alternate thumb plane correction now only applies when the alternate thumb target came from a real alternate surface hit.
  - Thumb local correction is rejected when the thumb tip would be driven behind the contacted surface by more than the configured margin.
  - Shared local-transform callers were updated so generic and two-handed grip paths use the same safety contract.

## INI Keys

New `[Grab]` keys were added to both `data/config/ROCK.ini` and the active production INI at `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`:

- `bGrabAdaptiveHeldResponseEnabled = true`
- `fGrabAdaptiveHeldLeadTimeMax = 0.035`
- `fGrabAdaptiveHeldMaxLeadDistanceGameUnits = 8.0`
- `fGrabAdaptiveHeldMaxAngularLeadDegrees = 18.0`
- `fGrabAdaptiveHeldFullErrorGameUnits = 20.0`
- `fGrabAdaptiveHeldCollisionLeadScale = 0.25`
- `bGrabControllerDerivedThrowVelocityEnabled = true`
- `fGrabThrowObjectVelocityBlend = 0.35`
- `fGrabThrowTangentialVelocityScale = 1.0`
- `fGrabThrowMaxVelocityHavok = 12.0`
- `bGrabThumbSurfaceSafetyEnabled = true`
- `fGrabThumbSurfaceSafetyMarginGameUnits = 1.0`

## Verification

- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\ConfigDefaultParitySourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\HandGrabNativeBoundarySourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\PhysicsInteractionModuleBoundarySourceTests.ps1`
- `ctest --test-dir build -C Release --output-on-failure`: 55/55 passed.
- `$env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release -- /m:1`: passed and deployed `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.

The first parallel Release build compiled and deployed the plugin but hit MSVC `C1083` permission-denied races while building shared test object files. Re-running the build serially with `/m:1` completed cleanly, so the failure was build parallelism contention rather than a source error.

## Tuning Notes

- If held objects feel too eager or clip during fast swings, lower `fGrabAdaptiveHeldMaxLeadDistanceGameUnits` first, then `fGrabAdaptiveHeldLeadTimeMax`.
- If throws still land short, raise `fGrabThrowObjectVelocityBlend` cautiously only if the held rigid body has stable velocity. Otherwise raise `fGrabThrowTangentialVelocityScale` or the existing throw multiplier before making the spring more aggressive.
- If thumb pose becomes too conservative on curved objects, increase `fGrabThumbSurfaceSafetyMarginGameUnits` before disabling `bGrabThumbSurfaceSafetyEnabled`.

## Boundaries

- No Ghidra verification was required because this pass did not introduce new native offsets, struct-layout assumptions, or native internal writes.
- HIGGS remains historical context only for this change. The implementation uses ROCK's current native spring, FRIK hand data, and FO4VR coordinate/scale wrappers.
