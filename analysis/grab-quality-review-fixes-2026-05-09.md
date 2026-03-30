# ROCK Grab Quality Review Fixes - 2026-05-09

This pass fixes the review gaps by moving release behavior onto the same data path as held updates instead of adding release-only shortcuts. The controller/body motion sampler is now shared by normal held updates and explicit grip release, release velocities are capped after all throw composition, angular velocity has its own bounded policy, and thumb posing carries the real alternate-curve hit surface instead of reusing the generic finger target.

## Implemented Fixes

- Release-frame motion capture.
  - Added `Hand::captureHeldReleaseMotion`.
  - Normal grip release now samples current-frame controller linear/angular motion and object velocity before `releaseGrabbedObject`.
  - `Hand::updateHeldObject` and release now share `recordHeldControllerMotionSample` and `recordHeldObjectVelocitySample`.

- Final release velocity cap.
  - `composeControllerReleaseVelocity` now caps the final written linear velocity after local velocity, tangential swing, object blend, throw multiplier, and player velocity are composed.

- Bounded angular throw velocity.
  - Added `composeControllerReleaseAngularVelocity`.
  - Added `[Grab]` keys:
    - `fGrabThrowAngularVelocityScale = 1.0`
    - `fGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0`
  - Controller-derived angular override is skipped when controller-derived throw is disabled or the capped angular result is zero.

- Pivot-based tangential throw.
  - Tangential velocity now uses the live held-body grab pivot converted to Havok units when the body frame is readable.
  - Raw hand position remains only a fallback lever origin.

- Alternate thumb hit propagation.
  - `FingerCurlValue` now records front-valid hit point and hit normal.
  - The alternate thumb path requires a real alternate curve hit point before declaring `usedAlternateThumbSurfaceHit`.
  - Surface safety now tests the actual alternate thumb curve surface instead of the shared finger target when available.
  - Alternate thumb local correction now reports success only when it actually writes a transform.

## Updated Files

- `src/physics-interaction/grab/GrabHeldObject.h`
- `src/physics-interaction/grab/GrabFinger.h`
- `src/physics-interaction/hand/Hand.h`
- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/core/PhysicsInteraction.cpp`
- `src/RockConfig.h`
- `src/RockConfig.cpp`
- `data/config/ROCK.ini`
- `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini`

## Verification

- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\ConfigDefaultParitySourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\HandGrabNativeBoundarySourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\PhysicsInteractionModuleBoundarySourceTests.ps1`
- `$env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release -- /m:1`
- `ctest --test-dir build -C Release --output-on-failure`: 55/55 passed.

The Release build deployed `ROCK.dll` and `ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
