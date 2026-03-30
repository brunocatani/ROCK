# Grab Polish Implementation - 2026-05-11

## Why This Implementation Is Structured This Way

This implementation separates diagnostic cleanup from behavior polish because ROCK's current grab quality depends on multiple coupled systems: native mouse-spring drive, shared constraints, visual hand correction, acquisition gating, contact freshness, config defaults, and tests. The chosen approach keeps verified FO4VR-native object drive authority intact, improves the visual/diagnostic surfaces that made HIGGS comparison hard, and adds telemetry-first scaffolding for non-equipped dynamic weapon grabs without changing equipped weapon support or actor/ragdoll behavior.

## Decisions Locked

- Ordinary one-hand loose-object grabs remain `NativeMouseSpring`.
- Peer-held loose-object joins remain `SharedConstraint`.
- Non-equipped weapons are treated as loose active grab objects with weapon-specific telemetry and neutral multipliers.
- Equipped weapon support is not changed.
- Actor/ragdoll grabbing is not changed.
- Visual hand polish remains visual-only through FRIK external transform publication.
- Ghidra is not needed for this pass because no native offsets, native action layout, body flags, or binary behavior are changed.

## Implementation Checklist

- [x] Split native and shared-constraint diagnostics.
- [x] Rename stale active-grab diagnostic labels.
- [x] Rename displayed legacy contact-policy reason strings.
- [x] Clarify packaged and production INI comments in place.
- [x] Expand bounded acquisition visual start distance from 14.0 to 28.0 game units.
- [x] Add neutral non-equipped dynamic weapon tuning multipliers and telemetry.
- [x] Add/update source tests.
- [x] Run focused source tests.
- [x] Run focused Release build.

## Changes Made

- Native single-hand grab creation now logs `driveMode=nativeMouseSpring` and no longer exposes shared-constraint tau/force fields as active native tuning.
- Shared peer-held constraint grab creation now logs `driveMode=sharedConstraint` and keeps tau/damping/force fields there.
- Shared-constraint transform-B pivot diagnostics now use `[active-pivot-b]` instead of stale `[surface]`.
- Contact policy diagnostic strings now use `compatContactSources` and `permissiveFallback` instead of legacy wording.
- `fGrabAcquisitionVisualStartDistanceGameUnits` default is now `28.0`, matching the near-converge envelope while preserving the bounded pre-touch visual settle.
- Loose non-equipped weapon refs now get neutral config multipliers and telemetry through the normal active grab path. Equipped weapon support and actor/ragdoll paths were not changed.
- Packaged `data/config/ROCK.ini` and production `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini` were updated in place.

## Verification Log

- Passed: `tests/HandGrabNativeBoundarySourceTests.ps1`.
- Passed: `tests/SharedHeldObjectGrabSourceTests.ps1`.
- Passed: `tests/ContactEvidenceOwnershipSourceTests.ps1`.
- Passed: `tests/ConfigDefaultParitySourceTests.ps1`.
- Passed: `tests/GeneratedBodyDriveSourceTests.ps1`.
- Earlier broad build attempt hit MSBuild `.tlog`/`.obj` file locks during concurrent workspace activity.
- Earlier focused build attempt reported a stash compile blocker before grab code; current focused build was re-run after the tree settled.
- Passed current focused build: `cmake --build build --config Release --target ROCK -- /m:1`.
- The current focused build copied `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins` and packaged `build/package/ROCK - v0.1.0 - 20260511.7z` through the existing post-build step.
- Follow-up review is tracked in `analysis/grab-polish-code-review-2026-05-11.md`.

## Follow-Up Review Fixes - 2026-05-11

- Fixed native mouse-spring tuning telemetry so the logged response/clamp scales use the same sanitized composed values handed to the native cinfo.
- Added neutral loose non-equipped weapon shared-constraint tuning multipliers for future weapon-specific two-hand loose-object polish.
- Split packaged and production INI comments so shared-constraint motor tuning is not confused with native mouse-spring tuning.
- Passed the same focused source tests plus focused Release build after the fixes.
