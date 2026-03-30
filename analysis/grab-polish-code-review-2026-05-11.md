# Grab Polish Code Review - 2026-05-11

## Scope

Reviewed the current grab polish pass in the dirty ROCK worktree, focused on ordinary one-hand native mouse-spring grabs, peer-held shared constraints, non-equipped loose weapon dynamic grab scaffolding, diagnostic cleanup, config exposure, and validation coverage. Equipped weapon support and actor/ragdoll grabbing remain intentionally unchanged.

## Findings

### Medium - Native tuning telemetry can disagree with actual cinfo scales when multipliers exceed the native wrapper clamp

- `src/physics-interaction/hand/HandGrab.cpp:3977` multiplies the base native mouse-spring scale by the loose-weapon multiplier.
- `src/physics-interaction/hand/HandGrab.cpp:4026` logs that multiplied value as `nativeLinearScale`/`nativeAngularScale`/`nativeClampScale`.
- `src/physics-interaction/native/NativeMouseSpringGrab.cpp:68` clamps `tuning.*Scale` to `0.05..4.0` again when writing cinfo at lines 154-156.
- `src/RockConfig.cpp:1005`, `1016`, and `1027` clamp the individual multipliers to `4.0`, but the final product is not normalized before logging or before handoff.

Impact: defaults are neutral and safe, but tuned loose-weapon values can produce logs such as `nativeLinearScale=5.40` while the actual native cinfo receives `4.00`. That makes in-game tuning harder because the log reports intent, not applied drive strength.

Recommended fix: sanitize the final composed native tuning values once in the hand layer or expose the applied sanitized values from `NativeMouseSpringGrab::create`/debug state, then log the same values that are written to the cinfo.

### Medium - Loose weapon tuning exists only for the native one-hand drive path

- `src/physics-interaction/hand/HandGrab.cpp:3963` applies loose-weapon native multipliers only when the drive mode is `NativeMouseSpring`.
- Peer-held joins log `looseWeapon=yes/no` at `src/physics-interaction/hand/HandGrab.cpp:4034`, but shared-constraint tau/damping/force behavior remains generic.

Impact: this is consistent with the telemetry-first decision and avoids touching equipped weapon support, but it means two-hand loose weapon grabs still cannot be tuned separately from normal loose objects. If loose rifles or long guns feel good one-handed but over/under-damped when the second hand joins, there is no dedicated shared-constraint tuning surface yet.

Recommended fix: add neutral loose-weapon shared-constraint multipliers or a weapon-class drive profile scaffold before changing any non-neutral behavior.

### Low - Config comments still mix shared-constraint and native mouse-spring concepts in one section

- `data/config/ROCK.ini:465` documents native mouse-spring settings.
- The same `Grab Drive Feel` section also contains shared-constraint tau/damping/force at lines 443-463.

Impact: the comments are better than before, but troubleshooting can still confuse "linear motor response" tau with native mouse-spring response scale. This was one of the original causes of grab-quality tuning confusion.

Recommended fix: split the section comments into explicit `Shared constraint drive` and `Native mouse-spring drive` blocks in both packaged and production INIs, without changing values.

### Low - Source tests verify presence, not the composed tuning boundary

- `tests/HandGrabNativeBoundarySourceTests.ps1:98` checks that native grabs reference configured response scales.
- `tests/HandGrabNativeBoundarySourceTests.ps1:104` checks loose weapon detection exists.
- No test currently asserts that composed native scale logging matches the sanitized native cinfo boundary.

Impact: source boundary tests catch accidental architecture regressions, but they would not catch the current final-product clamp mismatch or future log/drive divergence.

Recommended fix: add a small pure policy helper for composing native tuning scales and unit-test the clamp/log contract without needing RE runtime objects.

## Verification

- Passed: `tests/HandGrabNativeBoundarySourceTests.ps1`.
- Passed: `tests/ConfigDefaultParitySourceTests.ps1`.
- Passed: `tests/SharedHeldObjectGrabSourceTests.ps1`.
- Passed: `tests/ContactEvidenceOwnershipSourceTests.ps1`.
- Passed earlier in this pass: `tests/GeneratedBodyDriveSourceTests.ps1`.
- Passed current focused build: `cmake --build build --config Release --target ROCK -- /m:1`.
- The focused build copied `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins` and packaged `build/package/ROCK - v0.1.0 - 20260511.7z` through the existing post-build step.

## Notes

- The worktree contains many unrelated dirty and untracked changes. This review did not revert or normalize them.
- The prior implementation tracker still records an earlier stash compile failure. That is stale for the current focused ROCK target; the current focused build passed.

## Fix Plan

This fix keeps the current FO4VR-native drive split instead of replacing it with a new grab authority path: one-hand loose object grabs stay on the native mouse spring, peer-held joins stay on the shared constraint, and non-equipped weapons get neutral profile scaffolding on both paths. The native-scale fix composes the final applied values before handoff so logs, tests, and cinfo writes describe the same tuning. The shared-constraint additions are intentionally neutral now, but they give later weapon-specific work a stable config surface without touching equipped weapon support or actor/ragdoll grabbing.

## Fix Implementation

- Fixed native mouse-spring scale drift by adding `native_mouse_spring_grab::composeTuningScale`/`sanitizeTuningScale` and using the composed applied values for both cinfo handoff and creation telemetry.
- Added `GrabConstraintMotorTuning` so shared-constraint creation can receive a full linear/angular motor profile instead of only linear tau/damping plus global angular defaults.
- Added neutral non-equipped loose-weapon shared-constraint multipliers for linear/angular tau, collision tau, linear/angular damping, max force, angular force, and linear/angular recovery.
- Applied loose-weapon shared-constraint multipliers at both initial peer-held constraint creation and per-frame adaptive motor updates.
- Split packaged and production INI comments into shared-constraint vs native mouse-spring drive blocks, and added the new neutral loose-weapon shared-constraint keys in place.
- Updated source boundary tests for the native applied-scale helper and loose-weapon shared-constraint config scaffold.

## Fix Verification

- Passed: `tests/HandGrabNativeBoundarySourceTests.ps1`.
- Passed: `tests/ConfigDefaultParitySourceTests.ps1`.
- Passed: `tests/SharedHeldObjectGrabSourceTests.ps1`.
- Passed: `tests/ContactEvidenceOwnershipSourceTests.ps1`.
- Passed: `tests/GeneratedBodyDriveSourceTests.ps1`.
- Passed: `git diff --check` on touched tracked files; only existing CRLF conversion warnings were reported.
- Passed: `cmake --build build --config Release --target ROCK -- /m:1`.
- The Release build copied `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins` and packaged `build/package/ROCK - v0.1.0 - 20260511.7z` through the existing post-build step.
