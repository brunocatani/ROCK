# Grab Angular Authority Implementation Tracker - 2026-05-14

Purpose: track the active implementation so context compaction does not lose
the exact scope, rationale, files touched, and verification state.

## Current Objective

Implement the researched fix for custom dynamic grab rotation/world-direction
dependency, then review and build. Scope is rotation/angular authority only.

## Rationale

The N/S/E/W issue is caused by an unproven angular-vector contract in the
custom proxy grab path. ROCK's grab transforms use row-local axes, while the
current custom angular velocity producers derive public hknp angular vectors
from stored columns. Ghidra confirmed FO4VR `SetBodyAngularVelocity` and
generated-body `setVelocity` both expect a public hknp world angular vector and
then rotate it internally through hknp motion orientation.

The implementation should therefore stop hand-rolling the public hknp angular
vector from raw matrix columns. FO4VR already exposes `ComputeHardKeyFrame` at
`0x14153A6A0`; Ghidra confirmed its angular output feeds directly into
`SetBodyVelocity`, so it is the correct native boundary/oracle for angular
velocity convention.

## Files Intended To Touch

- `src/physics-interaction/hand/HandGrab.cpp`
- `src/physics-interaction/grab/GrabAuthorityProxyMotion.h` if proxy angular
  helper remains local, or replace its use from `HandGrab.cpp`
- this tracker file

No pivot, COM, collision layer, generated collider convention, loose-weapon
activation, equipped weapon, or actor ragdoll changes are intended.

## Implementation Shape

1. Add a local helper that calls FO4VR `ComputeHardKeyFrame` using:
   - live hknp body id;
   - target position in Havok units;
   - target quaternion from `niRowsToHavokQuaternion`;
   - current physics dt;
   - output linear/angular vectors.
2. Use the helper for held-object angular velocity in
   `applyProxyConstraintAngularVelocityDrive`.
3. Use the same helper for hidden proxy angular velocity in
   `flushPendingCustomGrabAuthority`, so proxy body and held object use one
   angular-vector convention.
4. Preserve ROCK finite-force behavior by applying the existing angular speed
   cap after the native angular vector is produced.
5. Preserve the current custom linear constraint, contact/body pivot, and mass
   force budget.

## Current Status

- [x] Research saved in
  `docs/grab-custom-integration-rotation-failure-map-2026-05-13.md`.
- [x] Active branch checked: `feature/ghidra-grab-motor-mapping`.
- [x] Dirty unrelated doc deletions observed; leave them untouched.
- [x] Implement hard-keyframe angular helper.
- [x] Route held-object direct angular velocity through helper.
- [x] Route proxy body angular velocity through helper.
- [x] Review for compile/runtime risks.
- [x] Build Release.
- [x] Commit focused implementation if safe.

## Implementation Notes

- Added `computeHardKeyframeVelocityForTarget(...)` in `HandGrab.cpp`.
- Held-object angular drive now uses the FO4VR hard-keyframe angular vector for
  `desiredBodyWorld`, then applies the existing ROCK angular speed cap.
- Hidden proxy angular velocity now uses the same hard-keyframe angular vector
  for `pending.proxyWorld`; proxy linear velocity still uses the existing
  target delta because the linear side was not the bug.
- Log diagnostic labels now identify the mode as
  `bodyFrameConstraint+hardKeyframeAngularVelocity`.
- Updated `tests/GrabAuthorityProxyFrameSourceTests.ps1` so the policy test
  requires the hard-keyframe angular boundary and rejects the old proxy
  matrix-column angular delta helper.

## Verification Results

- Release build succeeded:
  `cmake --build build --config Release`
- Build output deployed `ROCK.dll` and `ROCK.pdb` to
  `D:/FO4/mods/ROCK/F4SE/Plugins/`.
- Targeted proxy frame source test passed:
  `ctest --test-dir build -C Release --output-on-failure -R GrabAuthorityProxyFrameSourceTests`
- Full registered test suite passed:
  `ctest --test-dir build -C Release --output-on-failure`
  - 21/21 passed.

## Verification Gates

- Build must complete with:
  `cmake --build build --config Release`
- Source review must confirm:
  - no COM/pivot authority change;
  - no collider convention change;
  - proxy and held object use the same angular-vector boundary;
  - angular caps only limit magnitude, not axis direction;
  - current finite-force/mass policy remains in place.
