# Created Body Stutter / Havok Timestep Implementation Progress

This work moves ROCK-generated hand and weapon collider movement out of the late game-frame transform-write path and into a Havok-step-aligned velocity drive. The problem being solved is stutter/lag on bodies ROCK creates or manipulates: generated hand colliders, generated weapon colliders, and dynamic held objects constrained to those generated hand bodies. The rejected alternatives were damping/tau tuning and larger velocity caps, because those treat the visible lag after it already exists. HIGGS drives keyframed collision bodies by keyframe velocity around the physics step, while FO4VR's binary exposes a keyframe wrapper that computes velocity from real Havok delta and only falls back to transform writes on limits/teleports. ROCK should match that architecture instead of writing transform plus velocity every frame from FRIK wall-clock frame time.

## Progress

- [x] Capture baseline source facts from current ROCK code.
- [x] Add regression tests that fail on unconditional generated-body transform writes and missing Havok timing source.
- [x] Add Havok physics timing sampling and generated keyframed body drive policy.
- [x] Move hand and weapon generated-body normal movement to step-aligned pending targets.
- [x] Build and run focused tests.
- [x] Do final code review against the implementation plan.

## Notes

- Web access is forbidden for this work.
- Ghidra is allowed by the user for FO4VR binary checks; code and Ghidra are the trusted sources for behavior.
- Existing dirty worktree changes are treated as user-owned and must not be reverted.
- Red test: `tests/GeneratedBodyDriveSourceTests.ps1` fails on the current direct `computeHardKeyFrame` plus transform/velocity frame-update path.
- Green source boundary: `tests/GeneratedBodyDriveSourceTests.ps1` now passes after adding Havok timing, shared generated-body drive policy, step listener registration, and pending-target flushes.
- Release build: `$env:VCPKG_ROOT='C:/vcpkg'; cmake --build build --config Release` succeeded and produced `build/Release/ROCK.dll`.
- Focused checks passed: generated body drive source boundary, Bethesda physics body source boundary, Havok runtime native boundary, and Release policy test executables in `build/Release`.
