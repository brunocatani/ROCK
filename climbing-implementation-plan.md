# ROCK Contact-Driven VR Climbing Plan

## Summary

- Implement ROCK-native climbing without HIGGS contacts or a HIGGS dependency. VR-Climbing uses HIGGS only for compatibility/input suppression, not for climbing contact detection.
- Use contact-only latching from ROCK's native hand collision/contact stream. No raycast, probe, or shape-cast fallback in the climb subsystem.
- First implementation scope: contact latch, one/two-hand climbing, verified native player motion, release velocity, fall/auto-catch, haptics, debug, and optional AP drain.
- Free hands only: climbing cannot start while that hand is holding, pulling, locked, looting, stashing, consuming, two-handing, or weapon-owned.

## Interfaces And Boundaries

- Add an internal `src/physics-interaction/climb/` subsystem:
  - `ClimbRuntime`: owns per-hand latch state and global climb/fall/auto-catch state.
  - `ClimbContactEvidenceCache`: stores bounded native hand-surface contact records from the contact callback.
  - `ClimbSurfacePolicy`: accepts only configured climbable support layers and valid normals.
  - `ClimbMotionPolicy`: pure math for hand delta, two-hand averaging, smoothing, clamps, and release velocity.
- Expand `src/physics-interaction/native/CharacterControllerRuntime.*` or add `PlayerLocomotionRuntime.*`; no raw offsets or direct native calls in climb code.
- Do not change ROCK provider/public ABI for v1. Keep all climb state internal except optional debug overlay data.
- Add INI keys, default off for safe rollout:
  - `bClimbEnabled=false`
  - `iClimbContactMaxAgeFrames=2`
  - `iClimbContactGraceFrames=4`
  - `fClimbSmoothingSpeed=13.0`
  - `fClimbMaxLaunchSpeedGameUnits=600.0`
  - `fClimbMinLaunchSpeedGameUnits=5.0`
  - `fClimbVelocityHistorySeconds=0.10`
  - `bClimbAutoCatchEnabled=true`
  - `bClimbActionPointDrainEnabled=false`
  - `bDebugDrawClimbContacts=false`

## Implementation Changes

- Native verification gate:
  - Before coding movement, request explicit approval for each Ghidra operation per `AGENTS.md`.
  - Verify `bhkCharacterController::Move`, `GetPosition`, `SetPosition`, velocity set/get, and gravity/support handling.
  - Document results in `analysis/climb-player-locomotion-native-verification-YYYY-MM-DD.md`.
  - Use verified controller `Move` for frame deltas; use velocity APIs for zeroing while latched and launch on release. If velocity/gravity cannot be verified, stop rather than shipping an actor-position workaround.
- Contact evidence:
  - Record climb evidence directly from native hand contact callbacks when source is a ROCK hand and target layer passes climb surface policy.
  - Keep this cache separate from soft contact's `_nativeContactEvidence` so climbing can support more surface layers without changing normal soft-contact behavior.
  - Use only fresh raw contact points/normals. Reject dynamic props, actors, held objects, ROCK layers, query-only layers, and invalid body IDs.
  - Default climbable layers should mirror player support/world blockers, excluding clutter and actors. Flat ground latching stays disabled by default unless explicitly configured later.
- Climb runtime:
  - On grip press, latch only if the hand is free and has fresh climb contact evidence.
  - Store latch hand, target body/layer, contact point, oriented normal, raw hand transform, and player/controller position.
  - While latched, keep generated hand collision active so contact evidence continues.
  - Release that hand when grip is released, evidence is missing beyond grace frames, menu/world state becomes invalid, or native movement fails.
  - With two latched hands, average the two hand-derived movement deltas and reset baselines when either hand joins/leaves to prevent velocity spikes.
  - Compute release velocity from actual applied player/controller displacement samples, not raw hand motion alone.
- Soft contact coexistence:
  - Add a climb suppression mask to `SoftContactRuntime::update`.
  - For latched hands, soft contact clears only its external FRIK hand transform and haptic edge state.
  - Do not invalidate native/climb contact evidence and do not put the hand into existing grab-owned suppression states.
  - Non-latched hands keep normal soft contact behavior.
- Input and grab arbitration:
  - Replace one-off `consumeRawButtonState` use with a per-frame grab input sample shared by climbing and normal object grab.
  - Climbing consumes the hand's grab input only when it latches or remains latched.
  - Normal object grab receives cleared input for climb-owned hands and resets grab intent state.
  - If a free hand has both object selection and valid climb contact on the same press, climbing wins and transient selection is cleared.
  - Add external game-facing button suppression for climb-owned grip buttons so the game does not also consume active climbing input.
- Player motion:
  - While anchored, zero/suspend controller velocity through verified native wrappers and move the controller by bounded substepped deltas.
  - Clamp per-frame motion and use controller-resolved actual movement for velocity history.
  - On release, apply verified launch velocity if speed exceeds threshold; otherwise leave native gravity/fall behavior active.
  - Auto-catch only works when grip is held and fresh contact-only climb evidence appears during the fall/launch window.

## Test Plan

- Add C++ policy tests for surface layer/normal acceptance, contact freshness, free-hand gating, two-hand delta math, release velocity filtering, and auto-catch state transitions.
- Extend input tests for external per-hand grip suppression while preserving raw ROCK input.
- Add source-boundary tests:
  - climb code must not reference HIGGS, raycasts, `PhysicsShapeCast`, or raw REL/native offsets.
  - controller movement calls must live only in the native runtime boundary.
  - climb update must run before normal grab input and pass a climb arbitration mask into grab/soft-contact updates.
- Add soft-contact coexistence tests/source checks:
  - latched hands suppress soft-contact visual correction only.
  - climb does not invalidate native contact evidence.
  - non-latched hands still receive normal soft contact.
- Verify with Release build, full test suite, and in-game scenarios: wall latch, two-hand climb, object-selection conflict, held-item blocked latch, weapon-owned blocked latch, menu cleanup, release launch, auto-catch, and normal soft contact when climbing is disabled.

## Assumptions

- ROCK architecture wins over HIGGS; HIGGS contact callbacks are not used.
- First pass is contact-only by design, even if ray/probe fallback would make climbing more forgiving.
- Climbing is config-disabled by default until playtested because it changes player locomotion.
- If native player velocity/gravity behavior cannot be verified, implementation pauses instead of degrading to direct actor teleport movement.
