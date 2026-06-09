# Held Object Fingertip Pad Probe Enrichment Plan

Date: 2026-06-09

Project: ROCK

Scope: Held object grab hand-pose enrichment for FO4VR

Status: Planning note for the next implementation pass

Source authority: current ROCK local source, current hFRIK API contract, current local build/test behavior

External sources: none

Related diagrams:

- `hand-pose-splay-source-diagram.svg`
- `grab-finger-probe-target-diagram.svg`

Related recent commits:

- `170a299 feature/hand-pose: enrich mesh pose splay`
- `45fbeae docs/hand-pose: add splay source diagram`
- `37a3513 docs/grab-finger: add probe target diagram`

## 1. Problem Statement

Some held objects need very little finger curl, or even an open/negative-curl visual relation, depending on where the object is seated in the hand.

The current mesh finger pose solver can still curl fingers into the object in these cases. The result can look and feel unnatural because the solver primarily answers this question:

```text
How far can this finger curl before hitting the object?
```

That question is useful for wrapping around objects, but it is incomplete for light contact, broad surfaces, thin panels, handles, or objects held at a point where the natural pose should be mostly open.

The missing second question is:

```text
Is the fingertip/pad already on, near, or inside the object surface, and should the finger open back up instead of curling more?
```

The next improvement should add fingertip/pad surface probes as a second evidence channel for held object grabs.

## 2. Current Behavior Summary

Current object-grab finger pose logic uses a five-finger mesh solver.

Each solved pose has:

- `values[5]`, one scalar curl per finger.
- `jointValues[15]`, proximal/middle/distal values for thumb/index/middle/ring/pinky.
- `probeStart[5]` and `probeEnd[5]`, the debug-visible solver probe lines.
- `surfaceAimTarget[5]` and `surfaceAimNormal[5]`, accepted per-finger surface targets when valid.
- Object-local copies of surface targets/normals so targets can follow the held object.
- Thumb alternate curve fields for thumb-specific opposition correction.

The current debug probe lines are real solver data:

```text
probeStart[finger] -> probeEnd[finger]
```

They are copied from `fingerPose.probeStart` and `fingerPose.probeEnd` into `_grabFingerProbeStart` and `_grabFingerProbeEnd`.

The debug probe line is not necessarily the final target point used for local-transform correction or splay.

The final target point is:

```text
surfaceAimTarget[finger]
```

This target exists only when `surfaceAimTargetValid[finger] != 0`.

## 3. Current Probe Sources

Current five-finger probe construction happens in `solveGrabFingerPoseFromTriangles(...)` in `src/physics-interaction/grab/GrabFinger.h`.

For each finger, ROCK uses the live root-flattened finger skeleton.

The live skeleton source is:

```text
root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(...)
```

The solver derives:

- finger base position from the current live finger chain.
- finger open direction from the current live finger chain.
- palm normal from the current root-flattened hand basis.

For each finger, current broad mesh solving does roughly this:

```text
baseWorld = liveFingerBase + optionalPalmSeatProbeShift
toContact = explicitFingerTarget - baseWorld, if explicit target exists
toContact = openDirectionWorld, otherwise
probeDirection = normalized(toContact)
probeDistance = clamp(distanceToContact + padding, 6, 26)
probeStart[finger] = baseWorld
probeEnd[finger] = baseWorld + probeDirection * probeDistance
```

Constants currently used by the mesh finger solver:

```text
kFingerReachPadding = 6.0 game units
kMinFingerProbeDistance = 6.0 game units
kMaxFingerProbeDistance = 26.0 game units
kFingerProbeRadius = 1.25 game units
```

The debug-visible probes are therefore a real view into the current solver path, but they are not the whole contact model.

## 4. Current Target Sources

`surfaceAimTarget[finger]` can come from accepted mesh/curve evidence.

For normal broad mesh grabs:

- ROCK uses the held object's triangle set.
- The solver evaluates finger curves and fallback rays against candidate triangles.
- Valid front-facing accepted hits can become `surfaceAimTarget[finger]`.
- If no accepted hit/target exists, that finger has no surface aim target.

For pinch-pocket grabs:

- ROCK builds explicit thumb/index targets around the pinch surface.
- The explicit targets are derived from the pinch pocket surface hit and pinch axis.
- Thumb and index target data is more intentional than broad mesh probing.

For object movement:

- On grab, ROCK calls `captureSurfaceAimObjectLocal(...)`.
- This stores valid `surfaceAimTarget` and `surfaceAimNormal` values in object-local space.
- During held updates, ROCK calls `resolveSurfaceAimObjectLocal(...)` against the current held-object transform.
- That rebuilds current world-space targets as the object moves.

This means target data can follow the object after grab, but only for fingers that already have valid target data.

## 5. Why The Current Solver Can Curl Into Objects

The current base/curl probes are good at answering close-until-contact behavior.

They are weaker when the natural pose should be light contact or open contact.

Failure cases include:

- Object surface is already under a fingertip pad.
- The best visual pose is nearly open.
- The selected grip point causes a curl probe to cross through object volume.
- A broad mesh solve has no explicit per-finger targets and falls back toward a curled visual value.
- The accepted surface target is on the wrong side for visual pad contact.
- The finger local-transform correction aims toward a target but the coarse curl has already over-closed.

The important current limitation is this:

```text
The solver knows how to close toward geometry, but it does not yet have a strong held-state signal for opening away from geometry when the fingertip/pad is already contacting or penetrating the object.
```

## 6. Proposed Improvement

Add a secondary fingertip/pad probe channel for held object grabs.

This should not replace the current five base/curl probes.

The current probes answer:

```text
How much should the finger curl from an open pose?
```

The new pad probes should answer:

```text
Where is the surface relative to the current fingertip/pad, and should this finger remain open or open back up?
```

The system should use both signals:

- Base/curl probes keep coarse curl wrapping behavior.
- Fingertip/pad probes provide anti-penetration and surface-settling evidence.
- Pad evidence can improve `surfaceAimTarget` when it is better than the original curl probe hit.
- Pad evidence can bias curl more open when closing would put the finger inside the object.
- Pad evidence can improve splay and local-transform correction because both consume surface targets.

## 7. Proper Trigger Stage

The first implementation should run pad probes only after the object-hand relation is stable.

Recommended trigger:

```text
TouchHeld and later held updates only.
```

Do not run pad probes during:

- Selection.
- Far pull.
- Early near-converging.
- Gravity pulling.
- Unseated acquisition.

Reason:

- Before `TouchHeld`, the object is still moving into the hand relation.
- Early pad probes can read the wrong side of the object.
- Early pad probes can introduce noisy open/close changes during acquisition.
- At `TouchHeld`, ROCK already has a meaningful palm pocket, grip evidence point, object transform, and live finger relation.

Recommended first-stage state behavior:

- At `TouchHeld` transition, run the normal mesh curl solve first.
- Capture normal object-local surface targets as today.
- Run the first fingertip/pad probe refinement pass.
- Use pad evidence to open fingers back up if the pad is already on or inside the surface.
- During `HeldInit` and `HeldBody`, re-run pad refinement on the same cadence as existing finger pose updates.
- Clear pad-probe state on release, reacquire, provider invalidation, or mesh pose clear.

Possible future stage:

- Allow a weak late-acquisition pad pass in `NearConverging` only when the object is already inside touch range, pivot authority normal is trusted, live finger snapshot is valid, and the object transform is stable.
- This should not be part of the first implementation.

## 8. Data Sources For Pad Probes

Pad probes should use existing local runtime data.

Primary sources:

- Current root-flattened finger skeleton snapshot.
- Current held object transform.
- Current object-local mesh triangles rebuilt to world space.
- Current grip evidence point and grip normal.
- Current palm pocket / palm seat point.
- Existing `surfaceAimTarget` values, when valid.

The source should not be HIGGS, external authored tables, or assumptions from general modding behavior.

## 9. Pad Probe Start Points

The safest initial pad point is derived from live finger chain points.

Candidates:

- Distal pad center: midpoint of finger chain points 1 and 2.
- Fingertip point: finger chain point 2.
- Slightly backed-off pad center along the finger chain to avoid starting past thin geometry.

Recommended first version:

```text
padCenter = midpoint(chain.points[1], chain.points[2])
```

Reason:

- It is derived from currently available live skeleton data.
- It avoids relying on unknown local bone axes.
- It should approximate the visible distal pad better than the proximal base point.

Avoid using raw bone local `-Y` as the first implementation.

Reason:

- The finger bone local axis convention must be verified for both hands and all fingers.
- FRIK/hFRIK, root-flattened bones, and game bone transforms may not share an intuitive local-axis convention.
- A wrong local axis would make pad probes point sideways or through the finger.

Local bone axes can be evaluated later using a debug axis overlay.

## 10. Pad Probe Directions

The pad direction should be stable and derived from current contact context.

Recommended direction priority:

1. If current `surfaceAimTarget[finger]` is valid, use direction from pad center to that current target.
2. Otherwise, use direction from pad center to the current grip evidence point or palm seat point.
3. Blend with palm normal only if it improves stability after runtime visual inspection.
4. Do not use raw local bone `-Y` until local axes are verified.

The first implementation should prefer target-driven direction over bone-axis-driven direction.

Reason:

- Existing target/grip evidence already follows the held object.
- Target-driven direction is less likely to invert between hands.
- Target-driven direction fits the current object-local target architecture.

## 11. Pad Contact Witness Method

A simple ray alone may not be enough.

If the fingertip pad is already inside the object, a one-direction ray can fail or return the wrong exit face.

Recommended contact witness method:

- Use a short ray/sphere probe from the pad center toward the target direction.
- Also compute closest surface point from nearby object triangles to the pad center.
- Prefer the closest valid surface point when it is within a small max pad distance and passes normal/owner checks.
- Use ray hit when it provides a better front-facing surface witness.

This makes the pad probe more of a local surface witness than a pure ray cast.

Suggested initial distances:

```text
padProbeRadius = 0.75 to 1.25 game units
padProbeDistance = 4.0 to 8.0 game units
padClosestSurfaceMaxDistance = 3.0 to 6.0 game units
```

These values should be configurable or easy to tune.

## 12. Pad Probe Evidence Structure

Add a small fixed-size runtime evidence structure.

Suggested shape:

```cpp
struct FingerPadSurfaceEvidence
{
    RE::NiPoint3 startWorld{};
    RE::NiPoint3 endWorld{};
    RE::NiPoint3 hitPointWorld{};
    RE::NiPoint3 hitNormalWorld{};
    float distanceGameUnits = 0.0f;
    float quality = 0.0f;
    bool hit = false;
    bool fromClosestSurface = false;
    bool padMayBeInsideSurface = false;
};
```

Use fixed arrays:

```cpp
std::array<FingerPadSurfaceEvidence, 5>
```

Avoid heap allocation in per-frame held update paths.

## 13. Validity Guards

Pad probe evidence should be accepted only when all necessary state is valid.

Required guards:

- ROCK mesh finger pose is enabled.
- Object is in held state, preferably `TouchHeld` or later.
- Held object body and current object transform are available.
- Live root-flattened finger snapshot is valid.
- Candidate object triangles exist and belong to the held object relation.
- Pad evidence is finite.
- Hit normal is usable when normal-based rejection is needed.
- Surface witness does not obviously point to the wrong side of the object.

Recommended rejection reasons:

- `disabled`
- `not-touch-held`
- `missing-live-finger-snapshot`
- `missing-object-transform`
- `missing-triangles`
- `no-pad-hit`
- `bad-normal`
- `backside`
- `owner-mismatch`
- `too-far`

These reasons should be exposed through debug logging or overlay text if practical.

## 14. How Pad Evidence Should Affect Curl

The first implementation should only open fingers, not force extra closure.

Current hFRIK/ROCK convention:

```text
0.0 = closed/bent
1.0 = open/straight
```

Recommended first behavior:

```text
if pad evidence says the fingertip is on/inside/too close to object surface:
    increase that finger's curl value toward open
```

Do not use pad evidence to make fingers more closed in the first version.

Reason:

- The current base/curl solver already handles closure.
- The failure being targeted is over-curl and penetration.
- Opening-only refinement is easier to reason about and safer to roll back.

Initial output should stay in normalized `0.0..1.0` curl range.

Do not implement negative curl or hyper-open in the first pass.

Reason:

- hFRIK can internally blend some values outside normal range in specific paths, but ROCK's current hand pose path is mostly normalized and tuned around `0..1`.
- Hyper-open needs separate validation and likely separate clamping per finger.

Possible later extension:

- Limited `>1.0` open bias for selected fingers if hFRIK behavior is verified and visually useful.

## 15. How Pad Evidence Should Affect Surface Targets

Pad evidence can improve `surfaceAimTarget[finger]`.

Recommended rule:

- If pad evidence is valid and closer/better than the existing target, use pad hit point as the current `surfaceAimTarget[finger]`.
- If existing target is explicit and high confidence, keep it unless pad evidence indicates penetration or a clearly better local surface witness.
- If no existing target is valid, pad evidence can create one.

This directly improves:

- Splay calculation.
- Local-transform surface correction.
- Fingertip aim behavior.
- Debug observability.

When a pad target is accepted, it should also be stored object-local if it needs to persist across held updates.

However, for first implementation it may be better to recompute pad evidence each held update instead of persisting it across frames.

Reason:

- Pad evidence depends on current live fingertip position.
- Persisting pad targets can become stale if fingers move significantly.

## 16. How Pad Evidence Should Affect Local Transforms

Current local-transform correction uses `surfaceAimTarget[finger]` to bend finger bones toward a surface target.

With pad evidence:

- Use pad-derived target as a higher-quality surface aim target when valid.
- Keep existing correction strength and max correction limits.
- Do not add a second independent local-transform correction path.

Reason:

- One local-transform correction path is easier to maintain.
- The existing path already handles smoothing, max correction, thumb alternate plane correction, and FRIK baseline transforms.
- Pad probes should enrich the input data, not duplicate transform behavior.

## 17. How Pad Evidence Should Affect Splay

The recently added splay enrichment computes signed palm-plane angle from:

```text
live finger open direction -> current surfaceAimTarget
```

Pad evidence should improve this automatically if it updates `surfaceAimTarget`.

No separate splay-specific path should be added initially.

Reason:

- Splay should remain derived from current target geometry.
- Pad evidence is just a better target source.
- This avoids duplicate splay logic.

## 18. Debug Visualization Plan

Add separate debug visualization for pad probes.

Current debug distinction should be preserved:

- Current curl/base probes: existing color and role.
- New pad probes: separate color and role.
- Accepted final target points: separate marker if possible.
- Rejected pad probes: optional faint/gray line if debug verbosity is high.

Recommended new visual roles:

- `RightGrabFingerPadProbe`
- `LeftGrabFingerPadProbe`
- `RightGrabFingerSurfaceTarget`
- `LeftGrabFingerSurfaceTarget`

Recommended colors:

- Base/curl probes: current blue/pink or existing colors.
- Pad probes: yellow/orange.
- Accepted surface targets: red/orange points.
- Rejected pad probes: gray or low alpha.

Add an API/getter similar to:

```cpp
getGrabFingerProbeDebug(...)
```

Suggested new getter:

```cpp
getGrabFingerPadProbeDebug(...)
```

The debug overlay should make this visible:

```text
debug ray end != accepted surface target
```

This is important for tuning and for understanding why a finger opened or stayed curled.

## 19. Config Plan

Use explicit config only if needed for runtime tuning.

Possible settings:

```ini
bGrabFingerPadProbeEnabled = true/false
fGrabFingerPadProbeRadius = 1.0
fGrabFingerPadProbeDistance = 6.0
fGrabFingerPadClosestSurfaceMaxDistance = 4.0
fGrabFingerPadOpenBiasStrength = 0.65
fGrabFingerPadTargetOverrideMinQuality = 0.5
bDebugShowGrabFingerPadProbes = false
```

Default strategy:

- During initial implementation, default off or debug-gated may be acceptable for tuning.
- Before production commit, decide whether it is safe enough to default on.
- Avoid hidden dormant flags with no validation path.
- If a config is added, document it and add source-boundary tests around usage.

Alternative:

- Reuse existing mesh finger pose enablement and add only debug/tuning constants in code for the first narrow implementation.

The better choice depends on how much tuning is expected after first in-game test.

## 20. Implementation Steps

Step 1: Add pad evidence math helpers.

- Add pure math helpers near `grab_finger_pose_runtime` or a nearby bundled policy section.
- Compute pad center from live finger chain.
- Build direction from current target/grip/palm seat.
- Find local surface witness from current object triangles.
- Return fixed-array evidence for five fingers.

Step 2: Add runtime held-state integration.

- Run pad evidence only when `_grabFingerPosePublished` is true and current object transform is available.
- Run it after `resolveSurfaceAimObjectLocal(...)` and before `applyRockGrabHandPose(...)`.
- Use the same cadence as `rockGrabFingerPoseUpdateInterval`.
- Run first pass during `TouchHeld` transition when the mesh pose is first published.

Step 3: Add target refinement.

- For valid pad evidence, update a publish-time copy of `SolvedGrabFingerPose`.
- Do not mutate long-lived captured targets unless explicitly needed.
- Prefer pad target only when quality is high enough.
- Preserve explicit pinch targets unless pad evidence is clearly better or anti-penetration requires opening.

Step 4: Add curl open-bias refinement.

- Convert pad evidence into a per-finger minimum openness.
- Apply as `targetJointPose = max(targetJointPose, padOpenValue)` per affected joint.
- Keep values clamped to `0..1` in first implementation.
- Smooth using existing joint smoothing path.

Step 5: Preserve local-transform and splay pipeline.

- Feed enriched pose into `makeHandPoseDataFromJointValues(...)` with splay.
- Feed enriched surface targets into `publishLocalTransformPose(...)`.
- Do not add a second local-transform override path.

Step 6: Add debug visualization.

- Store pad probe start/end/hit data in `Hand` state.
- Add getter and overlay role(s).
- Add debug flag if needed.
- Render pad probes distinctly from current curl probes.

Step 7: Add tests.

- Add policy tests for stage gating.
- Add math tests for pad center and direction selection.
- Add tests that pad evidence can open but not close fingers.
- Add tests that invalid pad evidence does not override a valid existing target.
- Add tests that better pad evidence can create or replace `surfaceAimTarget`.
- Add source-boundary tests ensuring new debug config/roles are actually wired if a config flag is added.

Step 8: Validate.

- Run `cmake --preset custom-tests`.
- Run `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`.
- Run `ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%`.
- Run `cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m`.
- Verify deployment to `D:\FO4\mods\ROCK\F4SE\Plugins` if using `custom-fast`.

## 21. Acceptance Criteria

The first implementation is successful if:

- Existing curl probes still work and are not replaced.
- Pad probes run only during `TouchHeld` and held updates.
- Pad evidence can make over-curled fingers open back up.
- Pad evidence can improve `surfaceAimTarget` for splay/local transforms.
- No allocations are introduced in hot held update paths beyond existing mesh rebuild behavior.
- No per-frame blocking I/O is introduced.
- Debug visualization clearly distinguishes base/curl probes from pad probes and accepted targets.
- All tests pass.
- `custom-fast` Release build succeeds.
- The feature fails closed when live skeleton, object transform, or triangles are missing.

## 22. Risks

Risk: Local bone axes may not mean what they appear to mean.

Mitigation: Use live chain-derived pad centers and target-driven directions first. Verify local `-Y` separately before using it.

Risk: Pad probes may jitter as the hand/object relation changes.

Mitigation: Trigger only `TouchHeld+`, smooth curl changes, and require quality thresholds.

Risk: Closest-surface evidence may choose the wrong side of thin geometry.

Mitigation: Use normal checks, palm/grip direction checks, and backside rejection.

Risk: Pad evidence may fight the current local-transform correction.

Mitigation: Feed pad targets through the existing target path instead of adding a second transform path.

Risk: Pad opening may make fingers look too straight on objects that should be wrapped.

Mitigation: Opening-only refinement should be bounded by quality and distance. Use conservative strength initially.

Risk: Debug flag exists but visual output may not currently be fully wired for existing finger probes.

Mitigation: As part of implementation, verify debug overlay population and add tests/source checks so new pad debug roles are actually consumed.

## 23. Weapon Grab Scope

This plan targets held object grab first.

Weapon support grip is lower priority because:

- Weapon grips often use authored or semantically known poses.
- Support grips usually need stability more than adaptive per-finger pad behavior.
- Two-handed weapon authority already has stronger assumptions about hand placement.

Pad probes may still help later for unusual barrels, foregrips, handguards, or modded weapons.

Do not integrate pad probes into weapon grab until object-grab behavior is validated.

## 24. Recommended First Implementation Shape

The first production-grade pass should be conservative.

Recommended behavior:

```text
Feature target: held object grab only
Trigger: TouchHeld and held pose update cadence
Primary effect: open-bias fingers when pad evidence says the surface is already under/inside the fingertip
Secondary effect: improve surfaceAimTarget when pad evidence is higher quality
No effect: selection, early acquisition, weapon grab, physics constraint ownership
No first-pass hyper-open: keep curls clamped to 0..1
```

This directly addresses the observed failure:

```text
Some objects need little/no curl, but current probes make fingers curl into the object.
```

It also preserves the current architecture:

- Base/curl probes continue to solve coarse wrapping.
- Pad probes enrich held-state visual contact.
- hFRIK remains the pose publisher/renderer.
- ROCK keeps exact local-transform refinement as the final high-fidelity layer.

## 25. Key Implementation Principle

Do not treat fingertip pad probes as a new grab acquisition authority.

Treat them as held-object visual pose refinement.

The correct mental model is:

```text
Base/curl probes decide the coarse grasp.
Pad probes prevent visual over-curl and improve fingertip surface placement once the grasp is seated.
```
