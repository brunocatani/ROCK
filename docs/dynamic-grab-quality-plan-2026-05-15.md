# Dynamic Grab Quality Plan - 2026-05-15

## Current Authority Baseline

Validated production dynamic grab authority is:

- drive: hidden no-contact proxy plus finite custom linear/angular constraint;
- translation authority: live palm/generated-collider proxy anchor;
- rotation authority: root-flattened raw hand/controller frame;
- rotation reference name in code: `rawRotationPalmTranslation`;
- object-side pivot: selected contact/authored grip point frozen in BODY-local
  space;
- COM/MOTION: mass, inertia, diagnostics, and release calculations only.

The native mouse-spring path and selectable rotation modes were intentionally
removed after `716fcb1` and cleanup commits `ba9c3d6` / `bd63fb4`.

## Requested Work

Implement the grab-quality systems that were removed, forgotten, or not yet
ported cleanly to the new authority mode:

- multibody object grab, especially loose non-equipped weapons;
- long-object handling;
- mass-relative manipulation feel;
- object mass/inertia affecting swing and authority;
- precision finger placement;
- pivot and palm seating retuning now that the reference frame is stable;
- any legacy spring-era grab quality behavior that can be ported without
  reintroducing native mouse-spring authority;
- HIGGS dynamic-grab behaviors that apply to ROCK's FO4VR-native proxy motor
  architecture.

## Approved Local Sources

- Current ROCK:
  `E:\fo4dev\PROJECT_ROCK_V2\ROCK`
- Legacy spring-era backup:
  `E:\fo4dev\Backups\before motor change\PROJECT_ROCK_V2`
- HIGGS source:
  `E:\fo4dev\skirymvr_mods\source_codes\higgs`
- Prior research note:
  `E:\fo4dev\Backups\just in case\docs\custom-dynamic-grab-authority-current-findings-2026-05-12.md`

No web. No Ghidra for this pass unless separately approved.

## Non-Negotiable Design Constraints

- Do not resurrect native mouse-spring grab authority.
- Do not add selectable rotation-mode fallbacks.
- Do not use COM/MOTION as pivot, target frame, grip frame, or orientation
  authority.
- Do not alter equipped weapon grab or two-hand equipped weapon logic while
  implementing loose dynamic object quality.
- Actor ragdoll grab is out of scope unless a shared helper must be protected.
- Loose non-equipped weapons are dynamic multibody objects, not equipped weapon
  colliders.
- Feature work must be committed in coherent slices.
- This document must be updated before and after each implementation slice.

## Research Checklist

### Current ROCK

- [x] Detection and selected contact evidence.
- [x] Object-side pivot capture and palm seating.
- [x] Proxy creation, filtering, activation, and lifecycle.
- [x] Constraint creation and transform A/B updates.
- [x] Linear motor mass/force/tau policy.
- [x] Direct angular velocity authority and force budget.
- [x] Multibody active prep, body activation, lifecycle restore, and collision
      state restore.
- [x] Loose weapon identification and tuning surfaces.
- [x] Long-object or lever-arm handling.
- [x] Finger pose solve, local-transform correction, and held reapply.
- [x] Release velocity and COM-relative tangential swing.

### Legacy Spring-Era Backup

- [x] What it did for one-hand loose dynamic grab that felt acceptable.
- [x] Multibody/weapon activation behavior that later regressed.
- [x] Any body-set scan, activation, flag lease, or collision restore behavior
      missing from current custom authority.
- [x] Finger/pivot behavior that can be ported without native mouse spring.
- [x] Anything explicitly rejected because it depends on native mouse spring.

### HIGGS Dynamic Grab

- [x] Dynamic detection pipeline.
- [x] Dynamic held transition.
- [x] Constraint/motor creation and per-frame target update.
- [x] Mass registration and force scaling.
- [x] Inertia normalization/clamping/restoration.
- [x] Collision response/tau changes while held.
- [x] Hand deviation/lag behavior.
- [x] Long-object/lever-arm behavior.
- [x] Finger pose and mesh/contact-driven placement.
- [x] Loose weapon differences from generic objects.

## Implementation Slices To Decide After Mapping

No slice is approved until the mapping below is filled with concrete code
evidence.

1. Multibody loose-object activation/restore.
2. Long-object lever-arm angular force policy.
3. Mass-relative manipulation and hand deviation.
4. Loose weapon multibody specialization.
5. Precision finger placement and pivot seating retune.
6. Release velocity / swing polish.
7. Cleanup of obsolete diagnostics and stale docs/tests exposed by the feature
   work.

## Current ROCK Map

### Review Snapshot - 2026-05-15

Confirmed live production authority after the rollback/rebuild/fix sequence:

- ordinary dynamic one-hand loose grabs create a proxy-backed custom constraint;
- the proxy is a hidden no-contact keyframed body;
- proxy translation follows the live generated palm anchor;
- object rotation relation is captured from `rawRotationPalmTranslation`;
- body-B pivot remains the selected contact/authored grip point frozen in BODY
  space;
- `MOTION` and COM are still diagnostic/weight/release data only;
- native mouse-spring wrapper files and runtime fallback modes are removed.

Review issue found:

- `GrabAuthorityProxyMotion.h` still carried the old manual matrix-column
  angular velocity helper, and `GrabAuthorityProxy.h` still exposed a wrapper for
  it, even though production grab now computes held-object angular velocity
  through FO4VR's native hard-keyframe velocity boundary. The stale helper was
  not called by `HandGrab.cpp`, but leaving it available created a future
  reintroduction path for the same world-direction/N/S/E/W failure class.

Fix applied in this review:

- removed the stale proxy angular helper and wrapper;
- removed its tests;
- kept and expanded coverage for the remaining proxy linear velocity helper;
- added a source-boundary guard so the manual angular helper stays removed.

### Dynamic Pipeline Map - 2026-05-15

Current ordinary one-hand loose dynamic grab path:

- selection/body prep:
  - `Hand::grabSelectedObject` scans the selected reference with
    `object_physics_body_set::scanObjectPhysicsBodySet(...)` in
    `InteractionMode::ActiveGrab`;
  - scan options include the selected seed body, selected hit node,
    same-reference requirement, weapon-reference expansion, current held body
    set, and `rockObjectPhysicsTreeMaxDepth`;
  - active prep captures lifecycle state before recursive motion/collision prep
    and restores it on failed grab/release;
  - `_heldBodyIds = preparedBodySet.acceptedBodyIds()` is the canonical
    accepted body list used by lifecycle restore, nearby damping, collision
    suppression, inertia normalization, velocity release, and debug snapshots.
- pivot/reference frame:
  - selected mesh/authored/contact evidence chooses `grabGripPoint`;
  - three-phase pocket logic seats that point to the palm pocket and preserves
    object rotation unless an authored grab node exists;
  - visual/contact Pivot B is frozen in BODY space as
    `_grabFrame.pivotBBodyLocalGame`;
  - active constraint Pivot B is BODY-space data computed from the same
    body-A palm/body-B relation as the constraint target, not from COM/MOTION;
  - `constraintUsesMotionBodyAtGrab` is intentionally false after the verified
    runtime fix. MOTION/COM stays diagnostic/weight/release data only.
- proxy/custom authority:
  - `createProxyConstraintGrabDrive(...)` creates a hidden no-contact
    keyframed proxy body;
  - proxy translation is driven from the live generated palm anchor after
    conversion through
    `hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame`;
  - object rotation relation is captured from
    `rawRotationPalmTranslation`;
  - `updateProxyConstraintGrabDriveTarget(...)` writes transform A/B and the
    ragdoll angular target into the custom constraint each physics between
    phase;
  - `applyProxyConstraintAngularVelocityDrive(...)` gets the FO4VR-native hard
    keyframe angular vector, then clamps magnitude from the finite angular motor
    budget. This is the only production angular-velocity writer for dynamic
    grab.
- mass/inertia:
  - `normalizeGrabbedInertiaForBodies(...)` already operates on all accepted
    bodies with unique-motion dedupe;
  - `setHeldVelocity(...)` and release velocity also operate on the whole
    `_heldBodyIds` set with unique-motion dedupe;
  - fixed: `updateConstraintGrabDriveMotors(...)`, grab telemetry, and angular
    probe telemetry now use `readHeldBodyMassSummary(...)`, so multipart held
    objects are budgeted from the accepted body set instead of only the selected
    primary body;
  - fixed: direct angular velocity authority now writes each unique accepted
    motion through `setHeldAngularVelocity(...)` instead of writing only
    `_savedObjectState.bodyId`.
- loose weapons:
  - loose non-equipped weapons are detected with
    `isLooseWeaponGrabTarget(...)`;
  - current custom authority has loose weapon multipliers for linear tau,
    angular tau, collision tau, linear/angular damping, max force, angular force
    ratio, and recovery speeds;
  - fixed: loose weapons now use a `4.5` max-force multiplier, preserving the
    HIGGS dynamic ratio of `9000` weapon base linear force versus `2000`
    generic-object base linear force before mass capping;
  - equipped weapon and two-hand weapon behavior remain separate and unchanged.
- long objects:
  - fixed: the old unused bounds-axis helper was replaced with
    `computeLocalMeshMaxDistanceFromPoint(...)`;
  - the canonical grab frame stores `longObjectLeverGameUnits`, measured from
    the selected grip point to cached local mesh extents;
  - `applyProxyConstraintAngularVelocityDrive(...)` applies
    `computeLongObjectAngularSpeedScale(...)` to the held angular speed cap so
    long objects do not get the same tip-sweep authority as compact objects.
- hand/finger pose:
  - runtime mesh/contact target set is stored in the grab frame in object local
    space;
  - `solveGrabFingerPoseFromTriangles(...)` is called at grab using current
    triangles, live flattened finger skeleton snapshot, saved pose targets,
    backside rejection, and surface-plane tolerance;
  - pose is delayed during converge/pull and published once touch-held is
    reached, then re-applied on the configured interval with smoothing and local
    transform correction.

### Finger/Palm Seating Review - 2026-05-15

Code review found no new finger-placement bug to patch in this pass:

- HIGGS dynamic grab shifts finger probe starts by `palmToPoint = ptPos -
  palmPos` before curve intersection, so the hand solves as if the selected
  object point has already seated into the palm.
- ROCK reproduces that behavior in
  `solveGrabFingerPoseFromTriangles(...)`: `palmSeatProbeShift =
  poseTargets.seatPointWorld - grabAnchorWorld`, then `baseWorld =
  live.base + palmSeatProbeShift`.
- ROCK uses live root-flattened finger landmarks as the base transform source,
  not authored mesh markers, and only uses mesh/contact evidence to choose the
  seated surface and optional surface aim targets.
- ROCK captures `_grabFingerPose` at grab commit and re-applies that captured
  pose while held instead of recomputing from a moving physics body each frame.

Conclusion: precision finger placement already has the required HIGGS-style
math plus FO4VR root-flattened hand data. No code change was made for fingers
because the review found coverage, not a confirmed defect.

## Legacy Backup Map

Spring-era backup findings:

- one-hand loose object grab used native mouse spring as authority:
  `_heldDriveMode = NativeMouseSpring` and `_nativeGrab.create(...)`;
- shared/two-hand loose-object grab used the shared custom constraint path;
- the backup already contained most of the current non-authority quality
  scaffolding:
  - active object-tree scan/prep;
  - `_heldBodyIds = preparedBodySet.acceptedBodyIds()`;
  - recursive motion/collision enable;
  - inertia normalization for accepted bodies;
  - nearby damping;
  - mesh/contact/finger pose solve;
  - release velocity over the held body set.
- backup one-hand loose weapon handling had native-specific response knobs:
  - `fGrabLooseWeaponNativeLinearResponseMultiplier`;
  - `fGrabLooseWeaponNativeAngularResponseMultiplier`;
  - `fGrabLooseWeaponNativeAngularClampMultiplier`;
  - `fGrabLooseWeaponAdaptiveLeadMultiplier`.
- those native knobs are intentionally not portable as authority because the
  current design removed native mouse spring. The useful lesson is separate:
  loose weapons need their own weight/response policy surface in the custom
  authority path, not a native action fallback.

## HIGGS Dynamic Grab Map

Dynamic-only HIGGS findings from approved source:

- `Hand::TransitionHeld(...)`:
  - chooses `ptPos` from closest mesh/contact geometry when available;
  - computes `desiredNodeTransform = adjustedTransform` then translates it by
    `palmPos - ptPos`;
  - freezes `desiredNodeTransformHandSpace = inverseHand *
    desiredNodeTransform`;
  - creates pivot A from palm position in hand-body space and pivot B from
    `ptPos` in object rigid-body space;
  - creates a custom constraint through `CreateGrabConstraint(...)`;
  - collects all grabbed rigid bodies and saves contact callback delay and
    inverse inertia for each connected body;
  - clamps each connected body's inverse inertia by
    `grabbedObjectMaxInertiaRatio` and `grabbedObjectMinInertia`.
- `GrabConstraintData`:
  - uses transform atoms, ragdoll angular motors, and three linear motor atoms;
  - angular max force is derived from linear force through
    `grabConstraintAngularToLinearForceRatio`;
  - `setTargetRelativeOrientationOfBodies(...)` updates the angular target from
    the desired hand/object relation.
- held update in `State::HeldBody`:
  - updates visual hand from held object using
    `heldTransform * inverse(desiredNodeTransformHandSpace)`;
  - records hand deviation and releases if average hand distance exceeds the
    configured limit after the ignore window;
  - refreshes transform B from
    `desiredHandTransformHavokObjSpace * palmPosHandspace`, not from COM;
  - adjusts motor tau when colliding;
  - gives loose weapons a higher base linear force
    (`grabConstraintLinearMaxForceWeapon`);
  - caps final linear force by mass through
    `grabConstraintMaxForceToMassRatio`;
  - derives angular force from the linear force and angular-to-linear ratio;
  - registers connected bodies via `RegisterObjectMass(...)` and
    `RegisterPlayerSpaceBody(...)`.
- `RegisterObjectMass(...)`:
  - dedupes bodies per frame and accumulates held object mass for player speed
    reduction / global mass effect;
  - this is separate from pivot selection and never moves the grip to COM.
- finger pose:
  - HIGGS computes finger starts/normals/zero-angle vectors in hand/world space,
    shifts the finger starts by `palmToPoint`, checks curve intersections
    against nearby triangles, uses an alternate thumb curve if needed, clamps
    minimum curl to `0.2`, and blends with the finger animator.

## Gap Matrix

Confirmed gaps and fixes:

1. **Primary-only mass budget for multipart held objects**
   - Evidence before fix:
     - `_heldBodyIds` is the canonical whole-object body set.
     - inertia normalization and release velocity already use `_heldBodyIds`.
     - motor force and mass telemetry read only `_savedObjectState.bodyId`.
   - Effect:
     - multipart loose weapons can be budgeted as if only the selected piece
       exists, while the rest of the runtime treats the connected object as a
       set.
   - Fix:
     - add a unique-motion held body mass summary helper;
     - pass aggregate held mass to `GrabMotionController`;
     - keep primary mass as fallback only when the accepted set has no readable
       masses;
     - log aggregate mass, sampled body count, and unique motion count.

2. **Long-object lever policy wired**
   - Evidence before fix:
     - `computeLocalBoundsLongAxis(...)` existed but no live call used it for
       force/rotation.
   - Effect:
     - there is no explicit policy for long-object angular authority beyond
       mass/inertia and the hard-keyframe angular velocity cap.
   - Fix:
     - replace the unused bounds-axis helper with selected-grip-to-mesh-extents
       lever measurement;
     - store `longObjectLeverGameUnits` in the canonical grab frame;
     - scale the direct angular speed cap by configurable
       `referenceLever / actualLever`, clamped to a configured floor;
     - expose `bGrabLongObjectAngularScalingEnabled`,
       `fGrabLongObjectReferenceLeverGameUnits`, and
       `fGrabLongObjectMinAngularScale`.

3. **Loose-weapon custom policy specialized**
   - Evidence before fix:
     - custom loose-weapon multipliers exist but default to `1.0`;
     - spring-era native loose weapon response knobs were removed with native
       authority.
   - Effect before fix:
     - loose weapons had no out-of-box specialization beyond being
       marked `looseWeapon=yes`.
   - Fix:
     - keep tau, damping, angular-ratio, and recovery multipliers neutral;
     - set `fGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5`;
     - raise that key's clamp ceiling to allow the HIGGS dynamic ratio;
     - keep mass cap and long-object angular cap active, so this is not
       infinite weapon authority;
     - update the active production INI in place.

4. **Primary-only direct angular velocity writer**
   - Evidence:
     - aggregate mass, inertia normalization, release velocity, nearby damping,
       activation, and collision cleanup all use `_heldBodyIds`;
     - `applyProxyConstraintAngularVelocityDrive(...)` still wrote the direct
       hard-keyframe angular command only to `_savedObjectState.bodyId`.
   - Effect:
     - multipart loose weapons/objects could receive whole-object mass budgeting
       but one-body angular velocity authority, leaving connected parts to fight
       the primary body under fast wrist rotation.
   - Fix:
     - add `setHeldAngularVelocity(...)` with unique-motion dedupe over the
       primary body plus `_heldBodyIds`;
     - write the clamped angular velocity to every accepted unique motion;
     - log `angBodies` in proxy authority telemetry;
     - add source-boundary guards rejecting a return to primary-only angular
       writes.

## Implementation Log

- 2026-05-15 review cleanup: removed obsolete proxy matrix-column angular
  velocity helper. This does not change the validated runtime grab rotation
  path; it removes dead convention code now superseded by
  `computeHardKeyframeVelocityForTarget(...)`.
- 2026-05-15 mapping: current ROCK, spring-era backup, and HIGGS dynamic-only
  pass confirmed the first safe code fix was the primary-body-only mass budget.
- 2026-05-15 mass-budget slice:
  - added `HeldBodyMassSummary` with unique-motion dedupe over the primary body
    plus `_heldBodyIds`;
  - changed motor target solving to use aggregate held-object mass;
  - changed grab telemetry and angular probe mass reporting to show aggregate
    mass, primary mass, sampled bodies, and unique motions;
  - added a source-boundary guard preventing the motor mass input from returning
    to `_savedObjectState.bodyId` only.
- 2026-05-15 long-object angular slice:
  - removed the unused local-bounds-axis helper and replaced it with
    grip-to-extents lever measurement from cached local mesh triangles;
  - stored the lever in the canonical grab frame;
  - scaled the held angular velocity cap by lever length so long objects do not
    receive the same tip sweep authority as compact objects;
  - added config keys and source/unit tests for the long-object scale policy;
  - updated the active production INI in place with the new keys.
- 2026-05-15 full review follow-up:
  - found the direct angular hard-keyframe writer was still primary-body-only;
  - changed it to write the accepted held-body set with unique-motion dedupe;
  - added proxy authority telemetry for the number of angular bodies written;
  - added a source-boundary guard for the multipart angular-authority rule.
- 2026-05-15 loose-weapon specialization slice:
  - ported HIGGS' confirmed loose-weapon dynamic distinction into the custom
    authority path as a `4.5` max-force multiplier;
  - kept the same mass cap, angular-to-linear ratio, long-object angular cap,
    and non-equipped loose-object classification;
  - updated packaged and active production INIs in place;
  - added source-boundary guards for the loose-weapon max-force multiplier.

## Verification Log

- 2026-05-15 before cleanup: `ctest --test-dir build-tests -C Release
  --output-on-failure -j $env:NUMBER_OF_PROCESSORS` passed 21/21.
- 2026-05-15 after cleanup: policy test binaries rebuilt and full CTest passed
  21/21.
- 2026-05-15 final quality pass: `cmake --build build-fast --config Release
  --target ROCK -- /m` succeeded and deployed `ROCK.dll` / `ROCK.pdb` to
  `D:\FO4\mods\ROCK\F4SE\Plugins`.
- 2026-05-15 final quality pass: `ctest --test-dir build-tests -C Release
  --output-on-failure -j $env:NUMBER_OF_PROCESSORS` passed 21/21.

## Completion Audit - 2026-05-15

Prompt-to-artifact checklist:

- **Markdown plan and regular tracking**
  - Evidence: this document maps current ROCK, the spring-era backup, HIGGS
    dynamic grab, gaps, implementation slices, and verification.
  - Status: complete.
- **Use approved sources**
  - Evidence: sections above cite current ROCK, `E:\fo4dev\Backups\before motor
    change\PROJECT_ROCK_V2`, HIGGS source, and the 2026-05-12 research note.
  - Status: complete.
- **Multibody object grab, especially loose weapons**
  - Evidence: `_heldBodyIds = preparedBodySet.acceptedBodyIds()` is canonical;
    inertia normalization, release velocity, lifecycle, activation, mass
    budget, and direct angular authority now use the accepted held body set.
  - Status: complete for dynamic loose objects/weapons in this scope.
- **Long-object grab quality**
  - Evidence: `longObjectLeverGameUnits` is captured from selected grip point to
    cached mesh extents; `computeLongObjectAngularSpeedScale(...)` gates the
    held angular speed cap; config keys are present in packaged and production
    INIs.
  - Status: complete.
- **Mass-relative grab**
  - Evidence: `readHeldBodyMassSummary(...)` aggregates unique accepted
    motions; `GrabMotionController` receives `massSummary.motorMass()`; mass cap
    and angular-to-linear ratio remain active.
  - Status: complete.
- **Object mass/inertia affecting swing and authority**
  - Evidence: aggregate mass budgets force, long-object lever scales angular
    speed, inertia normalization/restoration still operate on accepted bodies,
    and release velocity uses controller-derived linear/angular policy.
  - Status: complete.
- **Precision finger placement**
  - Evidence: ROCK uses live root-flattened finger landmarks and HIGGS-style
    `palmSeatProbeShift` / `palmToPoint` solving, then captures and reapplies
    `_grabFingerPose` while held.
  - Status: complete; review found no code defect to patch.
- **Pivot and palm seating retune**
  - Evidence: dynamic pivot A resolves from the live generated palm anchor;
    fallback is raw hand origin, not the retired INI pivot; three-phase capture
    seats selected contact/authored grip to `pocket.palmCenterWorld` and freezes
    pivot B in BODY local space.
  - Status: complete for this code pass.
- **Legacy spring-era behavior that can be ported without mouse spring**
  - Evidence: body-set scan/prep, accepted-body lifecycle, collision restore,
    inertia, damping, finger solve, and loose-weapon distinction were mapped and
    retained/ported; native mouse-spring response/adaptive-lead knobs remain
    rejected.
  - Status: complete.
- **HIGGS dynamic behaviors that apply to ROCK FO4VR proxy motors**
  - Evidence: contact/palm pivot, preserved object rotation, finite
    linear/angular motor policy, mass cap, collision tau, startup angular fade,
    loose-weapon force ratio, inertia handling, hand deviation, and finger pose
    were mapped; implemented slices are documented above.
  - Status: complete within dynamic loose-object scope.
- **Commit every feature slice**
  - Evidence:
    - `c6de8ee fix/grab: remove stale proxy angular helper`
    - `b265372 fix/grab: use held body set mass for motor budget`
    - `1185d43 feature/grab: scale held angular cap for long objects`
    - `9adf5e8 fix/grab: apply angular authority to held body set`
    - `2918fcb feature/grab: specialize loose weapon motor force`
    - `c287cbb docs/grab: mark quality audit fixes complete`
  - Status: complete; final audit section added after `c287cbb`.

Residual risk:

- Runtime feel still needs user playtesting because finite-force tuning is
  perceptual and object-specific, but there is no uncovered prompt requirement
  or known code gap left in this scoped pass.
