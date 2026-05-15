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

- [ ] Detection and selected contact evidence.
- [ ] Object-side pivot capture and palm seating.
- [ ] Proxy creation, filtering, activation, and lifecycle.
- [ ] Constraint creation and transform A/B updates.
- [ ] Linear motor mass/force/tau policy.
- [ ] Direct angular velocity authority and force budget.
- [ ] Multibody active prep, body activation, lifecycle restore, and collision
      state restore.
- [ ] Loose weapon identification and tuning surfaces.
- [ ] Long-object or lever-arm handling.
- [ ] Finger pose solve, local-transform correction, and held reapply.
- [ ] Release velocity and COM-relative tangential swing.

### Legacy Spring-Era Backup

- [ ] What it did for one-hand loose dynamic grab that felt acceptable.
- [ ] Multibody/weapon activation behavior that later regressed.
- [ ] Any body-set scan, activation, flag lease, or collision restore behavior
      missing from current custom authority.
- [ ] Finger/pivot behavior that can be ported without native mouse spring.
- [ ] Anything explicitly rejected because it depends on native mouse spring.

### HIGGS Dynamic Grab

- [ ] Dynamic detection pipeline.
- [ ] Dynamic held transition.
- [ ] Constraint/motor creation and per-frame target update.
- [ ] Mass registration and force scaling.
- [ ] Inertia normalization/clamping/restoration.
- [ ] Collision response/tau changes while held.
- [ ] Hand deviation/lag behavior.
- [ ] Long-object/lever-arm behavior.
- [ ] Finger pose and mesh/contact-driven placement.
- [ ] Loose weapon differences from generic objects.

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

## Legacy Backup Map

Pending.

## HIGGS Dynamic Grab Map

Pending.

## Gap Matrix

Pending.

## Implementation Log

- 2026-05-15 review cleanup: removed obsolete proxy matrix-column angular
  velocity helper. This does not change the validated runtime grab rotation
  path; it removes dead convention code now superseded by
  `computeHardKeyframeVelocityForTarget(...)`.

## Verification Log

- 2026-05-15 before cleanup: `ctest --test-dir build-tests -C Release
  --output-on-failure -j $env:NUMBER_OF_PROCESSORS` passed 21/21.
- 2026-05-15 after cleanup: policy test binaries rebuilt and full CTest passed
  21/21.
