# Generated Collider Oversize Audit - 2026-05-08

This pass keeps ROCK's existing FO4VR-native generated collider architecture intact: root-flattened FRIK/Game skeleton samples still drive keyframed Havok bodies, and no movement, FRIK anchoring, collision-layer matrix, or native player controller behavior is changed. The fix belongs at the skeleton-frame-to-shape trust boundary because oversized bodies can come from a single bad live bone span or from compounded tuning scale before Havok shape creation.

## Scope

- Audited generated hand colliders in `src/physics-interaction/hand/HandBoneColliderSet.cpp`.
- Audited generated full-body colliders in `src/physics-interaction/body/BodyBoneColliderSet.cpp`.
- Audited shared generated collider frame/hull math in `src/physics-interaction/hand/HandColliderTypes.h`.
- Did not use HIGGS, Ghidra, web, or global collision-matrix edits.

## Findings

- Hand palm/finger and full-body capsule frames were derived directly from live skeleton endpoints without a final role-specific dimension plausibility check.
- Non-finite skeleton translations could enter shared frame math before the frame was marked invalid.
- Body scale sources are multiplicative: profile scale, role scale, and optional zone override scale. Each individual value was sanitized, but the final post-multiply dimensions still needed a hard plausibility boundary.
- Shape builders had minimum-size fallbacks, but no maximum-size protection. A valid finite but huge length/radius could still build an oversized convex hull.

## Changes Made

- Added shared generated-collider dimension validation in `HandColliderTypes.h`.
- Rejected non-finite segment endpoints, non-finite extrapolated parent spans, and non-finite palm anchor inputs before frames become valid.
- Added role-specific hand limits for palm and finger/segment colliders, with power-armor headroom.
- Added role-specific full-body limits for torso, arm/hand/finger, leg, and foot descriptors, with power-armor headroom.
- Validated body dimensions after all profile, role, and zone scales are applied.
- Made hull point generation resilient to non-finite dimensions so private callers cannot leak NaN/Inf into Havok points.
- Added C++ regression checks for oversized dimensions and non-finite endpoints.
- Added `tests/GeneratedColliderDimensionSourceTests.ps1` to keep these guards present at source level.

## Runtime Behavior

- Normal generated hand/body colliders continue to create and update unchanged.
- Implausible generated collider frames are rejected and logged instead of creating oversized Havok bodies.
- Existing generated bodies are not resized from rejected update frames; the bad frame is skipped.
- If a creation-time snapshot is bad enough that no valid body frames remain, creation fails through the existing retry path instead of creating broken geometry.

## Verification

- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\GeneratedColliderDimensionSourceTests.ps1` passed.
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\GeneratedBodyDriveSourceTests.ps1` passed.
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\ConfigDefaultParitySourceTests.ps1` passed.
- `cmake --build build --config Release` passed and deployed `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
- `ctest --test-dir build -C Release --output-on-failure` passed: 47/47.

## Remaining Notes

- The active production INI has `bHandBoneCollidersRequireAllFingerBones = false`, so partial hand creation is still allowed there. The new dimension guard protects partial hands from bad spans, but this setting can still intentionally create fewer hand segment bodies when finger bones are missing.
- Zone local offsets remain configurable and finite-only. This pass guards collider size, not intentional placement offsets.
