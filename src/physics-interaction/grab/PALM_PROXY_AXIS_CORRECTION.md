# Palm Proxy Axis Correction

Date: 2026-05-18

## Why This Exists

This note exists because the intermittent grab-start snap was not caused by random hand-relative orientation or by object mesh axes feeding bad values directly into the freeze solver. The real problem was a ROCK coordinate authority mismatch: generated hand colliders were visually usable, but the hidden grab authority proxy and frozen pivot math were being reasoned about as if runtime local `Z` were palm depth. In the active runtime proxy, local `-Y` is the palm-face direction. The fix keeps the working generated collider placement convention, adds an explicit grab-authority boundary, and makes the proxy, collider dimensions, frozen pivot, and held object rotation agree on one axis contract.

## Correct Runtime Convention

The corrected palm/proxy local basis is:

- Local `X`: points along the fingers.
- Local `+Y`: points toward the back of the hand.
- Local `-Y`: points into the palm face / palm skin.
- Local `Z`: points across the palm.

The decisive runtime check was the grab authority proxy offset. Setting the proxy offset to `(0, -2, 0)` places the hidden proxy seat into the palm skin. That means the seat direction is proxy-local `-Y`, not proxy-local `-Z`.

## What Was Wrong

Before this correction, multiple comments and grab paths implicitly treated runtime `Z` as the palm-depth axis. That made the generated palm proxy orientation wrong for the frozen point even though many box colliders still looked acceptable in game.

This was hard to see because many palm and object shapes have a dominant `X` dimension and comparatively small `Y`/`Z` variation. Grabs where hand `X` lined up with object `X` often looked correct because the axis error had little visible leverage. Grabs with stronger cross-axis differences exposed the problem: the object and visual hand snapped at acquisition toward a hidden orientation, then behaved correctly after settling because the ongoing manipulation math was otherwise coherent.

## Why The Hybrid Frame Seemed To Work

The older hybrid authority used generated proxy translation but raw hand rotation. That was not the true model. It avoided the broken proxy rotation by coincidence:

- Proxy translation still put the grab point near the palm.
- Raw hand rotation bypassed the Y/Z-swapped proxy basis.
- The frozen pivot and angular target could sometimes agree enough to hide the bug.

Once runtime `-Y` was confirmed as palm-face, the hybrid frame became a compensation for a bad proxy basis. The corrected model removes the raw-rotation proxy authority and uses the corrected proxy-palm frame for both translation and rotation.

## Implementation Rules

- Generated hand collider placement keeps the native column-stored frame that seats hknp shapes correctly.
- The explicit grab-authority boundary transposes that stored rotation before proxy/frozen-point math consumes it through ROCK local-vector transform helpers.
- Palm collider dimensions use local `Y` as palm depth and local `Z` as cross-palm width.
- Palm face/back offsets move along the derived palm-depth axis, not the cross-palm axis.
- Root flattened palm normals use runtime local `-Y` as the palm-face normal.
- Grab capture, ragdoll atom target seeding, held updates, debug readback, and object rotation all use the corrected proxy-palm relation.
- Configured grab authority proxy offsets remain in proxy-local space. `(0, -2, 0)` means "seat two game units toward the palm face."

## Files Carrying The Contract

- `src/physics-interaction/hand/HandColliderTypes.h`: generated-collider to grab-authority boundary, palm axis derivation, palm box dimensions.
- `src/physics-interaction/hand/HandBoneColliderSet.cpp`: palm face/back collider offsets and palm shape sizing.
- `src/physics-interaction/hand/HandFrame.h`: authored-to-runtime bridge and proxy local offset semantics.
- `src/physics-interaction/hand/HandGrab.cpp`: capture/update/readback authority now uses corrected proxy-palm space.
- `src/physics-interaction/hand/RootFlattenedFingerSkeletonRuntime.cpp`: runtime palm normal uses local `-Y`.
- `src/physics-interaction/grab/GrabCore.h`: saved grab relations are proxy-palm relations, not raw-rotation hybrid relations.
- `data/config/ROCK.ini` and `RockConfig.*`: default proxy offsets document and use the corrected local `-Y` seat.

## Verification

- `cmake --preset custom-fast`
- `cmake --build build-fast --config Release --target ROCK -- /m`
- `cmake --preset custom-tests`
- `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- `ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%`

