Date: 2026-05-22
Project: ROCK
Branch: feature/ghidra-grab-motor-mapping
Source authority: current local ROCK source, local ROCK.log samples, local CommonLibF4VR BSSkin layout
Confidence: high for non-dynamic skinned refusal root cause; moderate for zero-shape custom mesh follow-up

## Runtime evidence

- `ROCK.log` shows repeated dead-body close-grab failures such as:
  - `Skinned 'Skeleton:0': skipped invalid data bonePtrs=15 boneTransforms=0 vertices=4990 triangles=7346`
  - `Left hand GRAB failed: mesh contact required for 'Skeleton' ... totalTris=0 reason=meshContactRequired`
- The same log also shows custom-ammo failures where far pull starts successfully but the later grab still fails with:
  - `meshNode='10mmAmmo' ... shapes=0 ... totalTris=0 reason=meshContactRequired`

## Verified skinned-body cause

- `src/physics-interaction/grab/MeshGrab.h` was reading `BSSkin::Instance` fields with offsets that do not match the local FO4VR layout.
- The guarded native reads used:
  - `boneCount` from `+0x38`
  - bone-node array pointer from `+0x18`
- Local `CommonLibF4VR/CommonLibF4/include/RE/Bethesda/BSSkin.h` documents the verified FO4VR layout as:
  - `bonesData` at `+0x10`
  - `bonesCount` at `+0x18`
  - `boneData` at `+0x40`
- That mismatch explains the log signature:
  - the code was interpreting the wrong field as `boneCount`;
  - the code was then treating the `bonesCount` slot as a pointer, so every bone-node read failed and `bonePtrs` equaled the full reported bone count;
  - dynamic skinned meshes could still work because live dynamic vertices provide position authority even when bone-owner metadata is missing;
  - non-dynamic skinned meshes then produced zero usable triangles because their world-space reconstruction depends on valid bone ownership/transforms.

## Zero-shape custom mesh follow-up

- The custom-ammo/custom-mesh failures are a separate path from the skinned offset bug because the log reports `shapes=0 skinned=0 totalTris=0`.
- One low-risk source inconsistency was verified in `HandGrab.cpp`: mesh extraction used a hardcoded recursion depth of `10` while the rest of the object-tree scan already uses the configured `rockObjectPhysicsTreeMaxDepth`.
- The current follow-up is to align mesh extraction with the configured object-tree depth before assuming a broader geometry-support problem.

## Implementation implication

- Non-dynamic skinned close grabs should use the verified FO4VR `BSSkin::Instance` field layout for guarded native reads.
- Mesh extraction depth should stay coherent with the configured object-tree traversal depth so deep custom meshes are not silently excluded by a second, smaller recursion cap.
