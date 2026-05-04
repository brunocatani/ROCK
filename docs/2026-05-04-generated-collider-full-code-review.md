# 2026-05-04 Generated Collider Full Code Review

Review scope: generated hand colliders, generated weapon colliders, shared generated-body keyframe drive, Havok-step drive coordination, root-flattened hand-frame authority, runtime log evidence.

Findings:

1. Native placement failures are ignored after body creation.
   - `HandBoneColliderSet::createBodyForRole()` calls `placeGeneratedKeyframedBodyImmediately()` but publishes the body even if the placement or velocity reset fails.
   - `HandBoneColliderSet::create()` does the same for the palm anchor.
   - `WeaponCollision::createGeneratedWeaponBodiesInBank()` does the same for generated weapon hulls.
   - Impact: ROCK can log and publish "created" generated bodies while the bodies are still at the native default transform or a stale transform.

2. Physics-step drive results are ignored.
   - `HandBoneColliderSet::flushPendingPhysicsDrive()` and `WeaponCollision::flushPendingPhysicsDrive()` discard `GeneratedKeyframedBodyDriveResult`.
   - Impact: missing bodies, stale targets, teleport fallback, and failed native drives never invalidate/rebuild the generated collider set and usually never reach logs.

3. Success-side generated-body telemetry is missing.
   - `GeneratedKeyframedBodyDrive.cpp` logs only failed `driveToKeyFrame()` calls.
   - Impact: runtime logs cannot show target game position, target Havok position, live body position, body-target delta, scale revision, timing, teleport state, or body source. This blocks verification of the exact "other side of the planet" symptom.

4. Havok timestep assumption is not proven at the actual native boundary.
   - `HavokPhysicsTiming::driveDeltaSeconds()` passes `substepDeltaSeconds * substepCount` to `driveToKeyFrame()` from the `beforeWholePhysicsUpdate` listener.
   - Impact: if Bethesda's wrapper expects raw frame delta or per-substep delta instead, generated bodies can lag, overshoot, or stutter even with the correct target transform.

5. Weapon generated-body transforms still hide a mixed rotation convention.
   - `WeaponCollision::makeGeneratedBodyWorldTransform()` transposes `weaponRootTransform.rotate`, then the shared generated-body drive converts the result as if it were an ordinary Ni transform.
   - Impact: hand targets are normal root-flattened Ni transforms while weapon targets are pre-transposed package transforms. The position may be correct, but the shared API does not encode that distinction and the tests currently codify the intermediate transpose rather than the final native transform.

6. The regression tests are mostly regex guards.
   - `GeneratedBodyDriveSourceTests.ps1` catches the raw `NiTransform` callsite but does not exercise transform conversion numerically, placement-failure handling, missing-body handling, timing selection, or success telemetry.
   - Impact: build/tests can pass while the runtime generated-body path is still observably wrong.

7. The available runtime log predates the deployed unit-boundary fix.
   - `ROCK.log` last write: 2026-05-04 00:31:46.
   - deployed `ROCK.dll` last write: 2026-05-04 00:36:42.
   - Impact: current logs prove the old build used root-flattened bones, but they cannot prove the post-fix generated-body drive behavior.

Resolution pass:

1. Initial placement now fails closed.
   - Hand segment bodies, hand palm anchors, and generated weapon bodies are destroyed and not published when native initial placement or velocity reset fails.

2. Normal generated-body motion now crosses one shared game-to-Havok boundary.
   - `GeneratedKeyframedBodyDrive` converts queued `NiTransform` targets to `hkTransformf` before `BethesdaPhysicsBody::driveToKeyFrame()`.
   - Hand and weapon generated colliders share that path.

3. Havok drive timing is explicit.
   - Runtime sampling still reads the FO4VR bhkWorld timing globals.
   - The drive policy uses the raw whole-world delta for the single target written from the whole-step listener; substep timing remains in telemetry.

4. Failure results are consumed.
   - Hand and weapon generated collider flush paths now request rebuilds on missing body, failed teleport placement, or failed native keyframe drive.

5. Success telemetry is available for in-game verification.
   - Verbose generated-body logs include target game position, target Havok position, live body position, body-target delta, live body frame source, motion index, raw/substep/simulated/drive timing, scale, and teleport state.

6. Root flattened naming was corrected.
   - Internal source names and tests now use `GameRootFlattenedBoneTree` for the game root node plus flattened bone tree path.
   - `FrikRootFlattenedBoneTree` no longer appears in `src` or `tests`.

Verification:

- `cmake --build build --config Release`: passed and deployed `D:\FO4\mods\ROCK\F4SE\Plugins\ROCK.dll` at 2026-05-04 01:07:07.
- All PowerShell source tests: passed.
- All compiled `ROCK*Tests.exe`: passed.
- Latest `ROCK.log` still predates the deployed DLL, so runtime behavior must be checked from the next in-game log.
