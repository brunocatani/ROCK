# 2026-05-04 Generated Body Drive Unit Review

This implementation keeps root-flattened bone sampling unchanged because the runtime log shows generated hand colliders resolving all required bones from `FrikRootFlattenedBoneTree`. The failure is at the native generated-body drive boundary: initial placement converts game-space `NiTransform` translation to Havok-space `hkTransformf`, while normal keyframe movement still passed the raw game-space transform into Bethesda's keyframe-drive wrapper. The fix is to make the wrapper consume `hkTransformf` for keyframe drive and to use the same conversion path for normal movement and immediate placement, preventing a mixed game/Havok unit convention from sending generated bodies far from the player after creation.

Evidence from code/logs:
- `ROCK.log` shows `Direct skeleton bone cache rebuilt: source=FrikRootFlattenedBoneTree ... required=46/46`.
- `ROCK.log` shows `HandBoneCache resolved rootFlattenedTree ...`.
- `placeGeneratedKeyframedBodyImmediately()` converts `NiTransform` to `hkTransformf` before `setTransform()`.
- `driveGeneratedKeyframedBody()` passed `state.pendingTarget` directly to `driveToKeyFrame()`, crossing the native boundary in game units.

Verification plan:
- Add a source-boundary test rejecting `driveToKeyFrame(state.pendingTarget, ...)`.
- Require normal generated-body movement to call `driveToKeyFrame(makeHavokTransform(state.pendingTarget), ...)`.
- Build Release and run generated-body, root-flattened hand, source-boundary, and compiled test suites.

Verification results:
- Red test confirmed the bug guard: `GeneratedBodyDriveSourceTests.ps1` failed while normal movement still used raw `state.pendingTarget`.
- Green test passed after `BethesdaPhysicsBody::driveToKeyFrame()` was changed to accept `RE::hkTransformf` and `driveGeneratedKeyframedBody()` converted `state.pendingTarget` through `makeHavokTransform()`.
- Release build passed and copied `ROCK.dll` to `D:/FO4/mods/ROCK/F4SE/Plugins/`.
- All `*SourceTests.ps1` passed.
- All `build/Release/ROCK*Tests.exe` tests passed.
