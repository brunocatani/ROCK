# ROCK-Owned Weapon Visual Authority Implementation Plan

> Tracking plan for moving equipped weapon manipulation out of FRIK external authority and into ROCK as the runtime weapon owner.

## Goal

ROCK owns equipped weapon manipulation at runtime. FRIK remains the skeleton, IK, and hand-pose backend, but ROCK writes the final visible weapon transform and updates weapon collision bodies from that same final transform.

## Why This Approach

The current split is fragile: ROCK computes a two-handed weapon transform for collision, then asks FRIK to apply a matching visual weapon transform through external authority. If FRIK rejects the frame, validates a different weapon node, or overwrites the weapon later in its pass, collision moves but the actual gun does not. The higher-quality architecture is one runtime weapon authority, with FRIK used as a service provider for skeleton and hand animation.

## Key Changes

- ROCK becomes the runtime weapon transform writer for equipped weapon manipulation.
- FRIK's native offhand grip and weapon-position writes are disabled while ROCK owns weapon runtime authority.
- ROCK uses the same first-person weapon node source as FRIK: `f4vr::getWeaponNode()`.
- ROCK applies two-handed support-grip transforms to the actual weapon node directly.
- ROCK applies active two-handed visible hand targets from stored weapon-local hand frames through FRIK's skeleton-only arm-chain API, so controller movement guides the weapon solve without sliding the visual palms along the gun.
- ROCK updates generated weapon collision bodies only from the final visual weapon frame it actually applied.
- FRIK's existing two-handed pivot behavior is ported into ROCK:
  - primary grip remains the pivot anchor;
  - support hand drives aim direction;
  - weapon translation is recomputed after rotation so the primary grip does not drift;
  - primary/offhand hand frames and finger poses are kept in the same final frame.
- Existing FRIK weapon offsets remain compatible and are reused by ROCK rather than discarded.

## Implementation Tasks

### Task 1: Persist Plan And Add Pure Visual Transform Math

- Status: complete.
- Added this tracking plan.
- Added pure helper math for converting a target world transform to weapon parent-local space.
- Added regression tests that prove the parent-local transform recomposes to the target world transform.

### Task 2: Stop Depending On FRIK External Weapon Authority For Two-Handed Runtime

- Status: complete.
- `TwoHandedGrip::updateGripping()` applies the solved weapon world transform to the actual weapon node.
- FRIK remains the hand/finger pose backend through tagged hand-pose calls.
- The active two-handed runtime path no longer calls `setExternalWeaponAuthority()`.
- FRIK v6 external authority API entries and stale cleanup calls were removed from both codebases.

### Task 3: Use One Weapon Node Source

- Status: mostly complete.
- Replaced ROCK equipped weapon update, interaction, and grab-suppression lookups with `f4vr::getWeaponNode()`.
- Semantic mesh and weapon collision extraction now receive that same node from the runtime interaction path.
- Runtime diagnostics for mismatched legacy root-found and first-person weapon nodes remain to be added only if needed for field debugging.

### Task 4: Synchronize Visual And Collision Frames

- Status: complete for ROCK-owned one-handed and two-handed authority paths.
- Two-handed support grip stores `weaponNode->world` after ROCK applies the visual transform, then moves weapon collision bodies from that applied frame.
- Two-handed support grip now recomposes primary and support hand world targets from locked weapon-local hand frames after the final weapon solve, then sends those targets to FRIK API v6 for HIGGS-style clavicle-chain application.
- One-handed mesh primary grip authority is gated behind `bOneHandedMeshPrimaryGripAuthorityEnabled` and defaults off, so FRIK owns normal equip/game-start weapon orientation.
- If visual application fails, the collision follow-up frame is not published.
- Weapon collision now performs one final per-frame sync after weapon visual authority settles, using each generated hull's original mesh source root transform instead of applying mesh-local centers against the `Weapon` attachment root.
- Debug overlay logging already reports requested/applied hand mismatch while two-handed authority is active.

### Task 5: Suppress FRIK Runtime Weapon Writes While ROCK Owns Weapon Authority

- Status: complete with ROCK as weapon last-writer and FRIK as skeleton hand-target backend.
- ROCK now runs its frame update after the chained original frame update, so FRIK finishes its source-level skeleton/weapon pass before ROCK applies final weapon visual state.
- FRIK still owns normal one-handed equip/game-start weapon positioning. ROCK only takes the final visual hand/weapon write while active two-handed support grip is running.
- FRIK API v6 does not restore external weapon authority; it only exposes `applyExternalHandWorldTransform()` so ROCK can place visible hands through FRIK's first-person arm hierarchy instead of writing `RArm_Hand`/`LArm_Hand` directly.

### Task 6: Port FRIK Weapon Offset Compatibility

- Move FRIK weapon-offset naming/loading rules into a shared helper or duplicate them exactly in ROCK with tests.
- ROCK reads existing `FRIK_Config\Weapons_Offsets` JSON files for weapon, primary-hand, offhand, throwable, and back-of-hand UI offsets.
- ROCK preserves left-handed and power-armor suffix behavior.

### Task 7: Port Remaining FRIK Two-Handed Weapon Details

- Status: in progress.
- Port FRIK's primary pivot preservation, offhand rotation offset, scope-camera adjustment, and muzzle fix ownership into ROCK.
- Keep FRIK native code available only when ROCK authority is disabled.
- Document any native muzzle/scope behavior that requires binary verification before changing.
- 2026-04-27 finding: the right hand needs a hierarchy exclusion that the left hand does not. The equipped `Weapon` node lives under the primary/right hand chain, so FRIK's skeleton-only external hand target must update the right arm/fingers while ignoring the current weapon child. Without that exclusion, FRIK can pull the weapon by the old hand-relative local transform after ROCK has already solved the final visual weapon frame.
- 2026-04-27 implementation: FRIK `Skeleton::applyExternalHandWorldTransform()` now propagates the solved arm transform with the current weapon node name as the ignored child, matching FRIK's own primary-hand grip offset path. ROCK remains the final weapon owner.
- 2026-04-27 implementation: ROCK now ports FRIK's source-level muzzle fix after its final weapon/collision update, copying the projectile node world transform into the muzzle fire node so shots/effects originate from the current barrel tip after ROCK's last weapon write.
- 2026-04-27 finding: the support-hand lock was still stretching the visible arm because FRIK's external hand API was correcting the collarbone after FRIK had already solved the controller-driven arm. HIGGS avoids this class of artifact by moving the whole clavicle/arm chain toward the requested hand transform rather than writing the hand bone alone.
- 2026-04-27 implementation: FRIK now resets the targeted first-person arm chain to its frame baseline and reuses the same full-arm IK solve as `setArms()` for external ROCK hand targets. ROCK still owns the weapon frame; FRIK owns the visible skeleton deformation for the requested locked hand frame.
- 2026-04-27 implementation: FRIK now resyncs movement-dampened Pip-Boy screen history after a successful external hand target on the Pip-Boy arm, preventing the next frame from interpolating against the pre-authority arm pose.
- 2026-04-27 finding: the right hand still differed from native FRIK two-handed weapon handling because ROCK published the finger pose after asking FRIK to apply the locked wrist frame, and FRIK's primary weapon hand pose path still preferred `copy1StPerson()` over tagged external pose overrides. That let the wrist try to follow the weapon while the fingers remained finalized against the previous primary-hand frame.
- 2026-04-27 implementation: ROCK now publishes primary/support hand poses before calling FRIK's locked hand transform API. FRIK now lets tagged hand-pose overrides win over the primary weapon `copy1StPerson()` branch, then refreshes the hand pose and final bone/geometry arrays after each successful external hand authority apply.
- 2026-04-27 refinement: ROCK no longer publishes a primary/right hand pose during two-handed locked weapon authority. The right hand keeps FRIK's native/tuned weapon grip pose, while ROCK still sends the right wrist frame so the hand follows the pivoted weapon. ROCK continues to publish the support/left hand pose because that hand is driven by the mesh contact point.
- 2026-04-27 finding: the remaining right-hand snap on support-grab activation came from the primary mesh-grip capture path replacing the current FRIK/INI/native wrist rotation with the mesh-derived grip frame rotation. The left hand was unaffected because support capture already aligns only by contact point and preserves the current hand rotation.
- 2026-04-27 finding: on some weapons, the primary mesh-grip selector could still pick a stock, magazine, receiver, or other named/bounded part when support grip started. Even with wrist rotation preserved, replacing the right-hand anchor point moved the configured FRIK/INI grip relationship to the wrong mesh part.
- 2026-04-27 refinement: primary/right locked hand capture no longer selects a mesh grip point during two-handed activation. It captures the current FRIK/INI/native right-hand-to-weapon frame exactly as it exists at grab start, then stores that frame for later pivots. Support/left capture still uses the contacted mesh point.

## Test Plan

- Run ROCK pure transform tests after each math/policy change.
- Build ROCK Release after each runtime slice.
- Build FRIK Release after FRIK API suppression changes.
- Runtime checks:
  - two-handed rifle support grip pivots the actual visible weapon;
  - collision bodies stay glued to the visible weapon;
  - FRIK no longer logs active external weapon authority for ROCK two-handed grip;
  - ROCK logs visual authority active and final visual/collision mismatch near zero.

## Verification Log

- 2026-04-27: `cmake --build build --config Release --target ROCKTransformConventionTests` passed.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed.
- 2026-04-27: `cmake --build build --config Release` passed and produced `build\Release\ROCK.dll`.
- 2026-04-27: deployed `ROCK.dll` and `ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
- 2026-04-27: fixed weapon collision shifting by removing the pre-authority body transform write and deploying a source-root-based final collision sync build.
- 2026-04-27: disabled one-handed mesh primary visual authority by default to stop equip/game-start weapons from snapping to guessed mesh-grip rotations.
- 2026-04-27: removed the old FRIK v6 external weapon authority entries and contract headers from both codebases.
- 2026-04-27: added ROCK locked two-handed hand visual authority, recomposing both visible hand targets from stored weapon-local frames after the weapon solve.
- 2026-04-27: added FRIK API v6 skeleton-only hand target application and switched ROCK two-handed grip to call it instead of directly writing hand bones.
- 2026-04-27: added no-slide support target regression coverage; support controller motion along the primary/support axis preserves the captured grip separation while lateral motion still pivots the weapon.
- 2026-04-27: added ROCK muzzle authority regression coverage for the projectile-world to fire-node-local transform rule.
- 2026-04-27: FRIK external hand target propagation now ignores the equipped weapon child, preventing the right hand update from fighting ROCK's final weapon transform.
- 2026-04-27: ROCK applies final muzzle authority after ROCK's final weapon/collision sync so muzzle effects use the current projectile node/barrel-tip world transform.
- 2026-04-27: `cmake --build build --config Release --target ROCKTransformConventionTests` passed.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed.
- 2026-04-27: `cmake --build build --config Release` passed for FRIK and produced `build\Release\FRIK.dll`.
- 2026-04-27: `cmake --build build --config Release` passed for ROCK and produced `build\Release\ROCK.dll`.
- 2026-04-27: deployed `FRIK.dll`/`FRIK.pdb` to `D:\FO4\mods\FRIK 80\F4SE\Plugins` and `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
- 2026-04-27: FRIK full-arm external hand target solve built successfully in Release.
- 2026-04-27: ROCK Release build passed after the FRIK arm-authority API change.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after the FRIK arm-authority API change.
- 2026-04-27: deployed the matching FRIK/ROCK Release DLL and PDB files after the full-arm external hand target change.
- 2026-04-27: added a ROCK regression contract for two-handed hand authority order: weapon visual, hand pose, then locked hand visual.
- 2026-04-27: `cmake --build build --config Release` passed for FRIK after the right-hand/finger authority refresh change.
- 2026-04-27: `cmake --build build --config Release` passed for ROCK after the two-handed hand authority order change.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after the two-handed hand authority order change.
- 2026-04-27: deployed the matching FRIK/ROCK DLL and PDB files after the right-hand/finger authority refresh change.
- 2026-04-27: added a ROCK regression contract that primary/right two-handed grip pose publication stays disabled while support/left pose publication remains enabled.
- 2026-04-27: `cmake --build build --config Release` passed for ROCK after disabling primary/right two-handed pose publication.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after disabling primary/right two-handed pose publication.
- 2026-04-27: deployed updated `ROCK.dll`/`ROCK.pdb` after disabling primary/right two-handed pose publication.
- 2026-04-27: added a ROCK regression contract that locked two-handed hand capture does not use mesh grip-frame rotation at grab start.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after preserving primary/right grab-start wrist rotation.
- 2026-04-27: `cmake --build build --config Release` passed for ROCK after preserving primary/right grab-start wrist rotation.
- 2026-04-27: deployed updated `ROCK.dll`/`ROCK.pdb` after preserving primary/right grab-start wrist rotation.
- 2026-04-27: added a ROCK regression contract that two-handed primary/right grip capture does not select a mesh grip point at grab start.
- 2026-04-27: `cmake --build build --config Release` passed for ROCK after keeping the primary/right grip point on the current FRIK/INI/native relationship.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after keeping the primary/right grip point on the current FRIK/INI/native relationship.
- 2026-04-27: deployed updated `ROCK.dll`/`ROCK.pdb` after keeping the primary/right grip point on the current FRIK/INI/native relationship.
- 2026-04-27: removed dead primary/right mesh finger-pose publication code from the two-handed path; the right hand keeps FRIK's native/tuned pose and the grip source log now reports `current-frik`.
- 2026-04-27: `cmake --build build --config Release` passed after removing dead primary/right pose publication code.
- 2026-04-27: `build\Release\ROCKTransformConventionTests.exe` passed after removing dead primary/right pose publication code.
- 2026-04-27: deployed updated `ROCK.dll`/`ROCK.pdb` after removing dead primary/right pose publication code.

## Ghidra Boundary

No new native offsets are planned for the first two tasks. If implementation reaches hook timing, muzzle behavior, projectile/wand behavior, or any unknown native node ownership, pause and request approval before using Ghidra MCP.
