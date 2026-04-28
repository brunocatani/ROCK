# FRIK v6 Skeleton Hand Target Notes

FRIK v6 external weapon authority was removed on 2026-04-27. Runtime testing showed the two-mod weapon authority split was the wrong ownership model for ROCK's equipped-weapon manipulation. FRIK API v6 is now reused for a narrower, source-level role: applying externally solved hand world targets through FRIK's first-person arm chain.

## Current Boundary

- FRIK API is v6: v5 hand node access, hand world transforms, priority finger/joint poses, state queries, and offhand-grip blocking remain, with `applyExternalHandWorldTransform()` appended.
- ROCK no longer calls `setExternalWeaponAuthority` or `clearExternalWeaponAuthority`; those API entries and contract headers were deleted from both codebases.
- FRIK `WeaponPositionAdjuster` no longer stores, selects, expires, or applies external weapon authority frames.
- ROCK two-handed grip applies the solved visible weapon transform, recomposes both visible hand targets from stored weapon-local hand frames, and asks FRIK to apply those targets through the clavicle-to-hand hierarchy.
- ROCK preserves captured primary/support grip separation when building the support target so controller motion along the barrel axis cannot move the locked grip point.

## Reference

This follows the HIGGS two-handed pattern: capture hand-to-weapon transforms when the grab starts, solve the final weapon transform each frame, then set both hands and collision from that single frame. The hand application mirrors HIGGS' `UpdateClavicleToTransformHand()` approach at source level using FRIK's known `RArm_Collarbone`/`LArm_Collarbone` chains. No Ghidra operations were used because this stayed inside source-level FRIK/ROCK node transform ownership.
