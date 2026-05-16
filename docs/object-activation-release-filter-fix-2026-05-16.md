# Object Activation Release Filter Fix - 2026-05-16

## Why This Approach

The logs showed active prep succeeding for loose objects (`setMotion=ok`, `enableCollision=ok`) while release restored filters even when ROCK intentionally kept the object dynamic. Adding another wake call would not solve a dynamic body that has been put back into a non-colliding filter state. The lifecycle policy now keeps motion and collision ownership coherent: failure paths restore captured state, system-owned non-dynamic bodies are returned to the engine, and loose dynamic physical drops keep the active collision filter created for the grab.

## Local Findings

- Shovel test logs showed the selected body on collision layer `4` during grab after active prep.
- Release audit showed `restoredMotion=0 restoredFilter=1` for that same loose dynamic body.
- That combination can leave a physical loose object dynamic but with its pre-active filter restored, matching the reported broken collision/activation behavior.

## Implemented

- `BodyLifecycleSnapshot::makeRestorePlan()` no longer restores filters unconditionally during protected physical release.
- Loose dynamic bodies that remain dynamic after release keep their active collision filter.
- Failed setup and explicit restore-all plans still restore captured filters.
- System-owned non-dynamic bodies still restore both motion and filter on release.
- Added compiled lifecycle policy coverage for loose dynamic release and system-owned release.
