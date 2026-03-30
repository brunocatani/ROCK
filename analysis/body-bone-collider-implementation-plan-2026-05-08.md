# Full Body Bone Collider Set Plan - 2026-05-08

## Why This Approach

ROCK already has a production path for generated keyframed hknp bodies through the hand bone collider set. The full body collider set should use the same generated-body drive model, but it should live as its own `BodyBoneColliderSet` subsystem instead of being owned by either hand. Torso, arm, leg, shoulder, back, and later backpack or holster zones are character-body features, not hand features, and keeping that boundary separate avoids forcing future body interactions through hand state.

The main alternative was to extend `HandBoneColliderSet` to also create non-hand bones. That would reuse code but would make hand lifecycle, hand debug caps, and hand contact classification responsible for torso and leg bodies. That is wrong for future work: body colliders need to become shared anchors for shoulder stash, weapon holsters, backpack storage, and non-hand collision probes. A sibling generated-body subsystem is the cleaner long-term structure.

## Scope

- Create generated keyframed hknp bodies for the major FRIK/root-flattened body bones.
- Use the existing body descriptor tables in `SkeletonBoneDebugMath` for standard and power armor variants.
- Keep hand/finger colliders in `HandBoneColliderSet`; do not duplicate finger bodies in the body set.
- Drive the body collider transforms from the same skeleton source used by hand colliders.
- Route body collider visualization through the existing `bDebugDrawHandBoneColliders` visualizer toggle, with a separate maximum body-body draw cap for tuning.
- Gate runtime body collider creation through a config key so the system can be disabled without code changes.

## Collision Policy

The body colliders should use ROCK's generated hand/tool collision layer policy. This keeps generated body, hand, and other same-layer ROCK bodies from self-colliding while still allowing real object and weapon interactions through the existing layer matrix.

That choice is deliberate:

- It avoids solver noise from generated hands colliding with generated arms, torso, or legs.
- It lets future holster/backpack/shoulder interactions use stable body-space colliders without building an isolated fake sensor system.
- It keeps collision-layer ownership in one existing ROCK policy instead of introducing a third layer that would require broader matrix verification.

## Future Use

The first version is a physical body-collider foundation, not a complete stash or backpack system. Future systems should use these body IDs and bone roles as stable anchors for:

- Shoulder stash-to-inventory zones.
- Back weapon holsters.
- Backpack storage volumes and pull-out interaction zones.
- Body-relative grab exclusion and damping regions.
- Per-bone debug/tuning overlays.

## Verification Notes

This implementation does not require new FO4VR binary layout claims. It uses existing local ROCK/F4VR skeleton providers, hknp body creation wrappers, and already-mapped generated-body drive code. Ghidra is not needed unless runtime testing shows an hknp filter/material behavior that cannot be explained from the current local code.
