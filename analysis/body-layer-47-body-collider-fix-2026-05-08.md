# ROCK Body Collider Layer 47 Fix - 2026-05-08

## Rationale

Generated full-body colliders need a separate collision identity from hands because body/back/holster/backpack zones will become long-lived gameplay surfaces. Keeping them on layer 43 tied their solver behavior, static-world toggle, contact routing, and held-impact side effects to hand policy. Layer 47 is not a vanilla configured Fallout layer, but the verified hknp collision matrix has 64 rows, so ROCK can own row/column 47 only if it explicitly normalizes the row and every reciprocal bit.

## Implemented Direction

- `ROCK_LAYER_BODY = 47` is treated as ROCK-owned extended Havok matrix capacity, not a BGSCollisionLayer/vanilla configured layer.
- Collision policy now splits:
  - vanilla configured: `0..46`
  - matrix-addressable: `0..63`
  - ROCK extended: `47..63`
- Layer helpers reject `>=64` before bit shifts.
- Body layer policy starts from an explicit allowlist and sets both row 47 and reciprocal column bit 47.
- Body colliders now use `ROCK_LAYER_BODY` filter info instead of the hand layer.

## Body Layer Policy

- Body to hand 43: disabled.
- Body to ROCK weapon 44: enabled.
- Body to body 47: disabled.
- Body to biped/deadbip/biped-no-cc 8/32/33: disabled.
- Body to static/animstatic 1/2: config-backed by `bBodyBoneCollisionStaticWorldEnabled`, default true.
- Body to dynamic clutter/weapon/debris 4/5/19/20/25/29: enabled.
- Body to query-only 36/40/41/42: disabled.
- Body to projectile/spell/cone projectile 6/7/38: disabled.
- Body to noncollidable/char controller 15/30: disabled.

## Power Armor And Tuning

Power armor already uses a larger descriptor profile selected from the live skeleton snapshot. This pass keeps that FO4VR-native split and adds runtime scales on top of it:

- Standard profile radius/length/convex scales.
- Power armor profile radius/length/convex scales.
- Role radius/length scales for torso, arm/hand, leg, and foot segments.

The body collider set hashes the active profile/tuning values and rebuilds generated bodies when config reload changes dimensions.

## Internal Contact Runtime

Body contacts now classify as internal `Body` endpoints and route into `BodyContactRuntime`. This avoids public provider ABI changes while creating a future source for shoulder stash, holsters, backpack surfaces, and body-zone tuning.

Held-object impact evidence excludes body collider IDs so held objects brushing torso/back/legs do not trigger false impact haptics/events.

## PAPER Check

PAPER still has `FO4_LAYER_MAX_CONFIGURED = 47` and loops `<= FO4_LAYER_MAX_CONFIGURED` in `PAPER/src/reload/CollisionLayerPolicy.h`. That pattern is stale for ROCK because 47 is not vanilla configured. ROCK did not copy that pattern.

## Verification Targets

- Collision policy tests cover 47 as non-vanilla but matrix-addressable.
- Tests cover 64+ rejection and symmetric row/column 47 policy.
- Source tests cover body filter info, power-armor tuning, held-impact exclusion, body contact runtime, and hand collider scale-change recreation.
- Full ROCK Release build and test suite should be run after implementation.
