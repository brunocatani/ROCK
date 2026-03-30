# Player Character Controller Contact Filter - 2026-05-08

## Why

The native FO4VR player character controller owns locomotion support and world blockers, so disabling its collision layer globally makes the player fall through or lose hard-stop behavior. The safer implementation is a per-contact filter inside the existing native character-controller callback path: preserve native contacts for explicit support/blocker layers, suppress only player-controller rows against objects/actors/ROCK-generated bodies, and leave movement, HMD threshold behavior, FRIK anchoring, ROCK body colliders, and the global collision matrix unchanged.

## Scope

- `ProcessConstraintsCallback` now keeps the existing held-object row compaction and adds player-controller object filtering using the same generated-contact buffer view.
- `HandleBumpedCharacter` now uses the shared policy instead of hand-distance heuristics. It only suppresses when the incoming controller pointer matches the current player character controller and the target is a known character-controller target.
- If the player controller pointer, current hknp world, body id, or target filter layer cannot be read safely, the hook preserves native behavior for that row/callback.
- Non-player controller handling is not intentionally changed.

## Support Allowlist

Native player character-controller handling is preserved for:

- `1 STATIC`
- `2 ANIMSTATIC`
- `3 TRANSPARENT`
- `9 TREES`
- `13 TERRAIN`
- `17 GROUND`
- `26 TRANSPARENT_SMALL`
- `27 INVISIBLE_WALL`
- `28 TRANSPARENT_SMALL_ANIM`
- `31 STAIRHELPER`
- `34 AVOIDBOX`
- `35 COLLISIONBOX`

Everything outside that known support list is treated as an unwanted player-controller contact when the layer can be read, including clutter, guns, props, debris, shell casings, large clutter, actor/body layers, and ROCK layers 43/44/47.

## Fallback

The filter is enabled by default and can be disabled from the active ROCK INI:

```ini
bNativeCharacterControllerObjectContactFilterEnabled = false
```

Setting it to `false` makes both native player-controller object row filtering and `HandleBumpedCharacter` suppression pass through to native behavior. The packaged fallback INI and the active production INI were updated in place with the new key.

## Verification Added

- Policy tests cover support layers, filtered non-support layers, unknown target preservation, and disabled-filter preservation.
- Contact-buffer tests cover mixed-row compaction and preservation of unknown body ids while filtering known clutter rows.

## Review Fixes

The follow-up code review found several important hardening gaps, all fixed in the implementation:

- `HandleBumpedCharacter` now attempts to call the original native function from its SEH fallback when suppression was not already chosen, matching the safer `ProcessConstraintsCallback` pattern.
- Player-controller object filtering now runs even if `PhysicsInteraction::s_instance` is not initialized. Held-object filtering still requires initialized ROCK hand state; player object filtering only needs the controller identity, current hknp world, and target body layer.
- `ProcessConstraintsCallback` now uses the same validated relocated-entry trampoline helper as `HandleBumpedCharacter`. The verified 14-byte prefix at `0x141E4B7E0` is:

```text
48 8B C4
4C 89 48 20
4C 89 40 18
55
41 54
```

- Direct player actor `currentProcess->middleHigh->charController` access was moved behind `character_controller_runtime`, with local SEH containment in the native runtime boundary.
- Tests now cover the full support allowlist and an explicit non-support rejection table, including props, debris, shell casings, actor layers, charcontroller, ROCK hand/weapon/body layers, and a known high matrix layer.
- The shared generated-contact compactor now reports generic result reasons (`filteredGeneratedContactRows` / `noGeneratedContactRowsFiltered`) because it is no longer held-object-only.
