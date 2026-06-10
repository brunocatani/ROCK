# 2026-06-06 - Soft Contact World-Only Runtime Plan And Deferred Hand Layer Split

## Revision Status

Reviewed against ROCK source on 2026-06-06 before implementation.

Implementation status as of 2026-06-10:

- `cb858b6` made production soft contact world-only.
- Hand-hand, hand-body, and hand-weapon visual soft-contact runtime solve paths were removed.
- Stale non-world soft-contact config fields and INI reads were removed so persisted keys cannot re-enable unsupported behavior.
- `bSoftContactWorldEnabled` remains the primary user-facing toggle.
- World surface acceptance now covers stable support/blocker layers: static, animstatic, transparent, trees, terrain, ground, transparent small, invisible wall, transparent small anim, stair helper, avoid box, and collision box.
- `sSoftContactWorldShapeCastFilterInfo` was added after review so soft-contact fallback queries no longer share selection query tuning.
- Debug soft-contact snapshots and overlay labels now expose source kind: native, cached, query, or unknown.

User revision: production soft contact should interact only with world surfaces. Hand-hand, hand-body, and hand-weapon visual soft collision should be disabled or removed from the implementation plan because those interactions do not work well enough in practice.

Source authority: current ROCK source, local ROCK implementation notes, and the user revision above. No web, HIGGS, Ghidra, FO4 Mods MCP, or external references were used.

## Summary

The original layer-split diagnosis was correct for native hand-hand contact, but it is no longer the right first implementation target for soft contact.

ROCK soft contact should now be treated as a world-only visual stop system:

- keep world contact as the only production soft-contact visual authority;
- disable or remove hand-hand soft collision;
- disable or remove hand-body soft collision;
- disable or remove hand-weapon soft collision;
- do not enable new native right-left hand pairs for soft contact;
- do not make the side-hand layer split a prerequisite for improving soft contact;
- keep grab, weapon support, semantic contact, dynamic push, held-object contact, and provider evidence as separate systems with their own ownership rules.

This turns the immediate work from a broad native-contact conversion into a cleanup and hardening pass for world-only hand stops.

## Pre-Implementation Source Evidence

Before `cb858b6`, config defaults still enabled non-world soft contact:

- `src/RockConfig.h` has `rockSoftContactHandHandEnabled = true`.
- `src/RockConfig.h` has `rockSoftContactWeaponHandEnabled = true`.
- `src/RockConfig.h` has `rockSoftContactBodyEnabled = true`.
- `src/RockConfig.h` has `rockSoftContactWorldEnabled = true`.
- `src/RockConfig.cpp` reads `bSoftContactHandHandEnabled`, `bSoftContactWeaponHandEnabled`, `bSoftContactBodyEnabled`, and `bSoftContactWorldEnabled`.

Before `cb858b6`, runtime behavior in `src/physics-interaction/contact/SoftContactRuntime.cpp`:

- `buildHandShapes` builds synthetic hand capsules for hand-hand contact.
- `buildBodyShapes` captures a separate skeleton snapshot and builds body capsules.
- `solveShapeAgainstShapes` drives synthetic hand-hand and hand-body candidates.
- `solveShapeAgainstWeapon` calls `WeaponCollision::tryFindSoftContactForCapsule` for hand-weapon candidates.
- `solveWorldStaticContact` is the only path with native/query/cached-plane continuity.
- `candidatePriority` allowed hand-hand, weapon-hand, body, and world candidates to compete.

World path retained by the implementation:

- uses hand/world probes from palm, grab anchor, and finger tips;
- consumes fresh native world evidence when available;
- falls back to swept sphere and rest queries;
- keeps a cached world plane for contact continuity;
- drives world haptics from approach speed.

Resolved implementation limitation:

- `SoftContactWorldPolicy::acceptsWorldSurfaceLayer` previously accepted only `STATIC` and `ANIMSTATIC`, so terrain, ground, invisible walls, and other support/blocker surfaces could be missed. The production policy now accepts the stable support/blocker layer set listed in the implementation status above.

Layer state deferred for soft contact:

- generated hand bodies all use layer 43 through `ROCK_HAND_LAYER` and `kHandFilterInfo`;
- `ROCK_LAYER_RELOAD` is currently aliased to `ROCK_LAYER_HAND`;
- layer 43 is a hand/tool/reload compatibility layer today;
- side-specific hand layers remain a valid future direction if native hand-hand evidence is needed for another system, but world-only soft contact does not need them.

Related local note:

- `docs/implementation-notes/2026-06-06-collider-hot-path-performance.md` remains useful before enabling additional native contact pairs, but world-only soft contact does not require enabling those pairs.

## Implementation Decision

Production soft-contact visual authority is world-only.

Do not ship hand-hand, hand-body, or hand-weapon soft-contact production paths as disabled fallbacks, config-selected alternatives, or shadow implementations. If those paths are not part of production behavior, remove their runtime solving code or make them explicitly test/debug-only with no production effect.

Do not add side-specific hand layers as part of the current soft-contact work. Layer splitting should be a separate future task only if another system needs native hand-hand contact evidence.

Do not change production deployed INIs unless the user explicitly requests it. Source defaults and source behavior can still be updated so stale INI keys cannot re-enable unsupported non-world soft contact.

Keep these systems separate from visual soft contact:

- generated hand colliders used as native contact evidence;
- hand semantic contact;
- weapon support grip contact;
- dynamic push assist;
- held-object impact/contact handling;
- provider external contacts.

## Phase 1: Freeze Soft Contact To World Only

Goal: make source behavior reflect the world-only decision before tuning anything else.

Required changes:

- set source defaults for hand-hand, hand-body, and hand-weapon soft contact to disabled if the config fields remain temporarily;
- prevent stale INI keys from re-enabling those unsupported modes, or remove those config keys in the same coherent cleanup;
- update the active soft-contact log line so it reports world-only behavior and does not imply hand/body/weapon soft contact is active;
- update debug overlay labels so only world soft-contact candidates are shown in production;
- keep `bSoftContactWorldEnabled` as the primary user-facing soft-contact toggle.

Preferred cleanup:

- remove `rockSoftContactHandHandEnabled`, `rockSoftContactWeaponHandEnabled`, and `rockSoftContactBodyEnabled` if no runtime production path remains;
- remove associated INI reads rather than leaving config flags that do nothing;
- remove stale documentation claiming hand/body/weapon soft contact is supported.

If persisted INI compatibility is required later, document the ignored keys clearly and ensure source-boundary tests prove they cannot re-enable removed behavior.

## Phase 2: Remove Non-World Runtime Solves

Goal: make `SoftContactRuntime` cheaper and less fragile by removing unsupported candidate types.

Remove or production-disable these paths:

- synthetic hand-hand capsule solving;
- synthetic hand-body capsule solving;
- hand-weapon soft contact through `WeaponCollision::tryFindSoftContactForCapsule`;
- body snapshot capture from the soft-contact runtime;
- per-frame local hand/body shape vectors used only by removed paths;
- non-world `ContactKind` priority handling where it no longer has a caller;
- non-world response tuning such as weapon-specific correction scale and hard-stop penetration if no other runtime uses it.

Keep or refactor these paths:

- world probes built from palm, grab anchor, and finger tips;
- native world evidence consumption;
- world query fallback;
- cached world-plane continuity;
- world haptics;
- stronger-owner clears for grabs, held objects, dominant weapon authority, support grip, disabled hand state, and provider/FRIK unavailability.

Do not leave a production fallback that silently switches from world-only to hand/body/weapon collision.

## Phase 3: Harden World Contact

Goal: make the one remaining soft-contact behavior reliable.

World surface policy:

- review and expand `SoftContactWorldPolicy::acceptsWorldSurfaceLayer` beyond only `STATIC` and `ANIMSTATIC`;
- consider terrain, ground, invisible wall, trees, transparent support, stair helper, avoid box, and collision box layers where runtime checks confirm they behave as stable blockers;
- keep dynamic props out of world soft contact unless a separate visual-stop policy is explicitly designed.

World probe quality:

- keep the palm and grab-anchor probes;
- keep finger-tip probes, but validate they do not create sticky contact on thin surfaces;
- consider a small fixed-capacity per-hand probe table owned by `SoftContactRuntime` instead of rebuilding temporary containers;
- keep probe IDs stable so cached-plane continuity remains predictable.

Candidate selection:

- keep world contacts above all removed contact kinds by making world the only production candidate kind;
- prefer native/cached-plane evidence over query evidence when fresh;
- keep cached planes only while the current probe is still penetrating and tangent drift remains bounded;
- reject stale native evidence quickly when the hand moved away or ownership changed.

Correction stability:

- use the existing `soft_contact_math::smoothCorrection` or equivalent only if world contact still pops after cleanup;
- keep smoothing fast and clear it immediately on owner changes, grab transitions, support grip, weapon authority, disabled hand state, or invalid provider data;
- do not make the cached plane a sticky target when the hand has separated.

Haptics and debug:

- keep haptics world-only;
- include source kind: native world, cached plane, query sweep, or rest query;
- keep logs sampled and avoid contact-callback logging;
- expose target layer/filter/ref identity only behind existing diagnostics toggles.

## Phase 4: Native World Evidence Cleanup

Goal: keep native evidence useful without expanding contact scope.

Keep publishing and consuming hand-world native contact evidence.

Do not publish soft-contact visual evidence for these routes in the current implementation:

- right hand to left hand;
- left hand to right hand;
- hand to body;
- hand to weapon.

If those native contacts are still needed by semantic, provider, support-grip, or dynamic-push systems, keep that evidence in those systems and do not route it into `SoftContactRuntime`.

Native world evidence requirements:

- raw contact extraction stays lazy and only runs for routes that consume it;
- normals are oriented toward the source hand before correction is computed;
- evidence age remains small, currently one to two frames;
- source velocity is used only for world haptic approach speed and diagnostics;
- invalid body IDs, stale frames, non-finite contact points, and non-finite normals fail closed.

## Phase 5: Deferred Hand Layer Split

This is no longer part of the soft-contact implementation.

If a future task needs native right-hand to left-hand contact for another system, use the previous split direction:

- `ROCK_LAYER_HAND_TOOL = 43`
- `ROCK_LAYER_WEAPON = 44`
- `ROCK_LAYER_BODY = 47`
- `ROCK_LAYER_HAND_RIGHT = 48`
- `ROCK_LAYER_HAND_LEFT = 49`
- `ROCK_LAYER_RELOAD = ROCK_LAYER_HAND_TOOL`

Deferred split guardrails:

- do callback hot-path classification first;
- do not reuse vanilla layers 45 or 46;
- keep layer 43 as tool/reload compatibility, not generated hand identity;
- do not enable hand-body native contact by default;
- update object classification so side-hand layers are rejected as ROCK-generated hand bodies;
- update matrix registration, drift checks, logs, and tests to be side-aware.

## Validation Targets

Policy and config tests:

- source defaults make world soft contact the only enabled production mode;
- stale hand-hand, hand-body, and hand-weapon config keys cannot re-enable removed runtime behavior;
- `bSoftContactWorldEnabled` still disables/enables world soft contact;
- `sSoftContactWorldShapeCastFilterInfo` controls only world soft-contact fallback casts;
- removed non-world config keys are absent from source reads if the keys are deleted;
- stronger-owner states still clear world soft-contact visual authority.

Source-boundary tests:

- `SoftContactRuntime` no longer calls `solveShapeAgainstWeapon` in production;
- `SoftContactRuntime` no longer captures body skeleton snapshots for body soft contact;
- synthetic hand-hand and hand-body solve paths are removed or test/debug-only;
- non-world soft-contact candidates cannot reach `applyExternalHandWorldTransform`;
- soft-contact runtime does not allocate per-frame shape vectors for removed paths;
- no native hand-hand, hand-body, or hand-weapon evidence is consumed by `SoftContactRuntime`.
- fallback world casts use `rockSoftContactWorldShapeCastFilterInfo`, not selection tuning.
- debug snapshots expose soft-contact source kind.

World-contact tests:

- world surface policy covers the intended static/support/blocker layers;
- non-world dynamic props are not treated as world blockers unless explicitly allowed;
- cached-plane release clears when penetration ends;
- tangent drift limit prevents sticky wall contact;
- haptic edge fires only for world contact and respects cooldown/min approach speed.

Runtime profiler checks:

- compare `softContact` timing before and after removing non-world solves;
- compare `nativeContactCallbacks` to confirm no new hand-hand or hand-body native pair load was introduced;
- compare `handColliders`, `weaponCollision`, and `bodyColliders` to confirm cleanup did not move work into other hot paths.

In-game checks:

- free hand against walls and tables produces stable visual stop;
- free hand against terrain/ground and invisible blockers works where the expanded world policy allows it;
- hands do not soft-collide with each other;
- hands do not soft-collide with the equipped weapon;
- hands do not soft-collide with body colliders;
- weapon support grip, dominant weapon authority, grabbing, pulling, and held-object interactions do not fight soft contact;
- no sticky hand remains after leaving a world surface.

## Preferred Implementation Order

1. Freeze production behavior to world-only in config/source defaults.
2. Remove or production-disable hand-hand, hand-body, and hand-weapon solve paths.
3. Clean up logs, debug snapshots, config reads, and docs to stop advertising unsupported modes.
4. Harden world surface acceptance and cached-plane behavior.
5. Validate world-only runtime behavior in game.
6. Handle callback hot-path classification only before any future task enables more native pairs.
7. Treat the side-hand layer split as a separate deferred task, not a soft-contact requirement.

## Open Design Questions

- Answered: non-world config keys were deleted from source and packaged/default INIs. Persisted stale keys are ignored because the loader no longer reads them.
- Answered: the accepted world/support layer set is static, animstatic, transparent, trees, terrain, ground, transparent small, invisible wall, transparent small anim, stair helper, avoid box, and collision box.
- Watch in runtime: whether world correction needs smoothing after the non-world candidate competition is removed.
- Whether the side-hand layer split is still needed for any non-soft-contact feature. If yes, plan it separately after callback classification prework.
