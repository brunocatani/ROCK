# ROCK / PAPER Provider Boundary

ROCK no longer owns physical reload runtime behavior. It remains the provider of
hand physics, weapon collision, FRIK-backed skeleton evidence, generic external
body contact reporting, and offhand/support-grip arbitration. PAPER is the sole
owner of reload state, profiles, native ammo gates, reload visuals, reload
bodies, Papyrus/config behavior, and diagnostics.

ROCK is the root of the plugin stack. PAPER is a hard ROCK/FRIK-dependent module
that consumes this provider boundary; it is not designed as an independent mod.
Future SCISSORS/PLANCK work should use the same root-stack model.

## ROCK Provides

- `ROCKAPI_GetProviderApi`, a versioned C ABI for PAPER and future external
  consumers.
- Fixed-size POD provider snapshots with frame index, world pointers, FRIK/menu
  readiness, weapon form/generation, hand transforms, hand body IDs, weapon body
  IDs, hand state flags, offhand reservation, and registered external-body count.
- Weapon contact and evidence queries from ROCK's weapon collision system.
- Generic external body registration and hand-contact snapshots keyed by owner
  token, body ID, role, generation, contact policy, and owner hand.
- Offhand interaction reservation with `Normal`, `ReloadReserved`, and
  `ReloadPoseOverride`. Support grip and two-handed grip consume this generic
  reservation instead of checking reload state.
- Public `ROCKAPI_GetApi` v4 for non-reload hand/object callers. Hidden legacy
  v3 reload slots remain only as idle/false binary compatibility stubs.

## ROCK Does Not Provide

- Reload profiles or profile authoring.
- Native reload/ammo hooks.
- Native reload/ammo offsets in active ROCK code.
- Reload visual capture/restore.
- Reload body creation or reload-specific metadata publication.
- Reload Papyrus config behavior or holotape commands.
- Forwarding reload API behavior to PAPER. Legacy ROCK reload API stubs report
  idle/false only.
- Reload ESP, Papyrus source/PEX, authoring holotape, terminal records, or
  reload authoring/debug meshes.

## Cleanup Boundaries

ROCK clears provider-owned external registrations and offhand reservations when
physics shuts down, the skeleton is destroyed, the world changes, or the provider
instance is cleared. PAPER clears its owner token during reload cleanup and
dependency loss, while ROCK still enforces a provider-side cleanup floor.

The active ROCK INI keeps only hand, grab, weapon collision, support-grip, native
melee, selection, and generic debug settings. ROCK no longer ships reload
Papyrus scripts, reload ESP records, reload meshes, or reload authoring config
in its package.

## HIGGS Reference Pattern

HIGGS exposes stable callback and hand/weapon body access through its plugin API
instead of allowing other mods to patch its internals. ROCK follows that same
shape for provider evidence, while reload semantics live in PAPER so collision
and hand physics remain reload-agnostic.
