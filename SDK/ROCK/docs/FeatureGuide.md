# Feature guide

This guide groups the API by what a mod can build. Read `RuntimeContract.md` before using any writer or authority surface.

## Observe hands and interaction

Use frame snapshots, primary/offhand queries, hand frames, presented frames/poses, interaction state, collision availability, semantic contacts, and provider events to build gesture/state telemetry, contextual UI, contact-driven feedback, or input-aware interactions.

Physics hand frames and presented visual poses are intentionally different surfaces. Use physics frames for collision/control decisions and presented poses for visuals.

## Observe weapons

Use classification, evidence catalogs and point clouds, emitter snapshots, equipped-weapon state, grip state, composition, part poses, drive results, scope sight state, and semantic target resolution to build sights, emitters, attachment-aware logic, weapon diagnostics, or interaction affordances.

`weaponGenerationKey` binds all weapon-local identity. Rebuild caches whenever it changes.

## Control weapon parts

Weapon part targets declare what may be interacted with; drive targets request bounded transforms or semantic motion. Read target resolution first, refresh drives as leases, and inspect application results instead of assuming the requested transform was accepted.

Nonexclusive target flags allow compatible consumers to describe overlapping observations without taking hidden ownership. Actual handling/drive authority remains explicit.

## Integrate external physics bodies

Register body IDs owned by your mod, preferably in stable scope tokens. ROCK then emits owner/scoped contacts involving hands, weapons, held objects, body colliders, or other classified sources. Clear the scope before destroying/recycling bodies.

The provider does not own or keep your bodies alive. Registration is contact-routing metadata, not object lifetime transfer.

## Issue interaction commands

Force grab, force release, and thrown drop are queued through ROCK's canonical interaction paths. Commands validate owner, capability, hand, target identity, distance/generations, and current hand state before execution. Admission is not completion: retain the command ID and poll its terminal result.

Force grab is near-range and targets a live loose object/weapon. Force release defaults to a gentle physical drop. Thrown drop can use captured controller motion. Supplied Havok velocities are finite-checked but trusted and are not clamped.

## Reserve/suppress input

Hand input suppression and offhand reservation are owner-scoped leases. Publish only the exact flags/mode you need, refresh while active, inspect effective state, and clear/release immediately when the feature stops.

Raw wand state exposes sample metadata so a consumer can distinguish current input from stale/unavailable input. Native Pip-Boy/VATS/V.A.N.S. suppression remains explicit and separately feature-gated.

## Drive animation and presentation

Native animation authority, phase callbacks, runtime publication, hand visual authority, presented pose readback, and handling authority form a coordinated animation surface. Acquire the narrow authority first, write only under `VisualWriteAllowed`, use generation guards, and deterministically clear every authority/publication.

Exact-hand weapon requests are valid only for the currently equipped weapon and while the caller owns handling authority. Left-hand requests additionally require the ambidextrous handoff policy.

## Build touch mechanisms and climbing

Touch-grab targets support:

- `FixedAnchor`: attach a contacting hand to an exact body or wildcard surface without changing body motion type;
- `LimitedHinge`: expose bounded rotational motion on a named live body;
- `LimitedPrismatic`: expose bounded linear motion on a named live body.

Targets are replaced transactionally per scope and refreshed as leases. Exact-body registrations resolve before wildcard registrations. One wildcard descriptor owns at most one resolved body; publish separate right/left descriptors for simultaneous two-hand surface interaction.

Before a script or native system moves a held mechanism, request yield and wait until state reaches `Yielded`. Republishing with a new target generation re-arms a yielded/invalidated descriptor.

## Query the world

World raycasts are owner-callback-only, bounded, and use ROCK's validated filter policy. The current extended limits publish eight raycasts per owner per frame and the request range is capped at 8192 game units. Use them for short diagnostic/sight/interaction probes, not bulk scene queries.

## Publish diagnostics

Debug overlay lines/text are copied into a bounded owner publication and rendered through ROCK's stereo overlay. Collider visualization override selects one exact body from the current complete weapon body catalog and refreshes as a rolling lease. These surfaces are useful for development tools and user-facing diagnostics without building a second VR renderer.
