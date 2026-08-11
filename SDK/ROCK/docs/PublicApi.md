# Public API overview

ROCK exposes one append-only V1 provider table from `ROCK.dll`. Its purpose is to let independent FO4VR F4SE plugins observe and cooperate with the canonical hand/object/weapon runtime without sharing C++ implementation objects or creating duplicate physics/animation ownership paths.

## What the SDK exposes

### Runtime and hand state

Coherent frame/lifecycle snapshots; primary/offhand identity; physics and presented hand frames; interaction phase/target/held-body state; provider event cursors; raw wand state; input suppression state; semantic hand contacts; generated player collider descriptors; and hand collision availability.

### Weapon state

Compact and detailed weapon collision evidence; local point clouds; point queries; semantic classification; complete emitter state; grip/muzzle state; equipped transition state; installed composition; scope/sight state; weapon-part resolution, poses, and applied drive results; authored grip poses; and exact current weapon/body generation identity.

### External integration

Owner/scoped external body registration and enriched contact cursors let a consumer route contacts against bodies it owns. Registration transfers no lifetime or mutation ownership.

### Interaction and input control

Queued force grab, physical release, thrown drop, and cancellation; per-hand input suppression leases; offhand reservation leases; semantic weapon-part targets/drives; and exact-hand equipped-weapon requests under explicit handling authority.

### Animation and visuals

Native animation authority, animation phase callbacks, native animation runtime publication, hand/finger visual authority, equipped-weapon handling policy, bounded stereo debug overlays, and exact collider visualization focus.

### Touch mechanisms and world queries

Scoped fixed-anchor, limited-hinge, and limited-prismatic touch targets support climbable surfaces and physical controls. Owner-callback-only world raycasts provide a bounded provider-filtered query path.

## Discovery model

Use `RockProviderApi::initialize`. It prefers the safe immutable descriptor, verifies requested API/table extent, and exposes the returned table through `RockProviderApi::inst`. Then query limits/features and register a consumer for capabilities.

The three layers serve different purposes:

- table extent prevents reading a function pointer beyond an older table;
- feature bits describe behavior implemented by the loaded provider;
- granted capabilities authorize this owner to call stateful families.

## Read versus write surfaces

Value snapshots and synchronized discovery queries do not give mutation authority. Stateful publications and commands always require a registered owner and capability. Live scene/physics readbacks and all stateful writes are game-thread-only and belong in an owner callback unless the header explicitly says otherwise.

## Scope and replace semantics

Most multi-entry writer surfaces replace the caller's current set transactionally. Scope tokens let one consumer own multiple independently replaceable sets. A zero-count set is commonly an explicit scope clear; the dedicated clear function is preferred when it makes intent clearer.

## Safety summary

- Fail closed when lifecycle gates or provider state are absent.
- Bind requests to current generation identity.
- Never retain/dereference legacy pointer witnesses.
- Respect returned limits and copy counts.
- Treat rolling leases as per-frame/short-lived ownership.
- Treat command admission separately from command completion.
- Clear scopes/authorities before destroying their backing state.
- Unregister owner callbacks before unregistering the consumer.

Continue with `GettingStarted.md`, `RuntimeContract.md`, and `ApiIndex.md`.
