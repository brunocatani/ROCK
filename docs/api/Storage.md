# Storage (interface 15, major 2)

Header: `include/ROCK/Storage.h`. Physical storage for holster, backpack and
stash mods: claims on stored references, held-item contact suppression,
highlights, inventory-item displays with authored grips, shoulder routing and
stash conversion.

Interface `Storage`, major 2, minor 0, discovered through
`ROCKAPI_QueryInterfaceV1`. The former `ROCK_Storage*`,
`ROCK_InventoryDisplay*` and `ROCK_StashConversion_QueryV1` exports are
removed. Permissions: `Read` (1), `Write` (2), `Callbacks` (4). Every call
runs on the ROCK frame-owner thread.

## Claims (Write; release owner only)

`claim(owner, target, lease)` reserves a reference for
`kClaimReservationFrames`; the first claimant wins (`OwnerConflict`
otherwise), and repeating the claim renews it with the same lease. A held
target names the carrying hand and its Grab `gripSequence`; `Hand::None`
requires that neither hand holds it. `commit(owner, lease)` after physical
release makes the claim stored; disable or transfer the native item only
after `Ok`. `release(owner, lease)` after a withdrawal or a cancelled
deposit. Stored claims survive generation changes, but owner retirement
clears runtime leases: restore claims from your co-save.
`getClaimant(owner, referenceFormId, claimant)` (Read) reports the owner of
a live claim, or 0. At most `kMaxClaims` claims.

## Held items (Write)

- `suppressHeldContacts(owner, target)` stops the held reference's bodies
  colliding for three frames; renew each frame. Losing the exact grab,
  generation or owner restores them. `clearHeldContacts(owner)` ends it.
- `extractEquipped(owner, request, reference)` (released ROCK) places the
  native-equipped weapon in the world and returns its reference
  (`RequestQueued`); claim it, then retry `commit` while `Busy`.
- `draw(owner, request, command)` uses Grab's command protocol; firearm
  delivery also prepares the authored firing grip. `NotReady`: retry on a
  later frame.

## Highlights (Write; clear owner only)

`setCandidate` pulses an item held by a hand; `setSelection` shows a steady
selection on `nodeName` under a reference the other hand holds, while the
selecting hand is empty and `storedReference` is your committed claim. Both
are three-frame leases per hand; `Busy` yields to other native selection
owners, and a candidate pulse wins over a selection. `clearCandidate` and
`clearSelection` end them.

## Displays (Write to set, Read to query)

`setDisplay(owner, display)` registers a caller-owned visual of an inventory
item (a fresh Weapon `InventoryWeapon` capture) beneath the player skeleton
or a reference a ROCK hand holds. Refresh 1 to `kMaxLeaseFrames` frames
every frame and clear with `clearDisplay` before detaching. ROCK fits a
collider from the visible meshes that only empty physical hands touch;
`NoCollision` keeps the display visual-only. `Ok` means accepted, not that
the collider is ready. At most `kMaxDisplays`; `CapacityFull` never evicts
another owner.

- `getDisplayContact(owner, hand, contact)`: the freshest contact of an
  empty hand with one of your displays; `valid` 0 means none. Contacts
  expire after two frames.
- `getAuthoredGrip(owner, request, pose)`: the authored firing or support
  seat on a display, with copied hand and finger poses and the tracked
  wand in display and room coordinates. `request.sample` must be the
  current publication. `valid` 0 means no complete authored pose yet; no
  guessed grip is returned. The consumer owns pull thresholds and delivery.

## Shoulder routing (Write)

`setShoulderRouting(owner, routing)` suppresses ROCK's built-in shoulder
stash and retrieval for the hands in `handMask` (right 1, left 2; 0 clears)
for 1 to `kMaxLeaseFrames` frames. `clearDisplaysAndRouting(owner)` clears
your displays and routing.

## Stash conversion (Callbacks)

`registerStashConverter(owner, converter, userData)` installs one converter
per owner (`kMaxConverters` in total; null unregisters). ROCK calls it on the
frame owner after physical detachment and before native inventory
activation. Return `Consumed` with the inventory form and count, `Reject`, or
`Pass` only for items you do not recognize. The converter must not call
ROCK.

## Records

| Record | Used by | Fields |
| --- | --- | --- |
| `Target` | `claim`, `suppressHeldContacts` | `reference` (`ReferenceKey`), `hand`, `gripSequence` |
| `EquippedExtraction` | `extractEquipped` | `hand`, `weaponFormId`, the three guard generations |
| `Candidate` | `setCandidate` | `hand`, `reference`, `expectedBase`, the three guard generations, `gripSequence` |
| `Selection` | `setSelection` | `hand`, `parentReference`, `storedReference`, the three guard generations, `nodeName[64]` |
| `Display` | `setDisplay` | `displayId`, `item` (`weapon::InventoryWeapon`), `parentReference`, `leaseFrames`, `flags` (`DisplayFlag::NoCollision`), `nodeName[64]` |
| `Contact` | `getDisplayContact` | `sample`, `displayId`, `contactFrame`, `item`, `hand`, `valid` |
| `GripRequest` | `getAuthoredGrip` | `sample`, `displayId`, `hand`, `role` (`GripRole`: `Nearest`, `Firing`, `Support`) |
| `GripPose` | `getAuthoredGrip` | `sample`, `displayId`, `poseSequence`, `item`, `valid`, `hand`, `role`, `fingerMask`, `displayWorld`, `handInDisplay`, `fingerLocals[kFingerLocalTransformCount]` (15), `controllerInDisplay`, `controllerInTracking`, `gripInDisplay`, `distanceGame`, `acquisitionRadiusGame` |
| `ShoulderRouting` | `setShoulderRouting` | `handMask`, `leaseFrames`, the three guard generations; at most `kMaxRoutingOwners` owners |
| `StashResult` | stash converter return | `decision` (`StashDecision`: `Pass`, `Reject`, `Consumed`), `inventoryFormId`, `count` |
