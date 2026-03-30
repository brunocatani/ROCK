# Weapon Visual Remap Ghidra Notes - 2026-05-09

## Why This Path Exists

ROCK generates weapon colliders from the visible first-person weapon tree, but
manual workbench mod changes can leave that tree one transaction behind the
equipped instance data. Waiting or observing more frames does not repair the
tree; the visible source stays stale until another weapon mod edit forces a
native attach update. The fix is to request the narrow native queued weapon
attach task for the already-equipped instance, then keep collision publication
blocked until ROCK observes a changed visible source.

## Verified FO4VR Chain

- `BGSObjectInstanceExtra::AttachModToReference` mutates the equipped instance
  mod data, then routes actor update flag `0x37`. That update path is not an
  immediate first-person weapon attach-tree rebuild.
- `Update1StPersonArm` at `0x140EF6280` is an arm/transform solver. It is not a
  source tree rebuild and must not be used as a generated-collider retry.
- `AIProcess::RequestLoadAnimationsForWeaponChange` at `0x140E8D090` and
  `AIProcess::SetEquippedItem` at `0x140E806B0` are broad equip/animation paths.
  They are intentionally not part of ROCK's remap fix.
- `TaskQueueInterface::QueueAttachWeapon` is FO4VR address `0x140DAB8F0`,
  address-library ID `916430`. It queues task `0x12`; the task dispatcher calls
  the actor weapon attach virtual with a copied `BGSObjectInstance` and
  `BGSEquipIndex`.
- `TaskQueueInterface::QueueRemoveWeapon` is task `0x13`; it is not used for
  this fix.

## ROCK Contract

- `WeaponVisualRemapRuntime` is the only ROCK weapon-module file allowed to
  resolve or call `QueueAttachWeapon`.
- `WeaponCollision` may call only the named ROCK wrapper,
  `requestCurrentFirstPersonWeaponVisualRemap`.
- The wrapper must reject any current equipped candidate that does not match
  the pending form, instance-data, and object-instance-extra witness that
  triggered stale visible-source handling.
- A missing first-person weapon visual node with a changed equipped-instance
  witness is treated as the same desync class. ROCK may queue the narrow remap
  for that pending witness before returning from the missing-visual branch,
  including the no-cached-body/no-cached-witness case.
- The collider builder does not change. Stale visible sources keep existing
  bodies active and pending, but they are not accepted as the new body bank.
- The remap runtime prefers `middleHigh->equippedItems`, then falls back to
  first-person and actor equip data when that list has not caught up. Every
  fallback candidate still must match ROCK's pending form, instance-data, and
  object-instance-extra witness before the native task is queued.
- ROCK remap candidate snapshots are non-owning. They may cache raw form,
  instance-data, object-instance-extra, and equip-index witnesses, but must not
  store copied `BGSObjectInstance` values or construct owning
  `BGSObjectInstanceT` locals because that gives ROCK destructor ownership over
  engine-managed `BSTSmartPointer` instance data. The native queue call uses a
  borrowed two-pointer ABI view populated from the matched witness.
- `middleHigh->equippedItems` is a preferred source only when its array storage
  is currently readable through the guarded helper. During first-person
  skeleton teardown/rebuild the array can be transiently empty or null, so ROCK
  falls back to first-person/actor equip data instead of range-iterating it.
- Missing visual roots do not queue native remap for the first no-cache
  equipped witness. A native remap requires a cached equipped witness or stale
  visible source evidence so startup acquisition does not call the attach task
  for unrelated weapons.
- Native remap attempts are bounded per pending equipped-instance witness.
  Duplicate stale frames do not spam native work; failed target acquisition
  backs off, queued tasks get an observation window, and repeated no-change
  results exhaust the remap path for that witness.
- Replacement bodies publish only after the visible source signature or durable
  material geometry changes and settles through the existing generated source
  lifecycle.
