# Weapon Visual Remap Reset

ROCK now treats the workbench stack write as the native remap authority because the visible weapon tree can stay stale after the equipped inventory stack has already been mutated. `AttachModToReference` remains useful as secondary evidence, but it is not sufficient for the workbench path and must not authorize `QueueAttachWeapon`. The chosen approach patches `ApplyChangesFunctor::WriteDataImpl` through its validated vtable entry, records the equipped player stack after the original mutation returns, and lets generated collision publish only after the visual source actually changes.

This avoids broad equip or visual refresh calls. Inventory equips and game-load baselines can identify collision ownership, but native remap is limited to the post-mutation `workbenchApplyChanges` witness or a future source that proves the same stack boundary.
