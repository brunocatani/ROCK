# Weapon Workbench Mutation Boundary

This mapping exists because the stale visual-source logs proved the collider pipeline was waiting correctly while the mutation witness never appeared. The previous candidates, `ApplyChangesFunctor::WriteDataImpl`, `BGSInventoryItem::ModifyModDataFunctor::WriteDataImpl`, and `AttachModToReference`, all installed but did not fire during the tested workbench changes, so the next hook must come from the actual FO4VR object-instance mutation boundary rather than another visual refresh or equip path.

## Verified Runtime Evidence

- Latest log loaded the new ROCK binary and installed both stack-writer vtable hooks.
- `ApplyChangesFunctorWriteDataImplPostCall`, `ModifyModDataFunctorWriteDataImplPostCall`, and `AttachModToReferencePostCall` remained at zero during the tested changes.
- The collision pipeline observed stale visual source and preserved cached generated bodies, but native remap stayed blocked as `nativeRemapBlockedNoWorkbenchWitness`.

## Ghidra Mapping

- `BGSInventoryItem::FindAndWriteStackData` resolves at `0x1401B0510` and calls the supplied stack write functor through the stack wrapper.
- `ApplyChangesFunctor::WriteDataImpl` resolves at `0x140B53B70`; it mutates object-instance mod data and then installs the resulting `BGSObjectInstanceExtra` on the stack.
- `BGSInventoryItem::ModifyModDataFunctor::WriteDataImpl` resolves at `0x1401B02E0`; it writes/removes OMOD entries on stack-owned object-instance extra data.
- `BGSObjectInstanceExtra::AddMod` resolves through `REL::RelocationID(1191757, 2189025)` to `0x14003CBA0`.
- `BGSObjectInstanceExtra::RemoveMod` resolves through `REL::RelocationID(1136607, 2189027)` to `0x14003D020`.
- The direct `BGSObjectInstanceExtra` path at `0x14003D680` calls `AddMod` and performs reference visual invalidation afterward.
- The `0x1409C...` workbench/menu cluster contains both stack-functor calls and direct `BGSObjectInstanceExtra` add/remove calls, so the low-level object-instance add/remove boundary is the common post-mutation evidence point.

## ROCK Boundary

ROCK hooks the low-level add/remove functions after original execution and records a bounded `workbenchApplyChanges` signal that includes the exact `BGSObjectInstanceExtra` address. The pending equipped-stack scan must then match that address on the player's equipped weapon stack. Add transactions also require the changed OMOD to be active; remove transactions preserve the removed OMOD form but do not require it to remain active.

This keeps native visual remap authorization tied to real equipped-stack mutation evidence while avoiding broad equip, UI, or visual-refresh calls.
