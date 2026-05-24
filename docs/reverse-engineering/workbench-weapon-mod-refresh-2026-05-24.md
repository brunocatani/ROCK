# Workbench Weapon Mod Refresh Path

Date: 2026-05-24
Project: ROCK
Source used: local ROCK source/logs, local CommonLibF4VR headers, Ghidra MCP on the loaded FO4VR executable
Confidence: high for the native workbench call path, medium for runtime root-cause until confirmed with a live hook

## Question

When a weapon mod is changed in the weapon workbench, what native path updates the modded weapon state, and does that path repopulate the player first-person/root weapon graph nodes that ROCK scans for generated weapon collision?

## Verified Native Path

`ExamineMenu::SwitchMod` at `0x140b3d6f0` calls the menu's `AttachModToPreview` virtual slot.

`ExamineMenu::AttachModToPreview` at `0x140b3db40`:

- resets/highlights menu preview state;
- calls `CreateModdedInventoryItem` at `0x140b3ece0`;
- retrieves or creates a `BGSObjectInstanceExtra` from the selected inventory stack;
- applies the selected mod through `_anon_74BA5B6D::ApplyChangesFunctor::vfunction1` at `0x140b53c8c`;
- refreshes the ExamineMenu preview 3D through `ShowCurrent3D` at `0x140b3c490`.

`ExamineMenu::BuildConfirmed` at `0x140b3a270` applies the same `ApplyChangesFunctor` to the real inventory source when the mod is built.

The functor updates inventory extra data:

- removes stale extra type `0x35` (`BGSObjectInstanceExtra`) and type `0xBA` (`ExtraInstanceData`);
- creates or updates `BGSObjectInstanceExtra`;
- calls `BGSObjectInstanceExtra::AddMod` at `0x14003cba0`;
- calls `BGSObjectInstanceExtra::RemoveInvalidMods` at `0x14003d270`;
- regenerates cached instance data through `0x14008ae70`.

The workbench inventory path did not show a call to `BGSObjectInstanceExtra::AttachModToReference` at `0x14003d680`.

## Important Contrast

`BGSObjectInstanceExtra::AttachModToReference` at `0x14003d680` does perform reference-facing refresh work after changing the extra data. The function removes cached instance data, regenerates it, and then takes a loaded-reference refresh path:

- actor/reference path: virtual call at `+0x3B8` with argument `0x37`;
- non-actor reference path: virtual call at `+0x440`, plus loaded-ref update flags and cell/reference refresh handling.

That is not the path used by the normal workbench inventory mod change found above.

## ROCK Log Evidence

ROCK's telemetry shows the first-person `Weapon` node remains a reused root while its child package changes. Sample observed weapon subtree shapes from `C:\Users\SENECA\Documents\My Games\Fallout4VR\F4SE\ROCK*.log`:

- `Weapon  (0009983B)`: 30 subtree nodes, 20 NiNodes, 10 TriShapes.
- `Weapon  (0009983B)`: 33 subtree nodes, 21 NiNodes, 12 TriShapes.
- `Weapon  (0009983B)`: 34 subtree nodes, 21 NiNodes, 13 TriShapes.
- `Weapon  (FE023EA0)`: 46 subtree nodes, 33 NiNodes, 13 TriShapes.

The `AnimObject*` flattened transform rows are present, but the telemetry repeatedly reports `refNode='(null)'/0x0` and node-tree dumps report `matches=0` for those targets. That means the animation channel exists, but no registered `NiAVObject`/TriShape is available under the first-person/root trees for ROCK to use as a collider source.

## Implication For ROCK

This is not a frame-delay problem in ROCK's generated weapon collision path. ROCK sees the equipped weapon identity and visual hash change, but the native tree it can legally scan is already incomplete.

The likely stale point is the workbench path's inventory-data refresh: it regenerates object-instance data and menu preview 3D, but does not force the player first-person/root graph to repopulate attachment slot refNodes for the updated weapon. A future fix should target the post-workbench/equipped-weapon refresh boundary, not add more settle frames to generated weapon collision.

Candidate next verification:

- Hook or instrument the post-build workbench path and the `BGSOnPlayerModArmorWeaponEvent` sink to confirm event timing relative to the first-person weapon graph.
- Identify a safe native first-person/equipped-weapon rebuild function before attempting to force refresh. Do not call the reference `AttachModToReference` path for inventory items without a separate ownership/lifetime review.

## Failed ROCK Boundary

The first implementation used the actor branch of `BGSObjectInstanceExtra::AttachModToReference` without calling the mod-attach function itself. Ghidra disassembly showed the actor branch ending in a virtual call at `+0x3B8` with argument `0x37`, which maps to `TESObjectREFR::Set3DUpdateFlag(RESET_3D_FLAGS)` with model, skin, head, scale, and skeleton bits.

Runtime testing on 2026-05-24 showed that this path did execute after the workbench change, but it did not fix the stale weapon-part registration. Logs showed ROCK scheduling and applying the refresh, then rebuilding generated collision from the same incomplete first-person weapon graph. The dirty-flag path was therefore removed from production code.

The earlier candidate `Actor::HandleItemEquip(false)` was also rejected for this bug because live behavior showed equip/unequip does not advance the missing part registration. The observed failure is one mod change behind: a later workbench change registers the previous missing part while the newly changed part becomes the missing one.

## Corrected ROCK Boundary

Ghidra verification identified a narrower native boundary inside `ExamineMenu::BuildConfirmed`: after the inventory mod mutation, the function calls the same menu visual-refresh virtual used by workbench preview changes.

Verified callsite:

- virtual call address: `0x140B3AA24`;
- module offset used by ROCK: `0xB3AA24`;
- instruction bytes: `FF 90 20 01 00 00`;
- slot called: `ExamineMenu` vtable offset `+0x120`;
- call arguments observed in disassembly: `RCX=this`, `DL=1`.

ROCK hooks only that six-byte native callsite, validates the expected bytes before patching, lets the natural native refresh run first, and replays the same `ExamineMenu` visual refresh once when the equipped mod-instance signature changed relative to the last stable non-menu equipped weapon state.

The hook publishes a refresh epoch only when it performs that replay. The `WeaponVisualGraphRefreshCoordinator` consumes the epoch after menu blocking clears and a live weapon node is available, then invalidates generated weapon collision with reason `workbench-native-refresh-replay` so the next scan rebuilds from the graph after the replayed native registration pass.

Do not trigger this refresh from first equipped weapon observation or ordinary equip/unholster signature changes. Runtime testing showed that broad trigger can hit the normal first-person transition and make the weapon disappear.

Remaining runtime risk: this still needs live validation to confirm that replaying the exact workbench native refresh closes the observed N-1 graph-registration gap.
