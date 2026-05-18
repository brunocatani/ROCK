# First-Person Attach Root Plan

This plan exists because the current generated weapon collider path treats the first `Weapon` node under `firstPersonSkeleton` as both the transform drive root and the visual source root. Ghidra verification against `Fallout4VR.exe.unpacked.exe` shows that FO4VR native attach builds a generated weapon package separately and stores its root in `ActorEquipData::SlotData::node` before/while attaching it under `Weapon` or `WeaponLeft`. The correct fix is to keep the stable `Weapon` node as the collider drive root, but add native equip-slot package roots as visual source witnesses and log enough per-root evidence to distinguish a stale parent tree from an unbuilt native package.

## Current Evidence

- `f4vr::getWeaponNode()` is a F4VR-CommonFramework helper that only returns `findNode(getPlayer()->firstPersonSkeleton, "Weapon")`.
- ROCK calls that helper each game frame from `resolveEquippedWeaponInteractionNode()` and passes the result into `WeaponCollision::update()`.
- `WeaponCollision::makeGeneratedWeaponMeshRootCandidates()` currently scans:
  - `firstPersonSkeleton:Weapon`
  - `PlayerNodes.primaryWeapontoWeaponNode`
  - `PlayerNodes.primaryWeaponOffsetNode`
  - `updateWeaponNode`
- The native attach path stores the generated weapon package root at `ActorEquipData::SlotData::node`, matching offset `equipData + 0x10 + slotIndex * 0x58 + 0x30`.
- The package root is a visual source candidate, not necessarily the same node as the `Weapon` attach parent.

## Implementation Stages

### Stage 1 - Equip Slot Visual Root Probe

Add a helper in `WeaponCollision.cpp` that enumerates matching equipped weapon slots and collects non-null `SlotData::node` values.

Required behavior:
- Read both first-person `player->playerEquipData` and actor/current-process equip data, matching the same weapon form and instance-data witness already used by the equipped identity code.
- Return root candidates only; do not mutate native equip state.
- Label candidates distinctly, for example:
  - `PlayerEquipData.slot[N].node`
  - `ActorEquipData.slot[N].node`
- Deduplicate by raw node pointer using existing candidate dedup logic.
- Preserve `updateWeaponNode` as the transform drive root for generated hull conversion.

### Stage 2 - Native Lookup Comparison Probe

Add a diagnostic-only native lookup comparison for `Weapon`.

Required behavior:
- Use `RE::BSUtilities::GetObjectByName(firstPersonSkeleton, "Weapon", true, true)` if the include and ABI are already available through CommonLibF4VR.
- Treat this as a candidate only after validating it produces visible triangles and is not a duplicate of the local `findNode()` result.
- Log whether native lookup and local lookup return different pointers.
- Do not use `dontAttach=false` in production code unless separately verified; `false` may materialize or mutate attachment state.

### Stage 3 - Root Completeness Logging

Expand generated-source logging so in-game traces can prove which root owns the missing part.

For each root candidate, log:
- label
- pointer
- node name
- parent pointer/name if cheap and safe
- direct child count when the root is a `NiNode`
- visited `NiTriShape` count
- accepted visible/extractable triangle count
- accepted hull count

Exit criterion:
- A captured bad workbench case shows whether the missing part exists under `SlotData::node`, native `GetObjectByName`, or only appears after a later native attach/update.

### Stage 4 - Candidate Integration

Integrate matching `SlotData::node` roots into `makeGeneratedWeaponMeshRootCandidates()`.

Required behavior:
- Keep `firstPersonSkeleton:Weapon` first as the drive/parent witness.
- Add equip-slot package roots before broad PlayerNodes roots so the generated native package is evaluated directly.
- Continue merging all valid candidate roots; do not make candidates compete.
- Existing duplicate `NiTriShape` suppression remains pointer-based.

### Stage 5 - Completeness Gate

Update the rebuild/publish gate only after Stage 3 proves what is stale.

If `SlotData::node` contains the missing part:
- publish from the merged candidate source set once stable-source frame requirements pass.
- no native remap is needed for that failure mode.

If `SlotData::node` also lacks the missing part:
- keep the equipped instance pending.
- extend the late-source probe/generation-settle logic so a stable but incomplete source does not become the final collider generation.
- do not use OMOD/index remapping as a geometry substitute.

## Test And Verification Gates

- Source-boundary test must continue to forbid broad native refresh calls outside the allowed remap runtime.
- Add or update source-boundary coverage so `SlotData::node` is allowed as a read-only visual source candidate.
- Add unit/source tests that require:
  - `makeGeneratedWeaponMeshRootCandidates()` includes equip-slot node candidates.
  - candidate labels identify equip-data source and slot index.
  - native `GetObjectByName` lookup, if added, uses `dontAttach=true`.
- Run:
  - `cmake --preset custom-tests`
  - `cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
  - `ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- After code changes, run the fast plugin build:
  - `cmake --preset custom-fast`
  - `cmake --build build-fast --config Release --target ROCK -- /m`

## In-Game Validation Checklist

- Equip a known affected weapon.
- Record candidate-root logs before a workbench modification.
- Modify one weapon part that reproduces the stale/missing part behavior.
- Confirm whether the newly missing part appears under:
  - `firstPersonSkeleton:Weapon`
  - native `GetObjectByName(..., "Weapon", true, true)`
  - matching `SlotData::node`
  - PlayerNodes roots
- Modify another part and confirm the previous missing part moves into the candidate set.
- Verify generated hull count and source signatures follow the visible geometry, not only OMOD/index witness changes.

## Non-Goals

- Do not replace `getWeaponNode()` as the transform drive root without a separate transform-authority study.
- Do not force native remap for every stale source.
- Do not build colliders from OMOD/index data.
- Do not add a fallback that silently keeps old and new source systems active without logging and cleanup.
