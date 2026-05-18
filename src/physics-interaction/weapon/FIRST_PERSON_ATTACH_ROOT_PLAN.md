# First-Person Attach Root Plan

This plan exists because the current generated weapon collider path treats the first `Weapon` node under `firstPersonSkeleton` as both the transform drive root and the visual source root. Ghidra verification against `Fallout4VR.exe.unpacked.exe` shows that FO4VR native attach builds a generated weapon package separately and stores its root in `ActorEquipData::SlotData::node` before/while attaching it under `Weapon` or `WeaponLeft`. The correct fix is to keep the stable `Weapon` node as the collider drive root, but add native equip-slot package roots as visual source witnesses and log enough per-root evidence to distinguish a stale parent tree from an unbuilt native package.

## Implementation Status

- 2026-05-18 correction: The equip-slot package root is not a safe transform authority. It can expose useful read-only source geometry, but using it as the generated collision drive root when `firstPersonSkeleton:Weapon` is absent creates duplicate collider sets, wrong orientation, and stale colliders after unequip because it bypasses the real first-person weapon attachment lifecycle. The production fix restores the real `Weapon` node as the only drive root and keeps equip-slot roots as source witnesses only after that drive root exists.
- 2026-05-18: Stage 1, Stage 3, and Stage 4 are implemented in `WeaponCollision.cpp`. Generated weapon candidate discovery now includes matching first-person and actor `ActorEquipData::SlotData::node` roots, prefers the bounded authoritative equipped-instance witness when matching those slots, deduplicates them by node pointer, folds them into the visual key, and logs parent pointer/name plus direct child counts for every candidate.
- 2026-05-18: AMCAR/Thanatos test logs showed `firstPersonSkeleton:Weapon` can be absent entirely while the equipped identity is valid. The follow-up in-game test proved that substituting a matching equip-slot package root as the generated collision drive root is invalid. Missing `Weapon` remains an attach/drive-root publication bug to diagnose, not a condition where ROCK should create or drive weapon colliders from another root.
- Stage 2 remains diagnostic-only and is not implemented in production code. No native lookup or native refresh path was added by this change.
- Stage 5 remains dependent on in-game logs from an affected weapon with a valid first-person `Weapon` drive root. The next decision point is whether the missing barrel/handguard appears under the equip-slot package root while the drive root exists, or whether the native package itself is incomplete.

## Current Evidence

- `f4vr::getWeaponNode()` is a F4VR-CommonFramework helper that only returns `findNode(getPlayer()->firstPersonSkeleton, "Weapon")`.
- ROCK calls that helper each game frame from `resolveEquippedWeaponInteractionNode()` and passes the result into `WeaponCollision::update()`.
- `WeaponCollision::makeGeneratedWeaponMeshRootCandidates()` currently scans:
  - `firstPersonSkeleton:Weapon`
  - `PlayerNodes.primaryWeapontoWeaponNode`
  - `PlayerNodes.primaryWeaponOffsetNode`
  - `updateWeaponNode`
- The native attach path stores the generated weapon package root at `ActorEquipData::SlotData::node`, matching offset `equipData + 0x10 + slotIndex * 0x58 + 0x30`.
- The package root is a visual source candidate, not the generated collision transform authority.

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
