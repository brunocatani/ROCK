# Bad Grab COL / Axis-Map Investigation

Date: 2026-05-31

Project: ROCK

Branch: `feature/ghidra-grab-motor-mapping`

Scope: explain why bad dynamic grabs show near-180 column-style mismatch and object-origin-relative axis-map behavior without repeating earlier `target_bRca` formula experiments.

Source authority:
- Current local ROCK source.
- Ghidra MCP on the loaded FO4VR executable image.
- Prior local note recovered from commit `81847c6`.

## Tracking Log

- Checked current branch/status, read current proxy-grab telemetry paths, and confirmed the live probes already separate relation input, atom bytes, and solver-effective body pose.
- Re-checked FO4VR stock ragdoll constraint layout in Ghidra, then traced current freeze/capture code to see what part of the grab frame is still object-origin-relative.
- Implemented the first narrow decomposition experiment: creation now keeps `transformBRotation` identity while `target_bRca` continues to carry the row-view proxy-in-BODY relation.
- Runtime follow-up from hot-swappable modes: `transformB=R,target=R` and `transformB=I,target=R` each fix opposite angular classes. The same face can flip from bad to perfect by swapping only the transform-B mode, and the effect is hand-independent.
- Added an explicit testing plan for a principled `mode 4`: split the proxy-in-BODY relation evenly so transform-B and target_bRca both receive the same half-angle rotation instead of using one of the two current extremes.

## Ghidra Findings

- Stock `hkpRagdollConstraintData` still initializes the atom chain from object `+0x20` via `0x1419b2910`.
- The type-19 ragdoll motor atom is still a `0x60`-byte block starting at atom `+0xa0`.
- `target_bRca` is still a 3x4 float block at ragdoll atom `+0x10`, with `0x10` stride per row and the 4th float used as padding/alignment. The constructor writes identity rows into that area.
- The ragdoll private runtime fields are still at ragdoll atom `+0x04` and `+0x06`. Stock enable helper `0x1419b1d20` writes `0x0090` and `0x0094`, which is the same solver-results-first convention ROCK already mirrors with `0x60` / `0x64` for its smaller custom chain.
- This re-check did not expose a hidden extra orientation field or alternate 4-value payload that would explain the bad-grab class by itself.

## Current Code Findings

- `freezeGrabAuthorityFrame()` in `src/physics-interaction/grab/GrabCore.h` freezes `pivotBConstraintLocalGame` from the BODY frame, but it does not derive rotation from that pivot.
- `frozen.proxyAuthorityBodyHandSpace.rotate` comes straight from `objectInGeneratedProxyLocalSpace(input.proxyWorld, frozen.desiredBodyWorld)`.
- `alignLocalPointInTransformToLocalTarget()` only changes translation. It keeps the captured rotation class untouched while moving the proxy-local BODY relation so the selected pivot lands on pivot A.
- `writeGrabConstraintCreationAtoms()` in `src/physics-interaction/grab/GrabConstraintMath.h` currently writes the same `proxyInBody.rotate` into both `transformBRotation` and `target_bRca`.
- Held updates only rewrite `target_bRca` and transform-B translation. `transformBRotation` stays frozen from grab creation for the entire held grab.
- Because of that, the frozen BODY relation is still fundamentally object-origin/orientation-relative: pivot choice can fix translation coherence, but it cannot repair a bad captured rotation basis.
- This matches the stable symptom pattern:
  - relation/pivot deltas can stay clean;
  - live angular error can still explode;
  - column-view or near-180 axis-map diagnostics then act as evidence that the captured orientation class was bad before the motor tried to solve it.

## Stronger Suspect From The New Screenshots

- The screenshots show `ATOM` and `RELINV` at effectively zero while `SOLVEREFF` is still huge.
- That means the pure relation reconstruction path is coherent, but the full constraint-frame reconstruction is not.
- The most likely remaining reason is that BODY-to-proxy relation is being encoded twice in two different atom roles:
  - once as `target_bRca`
  - once again as `transformBRotation`
- If that is true, grabs whose desired BODY-to-proxy relation is near a cardinal axis permutation will fail in a stable, repeatable way, which matches the screenshots much better than random motor weakness.

## New Mode-Flip Interpretation

- The hot-swappable experiment shows that `transformB=R,target=R` and `transformB=I,target=R` are not random good/bad modes. Each one is accidentally correct for a different orientation class.
- That means the remaining problem is very likely decomposition, not just target storage:
  - changing `transformBRotation` without changing `target_bRca` changes the represented solver pose
  - the fact that opposite angle classes prefer opposite modes implies both current splits are incomplete encodings of one underlying desired constraint relation
- This points to a missing adaptation between the captured proxy/BODY relation and the actual solver constraint-frame decomposition, especially the BODY-local B frame used by `transformBRotation`.

## Why The Bad Grab Looks Object-Origin Relative

- `objectInGeneratedProxyLocalSpace()` builds the local rotation by transforming the BODY world row axes into proxy-local space.
- The grip pivot is applied afterward as a translation correction only.
- So when a bad grab happens, the visual/logged mismatch can track the BODY origin orientation even if the selected grip point is correct. That is expected from the current capture path.

## Why Repeating The Old `target_bRca` Patches Is The Wrong Loop

- Earlier row/column/residual experiments changed how the already-captured relation was written into `target_bRca`.
- The current source already proves a different middle-layer risk remains: a wrong rotation class can be frozen into `proxyAuthorityBodyHandSpace` before `target_bRca` is even authored.
- In that case, a high column-style delta is a symptom of bad captured orientation, not proof that the atom needs another row/column rewrite.

## Next Probe Direction

- Focus on the capture boundary that produces `proxyAuthorityBodyHandSpace.rotate`, not another `target_bRca` formula change.
- Specifically compare, on good vs bad grabs:
  - `proxyFrameWorldAtGrab.rotate`
  - `grabBodyWorldAtGrab.rotate`
  - `frozenAuthorityFrame.desiredBodyWorld.rotate`
  - `frozenAuthorityFrame.proxyAuthorityBodyHandSpace.rotate`
- If the bad class is already present there, the fix belongs in proxy/body frame capture or authority-frame freeze, not in the ragdoll atom writer.
- The next narrow code experiment should be separate from the reverted target-only family: keep `target_bRca` as the row-view BODY-to-proxy relation, but stop freezing `transformBRotation` from that same relation. Test identity transform-B rotation first as the minimal semantic check.
