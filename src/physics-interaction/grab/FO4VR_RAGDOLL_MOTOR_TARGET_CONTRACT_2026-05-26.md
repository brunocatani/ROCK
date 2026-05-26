# FO4VR Ragdoll Motor Target Contract

Date: 2026-05-26
Source: Ghidra, `Fallout4VR.exe`
Confidence: High

## Addresses

- `0x141a5f3b0`
  - `NpJacobianBuilder::buildJacobianFromRagdollMotorAtom_hkpConstraintQueryIn_`
  - PDB offset: `0x19DBA40`
- `0x1417d05f0`
  - Helper called by the ragdoll motor builder to expand `target_bRca`

## Finding

`target_bRca` is a body-B-local target basis, not a target already expressed in
the localized constraint-B frame.

The ragdoll motor builder reads atom `target_bRca` at atom offset `+0x10`, then
calls `0x1417d05f0` with the query/body-B rotation pointer and `target_bRca`.
That helper transforms each target row through the base body-B rotation before
the angular error rows are built.

## Storage Contract

`target_bRca` is a 3x4 float matrix. The solver row view is:

```text
row0 = [0], [1], [2]
row1 = [4], [5], [6]
row2 = [8], [9], [10]
padding = [3], [7], [11]
```

The meaningful rows must encode the desired target basis in body-B local space.

Identity rows mean "target is unrotated body-B axes." Identity is only correct
when the desired angular target is actually the base body-B basis.

If a desired frame has a non-identity body-B-local rotation, `target_bRca` must
carry that rotation in row storage. A separate local transform field may use a
different byte view, but the ragdoll motor target itself needs this row view.

## Practical Rule

For a desired target frame `T` and body-B world frame `B`:

```text
target_bRca rows = rotation(inverse(B) * T)
```

Do not feed identity just because a localized constraint frame reconstructs the
same world frame. The ragdoll motor expands `target_bRca` from base body B before
solving, so the row target must describe the body-B-local target rotation.
