# Dynamic Grab Authority Review Fix - 2026-05-16

## Why this pass exists

The dynamic loose-object grab now uses the root-flattened hand bone as the
authority frame. This pass hardens that model after review: the proxy consumes
the full hand transform, so translation-only validation was not enough, and
older palm-collider names/readbacks made the runtime path look mixed even after
the authority source was simplified.

## Scope

- Ordinary dynamic one-hand loose-object grab.
- Loose non-equipped weapons that use the dynamic loose-object path.
- Debug/telemetry naming around dynamic grab authority.
- The legacy `fRightGrabPivotAHandspace*` and `fLeftGrabPivotAHandspace*`
  config remains available for API/equipped two-hand/non-dynamic callers.

Out of scope:

- Equipped weapon and two-hand weapon behavior changes.
- Actor ragdoll grab.
- Reintroducing generated palm-collider authority.

## Fixes made

- Added full raw hand authority transform validation:
  - translation must be finite;
  - rotation matrix entries must be finite;
  - scale must be finite and positive.
- Dynamic capture no longer reads the live hand collider as a fallback authority
  frame. If the root-flattened hand transform is invalid, grab creation fails
  instead of silently mixing sources.
- `createProxyConstraintGrabDrive` no longer accepts the removed raw-hand
  parameter. The authority transform is the raw hand transform for dynamic grab.
- Telemetry names now distinguish:
  - generated palm collider target;
  - hand-bone grab authority;
  - proxy readback.
- Shipped and active INI comments now state that the old handspace PivotA is
  retained for non-dynamic/API/equipped two-hand callers, not dynamic loose grab.

## Important preserved behavior

- Dynamic grab still gets PivotA from the root-flattened hand authority origin.
- COM remains weight/solver data, not grip fallback.
- Generated palm/finger colliders remain collision/contact evidence only.
- Old configured PivotA values stay present for systems that still need them.

## Validation plan

- Source-boundary tests must reject generated palm-collider authority and old
  PivotA authority in dynamic loose grab.
- Runtime build must deploy through `build-fast`.
- In-game validation should confirm that debug logs say hand-bone authority when
  reporting the dynamic authority frame.
