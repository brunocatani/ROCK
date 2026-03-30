# Body Contact Review Fixes - 2026-05-08

## Rationale

The review found that generated body colliders were physically present but not yet useful as stable gameplay evidence. Shoulder stash, future holsters, and backpack interactions need named body zones, per-zone tuning, and a safe contact snapshot surface. The fix keeps this FO4VR-native: generated body colliders remain ROCK-owned hknp bodies on layer 47, contacts are still recorded from ROCK's existing contact callback, and consumers receive value snapshots instead of reaching into physics internals.

## Changes

- Added stable body zone identity in `src/physics-interaction/body/BodyZone.h`.
- Tagged standard and power-armor body collider descriptors with stable zones and sides.
- Extended generated body metadata and body-contact records with source and target zone/side fields.
- Exposed body contacts through provider API v6 with `RockProviderBodyContactV6` and `getBodyContactSnapshotV6`.
- Added descriptor-specific body collider tuning through `sBodyBoneColliderZoneScaleOverrides`.
- Supported zone override format:
  - `Zone=radius,length,convex`
  - `Standard.Zone=radius,length,convex`
  - `PowerArmor.Zone=radius,length,convex`
  - Optional local offset form: `Zone=radius,length,convex,offsetX,offsetY,offsetZ`
  - Profile-specific entries take precedence over generic zone entries regardless of ordering.
- Replaced runtime collision registration's separate hand/weapon/reload/body calls with `applyRockGeneratedLayerPolicies`, so body layer 47 and weapon layer 44 are normalized together.
- Added direct `ROCKBodyContactRuntimeTests` coverage for invalid rejection, stable zones, snapshot order, wraparound, and reset.

## Verification

- No web, HIGGS, Ghidra, or production INI access was used.
- Configured no-copy build with `POST_BUILD_COPY_PLUGIN=OFF`.
- Built Release successfully in `build_nocopy`.
- `ctest --test-dir build_nocopy -C Release --output-on-failure` passed: 44/44.

## Notes For Shoulder Stash

- Use `RockProviderBodyZoneKind::LeftShoulder` and `RightShoulder` instead of descriptor indices.
- Power armor uses the same zone names with larger descriptor dimensions.
- If shoulder positions are off in-game, tune with `PowerArmor.LeftShoulder=...` or `Standard.LeftShoulder=...` before changing descriptor tables.
- The provider row is contact evidence only; stash state transitions and inventory transfer remain separate gameplay logic.
