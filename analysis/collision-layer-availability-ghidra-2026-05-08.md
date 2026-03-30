# Collision Layer Availability Ghidra Check - 2026-05-08

## Why This Check

ROCK needs a collision-layer decision for generated body colliders that does not silently reuse the hand layer `43` or weapon layer `44`. The safe way to answer that is to separate vanilla configured layers from matrix-addressable but unnamed filter rows: vanilla layer names and defaults decide what Bethesda systems understand, while the `bhkCollisionFilter` matrix decides what Havok contact filtering can technically evaluate. This note records the Ghidra evidence before any layer policy change is made.

## Ghidra Evidence

- Loaded FO4VR image base is `0x140000000`, with VR-specific strings present.
- `bhkCollisionFilter::bhkCollisionFilter` at `0x141E11460` loads layer-name pointers from `0x1438AF760`.
- The layer-name load loop uses count `0x2F`, so it loads exactly 47 layer names: indices `0..46`.
- The same constructor allocates/initializes 64 layer-name string slots at filter offset `+0x3B0`, then separately loads only the 47 named layer strings. The extra string slots are capacity, not configured Bethesda layers.
- Verified tail of the layer table:
  - layer `43`: table pointer `0x1438AF8B8` -> `UNUSED_0` at `0x142E841A0`
  - layer `44`: table pointer `0x1438AF8C0` -> `UNUSED_1` at `0x142E841B0`
  - layer `45`: table pointer `0x1438AF8C8` -> `SPELLEXPLOSION` at `0x142E841C0`
  - layer `46`: table pointer `0x1438AF8D0` -> `DROPPINGPICK` at `0x142E841D0`
- The next pointer table begins at `0x1438AF8E0` for material names, so layer `47` is not a vanilla named/configured collision layer.
- String search found only `UNUSED_0` and `UNUSED_1` in the collision-layer name range. There is no `UNUSED_2` or named collision-layer string after `DROPPINGPICK`.
- `FUN_141E11950` initializes the `bhkCollisionFilter` collision matrix at filter object offset `+0x1A0` with `0x200` bytes, which is 64 rows of 64-bit masks.
- `FUN_141E11950` initializes those 64 rows to all bits set first, then applies Bethesda pair exclusions and row masks. The explicit Bethesda setup observed stays within the configured `0..46` layer range; no row `47` initialization was found.
- `FUN_141E115B0` extracts each layer as `filterInfo & 0x7F`, then indexes row `+0x1A0 + layer * 8`, and tests the other layer with `otherLayer & 0x3F`.
- Therefore:
  - layers `0..46` are vanilla configured/named layers;
  - layers `47..63` are matrix-addressable but unnamed/unconfigured by vanilla setup;
  - layers `64..127` must not be used because the row index walks past the 64-row matrix while the bit index aliases modulo 64.
- `FUN_141E13DF0` sets one full row and mirrors it, but its mirror loop also uses count `0x2F`, so vanilla row-wide setup mirrors only layers `0..46`.
- `FUN_141E13D70` sets or clears a pair symmetrically and has no range guard. It is only called from the vanilla matrix initializer in the checked binary xrefs, and all observed vanilla call arguments stay inside `0..46`.
- `FUN_141DF5B80` -> `FUN_14153AF00` writes a body's collision filter info directly to body offset `+0x44`. `hknpBody::initFromCinfo` at `0x1415616F0` copies `hknpBodyCinfo +0x14` directly into the same body field. No clamp to `0..46` was found on either path.

## BGSCollisionLayer Record Pass

- `BGSCollisionLayer::BGSCollisionLayer` at `0x14048FC40` matches the local CommonLibF4VR layout: `collisionIdx +0x38`, `debugColor +0x3C`, `flags +0x40`, `name +0x48`, `collidesWith +0x50`.
- `BGSCollisionLayer` load at `0x14048FD40` reads:
  - `BNAM` into `collisionIdx +0x38`;
  - `FNAM` into `debugColor +0x3C`;
  - `GNAM` into `flags +0x40`;
  - `MNAM` into `name +0x48`;
  - `INTV` count and `CNAM` form IDs into the `collidesWith` list.
- No clamp was found when loading `BNAM`; a record can represent an index above `46`.
- Post-load at `0x140490100` resolves `CNAM` form IDs to `BGSCollisionLayer*` entries and inserts the record into a dynamic hash map keyed by `collisionIdx`.
- Lookup helper `FUN_140490350` retrieves a `BGSCollisionLayer*` by numeric layer index. Its observed code xrefs at `0x14074B5D0`, `0x14074BBD0`, `0x14074DDA0`, and `0x14074DF60` use `filterInfo & 0x7F` to look up layer flags for navmesh-obstacle behavior.
- I did not find evidence that the `BGSCollisionLayer` registry populates the `bhkCollisionFilter` matrix for rows `47..63`. The matrix constructor uses the static 47-name table and hardcoded vanilla pair setup instead.

## Second-Pass Result For `47+`

There is more capacity after `46`, but there is not more vanilla configured layer data.

Rows `47..63` are real `bhkCollisionFilter` rows because the matrix is 64 rows wide and the filter predicate indexes them. They are not named by the static collision-layer table, and they are not configured by the vanilla row/mirror helper's 47-layer loop. `BGSCollisionLayer` records can carry higher numeric indices as form metadata, but the checked readers use that registry for flags such as navmesh-obstacle classification, not for establishing Havok collision pairs.

The important operational detail is that row `47` is not initialized to a clean isolated state. The matrix starts as all-ones for every row, then Bethesda clears selected vanilla pairs. Since vanilla does not intentionally normalize row/column `47`, using layer `47` without ROCK explicitly setting row `47` and every row's bit `47` would produce broad default collisions and inconsistent behavior for rows that vanilla later overwrote with row masks.

## Current Project Implications

- ROCK's current `CollisionLayerPolicy.h` intentionally treats vanilla configured layers as `0..46`, so layer `47` is currently rejected by `isConfiguredLayer`, `setPair`, `maskEnablesLayer`, and policy validation.
- If ROCK uses `47`, the policy needs a separate concept: vanilla configured layers `0..46` and ROCK extended matrix layers `47..63`.
- PAPER's local reload policy still has `FO4_LAYER_MAX_CONFIGURED = 47` and loops with `<= FO4_LAYER_MAX_CONFIGURED`, which includes row/bit `47`. That is not aligned with the verified vanilla configured count and should not be treated as proof that `47` is a vanilla layer.

## Result

There is no additional vanilla-configured unused layer after ROCK's current `43` and `44`. Layers `45` and `46` are native layers and should not be repurposed.

The first technically available extended row is `47`, but it must be treated as a ROCK-owned extended layer, not as a vanilla configured layer. Using it correctly requires explicit matrix ownership for row/column `47` and project code that allows the 64-row filter matrix range. Existing project policy currently treats configured layers as `0..46`, so `47` is not currently wired as a valid project layer.

## Recommendation

If generated body colliders need a dedicated layer separate from hands and weapon hulls, use `47` as `ROCK_LAYER_BODY` only with a deliberate extended-layer policy:

- keep it below `64`;
- configure all body-layer pairs explicitly through the `bhkCollisionFilter` matrix, including both row `47` and bit `47` in the other rows;
- do not rely on the vanilla layer-name table or `BGSCollisionLayer` records to configure Havok pairs for it;
- update project validation that currently clamps or rejects layers above `46`;
- log the row mask at registration like layers `43` and `44`.

If a layer must be vanilla-configured without extending ROCK's matrix ownership, then there is no free layer other than the already-claimed `43` and `44`.
