# ROCK Code Review Fixes - 2026-05-09

## Why This Fix Was Done This Way

The review found an ABI break and a stale-world lifetime leak risk. The API fix keeps ROCK's public header aligned with current ownership while preserving the exported function-table size consumed by older sibling plugins. The body collider fix keeps stale-world shutdown from touching lost Havok bodies, but still releases shape references owned directly by ROCK.

## Changes

- Restored a private `ROCKApiPrefixWithLegacyTail` export table in `src/api/ROCKApi.cpp`.
- Kept the removed legacy tail slots inert and opaque so ROCK does not regain feature ownership.
- Added source tests for API compatibility and body collider lifetime invariants.
- Changed `BodyBoneColliderSet::reset()` to release owned shape refs while abandoning stale body handles.
- Removed the `docs/` ignore entry so project notes can be tracked again.

## Verification

- `git diff --check`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\RockApiCompatibilitySourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\BodyColliderLifetimeSourceTests.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\ReloadDetachmentSourceTests.ps1`
- `cmake --build build --config Release -- /m:1`
- `ctest --test-dir build -C Release --output-on-failure`
