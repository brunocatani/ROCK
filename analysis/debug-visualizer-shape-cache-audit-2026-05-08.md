# Debug Visualizer Shape Cache Audit - 2026-05-08

The generated hand/body colliders are not changed by this pass. The bug was in the debug visualizer mesh cache: rendered GPU meshes were cached by raw Havok `hknpShape*` address only. Generated ROCK colliders are destroyed and recreated when the source skeleton/profile changes, including power armor transitions. Havok can reuse the same shape address for a different collider size, so the visualizer could keep drawing an old mesh even though the actual collider was correct.

## Finding

- `DebugBodyOverlay.cpp` used `ShapeKey{ body.shapeAddress }`.
- The cache is only cleared when the world pointer or debug shape decode settings change.
- Power armor resizing changes generated shape geometry without necessarily changing the world pointer.
- If Havok reuses a shape allocation address, the overlay sees a cache hit and draws stale GPU buffers.

## Fix

- Added a geometry fingerprint to `ShapeKey`.
- Convex/polytope shapes hash type, convex radius, support vertex count, and support vertices.
- Sphere/capsule/scaled-shape paths hash their relevant dimensions and nested scaled-shape geometry.
- Cache lookup now uses `makeShapeKey(body.shapeAddress)` instead of the pointer alone.

## Verification

- Added `tests/DebugOverlayShapeCacheSourceTests.ps1`.
- `powershell -NoProfile -ExecutionPolicy Bypass -File tests\DebugOverlayShapeCacheSourceTests.ps1` passed.
- `build\Release\ROCKDebugOverlayPolicyTests.exe` passed.
- `cmake --build build --config Release` passed and deployed `ROCK.dll`/`ROCK.pdb` to `D:\FO4\mods\ROCK\F4SE\Plugins`.
- `ctest --test-dir build -C Release --output-on-failure` passed: 48/48.
