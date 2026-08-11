# Buildable example plugins

These examples are intentionally small FO4VR F4SE plugins. Each target uses the shipped public header only, negotiates the exact capability set it needs, runs frame-sensitive work inside an owner callback, and unregisters its owner on session teardown.

They are teaching projects, not production mods. Copy one target, rename its plugin definition, then add configuration and user-facing behavior appropriate to your mod.

## Included examples

| Target | What it demonstrates |
| --- | --- |
| `ROCKSDKHandStateMonitor` | Owner callbacks, hand interaction snapshots, event cursors, and transition-only logging. |
| `ROCKSDKWeaponInspector` | Equipped-weapon generations, composition, part-pose readback, and scope state. |
| `ROCKSDKSurfaceClimber` | Two independent wildcard `FixedAnchor` targets for left/right surface grabs, rolling leases, lifecycle guards, and state polling. |
| `ROCKSDKContactVisualizer` | Semantic finger/hand contacts, player-collider discovery, and bounded debug-overlay publication. |
| `ROCKSDKCapabilityReporter` | Base/extended limits, feature words, table extent, and public structure-size discovery. |
| `ROCKSDKInputChordLease` | Fresh raw-button sampling, a two-button offhand chord, narrow suppression leases, and effective-state observability. |
| `ROCKSDKWeaponCatalogDumper` | Weapon classification, the complete evidence catalog, record identity, semantic roles, and emitters. |
| `ROCKSDKMuzzleRayVisualizer` | Equipped muzzle state, bounded world raycasts, hit normals, and leased overlay lines. |
| `ROCKSDKAnimationObserver` | Animation phase callbacks plus native-authority, authored-grip, and presented-pose observation. |
| `ROCKSDKWeaponPartDriver` | Non-exclusive part targeting, attach-only grip state, weapon-root-local rolling drives, result readback, and cleanup. |
| `ROCKSDKTouchMechanism` | Exact-body limited-prismatic touch targets, target generations, state transitions, and scope cleanup. |
| `ROCKSDKColliderFocus` | Complete weapon-body selection and a short exact-collider visualization override. |
| `ROCKSDKExternalContactSensor` | Scoped external-body registration, contact cursors, bounded drains, and deterministic body teardown. |
| `ROCKSDKForceGrabCommand` | Value-identity force-grab admission, lifecycle polling, timeout cancellation, and terminal results. |
| `ROCKSDKOffhandLease` | Acquire/renew/release semantics for a short offhand reservation lease. |
| `ROCKSDKEquippedHandPolicy` | Rolling equipped-weapon handling authority followed by a generation-bound exact-hand request. |
| `ROCKSDKVisualHandOffset` | Presented-hand readback and a short presentation-only visual-authority publication. |

`MinimalProviderConsumer.cpp` remains a single-file integration fragment for projects that already own their F4SE bootstrap. The 17 targets above are complete DLL examples.

Examples that can move physics, reserve input/authority, or replace an overlay
ship with an explicit `false` activation constant or unresolved value identity.
They compile the complete ownership and cleanup path without silently changing
gameplay when somebody builds the catalog unchanged. Read-only reporters,
short-lived diagnostics, the input chord, and the surface example run directly.

## Standalone build

Configure with the same local CommonLibF4VR and vcpkg used by your plugin:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
  -DCOMMON_LIB_F4VR_PATH="F:/path/to/CommonLibF4VR"
cmake --build build --config Release -- /m:1 /p:CL_MPCount=2
```

The loader check runs during configuration and fails if the shared bootstrap loses the FO4VR identity/executable gate or compares the F4SE loader runtime with a VR executable-version constant.

## Runtime behavior

All examples require:

- Fallout4VR.exe `1.2.72.0`;
- a working F4SEVR installation;
- `ROCK.dll` loaded in the same process;
- the capability and table extent named by the example.

Logs are written to the normal Fallout 4 VR F4SE log directory under each target name. Build the aggregate `ROCKSDKExamplePlugins` target to compile the full catalog, or build one named target while developing a specific pattern.

The surface-climbing example is deliberately always active while physics writes are allowed, and the input-chord example suppresses only while its demonstrated two-button chord is held. Add an explicit user setting and narrower product policy before deriving a production mod from either one.
