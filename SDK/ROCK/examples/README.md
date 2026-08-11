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

`MinimalProviderConsumer.cpp` remains a single-file integration fragment for projects that already own their F4SE bootstrap. The four targets above are complete DLL examples.

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

Logs are written to the normal Fallout 4 VR F4SE log directory under each target name. The surface-climbing example is deliberately always active while physics writes are allowed; add an explicit user setting or activation condition before deriving a production mod from it.
