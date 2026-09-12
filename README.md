# ROCK - Realistic Overengineered Character Kinetics

Fallout 4 VR physics-based hand interaction plugin for the ROCK/PAPER/SCISSORS interaction stack.

My aim with ROCK is to bring as much immersion and realism to Fallout 4 VR as possible: the world should react to your body, hands, weapons, and held objects instead of feeling like a flat VR overlay.

## Features

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.
- Physics-based hand/object interaction built for Fallout 4 VR and the current FRIK skeleton provider.
- Separate consumer settings and optional developer overrides, with a compiled configuration catalog and hot reload.

<p align="center">
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s"><strong>Watch the ROCK demo video</strong></a>
  <br><br>
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s">
    <img src="https://img.youtube.com/vi/tPaIzmZ9P0o/maxresdefault.jpg" alt="ROCK video preview" width="800">
  </a>
  <br>
  <sub>Click the preview to watch on YouTube. Starts at 1:53.</sub>
</p>

## Runtime installation and configuration

ROCK now uses **upstream FRIK v78.2 or newer**. Install FRIK from the
[upstream Fallout 4 VR Body project](https://github.com/rollingrock/Fallout-4-VR-Body)
before installing ROCK. The separate hFRIK / FRIK Experimental dependency is
retired; ROCK uses upstream FRIK's API V2 integration.

The game target is Fallout4VR.exe 1.2.72.0 with F4SE VR. Install the ROCK mod
package through your mod manager so `ROCK.dll` lands under `Data/F4SE/Plugins`.

The active configuration is beneath the user's Documents known folder:

```text
My Games/Fallout4VR/Mods_Config/ROCK/ROCK.ini
My Games/Fallout4VR/Mods_Config/ROCK/ROCK_Developer.ini
```

ROCK creates the consumer file from compiled defaults only when missing.
Existing files load unchanged, with missing keys using compiled defaults.
Developer settings are optional overrides, created by non-default edits and
removed on reset. Both files support hot reload. Example INIs are human
references and must not be shipped, embedded or copied as runtime defaults.

RobCo PALM's ROCK and Developer pages read the compiled catalog and write through
`GetROCKConfigurationApi`. Interactive panels are hosted by the separate RPS UI
Framework plugin. These are separate companion mods. See the
[ROCK SDK configuration guide](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/Configuration.md)
for the configuration API and ownership contract.

## Requirements

- Fallout 4 VR
- F4SE VR
- [Upstream FRIK](https://github.com/rollingrock/Fallout-4-VR-Body), **v78.2 or newer**, installed separately
- CMake, Visual Studio 2022, and vcpkg for local builds
- ArthurHub CommonLibF4VR. This is the ROCK runtime plugin's only external C++ source-project dependency; the workspace checkout at `libraries_and_tools/CommonLibF4VR` is used by default and `COMMON_LIB_F4VR_PATH` can override it.
- The independent `RPS_SDK` repository for SDK example tests and release packaging. A sibling checkout is discovered automatically; `RPS_SDK_ROOT` can override its location.

ROCK does not require F4VR-CommonFramework. The logging, FO4VR runtime, menu, resource, and controller support it needs is ROCK-owned under `src/rock_support/`. The small OpenVR SDK ABI used for controller input and haptics is vendored under `third_party/openvr/`, so it is not downloaded or built as another source project.

## Build

Fast plugin build and auto-deploy:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m:1 /p:CL_MPCount=2
```

Release package build:

```powershell
cmake --preset custom-release
cmake --build build-release --config Release --target ROCK -- /m:1 /p:CL_MPCount=2
```

## Test

```powershell
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m:1 /p:CL_MPCount=2
ctest --test-dir build-tests -C Release --output-on-failure -j 4
```

## Public API

ROCK's public FO4VR F4SE SDK is maintained in the independent `RPS_SDK`
repository. In this workspace it is the sibling project at `../RPS_SDK`, with
the ROCK module under `SDK/ROCK`.

- `../RPS_SDK/SDK/ROCK/include/ROCKProviderApi.h` is the stable provider API for integrations.
- `../RPS_SDK/SDK/ROCK/include/ROCKApi.h` is an alias for the same v1 API table.
- API v1 includes ROCK-issued owner tokens, capability grants, provider limits, feature bits, owner-filtered external contacts, and queued interaction commands.
- The current V1 table has 99 function pointers (792 bytes on x64), including Power Armor classification, linked-frame and animated armor-hand queries, and specific-point grab commands. Require the PA table extent and capability grants before using these additions.

Start with `../RPS_SDK/SDK/ROCK/docs/PublicApi.md` and
`../RPS_SDK/SDK/ROCK/examples/MinimalProviderConsumer.cpp`. ROCK test builds
compile the independent example catalog and reject any drift between its public
headers and ROCK's runtime ABI headers. Release packaging also consumes that
verified independent SDK tree.

The [Power Armor integration guide](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/FeatureGuide.md#power-armor-and-reference-details)
and its buildable example cover validity flags, command completion, native grip
release, and cleanup. ROCK's public provider API remains V1; upstream FRIK's API
V2 is a separate skeleton-provider contract.

## Credits

- **[rollingrock](https://github.com/rollingrock)** - Creator of FRIK (Fallout 4 VR Body)
- **[arthurhub](https://github.com/ArthurHub) (L.H.Adonis)** - For work on FRIK, CommonLibF4VR, and F4VR-CommonFramework
- **[HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930)** by FlyingParticle - The Skyrim VR mod whose architecture and approach inspired ROCK's original direction
- The Fallout 4 VR modding community

## License

See [LICENSE](LICENSE) for details.
