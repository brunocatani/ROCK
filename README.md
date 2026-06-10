# ROCK - Realistic Overengineered Character Kinetics

Fallout 4 VR physics-based hand interaction plugin for the ROCK/PAPER/SCISSORS interaction stack.

My aim with ROCK is to bring as much immersion and realism to Fallout 4 VR as possible: the world should react to your body, hands, weapons, and held objects instead of feeling like a flat VR overlay.

## Features

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.
- Physics-based hand/object interaction built for Fallout 4 VR and the current FRIK skeleton provider.
- Configurable production `ROCK.ini` for grab, collision, weapon support, debug, and haptic tuning.

<p align="center">
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s"><strong>Watch the ROCK demo video</strong></a>
  <br><br>
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s">
    <img src="https://img.youtube.com/vi/tPaIzmZ9P0o/maxresdefault.jpg" alt="ROCK video preview" width="800">
  </a>
  <br>
  <sub>Click the preview to watch on YouTube. Starts at 1:53.</sub>
</p>

## Release Install

ROCK v0.5 currently requires the matching hFRIK release named **FRIK Experimental**. Download and install that hFRIK release before using ROCK v0.5:

TEMPORARILY REMOVED, I`M UPDATING THE MOD TO USE PROD VERSION OF FRIK, A FEW DAYS UNTIL I CAN DO SO.
```text
NOT WORKING AT THE MOMENT - https://github.com/brunocatani/hFRIK/releases/tag/frik-experimental
```

This hFRIK dependency is temporary. It is required only until the hFRIK modifications are merged upstream into `github.com/rollingrock/Fallout-4-VR-Body`.

Install the ROCK release package into the Fallout 4 VR mod data path so `ROCK.dll` lands in:

```text
Data\F4SE\Plugins\ROCK.dll
```

The ROCK release also ships the production `ROCK.ini`. Place that file at:

```text
%USERPROFILE%\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini
```

Create the `ROCK_Config` folder if it does not already exist. The release archive includes the file under `ROCK_Config\ROCK.ini` so the intended destination is clear.

## Requirements

- Fallout 4 VR
- F4SE VR
- FRIK Experimental release, installed separately: `https://github.com/brunocatani/hFRIK/releases/tag/frik-experimental`
- CMake, Visual Studio 2022, and vcpkg for local builds
- Local Fallout 4 VR/CommonLib dependencies configured in `CMakeUserPresets.json`

## Build

Fast plugin build and auto-deploy:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m
```

Release package build:

```powershell
cmake --preset custom-release
cmake --build build-release --config Release --target ROCK -- /m
```

## Test

```powershell
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m
ctest --test-dir build-tests -C Release --output-on-failure
```

## Public API

ROCK ships a public SDK for FO4VR F4SE plugins under `SDK/ROCK`.

- `SDK/ROCK/include/ROCKProviderApi.h` is the stable provider API for new integrations.
- `SDK/ROCK/include/ROCKApi.h` is the legacy/simple compatibility API and should not be used for new write/control behavior.
- Provider API v9 adds ROCK-issued owner tokens, capability grants, provider limits, feature bits, and owner-filtered external contacts.

Start with `SDK/ROCK/docs/PublicApi.md` and `SDK/ROCK/examples/MinimalProviderConsumer.cpp`. Public force grab/release commands are intentionally not exposed until they can run through a bounded ROCK-owned command queue from a safe update point.

## Credits

- **[rollingrock](https://github.com/rollingrock)** - Creator of FRIK (Fallout 4 VR Body)
- **[arthurhub](https://github.com/ArthurHub) (L.H.Adonis)** - For work on FRIK, CommonLibF4VR, and F4VR-CommonFramework
- **[HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930)** by FlyingParticle - The Skyrim VR mod whose architecture and approach inspired ROCK's original direction
- The Fallout 4 VR modding community

## License

See [LICENSE](LICENSE) for details.
