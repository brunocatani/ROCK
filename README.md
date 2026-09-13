# ROCK — Realistic Overengineered Character Kinetics

Fallout 4 VR physics and interaction plugin, and the foundation of the ROCK / PAPER / SCISSORS suite. ROCK provides hand and body collision, object handling, weapon interaction, and public APIs for other mods.

[Download and player guide](https://www.nexusmods.com/fallout4/mods/108881) · [RPS SDK](https://github.com/brunocatani/RPS_SDK) · [Developer documentation](https://devartificial.pro)

<p align="center">
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s">
    <img src="https://img.youtube.com/vi/tPaIzmZ9P0o/maxresdefault.jpg" alt="Watch the ROCK demo" width="800">
  </a>
  <br>
  <sub>Watch the ROCK demo on YouTube, starting at 1:53.</sub>
</p>

## Features

- Hand, finger, body, and weapon collision with the world and physical objects.
- Object grabbing, two-hand handling, distance pulling, and natural release.
- Animation-derived weapon grips, hand switching, grip-dependent recoil, and weapon-mounted scopes.
- Physical grenades, aid items, mouth consumption, grab-to-equip, and shoulder storage.
- Runtime configuration with hot reload, optional developer overrides, and RobCo PALM integration.
- Public interfaces for physics interaction, weapon handling, animation phases, and configuration.

## Runtime requirements

- Fallout 4 VR `1.2.72.0` and [F4SEVR](https://f4se.silverlock.org/).
- [FRIK v78.2 or newer](https://www.nexusmods.com/fallout4/mods/53464), installed separately.
- [VR Address Library for F4SEVR](https://www.nexusmods.com/fallout4/mods/64879).

The plugin installs to `Data/F4SE/Plugins/ROCK.dll`. The [Nexus page](https://www.nexusmods.com/fallout4/mods/108881) covers controls, compatibility, known issues, and player recommendations. Custom weapon animations are recommended when using [PAPER](https://github.com/brunocatani/PAPER).

## Building from source

Requires Windows x64, Visual Studio 2022 with the v143 C++ toolset and Windows SDK, CMake 4.2 or newer, PowerShell 7 (`pwsh`), and vcpkg. ROCK uses C++23.

| Dependency | Purpose | Location override |
| --- | --- | --- |
| [CommonLibF4VR](https://github.com/ArthurHub/CommonLibF4VR) | FO4VR engine and F4SE declarations; linked directly | `COMMON_LIB_F4VR_PATH` |
| [RPS SDK](https://github.com/brunocatani/RPS_SDK) | UI input headers, integration examples, ABI checks, and packaged SDK | `RPS_SDK_ROOT` |

The default layout places ROCK and RPS_SDK next to each other under `main_projects/`, with CommonLibF4VR under `libraries_and_tools/` at the workspace root. Set the overrides when using another layout. ROCK owns its runtime support in [src/rock_support](src/rock_support/) and vendors the required OpenVR ABI in [third_party/openvr](third_party/openvr/).

Copy [CMakeUserPresets.json.template](CMakeUserPresets.json.template) to `CMakeUserPresets.json`, remove the opening comment block, and set your dependency paths. Set `COPY_PLUGIN_BASE_PATH` to a dedicated ROCK mod folder. The inherited presets assume vcpkg is at `C:/vcpkg`; override the preset's `VCPKG_ROOT` environment value and `CMAKE_TOOLCHAIN_FILE` together if needed.

Run from the ROCK repository root:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK -- /m:1 /p:CL_MPCount=2
```

The fast preset deploys `ROCK.dll`, `ROCK.pdb`, and the files under `data/mod` to the configured mod folder. It also removes retired ROCK assets from that destination. Runtime INIs are not deployed.

### Tests

The existing test selector configures and runs checks affected by the current changes:

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

Use `-BaseRef <commit>` to select a committed range, `-PlanOnly` to inspect selection, or `-All` for the complete suite. Changes with uncertain coverage expand to broader checks.

The full suite can also be built and run directly:

```powershell
cmake --preset custom-tests
cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m:1 /p:CL_MPCount=2
ctest --test-dir build-tests -C Release --output-on-failure -j 4
```

<details>
<summary>Local release packaging</summary>

```powershell
cmake --preset custom-release
cmake --build build-release --config Release --target ROCK -- /m:1 /p:CL_MPCount=2
```

The release preset creates a local archive using the independent RPS SDK checkout. Packaging does not publish a GitHub release.

</details>

## Public APIs and integration

Start with the [ROCK SDK guide](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/PublicApi.md) and [minimal consumer example](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/examples/MinimalProviderConsumer.cpp).

ROCK Provider API **V1** uses owner registration, capability grants, feature discovery, provider limits, and queued commands. Consumers should check the required table extent and capabilities before using an interface and follow the SDK's lifetime and cleanup rules.

- [Discovery and capabilities](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/DiscoveryAndCapabilities.md)
- [Runtime contract](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/RuntimeContract.md)
- [Power Armor integration](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/FeatureGuide.md#power-armor-and-reference-details)
- [Configuration API](https://github.com/brunocatani/RPS_SDK/blob/main/SDK/ROCK/docs/Configuration.md)

The configuration API is a separate contract exposed through `GetROCKConfigurationApi`. FRIK's skeleton-provider API **V2** is also independent of ROCK Provider V1. SDK example tests check the public headers against ROCK's runtime ABI.

## Source layout

| Path | Responsibility |
| --- | --- |
| [src/ROCKMain.cpp](src/ROCKMain.cpp) | Plugin initialization and lifecycle |
| [src/physics-interaction](src/physics-interaction/) | Physics, grabbing, weapons, input, and interaction systems |
| [src/api](src/api/) | Provider and configuration interfaces |
| [src/config](src/config/) and [src/RockConfig.cpp](src/RockConfig.cpp) | Configuration storage, catalog, and loading |
| [src/rock_support](src/rock_support/) | Runtime, logging, menus, and controller support |
| [tests](tests/) and [tools/Invoke-RockTests.ps1](tools/Invoke-RockTests.ps1) | Policy tests, source checks, and test selection |

## Configuration

Both runtime files live under the user's Windows Documents folder:

```text
My Games/Fallout4VR/Mods_Config/ROCK/ROCK.ini
My Games/Fallout4VR/Mods_Config/ROCK/ROCK_Developer.ini
```

A missing consumer file is created from compiled defaults. Existing files are preserved; missing keys use their compiled defaults. Developer overrides are optional and are created by non-default edits. Resetting an override removes it, and an empty developer file is removed. Both files hot reload.

The [consumer example](data/config/ROCK_example.ini) and [developer example](data/config/ROCK_Developer_example.ini) are documentation only. They are never runtime inputs or installation templates.

RobCo PALM reads the compiled catalog and writes through ROCK's configuration API. Interactive panels use the separate RPS UI Framework plugin.

## Credits

- **[L.H. Adonis](https://www.patreon.com/theartofdev)** — For CommonLibF4VR, F4VR-CommonFramework, and supporting the FRIK changes needed by ROCK.
- **[Cylon Surfer](https://www.patreon.com/CylonSurferMods)** — For support, advice, testing, and compatibility checks.
- **[Asciimov](https://www.patreon.com/cw/Asciimov)** — For extensive testing, Heisenberg integration work, and serving as the main tester.
- **[RollingRock](https://www.patreon.com/rollingrock)** — For creating FRIK and supporting the project.
- **[FlyingParticle (Atom)](https://www.patreon.com/flyingparticle)** — For creating [HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930), the original inspiration for ROCK.
- The Fallout 4 VR modding community.

## License

[GNU GPL v3 only (GPL-3.0-only)](LICENSE).
