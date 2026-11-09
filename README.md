# ROCK - Realistic Overengineered Character Kinetics

[![License: GPL-3.0-only](https://img.shields.io/badge/license-GPL--3.0--only-blue.svg?style=flat-square)](LICENSE)

> A Fallout 4 VR F4SE plugin that gives the player physical hands: grabbing, hand and weapon collision, weapon handling, and a provider API other mods build on.

[Download on Nexus Mods](https://www.nexusmods.com/fallout4/mods/108881) · [API reference](docs/api/Overview.md) · [Developer guide](https://devartificial.pro/docs/apis/rock) · [Changelog](CHANGELOG.md)

<p align="center">
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s">
    <img src="https://img.youtube.com/vi/tPaIzmZ9P0o/maxresdefault.jpg" alt="Watch the ROCK demo" width="800">
  </a>
  <br>
  <sub>Watch the ROCK demo on YouTube, starting at 1:53.</sub>
</p>

## Table of Contents

- [Background](#background)
- [Install](#install)
- [Configuration](#configuration)
- [Building mods with ROCK](#building-mods-with-rock)
- [Building ROCK from source](#building-rock-from-source)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Background

Fallout 4 VR gives the player hands that pass through everything. ROCK makes them physical: they collide with the world, pick things up, hold weapons by their grips and hand items to other systems. It runs as an F4SE plugin (`ROCK.dll`) on top of [FRIK](https://github.com/rollingrock/Fallout-4-VR-Body)'s body.

What it provides:

- **Physical hands** - hand, finger, body and weapon collision with the world and physical objects.
- **Grabbing** - pick up objects, hold them with two hands, pull them from a distance and catch them. A held object sits in your palm with your fingers posed around it.
- **Two weapons at once** - equip a weapon in each hand, each with its own trigger, reload, scope and collision. An equip goes into the hand that chose it and keeps the other hand's weapon.
- **Weapon handling** - firing and support grips taken from each weapon's animation, hand switching, recoil applied to the firing hand, scopes, bipods, a reverse grip for one-handed melee weapons, and one ammo counter per held weapon.
- **Physical items** - grenades, aid items, eating and drinking at your mouth, grabbing to equip, and stashing a weapon at your shoulder.
- **Controls** - a button layout (`[ControllerInput.Left]` and `[ControllerInput.Right]`) that ROCK, its add-ons and the game all follow, plus SteamVR binding templates for Index, Vive wands and Steam Frame controllers.
- **Configuration** - typed TOML settings that reload while the game runs, with in-game pages in RobCo PALM.
- **Provider API** - 13 interfaces that other mods use to read hands and weapons and to request grabs, equips and input.

ROCK is the base of the RPS suite: [SCISSORS](https://github.com/brunocatani/SCISSORS) adds NPC interactions, [PAPER](https://github.com/brunocatani/PAPER) adds reload animations, and RobCo PALM, RPS UI Framework and ROCK Wall Climbing build on it.

## Install

### Requirements

- Fallout 4 VR `1.2.72.0` and [F4SEVR](https://f4se.silverlock.org/).
- [VR Address Library for F4SEVR](https://www.nexusmods.com/fallout4/mods/64879).
- [FRIK](https://www.nexusmods.com/fallout4/mods/53464) 0.79.1 or newer, with FRIK API 2.3. ROCK runs inside FRIK's frame phases and refuses an older FRIK API.
- Recommended: [RPS UI Framework](https://www.nexusmods.com/fallout4/mods/108951). ROCK's grab indicators and ammo counters draw through it; without it they are not shown.
- Optional: RobCo PALM, for ROCK's in-game settings pages.
- Every mod that builds on ROCK must be a version made for ROCK 1.0. ROCK, RPS UI Framework, RobCo PALM, PAPER and the ROCK add-ons are released together; install them together.

### Recommended install

Install the archive from [Nexus Mods](https://www.nexusmods.com/fallout4/mods/108881) with your mod manager. `ROCK.dll` goes to `Data/F4SE/Plugins`. The archive also holds SteamVR bindings (`steamvr_bindings/`) for the controllers below; they are installed into SteamVR, not the game.

### Index, Vive and Steam Frame controllers

ROCK expects the Oculus Touch (Quest) button layout. For these controllers the archive's `steamvr_bindings/` folder has a SteamVR binding that maps the buttons to that layout and makes the game see Touch controllers:

| Controller | File | Name in SteamVR |
| --- | --- | --- |
| Valve Index | `legacy_bindings_knuckles_rock.json` | ROCK - Index as Touch |
| HTC Vive wands | `legacy_bindings_vive_controller_rock.json` | ROCK - Vive wands as Touch |
| Steam Frame | `legacy_bindings_frame_controller_rock.json` | ROCK - Steam Frame as Touch |

1. Copy the file for your controller into `Documents\steamvr\input\imports`. Create the folder if it does not exist.
2. In SteamVR, open the controller bindings for Fallout 4 VR: Settings, Controllers, Manage Controller Bindings, then Fallout 4 VR.
3. Choose a custom binding and select the ROCK binding from the bindings in your documents folder.

Menu names can differ between SteamVR versions. After that, the `[ControllerInput.Left]` and `[ControllerInput.Right]` tables of `ROCK.toml` can move any button further.

### Compatibility

- **SCISSORS** owns NPC interactions: NPC collision, physical melee, blades, held NPCs and active ragdoll. ROCK 1.0 has no NPC features of its own. The original SCISSORS mod is a different, retired mod that the new SCISSORS replaces under the same name; remove it.
- **Mods built for ROCK 0.92 or earlier** use provider API 1, which ROCK 1.0 does not serve. They stop the game: at the main menu an error box names each one, and the game closes when the box is closed. Update or remove them; they work again once rebuilt against ROCK 1.0's headers.
- **Upgrading from ROCK 0.92** keeps your settings: on its first start ROCK 1.0 carries every option you changed in `ROCK.ini` and `ROCK_Developer.ini` into `ROCK.toml` and `ROCK_Developer.toml`, then deletes the old INI files. Options you left at their old defaults take ROCK 1.0's defaults. ROCK.log lists the settings that no longer exist. NPC settings moved to SCISSORS.

## Configuration

ROCK reads two files under your Windows Documents folder:

```text
My Games/Fallout4VR/Mods_Config/ROCK/ROCK.toml
My Games/Fallout4VR/Mods_Config/ROCK/ROCK_Developer.toml
```

Both are [TOML](https://toml.io) files. Options are grouped into tables such as `[Grabbing]` or `[ControllerInput.Left]`, and values are typed: `true`/`false`, whole numbers, decimal numbers, `"text"` and lists of three numbers such as `[0.0, 7.0, -7.0]` for offsets. Help for each option sits above it on lines starting with `##`.

`ROCK.toml` is created from the compiled defaults when it is missing. When ROCK starts it adds options the file lacks, refreshes the `##` help lines and moves options it no longer reads to a commented block at the end. Your own notes on lines starting with a single `#` above an option are kept. `ROCK_Developer.toml` is optional and holds only developer overrides. Both files reload while the game runs. RobCo PALM's ROCK pages edit them through ROCK's configuration API.

If a file has a syntax error, ROCK writes the file name, line and column to `ROCK.log`, keeps the values it last read from that file and leaves the file untouched until it is fixed. A value of the wrong type (text for a number, for example) is logged with its line, and that option keeps its last value.

[ROCK_example.toml](data/config/ROCK_example.toml) and [ROCK_Developer_example.toml](data/config/ROCK_Developer_example.toml) document every option. They are references only, never files to install.

## Building mods with ROCK

ROCK 1.0 publishes **provider API 2.0**: 13 interfaces your F4SE plugin discovers at runtime from `ROCK.dll`. The headers are in [include/ROCK/](include/ROCK/); take them from the release tag you ship against. They need only the C++20 standard library, and there is nothing to link.

### Set up

Point a CMake cache variable at ROCK's `include` directory:

```cmake
set(ROCK_INCLUDE_DIR "" CACHE PATH "ROCK include directory (the folder that contains ROCK/)")
target_include_directories(MyPlugin PRIVATE "${ROCK_INCLUDE_DIR}")
```

### Connect

```cpp
#include <ROCK/Client.h>
#include <ROCK/Hands.h>

rock::api::Client client;
const rock::api::hands::Api* hands{};

void connectRock() {
    auto* module = GetModuleHandleW(L"ROCK.dll");
    if (!module) return;  // ROCK absent: run without it
    auto query = reinterpret_cast<rock::api::QueryInterfaceV1>(
        GetProcAddress(module, rock::api::kQueryExportName));
    if (!query || client.connect(query, "My Mod") != rock::api::Status::Ok) return;
    client.acquire(static_cast<std::uint32_t>(rock::api::hands::Permission::Read), hands);
}

// Each frame:
rock::api::hands::HandFrame frame{};
if (hands && hands->getHandFrame(client.owner(), rock::api::Hand::Right, &frame) == rock::api::Status::Ok) {
    // frame.transform is the right hand in world space
}
```

Call `client.close()` on the game thread before your plugin's callback data goes away.

### Versions

- `Client` asks for exactly the major, minor and table size your headers declare. Do the same if you call `ROCKAPI_QueryInterfaceV1` yourself, and read optional additions from the descriptor's `minor`; never try versions one after another.
- A version mismatch stops the game. When ROCK refuses a request (`UnknownInterface`, `UnsupportedMajor`, `UnsupportedMinor`, `TableTooSmall`), it logs your DLL, shows an error box naming it at the main menu, and closes the game when the box is closed. A mod built for provider API 1 gets this.
- When `ROCK.dll` is not loaded, turn your ROCK features off and keep running.
- When a loaded ROCK refuses your mod, stop the game the same way so the player knows which mod to update. [RPS Framework](https://github.com/brunocatani/RPS_Framework)'s header-only `RPS/Runtime/RequiredUpdate.h` shows the box and closes the game.

### Interfaces

| Interface | Use it to |
| --- | --- |
| [Core](docs/api/Core.md) | Register your mod, bind interfaces, read ROCK's lifecycle, get a callback every frame. |
| [Hands](docs/api/Hands.md) | Read hand and headset poses, and what each hand holds of the equipped weapons. |
| [Collision](docs/api/Collision.md) | Read hand and body contacts, cast rays, keep ejected magazines clear, watch dropped items come to rest. |
| [Grab](docs/api/Grab.md) | Grab and release for the player, read what a hand holds, reserve a hand, guide or place a held object. |
| [Touch](docs/api/Touch.md) | Make doors, levers, sliders and climbable surfaces holdable. |
| [Weapon](docs/api/Weapon.md) | Read the equipped weapons, equip and unequip from the inventory, draw from a holster. |
| [WeaponParts](docs/api/WeaponParts.md) | Find, grab and move the parts of the equipped weapon (physical reloads). |
| [Animation](docs/api/Animation.md) | Run code at fixed points of each animation frame, take over arm, hand and weapon poses. |
| [Input](docs/api/Input.md) | Read controller buttons, suppress ROCK and game input on a hand, lease the reload button. |
| [PlayerController](docs/api/PlayerController.md) | Read the player's character controller and request native jumps. |
| [Configuration](docs/api/Configuration.md) | Read and change ROCK's settings. |
| [NativeInteraction](docs/api/NativeInteraction.md) | Extend ROCK's physics step with your own Havok bodies and weapon driving. |
| [Storage](docs/api/Storage.md) | Build holsters, backpacks and stashes. |

### Rules every consumer follows

- One owner per mod, named after the mod. Bind only the permissions you use.
- Value-initialize every record (`{}`); ROCK checks each record's `size` and `version`.
- Most calls run on the game thread; the pages mark the ones that are safe from any thread.
- Leases (suppressions, authorities, targets) last a few frames: renew them every frame you need them.
- `RequestQueued` means accepted, not done: poll the result.
- Weapons are addressed by weapon slot: 0 is the first equipped weapon, 1 the second. Which hand holds a weapon is data, never implied by the slot.

Start with the [API overview](docs/api/Overview.md), then the page of each interface you use. The [developer guide](https://devartificial.pro/docs/apis/rock) adds a complete first consumer, recipes and a [migration guide](https://devartificial.pro/docs/apis/rock/guides/migration) for mods built on provider API 1.

## Building ROCK from source

### Prerequisites

- Windows x64 with Visual Studio 2022 (v143 toolset and a Windows SDK), x64 only, with Desktop development with C++
  - `winget install -e --id Microsoft.VisualStudio.2022.Community --override "--passive --wait --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended --norestart"`
- CMake 4.2 or newer
  - `winget install -e --id Kitware.CMake`
- PowerShell 7 (`pwsh`), used by the test selector
  - `winget install -e --id Microsoft.PowerShell`
- [vcpkg](https://github.com/microsoft/vcpkg), with the `VCPKG_ROOT` environment variable pointing to it
  - `git clone https://github.com/microsoft/vcpkg.git`, run `bootstrap-vcpkg.bat`, then `setx VCPKG_ROOT "<path to vcpkg>"`
- Checkouts of these repositories:

| Repository | What ROCK uses | Setting |
| --- | --- | --- |
| [CommonLibF4VR](https://github.com/brunocatani/CommonLibF4VR) (`master`) | Linked directly: engine types, Havok, F4SE, relocation | `COMMON_LIB_F4VR_PATH` |
| [RPS UI Framework](https://github.com/brunocatani/RPS_UI_Framework) | Headers in `include/RPS_UI_Framework/`: world drawing and number overlays, bound at runtime | `RPSUI_INCLUDE_DIR` (its `include` folder) |
| [RobCo PALM](https://github.com/brunocatani/RobCo_PALM) | Header `include/RobCoPALM/Developer.h`: the Developer wheel capture switch, bound at runtime | `PALM_INCLUDE_DIR` (its `include` folder) |
| [RPS Framework](https://github.com/brunocatani/RPS_Framework) | Linked statically: the settings files (`RPS::Settings`); header `RPS/Runtime/RequiredUpdate.h`: the version-mismatch error box | `RPSFramework_DIR` (the package its `custom-fast` build stages, `build-fast/stage/lib/cmake/RPSFramework`) |

ROCK is built and tested against that CommonLibF4VR fork: ArthurHub's [CommonLibF4VR](https://github.com/ArthurHub/CommonLibF4VR) with Fallout 4 VR additions. It links nothing from RPS UI Framework or RobCo PALM, and runs without them installed; RPS Framework is linked into `ROCK.dll`. F4VR-CommonFramework is not needed.

ROCK uses C++23. vcpkg installs the other libraries from `vcpkg.json`. The OpenVR ABI ROCK needs is vendored in [third_party/openvr](third_party/openvr/).

### Build

Copy [CMakeUserPresets.json.template](CMakeUserPresets.json.template) to `CMakeUserPresets.json`, remove its comment block, and set:

- `COMMON_LIB_F4VR_PATH` to your CommonLibF4VR checkout;
- `RPSUI_INCLUDE_DIR` and `PALM_INCLUDE_DIR` to the `include` folders of those checkouts, and `RPSFramework_DIR` to RPS Framework's staged package;
- `COPY_PLUGIN_BASE_PATH` to the mod folder the build copies `ROCK.dll` and `ROCK.pdb` into (an MO2 mod folder or the game's `Data` folder).

Configuring stops with an error naming any of these that is not found.

Then, from the repository root:

```powershell
cmake --preset custom-fast
cmake --build build-fast --config Release --target ROCK
```

### Tests

Tests share `build-fast` but are built only on request. The test selector builds and runs the tests your changes affect:

```powershell
pwsh -NoProfile -File tools/Invoke-RockTests.ps1
```

`-PlanOnly` shows the selection, `-BaseRef <commit>` covers a committed range, and `-All` runs everything. To build and run every test directly:

```powershell
cmake --build build-fast --config Release --target build_tests
ctest --test-dir build-fast -C Release --output-on-failure
```

### Release package

```powershell
cmake --preset custom-release
cmake --build build-release --config Release --target ROCK
```

This writes a release's two files to `build-release/package`: a `.zip` that installs as it is, holding `F4SE/Plugins/ROCK.dll`, any game data under `data/mod` and `steamvr_bindings/`, and this readme beside it. It publishes nothing.

### Source layout

| Path | Contents |
| --- | --- |
| [include/ROCK](include/ROCK/) | Public provider API headers |
| [docs/api](docs/api/) | One page per provider interface |
| [src/ROCKMain.cpp](src/ROCKMain.cpp) | Plugin initialization and lifecycle |
| [src/physics-interaction](src/physics-interaction/) | Physics, grabbing, weapons, input and interaction |
| [src/api](src/api/) | Provider interface implementations |
| [src/config](src/config/) | Settings catalog, storage and loading |
| [src/rock_support](src/rock_support/) | Logging, runtime helpers, menus and controller input |
| [steamvr_bindings](steamvr_bindings/) | SteamVR binding templates for non-Quest controllers |
| [tests](tests/) and [tools](tools/) | Tests, the test selector and developer scripts |

## Acknowledgements

- **[RollingRock](https://www.patreon.com/rollingrock)** - For creating FRIK, implementing ROCK's migration to FRIK API 2.3, and contributing code, debugging, in-game testing, and continued support to improve frame timing, weapon handling, and scope compatibility.
- **[L.H. Adonis](https://www.patreon.com/theartofdev)** - For CommonLibF4VR, F4VR-CommonFramework, and supporting the FRIK changes needed by ROCK.
- **[Cylon Surfer](https://www.patreon.com/CylonSurferMods)** - For support, advice, testing, and compatibility checks.
- **[Asciimov](https://www.patreon.com/cw/Asciimov)** - For extensive testing, Heisenberg integration work, and serving as the main tester.
- **[FlyingParticle (Atom)](https://www.patreon.com/flyingparticle)** - For creating [HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930), the original inspiration for ROCK.
- The Fallout 4 VR modding community.

## License

[GNU GPL v3 only (GPL-3.0-only)](LICENSE).
