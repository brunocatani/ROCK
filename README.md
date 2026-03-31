# ROCK — Realistic Overengineered Character Kinetics

A physics-based hand interaction system for **Fallout 4 VR**, porting the functionality of [HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930) (Skyrim VR) to the Fallout 4 VR engine. ROCK builds on top of [FRIK](https://www.nexusmods.com/fallout4/mods/53464/) as the body and skeleton provider, adding realistic physics-driven hand interactions to the game.

## Features

- **Physics-based hand collision** — Hands interact with the game world through Havok physics, not animations
- **Object grabbing** — Pick up and manipulate objects using physics constraints (keyframed and dynamic modes)
- **Weapon collision** — Physical weapon interactions with the environment
- **Object detection** — Proximity-based detection of interactable objects near the player's hands
- **Palm transform tracking** — Accurate hand/palm positioning mapped from VR controller input
- **Hot-reload configuration** — Tweak settings on the fly via `RockConfig` file watcher

## How It Works

ROCK integrates directly with Fallout 4 VR's Havok 2014 physics engine (`hknpWorld`). It registers custom collision bodies on dedicated layers, sets up constraint-based grabbing, and manages per-frame physics updates synchronized with the game's simulation step. The system is designed as a coherent whole — hand collision, object detection, grab constraints, and weapon physics are interconnected subsystems, not isolated patches.

## Status

**Early development (v0.1.0)** — Core hand collision and object detection are functional. Grab mechanics and weapon physics are under active development.

## Build

Requires CMake, VS2022, and vcpkg:

```bash
VCPKG_ROOT="C:/vcpkg" cmake --preset custom
cmake --build build --config Release
```

Output: `build/Release/ROCK.dll`

## Based On

ROCK is built on top of the FRIK (Fallout 4 VR Body) project:

- [FRIK on Nexus Mods](https://www.nexusmods.com/fallout4/mods/53464/)
- [FRIK Wiki](https://github.com/rollingrock/Fallout-4-VR-Body/wiki)
- [FRIK Development](https://github.com/rollingrock/Fallout-4-VR-Body/wiki/Development)

## Credits

- **[rollingrock](https://github.com/rollingrock)** — Creator of FRIK (Fallout 4 VR Body)
- **[arthurhub](https://github.com/ArthurHub) (Adonis)** — For his great work on FRIK , [CommonLibF4VR](https://github.com/alandtse/CommonLibF4) and [F4VR-CommonFramework](https://github.com/ArthurHub/F4VR-CommonFramework)
- **[HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930)** by FlyingParticle — The Skyrim VR mod whose architecture and approach ROCK ports to Fallout 4 VR
- The Fallout 4 VR modding community

## License

See [LICENSE](LICENSE) for details.
