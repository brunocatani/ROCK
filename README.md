# ROCK — Realistic Overengineered Character Kinetics

Fallout 4 VR physics-based hand interaction plugin for the ROCK/PAPER/SCISSORS interaction stack.

<p align="center">
  <a href="https://www.youtube.com/watch?v=tPaIzmZ9P0o&t=113s">
    <img src="https://img.youtube.com/vi/tPaIzmZ9P0o/maxresdefault.jpg" alt="ROCK video preview" width="800">
  </a>
</p>

## Requirements

- CMake
- Visual Studio 2022
- vcpkg
- Local Fallout 4 VR/CommonLib dependencies configured in `CMakeUserPresets.json`

## Build

```powershell
cmake --preset custom
cmake --build build --config Release
```

Output: `build/Release/ROCK.dll`

## Test

```powershell
ctest --test-dir build -C Release
```

## Credits

- **[rollingrock](https://github.com/rollingrock)** — Creator of FRIK (Fallout 4 VR Body)
- **[arthurhub](https://github.com/ArthurHub) (L.H.Adonis)** — For his great work on FRIK , [CommonLibF4VR](https://github.com/alandtse/CommonLibF4) and [F4VR-CommonFramework](https://github.com/ArthurHub/F4VR-CommonFramework)
- **[HIGGS](https://www.nexusmods.com/skyrimspecialedition/mods/43930)** by FlyingParticle — The Skyrim VR mod whose architecture and approach ROCK ports to Fallout 4 VR, wouldnt be possible without the great engineering this man did.
- The Fallout 4 VR modding community

## License

See [LICENSE](LICENSE) for details.
