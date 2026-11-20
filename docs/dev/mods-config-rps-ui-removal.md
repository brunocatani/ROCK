# Ticket: remove ROCK's pages from RPS UI Framework's settings menu

**Status:** open, not started; don't start until the user says so. **Repo:** RPS UI Framework (`C:\Stuff\GitHub\Mine\FO4VR-RPS_UI_Framework`, the user's fork `ArthurHub/FO4VR-RPS_UI_Framework`, upstream `brunocatani/RPS_UI_Framework`), then an upstream PR. **Depends on:** `mods-config-palm-removal.md` (ROCK retires the Configuration interface).

## Goal

ROCK's settings move to Mods Config FO4 VR. The author of RobCo PALM and RPS UI Framework agreed to drop ROCK's pages from RPS UI's settings menu (PALM's Config, and the RPS UI Settings holotape). The other mods' pages stay: PAPER, SCISSORS, Emitters, CSAH, DIAMOND and RPS UI's own.

## What goes

- The ROCK and ROCK Dev pages: `RpsMod::Rock` and `RpsMod::RockDeveloper` (`src/settings_menu/RpsMod.h`), `isRockConfiguration`, `loadedRockMod` (`LoadedRpsMods.h`).
- The Configuration client: `src/settings_menu/RockConfigurationClient.h`, and the ROCK connection in `ConfiguratorRuntime.cpp` / `.h`, including the `Developer.EnableDeveloperSettings` check that shows the ROCK Dev page.
- The Setup wizard button on the ROCK page (`ConfiguratorRuntime.cpp`, the `openSetupWizard` call). The wizard moves to Mods Config (`mods-config-setup-wizard-launch.md`).
- ROCK's controls: `RockSettingControlPolicy.cpp` / `.h` (hard-coded dropdowns and ranges).
- ROCK in the file preview: `IniSettingsStore` and `initializeRpsPreview`'s `rockApi` parameter, and the preview's ROCK files (`preview/SettingsMenuPreview.cpp`).
- Tests: `RockConfigurationBridgeTests.cpp`, `RockSettingControlPolicyTests.cpp`, and the ROCK cases in the other settings menu tests.
- Build: `ROCK_INCLUDE_DIR` in `CMakeLists.txt`, if nothing else includes ROCK's headers. Check WorldDraw and the other ROCK-facing code before removing it.
- Docs: `docs/api/SettingsMenu.md`, README, AGENTS.md, CHANGELOG.

## Before starting

- **Shared code.** SCISSORS' pages read SCISSORS' own Configuration table through code that may share helpers with ROCK's (`ScissorsSettingsTests.cpp` mentions ROCK). Keep what SCISSORS uses.
- **Builds against ROCK's headers.** RPS UI compiles against ROCK's `include/` (`ROCK_INCLUDE_DIR`). Once ROCK deletes `include/ROCK/Configuration.h`, RPS UI's ROCK pages no longer compile against that checkout. Until this ticket lands, build RPS UI against ROCK 1.0.x headers.
- **The release.** A released RPS UI that still asks for ROCK's Configuration interface gets a refusal, which ROCK logs once, and must show no ROCK page. Check that the ROCK page hides when the interface is refused, so either release order is safe.
