# Ticket: ROCK's settings leave PALM's Config

**Status:** open, next. **Branch:** `mods-config-support`.

## Goal

ROCK's settings are edited in Mods Config FO4 VR only. The author of RobCo PALM and RPS UI Framework agreed to drop ROCK's pages from RPS UI's settings menu (PALM's Config); that removal is its own ticket, `mods-config-rps-ui-removal.md`. This ticket removes ROCK's side: everything ROCK keeps only so those pages can read and write its settings.

## Decisions

- **The Configuration interface (13) is retired**, as Diagnostics (12) is:
  - `include/ROCK/Configuration.h` is deleted; `Abi.h` keeps `Configuration=13` with a "retired, never reuse" note;
  - Discovery refuses it and logs the calling DLL once;
  - `ConfigurationProvider.cpp` and `ConfigurationProviderTests` go, with the interface's entries in `ROCKPublicHeaderTests` and the ABI tables (`ModularAbiLayoutChecks.h`, `ModularTestTables.h`);
  - `docs/api/Configuration.md` goes; `Overview.md`, `Migration.md` and `skills/rock-1.0-migration/SKILL.md` say it is retired; CHANGELOG.
  - Its only known consumer is RPS UI's settings menu. SCISSORS and PAPER aren't checked out here; the user confirms neither reads it.
- **The setup wizard keeps working.** It reads and saves its options through `RockConfig::visitSettings` / `persistSetting`, which stay, typed with ROCK-internal types (RPS Settings' group and `Setting`) instead of the interface's. `SettingMetadata`'s group uses the same type. The wizard writes only plain options, never bindings, so the conversions below never reached it.
- **The PALM-only conversions go:**
  - `apiSetting`, `storeValueFromApi`, `positionText`, `bindingLine` and the `ApiSetting` record (`src/config/RockSettings.cpp` / `.h`), including "a Float3 write replaces only a transform's `pos`";
  - `parseBindingLine` and `formatBindingLine` (`FunctionBindingText`). The input log (`InputRemapRuntime.cpp`, "Binding {}: {} (default {})") prints each binding as its table, the form in `ROCK.toml`, so ROCK keeps one binding text form;
  - their tests in `ConfigurationStoreTests.cpp`;
  - AGENTS.md's invariant that table forms reach the Configuration API as types RPS UI knows.
- **The RPS Framework fork stays as it is.** All six commits on `settings-transform-type` serve the forms Mods Config writes (Transform, Table, `parseTable`) and the upgrade of 1.0.x files (renamed keys, conversions). None was for PALM.
- **`Developer.EnableDeveloperSettings` is retired.** It only showed RPS UI's ROCK Dev page. It has no legacy INI row; ROCK's startup rewrite moves the key into `ROCK.toml`'s block of unused options. Developer options are edited by hand until `mods-config-developer-page.md`.
- **Text that sends players to PALM for settings points at Mods Config FO4 VR:**
  - `kPlayerHeader` ("Change these in game in the RobCo PALM wheel menu") and `kDeveloperHeader` ("PALM saves only developer options…", which becomes how to add a key by hand), then regenerate `data/config/*_example.toml`;
  - README: the Configuration feature line, and the Recommended line that names PALM as the way to the settings menu;
  - AGENTS.md: "What ROCK does not own → Settings screens", the `data/mod/` map row, and "check `spec.json`" in the "A setting" row of Where to start.
  - PALM mentions about its wheel and the stick click stay.
- **The wizard's own "PALM's Config" text stays** until it can name a real button: `mods-config-setup-wizard-launch.md`.
- **Nothing is released** until every `mods-config-*` ticket is done.
