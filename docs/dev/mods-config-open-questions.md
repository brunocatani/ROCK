# Ticket: Mods Config page, open questions and suggestions

**Status:** open. **Branch:** `mods-config-support`.

## Where the work stands

- **Committed:** the page (`spec.json`, `ROCKModsConfigSpecTests`), the firing-hand transforms and the `[Bindings]` tables, with their rows on the page.
- **ROCK's settings leave PALM's Config.** The author of RobCo PALM and RPS UI Framework agreed to drop ROCK's pages from RPS UI's settings menu, so Mods Config FO4 VR becomes the only in-game editor of ROCK's settings.
  - Done: `mods-config-palm-removal.md` (the Configuration interface and what only PALM's pages used are gone; ROCK's text points at Mods Config).
  - Next: `mods-config-setup-wizard-launch.md`, then the RPS UI side (`mods-config-rps-ui-removal.md`, only when the user says so), and `mods-config-developer-page.md`.
- **Nothing is released** until every `mods-config-*` ticket is done.
- **Still open:** the in-game Developer report check (turn on Developer tools, press ROCK's row in the front page's Developer section, then **Read the live copy**; fix what the report or `ModsConfigFO4VR.log` names).

The other tickets: `mods-config-palm-removal.md`, `mods-config-setup-wizard-launch.md`, `mods-config-rps-ui-removal.md`, `mods-config-developer-page.md`, `mods-config-input-bindings.md`, `mods-config-transforms.md`, `mods-config-native-api.md`, `mods-config-prompt-improvements.md`, `mods-config-left-firing-trigger.md`.

## Decisions taken

These are recorded so they aren't asked again.

- **File:** `ROCK.toml` only, at root `myGames`, path `Mods_Config/ROCK/ROCK.toml`.
- **Identity:** `modName` `ROCK`, `f4sePluginRequirements` `["ROCK"]` (not FRIK's `F4VRBody`), `category` `VR`, `applies` `save`.
- **Front page:** About, then Help (Nexus and GitHub buttons, green how-to rows), then a folded Developer section with Log Level.
- **Pages:** Interaction, Weapons (with a Melee section), Recoil, Scopes and Grenades, Controls.
- **Rows:** labels from the catalog; choices in the object form with per-option help; dependent rows disabled with `enabledWhen`, using explicit ids.
- **Bindings:** `cf.inputBinding` rows (`mods-config-input-bindings.md`). The button layout uses dropdowns of the 14 canonical values, as PALM does.
- **Left out:**
  - `SetupWizard.ShownEdition`;
  - `Developer.EnableDeveloperSettings`, which is retired (`mods-config-palm-removal.md`);
  - all of `ROCK_Developer.toml`, until `mods-config-developer-page.md`.
- **PALM's Config:** ROCK's pages leave it; the decisions are in `mods-config-palm-removal.md`.

## Open questions

1. **GitHub URL.** The Help button opens `https://github.com/ArthurHub/FO4VR-ROCK/blob/main/README.md`, this repo's `origin`. Is that ROCK's public home?
2. **Log Level 3–5.** The stepper offers Info, Debug, Trace and Off, as decided. A `ROCK.toml` holding Warn, Error or Critical (3–5), which ROCK accepts, shows invalid and blocks Save until changed. Keep that, or add the three back?
3. **Bipod Hands-Free Distance range**, chosen as 0.5–10 m. ROCK only requires a value above 0.
4. **Slider steps** were chosen, not found: 0.05 s, 5 %, 0.5° for the left aim, 0.25 or 0.1 game units, 10 game units/s for the swing speeds.
5. **Banner.** ROCK ships no textures, so the front page has none. Add a `Textures/ROCK/banner.dds` under `data/mod`?
6. **`applies` exceptions.** Immersive Scopes takes the scope hook until restart, and Scopes and Grenades want their button released. They are `save` with a note in the help. Should Immersive Scopes be `restart`?

## Suggestions

1. **ROCK writes canonical spellings at startup.** ROCK already rewrites `ROCK.toml`'s help lines when it starts. If it also rewrote button layout values to their canonical spelling (`"Grip"` to `"grip"`, `"stick"` to `"thumbstick"`), the dropdown's invalid-alias case goes away.
2. **One source for labels, options and ranges.** ROCK's catalog help and `spec.json` each hold a copy (RPS UI's `RockSettingControlPolicy.cpp` held a third, which goes with `mods-config-rps-ui-removal.md`). The test checks keys, types and defaults, not ranges, choice values or labels.
   - Either extend the test to parse the catalog's `N = label` and `Range: a to b` text and compare it with the spec's options and `min`/`max`,
   - or generate the spec's choice and range fields from the catalog.
3. **A developer page later:** now `mods-config-developer-page.md`.

## What to check in the headset

These are spec choices the docs allow but no game run has confirmed yet:

- **Nested tables.** Storage paths of three names (`["FiringHand", "Left", "AimYawDegrees"]`, `["ControllerInput", "Right", "A"]`). After a save, `ROCK.toml` should still have one `[FiringHand.Left]` and one `[ControllerInput.Right]` table, with no inline or duplicate table.
- **Reload.** ROCK picks up a Save: `ROCK.log` shows the reload, and the change shows in play.
- **Startup rewrite.** A value saved from the menu survives ROCK's rewrite of its help lines on the next game start.
- **Disabled rows.** Settle Time and Release Mode follow Auto Equip, both ways. The Recoil Strength section follows Immersive Recoil. Scope Hold Time and Grenade Fuse follow their switches.
- **Display.** The `°/s` unit on Scoped Filtering Release Speed; steppers with 4 and 5 options; the two Button Layout sections nested in a folded section.
- **Binding text inputs.** Typing on the on-screen keyboard; a deliberate typo falls back to the default with a warning in `ROCK.log`.

## Mods Config feature requests

Collected from the tickets, for the F4VR-Mods-Config repo:

1. A per-file "a default removes the key, and an empty file is deleted" mode, for files like `ROCK_Developer.toml` (`mods-config-developer-page.md`).
2. An offline validator for `spec.json`, printing the Developer report without the game (`mods-config-prompt-improvements.md`).

The transforms and the input bindings need nothing from Mods Config: ROCK stores them in the framework's tables (`mods-config-transforms.md`, `mods-config-input-bindings.md`).

## ROCK text that should mention Mods Config

Done with `mods-config-palm-removal.md`: `kPlayerHeader`, `kDeveloperHeader`, README, AGENTS.md and the CHANGELOG's Unreleased section.

## Environment notes

- `cmake --preset custom-fast` fails on this machine's existing `build-fast` cache ("generator platform: x64 does not match the platform used previously"). A plain `cmake --build build-fast` reconfigures and builds.
- There is no `.pre-commit-config.yaml`, and no clang-format on PATH, so `tests/ModsConfigSpecTests.cpp` is hand-formatted.
