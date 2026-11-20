# Ticket: open ROCK's setup wizard from Mods Config

**Status:** open, not started. Blocks the release: without it the wizard opens only by itself, on a new edition. **Depends on:** nothing; Mods Config's native API already has functions, quick launch and `minimizeMenu`. Split from `mods-config-native-api.md`, whose client this starts.

## Goal

A player opens ROCK's setup wizard from Mods Config FO4 VR. PALM's Config used to open it (RPS UI's ROCK page, through the Configuration interface's `openSetupWizard`); that interface is retired (`mods-config-palm-removal.md`) and the page goes (`mods-config-rps-ui-removal.md`).

## Work

- [ ] Copy Mods Config's client header `src/api/ModsConfigFO4Api.h` into ROCK (`src/api/` or a third-party folder), and connect under `modName` `ROCK` from `kPostLoad`. ROCK runs as before when Mods Config isn't installed. Note in AGENTS.md that this copied header is how Mods Config's API is meant to be used (AGENTS.md's "never copy ROCK's headers" is about ROCK's consumers).
- [ ] `addFunction("openSetupWizard")`: `minimizeMenu`, then `setup_wizard::requestOpen()`; `restoreMenu` when the wizard closes. Decide whether Mods Config stays minimized until the wizard closes or closes outright: the wizard saves straight to `ROCK.toml`, and `restoreMenu` reads ROCK's values again.
- [ ] A quick launch entry on Mods Config's front page (`addQuickLaunch`), such as "ROCK: Setup wizard".
- [ ] A Setup wizard button on ROCK's front page in `spec.json`, calling the function.
- [ ] `requestOpen` returns false without RPS UI Framework: the wizard is an RPS UI panel. Say so to the player (the function's result, or the button disabled).
- [ ] Rewrite the wizard's text that sends the player to PALM's Config, which no longer opens it:
  - `src/setup-wizard/SetupWizardPanel.cpp`: "Open the wizard again from PALM's Config once SteamVR runs the game." (the SteamVR NotApplicable status) and "To change anything later, open this wizard again from PALM's Config, on the ROCK page." (the review page);
  - README, "The setup wizard": it says to set `ShownEdition` to 0 to see the wizard again; name the button;
  - the opening paths in `src/setup-wizard/SetupWizard.h`'s comment.
- [ ] Check the preview's captures after the text change (`ROCKSetupWizardPreview.exe --capture <folder>`).

## To check in the headset

- The button and the quick launch entry open the wizard in front of the player, with Mods Config out of the way.
- Saving the wizard, then returning to Mods Config, shows the wizard's values.
