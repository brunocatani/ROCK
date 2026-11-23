# Ticket: open ROCK's setup wizard from Mods Config

**Status:** done; built, every test passes, preview captures checked, checked in the headset. Split from `mods-config-native-api.md`, whose client this started.

## Goal

A player opens ROCK's setup wizard from Mods Config FO4 VR. PALM's Config used to open it (RPS UI's ROCK page, through the Configuration interface's `openSetupWizard`); that interface is retired (`mods-config-palm-removal.md`) and the page goes (`mods-config-rps-ui-removal.md`).

## Decisions

- **Mods Config steps aside and comes back.** The function minimizes the menu (its edits stay pending) and opens the wizard; once the wizard is gone, ROCK restores the menu, which reads ROCK's values again. Mods Config's menu is a world panel, not a game menu, so the wizard would otherwise open beside it and both would take the pointer.
- **A pending edit on the row the wizard writes stays the player's**, and a later Save writes it over the wizard's value: Mods Config's read-again keeps pending edits. Accepted; it needs an unsaved edit on exactly the option the wizard changes.
- **Code:** `src/mods-config/`: `ModsConfigFO4Api.h` (Mods Config's header, copied as is from its commit `5a74c6c`) and `ModsConfigClient` (ROCK's client). Ticked from `prepareRuntimeFrame` after the wizard.
- **Launch point:** the Setup Wizard button in the Help section of ROCK's page, running `openSetupWizard`. No quick launch entry on Mods Config's front page. Without RPS UI Framework the function fails with "The setup wizard needs RPS UI Framework."

## As built

- The client connects at the end of ROCK's `kGameLoaded` initialization, not `kPostLoad`: the wizard needs ROCK running, and without it the page's button is disabled ("no function"), which is right.
- `setup_wizard::isActive()` (open, or an open request waiting) tells the client when to restore. A request the wizard can't honour (no headset pose, a game menu holding the controllers) clears on that frame, so the menu comes back at once.
- The wizard's two strings, the Help section's "Setup wizard:" row, README and the `SetupWizard.h` comment name Mods Config.

## To check in the headset

Confirmed in game on 2026-11-23:

- The button opens the wizard, and Mods Config comes back after it.
- Finishing the wizard shows its values on ROCK's page.
- Pressing the button while the wizard is already up minimizes the menu and leaves the wizard as it is.
