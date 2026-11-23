# Ticket: Grab highlight color and strength don't apply

**Status:** open, not started. Found while testing the Mods Config live preview (`mods-config-native-api.md`); not caused by it.

## Symptoms

- **Grab Highlight Color** (`[SelectionFeedback] HighlightColor`) never changes the outline's color: not live, not after Save, not after a game restart. `ROCK.toml` holds the chosen value (`"red"`).
- **Grab Highlight Strength** (`HighlightIntensityMode`) changes the outline only after a save is loaded.

## What the code does

`applyRockGrabHighlightOverride` (`src/physics-interaction/hand/VatsSelectionHighlight.cpp`) reads both settings on every highlight refresh and calls the native VATS override setter (`REL::ID(403512)`) with `color × intensity`. The first call saves the game's override state and restores it once ROCK's last target ends. While a target is lit, ROCK stops refreshing when the override state differs from what it last wrote ("changed outside ROCK").

## Leads (unconfirmed)

- Strength and color go through the same call, yet only the strength shows. The outline may take only its brightness from the VATS override and its hue from elsewhere, such as the activation (rollover) highlight's color (`kVatsHighlightUseObjectRolloverState`).
- Strength applying only after a save loads fits the saved override state being captured once and reused, or the "changed outside ROCK" guard holding the old value while some target stays lit.

## Work

- [ ] With `LogLevel` at Debug, check for "override not refreshed" and "restore skipped" lines while changing the color.
- [ ] Find where the outline's hue comes from: read the override globals (`REL::ID(384369)` and the color base) and the rollover shader state while a target is lit.
- [ ] Make color and strength apply on the next refresh of a lit target.
