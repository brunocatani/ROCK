# Ticket: improve the Mods Config "write a config spec" agent prompt

**Status:** open. **Applies to:** the prompt in Mods Config FO4 VR's `docs/for-mod-authors/ai-agent.md` (the one used to write ROCK's spec), not to ROCK.

What writing ROCK's spec showed the prompt should say. Each item is something the agent had to discover, or that needed a round of questions it could have skipped.

## Add

1. **Real mods' specs as references, beside the Showcase.** FRIK (`data/mod/ModsConfigFO4/Config/FRIK/spec.json`) and Immersive Flashlight VR (`.../ImmersiveFlashlightVR/spec.json`) show the house style that the Showcase doesn't:
   - the front page as About, then Help (a wiki button and green `<font color='#10FA78'>Topic:</font>` how-to rows), then a folded Developer section with Log level as a stepper;
   - choices in the object form, with a `help` per option;
   - explicit short `id`s for conditions;
   - units mostly in the help, with `display.unit` for `°` and `ms`;
   - a `help` on every page.

   Point to their GitHub URLs, and say what is copied from them and what isn't. `{clientState:modVersion}` and `CallNativeFunction` buttons need a native client.
2. **Look at the mod's existing in-game settings UI.** ROCK already had pages in RPS UI Framework's settings menu (`RockSettingControlPolicy.cpp`), which hold its labels, its dropdown values and its ranges. A mod's own menu, or MCM, is the best map of how players see each setting.
3. **Check who else writes the file, and its rules.** ROCK's `ROCK_Developer.toml` holds only non-default values and is deleted when empty. A menu that writes defaults breaks that, so it was left out. The prompt should ask: is the menu the only writer, and does the mod's store have rules a plain writer breaks?
4. **Controls for values with no closed list.** A `dropdown` makes any file value outside its options invalid, which blocks Save for the whole mod. Use a dropdown only for a closed set the mod writes canonically. Free-form grammars (ROCK's binding strings) go in `textinput`. Check whether the mod's parser accepts aliases or any case: ROCK's button layout does, so a hand-edited alias shows invalid in a dropdown.
5. **Arrays have no control.** Say so up front: list them as left out, or ask. Don't write them as strings, which changes the TOML type.
6. **`applies` for a mod that isn't on the Common Framework.** A file watcher or a reload on save is `save`. Say how to find it in the code (a watcher or a reload call).
7. **Where the spec goes in the mod's repo.** Point to the mod's packaging (`data/mod/` here, copied by the package and post-build steps) and say it needs a CMake reconfigure.
8. **A drift test, when the mod has a settings catalog.** Suggest a test that checks the spec's storage keys, types and defaults against the mod's own catalog, and fails on a new option until it's placed or listed as left out (`ROCKModsConfigSpecTests` here).
9. **Interview before writing.** Scope decisions (which files, which settings, which control per kind, page layout) took several rounds of questions. Ask the prompt to list them as numbered questions with a recommended answer before writing anything, instead of "ask me before you guess" alone.

## Gaps in the tooling the prompt relies on

- **No offline validator.** The agent can't check `spec.json` without the game: the check loop is the player in the headset. A desktop validator (the preview exe or a CLI, `--validate <spec.json>`, printing the Developer report) would let the agent fix errors before the in-game step.
- **"Read the live copy"** works only while the game runs. The prompt could tell the agent to check whether it can reach `Documents\My Games\Fallout4VR\Mods_Config\ModsConfigFO4VR\Dev\<mod>\spec.json` before planning the loop.
