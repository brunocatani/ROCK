# Ticket: `[Bindings]` as Mods Config input bindings

**Status:** decided, not started. **Depends on:** nothing outside ROCK and its RPS Framework fork. Mods Config is unchanged, as for the transforms (`mods-config-transforms.md`).

## Goal

The 12 `[Bindings]` rows of ROCK's Mods Config page (`data/mod/ModsConfigFO4/Config/ROCK/spec.json`, Controls page) become `cf.inputBinding` rows, the menu's binding editor, instead of text inputs.

## Today

- `ROCK.toml` keeps each binding as a TOML string in ROCK's own grammar: `Reload = "firing tap a"`, `"<hand> <gesture> <control> [+<hand> <control>]"` or `none` (`parseBinding`, `src/physics-interaction/input/FunctionBindingPolicy.h`).
- ROCK's `press` presents the home press while held. Its `hold` presents a press held from the hold time until release ([FunctionBindingPolicy.h](../../src/physics-interaction/input/FunctionBindingPolicy.h), `Phase::Pending`).
- Hold times: `[Bindings] HoldSeconds` (0.35), except Scope (`[Scopes] ManualScopeHoldSeconds`, 0.30), Pause (`[ControllerInput] PipboyPauseHoldSeconds`, 0.35) and Grenade (the game's `fVANSButtonHeldThreshold`, 0.25). `[Bindings] DoubleTapSeconds` (0.30) is the double-tap gap.
- Scope's, Pause's and Grenade's times also drive the home-button gestures, outside the binding recognizer (`InputRemapRuntime.cpp`):
  - A: short press reloads, hold scopes, on both weapon slots (`manual_scope_input_policy`);
  - left Y: tap Pip-Boy, hold Pause (`pipboy_pause_gesture_policy`), the native menu handler's release, and FRIK's Pip-Boy hold override (`configurePipboyInput`);
  - B: VATS or grenade (`vats_grenade_gesture_policy`) and the native VATS suppression hook (`hookedNativeVatsVansDecision`).
- RPS UI's settings menu (PALM's Config) and the Mods Config page show the bindings as text fields.

## Decisions

1. **Storage: CommonFramework's table.** Each `[Bindings]` key holds the framework's inline table, which `cf.inputBinding` reads as is: `Reload = { hand = "primary", type = "tap", button = "a" }`. Keys keep their names (no `k` prefix, no `kRenamedKeys` rows). Off is `{ type = "none" }`.
2. **RPS Framework fork, branch `settings-transform-type`:**
   - a generic inline-table value type: RPS keeps the table's TOML text and knows nothing of bindings; ROCK parses and validates it;
   - a generic startup value converter, next to the renamed keys: the mod registers a function per option that gets the old value and read access to the file's other keys, and returns the new value. RPS applies it before organizing the file, and to values from INI migration.
3. **Hands.** For Fire, Reload, Scope and SupportTrigger, `primary` is the hand on the weapon's firing grip and `offhand` the support hand, resolved live (`context.firingIsLeft`); those rows offer only `primary` and `offhand` (`valueOptions.hands`). For the other eight, `primary` is the right hand. `any` is ROCK's `each`, for Grab. `firing`, `support` and `each` stay readable as aliases.
4. **Gestures.** ROCK's `press` is `hold` with no duration; ROCK's `hold` is `hold` with a `duration`; `tap` and `double` are unchanged. Rows offer `hold`, `tap` and `double` (`valueOptions.types`).
5. **Controls: logical buttons only:** `trigger`, `grip`, `a`, `b` (`menu` read as `b`), `thumbstick`. Physical extras (`grip_click`, `axis3`, `axis4`, `pad_*`) stay in the button layout (`[ControllerInput.Left]`/`[ControllerInput.Right]`). A chord is the framework's `modifier = { hand = "offhand", button = "grip" }`.
6. **Durations live on the binding.** `HoldSeconds`, `DoubleTapSeconds`, `ManualScopeHoldSeconds` and `PipboyPauseHoldSeconds` are retired. Every timed `hold` and every `double` ROCK writes, defaults included, carries an explicit `duration`, so the editor shows the real time.
   - Scope's, Pause's and Grenade's durations also drive their home gestures: the A split on both weapon slots, the Y split, the native menu handler, FRIK's Pip-Boy override, the VATS/grenade split and the VATS suppression hook.
   - Grenade's default is a fixed 0.25 s, no longer following `fVANSButtonHeldThreshold`.
   - On one control, a tap waits as long as the longest `double` duration there.
7. **Duration rules.**
   - `hold`: `0` or none means at once; a positive value is clamped to 0.05–2 s.
   - `double`: clamped to 0.15–0.6 s; none means the framework's 0.4 s.
   - Out of range: clamped, with a warning; the file keeps what the player wrote.
   - A `duration` on a `tap` makes the binding invalid, as in the framework.
   - A Scope or Pause hold at 0 is allowed; its home partner (Reload, Pip-Boy) then never fires on that button.
8. **Unsupported framework words** (`press`, `release`, `longpress`, `touch`, `axis`, `system`, `suppress`, `threshold`, `left`/`right` on a weapon function) make the binding invalid: the home binding applies, with a warning in `ROCK.log`, and the file keeps what the player wrote.
9. **Migration**, through the RPS converter, once at startup:
   - an old string is read with the old grammar (old `press`/`hold` meanings) and written as the table;
   - the four retired hold times fold into the matching durations: `ManualScopeHoldSeconds` into Scope, `PipboyPauseHoldSeconds` into Pause, `HoldSeconds` into the other timed holds, `DoubleTapSeconds` into the doubles; the retired keys then become comments (RPS's trailing block);
   - an old string naming a physical extra becomes the function's home binding, with a warning naming the dropped control and pointing to the button layout;
   - a string ROCK can't read becomes the home binding, with a warning;
   - a `ROCK.ini` migration lands in the same tables.
10. **Configuration API and PALM.** Each binding is published as a `String` in a line form of framework words, `<hand> <type> <button> [duration] [+<hand>:<button>]` (`primary hold a 0.30`), and a written line is parsed into the table. The aliases `firing`, `support` and `each` are accepted; `press` is refused, so an old string typed into PALM can't be misread. A line that doesn't parse is refused with its reason, and the file keeps the last binding. RPS UI and PALM are unchanged; RPS UI's stale range row for `ManualScopeHoldSeconds` does no harm.
11. **Mods Config page.**
    - The 12 binding rows are `cf.inputBinding`, limiting `hands`, `types` and `buttons` per row; the grammar text row goes.
    - The four retired rows go: Scope Hold Time (Scopes page), Pip-Boy/Pause Hold Time, Hold Time and Double Tap Time (Controls page).
    - The Scopes page gets a text row: Scope Hold Time is now the Scope binding's duration, on the Controls page.
    - The Scope row takes Scope Hold Time's `enabledWhen` (Immersive Scopes).
    - The Scope and Pause help lines say what their duration also splits (Reload or scope on A, Pip-Boy or Pause on Y).
12. **Public API wording.** `include/ROCK/Input.h` (the scope lease's Accept classification) and `docs/api/Input.md` name the Scope binding's duration instead of `Scopes.ManualScopeHoldSeconds`. No ABI change.

Rejected:

- A Mods Config opt-in reading the framework's line form from a TOML string: the change belongs in ROCK.
- A dedicated `InputBinding` value type in RPS: the binding words are the framework's and ROCK's, not RPS's.
- ROCK editing `ROCK.toml` before RPS loads it: the RPS store owns the files.
- Keeping Scope and Pause hold times as separate settings, or using them when the binding has no duration: the editor would show "at once" on the rows players tune most.

## Work

One staged change at a time, with a go-ahead before each.

- [ ] RPS fork: generic inline-table value type. `SettingsStoreTests`: a table round-trips; a malformed table is kept as written.
- [ ] RPS fork: startup value converter, also on INI values. `SettingsStoreTests`: it runs once, and on INI values.
- [ ] ROCK: `[Bindings]` as tables:
  - parser for the framework words, aliases and duration rules; per-function `primary`/`offhand`;
  - converter for old strings and the four retired hold times;
  - home gestures and FRIK reading binding durations;
  - API line form; catalog help in the new form; regenerate `data/config/*_example.toml`;
  - `Input.h` and `docs/api/Input.md` wording; README's `[Bindings]` text.
  - Tests: `ConfigurationStoreTests` for migration (strings, retired keys, extra controls, `ROCK.ini`); the table parser as a configuration contract.
- [ ] ROCK: spec rows (decision 11); `ROCKModsConfigSpecTests` compares a `cf.inputBinding` row's JSON `default` with ROCK's compiled default through the parser.

## Check in the headset

- A short B press still opens VATS, and a hold draws a grenade, with Grenade at 0.25 s (the VATS suppression hook now uses ROCK's duration).
- FRIK's Pip-Boy hold follows the Pause duration, after a change made in the menu.
- The second weapon's A split (reload or scope) follows the Scope duration.
- An old `ROCK.toml` migrates: tables written, retired keys commented, custom hold times carried over.
- The editor: weapon rows offer only `primary`/`offhand`; a saved binding takes effect after Save.
- PALM: the text field shows the line form; a bad line is refused.
