# Ticket: `[Bindings]` as Mods Config input bindings

**Status:** open, not started. **Depends on:** a Mods Config FO4 VR schema addition (see 4).

## Goal

The 12 `[Bindings]` rows of ROCK's Mods Config page (`data/mod/ModsConfigFO4/Config/ROCK/spec.json`, Controls page) become `cf.inputBinding` rows, the menu's binding editor, instead of text inputs.

## Today

- `ROCK.toml` keeps each binding as a TOML string in ROCK's own grammar: `Reload = "firing tap a"`, `"<hand> <gesture> <control> [+<hand> <control>]"` or `none` (`parseBinding`, `src/physics-interaction/input/FunctionBindingPolicy.h`).
- RPS UI Framework's settings menu (PALM's Config) shows them as plain text fields. The Mods Config page does the same: `textinput`, `maxLength` 64.
- A string ROCK can't parse falls back to the function's home binding, with a warning in `ROCK.log`.

## Why it doesn't fit `cf.inputBinding` now

| | ROCK | CommonFramework (`cf.inputBinding`) |
| --- | --- | --- |
| Storage in TOML | a string | an inline table, `{ hand = "offhand", type = "tap", button = "grip" }`; the line form is read only from INI files (`SettingSpec.cpp`, `TableForm`) |
| Hands | `firing`, `support`, `primary`, `offhand`, `left`, `right`, `each` | `primary`, `offhand`, `left`, `right`, `any` |
| While held | `press` | `hold` |
| Once, on the way down | none | `press` |
| After a hold time | `hold` (global `[Bindings] HoldSeconds`) | `longpress` (per-binding `duration`, 0.6 s) |
| Tap, double | `tap`, `double` | `tap`, `double` |
| Controls | `trigger`, `grip`, `a`/`x`, `b`/`y`, `thumbstick`, `grip_click`, `axis3`, `axis4`, `pad_up`/`down`/`left`/`right`/`center` | `trigger`, `grip`, `a`, `menu` (B/Y), `thumbstick`, `system` |
| Modifier | `+offhand grip` | `+offhand:grip` |

## Decisions

1. **Hands: no framework change.** ROCK resolves `primary`/`offhand` per binding.
   - For the four weapon functions (Fire, Reload, Scope, SupportTrigger), `primary` is the hand on the weapon's firing grip and `offhand` the support hand, resolved live from ROCK's weapon routing (`InputRemapRuntime.cpp`, `context.firingIsLeft`). The help line says "primary: the hand holding the weapon's trigger".
   - For the other eight (Use, VATS, Grenade, Pip-Boy, Pause, StickActions, PALM, Grab), `primary` is the right hand and `offhand` the left, as today.
   - A fixed hand on a weapon function makes no sense and breaks play: `Fire = "right press trigger"` with a left-hand gun lands on the support hand's trigger, the home of SupportTrigger, so the two collide. Offer only `primary` and `offhand` for those four (`valueOptions.hands`).
   - `each` maps to the framework's `any` for Grab.
   - `firing`, `support` and `each` stay readable as aliases so existing files keep working.
2. **Gestures: ROCK reads the framework's words.** `hold` means while held (ROCK's `press` today), `longpress` means after a hold time (ROCK's `hold`), `tap` and `double` are unchanged. A `longpress` `duration` overrides `HoldSeconds` for that binding. The framework's `press` (on the way down) isn't offered: `valueOptions.types` limits the rows to `hold`, `tap`, `longpress` and `double`.
   - ROCK's old `press`/`hold` meanings flip. Migration must rewrite old strings, or the parser must tell the old grammar from the new one. Decide which when implementing.
3. **Controls: logical buttons only.** Bindings name the five logical buttons (`trigger`, `grip`, `a`, `menu`, `thumbstick`). Physical extras (`grip_click`, `axis3`, `axis4`, `pad_*`) stay in the button layout (`[ControllerInput.Left]`/`[ControllerInput.Right]`), which maps them onto a logical button. What's lost is binding a function to a sixth physical control no logical button uses.
4. **Storage: Mods Config reads the line form from a TOML string.** A new opt-in field on `cf.inputBinding` (name to be decided in Mods Config) reads and writes the framework's line form, `<hand> <type> <button> [duration] [+hand:button]`, in a TOML string value. ROCK parses that grammar alongside its old one.
   - Rejected: RPS Framework gaining a table value type, with ROCK migrating `[Bindings]` to inline tables. That needs changes in RPS Framework, ROCK and RPS UI / PALM's editor.

## Work

- [ ] Mods Config FO4 VR: opt-in line form for `cf.inputBinding` in a TOML string (schema, loader, editor, docs, showcase).
- [ ] ROCK: parse the framework's words (`hold`, `longpress`, `any`, `menu`, `+hand:button`, `duration`) and the per-function meaning of `primary`/`offhand`; decide the migration of old strings; write the framework form.
- [ ] ROCK catalog help for `[Bindings]` in the new grammar; regenerate `data/config/*_example.toml`.
- [ ] RPS UI / PALM: its text field keeps working; check that it accepts the new grammar.
- [ ] Spec: replace the 12 `textinput` rows with `cf.inputBinding`, limiting `hands`, `types` and `buttons` per row, and drop the grammar text row.
- [ ] `ROCKModsConfigSpecTests`: a `cf.inputBinding` row's `default` is a JSON table, so compare it to ROCK's compiled default string through the parser.
