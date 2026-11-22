# Ticket: a ROCK developer page in Mods Config

**Status:** done. Part of the Mods Config release. Still to check in the headset: see [In the headset](#in-the-headset).

## Goal

A Mods Config FO4 VR page for `ROCK_Developer.toml`, which RPS UI's ROCK Dev page edited until it was removed (`mods-config-rps-ui-removal.md`). Without it, developer options were edited by hand.

## Before

- `ROCK_Developer.toml` holds 287 options in 19 tables, one table per category, all developer overrides of ROCK's compiled defaults.
- `spec.json` covered only `ROCK.toml`, and `ROCKModsConfigSpecTests` skipped the developer group.
- `Developer.EnableDeveloperSettings`, which showed the ROCK Dev page, is retired (`mods-config-palm-removal.md`).

## Decisions

### How the file is written

- Mods Config's Save writes only the keys that changed, and a missing key reads as its default, so untouched rows never reach the file. Mods Config needs no change.
- What is left: Reset writes the default as a key, which would pin today's default against a later change of ROCK's compiled one. RPS Settings closes it: a reload rewrites the developer file only when it holds a key equal to its default, dropping it and deleting the file once no keys remain, and the startup rewrite drops default-valued developer keys too. A reload never reshuffles a file being edited by hand otherwise. This is a change in the RPS Framework fork (2b8d676). Because a load no longer keeps a file of defaults, the reference examples come from the store's `organizedDefaults` (015a9ff).

### Who sees it

- `[Developer] ShowDeveloperPages` in `ROCK.toml`, off by default, a switch in the front page's Developer section next to Log Level. ROCK only declares it; the page's `visibleWhen` follows its pending value, so it inherits the spec's `save`.
- Not `EnableDeveloperSettings`: RPS UI still reads that key to show its own ROCK Dev page until `mods-config-rps-ui-removal.md`.
- No Mods Config condition reads its own Developer tools switch, which is why the switch is ROCK's.

### The page

- One page, "Developer" (`"id": "developer"`), at the end of the page list, visible while `ShowDeveloperPages` is on. Its content is a short text on how the file works; its 19 sub-pages are the categories and cover every developer option.
- Generated from the catalog: `ROCKModsConfigSpecTests --write-developer-page <spec.json>` replaces only the `developer` page and leaves the hand-written rest of `spec.json` as it is. The test fails when the page is stale.
- Rows: the label is the key split into words, the help is the catalog's. Bool is a `switcher`, int and float a `slider`, string a `textinput`, a transform a `cf.transform`.
- Slider ranges: the clamp the loader declares as it reads an option (`SettingReader::clampRange`, from the `readClamped*` helpers), or `SliderRange` in the catalog (110 options), which may also narrow a declared clamp too wide to slide over. Catalog ranges follow the limits the loader enforces in other ways; where it has none they are guesses from the default, the unit and the help. The step is a hundredth of the range, or a tenth of the default when that is finer, rounded down to 1, 2 or 5 times a power of ten.
- `applies` is `save` for every row. A sweep of where all 287 options are read found none read only at startup. It went by function names, so an option copied once elsewhere could be missed; add a restart list to the generator if one appears.

### The 10 three-number arrays

Mods Config has no array control, so they become transforms, renamed without their unit suffix; `kRenamedKeys` rows carry the old values (`Float3ToTransformPosition`).

- Offsets: `{ pos = [x, y, z] }`, sliders by 0.1: -100..100 (the scope and shoulder stash offsets), -120..120 (mouth), -20..20 (grab proxies).
- `GrabPinchDetectionDirection`: `{ pos = [...] }`, sliders -1..1; ROCK normalizes it.
- The two scope rotations: `{ rot = [pitch, roll, yaw] }`, as `rot` turns about X, Y, Z (heading, roll, attitude) and ROCK's pitch, yaw and roll about X, Z, Y. Sliders -180..180 by 1. Their old values are not carried (RPS's carry can't reorder; both default to 0); the old line stays in the file's block of unused options. Their six INI pitch, yaw and roll rows left the frozen INI table, as an INI row fills only a transform's pos.

### The test

`ROCKModsConfigSpecTests` requires every developer option to have exactly one row on the developer page, with its type and default, and no developer option in `kLeftOut`. A numeric slider must not go past what the loader accepts: both its ends must run differently from one step inside them.

## Work

1. [x] RPS fork: the reload tidy, and `organizedDefaults` for the examples.
2. [x] ROCK: the 10 arrays become transforms, with their renames.
3. [x] ROCK: `ShowDeveloperPages`, the generated page, the test, `kDeveloperHeader`, the docs.

## In the headset

- Show Developer Pages shows and hides the Developer page at once, before Save.
- A change on a developer page creates `ROCK_Developer.toml` holding only that key, and ROCK picks it up.
- Reset of that row, then Save: ROCK's reload drops the key and deletes the file; the row still reads the default.
- The sliders' ranges and steps are usable; the transform rows' sliders move the offsets and the scope rotations about the axes their help names.
