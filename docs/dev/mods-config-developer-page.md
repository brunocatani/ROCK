# Ticket: a ROCK developer page in Mods Config

**Status:** open, not started. Not needed for the release.

## Goal

A Mods Config FO4 VR page for `ROCK_Developer.toml`, which RPS UI's ROCK Dev page edited until it was removed (`mods-config-rps-ui-removal.md`). Until this lands, developer options are edited by hand.

## Today

- `ROCK_Developer.toml` holds about 290 options across some 20 categories, all developer overrides of ROCK's compiled defaults. RPS Settings writes only options that differ from their defaults, and deletes the file when none remain.
- `spec.json` covers only `ROCK.toml`. `ROCKModsConfigSpecTests` skips the developer group.
- `Developer.EnableDeveloperSettings`, which showed the ROCK Dev page, is retired (`mods-config-palm-removal.md`).

## First question: how the file is written

Mods Config writes every value its page shows. The developer file's contract is the opposite: a default is no key, and an empty file is deleted. Choose one:

1. **A Mods Config feature:** a per-file mode where a default removes the key and an empty file is deleted. A feature request for the F4VR-Mods-Config repo, listed in `mods-config-open-questions.md` (Mods Config feature requests, 1).
2. **ROCK writes the full file.** RPS Settings writes every developer option, and the "only overrides" contract goes. A diff from the defaults then has to be read off the file.
3. **ROCK tidies after a save.** Mods Config writes the full file, and ROCK's reload (or `subscribeToSaves`, `mods-config-native-api.md`) removes keys at their default. The page and the file disagree until the next read.

Lean: 1.

## Then

- [ ] Which options belong on the page. Most are tuning that only matters with ROCK's debug overlays and logs; a page per category, or only the ones a developer changes in play.
- [ ] Shown only to developers: Mods Config's Developer tools switch, or a page that is folded by default.
- [ ] Extend `ROCKModsConfigSpecTests` to the developer group.
- [ ] `kDeveloperHeader` (`src/config/RockSettings.cpp`) names the page, then regenerate `data/config/*_example.toml`.
