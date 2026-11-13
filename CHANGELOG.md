# Changelog

## Unreleased

### Changed

- **Mods built for another ROCK API version no longer close the game.** ROCK refuses
  them and names each one in `ROCK.log`; the game keeps running with their ROCK
  features off. No error box appears at the main menu.
- **Developer settings:** `GrabFingerPoseMode` is removed and `WeaponGrabPoseMode`
  becomes `WeaponGrabCapsuleProbes` in `ROCK_Developer.toml`. Old values of either
  key are dropped.

### Fixed

- **Workshop mode opens again.** In a settlement's build area, holding the off-hand
  menu button (Y on Touch controllers) enters Workshop mode as in the base game. There
  a tap still opens the Pip-Boy, and Pause opens when you release a hold shorter than
  the Workshop hold.
- **Scopes only on real optics.** Iron sights, red dots, holographic and laser sights
  no longer get ROCK's scope view; they stay sights. A weapon gets the scope view for
  an optic named as a scope or marked as one, or one magnifying 3x or more; nothing
  under 2x gets it.
- **Scopes from weapon mods are recognized** when the mod sets the magnification
  directly instead of through a zoom record.
- **The scope view sits on the optic itself**, also when a weapon mod mounts it on its
  own attach point (such as `P-Fourx`) instead of the standard scope slot.
- `ROCK.log` lists, for every equipped weapon, whether it gets a scope, which zoom
  data and installed mods decided it, and where the view is anchored, so a scope
  report can be checked from the log.
- **Commanding companions works with a weapon drawn.** In command mode the Use button
  gives the order wherever you point, including bare ground for "move here", instead
  of reloading or raising the scope. This holds with moved `[Bindings]` too.

## v1.0.0

ROCK 1.0 reworks controls, grabbing, weapon handling, settings and performance, and
replaces the provider API.

### Added

- **Setup wizard.** Opens by itself the first time you play with this ROCK, 10 seconds
  after you are in the game with no menu, loading screen or scene in the way (on a new
  game, after the mirror), and again when an update has something new to show. It asks
  which controllers you use, sets the main choices by topic (grabbing, holding weapons,
  aiming and shooting, grenades and healing, bipod and VATS) and saves them after a
  review; "Use recommended settings" does it in one click. Open it again from the
  settings menu (RobCo PALM's Config or the RPS UI Settings holotape), ROCK page.
- **SteamVR bindings.** ROCK uses the Quest/Touch layout. On SteamVR, Index controllers
  and Vive wands on SteamVR's default binding get ROCK's binding through the wizard; a
  binding you chose in SteamVR is never replaced. Templates for Index, Vive wands and
  Steam Frame ship in `steamvr_bindings/`.
- **Remap every control.** `[ControllerInput.Left]` and `[ControllerInput.Right]` move
  each hand's buttons; `[Bindings]` gives every action (grab, fire, use, reload, scope,
  VATS, grenade, Pip-Boy, Pause, bipod, PALM) any button with a press, tap, hold or
  double tap, optionally with a second button held. A moved action still reaches the
  game and other mods as its usual button.
- **Two weapons at once.** A weapon in each hand, any combination including melee, each
  with its own trigger, reload, scope and collision. Equips go into the hand that chose
  them; the Pip-Boy marks each weapon's hand.
- **Auto Equip and Weapon Release Mode.** A loose weapon equips as your palm settles on
  its grip (or on the trigger, if you prefer); letting go keeps it equipped, keeps it
  loose in your hand, or drops it with your hand's motion (default).
- **Grabbing.** Grab Direction (palm down, HIGGS style, or along the fingers, Alyx
  style) and Far Grab Style (immediate, or lock on and flick). Pinch grabs for small
  objects. Built-in grab poses for 3,383 objects; objects without one borrow the pose
  of the most similar ones. Pulled objects follow your hand while you move.
- **Weapon handling.** Reverse grip on one-handed melee weapons (trigger), Gunstock
  mode, shake filtering presets with extra smoothing while scoped, optional use of your
  saved FRIK weapon offsets (with their scale), recoil per weapon type on the firing
  hand, and a bipod-latched weapon that stays equipped hands-free nearby.
- **Ammo counters.** One per held weapon: Minimal, Immersive or Vanilla.
- **Prompt placement.** The quick loot panel and activate prompts can follow the hand
  pointing at them or stand at the object.
- **Provider API 2.0.** 13 interfaces with headers in `include/ROCK/` and one page per
  interface in `docs/api/`, plus a migration guide for mods built on API 1.

### Changed

- **Settings moved to TOML.** `ROCK.toml` and `ROCK_Developer.toml` in
  `Mods_Config\ROCK` reload while the game runs and match the settings menu pages. On
  the first start, every option you changed in `ROCK.ini` and `ROCK_Developer.ini` is
  carried over and the INI files are deleted; options left at their old defaults take
  the new defaults.
- The right stick click opens RobCo PALM, or the game's Favorites menu when PALM is not
  installed; with Bipod Mode on, it works the bipod while the weapon touches a surface.
- `DefaultHand` in `[FiringHand]` replaces left-handed mode.

### Fixed

- No more phantom grabs: you hold the object your fingers touched, where they touched it.
- Held and pulled objects stay steady and are caught at low frame rates; released
  objects leave at your own speed plus the throw.
- Dynamic hands and held weapons stop against fixed props; dropped weapons keep their
  collision until they come to rest.
- Weapons and the support hand stay put through menus.
- Grabs on detailed weapons and objects no longer stall the frame, with fewer per-frame
  scans overall.
- The wrist Pip-Boy no longer opens when you look at it.
- The shoulder stash takes caps stashes and other loot containers.
- The game closes normally on exit, and ROCK's hooks work alongside other plugins'
  patches of the same game functions.

### Compatibility

- Requires FRIK 0.79.1 or newer with FRIK API 2.3, and RPS UI Framework, which draws
  the setup wizard, grab indicators and ammo counters. ROCK, RPS UI Framework, RobCo PALM, PAPER
  and the ROCK add-ons are released together; install them together.
- **NPC interactions moved to SCISSORS** (NPC collision, blade penetration, NPC pushing,
  corpses). Rocky mode is removed. The original SCISSORS mod does not work with ROCK 1.0.
- **Mods built for ROCK 0.92 or earlier** use provider API 1, which ROCK 1.0 does not
  serve: an error box at the main menu names each one and the game closes. Update or
  remove them. The RPS SDK is no longer used.

## v0.92.2

### Fixed

- Prevented repeated dynamic-hand resets while walking with one hand pressed
  against the other hand or an equipped weapon. Self-contact limits now account
  for shared player movement; world contact retains its stationary safety limit.
- Added sustained surface-contact observations for held loose objects, so a
  bottle resting against a surface can qualify for placement without an impact.
- Validated placement against a fresh, bounded body scan rather than historical
  grab-cache completeness. Valid static parts, such as the Timberwolf's fixed
  component, remain static while movable parts are anchored. Invalid collision
  layers and incomplete body sets remain rejected with diagnostic details.

### Added

- Grab API 1.1 exposes held-object/contact observations, one-frame placement
  requests, cancellation and completion results. Requests expire on lost renewal
  and are cancelled when their owner or required runtime state disappears.
- Input API 1.1 exposes placement-click reservation through the complete press
  and release, allowing input consumers to avoid reusing that click.
- Native placement coordinates release of grab constraints, keyframing and the
  game's position, rotation and Havok save state. Ordinary grab and pull-to-grab
  retain their existing unlock behavior; no decoration co-save registry is added.

### Feature ownership and compatibility

- Decoration activation and gesture handling belong to the optional **ROCK Pimp
  my House** add-on. ROCK has no built-in decoration mode or decoration INI key.
- Released Grab 1.0 and Input 1.0 contracts, table prefixes and Core requirements
  remain unchanged. Consumers explicitly negotiate the new minor versions.
- Builds require [RPS SDK df02e89 or newer](https://github.com/brunocatani/RPS_SDK/commit/df02e89),
  containing `ROCK/GrabV1_1.h` and `ROCK/InputV1_1.h`.

### Validation

- Release builds passed for ROCK, ROCK Pimp my House, RobCo PALM and the SDK
  examples. Fourteen targeted tests passed across those projects during
  implementation, including ABI/negotiation, placement policy and input handling.
- The moving self-contact fix and native anchoring were tested in game before
  extraction. The extracted add-on's first load and save/reload/cell-return
  persistence still require a correlated runtime test.
