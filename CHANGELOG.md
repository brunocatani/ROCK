# Changelog

## v1.0.0

### Fixed

- Held objects stay steady on slow frames: per-frame motion limits scale with
  frame time, so a low frame rate no longer drops or lags what you hold.
- Pulled objects are caught even when they cross the hand between frames, and
  objects stuck in transit are placed on the hand.
- Dynamic hands and held weapons stop against fixed props. Dropped weapons keep
  their equipped collision until they come to rest. Ejected casings and
  magazines are kept clear of the hand and weapon bodies.
- After the firing grip lets go, the hand no longer snaps up to 12 units behind
  the controller while it is still inside the weapon.
- Weapons stay in your hands inside menus and when leaving them, the support
  hand stays on two-handed weapons, and both weapons and melee weapons stay
  visible through menus.
- The ammo counter stays at the firing grip when the grip detaches.
- A gun with no pistol or rifle keyword is sized from its equip slot.
- Native player melee no longer launches movable objects: the game's object hit
  pushed any non-actor body very hard.
- A finger that cannot clear the mesh keeps its open pose instead of bending
  back. Count-based loose ammo is held by its visible body.
- A closed wrist Pip-Boy no longer opens or hums when you look at it. On
  lighthouse controllers the Y tap opens the Pip-Boy and the wand-pad Favorites
  menu no longer opens by accident.
- The game closes normally on exit; ROCK no longer raises R6025 during shutdown.
- ROCK's hooks chain behind another plugin's patch or detour of the same game
  function instead of refusing to install.
- Weapon support grabs on detailed rifles no longer stall the frame: the finger
  solve walks an index built at equip instead of scanning every triangle.
- Fewer per-frame scans for weapon bones and controller fallbacks; native
  memory reads no longer use `VirtualQuery`.

### Added

- **Two weapons at once.** Equip a weapon in each hand, any combination,
  including a melee weapon as the second weapon. The second weapon has its own
  trigger, reload, scope, dry-fire click and generated collision. The Pip-Boy
  marks each equipped weapon with its hand (R / L). An equip from the inventory,
  the Pip-Boy or a holster mod goes into the hand that chose it and keeps the
  other hand's weapon. Which hand holds each weapon is saved with your game.
  Every equip is ready to fire at once.
- **Weapon Equip Mode** (`iWeaponEquipMode`): 1 trigger equip, no drop;
  2 trigger equip, toggle drop (default); 3 trigger equip, auto drop; 4 auto
  equip once the palm settles on the firing grip (`fAutoEquipSettleSeconds`).
  An auto drop leaves with the hand's motion, thrown like a loose object.
- **Ammo counters.** One counter per held weapon at that weapon's hand,
  replacing the game's counter. `iAmmoCounterStyle`: 1 Minimal, 2 Immersive
  (game HUD style), 3 Vanilla (back of the hand). A reload mod can show a
  chambered-round marker beside the number.
- **Loot to Hand** (`bLootToHand`, on by default): an item taken from the
  quick-loot view lands in the hand pointing at the container. The quick-loot
  panel works with either hand and stands at its container.
- **Pinch grab:** the object stays where thumb and finger found it and only the
  fingers close onto it.
- **Grab direction** (`iCloseGrabDirection`, `iFarGrabDirection`): which way
  your hand points to aim close and far grabs, from out of the palm to along
  the fingers.
- **Button layout** (`[Controls]`): reassign each hand's grip, trigger, A/B,
  X/Y and stick click. ROCK, its add-ons and the game all follow it. SteamVR
  binding templates for Index, Vive wands and Steam Frame controllers ship in
  `steamvr_bindings`.
- **Vanilla Favorites menu** (`bVanillaFavoritesMenu`, off by default): lets the
  stick click open the game's Favorites menu again.
- **Recoil** (`[Recoil]`): recoil is applied to the firing hand, per weapon type
  and one- or two-handed hold. Native kickback no longer fights ROCK's hands.
- Far weapon grabs take the firing grip; close grabs take the nearest authored
  grip.
- Scoped filtering: heavier, speed-adaptive smoothing while you look through a
  scope (`fScopedFilterRotationSeconds`, `fScopedFilterTranslationSeconds`,
  `fScopedFilterReleaseDegreesPerSecond`). Gunstock mode (`bGunstockMode`) and
  filtering presets (`iWeaponFilteringPreset`) are player settings.
- Bipod: a latched weapon stays equipped hands-free within
  `fBipodHandsFreeRadiusMeters`.
- Optional: use the weapon offsets you saved in FRIK instead of ROCK's grip
  (`bFrikUserWeaponOffsets`, off by default).
- One-handed melee weapons: the trigger toggles a reverse grip, saved with your
  game. `fVanillaMeleeSwingSpeed` and `fVanillaGunBashSpeed` set the hand speeds
  that start a vanilla swing and a gun bash; wrist rotation alone never swings.
- Saved grab offsets report which hands were saved.
- The game's weapon equip, unequip and inventory animation sounds are silenced.
- **Provider API 2.0**: 13 interfaces whose headers are in `include/ROCK/`, with
  one page per interface in `docs/api/`. New for holster, equipment and reload
  mods: unequip and inventory draw, warm inventory weapons, the firing aim, the
  second weapon's parts and reload state, a fire block that leaves Accept
  native, the chambered-round marker, placing the equipped weapon through held
  placement, and bodies that pass static bodies.

### Feature ownership and compatibility

- Requires FRIK 0.79.1 or newer with FRIK API 2.3. RPS UI Framework is
  recommended (grab indicators and ammo counters draw through it) and RobCo PALM
  is optional (settings pages). ROCK, RPS UI Framework, RobCo PALM, PAPER and
  the ROCK add-ons are released together; install them together.
- **NPC interactions left ROCK.** NPC collision, blade penetration, NPC pushing,
  corpse interaction, grenade NPC effects and NPC visualization belong to
  **SCISSORS**, with its own INI and PALM pages. Rocky bare-fist mode is
  removed; free-hand strikes come with SCISSORS' physical melee. The original
  SCISSORS mod, a different mod the new SCISSORS replaces under the same name,
  does not work with ROCK 1.0.
- **Mods built for ROCK 0.92 or earlier** use provider API 1, which ROCK 1.0
  does not serve. They stop the game: at the main menu an error box names each
  one, and the game closes when the box is closed. The RPS SDK is no longer
  used; mods build against the headers in `include/ROCK/`.
- **Settings were reorganized.** `ROCK.ini` is grouped into sections
  (`[FiringHand]`, `[Grabbing]`, `[Grenades]`, `[Highlight]`, `[Recoil]`,
  `[NativeGameplay]`, `[Controls]` and others) with help text above each key.
  Your values move with their keys into the new sections. Keys ROCK no longer
  reads from that file go to an `[Unused]` comment block at the end, where they
  never apply. `sDefaultHand` replaces `bLeftHandedMode`. Developer-only keys
  (grab input timing, firing-grip haptics, native scope fallback offsets) moved
  to `ROCK_Developer.ini`; a value you had set for one of them, or for
  `bLeftHandedMode`, lands in `[Unused]` and has to be set again.
- **Removed settings:** `bAmbidextrousFiringGripEnabled` (two weapons are always
  allowed), `iWeaponDropMode` (use `iWeaponEquipMode`),
  `bKeepPreviousWeaponInHandOnEquip`, `bFiringGripDetachPosePreservationEnabled`,
  `bNativeScopeForceFiringGripFallback`, `bEnableVanillaMelee` (vanilla melee is
  always on), `bNativeCharacterControllerObjectContactFilterEnabled` (always
  on), `bRockyModeEnabled`, `fRockyModeHoldSeconds`, and `npcDynamicCollisions`
  and `bBladePenetrationEnabled` (now in `SCISSORS.ini`).

### Validation

- The full test suite passed during release preparation, including the public
  header, ABI layout and discovery negotiation tests.
- Test bench sessions loaded ROCK with RPS UI Framework, RobCo PALM, PAPER, ROCK
  Wall Climbing and ROCK Pimp my House all bound to provider API 2.0 and no
  version refusal.

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
