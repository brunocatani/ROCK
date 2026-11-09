# Changelog

## v1.0.0

### Fixed

- No more phantom grabs. A close grab happens only when one of your fingers
  touches the object's visible mesh; inside an object's collision but off its
  mesh, nothing is grabbed, and keeping the grip held grabs the moment a finger
  reaches the mesh. A grab refused that way lets that hand pass through the
  object's collision, so you can reach in and grab the exact part you want;
  the object collides with the hand again once you let go of the grip and pull
  the hand out. Grabs never hold the collision instead of the mesh, and a
  far pull flies the visible mesh to your palm rather than the collision's
  centre.
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
- Fingers never bend back past the open pose. A finger whose open pose sits
  inside the object keeps the open pose, or closes inward onto a handle or hole
  the hand reached into; saved grab poses with bent-back fingers show the open
  pose. Count-based loose ammo is held by its visible body.
- Loose rounds and ammo boxes: the hand no longer grabs or bumps the invisible
  box a pushed or dropped round left behind. It stays with the visible round,
  so both hands grab the round and it reloads where it lies.
- The second hand joins an object the other hand holds from where it touches
  it, so long objects no longer refuse the second grip.
- Saved grab offsets of objects whose parts move against each other (ragdoll
  toys, hides, tongs) are measured against the grabbed part and apply only to
  a grab of that part; older root-based offsets of such objects are not
  applied. Save them again while holding the part.
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
- **Weapon Equip Mode** (`WeaponHandling.WeaponEquipMode`): 1 trigger equip, no drop;
  2 trigger equip, toggle drop (default); 3 trigger equip, auto drop; 4 auto
  equip once the palm settles on the firing grip (`WeaponHandling.AutoEquipSettleSeconds`).
  An auto drop leaves with the hand's motion, thrown like a loose object.
- **Ammo counters.** One counter per held weapon at that weapon's hand,
  replacing the game's counter. `WeaponHandling.AmmoCounterStyle`: 1 Minimal, 2 Immersive
  (game HUD style), 3 Vanilla (back of the hand). A reload mod can show a
  chambered-round marker beside the number.
- **Loot to Hand** (`Grabbing.LootToHand`, on by default): an item taken from the
  quick-loot view lands in the hand pointing at the container.
- **Quick Loot Panel Placement** (`Grabbing.QuickLootPanelPlacement`): 1 on the
  right hand (game default), 2 on the hand pointing at the container (the
  closer hand when both point), 3 at the container it lists (default). Modes 2
  and 3 list containers picked by either hand.
- **Activate Prompt Placement** (`Grabbing.ActivatePromptPlacement`): 1 on both
  hands (game default), 2 on the closer hand only when both point at the same
  object, 3 at the object itself, shown once (default). Activation is unchanged.
- **Pinch grab:** the object stays where thumb and finger found it and only the
  fingers close onto it. Any object that sits between thumb and index when you
  close your hand is pinched, whatever its size: plants, handles and rims no
  longer get pulled into the palm. Thumb and index open as far as the object is
  thick and stop on it instead of closing into a fist beside it, and a loose
  round or shell can be pinched.
- **Grab direction** (`Grabbing.CloseGrabDirection`, `Grabbing.FarGrabDirection`): which way
  your hand points to aim close and far grabs, from out of the palm to along
  the fingers.
- **Button layout** (`[ControllerInput.Left]`, `[ControllerInput.Right]`): reassign each hand's grip, trigger, A/B,
  X/Y and stick click. ROCK, its add-ons and the game all follow it. SteamVR
  binding templates for Index, Vive wands and Steam Frame controllers ship in
  `steamvr_bindings`.
- **Vanilla Favorites menu** (`ControllerInput.VanillaFavoritesMenu`, off by default): lets the
  stick click open the game's Favorites menu again.
- **Recoil** (`[WeaponRecoil]`): recoil is applied to the firing hand, per weapon type
  and one- or two-handed hold. Native kickback no longer fights ROCK's hands.
- Far weapon grabs take the firing grip; close grabs take the nearest authored
  grip.
- Scoped filtering: heavier, speed-adaptive smoothing while you look through a
  scope (`WeaponHandling.ScopedFilterRotationSeconds`, `WeaponHandling.ScopedFilterTranslationSeconds`,
  `WeaponHandling.ScopedFilterReleaseDegreesPerSecond`). Gunstock mode (`WeaponHandling.GunstockMode`) and
  filtering presets (`WeaponHandling.WeaponFilteringPreset`) are player settings.
- Bipod: a latched weapon stays equipped hands-free within
  `WeaponHandling.BipodHandsFreeRadiusMeters`.
- Optional: use the weapon offsets you saved in FRIK instead of ROCK's grip
  (`WeaponHandling.FrikUserWeaponOffsets`, off by default).
- One-handed melee weapons: the trigger toggles a reverse grip, saved with your
  game. `NativeGameplay.VanillaMeleeSwingSpeed` and `NativeGameplay.VanillaGunBashSpeed` set the hand speeds
  that start a vanilla swing and a gun bash; wrist rotation alone never swings.
- Saved grab offsets report which hands were saved.
- Built-in grab poses for about 3,000 objects: a pulled or force-grabbed object
  without a saved offset is held in its authored pose instead of the dynamic
  seat. Your own saved offsets still take priority, hand by hand.
- A pulled object with a saved or built-in pose turns into that pose while it flies
  and arrives already in it, instead of turning at the last moment in your hand.
- Grenades and Molotovs are always held in their grenade or Molotov pose, on every grab,
  close or pulled. A second hand grabbing a grenade you already hold grabs it freely.
- Saved grab offsets record the grip you see: a save no longer includes the
  physics drive's lag or the two-hand shared solve, so a later pull reproduces
  it exactly.
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
- **Settings moved to TOML.** ROCK reads `ROCK.toml` and `ROCK_Developer.toml`
  in `Mods_Config\ROCK` and no longer reads `ROCK.ini` or `ROCK_Developer.ini`.
  Settings start from their defaults; set them again in PALM or in the file and
  delete the old INIs. Options are grouped into tables matching the PALM pages
  (`[Grabbing]`, `[FiringHand]`, `[ControllerInput.Left]` and others), keys lose
  their type letter (`iWeaponEquipMode` is now `WeaponEquipMode` in
  `[WeaponHandling]`), values are typed, and each X/Y/Z offset is one list such
  as `AimOffsetGameUnits = [0.0, 0.0, 0.0]`. A syntax error is logged with its
  line and the file keeps its last values until it is fixed. `DefaultHand` in
  `[FiringHand]` replaces `bLeftHandedMode`.
- **Removed settings:** `bAmbidextrousFiringGripEnabled` (two weapons are always
  allowed), `iWeaponDropMode` (use `iWeaponEquipMode`),
  `bKeepPreviousWeaponInHandOnEquip`, `bFiringGripDetachPosePreservationEnabled`,
  `bNativeScopeForceFiringGripFallback`, `bEnableVanillaMelee` (vanilla melee is
  always on), `bNativeCharacterControllerObjectContactFilterEnabled` (always
  on), `bRockyModeEnabled`, `fRockyModeHoldSeconds`, `MeshContact.GrabMeshContactOnly`
  (grabs always hold the mesh), and `npcDynamicCollisions` and
  `bBladePenetrationEnabled` (now in `SCISSORS.ini`).

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
