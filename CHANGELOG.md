# Changelog

## v1.0.0

### Fixed

- No more phantom grabs. A close grab holds an object where your hand found it
  only when one of your fingers touches its visible mesh. For an object
  your other hand holds, a grab inside the collision but off the mesh waits: keeping the grip held grabs the moment a finger reaches
  the mesh, and meanwhile that hand passes through the object's collision, so
  you can reach in and grab the exact part you want; the object collides with
  the hand again once you let go of the grip and pull the hand out. Grabs never
  hold the collision instead of the mesh, and a far pull flies the visible mesh
  to your palm rather than the collision's centre.
- Grabbing an NPC's limb or a corpse lands on the first press: when the body's
  collision keeps your fingers off the visible mesh, the grab takes the nearest
  point of the mesh instead of waiting.
- Held objects stay steady on slow frames: per-frame motion limits scale with
  frame time, so a low frame rate no longer drops or lags what you hold.
- Pulled objects are caught even when they cross the hand between frames, and
  objects stuck in transit are placed on the hand.
- Objects let go of while walking or running leave at your own speed plus your
  arm's throw, instead of flying far ahead of you: the throw strength scales
  only the arm's motion, and turning no longer flings what you release.
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
- Pinch grabs of detailed objects no longer stall the frame: measuring the
  object's thickness between thumb and index skips the triangles that cannot
  lie between them. Starting a weapon support grab also measures fewer finger
  distances and merges its candidates without a repeated search.
- Fewer per-frame scans for weapon bones and controller fallbacks; native
  memory reads no longer use `VirtualQuery`.

### Added

- **Function bindings** (`[Bindings]`): each action (grab, fire, use, reload,
  scope, VATS, grenade, Pip-Boy, Pause, bipod and Favorites, PALM) takes any
  button with a press, tap, hold or double tap, optionally while another button
  is held. A moved action reaches the game and other mods as its usual button,
  so they keep working. The defaults change nothing.
- **Setup wizard.** Pick each action's button by pressing it, choose tap, hold
  or double tap, then go through ROCK's main options one page each; nothing is
  saved until the review page. It opens from PALM's Config (ROCK page); mods
  open it with `Configuration::openSetupWizard`. It also opens by itself on the
  first game start with this ROCK and whenever an update has something new to
  show, 10 seconds after the game is free of menus, message boxes and scenes
  (on a new game, after the mirror).
- **SteamVR binding offer.** On SteamVR, when Index controllers or Vive wands
  use SteamVR's default binding for Fallout 4 VR (A and X arrive as grip, or
  are missing), the setup wizard opens by itself and offers ROCK's
  binding; one choice sets it for Fallout 4 VR. A binding chosen in SteamVR is
  never replaced. `OfferSteamVRBinding` turns the automatic offer off.
- **Two weapons at once.** Equip a weapon in each hand, any combination,
  including a melee weapon as the second weapon. The second weapon has its own
  trigger, reload, scope, dry-fire click and generated collision. The Pip-Boy
  marks each equipped weapon with its hand (R / L). An equip from the inventory,
  the Pip-Boy or a holster mod goes into the hand that chose it and keeps the
  other hand's weapon. Which hand holds each weapon is saved with your game.
  Every equip is ready to fire at once.
- **Weapon Equip Mode** (`WeaponHandling.WeaponEquipMode`): 1 trigger equip, no drop;
  2 trigger equip, toggle drop; 3 trigger equip, auto drop; 4 auto equip once
  the palm settles on the firing grip (default; `WeaponHandling.AutoEquipSettleSeconds`).
  An auto drop leaves with the hand's motion, thrown like a loose object.
- **Ammo counters.** One counter per held weapon at that weapon's hand,
  replacing the game's counter. `WeaponHandling.AmmoCounterStyle`: 1 Minimal, 2 Immersive
  (game HUD style, default), 3 Vanilla (back of the hand). A reload mod can show a
  chambered-round marker beside the number.
- **Quick Loot Panel Placement** (`Grabbing.QuickLootPanelPlacement`): 1 on the
  right hand (game default), 2 on the hand pointing at the container (the
  closer hand when both point, default), 3 at the container it lists. Modes 2
  and 3 list containers picked by either hand, and the panel's button hints
  go with it. Left X on the container the panel lists takes the selected item,
  as A does, instead of opening the container (Oculus/Quest controllers).
- **Activate Prompt Placement** (`Grabbing.ActivatePromptPlacement`): 1 on both
  hands (game default), 2 on the closer hand only when both point at the same
  object (default), 3 at the object itself, shown once. Activation is unchanged.
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
- **Vanilla Favorites menu** (`ControllerInput.VanillaFavoritesMenu`, on by default): the
  stick click opens the game's Favorites menu. While RobCo PALM is installed,
  ROCK blocks the Favorites menu and the stick click opens PALM. With Bipod
  Mode on, a click while the weapon touches a surface or is latched works the
  bipod instead.
- **Recoil** (`[WeaponRecoil]`): recoil is applied to the firing hand, per weapon type
  and one- or two-handed hold. Native kickback no longer fights ROCK's hands.
- Far weapon grabs take the firing grip; close grabs take the nearest authored
  grip.
- Scoped filtering: heavier, speed-adaptive smoothing while you look through a
  scope (`WeaponHandling.ScopedFilterRotationSeconds`, `WeaponHandling.ScopedFilterTranslationSeconds`,
  `WeaponHandling.ScopedFilterReleaseDegreesPerSecond`). Gunstock mode (`WeaponHandling.GunstockMode`) and
  filtering presets (`WeaponHandling.WeaponFilteringPreset`, Steady by default) are
  player settings.
- Bipod: a latched weapon stays equipped hands-free within
  `WeaponHandling.BipodHandsFreeRadiusMeters`. The bipod listens to the stick
  of the hand on the weapon: the firing hand while both hands hold it, the hand
  still holding it otherwise, and the hand that let go last while it stands
  hands-free. The other stick keeps its own actions.
- Each stick click acts only for its own hand. An add-on placing held objects
  (such as Pimp my House) places the object in the hand whose stick you click,
  and the other hand's click keeps the Favorites menu, PALM or the game's own
  action. A click the bipod or a placement takes never also reaches the game.
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
- Objects without a pose of their own, such as items added by other mods, borrow
  one: ROCK compares the object's shape with the objects that have an authored
  pose and holds it the way the most similar ones are held, with the fingers
  closed on its own mesh. Pulls of such objects fly into that pose too.
- A close grab that touches nothing no longer does nothing: an object or a
  weapon near your hand comes the last bit to the hand and settles in its pose
  or grip, like a short pull. Touching it first still holds it exactly where
  your hand found it.
- Small objects such as a single cartridge can be pinched: an object between
  your thumb and index finger counts as touched even when neither finger's
  closing path meets it, and the two fingers bend onto it instead of closing
  beside it.
- Small objects pass from hand to hand: grabbing something too small for two
  hands, such as a cartridge, with your other hand takes it, and the first hand
  lets go. Both hands no longer hold it at once and turn it against each other.
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
  in `Mods_Config\ROCK`. On the first start, every option you changed in
  `ROCK.ini` and `ROCK_Developer.ini` is carried into the TOML files and the
  INIs are deleted; options left at their old defaults take the new defaults,
  and ROCK.log lists settings that no longer exist. Options are grouped into
  tables matching the PALM pages
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
