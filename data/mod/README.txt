ROCK v0.92.2 - Release Install Notes
==================================

Requirements
------------

- Fallout 4 VR 1.2.72.0 and F4SEVR.
- Upstream FRIK 0.79.1 or newer with FRIK API v2.3 (frame phases), installed separately.
- VR Address Library for F4SEVR.

FRIK: https://www.nexusmods.com/fallout4/mods/53464
VR Address Library: https://www.nexusmods.com/fallout4/mods/64879
F4SEVR: https://f4se.silverlock.org/

Install this archive through your mod manager. The plugin files belong at:

Data\F4SE\Plugins\ROCK.dll
Data\F4SE\Plugins\ROCK.pdb

The SDK folder contains the current modular headers and buildable examples.
Start with SDK/ROCK/modular_examples/ReadHands.cpp. The examples/CMakeLists.txt
supports building the full example set with CommonLibF4VR and vcpkg.
The retired monolithic headers are not compatible with this release.
Current developer documentation: https://devartificial.pro/docs/rps-sdk/rock/reference/api-index
Player instructions and compatibility notes:
https://www.nexusmods.com/fallout4/mods/108881

ROCK stores its runtime configuration beneath your Windows Documents folder:

My Games\Fallout4VR\Mods_Config\ROCK

ROCK.ini holds the player options. When it is missing, ROCK creates it with
every option at its default and the help for each option above it. The easiest
way to change options is the RobCo PALM wheel menu; editing the file also works,
and both take effect while the game runs.

Each time ROCK starts it tidies its INI files, keeping every value you set:
- options added by a new ROCK version are added at their defaults;
- the help lines (lines starting with ;) are rewritten to match this version;
- options ROCK no longer reads are moved into the [Unused] block at the end of
  the file as comments, so you can still see your old values. Delete them any
  time. Your own notes are kept when they start with #.

ROCK_Developer.ini is optional. Developer defaults remain in code and are shown
in the wheel menu even without this file. Changing a developer setting creates
or updates only its override. Restoring a default removes that key, and the file
is removed when no options remain.
Runtime and example INIs are not included in this archive.

Upgrading from older versions: bToggleGrab, bAutoDrop, iWeaponDropMode and
bAutoEquipWeapons are replaced by iWeaponGrabMode and iWeaponEquipMode.
The default is iWeaponGrabMode=1 (toggle both grips) and iWeaponEquipMode=2
(trigger equip with Toggle Drop). iWeaponEquipMode=4 is Auto Equip, which
always uses Auto Drop and has no trigger equip.
Equipping a weapon into a free hand keeps the weapon the other hand holds;
two weapons can be equipped at once.
bLeftHandedMode is replaced by sDefaultHand = right or left. Old entries for
it, and for options that moved to ROCK_Developer.ini (grab press leeway,
firing-grip haptics and reattach thickness, scope placement offsets), go to
the [Unused] block; set changed values again in the wheel menu.
Existing configured values are preserved. Keep SavedGrabOffsets when updating; automatic authored
grip JSON persistence has been removed in favor of session discovery.
NPC collisions, experimental blade penetration and NPC collider visualization
belong to SCISSORS Redux. Configure its own SCISSORS_Redux.ini and optional
SCISSORS_Redux_Developer.ini under Mods_Config/SCISSORS_Redux in Documents.
ROCK no longer exposes or forwards those settings. Its shared native physics
and visualization services remain available to independently bound consumers.

The source references data/config/ROCK_example.ini and
data/config/ROCK_Developer_example.ini list every supported option with its
default and help. They are human references, never runtime templates.

Features
--------

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.

Higgs Pull Mode (Grabbing/iFarGrabMode=2): hold grab to lock, flick toward yourself,
then keep holding to pull the object toward your hand and catch it automatically.
Release before the flick or during the pull to cancel. Immediate Pull (mode 1)
also auto-catches and does not require a flick.

World pickup/use takes priority over right-A reload/scope input. With no usable
target, tap to reload or hold to scope. ROCK-held objects retain take protection.

Decoration activation belongs to the optional ROCK Pimp my House add-on.
ROCK supplies its native placement APIs but has no decoration-mode INI toggle.
