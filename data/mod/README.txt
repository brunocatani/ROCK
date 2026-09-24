ROCK v0.9 - Release Install Notes
================================

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
The retired monolithic headers are not compatible with ROCK 0.9.
Current developer documentation: https://devartificial.pro/docs/rps-sdk/rock/reference/api-index
Player instructions and compatibility notes:
https://www.nexusmods.com/fallout4/mods/108881

ROCK stores its runtime configuration beneath your Windows Documents folder:

My Games\Fallout4VR\Mods_Config\ROCK

ROCK.ini contains consumer settings. When absent, ROCK creates it with all
consumer defaults compiled into the plugin. Existing files are loaded unchanged.

ROCK_Developer.ini is optional. Developer defaults remain in code and are shown
in the wheel menu even without this file. Changing a developer setting creates
or updates only its override. Existing supplied developer entries are preserved
on load. Restoring a default removes that key; an empty developer file is removed.
Either file supports hot reload.
The wheel menu uses ROCK's configuration interface to display and save settings.
Runtime and example INIs are not included in this archive.

Upgrading from older versions: bToggleGrab and bAutoDrop are replaced by
iWeaponGrabMode and iWeaponDropMode. The default is iWeaponGrabMode=1
(toggle both grips) and iWeaponDropMode=2 (Toggle Drop).
Toggle Drop preserves the former retained-in-hand Auto Drop behavior.
Keeping the previous weapon in the other hand when equipping another is now
enabled by default through bKeepPreviousWeaponInHandOnEquip=true.
Back up and delete an old ROCK.ini before launching to regenerate it with
the current defaults. Keep SavedGrabOffsets and AuthoredWeaponGripCache.
NPC dynamic collisions are enabled by default through npcDynamicCollisions
in ROCK.ini. Update dependent addons to versions using the modular ROCK API.

Experimental knife and sword penetration is disabled by default. Enable
bBladePenetrationEnabled in ROCK.ini to use it; npcDynamicCollisions is also
required. Disabling penetration releases an embedded blade.

The source references data/config/ROCK_example.ini and
data/config/ROCK_Developer_example.ini list every supported option and default
in their respective group. They are human references, never runtime templates.

Features
--------

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.
