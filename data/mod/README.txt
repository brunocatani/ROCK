ROCK v0.5 - Release Install Notes
=================================

ROCK currently requires the matching hFRIK release named FRIK Experimental.
Download and install FRIK Experimental before using ROCK v0.5.

Download FRIK Experimental here:
https://github.com/brunocatani/hFRIK/releases/tag/frik-experimental

This hFRIK dependency is temporary. It is required only until the hFRIK
modifications are merged upstream into:

github.com/rollingrock/Fallout-4-VR-Body

Install ROCK.dll and ROCK.pdb into:

Data\F4SE\Plugins

ROCK stores its runtime configuration under:

%USERPROFILE%\Documents\My Games\Fallout4VR\Mods_Config\ROCK

ROCK.ini contains consumer settings. When absent, ROCK creates it with all
consumer defaults compiled into the plugin. Existing files are loaded unchanged.

ROCK_Developer.ini is optional. Developer defaults remain in code and are shown
in the wheel menu even without this file. Changing a developer setting creates
or updates only its override. Existing supplied developer entries are preserved
on load. Restoring a default removes that key; an empty developer file is removed.
Either file supports hot reload.
The wheel menu uses ROCK's configuration interface to display and save settings.

The source references data/config/ROCK_example.ini and
data/config/ROCK_Developer_example.ini list every supported option and default
in their respective group. They are human references, never runtime templates.

Features
--------

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.
