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

ROCK does not ship a configuration file. It loads its only active ROCK.ini at:

%USERPROFILE%\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini

If that file is absent, ROCK creates the directory and writes a complete
ROCK.ini from its compiled C++ defaults. Existing files are loaded unchanged.

Features
--------

- Full body per-bone collision, including individual finger collisions.
- Full weapon collision for vanilla and modded weapons.
- Dynamic object grabbing, two-hand grabs, pull/catch flow, and held-object release behavior.
- Realistic gunplay support that lets you grab and use different parts of the gun for fire support.
