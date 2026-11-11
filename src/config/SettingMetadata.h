#pragma once

#include <ROCK/Configuration.h>
#include <string_view>

namespace rock::config
{
    struct SettingMetadata
    {
        std::string_view table, key, category, description;
        api::configuration::Group group;
    };

    // TOML tables, presentation order, grouping, and help for the settings files and the wheel menu.
    // Setting types and defaults are declared by the compiled loader.
    inline constexpr SettingMetadata kSettingMetadata[] = {
        { "Logging", "LogLevel", "01. Logging",
            "How much ROCK writes to ROCK.log.\n0 = Trace. 1 = Debug. 2 = Info. 3 = Warn. 4 = Error. 5 = Critical. "
            "6 = Off.\nTrace and Debug write a lot and can cost frame time; keep Info for normal play.",
            api::configuration::Group::Consumer },
        { "Grenades", "ImmersiveGrenades", "02. Grenades",
            "Immersive Grenades\nHold the grenade button (right B by default) to draw a grenade into your hand and "
            "throw it by hand.\nSelecting a grenade in PALM also puts it in your hand.\nFalse: the game's own hold "
            "and release throw.\nRelease the grenade button before changing this; a grenade already drawn keeps its "
            "mode.",
            api::configuration::Group::Consumer },
        { "Grenades", "RealisticGrenadeFuseSeconds", "02. Grenades",
            "Grenade Fuse\nSeconds from pulling the pin to detonation for grenades drawn by hand.\n0 uses the "
            "grenade's own fuse time. Range: 0 to 30.",
            api::configuration::Group::Consumer },
        { "ImmersiveAid", "Enabled", "03. Immersive Aid",
            "Immersive Aid\nStimpaks and Med-X sit in a fixed syringe grip and inject after touching your body for "
            "0.5 seconds.\nFalse: they are grabbed like any object and used by bringing them to your mouth.\nRelease "
            "and grab again to see the change on an item already in your hand.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "GunstockMode", "04. Weapon Handling",
            "Gunstock Mode\nTwo-handed firearms aim along the line from your firing hand to your support hand, like "
            "a rifle with a stock.\nHand and finger poses stay as authored; the support hand may sit slightly away "
            "from its controller.\nMelee weapons, weapon parts you operate and bipod-latched weapons are not "
            "affected.\nFalse: normal two-hand aiming.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "FrikUserWeaponOffsets", "04. Weapon Handling",
            "FRIK Weapon Offsets\nUse the weapon offset files you saved in FRIK (FRIK_Config\\Weapons_Offsets), "
            "including their weapon scale, instead of ROCK's grip.\nFRIK's built-in default offsets are never "
            "used.\nWeapons with a saved file use dynamic support grabs only and skip PAPER's reload and cycle "
            "animations.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "BipodMode", "04. Weapon Handling",
            "Bipod Mode\nClick the right stick to latch any equipped weapon part to the surface it touches.\nWhile "
            "latched, recoil is a fixed 10% whatever your hands or armor.\nFalse: the click is left to other mods.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "BipodHandsFreeRadiusMeters", "04. Weapon Handling",
            "Bipod Hands-Free Distance\nA latched weapon stays equipped with no hand on it while your headset is "
            "within this distance of the latch point, in metres.\nFarther away it is left in place as a world object "
            "(it does not fall). Must be above 0.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "DetachEitherHand", "04. Weapon Handling",
            "Detach Either Hand\nEither hand may let go of the weapon, including the hand on the firing grip; the "
            "other hand keeps holding it.\nGrips release as Weapon Grab Mode sets. Parts you only attach to release "
            "when you let go of the button.\nWhat happens when the last hand lets go is set by Weapon Equip Mode.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "WeaponEquipMode", "04. Weapon Handling",
            "Weapon Equip Mode\n1 = Trigger Equip with no drop (game default). 2 = Trigger Equip with Toggle Drop.\n"
            "3 = Trigger Equip with Auto Drop. 4 = Auto Equip (always Auto Drop).\nTrigger Equip: pull the trigger "
            "while holding a loose weapon to equip it in that hand.\nWhen the last hand lets go of an equipped "
            "weapon:\nmode 1 keeps it equipped and held;\nmode 2 unequips it into a loose weapon held in that hand: "
            "press grab again and release to drop or stash it, or pull the trigger to equip it again;\nmode 3 drops "
            "it in place, with the same release rules but no forced grab.\nMode 4 equips a loose weapon once your "
            "palm settles on its firing grip (see Auto Equip Settle Time), without the trigger.\nFar grabs seat a "
            "loose weapon at its firing grip and close grabs at its nearest grip; touch grabs equip only inside a "
            "grip zone.\nWith a weapon already in the other hand, the new one becomes the second weapon.\nApplies "
            "per weapon. Weapon Grab Mode sets the release gesture in every mode.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "WeaponGrabMode", "04. Weapon Handling",
            "Weapon Grab Mode\n1 = Toggle both grips. 2 = Toggle firing grip only. 3 = Hold both grips.\nToggle: "
            "press grab to take the grip and press again to let go. Hold: the grip lasts while you hold grab.\nMode "
            "2 toggles whichever hand holds the firing grip; support grips must be held.\nParts you only attach to "
            "always need grab held. Ordinary objects and surfaces always use hold to grab.\nWeapon Equip Mode "
            "decides whether the last release keeps a loose grab or drops the weapon.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "AutoEquipSettleSeconds", "04. Weapon Handling",
            "Auto Equip Settle Time\nSeconds your palm must stay on the firing grip before Auto Equip (Weapon Equip "
            "Mode 4) equips. Range: 0 to 2.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "GrabAnywhereOnWeapon", "04. Weapon Handling",
            "Grab Anywhere on Weapon\nTrue: your support hand can hold a firearm anywhere.\nFalse: firearms with "
            "authored support grips take the support hand only there (default).\nMelee weapons, and firearms without "
            "an authored support grip, can always be held anywhere.\nApplies to either support hand.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "MeleeGripPitchDegrees", "04. Weapon Handling",
            "Melee Grip Angle\nTilts a held melee weapon and your hand together around the grip, in degrees, on top "
            "of ROCK's built-in 30 degree forward tilt.\nPositive tilts further forward, negative tilts back; -30 "
            "gives the weapon's authored angle. Range: -180 to 180.\nRegrab a loose weapon to see the change.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "WeaponFilteringPreset", "04. Weapon Handling",
            "Shake Filtering\n0 = Off. 1 = Responsive. 2 = Balanced. 3 = Steady. 4 = Heavy.\nSmooths controller "
            "shake on equipped firearms, loose weapons and held objects. Higher steadies more but responds more "
            "slowly.\nRotation and position smoothing: Responsive 12 and 6 ms, Balanced 25 and 12 ms, Steady 45 and "
            "22 ms, Heavy 70 and 35 ms.\nFree hands, FRIK and recoil are not affected. Applies at once.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "ScopedFilterRotationSeconds", "04. Weapon Handling",
            "Scoped Filtering: Rotation\nExtra rotation smoothing, in seconds, while you look through a scope, with "
            "or without Shake Filtering.\n0 here and in Scoped Filtering: Position turns scoped filtering off. "
            "Range: 0 to 1.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "ScopedFilterTranslationSeconds", "04. Weapon Handling",
            "Scoped Filtering: Position\nExtra position smoothing, in seconds, while you look through a scope. "
            "Range: 0 to 1.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "ScopedFilterReleaseDegreesPerSecond", "04. Weapon Handling",
            "Scoped Filtering: Release Speed\nHow fast you turn the controller, in degrees per second, before the "
            "scoped smoothing drops to half.\nHolding still gets full smoothing; swinging to a new target loosens "
            "it.\n0 keeps full smoothing at any speed. Range: 0 to 720.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "FiringGripReattachRadiusGameUnits", "04. Weapon Handling",
            "Firing Grip Reach\nHow far from the firing grip a hand can take it over or grab it again, in game units "
            "(70 = 1 metre).\nThe zone is a quarter circle on each side of the grip, from straight out to straight "
            "down.\nPress grab inside it to move the firing grip to that hand; the firing hand can also pass it with "
            "a grab tap. Range: 0.25 to 30.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "GripZoneIndicatorDiameterGameUnits", "04. Weapon Handling",
            "Grip Zone Marker Size\nDiameter of the sphere that marks a weapon's grip zones, in game units (70 = 1 "
            "metre). Range: 0.1 to 5.",
            api::configuration::Group::Consumer },
        { "WeaponHandling", "AmmoCounterStyle", "04. Weapon Handling",
            "Ammo Counter Style\n1 = Minimal. 2 = Immersive. 3 = Vanilla.\nMinimal: white loaded and reserve rounds "
            "beside the firing grip on the palm side, shown while you look at them.\nImmersive: the same place and "
            "look to show, in the game's HUD color and bracket frame, loaded over reserve.\nVanilla: the game's "
            "counter on the back of the hand holding the weapon, in HUD color, always shown (mirrored on the left "
            "hand).\nEvery style replaces the game's counter with one counter per drawn weapon. The counters are "
            "drawn through RPS UI Framework.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "ImmersiveRecoil", "05. Weapon Recoil",
            "Immersive Recoil\nUses the recoil strengths below, for either hand and in power armor.\nFalse: the "
            "game's own recoil strength, on the hand that fires.\nThe developer setting Emergency Disable Recoil "
            "overrides both.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "LaserRecoilPercent", "05. Weapon Recoil",
            "Laser Recoil\nRecoil for laser weapons in any hold and in power armor: 100 = 10% of the game's "
            "movement and rotation, 0 = none. Range: 0 to 300.\nA bipod-latched weapon keeps its fixed 10%.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "PistolOneHandRecoilPercent", "05. Weapon Recoil",
            "Recoil Strength by Weapon Type\nPercent of each weapon type's recoil: 0 = none, 100 = full, 300 = "
            "triple. Range: 0 to 300.\nOne Hand applies without a support grip, Two Hand with one.\nBipod latching "
            "and power armor use their own fixed recoil.\nThis key: pistols held with one hand.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "PistolTwoHandRecoilPercent", "05. Weapon Recoil", "Pistols held with two hands. Range: 0 to 300.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "RifleOneHandRecoilPercent", "05. Weapon Recoil", "Rifles held with one hand. Range: 0 to 300.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "RifleTwoHandRecoilPercent", "05. Weapon Recoil", "Rifles held with two hands. Range: 0 to 300.",
            api::configuration::Group::Consumer },
        { "WeaponRecoil", "ShotgunOneHandRecoilPercent", "05. Weapon Recoil",
            "Shotguns held with one hand. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "WeaponRecoil", "ShotgunTwoHandRecoilPercent", "05. Weapon Recoil",
            "Shotguns held with two hands. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "WeaponRecoil", "HeavyOneHandRecoilPercent", "05. Weapon Recoil",
            "Heavy weapons held with one hand. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "WeaponRecoil", "HeavyTwoHandRecoilPercent", "05. Weapon Recoil",
            "Heavy weapons held with two hands. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "WeaponRecoil", "DefaultOneHandRecoilPercent", "05. Weapon Recoil",
            "Any other firearm held with one hand. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "WeaponRecoil", "DefaultTwoHandRecoilPercent", "05. Weapon Recoil",
            "Any other firearm held with two hands. Range: 0 to 300.", api::configuration::Group::Consumer },
        { "Grabbing", "FarGrabMode", "06. Grabbing",
            "Far Grab Mode\n1 = Immediate Pull. 2 = Higgs Pull Mode.\nImmediate Pull: grabbing a far object pulls it "
            "to your hand and catches it, with no flick.\nHiggs Pull Mode: hold grab to lock on, flick your hand "
            "toward yourself, then keep holding to pull the object in and catch it.\nRelease before the flick or "
            "during the pull to cancel. Applies to the next far grab.",
            api::configuration::Group::Consumer },
        // PALM builds its dropdown from the "N = label" pairs; labels may not contain , ; or a full stop.
        { "Grabbing", "CloseGrabDirection", "06. Grabbing",
            "Which way your hand reaches for nearby objects.\n0 = Out of the palm. 20 = Mostly out of the palm. "
            "45 = Between palm and fingers. 75 = Mostly along the fingers. 90 = Along the fingers.\nThe number is the "
            "angle in degrees from the palm toward the fingers. Applies at once, on each hand.",
            api::configuration::Group::Consumer },
        { "Grabbing", "FarGrabDirection", "06. Grabbing",
            "Which way your hand points to aim far grabs.\n0 = Out of the palm. 20 = Mostly out of the palm. "
            "45 = Between palm and fingers. 75 = Mostly along the fingers. 90 = Along the fingers.\nThe number is the "
            "angle in degrees from the palm toward the fingers. The object must also be in your view. Applies at "
            "once, on each hand.",
            api::configuration::Group::Consumer },
        { "Grabbing", "QuickLootPanelPlacement", "06. Grabbing",
            "Quick Loot Panel Placement\n1 = Right Hand. 2 = Closer Hand. 3 = At Container.\nRight Hand: the game's "
            "own panel on the right hand; it lists only what the right hand points at.\nCloser Hand: the panel rides "
            "the hand pointing at the container, mirrored on the left hand; when both hands point at containers the "
            "closer one takes it.\nAt Container: the panel stands upright at the container it lists, turned toward "
            "you, at a fixed size; either hand can point at it.\nThe button hints go with the panel; the buttons "
            "themselves stay on the right controller in every mode. Applies at once.",
            api::configuration::Group::Consumer },
        { "Grabbing", "ActivatePromptPlacement", "06. Grabbing",
            "Activate Prompt Placement\n1 = Both Hands. 2 = Closer Hand. 3 = At Object.\nThe prompts each hand shows "
            "for what it points at (Open, Talk, Take and the rest).\nBoth Hands: the game's own prompts on each hand; "
            "two hands on one door show it twice.\nCloser Hand: when both hands point at the same object only the "
            "closer hand shows its prompt.\nAt Object: the prompt stands upright at the object it names, turned "
            "toward you, at a fixed size, and shows once when both hands point at it.\nActivation itself is "
            "unchanged. Applies at once.",
            api::configuration::Group::Consumer },
        { "FiringHand", "DefaultHand", "07. Firing Hand",
            "Default Hand\nright or left: the firing hand for equips that come without a hand (favorites, PALM, "
            "scripts and other mods).\nGrenade draws prefer this hand and use the other one when it is busy.\nPip-Boy "
            "and trigger equips and grip handoffs always use the hand that acts.\nApplies on the next equip or draw. "
            "The game's own left-handed mode is not supported and stays off.",
            api::configuration::Group::Consumer },
        { "FiringHand.Left", "AimYawDegrees", "07. Firing Hand",
            "Left Firing Aim: Yaw\nTurns the aim of a weapon fired from the left hand, in degrees. 0 uses the plain "
            "mirror of the right-hand hold. Range: -30 to 30.",
            api::configuration::Group::Consumer },
        { "FiringHand.Left", "AimPitchDegrees", "07. Firing Hand",
            "Left Firing Aim: Pitch\nTilts the aim of a weapon fired from the left hand up or down, in degrees. "
            "Range: -30 to 30.",
            api::configuration::Group::Consumer },
        { "FiringHand.Left", "AimOffsetGameUnits", "07. Firing Hand",
            "Left Firing Aim: Offset\nShifts the aim of a weapon fired from the left hand, in game units (70 = 1 "
            "metre), as [x, y, z]. Each value: -15 to 15.",
            api::configuration::Group::Consumer },
        { "FiringHand.Left", "GripOffsetGameUnits", "07. Firing Hand",
            "Left Firing Grip\nMoves where the left hand holds the firing grip, in game units (70 = 1 metre), as "
            "[along the fingers, into or out of the palm, across the palm].\nAll 0 = the plain mirror of the "
            "right-hand grip. The hand stays put and the weapon moves the other way. Each value: -15 to 15. Regrab "
            "to apply.",
            api::configuration::Group::Consumer },
        { "FiringHand.Right", "SupportGripOffsetGameUnits", "07. Firing Hand",
            "Right Support Grip\nMoves where the right hand holds a support grip when the left hand fires, in game "
            "units (70 = 1 metre), as [along the fingers, into or out of the palm, across the palm].\nAll 0 = the "
            "plain mirror of the left-hand support grip. Each value: -15 to 15. Regrab to apply.",
            api::configuration::Group::Consumer },
        { "Scopes", "EnableImmersiveScopes", "08. Scopes",
            "Immersive Scopes\nROCK places the scope view on the weapon, raises it while you hold the scope button "
            "(right A by default), and corrects the overlay and zoom.\nFalse: the game's own scopes.\nStarting with "
            "this off leaves the game's scope hook free for other scope mods. Turning it on takes the hook until you "
            "restart; if another mod owns it, ROCK's scopes stay off.\nRelease the scope button before changing "
            "this. Reload stays on a tap of the same button.",
            api::configuration::Group::Consumer },
        { "Scopes", "ManualScopeHoldSeconds", "08. Scopes",
            "Scope Hold Time\nSeconds to hold the scope button (right A, or left X for a left firing hand) before the "
            "scope view comes up; a shorter press reloads. Range: 0.05 to 2.",
            api::configuration::Group::Consumer },
        { "ControllerInput", "SuppressNativeVats", "09. Controller Input",
            "Block VATS\nStops the game's VATS from opening on the VATS button (right B by default).\nThe button "
            "still works for ROCK and other mods.",
            api::configuration::Group::Consumer },
        { "ControllerInput", "VanillaFavoritesMenu", "09. Controller Input",
            "Vanilla Favorites Menu\nTrue: the game's Favorites menu opens on its usual button, the right stick "
            "click (default).\nWhile RobCo PALM is installed, ROCK always blocks the Favorites menu and the stick "
            "click opens PALM.\nWith Bipod Mode on, a click while the weapon touches a surface or is latched works "
            "the bipod instead of opening Favorites.\nFalse: ROCK blocks the Favorites menu and the stick click belongs to Bipod Mode.",
            api::configuration::Group::Consumer },
        { "ControllerInput", "OfferSteamVRBinding", "09. Controller Input",
            "Offer ROCK's SteamVR Binding\nTrue: when Index controllers or Vive wands use SteamVR's default binding "
            "for Fallout 4 VR, the setup wizard opens by itself once per game start, when no menu or scene is in "
            "the way, and offers ROCK's binding (the default "
            "binding turns A and X into grip). It never replaces a binding you chose in SteamVR.\nFalse: the wizard "
            "opens only when you open it; it still offers the binding then.",
            api::configuration::Group::Consumer },
        { "ControllerInput", "PipboyPauseHoldSeconds", "09. Controller Input",
            "Pip-Boy and Pause Hold Time\nTap left Y to open the Pip-Boy; hold it this many seconds to open the Pause "
            "menu instead. Range: 0.15 to 2.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Right", "Trigger", "09. Controller Input",
            "Button Layout\nThe physical input that acts as each ROCK button, per hand. ROCK, PALM, PAPER, SCISSORS "
            "Redux, FRIK and the game all follow it.\nThe defaults are the Quest and Touch layout and change "
            "nothing.\nValues: trigger, grip, a, b (x, y on the left), thumbstick, grip_click, axis3, axis4, pad_up, "
            "pad_down, pad_left, pad_right, pad_center (trackpad click regions), none.\nSwapping two buttons moves "
            "every action of each. Stick movement is never remapped.\nControllers whose SteamVR binding merges "
            "buttons (Index A reports as grip) need the binding fixed in SteamVR first; ROCK only sees what SteamVR "
            "sends.\nThis key: the right trigger.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Right", "Grab", "09. Controller Input", "The right grab button (grip by default).",
            api::configuration::Group::Consumer },
        { "ControllerInput.Right", "A", "09. Controller Input", "Right A: reload tap, scope hold.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Right", "B", "09. Controller Input", "Right B: grenade draw hold, VATS.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Right", "StickClick", "09. Controller Input",
            "Right stick click: bipod latch; opens PALM by default.", api::configuration::Group::Consumer },
        { "ControllerInput.Left", "Trigger", "09. Controller Input", "The left trigger.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Left", "Grab", "09. Controller Input", "The left grab button (grip by default).",
            api::configuration::Group::Consumer },
        { "ControllerInput.Left", "X", "09. Controller Input",
            "Left X: reload tap and scope hold for a left firing hand.", api::configuration::Group::Consumer },
        { "ControllerInput.Left", "Y", "09. Controller Input", "Left Y: Pip-Boy tap, Pause hold.",
            api::configuration::Group::Consumer },
        { "ControllerInput.Left", "StickClick", "09. Controller Input", "Left stick click.",
            api::configuration::Group::Consumer },
        { "SetupWizard", "ShownEdition", "09. Controller Input",
            "Setup Wizard Shown\nThe setup wizard opens by itself once, 10 seconds after you are in the game with no "
            "menu or scene in the way, whenever this ROCK brings a newer edition of it (new controls or options to "
            "show). On a new game it waits until you leave the mirror. ROCK writes the edition here when the wizard "
            "opens.\n0 = show it at the next game start. 999 = never open it by itself.",
            api::configuration::Group::Consumer },
        { "Bindings", "Grab", "09. Controller Input",
            "Bindings\nThe control and gesture of each function: \"<hand> <gesture> <control>\", or \"none\".\n"
            "Hands: firing (the hand on the weapon's firing grip), support, primary, offhand, left, right, each.\n"
            "Gestures: press (acts while held), tap, hold, double (double tap).\nControls: as in the button layout "
            "above. Add \"+<hand> <control>\" for a button that must be held at the same time.\nThe ROCK setup "
            "wizard, opened from PALM's Config, sets these by pressing the control.\nThis key: grab with each hand.",
            api::configuration::Group::Consumer },
        { "Bindings", "Fire", "09. Controller Input", "Fire the equipped weapon.", api::configuration::Group::Consumer },
        { "Bindings", "SupportTrigger", "09. Controller Input",
            "The support hand's trigger: a second weapon, or the support action of what that hand holds.",
            api::configuration::Group::Consumer },
        { "Bindings", "Use", "09. Controller Input",
            "Use what the primary hand points at (take, open, talk). It owns the whole press while there is something "
            "to use.", api::configuration::Group::Consumer },
        { "Bindings", "Reload", "09. Controller Input", "Reload the equipped weapon.",
            api::configuration::Group::Consumer },
        { "Bindings", "Scope", "09. Controller Input", "Look through the weapon's scope while active.",
            api::configuration::Group::Consumer },
        { "Bindings", "Vats", "09. Controller Input", "Open VATS.", api::configuration::Group::Consumer },
        { "Bindings", "Grenade", "09. Controller Input", "Draw a grenade into the hand.",
            api::configuration::Group::Consumer },
        { "Bindings", "Pipboy", "09. Controller Input", "Open the Pip-Boy.", api::configuration::Group::Consumer },
        { "Bindings", "Pause", "09. Controller Input", "Open the Pause menu.", api::configuration::Group::Consumer },
        { "Bindings", "StickActions", "09. Controller Input",
            "Bipod latch and held placement while the weapon touches a surface, and the game's Favorites menu.",
            api::configuration::Group::Consumer },
        { "Bindings", "Palm", "09. Controller Input", "Open RobCo PALM's wheel.", api::configuration::Group::Consumer },
        { "Bindings", "HoldSeconds", "09. Controller Input",
            "Hold Time\nSeconds a hold binding needs before it acts, for functions without their own hold time "
            "(Scope and Pause have theirs; Grenade uses the game's). Range: 0.1 to 2.",
            api::configuration::Group::Consumer },
        { "Bindings", "DoubleTapSeconds", "09. Controller Input",
            "Double Tap Time\nLongest gap between the two presses of a double tap. A tap on the same control waits "
            "this long before it acts. Range: 0.15 to 0.6.",
            api::configuration::Group::Consumer },
        { "NativeGameplay", "VanillaMeleeSwingSpeed", "10. Native Gameplay",
            "Melee Swing Speed\nHand speed that starts a vanilla melee swing, in game units per second (70 = 1 metre "
            "per second). The game uses 200.\nTurning the wrist alone never swings. Range: 50 to 2000.",
            api::configuration::Group::Consumer },
        { "NativeGameplay", "VanillaGunBashSpeed", "10. Native Gameplay",
            "Gun Bash Speed\nHand speed that starts a vanilla gun bash, in game units per second (70 = 1 metre per "
            "second). The game uses 200.\nTurning the wrist alone never bashes. Range: 50 to 2000.",
            api::configuration::Group::Consumer },
        { "SelectionFeedback", "HighlightEnabled", "11. Selection Feedback",
            "Grab Highlight\nOutlines the object your hand would grab.", api::configuration::Group::Consumer },
        { "SelectionFeedback", "HighlightIntensityMode", "11. Selection Feedback",
            "Grab Highlight Strength\n1 = 40%. 2 = 60%. 3 = 80%. 4 = 100%.", api::configuration::Group::Consumer },
        { "SelectionFeedback", "HighlightColor", "11. Selection Feedback", "Grab Highlight Color\nred, blue, orange or white.",
            api::configuration::Group::Consumer },
        { "DeveloperControls", "DeveloperModeEnabled", "01. Developer Controls",
            "Gates advanced/tuning-only features not meant for normal play. Currently:\npressing A while holding an "
            "object records/overwrites a saved grab offset\nfor that object+hand. Saved offsets keep applying on "
            "future grabs\nregardless of this setting - it only gates recording new ones.",
            api::configuration::Group::Developer },
        { "DeveloperControls", "EmergencyDisableRecoil", "01. Developer Controls",
            "Emergency Disable Recoil\nON: disables native VR recoil and kickback plus ROCK weapon/hand recoil, "
            "overriding every recoil profile.\nOFF: restores native recoil/kickback and the existing Immersive Recoil "
            "setting.\nApplies immediately, independently of Developer Mode. Does not change projectile spread.",
            api::configuration::Group::Developer },
        { "LoggingAndProfiling", "LogPattern", "02. Logging and Profiling", "Text format for each ROCK.log line.",
            api::configuration::Group::Developer },
        { "LoggingAndProfiling", "LogSampleMilliseconds", "02. Logging and Profiling",
            "Minimum time between repeated sampled diagnostic messages.", api::configuration::Group::Developer },
        { "LoggingAndProfiling", "LogFlushImmediate", "02. Logging and Profiling",
            "Writes each ROCK.log line to disk as it is logged.\nOff, the log is block buffered, so a freeze or a "
            "crash loses the last lines\nand every log ends mid-line. On, the tail survives, at one file write per "
            "line.\nLeave off unless you are chasing a hang.",
            api::configuration::Group::Developer },
        { "LoggingAndProfiling", "PerformanceProfilerEnabled", "02. Logging and Profiling",
            "Asynchronous performance profiler.\nIndependent from LogLevel. When enabled, timing scopes stay on the "
            "frame path,\nbut completed profiler windows are queued to a bounded background writer.\nOutput goes to "
            "ROCK_Profiler.log, not ROCK.log. File formatting/writes are not done\nby grab, reload, physics, or frame "
            "update code. If the writer falls behind, profiler\nwindows are dropped instead of stalling gameplay.",
            api::configuration::Group::Developer },
        { "LoggingAndProfiling", "PerformanceProfilerLogIntervalFrames", "02. Logging and Profiling",
            "Number of frames per queued profiler window after warmup. Larger values reduce file traffic.",
            api::configuration::Group::Developer },
        { "LoggingAndProfiling", "PerformanceProfilerWarmupFrames", "02. Logging and Profiling",
            "Startup/reload frames ignored before profiler windows are emitted.",
            api::configuration::Group::Developer },
        { "PalmAndGrabOffsets", "RightGrabAuthorityProxyOffsetGameUnits", "03. Palm and Grab Offsets",
            "Right-hand local offset [x, y, z] for the hidden grab authority proxy/seat point, in corrected palm "
            "proxy space.\nX follows the fingers, -Y moves into the palm face/skin, +Y moves to the back of the "
            "hand,\nand Z moves across the palm. This does not move the real palm/finger physics colliders.",
            api::configuration::Group::Developer },
        { "PalmAndGrabOffsets", "LeftGrabAuthorityProxyOffsetGameUnits", "03. Palm and Grab Offsets",
            "Left-hand local offset [x, y, z] for the hidden grab authority proxy/seat point, in the same axes.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionBlocksProjectiles", "04. Weapon Collision and Support",
            "Controls whether ROCK weapon hulls collide with projectile/spell layers.\nFalse avoids self-collisions "
            "with modded projectile systems while aiming.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionBlocksSpells", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponShellCollisionGraceMs", "04. Weapon Collision and Support",
            "New casings ignore the firing weapon's generated colliders for this many simulation "
            "milliseconds.\nPositive values protect at least one physics step. Zero disables the grace period. Range: "
            "0-500 ms.\nWorld contacts and native casing lifetime are preserved. Changes apply to new casings.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionPreserveGaps", "04. Weapon Collision and Support",
            "False: original convex fitting. True: bounded gap-preserving compounds.\nChanging this key rebuilds "
            "weapon colliders after a live settings reload.\nCompare with PerformanceProfilerEnabled: "
            "weaponColliderBuild/Create vs weaponGapColliderBuild/Create.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionVisualStabilizationSeconds", "04. Weapon Collision and Support",
            "Seconds ROCK waits for the visual weapon tree to stay visible/stable before replacing generated hulls "
            "after weapon/mod swaps.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionMaxLinearVelocity", "04. Weapon Collision and Support",
            "Maximum speed allowed when ROCK keyframes generated weapon hulls.", api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionMaxAngularVelocity", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionGripRecoveryDistanceGameUnits", "04. Weapon Collision and Support",
            "Immediately resets a failed dynamic weapon collider onto the firing hand if\nits live firing-grip "
            "authority point is farther away than this distance.\nThis catastrophic-failure limit is separate from "
            "normal wall-contact\nconvergence. 210 game units is about 3 metres. Game units (~1.4cm each).",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponInteractionTouchRadius", "04. Weapon Collision and Support",
            "Small sphere around each hand's palm point. If it overlaps a\ngenerated weapon collider, the grab uses "
            "ROCK's unrestricted dynamic seat.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponInteractionProbeRadius", "04. Weapon Collision and Support",
            "Broader per-hand search used when the palm is not touching the weapon.\nWith authored weapon grips "
            "enabled, a grab acquired only by this probe uses\nthe modeler's authored firing/support seat instead of "
            "the dynamic mesh seat.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "DebugWeaponSupportDynamicOnly", "04. Weapon Collision and Support",
            "Testing only: authored support seats are never used, so every support grab\nruns the dynamic mesh grip. "
            "Use to test dynamic grips where an authored zone\nwould win. Leave off for play.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "FiringGripProximitySupportRadius", "04. Weapon Collision and Support",
            "Every support grab inside this distance from the firing grip stays visual-only\nso pistols retain a "
            "stable support pose without competing weapon authority.\nROCK always owns this behavior. An active "
            "equipped-weapon addon may override\nonly the radius through the ROCK V1 API. Game units (~1.4cm each).",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpEnabled", "04. Weapon Collision and Support",
            "Eases ROCK's external FRIK hand target into an equipped weapon support grip.\nThis is visual-only: weapon "
            "aim authority remains immediate.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMin", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMax", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMinDistance", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMaxDistance", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatEnabled", "04. Weapon Collision and Support",
            "Seats the visual support palm toward the contacted surface without rotating the weapon.",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatMaxDegrees", "04. Weapon Collision and Support", "",
            api::configuration::Group::Developer },
        { "WeaponCollisionAndSupport", "FiringGripReattachCylinderRadiusGameUnits", "04. Weapon Collision and Support",
            "Thickness of the firing-grip reattach zone in game units (70 = 1 metre): how far above the grip point "
            "and how far forward or back along the barrel the palm may sit.\nA handling mod that selects the cylinder "
            "zone uses it as the cylinder radius. Range: 0.1 to 30.",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandCollisionStaticWorldEnabled", "05. Hand and Body Collision",
            "Allows generated hand colliders to collide with static/animstatic world geometry for native contact "
            "evidence.",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "GlobalSurfaceGrabEnabled", "05. Hand and Body Collision",
            "Makes dynamic-hand world-surface contacts eligible for fixed-anchor grab fallback.",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "SurfaceMeshGrabEnabled", "05. Hand and Body Collision",
            "Uses the contacted render mesh as the spatial authority for fixed-surface grabs.",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "SurfaceMeshGrabMaxProjectionDistanceGameUnits", "05. Hand and Body Collision", "",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "SurfaceMeshGrabMaxTriangles", "05. Hand and Body Collision", "",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandBoneColliderRadiusScaleOverrides", "05. Hand and Body Collision",
            "Optional generated hand capsule-like collider radius scale overrides. Palm-box roles are controlled by "
            "X/Y/Z below.\nRole names: ThumbBase, ThumbMiddle, ThumbTip, IndexBase, IndexMiddle, IndexTip, MiddleBase, "
            "MiddleMiddle,\nMiddleTip, RingBase, RingMiddle, RingTip, PinkyBase, PinkyMiddle, PinkyTip.\nFormat: "
            "Role=radiusScale;PowerArmor.Role=radiusScale",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandPalmColliderDimensionScaleOverrides", "05. Hand and Body Collision",
            "Optional generated palm-box collider local dimension scale overrides.\nLocal X=fingers/segment length, "
            "local Y=palm depth, local Z=cross-palm width.\nRole names: PalmAnchor, PalmFace, PalmBack, PalmHeel, "
            "ThumbPad.\nFormat: Role=xScale,yScale,zScale;PowerArmor.Role=xScale,yScale,zScale",
            api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandBoneCollidersRequirePalmAnchor", "05. Hand and Body Collision",
            "Requires the palm anchor before creating hand bone colliders.", api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandBoneColliderMaxLinearVelocity", "05. Hand and Body Collision",
            "Speed clamps for bone-driven hand colliders.", api::configuration::Group::Developer },
        { "HandAndBodyCollision", "HandBoneColliderMaxAngularVelocity", "05. Hand and Body Collision", "",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullToObjectCenterEnabled", "06. Selection and Pulling",
            "Far grabs pull the visual mesh point nearest the mesh's centre to the hand\ninstead of the mesh point "
            "nearest the ray hit (a hit at selection range is aim\nnoise, not grip intent). The arrival seat then "
            "settles the mesh onto the palm.",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullLongAxisPresentationEnabled", "06. Selection and Pulling",
            "Long-object presentation during pull flight: rotates elongated props (brooms,\nbottles) so their mesh "
            "long axis lines up with the hand's thumb->pinky line\nbefore arrival. Flight-only; held objects are never "
            "rotated. Elongation is the\nRMS extent ratio between the mesh's dominant and second axis (compact "
            "objects\n~1 skip presentation). Gain is proportional rad/s per rad of error.",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullPresentationMinElongationRatio", "06. Selection and Pulling", "",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullPresentationAngularGainPerSecond", "06. Selection and Pulling", "",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullPresentationMaxAngularSpeedRadiansPerSecond", "06. Selection and Pulling", "",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "PullPresentationGripAxisTiltDegrees", "06. Selection and Pulling",
            "Tilts the presentation target off pure cross-palm Z toward fingers-forward\n+X, in degrees (0..45): the "
            "thumb occupies the pure-Z line, so the natural\nlong-object hold sits a few degrees toward the "
            "fingertips. Applies to both\nthe pull-flight servo and the force-grab seat alignment.\nThis is an "
            "anatomical magnitude, not a signed rotation. Authored Z reverses\nits cross-palm direction between hands, "
            "but +X points toward the fingertips\non both. ROCK resolves the live axes first, then tilts each signed Z "
            "axis\ntoward its own +X, so one value has the same meaning for both hands.",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "ForceGrabSeatAlignmentEnabled", "06. Selection and Pulling",
            "Applies long-axis presentation alignment to force-grab arrival seats.",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "GrabSeatRollAlignmentEnabled", "06. Selection and Pulling",
            "Applies face-to-palm roll alignment to programmatic arrival seats.",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "GrabSeatRollMinSecondElongationRatio", "06. Selection and Pulling",
            "Shape-classification gate shared by the flight presentation servo and the\ngrab pose solver: "
            "sqrt(lambda2/lambda3) of the mesh PCA at or above this\nratio marks a distinct flat face (plate class). "
            "The solver uses the class\nfor its rod-only terms; plates keep their edge/corner holds (31 "
            "verified\nplate holds put the face normal 63 deg off the palm normal - edge holds are\nthe intended grip, "
            "not an artifact).",
            api::configuration::Group::Developer },
        { "SelectionAndPulling", "ObjectPhysicsTreeMaxDepth", "06. Selection and Pulling",
            "Maximum scene-node depth used when scanning an object for physics bodies.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLinearTau", "07. Grab Drive",
            "Shared dynamic-grab response for held objects and equipped weapon colliders. Higher tau increases "
            "positional recovery. Damping calms oscillation; angular response remains independent.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLinearDamping", "07. Grab Drive", "", api::configuration::Group::Developer },
        { "GrabDrive", "GrabLinearProportionalRecovery", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLinearConstantRecovery", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabAngularTau", "07. Grab Drive",
            "Angular shared-constraint motor response. Same idea as linear motor, but for rotation.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabAngularDamping", "07. Grab Drive", "", api::configuration::Group::Developer },
        { "GrabDrive", "GrabAngularProportionalRecovery", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabAngularConstantRecovery", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabConstraintMaxForce", "07. Grab Drive",
            "Contact and initial-attachment linear force ceiling. Free motion uses actual mass and Free Linear "
            "Acceleration.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabEffectiveMotorMassFloorEnabled", "07. Grab Drive",
            "Keeps the motor's effective mass finite for very light or unstable bodies.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabEffectiveMotorMassFloor", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabMaxForceToMassRatio", "07. Grab Drive",
            "Contact force per unit effective mass. This caps the contact ceiling for light objects.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabFreeLinearAcceleration", "07. Grab Drive",
            "Free-motion linear effort per unit effective driven mass, in Havok acceleration units. Contact keeps its "
            "separate ceiling.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabFreeAngularAcceleration", "07. Grab Drive",
            "Free-motion angular effort per unit maximum principal inertia. The grip lever adds capacity to oppose "
            "off-center linear effort; the native solver determines the applied torque.",
            api::configuration::Group::Developer },
        { "GrabDrive", "ForceGrabAttachSettleSeconds", "07. Grab Drive",
            "Waits for a stable live hand transform before a force grab freezes its relation.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabForceFadeInTime", "07. Grab Drive",
            "Time for grab force to ramp in after pickup.", api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearTauMultiplier", "07. Grab Drive",
            "Weapon response multipliers for both loose and equipped weapons. Max Force and Angular Force multipliers "
            "control contact and initial-attachment budgets. Free-motion budgets use actual mass, inertia, and the "
            "grip offset.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularTauMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintCollisionTauMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearDampingMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularDampingMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintMaxForceMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularForceMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearRecoveryMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularRecoveryMultiplier", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabTauMin", "07. Grab Drive",
            "Contact-softened tau. Normal held linear/angular tau stay fixed at their base values.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabTauLerpSpeed", "07. Grab Drive", "", api::configuration::Group::Developer },
        { "GrabDrive", "GrabLongObjectAngularScalingEnabled", "07. Grab Drive",
            "Reduces held angular speed for long objects by grip-to-extents lever length.\nThis keeps long "
            "guns/objects from sweeping as if their far end had no leverage.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLongObjectReferenceLeverGameUnits", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabLongObjectMinAngularScale", "07. Grab Drive", "",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabMaxInertiaRatio", "07. Grab Drive",
            "Clamps extreme inertia tensors on grabbed rigid bodies to keep the solver stable.",
            api::configuration::Group::Developer },
        { "GrabDrive", "GrabMinInertia", "07. Grab Drive", "", api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ThrowVelocityMultiplier", "08. Grab Safety and Release",
            "Multiplies held-object velocity when you release to throw.", api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabControllerDerivedThrowVelocityEnabled", "08. Grab Safety and Release",
            "Throws follow tracked hand movement. Wrist rotation supplies spin without steering the throw.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabThrowMaxVelocityHavok", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabThrowAngularVelocityScale", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabThrowMaxAngularVelocityRadiansPerSecond", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabReleaseHandCollisionDelaySeconds", "08. Grab Safety and Release",
            "Keeps hand collision suppressed briefly after release so the object is not launched instantly.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashEnabled", "08. Grab Safety and Release",
            "Shoulder stash transfers a held loose item to inventory when grip is released inside the HMD-relative "
            "back volume.\nGenerated FRIK body-zone colliders are backup evidence for tracking loss and remain the "
            "preferred path for holsters.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "EquippedWeaponShoulderStashEnabled", "08. Grab Safety and Release",
            "Equipped weapons, either one, use the same HMD-relative back volumes and return to the "
            "inventory.\nToggle-grab: press inside with the hand carrying the weapon.\nHold-to-grab: hold inside and "
            "release.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashUseBodyZoneColliders", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashUseHmdBackVolume", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashExitPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashRecentContactSeconds", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashSustainedContactMissSeconds", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackRightOffsetGameUnits", "08. Grab Safety and Release",
            "Centre of the right shoulder stash volume behind the headset, [x, y, z] in game units.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackLeftOffsetGameUnits", "08. Grab Safety and Release",
            "Centre of the left shoulder stash volume behind the headset, [x, y, z] in game units.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackRadiusGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackExitPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackMinBehindGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashShowCollectedNotifications", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeEnabled", "08. Grab Safety and Release",
            "Mouth consume uses a tight HMD-relative sphere near the mouth.\nOnly loose ALCH ingestibles can become "
            "mouth candidates; other objects use backpack stash/drop paths.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeAllowPoison", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeHmdOffsetGameUnits", "08. Grab Safety and Release",
            "Mouth position relative to the headset, [x, y, z] in game units. Each value: -120 to 120.",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeRadiusGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeExitPaddingGameUnits", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeMinDwellSeconds", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeMaxSpeedGameUnitsPerSecond", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementSlowdownEnabled", "08. Grab Safety and Release",
            "Reduces player movement speed while carrying held mass.", api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMassProportion", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMassExponent", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMaxReduction", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementFadeOutSeconds", "08. Grab Safety and Release", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabInputIntentStateEnabled", "09. Grab Acquisition",
            "Grab Press Leeway\nKeeps a held grab press alive while the hand becomes ready to grab (after a selection "
            "change, a pull catch or input suppression), so a press made a moment early still grabs.\nReleasing grab "
            "always cancels at once.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabInputLeewaySeconds", "09. Grab Acquisition",
            "Seconds a held grab press waits for the hand to become ready to grab. Range: 0 to 0.5.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabInputForceSeconds", "09. Grab Acquisition",
            "After that wait, seconds the held press keeps being sent every frame before ROCK stops trying. Range: 0 "
            "to 0.3.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabTouchAcquireDistanceGameUnits", "09. Grab Acquisition",
            "Distance from the hand pocket where an object is considered already touching.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabNearConvergeDistanceGameUnits", "09. Grab Acquisition",
            "Distance where the object should converge into the palm pocket instead of finalizing immediately.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabTwoHandPivotMinSeparationGameUnits", "09. Grab Acquisition",
            "Two hands on one loose non-weapon object turn it along the line between their grip points only when "
            "those points are at least this far apart.\nCloser than this (two pinches on a cartridge), the object "
            "stays as the first hand holds it, and the hand left holding it keeps its own grip.\n0 = always pivot. "
            "Applies to the next two-hand hold.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabPocketDepthGameUnits", "09. Grab Acquisition",
            "Hand-pocket offset and radius, in game units, from the root-flattened palm frame.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabPocketRadiusGameUnits", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabSeatDepthMaxGameUnits", "09. Grab Acquisition",
            "Legacy seat-depth stop and penetration backstop dimensions, in game units.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabSeatDepthFootprintRadiusGameUnits", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabSeatPenetrationBackstopFootprintRadiusGameUnits", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabSeatDepthSkinGameUnits", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabCloseSeatMode", "09. Grab Acquisition",
            "0 = Current seat. 1 = Local grasp solve.\nApplies on the next close grab of a loose object. Pulled, "
            "forced, pinched and authored grabs are unchanged.\nMode 1 reads the mesh in front of the palm. A handle, "
            "neck, rim or round body is turned into the hand,\nand the fingers close on that part instead of the body "
            "behind it.\nWhen no such part is found the current seat is used. ROCK.log reports the result of each "
            "grab.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabGripInsetGameUnits", "09. Grab Acquisition",
            "Moves the captured grip point slightly inside the object from the raw hit/mesh seed.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabConvergeMaxTimeSeconds", "09. Grab Acquisition",
            "Optional near-converge diagnostic timeout. A value of 0 disables timeout promotion.",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabConvergeStableSeconds", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabConvergeMaxSeparatingSpeedGameUnitsPerSecond", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabAcquisition", "GrabAcquisitionVisualStartDistanceGameUnits", "09. Grab Acquisition", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabSurfaceBehindPalmToleranceGameUnits", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabOppositionContactMaxAgeSeconds", "10. Grab Contact Evidence",
            "Maximum age in seconds for semantic contacts used as touch evidence.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchPocketEnabled", "10. Grab Contact Evidence",
            "Small-object pinch pocket. Chosen only at close-grab commit, then frozen for the hold.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchCloseSelectionEnabled", "10. Grab Contact Evidence",
            "Allows a fallback close-selection cast along the pinch direction when the palm close cast misses.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchMaxPocketDistanceGameUnits", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchMinFingerGapGameUnits", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchMaxFingerGapGameUnits", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchThumbIndexMaxOpenValue", "10. Grab Contact Evidence",
            "Thumb and opposed finger curl of the weapon support hand's opposition pose\n(1 open, 0 closed). A "
            "loose-object pinch opens to the object's thickness instead.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchOtherFingerCurlValue", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchDetectionDirectionHandspace", "10. Grab Contact Evidence",
            "Pinch detection direction [x, y, z] in ROCK hand space: X=fingers, Y=palm depth, Z=signed cross-palm.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabPinchDetectionAxisBlend", "10. Grab Contact Evidence",
            "Blends the live thumb-index axis with the authored pinch direction for mesh surface search.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpEnabled", "10. Grab Contact Evidence",
            "Moves the rendered hand toward the mesh-matched held pose during grab startup.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpTimeMin", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpTimeMax", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpMinDistance", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpMaxDistance", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnEnabled", "10. Grab Contact Evidence",
            "Smooths the rendered hand back to live tracking after releasing a grabbed object.",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnTimeMin", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnTimeMax", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnMinDistance", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnMaxDistance", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnMinAngleDegrees", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnMaxAngleDegrees", "10. Grab Contact Evidence", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "WeaponGrabPoseMode", "11. Finger Poses",
            "0 = Follow grab finger mode. 1 = Current swept arc. 2 = July 9 calibrated slice.\n3 = Capsule probes "
            "(testing): the loose-object finger sweep, with thumb and index reaching onto small parts such as bolts "
            "and slides.\nApplies on the next "
            "dynamic equipped weapon or part grip, on either hand.\nJuly 9 uses the contacted part, translation-only "
            "palm seating and captured finger directions.\nUses current skeleton calibration and transform safety "
            "checks. Authored poses retain priority.",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerPoseMode", "11. Finger Poses",
            "1 = Current swept arc. 2 = June 2 legacy plane slice.\nApplies on the next dynamic weapon support grip "
            "when WeaponGrabPoseMode = 0.\nLoose-object grabs use LooseGrabFingerPoseMode.\nLegacy captures at "
            "contact and omits later pad refinement and splay.\nAuthored and saved finger poses keep their existing "
            "priority.",
            api::configuration::Group::Developer },
        { "FingerPoses", "LooseGrabFingerPoseMode", "11. Finger Poses",
            "0 = Capsule sweep.\n1 = Swept arc: the finger solve loose grabs used before the capsule sweep\n(pad-probe "
            "refinement, contact splay, surface-aim local transforms).\nApplies on the next loose-object grab. Weapon "
            "grips use WeaponGrabPoseMode.",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabMeshFingerPoseEnabled", "11. Finger Poses",
            "Computes custom finger curl from the held object's visible mesh.", api::configuration::Group::Developer },
        { "FingerPoses", "GrabMeshJointPoseEnabled", "11. Finger Poses",
            "Sends per-joint finger values to FRIK when available instead of one curl per finger.",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerMinValue", "11. Finger Poses",
            "Lowest allowed finger value. 0=fully closed, 1=fully straight.", api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerPoseSmoothingSpeed", "11. Finger Poses",
            "Maximum per-second finger pose movement.", api::configuration::Group::Developer },
        { "FingerPoses", "GrabMeshLocalTransformPoseEnabled", "11. Finger Poses",
            "Uses FRIK's authored finger joint transforms plus bounded mesh-contact aim correction.",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerLocalTransformSmoothingSpeed", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerLocalTransformMaxCorrectionDegrees", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerSurfaceAimStrength", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerRejectBacksideHits", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerSurfacePlaneToleranceGameUnits", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerSweepContactRadiusGameUnits", "11. Finger Poses",
            "Swept-arc finger solver: finger-pad contact sphere radius. Bigger = fingers\nstop earlier (hover), "
            "smaller = deeper wrap before contact.\nFinger-pad contact radius for the swept-arc solver (0.05-4). The "
            "old\nfinger-width air gap was a solver bug (fixed: chord de-rotation), NOT this\nradius - values near 0 "
            "now make fingers clip into the mesh.",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabFingerSweepMaxOpenValue", "11. Finger Poses",
            "How far past the authored open pose fingers may hyper-extend when an\nobject is thicker than the open "
            "finger span (hFRIK flex units: 1.0 =\nauthored open pose, 2.0 = engine ceiling). The swept-arc close "
            "starts\nfrom the most-open allowed row, so a raised cap lets fingers lie flat\non surfaces that "
            "interpenetrate the default open hand. Valid range\n1.0-2.0; out-of-range silently falls back (1.4 fingers "
            "/ 2.0 thumb).",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabThumbSweepMaxOpenValue", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabThumbOppositionStrength", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabThumbAlternateCurveStrength", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabThumbSurfaceSafetyEnabled", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "FingerPoses", "GrabThumbSurfaceSafetyMarginGameUnits", "11. Finger Poses", "",
            api::configuration::Group::Developer },
        { "MeshContact", "GrabLateralWeight", "12. Mesh Contact",
            "Weighting used when choosing the best mesh triangle for grab contact.",
            api::configuration::Group::Developer },
        { "MeshContact", "GrabDirectionalWeight", "12. Mesh Contact", "",
            api::configuration::Group::Developer },
        { "MeshContact", "GrabMaxTriangleDistance", "12. Mesh Contact", "",
            api::configuration::Group::Developer },
        { "MeshContact", "GrabContactPatchMaxNormalAngleDegrees", "12. Mesh Contact", "",
            api::configuration::Group::Developer },
        { "MeshContact", "GrabAlignmentMaxSelectionToMeshDistance", "12. Mesh Contact",
            "Furthest the grab point may snap from the selection hit onto the mesh.",
            api::configuration::Group::Developer },
        { "CloseHandCurl", "SelectedCloseFingerCurlEnabled", "13. Close-Hand Curl",
            "Pre-curls the hand when close selection is active.", api::configuration::Group::Developer },
        { "CloseHandCurl", "SelectedCloseFingerAnimMaxHandSpeed", "13. Close-Hand Curl", "",
            api::configuration::Group::Developer },
        { "CloseHandCurl", "SelectedCloseFingerAnimValue", "13. Close-Hand Curl", "",
            api::configuration::Group::Developer },
        { "Haptics", "FiringGripHapticDurationSeconds", "14. Haptics",
            "Length of the pulse on the acting hand when the firing grip attaches or detaches, in seconds. Range: 0.01 "
            "to 0.5.",
            api::configuration::Group::Developer },
        { "Haptics", "FiringGripAttachHapticIntensity", "14. Haptics",
            "Strength of the firing-grip attach pulse. Range: 0 to 1.", api::configuration::Group::Developer },
        { "Haptics", "FiringGripDetachHapticIntensity", "14. Haptics",
            "Strength of the firing-grip detach pulse. Range: 0 to 1.", api::configuration::Group::Developer },
        { "Haptics", "GrabHapticsEnabled", "14. Haptics",
            "Pulses for grab commit, pull start/catch, and held-object impacts.",
            api::configuration::Group::Developer },
        { "Haptics", "GrabHapticDurationSeconds", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "GrabHapticBaseIntensity", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "GrabHapticMaxIntensity", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "GrabHapticMassScale", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "GrabHapticMassExponent", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "PullStartHapticIntensity", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "PullCatchHapticIntensity", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "SelectionLockHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "SelectionLockReleaseHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "SelectionLockReleaseHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "SurfaceGrabHapticsEnabled", "14. Haptics",
            "Fixed-surface grabs use a longer confirmation pulse.", api::configuration::Group::Developer },
        { "Haptics", "SurfaceGrabHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "SurfaceGrabHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticsEnabled", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticBaseIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticMaxIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticSpeedScale", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticMassScale", "14. Haptics", "", api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticMassExponent", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticMinSpeedGameUnits", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticCooldownSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "HeldImpactHapticDampedMultiplier", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashHapticsEnabled", "14. Haptics",
            "Shoulder stash haptics pulse clearly while an item is inside the stash zone, then fire a stronger "
            "confirmation on transfer.",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCandidateHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCandidateHapticBaseIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCandidateHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCandidateHapticIntervalSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCommitHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "ShoulderStashCommitHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeHapticsEnabled", "14. Haptics",
            "Mouth consume haptics pulse while a consumable is in the mouth zone, then fire a confirmation on consume.",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCandidateHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCandidateHapticBaseIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCandidateHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCandidateHapticIntervalSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCommitHapticDurationSeconds", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "MouthConsumeCommitHapticIntensity", "14. Haptics", "",
            api::configuration::Group::Developer },
        { "Haptics", "DebugDrawGrabAuthorityProxy", "14. Haptics",
            "Draws the hidden no-contact grab authority proxy target/axes used by ROCK dynamic grab.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowColliders", "15. Debug Overlay",
            "Master switch for all ROCK-owned collider visualizations below.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowTargetColliders", "15. Debug Overlay",
            "Draws the selected or held target body when the collider master is enabled.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowHandAxes", "15. Debug Overlay",
            "Draws live transform axes. Red=X, green=Y, blue=Z.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowGrabPivots", "15. Debug Overlay",
            "Draws grab constraint pivots and the error line between hand-side and object-side pivots.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowGrabPocketNormal", "15. Debug Overlay",
            "Draws the resolved mesh surface frame after a dynamic grab.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabForceTorque", "15. Debug Overlay",
            "Draws a focused held-grab pivot force/lever view: target pivot, live pivot, correction vector, lever arm, "
            "and torque witness.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabForceTorqueText", "15. Debug Overlay",
            "Adds a compact world label to the focused grab force/lever view.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPivotSourceCollider", "15. Debug Overlay",
            "Draws the hknp body/collider that owns the frozen object-side pivot B.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPivotSourceEvidence", "15. Debug Overlay",
            "Draws the mesh triangle/contact-patch evidence that produced the frozen pivot.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabSupportFrame", "15. Debug Overlay",
            "Draws the frozen grab support face frame at pivot B: normal, support axis, and binormal.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPockets", "15. Debug Overlay",
            "Draws the live palm pocket and thumb/index pinch pocket for each hand.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabFingerProbes", "15. Debug Overlay",
            "Draws the grab finger solve live: while a hand has an object selected, the finger spheres the solve "
            "tests at the curl it would choose (grey wire = closed without touching, green = closed onto the "
            "surface, yellow = opened to clear, red = blocked; bright spheres touch the mesh) and each fingertip's "
            "sweep path. While holding, the spheres follow the drawn fingers.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugShowPalmVectors", "15. Debug Overlay",
            "Draws palm center plus close/far selection aim rays.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawHandColliders", "15. Debug Overlay",
            "More specific overlay filters and draw limits.", api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawHandBoneColliders", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawBodyBoneColliders", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawDynamicHandColliders", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawHandBoneContacts", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxHandBoneBodiesDrawn", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxBodyBoneBodiesDrawn", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawWeaponColliders", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGrabbedWeaponPartCollider", "15. Debug Overlay",
            "Draws the exact current weapon-part collider selected by each active part grip.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawNativeScopeActivation", "15. Debug Overlay",
            "Native-scope diagnosis: sight bounds, live camera, exact pre-activation target pose/aim/up guides, write "
            "readback, parent/local camera, and HMD guides.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawAuthoredGripActivationZones", "15. Debug Overlay",
            "Visualizes the topology-specific authored cone/arc, live approach vector, and\noptional palm/fingertip "
            "collision witnesses. Witnesses are diagnostic only.\nAlso draws the firing-grip reattach zone and live "
            "palm vectors during part carry.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawWeaponAuthority", "15. Debug Overlay",
            "Draws two-handed weapon/hand/FRIK authority frames and mismatch evidence.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawControllerAim", "15. Debug Overlay",
            "Compare both tracked wand frames, reference aim (cyan), configured aim (gold), damped aim (purple) and "
            "final barrel (green). The reference basis is a model, not physical ground truth. Does not change aiming.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawGunstock", "15. Debug Overlay",
            "Gunstock measurements: exact tracked controller origins/axes, controller baseline, weapon forward and "
            "authored hand seats. Each support grab marks a sample. Does not change aiming.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawLooseWeaponGripZones", "15. Debug Overlay",
            "Draws the loose-weapon projected grip point, palm relation, and equip radius state.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugDrawDynamicWeaponColliders", "15. Debug Overlay",
            "Draws the layer-51 dynamic weapon compound proxy and contact/correction telemetry.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxWeaponBodiesDrawn", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeCapturesPerFrame", "15. Debug Overlay",
            "Bounded overlay work and memory budgets. Zero disables the corresponding capture/upload/draw stream where "
            "noted.\nRanges: captures 0..32, compound slots 1..1024, depth 1..8, queued/completed 0..256,\nuploads "
            "0..16, cache entries 16..4096, cache bytes 1048576..536870912,\nbody instances 0..195, line vertices "
            "0..65536, text vertices 0..131072.",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxConvexSupportVertices", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxCompoundChildren", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxCompoundDepth", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeQueuedJobs", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeCompletedJobs", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeUploadsPerFrame", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeCacheEntries", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxShapeCacheBytes", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxBodyInstances", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugMaxLineVertices", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "DebugOverlay", "DebugUseBoundsForHeavyConvex", "15. Debug Overlay", "",
            api::configuration::Group::Developer },
        { "LogsAndTelemetry", "DebugVerboseLogging", "16. Logs and Telemetry",
            "Very noisy per-frame diagnostic logging. Use only while investigating a specific issue.",
            api::configuration::Group::Developer },
        { "LogsAndTelemetry", "DebugTestbenchTelemetry", "16. Logs and Telemetry",
            "Publishes per-frame hand, head and controller state and ROCK events to shared memory for the external "
            "test bench. Leave off for play.",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugShowRootFlattenedFingerSkeletonMarkers", "17. Skeleton Debug",
            "Draws markers and axes for live first-person skeleton bones.", api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugShowSkeletonBoneVisualizer", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneMode", "17. Skeleton Debug",
            "0 = minimal/core bones, 1 = core body and fingers, higher modes may draw more if supported.",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneSource", "17. Skeleton Debug",
            "Selects the skeleton source used by the debug visualizer.", api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugMaxSkeletonBonesDrawn", "17. Skeleton Debug",
            "Draw limits and filters for skeleton debug drawing.", api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugDrawSkeletonBoneAxes", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugMaxSkeletonBoneAxesDrawn", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonAxisBoneFilter", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugLogSkeletonBones", "17. Skeleton Debug",
            "Logs bone names and truncation information to ROCK.log.", api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugLogSkeletonBoneTruncation", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneLogIntervalFrames", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneLogFilter", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugRootFlattenedFingerSkeletonMarkerSize", "17. Skeleton Debug",
            "Marker and axis sizes for skeleton debug visuals.", api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBonePointSize", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneAxisLength", "17. Skeleton Debug", "",
            api::configuration::Group::Developer },
        { "PhysicsTiming", "HavokTimingFixEnabled", "18. Physics Timing",
            "Stabilizes FO4VR physics timing when rendering falls below the reference rate.",
            api::configuration::Group::Developer },
        { "PhysicsTiming", "VatsPhysicsFixes", "18. Physics Timing",
            "VATS Physics Fixes\nPreserves equipped-weapon handling speed during VATS and other game slow "
            "motion.\nCompensates the weapon drive and constraint while keeping dynamic collision.\nChanges apply at "
            "the next physics step; false restores uncompensated handling.",
            api::configuration::Group::Developer },
        { "PhysicsTiming", "HavokTimingFixMinPhysicsFrameRate", "18. Physics Timing", "",
            api::configuration::Group::Developer },
        { "PhysicsTiming", "HavokTimingFixMaxSubsteps", "18. Physics Timing", "",
            api::configuration::Group::Developer },
        { "ScopePlacement", "NativeScopeFiringGripFallbackOffsetGameUnits", "19. Scope Placement",
            "Scope Fallback Anchor\nWhen a weapon has no usable generated Scope or Sight geometry, the scope anchors "
            "at the firing grip, moved by these values.\nOffsets are game units (70 = 1 metre) in the weapon's axes: "
            "X sideways, +Y toward the muzzle, Z up; all zero puts the anchor on the firing grip.\nRotations are "
            "degrees added after the game's camera calibration: pitch about X, yaw about Z, roll about the muzzle "
            "axis.\nPosition moves the activation point, camera and overlay together; rotation corrects the camera "
            "and overlay only.\nThis key: offset [x, y, z]. Each value: -100 to 100.",
            api::configuration::Group::Developer },
        { "ScopePlacement", "NativeScopeFiringGripFallbackRotationDegrees", "19. Scope Placement",
            "Scope fallback rotation [pitch, yaw, roll], in degrees. Each value: -180 to 180.",
            api::configuration::Group::Developer },
        { "ScopePlacement", "NativeScopeOverlayOffsetGameUnits", "19. Scope Placement",
            "Scope Overlay\nFine-tunes only the game's own world_scope.nif overlay. The activation camera stays on "
            "the resolved sight or firing-grip anchor, so these values do not change when the scope activates.\nAll "
            "zero keeps automatic placement with the game's own model orientation and scale.\nOffsets are game units "
            "in the calibrated model frame; rotations are degrees: pitch about X, yaw about Z, roll about the optical "
            "Y axis.\nTune one axis at a time; replacement scope meshes can use a reversed sign.\nThis key: offset "
            "[x, y, z]. Each value: -100 to 100.",
            api::configuration::Group::Developer },
        { "ScopePlacement", "NativeScopeOverlayRotationDegrees", "19. Scope Placement",
            "Scope overlay rotation [pitch, yaw, roll], in degrees. Each value: -180 to 180.",
            api::configuration::Group::Developer },
    };
}
