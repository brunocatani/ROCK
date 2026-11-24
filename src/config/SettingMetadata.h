#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "physics-interaction/weapon/grip/GripZoneIndicatorPolicy.h"
#include "physics-interaction/weapon/native/ShellCasingGracePolicy.h"

namespace rock::config
{
    // The file that holds an option: ROCK.toml for players, ROCK_Developer.toml for developer overrides.
    enum class Group : std::uint8_t
    {
        Player,
        Developer,
    };

    // A number's range, or each part's of a transform.
    struct SettingRange
    {
        double minimum = 0.0;
        double maximum = 0.0;
    };

    struct SettingRanges
    {
        // What the loader clamps the option to (the readClamped* helpers).
        std::optional<SettingRange> clamp{};
        // What a slider on Mods Config's Developer page covers, for an option the loader limits in another way or
        // not at all, or whose clamp is too wide to slide over. Never past what the loader accepts.
        std::optional<SettingRange> slider{};
    };

    constexpr SettingRanges clampTo(double minimum, double maximum, std::optional<SettingRange> slider = {})
    {
        return { SettingRange{ minimum, maximum }, slider };
    }

    constexpr SettingRanges slideOver(double minimum, double maximum) { return { {}, SettingRange{ minimum, maximum } }; }

    struct SettingMetadata
    {
        std::string_view table, key, category, description;
        Group group;
        SettingRanges ranges{};
    };

    // TOML tables, file order, category headings and help for the settings files. Setting types and
    // defaults are declared by the compiled loader. A choice's help lists "N = label" pairs, then a
    // "label: line" explaining each.
    inline constexpr SettingMetadata kSettingMetadata[] = {
        { "Grabbing", "GrabDirection", "01. Grabbing",
            "Grab Direction\n1 = Palm Down (HIGGS Style). 2 = Fingers (Alyx Style).\nPalm Down (HIGGS Style): your "
            "hand reaches out of the palm, for objects near and far.\nFingers (Alyx Style): your hand reaches along "
            "your fingers, for objects near and far.\nFar grabs also need the object in your view. Applies at once, "
            "on each hand.",
            Group::Player },
        { "Grabbing", "FarGrabStyle", "01. Grabbing",
            "Far Grab Style\n1 = Immediate. 2 = Pull.\nImmediate: grabbing a far object brings it to your hand at "
            "once, with no flick.\nPull: hold grab to lock on, then flick your hand toward yourself; release before "
            "the flick to cancel.\nA pulled object follows your hand while you move or turn. Keep holding grab to "
            "catch it as it arrives, or let go and press grab again as it reaches you. Applies to the next far grab.",
            Group::Player },
        { "SelectionFeedback", "HighlightEnabled", "02. Grab Highlight",
            "Grab Highlight\nOutlines the object your hand would grab.", Group::Player },
        { "SelectionFeedback", "HighlightIntensityMode", "02. Grab Highlight",
            "Grab Highlight Strength\n1 = 40%. 2 = 60%. 3 = 80%. 4 = 100%.", Group::Player },
        { "SelectionFeedback", "HighlightColor", "02. Grab Highlight", "Grab Highlight Color\nred, blue, orange or white.",
            Group::Player },
        { "Prompts", "QuickLootPanelPlacement", "03. Prompts and Quick Loot",
            "Quick Loot Panel Placement\n1 = Right Hand. 2 = Closer Hand. 3 = At Container.\nRight Hand: the game's "
            "own panel on the right hand; it lists only what the right hand points at.\nCloser Hand: the panel rides "
            "the hand pointing at the container, mirrored on the left hand; when both hands point at containers the "
            "closer one takes it.\nAt Container: the panel stands upright at the container it lists, turned toward "
            "you, at a fixed size; either hand can point at it.\nThe button hints go with the panel; the buttons "
            "themselves stay on the right controller in every mode. Applies at once.",
            Group::Player },
        { "Prompts", "ActivatePromptPlacement", "03. Prompts and Quick Loot",
            "Activate Prompt Placement\n1 = Both Hands. 2 = Closer Hand. 3 = At Object.\nThe prompts each hand shows "
            "for what it points at (Open, Talk, Take and the rest).\nBoth Hands: the game's own prompts on each hand; "
            "two hands on one door show it twice.\nCloser Hand: when both hands point at the same object only the "
            "closer hand shows its prompt.\nAt Object: the prompt stands upright at the object it names, turned "
            "toward you, at a fixed size, and shows once when both hands point at it.\nActivation itself is "
            "unchanged. Applies at once.",
            Group::Player },
        { "WeaponHandling", "GunstockMode", "04. Weapon Handling",
            "Gunstock Mode\nTwo-handed firearms aim along the line from your firing hand to your support hand, like "
            "a rifle with a stock.\nHand and finger poses stay as authored; the support hand may sit slightly away "
            "from its controller.\nMelee weapons, weapon parts you operate and bipod-latched weapons are not "
            "affected.\nFalse: normal two-hand aiming.",
            Group::Player },
        { "WeaponHandling", "FrikUserWeaponOffsets", "04. Weapon Handling",
            "FRIK Weapon Offsets\nUse FRIK's weapon offsets, including their weapon scale, instead of ROCK's grip: "
            "the ones you saved in FRIK's weapon adjustment, and FRIK's built-in ones while FRIK Built-in Offsets "
            "is on.\nWeapons with an offset skip PAPER's reload and cycle animations.\nNeeds FRIK 0.80.2 or newer.",
            Group::Player },
        { "WeaponHandling", "FrikBuiltInWeaponOffsets", "04. Weapon Handling",
            "FRIK Built-in Offsets\nWith FRIK Weapon Offsets on, also use the weapon offsets FRIK ships with for "
            "weapons you have not adjusted.\nFalse: only the offsets you saved in FRIK.",
            Group::Player },
        { "WeaponHandling", "BipodMode", "04. Weapon Handling",
            "Bipod Mode\nClick the right stick to latch any equipped weapon part to the surface it touches.\nWhile "
            "latched, recoil is a fixed 10% whatever your hands or armor.\nFalse: the click is left to other mods.",
            Group::Player },
        { "WeaponHandling", "BipodHandsFreeRadiusMeters", "04. Weapon Handling",
            "Bipod Hands-Free Distance\nA latched weapon stays equipped with no hand on it while your headset is "
            "within this distance of the latch point, in metres.\nFarther away it is left in place as a world object "
            "(it does not fall). Must be above 0.",
            Group::Player },
        { "WeaponHandling", "DetachEitherHand", "04. Weapon Handling",
            "Detach Either Hand\nEither hand may let go of the weapon, including the hand on the firing grip; the "
            "other hand keeps holding it.\nGrips release as Weapon Grab Mode sets. Parts you only attach to release "
            "when you let go of the button.\nWhat happens when the last hand lets go is set by Weapon Release Mode.",
            Group::Player },
        { "WeaponHandling", "AutoEquip", "04. Weapon Handling",
            "Auto Equip\nHow a loose weapon you hold gets equipped.\nTrue: once your palm settles on its firing grip "
            "(see Auto Equip Settle Time), no trigger needed (default).\nFalse: pull the trigger while holding it to "
            "equip it in that hand.\nFar grabs seat a loose weapon at its firing grip and close grabs at its nearest "
            "grip; touch grabs equip only inside a grip zone.\nWith a weapon already in the other hand, the new one "
            "becomes the second weapon.\nWith Auto Equip on, letting go always drops the weapon: kept in your hand, "
            "it would equip again at once.",
            Group::Player },
        { "WeaponHandling", "WeaponReleaseMode", "04. Weapon Handling",
            "Weapon Release Mode\nWhat happens when the last hand lets go of an equipped weapon.\n1 = Stays "
            "equipped. 2 = Stays loose in hand. 3 = Drops.\nStays equipped: it stays equipped and held, as in the "
            "game.\nStays loose in hand: press grab again and release to drop or stash it, or pull the trigger to "
            "equip it again.\nDrops: it drops where it is, leaving with your hand's motion (default).\nApplies only "
            "with Auto Equip off; Auto Equip always drops.\nApplies per weapon. Weapon Grab Mode sets the release "
            "gesture.",
            Group::Player },
        { "WeaponHandling", "WeaponGrabMode", "04. Weapon Handling",
            "Weapon Grab Mode\n1 = Toggle both grips. 2 = Toggle firing grip only. 3 = Hold both grips.\nToggle: "
            "press grab to take the grip and press again to let go. Hold: the grip lasts while you hold grab.\nMode "
            "2 toggles whichever hand holds the firing grip; support grips must be held.\nParts you only attach to "
            "always need grab held. Ordinary objects and surfaces always use hold to grab.\nWeapon Release Mode "
            "decides whether the last release keeps a loose grab or drops the weapon.",
            Group::Player },
        { "WeaponHandling", "AutoEquipSettleSeconds", "04. Weapon Handling",
            "Auto Equip Settle Time\nSeconds your palm must stay on the firing grip before Auto Equip equips. "
            "Range: 0 to 2.",
            Group::Player, clampTo(0.0, 2.0) },
        { "WeaponHandling", "GrabAnywhereOnWeapon", "04. Weapon Handling",
            "Grab Anywhere on Weapon\nTrue: your support hand can hold a firearm anywhere.\nFalse: firearms with "
            "authored support grips take the support hand only there (default).\nMelee weapons, and firearms without "
            "an authored support grip, can always be held anywhere.\nApplies to either support hand.",
            Group::Player },
        { "WeaponHandling", "MeleeGripPitchDegrees", "04. Weapon Handling",
            "Melee Grip Angle\nTilts a held melee weapon and your hand together around the grip, in degrees, on top "
            "of ROCK's built-in 30 degree forward tilt.\nPositive tilts further forward, negative tilts back; -30 "
            "gives the weapon's authored angle. Range: -180 to 180.\nRegrab a loose weapon to see the change.",
            Group::Player, clampTo(-180.0, 180.0) },
        { "WeaponHandling", "WeaponFilteringPreset", "04. Weapon Handling",
            "Shake Filtering\n0 = Off. 1 = Responsive. 2 = Balanced. 3 = Steady. 4 = Heavy.\nSmooths controller "
            "shake on equipped firearms, loose weapons and held objects. Higher steadies more but responds more "
            "slowly.\nRotation and position smoothing: Responsive 12 and 6 ms, Balanced 25 and 12 ms, Steady 45 and "
            "22 ms, Heavy 70 and 35 ms.\nFree hands, FRIK and recoil are not affected. Applies at once.",
            Group::Player },
        { "WeaponHandling", "ScopedFilterRotationSeconds", "04. Weapon Handling",
            "Scoped Filtering: Rotation\nExtra rotation smoothing, in seconds, while you look through a scope, with "
            "or without Shake Filtering.\n0 here and in Scoped Filtering: Position turns scoped filtering off. "
            "Range: 0 to 1.",
            Group::Player, clampTo(0.0, 1.0) },
        { "WeaponHandling", "ScopedFilterTranslationSeconds", "04. Weapon Handling",
            "Scoped Filtering: Position\nExtra position smoothing, in seconds, while you look through a scope. "
            "Range: 0 to 1.",
            Group::Player, clampTo(0.0, 1.0) },
        { "WeaponHandling", "ScopedFilterReleaseDegreesPerSecond", "04. Weapon Handling",
            "Scoped Filtering: Release Speed\nHow fast you turn the controller, in degrees per second, before the "
            "scoped smoothing drops to half.\nHolding still gets full smoothing; swinging to a new target loosens "
            "it.\n0 keeps full smoothing at any speed. Range: 0 to 720.",
            Group::Player, clampTo(0.0, 720.0) },
        { "WeaponHandling", "FiringGripReattachRadiusGameUnits", "04. Weapon Handling",
            "Firing Grip Reach\nHow far from the firing grip a hand can take it over or grab it again, in game units "
            "(70 = 1 metre).\nThe zone is a quarter circle on each side of the grip, from straight out to straight "
            "down.\nPress grab inside it to move the firing grip to that hand; the firing hand can also pass it with "
            "a grab tap.\nPointing at it in Mods Config outlines it on the drawn weapon. Range: 0.25 to 30.",
            Group::Player, clampTo(0.25, 30.0) },
        { "WeaponHandling", "GripZoneIndicatorDiameterGameUnits", "04. Weapon Handling",
            "Grip Zone Marker Size\nDiameter of the sphere that marks a weapon's grip zones, in game units (70 = 1 "
            "metre).\nPointing at it in Mods Config shows every marker on the drawn weapon. Range: 0.1 to 5.",
            Group::Player, clampTo(grip_zone_indicator_policy::kMinimumDiameterGameUnits, grip_zone_indicator_policy::kMaximumDiameterGameUnits) },
        { "WeaponHandling", "AmmoCounterStyle", "04. Weapon Handling",
            "Ammo Counter Style\n1 = Minimal. 2 = Immersive. 3 = Vanilla.\nMinimal: white loaded and reserve rounds "
            "beside the firing grip on the palm side, shown while you look at them.\nImmersive: the same place and "
            "look to show, in the game's HUD color and bracket frame, loaded over reserve.\nVanilla: the game's "
            "counter on the back of the hand holding the weapon, in HUD color, always shown (mirrored on the left "
            "hand).\nEvery style replaces the game's counter with one counter per drawn weapon. The counters are "
            "drawn through RPS UI Framework.",
            Group::Player },
        { "WeaponRecoil", "ImmersiveRecoil", "05. Weapon Recoil",
            "Immersive Recoil\nUses the recoil strengths below, for either hand and in power armor.\nFalse: the "
            "game's own recoil strength, on the hand that fires.\nThe developer setting Emergency Disable Recoil "
            "overrides both.",
            Group::Player },
        { "WeaponRecoil", "LaserRecoilPercent", "05. Weapon Recoil",
            "Laser Recoil\nRecoil for laser weapons in any hold and in power armor: 100 = 10% of the game's "
            "movement and rotation, 0 = none. Range: 0 to 300.\nA bipod-latched weapon keeps its fixed 10%.",
            Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "PistolOneHandRecoilPercent", "05. Weapon Recoil",
            "Recoil Strength by Weapon Type\nPercent of each weapon type's recoil: 0 = none, 100 = full, 300 = "
            "triple. Range: 0 to 300.\nOne Hand applies without a support grip, Two Hand with one.\nBipod latching "
            "and power armor use their own fixed recoil.\nThis key: pistols held with one hand.",
            Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "PistolTwoHandRecoilPercent", "05. Weapon Recoil", "Pistols held with two hands. Range: 0 to 300.",
            Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "RifleOneHandRecoilPercent", "05. Weapon Recoil", "Rifles held with one hand. Range: 0 to 300.",
            Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "RifleTwoHandRecoilPercent", "05. Weapon Recoil", "Rifles held with two hands. Range: 0 to 300.",
            Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "ShotgunOneHandRecoilPercent", "05. Weapon Recoil",
            "Shotguns held with one hand. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "ShotgunTwoHandRecoilPercent", "05. Weapon Recoil",
            "Shotguns held with two hands. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "HeavyOneHandRecoilPercent", "05. Weapon Recoil",
            "Heavy weapons held with one hand. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "HeavyTwoHandRecoilPercent", "05. Weapon Recoil",
            "Heavy weapons held with two hands. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "DefaultOneHandRecoilPercent", "05. Weapon Recoil",
            "Any other firearm held with one hand. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "WeaponRecoil", "DefaultTwoHandRecoilPercent", "05. Weapon Recoil",
            "Any other firearm held with two hands. Range: 0 to 300.", Group::Player, clampTo(0.0, 300.0) },
        { "FiringHand", "DefaultHand", "06. Firing Hand",
            "Default Hand\nright or left: the firing hand for equips that come without a hand (favorites, PALM, "
            "scripts and other mods).\nGrenade draws prefer this hand and use the other one when it is busy.\nPip-Boy "
            "and trigger equips and grip handoffs always use the hand that acts.\nApplies on the next equip or draw. "
            "The game's own left-handed mode is not supported and stays off.",
            Group::Player },
        { "FiringHand.Left", "AimYawDegrees", "06. Firing Hand",
            "Left Firing Aim: Yaw\nTurns the aim of a weapon fired from the left hand, in degrees. 0 uses the plain "
            "mirror of the right-hand hold. Range: -30 to 30.",
            Group::Player, clampTo(-30.0, 30.0) },
        { "FiringHand.Left", "AimPitchDegrees", "06. Firing Hand",
            "Left Firing Aim: Pitch\nTilts the aim of a weapon fired from the left hand up or down, in degrees. "
            "Range: -30 to 30.",
            Group::Player, clampTo(-30.0, 30.0) },
        { "FiringHand.Left", "AimOffset", "06. Firing Hand",
            "Left Firing Aim: Offset\nShifts the aim of a weapon fired from the left hand, in game units (70 = 1 "
            "metre), as { pos = [along the fingers, out of the palm, across the palm] }.\nThe weapon moves that way, "
            "at once. Each value: -15 to 15.",
            Group::Player, clampTo(-15.0, 15.0) },
        { "FiringHand.Left", "GripOffset", "06. Firing Hand",
            "Left Firing Grip\nMoves where the left hand holds the firing grip, in game units (70 = 1 metre), as "
            "{ pos = [along the fingers, into or out of the palm, across the palm] }.\nAll 0 = the plain mirror of "
            "the right-hand grip. The hand stays put and the weapon moves the other way. Each value: -15 to 15. "
            "Regrab to apply.",
            Group::Player, clampTo(-15.0, 15.0) },
        { "FiringHand.Right", "SupportGripOffset", "06. Firing Hand",
            "Right Support Grip\nMoves where the right hand holds a support grip when the left hand fires, in game "
            "units (70 = 1 metre), as { pos = [along the fingers, into or out of the palm, across the palm] }.\nAll "
            "0 = the plain mirror of the left-hand support grip. Each value: -15 to 15. Regrab to apply.",
            Group::Player, clampTo(-15.0, 15.0) },
        { "Scopes", "EnableImmersiveScopes", "07. Scopes",
            "Immersive Scopes\nROCK places the scope view on the weapon, raises it while you hold the scope button "
            "(right A by default), and corrects the overlay and zoom.\nFalse: the game's own scopes.\nStarting with "
            "this off leaves the game's scope hook free for other scope mods. Turning it on takes the hook until you "
            "restart; if another mod owns it, ROCK's scopes stay off.\nRelease the scope button before changing "
            "this. Reload stays on a tap of the same button.",
            Group::Player },
        { "Grenades", "ImmersiveGrenades", "08. Grenades",
            "Immersive Grenades\nHold the grenade button (right B by default) to draw a grenade into your hand and "
            "throw it by hand.\nSelecting a grenade in PALM also puts it in your hand.\nFalse: the game's own hold "
            "and release throw.\nRelease the grenade button before changing this; a grenade already drawn keeps its "
            "mode.",
            Group::Player },
        { "Grenades", "RealisticGrenadeFuseSeconds", "08. Grenades",
            "Grenade Fuse\nSeconds from pulling the pin to detonation for grenades drawn by hand.\n0 uses the "
            "grenade's own fuse time. Range: 0 to 30.",
            Group::Player, clampTo(0.0, 30.0) },
        { "ImmersiveAid", "Enabled", "09. Immersive Aid",
            "Immersive Aid\nStimpaks and Med-X sit in a fixed syringe grip and inject after touching your body for "
            "0.5 seconds.\nFalse: they are grabbed like any object and used by bringing them to your mouth.\nRelease "
            "and grab again to see the change on an item already in your hand.",
            Group::Player },
        { "ControllerInput", "SuppressNativeVats", "10. Controller Input",
            "Block VATS\nStops the game's VATS from opening on the VATS button (right B by default).\nThe button "
            "still works for ROCK and other mods.",
            Group::Player },
        { "ControllerInput", "OfferSteamVRBinding", "10. Controller Input",
            "Offer ROCK's SteamVR Binding\nTrue: when Index controllers or Vive wands use SteamVR's default binding "
            "for Fallout 4 VR, the setup wizard opens by itself once per game start, when no menu or scene is in "
            "the way, and offers ROCK's binding (the default "
            "binding turns A and X into grip). It never replaces a binding you chose in SteamVR.\nFalse: the wizard "
            "opens only when you open it; it still offers the binding then. Choosing other controllers in the "
            "wizard sets this.",
            Group::Player },
        { "ControllerInput.Right", "Trigger", "10. Controller Input",
            "Button Layout\nThe physical input that acts as each ROCK button, per hand. ROCK, PALM, PAPER, SCISSORS "
            "Redux, FRIK and the game all follow it.\nThe defaults are the Quest and Touch layout and change "
            "nothing.\nValues: trigger, grip, a, b (x, y on the left), thumbstick, grip_click, axis3, axis4, pad_up, "
            "pad_down, pad_left, pad_right, pad_center (trackpad click regions), none.\nSwapping two buttons moves "
            "every action of each. Stick movement is never remapped.\nControllers whose SteamVR binding merges "
            "buttons (Index A reports as grip) need the binding fixed in SteamVR first; ROCK only sees what SteamVR "
            "sends.\nThis key: the right trigger.",
            Group::Player },
        { "ControllerInput.Right", "Grab", "10. Controller Input", "The right grab button (grip by default).",
            Group::Player },
        { "ControllerInput.Right", "A", "10. Controller Input", "Right A: reload tap, scope hold.",
            Group::Player },
        { "ControllerInput.Right", "B", "10. Controller Input", "Right B: grenade draw hold, VATS.",
            Group::Player },
        { "ControllerInput.Right", "StickClick", "10. Controller Input",
            "Right stick click: bipod latch; opens PALM by default.", Group::Player },
        { "ControllerInput.Left", "Trigger", "10. Controller Input", "The left trigger.",
            Group::Player },
        { "ControllerInput.Left", "Grab", "10. Controller Input", "The left grab button (grip by default).",
            Group::Player },
        { "ControllerInput.Left", "X", "10. Controller Input",
            "Left X: reload tap and scope hold for a left firing hand.", Group::Player },
        { "ControllerInput.Left", "Y", "10. Controller Input", "Left Y: Pip-Boy tap, Pause hold.",
            Group::Player },
        { "ControllerInput.Left", "StickClick", "10. Controller Input", "Left stick click.",
            Group::Player },
        { "SetupWizard", "ShownEdition", "10. Controller Input",
            "Setup Wizard Shown\nThe setup wizard opens by itself once, 10 seconds after you are in the game with no "
            "menu or scene in the way, whenever this ROCK brings a newer edition of it (new pages or options to "
            "show). On a new game it waits until you leave the mirror. ROCK writes the edition here when the wizard "
            "opens.\n0 = show it at the next game start. 999 = never open it by itself.",
            Group::Player },
        { "Bindings", "Grab", "10. Controller Input",
            "Bindings\nEach function's control, as { hand = \"primary\", type = \"tap\", button = \"a\" }, or "
            "{ type = \"none\" }.\nhand: primary, offhand, left, right, any. Fire, Reload, Scope and SupportTrigger "
            "take only primary (the hand holding the weapon's trigger) and offhand; elsewhere primary is the right "
            "hand.\ntype: hold (acts while held; with duration = seconds, from that time until release, 0.05 to 2), "
            "tap, double (duration = the longest gap, 0.15 to 0.6, 0.4 when left out).\nbutton: trigger, grip, a, b, "
            "thumbstick. Other controls reach these through the button layout above.\nAdd modifier = { hand = "
            "\"offhand\", button = \"grip\" } for a button held at the same time (the binding's hand when left "
            "out).\nThis key: grab with each hand.",
            Group::Player },
        { "Bindings", "Fire", "10. Controller Input", "Fire the equipped weapon.", Group::Player },
        { "Bindings", "SupportTrigger", "10. Controller Input",
            "The support hand's trigger: a second weapon, or the support action of what that hand holds.",
            Group::Player },
        { "Bindings", "Use", "10. Controller Input",
            "Use what the primary hand points at (take, open, talk). It owns the whole press while there is something "
            "to use.", Group::Player },
        { "Bindings", "Reload", "10. Controller Input", "Reload the equipped weapon.",
            Group::Player },
        { "Bindings", "Scope", "10. Controller Input",
            "Look through the weapon's scope while active. On its home button (A) the duration is also how long a "
            "press lasts before it scopes instead of reloading.",
            Group::Player },
        { "Bindings", "Vats", "10. Controller Input", "Open VATS.", Group::Player },
        { "Bindings", "Grenade", "10. Controller Input",
            "Draw a grenade into the hand. On its home button (B) the duration is also how long a press lasts before "
            "it draws a grenade instead of opening VATS.",
            Group::Player },
        { "Bindings", "Pipboy", "10. Controller Input", "Open the Pip-Boy.", Group::Player },
        { "Bindings", "Pause", "10. Controller Input",
            "Open the Pause menu. On its home button (left Y) the duration is also how long a press lasts before it "
            "opens Pause instead of the Pip-Boy.",
            Group::Player },
        { "Bindings", "StickActions", "10. Controller Input",
            "Bipod latch and held placement while the weapon touches a surface, and the game's Favorites menu.",
            Group::Player },
        { "Bindings", "Palm", "10. Controller Input", "Open RobCo PALM's wheel.", Group::Player },
        { "NativeGameplay", "VanillaMeleeSwingSpeed", "11. Native Gameplay",
            "Melee Swing Speed\nHand speed that starts a vanilla melee swing, in game units per second (70 = 1 metre "
            "per second). The game uses 200.\nTurning the wrist alone never swings. Range: 50 to 2000.",
            Group::Player, clampTo(50.0, 2000.0) },
        { "NativeGameplay", "VanillaGunBashSpeed", "11. Native Gameplay",
            "Gun Bash Speed\nHand speed that starts a vanilla gun bash, in game units per second (70 = 1 metre per "
            "second). The game uses 200.\nTurning the wrist alone never bashes. Range: 50 to 2000.",
            Group::Player, clampTo(50.0, 2000.0) },
        { "Logging", "LogLevel", "12. Logging",
            "Log Level\nHow much ROCK writes to ROCK.log.\n0 = Trace. 1 = Debug. 2 = Info. 3 = Warn. 4 = Error. "
            "5 = Critical. 6 = Off.\nTrace and Debug write a lot and can cost frame time; keep Info for normal play.",
            Group::Player },
        { "Developer", "ShowDeveloperPages", "13. Developer",
            "Show Developer Pages\nShows the Developer page on ROCK's page in Mods Config FO4 VR, which edits "
            "ROCK_Developer.toml.\nOnly Mods Config reads this option.",
            Group::Player },
        { "DeveloperControls", "DeveloperModeEnabled", "01. Developer Controls",
            "Gates advanced/tuning-only features not meant for normal play. Currently:\npressing A while holding an "
            "object records/overwrites a saved grab offset\nfor that object+hand. Saved offsets keep applying on "
            "future grabs\nregardless of this setting - it only gates recording new ones.",
            Group::Developer },
        { "DeveloperControls", "EmergencyDisableRecoil", "01. Developer Controls",
            "Emergency Disable Recoil\nON: disables native VR recoil and kickback plus ROCK weapon/hand recoil, "
            "overriding every recoil profile.\nOFF: restores native recoil/kickback and the existing Immersive Recoil "
            "setting.\nApplies immediately, independently of Developer Mode. Does not change projectile spread.",
            Group::Developer },
        { "LoggingAndProfiling", "LogPattern", "02. Logging and Profiling", "Text format for each ROCK.log line.",
            Group::Developer },
        { "LoggingAndProfiling", "LogSampleMilliseconds", "02. Logging and Profiling",
            "Minimum time between repeated sampled diagnostic messages.", Group::Developer, slideOver(250.0, 60000.0) },
        { "LoggingAndProfiling", "LogFlushImmediate", "02. Logging and Profiling",
            "Writes each ROCK.log line to disk as it is logged.\nOff, the log is block buffered, so a freeze or a "
            "crash loses the last lines\nand every log ends mid-line. On, the tail survives, at one file write per "
            "line.\nLeave off unless you are chasing a hang.",
            Group::Developer },
        { "LoggingAndProfiling", "WeaponVisibilityTrace", "02. Logging and Profiling",
            "Weapon visibility trace.\nWrites a VISIBILITY line to ROCK.log whenever the reason an equipped weapon "
            "would not render changes\n(a hidden node and who hid it, lost geometry, a stale bound, the weapon away "
            "from its hand, the scope),\nand a VISIBILITY tick once per second with the full state, the cell and the "
            "position.\nIndependent from LogLevel. Walks the equipped weapons each frame; leave off for normal play.",
            Group::Developer },
        { "LoggingAndProfiling", "PerformanceProfilerEnabled", "02. Logging and Profiling",
            "Asynchronous performance profiler.\nIndependent from LogLevel. When enabled, timing scopes stay on the "
            "frame path,\nbut completed profiler windows are queued to a bounded background writer.\nOutput goes to "
            "ROCK_Profiler.log, not ROCK.log. File formatting/writes are not done\nby grab, reload, physics, or frame "
            "update code. If the writer falls behind, profiler\nwindows are dropped instead of stalling gameplay.",
            Group::Developer },
        { "LoggingAndProfiling", "PerformanceProfilerLogIntervalFrames", "02. Logging and Profiling",
            "Number of frames per queued profiler window after warmup. Larger values reduce file traffic.",
            Group::Developer, slideOver(30.0, 6000.0) },
        { "LoggingAndProfiling", "PerformanceProfilerWarmupFrames", "02. Logging and Profiling",
            "Startup/reload frames ignored before profiler windows are emitted.",
            Group::Developer, slideOver(0.0, 6000.0) },
        { "PalmAndGrabOffsets", "RightGrabAuthorityProxyOffset", "03. Palm and Grab Offsets",
            "Right-hand local position for the hidden grab authority proxy/seat point, in corrected palm "
            "proxy space.\nX follows the fingers, -Y moves into the palm face/skin, +Y moves to the back of the "
            "hand,\nand Z moves across the palm. This does not move the real palm/finger physics colliders.",
            Group::Developer, slideOver(-20.0, 20.0) },
        { "PalmAndGrabOffsets", "LeftGrabAuthorityProxyOffset", "03. Palm and Grab Offsets",
            "Left-hand local position for the hidden grab authority proxy/seat point, in the same axes.",
            Group::Developer, slideOver(-20.0, 20.0) },
        { "WeaponCollisionAndSupport", "WeaponCollisionBlocksProjectiles", "04. Weapon Collision and Support",
            "Controls whether ROCK weapon hulls collide with projectile/spell layers.\nFalse avoids self-collisions "
            "with modded projectile systems while aiming.",
            Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionBlocksSpells", "04. Weapon Collision and Support", "",
            Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponShellCollisionGraceMs", "04. Weapon Collision and Support",
            "New casings ignore the firing weapon's generated colliders for this many simulation "
            "milliseconds.\nPositive values protect at least one physics step. Zero disables the grace period. Range: "
            "0-500 ms.\nWorld contacts and native casing lifetime are preserved. Changes apply to new casings.",
            Group::Developer, clampTo(0.0, shell_casing_grace::kMaximumMilliseconds) },
        { "WeaponCollisionAndSupport", "WeaponCollisionPreserveGaps", "04. Weapon Collision and Support",
            "False: original convex fitting. True: bounded gap-preserving compounds.\nChanging this key rebuilds "
            "weapon colliders after a live settings reload.\nCompare with PerformanceProfilerEnabled: "
            "weaponColliderBuild/Create vs weaponGapColliderBuild/Create.",
            Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponCollisionVisualStabilizationSeconds", "04. Weapon Collision and Support",
            "Seconds ROCK waits for the visual weapon tree to stay visible/stable before replacing generated hulls "
            "after weapon/mod swaps.",
            Group::Developer, slideOver(0.0, 0.66) },
        { "WeaponCollisionAndSupport", "WeaponCollisionMaxLinearVelocity", "04. Weapon Collision and Support",
            "Maximum speed allowed when ROCK keyframes generated weapon hulls.", Group::Developer, slideOver(50.0, 2000.0) },
        { "WeaponCollisionAndSupport", "WeaponCollisionMaxAngularVelocity", "04. Weapon Collision and Support", "",
            Group::Developer, slideOver(50.0, 2000.0) },
        { "WeaponCollisionAndSupport", "WeaponCollisionGripRecoveryDistanceGameUnits", "04. Weapon Collision and Support",
            "Immediately resets a failed dynamic weapon collider onto the firing hand if\nits live firing-grip "
            "authority point is farther away than this distance.\nThis catastrophic-failure limit is separate from "
            "normal wall-contact\nconvergence. 210 game units is about 3 metres. Game units (~1.4cm each).",
            Group::Developer, clampTo(35.0, 700.0) },
        { "WeaponCollisionAndSupport", "WeaponInteractionTouchRadius", "04. Weapon Collision and Support",
            "Small sphere around each hand's palm point. If it overlaps a\ngenerated weapon collider, the grab uses "
            "ROCK's unrestricted dynamic seat.",
            Group::Developer, clampTo(0.25, 6.0) },
        { "WeaponCollisionAndSupport", "WeaponInteractionProbeRadius", "04. Weapon Collision and Support",
            "Broader per-hand search used when the palm is not touching the weapon.\nWith authored weapon grips "
            "enabled, a grab acquired only by this probe uses\nthe modeler's authored firing/support seat instead of "
            "the dynamic mesh seat.",
            Group::Developer, slideOver(0.0, 50.0) },
        { "WeaponCollisionAndSupport", "DebugWeaponSupportDynamicOnly", "04. Weapon Collision and Support",
            "Testing only: authored support seats are never used, so every support grab\nruns the dynamic mesh grip. "
            "Use to test dynamic grips where an authored zone\nwould win. Leave off for play.",
            Group::Developer },
        { "WeaponCollisionAndSupport", "FiringGripProximitySupportRadius", "04. Weapon Collision and Support",
            "Every support grab inside this distance from the firing grip stays visual-only\nso pistols retain a "
            "stable support pose without competing weapon authority.\nROCK always owns this behavior. An active "
            "equipped-weapon addon may override\nonly the radius through the ROCK V1 API. Game units (~1.4cm each).",
            Group::Developer, clampTo(0.25, 30.0) },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpEnabled", "04. Weapon Collision and Support",
            "Eases ROCK's external FRIK hand target into an equipped weapon support grip.\nThis is visual-only: weapon "
            "aim authority remains immediate.",
            Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMin", "04. Weapon Collision and Support", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMax", "04. Weapon Collision and Support", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMinDistance", "04. Weapon Collision and Support", "",
            Group::Developer, clampTo(0.0, 80.0) },
        { "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMaxDistance", "04. Weapon Collision and Support", "",
            Group::Developer, clampTo(0.0, 120.0) },
        { "WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatEnabled", "04. Weapon Collision and Support",
            "Seats the visual support palm toward the contacted surface without rotating the weapon.",
            Group::Developer },
        { "WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatMaxDegrees", "04. Weapon Collision and Support", "",
            Group::Developer, clampTo(0.0, 75.0) },
        { "WeaponCollisionAndSupport", "FiringGripReattachCylinderRadiusGameUnits", "04. Weapon Collision and Support",
            "Thickness of the firing-grip reattach zone in game units (70 = 1 metre): how far above the grip point "
            "and how far forward or back along the barrel the palm may sit.\nA handling mod that selects the cylinder "
            "zone uses it as the cylinder radius. Range: 0.1 to 30.",
            Group::Developer, clampTo(0.1, 30.0) },
        { "HandAndBodyCollision", "HandCollisionStaticWorldEnabled", "05. Hand and Body Collision",
            "Allows generated hand colliders to collide with static/animstatic world geometry for native contact "
            "evidence.",
            Group::Developer },
        { "HandAndBodyCollision", "GlobalSurfaceGrabEnabled", "05. Hand and Body Collision",
            "Makes dynamic-hand world-surface contacts eligible for fixed-anchor grab fallback.",
            Group::Developer },
        { "HandAndBodyCollision", "SurfaceMeshGrabEnabled", "05. Hand and Body Collision",
            "Uses the contacted render mesh as the spatial authority for fixed-surface grabs.",
            Group::Developer },
        { "HandAndBodyCollision", "SurfaceMeshGrabMaxProjectionDistanceGameUnits", "05. Hand and Body Collision", "",
            Group::Developer, slideOver(1.0, 128.0) },
        { "HandAndBodyCollision", "SurfaceMeshGrabMaxTriangles", "05. Hand and Body Collision", "",
            Group::Developer, slideOver(256.0, 100000.0) },
        { "HandAndBodyCollision", "HandBoneColliderRadiusScaleOverrides", "05. Hand and Body Collision",
            "Optional generated hand capsule-like collider radius scale overrides. Palm-box roles are controlled by "
            "X/Y/Z below.\nRole names: ThumbBase, ThumbMiddle, ThumbTip, IndexBase, IndexMiddle, IndexTip, MiddleBase, "
            "MiddleMiddle,\nMiddleTip, RingBase, RingMiddle, RingTip, PinkyBase, PinkyMiddle, PinkyTip.\nFormat: "
            "Role=radiusScale;PowerArmor.Role=radiusScale",
            Group::Developer },
        { "HandAndBodyCollision", "HandPalmColliderDimensionScaleOverrides", "05. Hand and Body Collision",
            "Optional generated palm-box collider local dimension scale overrides.\nLocal X=fingers/segment length, "
            "local Y=palm depth, local Z=cross-palm width.\nRole names: PalmAnchor, PalmFace, PalmBack, PalmHeel, "
            "ThumbPad.\nFormat: Role=xScale,yScale,zScale;PowerArmor.Role=xScale,yScale,zScale",
            Group::Developer },
        { "HandAndBodyCollision", "HandBoneCollidersRequirePalmAnchor", "05. Hand and Body Collision",
            "Requires the palm anchor before creating hand bone colliders.", Group::Developer },
        { "HandAndBodyCollision", "HandBoneColliderMaxLinearVelocity", "05. Hand and Body Collision",
            "Speed clamps for bone-driven hand colliders.", Group::Developer, slideOver(50.0, 2000.0) },
        { "HandAndBodyCollision", "HandBoneColliderMaxAngularVelocity", "05. Hand and Body Collision", "",
            Group::Developer, slideOver(50.0, 2000.0) },
        { "SelectionAndPulling", "PullToObjectCenterEnabled", "06. Selection and Pulling",
            "Far grabs pull the visual mesh point nearest the mesh's centre to the hand\ninstead of the mesh point "
            "nearest the ray hit (a hit at selection range is aim\nnoise, not grip intent). The arrival seat then "
            "settles the mesh onto the palm.",
            Group::Developer },
        { "SelectionAndPulling", "PullLongAxisPresentationEnabled", "06. Selection and Pulling",
            "Long-object presentation during pull flight: rotates elongated props (brooms,\nbottles) so their mesh "
            "long axis lines up with the hand's thumb->pinky line\nbefore arrival. Flight-only; held objects are never "
            "rotated. Elongation is the\nRMS extent ratio between the mesh's dominant and second axis (compact "
            "objects\n~1 skip presentation). Gain is proportional rad/s per rad of error.",
            Group::Developer },
        { "SelectionAndPulling", "PullPresentationMinElongationRatio", "06. Selection and Pulling", "",
            Group::Developer, slideOver(1.0, 10.0) },
        { "SelectionAndPulling", "PullPresentationAngularGainPerSecond", "06. Selection and Pulling", "",
            Group::Developer, slideOver(0.0, 30.0) },
        { "SelectionAndPulling", "PullPresentationMaxAngularSpeedRadiansPerSecond", "06. Selection and Pulling", "",
            Group::Developer, slideOver(0.0, 40.0) },
        { "SelectionAndPulling", "PullPresentationGripAxisTiltDegrees", "06. Selection and Pulling",
            "Tilts the presentation target off pure cross-palm Z toward fingers-forward\n+X, in degrees (0..45): the "
            "thumb occupies the pure-Z line, so the natural\nlong-object hold sits a few degrees toward the "
            "fingertips. Applies to both\nthe pull-flight servo and the force-grab seat alignment.\nThis is an "
            "anatomical magnitude, not a signed rotation. Authored Z reverses\nits cross-palm direction between hands, "
            "but +X points toward the fingertips\non both. ROCK resolves the live axes first, then tilts each signed Z "
            "axis\ntoward its own +X, so one value has the same meaning for both hands.",
            Group::Developer, slideOver(0.0, 45.0) },
        { "SelectionAndPulling", "GrabSeatRollMinSecondElongationRatio", "06. Selection and Pulling",
            "Shape-classification gate shared by the flight presentation servo and the\ngrab pose solver: "
            "sqrt(lambda2/lambda3) of the mesh PCA at or above this\nratio marks a distinct flat face (plate class). "
            "The solver uses the class\nfor its rod-only terms; plates keep their edge/corner holds (31 "
            "verified\nplate holds put the face normal 63 deg off the palm normal - edge holds are\nthe intended grip, "
            "not an artifact).",
            Group::Developer, slideOver(1.0, 10.0) },
        { "SelectionAndPulling", "ObjectPhysicsTreeMaxDepth", "06. Selection and Pulling",
            "Maximum scene-node depth used when scanning an object for physics bodies.",
            Group::Developer, slideOver(1.0, 32.0) },
        { "GrabDrive", "GrabLinearTau", "07. Grab Drive",
            "Shared dynamic-grab response for held objects and equipped weapon colliders. Higher tau increases "
            "positional recovery. Damping calms oscillation; angular response remains independent.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "GrabDrive", "GrabLinearDamping", "07. Grab Drive", "", Group::Developer, slideOver(0.0, 2.0) },
        { "GrabDrive", "GrabLinearProportionalRecovery", "07. Grab Drive", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabDrive", "GrabLinearConstantRecovery", "07. Grab Drive", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabDrive", "GrabAngularTau", "07. Grab Drive",
            "Angular shared-constraint motor response. Same idea as linear motor, but for rotation.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "GrabDrive", "GrabAngularDamping", "07. Grab Drive", "", Group::Developer, slideOver(0.0, 2.0) },
        { "GrabDrive", "GrabAngularProportionalRecovery", "07. Grab Drive", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabDrive", "GrabAngularConstantRecovery", "07. Grab Drive", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabDrive", "GrabConstraintMaxForce", "07. Grab Drive",
            "Contact and initial-attachment linear force ceiling. Free motion uses actual mass and Free Linear "
            "Acceleration.",
            Group::Developer, slideOver(0.0, 10000.0) },
        { "GrabDrive", "GrabEffectiveMotorMassFloor", "07. Grab Drive",
            "Keeps the motor's effective mass finite for very light or unstable bodies.",
            Group::Developer, clampTo(0.0, 100.0) },
        { "GrabDrive", "GrabMaxForceToMassRatio", "07. Grab Drive",
            "Contact force per unit effective mass. This caps the contact ceiling for light objects.",
            Group::Developer, slideOver(0.0, 2000.0) },
        { "GrabDrive", "GrabFreeLinearAcceleration", "07. Grab Drive",
            "Free-motion linear effort per unit effective driven mass, in Havok acceleration units. Contact keeps its "
            "separate ceiling.",
            Group::Developer, clampTo(1.0, 10000.0) },
        { "GrabDrive", "GrabFreeAngularAcceleration", "07. Grab Drive",
            "Free-motion angular effort per unit maximum principal inertia. The grip lever adds capacity to oppose "
            "off-center linear effort; the native solver determines the applied torque.",
            Group::Developer, clampTo(1.0, 60000.0) },
        { "GrabDrive", "ForceGrabAttachSettleSeconds", "07. Grab Drive",
            "Waits for a stable live hand transform before a force grab freezes its relation.",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabDrive", "GrabForceFadeInTime", "07. Grab Drive",
            "Time for grab force to ramp in after pickup.", Group::Developer, slideOver(0.0, 1.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearTauMultiplier", "07. Grab Drive",
            "Weapon response multipliers for both loose and equipped weapons. Max Force and Angular Force multipliers "
            "control contact and initial-attachment budgets. Free-motion budgets use actual mass, inertia, and the "
            "grip offset.",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularTauMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintCollisionTauMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearDampingMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularDampingMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintMaxForceMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 8.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularForceMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 8.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintLinearRecoveryMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabLooseWeaponSharedConstraintAngularRecoveryMultiplier", "07. Grab Drive", "",
            Group::Developer, clampTo(0.05, 4.0) },
        { "GrabDrive", "GrabTauMin", "07. Grab Drive",
            "Contact-softened tau. Normal held linear/angular tau stay fixed at their base values.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "GrabDrive", "GrabTauLerpSpeed", "07. Grab Drive", "", Group::Developer, slideOver(0.0, 5.0) },
        { "GrabDrive", "GrabLongObjectAngularScalingEnabled", "07. Grab Drive",
            "Reduces held angular speed for long objects by grip-to-extents lever length.\nThis keeps long "
            "guns/objects from sweeping as if their far end had no leverage.",
            Group::Developer },
        { "GrabDrive", "GrabLongObjectReferenceLeverGameUnits", "07. Grab Drive", "",
            Group::Developer, slideOver(1.0, 240.0) },
        { "GrabDrive", "GrabLongObjectMinAngularScale", "07. Grab Drive", "",
            Group::Developer, slideOver(0.05, 1.0) },
        { "GrabDrive", "GrabMaxInertiaRatio", "07. Grab Drive",
            "Clamps extreme inertia tensors on grabbed rigid bodies to keep the solver stable.",
            Group::Developer, slideOver(1.0, 100.0) },
        { "GrabDrive", "GrabMinInertia", "07. Grab Drive", "", Group::Developer, clampTo(0.0001, 100.0, SettingRange{ 0.0001, 1.0 }) },
        { "GrabSafetyAndRelease", "ThrowVelocityMultiplier", "08. Grab Safety and Release",
            "Multiplies held-object velocity when you release to throw.", Group::Developer, slideOver(0.0, 5.0) },
        { "GrabSafetyAndRelease", "GrabControllerDerivedThrowVelocityEnabled", "08. Grab Safety and Release",
            "Throws follow tracked hand movement. Wrist rotation supplies spin without steering the throw.",
            Group::Developer },
        { "GrabSafetyAndRelease", "GrabThrowMaxVelocityHavok", "08. Grab Safety and Release", "",
            Group::Developer, slideOver(1.0, 60.0) },
        { "GrabSafetyAndRelease", "GrabThrowAngularVelocityScale", "08. Grab Safety and Release", "",
            Group::Developer, slideOver(0.0, 2.0) },
        { "GrabSafetyAndRelease", "GrabThrowMaxAngularVelocityRadiansPerSecond", "08. Grab Safety and Release", "",
            Group::Developer, slideOver(0.0, 60.0) },
        { "GrabSafetyAndRelease", "GrabReleaseHandCollisionDelaySeconds", "08. Grab Safety and Release",
            "Keeps hand collision suppressed briefly after release so the object is not launched instantly.",
            Group::Developer, slideOver(0.0, 3.0) },
        { "GrabSafetyAndRelease", "ShoulderStashEnabled", "08. Grab Safety and Release",
            "Shoulder stash transfers a held loose item to inventory when grip is released inside the HMD-relative "
            "back volume.\nGenerated FRIK body-zone colliders are backup evidence for tracking loss and remain the "
            "preferred path for holsters.",
            Group::Developer },
        { "GrabSafetyAndRelease", "EquippedWeaponShoulderStashEnabled", "08. Grab Safety and Release",
            "Equipped weapons, either one, use the same HMD-relative back volumes and return to the "
            "inventory.\nToggle-grab: press inside with the hand carrying the weapon.\nHold-to-grab: hold inside and "
            "release.",
            Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashUseBodyZoneColliders", "08. Grab Safety and Release", "",
            Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashUseHmdBackVolume", "08. Grab Safety and Release", "",
            Group::Developer },
        { "GrabSafetyAndRelease", "ShoulderStashEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 40.0) },
        { "GrabSafetyAndRelease", "ShoulderStashExitPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 60.0) },
        { "GrabSafetyAndRelease", "ShoulderStashRecentContactSeconds", "08. Grab Safety and Release", "",
            Group::Developer, slideOver(0.0, 0.66) },
        { "GrabSafetyAndRelease", "ShoulderStashSustainedContactMissSeconds", "08. Grab Safety and Release", "",
            Group::Developer, slideOver(0.0, 1.33) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackRightOffset", "08. Grab Safety and Release",
            "Centre of the right shoulder stash volume behind the headset, a position in game units.",
            Group::Developer, slideOver(-100.0, 100.0) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackLeftOffset", "08. Grab Safety and Release",
            "Centre of the left shoulder stash volume behind the headset, a position in game units.",
            Group::Developer, slideOver(-100.0, 100.0) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackRadiusGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(1.0, 80.0) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 40.0) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackExitPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 60.0) },
        { "GrabSafetyAndRelease", "ShoulderStashHmdBackMinBehindGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 40.0) },
        { "GrabSafetyAndRelease", "ShoulderStashShowCollectedNotifications", "08. Grab Safety and Release", "",
            Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeEnabled", "08. Grab Safety and Release",
            "Mouth consume uses a tight HMD-relative sphere near the mouth.\nOnly loose ALCH ingestibles can become "
            "mouth candidates; other objects use backpack stash/drop paths.",
            Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeAllowPoison", "08. Grab Safety and Release", "",
            Group::Developer },
        { "GrabSafetyAndRelease", "MouthConsumeHmdOffset", "08. Grab Safety and Release",
            "Mouth position relative to the headset, in game units. Each value: -120 to 120.",
            Group::Developer, slideOver(-120.0, 120.0) },
        { "GrabSafetyAndRelease", "MouthConsumeRadiusGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(1.0, 80.0) },
        { "GrabSafetyAndRelease", "MouthConsumeEnterPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 40.0) },
        { "GrabSafetyAndRelease", "MouthConsumeExitPaddingGameUnits", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 60.0) },
        { "GrabSafetyAndRelease", "MouthConsumeMinDwellSeconds", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabSafetyAndRelease", "MouthConsumeMaxSpeedGameUnitsPerSecond", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 1000.0) },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementSlowdownEnabled", "08. Grab Safety and Release",
            "Reduces player movement speed while carrying held mass.", Group::Developer },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMassProportion", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 10.0) },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMassExponent", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 4.0) },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementMaxReduction", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 99.0) },
        { "GrabSafetyAndRelease", "GrabHeldMassMovementFadeOutSeconds", "08. Grab Safety and Release", "",
            Group::Developer, clampTo(0.0, 60.0) },
        { "GrabAcquisition", "GrabInputLeewaySeconds", "09. Grab Acquisition",
            "Seconds a held grab press waits for the hand to become ready to grab. Range: 0 to 0.5.",
            Group::Developer, slideOver(0.0, 0.5) },
        { "GrabAcquisition", "GrabInputForceSeconds", "09. Grab Acquisition",
            "After that wait, seconds the held press keeps being sent every frame before ROCK stops trying. Range: 0 "
            "to 0.3.",
            Group::Developer, slideOver(0.0, 0.3) },
        { "GrabAcquisition", "GrabTouchAcquireDistanceGameUnits", "09. Grab Acquisition",
            "Distance from the hand pocket where an object is considered already touching.",
            Group::Developer, slideOver(0.5, 20.0) },
        { "GrabAcquisition", "GrabNearConvergeDistanceGameUnits", "09. Grab Acquisition",
            "Distance where the object should converge into the palm pocket instead of finalizing immediately.",
            Group::Developer, slideOver(4.0, 100.0) },
        { "GrabAcquisition", "GrabTwoHandPivotMinSeparationGameUnits", "09. Grab Acquisition",
            "Two hands on one loose non-weapon object turn it along the line between their grip points only when "
            "those points are at least this far apart.\nCloser than this (two pinches on a cartridge), the object "
            "stays as the first hand holds it, and the hand left holding it keeps its own grip.\n0 = always pivot. "
            "Applies to the next two-hand hold.",
            Group::Developer, slideOver(0.0, 30.0) },
        { "GrabAcquisition", "GrabPocketDepthGameUnits", "09. Grab Acquisition",
            "Hand-pocket offset and radius, in game units, from the root-flattened palm frame.",
            Group::Developer, slideOver(0.0, 30.0) },
        { "GrabAcquisition", "GrabPocketRadiusGameUnits", "09. Grab Acquisition", "",
            Group::Developer, slideOver(0.5, 30.0) },
        { "GrabAcquisition", "GrabSeatDepthMaxGameUnits", "09. Grab Acquisition",
            "Legacy seat-depth stop and penetration backstop dimensions, in game units.",
            Group::Developer, slideOver(0.0, 100.0) },
        { "GrabAcquisition", "GrabSeatDepthFootprintRadiusGameUnits", "09. Grab Acquisition", "",
            Group::Developer, slideOver(1.0, 30.0) },
        { "GrabAcquisition", "GrabSeatPenetrationBackstopFootprintRadiusGameUnits", "09. Grab Acquisition", "",
            Group::Developer, slideOver(1.0, 30.0) },
        { "GrabAcquisition", "GrabSeatDepthSkinGameUnits", "09. Grab Acquisition", "",
            Group::Developer, slideOver(0.0, 5.0) },
        { "GrabAcquisition", "GrabCloseSeatMode", "09. Grab Acquisition",
            "0 = Current seat. 1 = Local grasp solve.\nApplies on the next close grab of a loose object. Pulled, "
            "forced, pinched and authored grabs are unchanged.\nMode 1 reads the mesh in front of the palm. A handle, "
            "neck, rim or round body is turned into the hand,\nand the fingers close on that part instead of the body "
            "behind it.\nWhen no such part is found the current seat is used. ROCK.log reports the result of each "
            "grab.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "GrabAcquisition", "GrabGripInsetGameUnits", "09. Grab Acquisition",
            "Moves the captured grip point slightly inside the object from the raw hit/mesh seed.",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabAcquisition", "GrabConvergeMaxTimeSeconds", "09. Grab Acquisition",
            "Optional near-converge diagnostic timeout. A value of 0 disables timeout promotion.",
            Group::Developer, slideOver(0.0, 2.0) },
        { "GrabAcquisition", "GrabConvergeStableSeconds", "09. Grab Acquisition", "",
            Group::Developer, slideOver(0.012, 0.132) },
        { "GrabAcquisition", "GrabConvergeMaxSeparatingSpeedGameUnitsPerSecond", "09. Grab Acquisition", "",
            Group::Developer, slideOver(0.0, 200.0) },
        { "GrabAcquisition", "GrabAcquisitionVisualStartDistanceGameUnits", "09. Grab Acquisition", "",
            Group::Developer, slideOver(0.5, 100.0) },
        { "GrabContactEvidence", "GrabSurfaceBehindPalmToleranceGameUnits", "10. Grab Contact Evidence", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "GrabContactEvidence", "GrabOppositionContactMaxAgeSeconds", "10. Grab Contact Evidence",
            "Maximum age in seconds for semantic contacts used as touch evidence.",
            Group::Developer, slideOver(0.0, 0.66) },
        { "GrabContactEvidence", "GrabPinchCloseSelectionEnabled", "10. Grab Contact Evidence",
            "Allows a fallback close-selection cast along the pinch direction when the palm close cast misses.",
            Group::Developer },
        { "GrabContactEvidence", "GrabPinchMaxPocketDistanceGameUnits", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.1, 80.0) },
        { "GrabContactEvidence", "GrabPinchMinFingerGapGameUnits", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 40.0) },
        { "GrabContactEvidence", "GrabPinchMaxFingerGapGameUnits", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.1, 80.0) },
        { "GrabContactEvidence", "GrabPinchThumbIndexMaxOpenValue", "10. Grab Contact Evidence",
            "Thumb and opposed finger curl of the weapon support hand's opposition pose\n(1 open, 0 closed). A "
            "loose-object pinch opens to the object's thickness instead.",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabPinchOtherFingerCurlValue", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabPinchDetectionDirection", "10. Grab Contact Evidence",
            "Pinch detection direction in ROCK hand space, stored as a position and normalized: X=fingers, Y=palm "
            "depth, Z=signed cross-palm.",
            Group::Developer, slideOver(-1.0, 1.0) },
        { "GrabContactEvidence", "GrabPinchDetectionAxisBlend", "10. Grab Contact Evidence",
            "Blends the live thumb-index axis with the authored pinch direction for mesh surface search.",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabHandLerpEnabled", "10. Grab Contact Evidence",
            "Moves the rendered hand toward the mesh-matched held pose during grab startup.",
            Group::Developer },
        { "GrabContactEvidence", "GrabHandLerpTimeMin", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabHandLerpTimeMax", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabHandLerpMinDistance", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 80.0) },
        { "GrabContactEvidence", "GrabHandLerpMaxDistance", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 120.0) },
        { "GrabContactEvidence", "GrabHandReturnEnabled", "10. Grab Contact Evidence",
            "Smooths the rendered hand back to live tracking after releasing a grabbed object.",
            Group::Developer },
        { "GrabContactEvidence", "GrabHandReturnTimeMin", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabHandReturnTimeMax", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "GrabContactEvidence", "GrabHandReturnMinDistance", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 80.0) },
        { "GrabContactEvidence", "GrabHandReturnMaxDistance", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 120.0) },
        { "GrabContactEvidence", "GrabHandReturnMinAngleDegrees", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 180.0) },
        { "GrabContactEvidence", "GrabHandReturnMaxAngleDegrees", "10. Grab Contact Evidence", "",
            Group::Developer, clampTo(0.0, 180.0) },
        { "FingerPoses", "WeaponGrabCapsuleProbes", "11. Finger Poses",
            "Testing: pose dynamic equipped weapon and part grips with the loose-object capsule probes, with thumb "
            "and index reaching onto small parts such as bolts and slides, instead of the swept arc.\nApplies on the "
            "next dynamic weapon grip, on either hand. Authored poses retain priority.",
            Group::Developer },
        { "FingerPoses", "LooseGrabFingerPoseMode", "11. Finger Poses",
            "0 = Capsule sweep.\n1 = Swept arc: the finger solve loose grabs used before the capsule sweep\n(pad-probe "
            "refinement, contact splay, surface-aim local transforms).\nApplies on the next loose-object grab. Weapon "
            "grips use WeaponGrabCapsuleProbes.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "FingerPoses", "GrabMeshJointPoseEnabled", "11. Finger Poses",
            "Sends per-joint finger values to FRIK when available instead of one curl per finger.",
            Group::Developer },
        { "FingerPoses", "GrabFingerMinValue", "11. Finger Poses",
            "Lowest allowed finger value. 0=fully closed, 1=fully straight.", Group::Developer, slideOver(0.0, 1.0) },
        { "FingerPoses", "GrabFingerPoseSmoothingSpeed", "11. Finger Poses",
            "Maximum per-second finger pose movement.", Group::Developer, slideOver(0.0, 60.0) },
        { "FingerPoses", "GrabMeshLocalTransformPoseEnabled", "11. Finger Poses",
            "Uses FRIK's authored finger joint transforms plus bounded mesh-contact aim correction.",
            Group::Developer },
        { "FingerPoses", "GrabFingerLocalTransformSmoothingSpeed", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 60.0) },
        { "FingerPoses", "GrabFingerLocalTransformMaxCorrectionDegrees", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 90.0) },
        { "FingerPoses", "GrabFingerSurfaceAimStrength", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 1.0) },
        { "FingerPoses", "GrabFingerRejectBacksideHits", "11. Finger Poses", "",
            Group::Developer },
        { "FingerPoses", "GrabFingerSurfacePlaneToleranceGameUnits", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 10.0) },
        { "FingerPoses", "GrabFingerSweepContactRadiusGameUnits", "11. Finger Poses",
            "Swept-arc finger solver: finger-pad contact sphere radius. Bigger = fingers\nstop earlier (hover), "
            "smaller = deeper wrap before contact.\nFinger-pad contact radius for the swept-arc solver (0.05-4). The "
            "old\nfinger-width air gap was a solver bug (fixed: chord de-rotation), NOT this\nradius - values near 0 "
            "now make fingers clip into the mesh.",
            Group::Developer, slideOver(0.05, 4.0) },
        { "FingerPoses", "GrabFingerSweepMaxOpenValue", "11. Finger Poses",
            "How far past the authored open pose fingers may hyper-extend when an\nobject is thicker than the open "
            "finger span (hFRIK flex units: 1.0 =\nauthored open pose, 2.0 = engine ceiling). The swept-arc close "
            "starts\nfrom the most-open allowed row, so a raised cap lets fingers lie flat\non surfaces that "
            "interpenetrate the default open hand. Valid range\n1.0-2.0; out-of-range silently falls back (1.4 fingers "
            "/ 2.0 thumb).",
            Group::Developer, slideOver(1.0, 2.0) },
        { "FingerPoses", "GrabThumbSweepMaxOpenValue", "11. Finger Poses", "",
            Group::Developer, slideOver(1.0, 2.0) },
        { "FingerPoses", "GrabThumbOppositionStrength", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 1.0) },
        { "FingerPoses", "GrabThumbAlternateCurveStrength", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 1.0) },
        { "FingerPoses", "GrabThumbSurfaceSafetyMarginGameUnits", "11. Finger Poses", "",
            Group::Developer, slideOver(0.0, 5.0) },
        { "MeshContact", "GrabLateralWeight", "12. Mesh Contact",
            "Weighting used when choosing the best mesh triangle for grab contact.",
            Group::Developer, slideOver(0.0, 1.0) },
        { "MeshContact", "GrabDirectionalWeight", "12. Mesh Contact", "",
            Group::Developer, slideOver(0.0, 1.0) },
        { "MeshContact", "GrabMaxTriangleDistance", "12. Mesh Contact", "",
            Group::Developer, slideOver(1.0, 500.0) },
        { "MeshContact", "GrabContactPatchMaxNormalAngleDegrees", "12. Mesh Contact", "",
            Group::Developer, slideOver(0.0, 179.0) },
        { "MeshContact", "GrabAlignmentMaxSelectionToMeshDistance", "12. Mesh Contact",
            "Furthest the grab point may snap from the selection hit onto the mesh.",
            Group::Developer, slideOver(0.0, 50.0) },
        { "CloseHandCurl", "SelectedCloseFingerCurlEnabled", "13. Close-Hand Curl",
            "Pre-curls the hand when close selection is active.", Group::Developer },
        { "CloseHandCurl", "SelectedCloseFingerAnimMaxHandSpeed", "13. Close-Hand Curl", "",
            Group::Developer, slideOver(0.0, 5.0) },
        { "CloseHandCurl", "SelectedCloseFingerAnimValue", "13. Close-Hand Curl", "",
            Group::Developer, slideOver(0.0, 1.0) },
        { "Haptics", "FiringGripHapticDurationSeconds", "14. Haptics",
            "Length of the pulse on the acting hand when the firing grip attaches or detaches, in seconds. Range: 0.01 "
            "to 0.5.",
            Group::Developer, clampTo(0.01, 0.50) },
        { "Haptics", "FiringGripAttachHapticIntensity", "14. Haptics",
            "Strength of the firing-grip attach pulse. Range: 0 to 1.", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "FiringGripDetachHapticIntensity", "14. Haptics",
            "Strength of the firing-grip detach pulse. Range: 0 to 1.", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "GrabHapticsEnabled", "14. Haptics",
            "Pulses for grab commit, pull start/catch, and held-object impacts.",
            Group::Developer },
        { "Haptics", "GrabHapticDurationSeconds", "14. Haptics", "", Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "GrabHapticBaseIntensity", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "GrabHapticMaxIntensity", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "GrabHapticMassScale", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "GrabHapticMassExponent", "14. Haptics", "", Group::Developer, clampTo(0.0, 2.0) },
        { "Haptics", "PullStartHapticIntensity", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "PullCatchHapticIntensity", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "SelectionLockHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "SelectionLockReleaseHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "SelectionLockReleaseHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "SurfaceGrabHapticsEnabled", "14. Haptics",
            "Fixed-surface grabs use a longer confirmation pulse.", Group::Developer },
        { "Haptics", "SurfaceGrabHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "SurfaceGrabHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticsEnabled", "14. Haptics", "", Group::Developer },
        { "Haptics", "HeldImpactHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "HeldImpactHapticBaseIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticMaxIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticSpeedScale", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticMassScale", "14. Haptics", "", Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticMassExponent", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 2.0) },
        { "Haptics", "HeldImpactHapticMinSpeedGameUnits", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1000.0) },
        { "Haptics", "HeldImpactHapticCooldownSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "HeldImpactHapticDampedMultiplier", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "ShoulderStashHapticsEnabled", "14. Haptics",
            "Shoulder stash haptics pulse clearly while an item is inside the stash zone, then fire a stronger "
            "confirmation on transfer.",
            Group::Developer },
        { "Haptics", "ShoulderStashCandidateHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "ShoulderStashCandidateHapticBaseIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "ShoulderStashCandidateHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "ShoulderStashCandidateHapticIntervalSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 2.0) },
        { "Haptics", "ShoulderStashCommitHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "ShoulderStashCommitHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "MouthConsumeHapticsEnabled", "14. Haptics",
            "Mouth consume haptics pulse while a consumable is in the mouth zone, then fire a confirmation on consume.",
            Group::Developer },
        { "Haptics", "MouthConsumeCandidateHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "MouthConsumeCandidateHapticBaseIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "MouthConsumeCandidateHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "MouthConsumeCandidateHapticIntervalSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 2.0) },
        { "Haptics", "MouthConsumeCommitHapticDurationSeconds", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 0.2) },
        { "Haptics", "MouthConsumeCommitHapticIntensity", "14. Haptics", "",
            Group::Developer, clampTo(0.0, 1.0) },
        { "Haptics", "DebugDrawGrabAuthorityProxy", "14. Haptics",
            "Draws the hidden no-contact grab authority proxy target/axes used by ROCK dynamic grab.",
            Group::Developer },
        { "DebugOverlay", "DebugShowColliders", "15. Debug Overlay",
            "Master switch for all ROCK-owned collider visualizations below.", Group::Developer },
        { "DebugOverlay", "DebugShowTargetColliders", "15. Debug Overlay",
            "Draws the selected or held target body when the collider master is enabled.",
            Group::Developer },
        { "DebugOverlay", "DebugShowHandAxes", "15. Debug Overlay",
            "Draws live transform axes. Red=X, green=Y, blue=Z.", Group::Developer },
        { "DebugOverlay", "DebugShowGrabPivots", "15. Debug Overlay",
            "Draws grab constraint pivots and the error line between hand-side and object-side pivots.",
            Group::Developer },
        { "DebugOverlay", "DebugShowGrabPocketNormal", "15. Debug Overlay",
            "Draws the resolved mesh surface frame after a dynamic grab.", Group::Developer },
        { "DebugOverlay", "DebugDrawGrabForceTorque", "15. Debug Overlay",
            "Draws a focused held-grab pivot force/lever view: target pivot, live pivot, correction vector, lever arm, "
            "and torque witness.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabForceTorqueText", "15. Debug Overlay",
            "Adds a compact world label to the focused grab force/lever view.", Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPivotSourceCollider", "15. Debug Overlay",
            "Draws the hknp body/collider that owns the frozen object-side pivot B.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPivotSourceEvidence", "15. Debug Overlay",
            "Draws the mesh triangle/contact-patch evidence that produced the frozen pivot.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabSupportFrame", "15. Debug Overlay",
            "Draws the frozen grab support face frame at pivot B: normal, support axis, and binormal.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabPockets", "15. Debug Overlay",
            "Draws the live palm pocket and thumb/index pinch pocket for each hand.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabFingerProbes", "15. Debug Overlay",
            "Draws the grab finger solve live: while a hand has an object selected, the finger spheres the solve "
            "tests at the curl it would choose (grey wire = closed without touching, green = closed onto the "
            "surface, yellow = opened to clear, red = blocked; bright spheres touch the mesh) and each fingertip's "
            "sweep path. While holding, the spheres follow the drawn fingers.",
            Group::Developer },
        { "DebugOverlay", "DebugShowPalmVectors", "15. Debug Overlay",
            "Draws palm center plus close/far selection aim rays.", Group::Developer },
        { "DebugOverlay", "DebugDrawHandColliders", "15. Debug Overlay",
            "More specific overlay filters and draw limits.", Group::Developer },
        { "DebugOverlay", "DebugDrawHandBoneColliders", "15. Debug Overlay", "",
            Group::Developer },
        { "DebugOverlay", "DebugDrawBodyBoneColliders", "15. Debug Overlay", "",
            Group::Developer },
        { "DebugOverlay", "DebugDrawDynamicHandColliders", "15. Debug Overlay", "",
            Group::Developer },
        { "DebugOverlay", "DebugDrawHandBoneContacts", "15. Debug Overlay", "",
            Group::Developer },
        { "DebugOverlay", "DebugMaxHandBoneBodiesDrawn", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 48.0) },
        { "DebugOverlay", "DebugMaxBodyBoneBodiesDrawn", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 64.0) },
        { "DebugOverlay", "DebugDrawWeaponColliders", "15. Debug Overlay", "",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGrabbedWeaponPartCollider", "15. Debug Overlay",
            "Draws the exact current weapon-part collider selected by each active part grip.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawNativeScopeActivation", "15. Debug Overlay",
            "Native-scope diagnosis: sight bounds, live camera, exact pre-activation target pose/aim/up guides, write "
            "readback, parent/local camera, and HMD guides.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawAuthoredGripActivationZones", "15. Debug Overlay",
            "Visualizes the topology-specific authored cone/arc, live approach vector, and\noptional palm/fingertip "
            "collision witnesses. Witnesses are diagnostic only.\nAlso draws the firing-grip reattach zone and live "
            "palm vectors during part carry.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawWeaponAuthority", "15. Debug Overlay",
            "Draws two-handed weapon/hand/FRIK authority frames and mismatch evidence.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawControllerAim", "15. Debug Overlay",
            "Compare both tracked wand frames, reference aim (cyan), configured aim (gold), damped aim (purple) and "
            "final barrel (green). The reference basis is a model, not physical ground truth. Does not change aiming.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawGunstock", "15. Debug Overlay",
            "Gunstock measurements: exact tracked controller origins/axes, controller baseline, weapon forward and "
            "authored hand seats. Each support grab marks a sample. Does not change aiming.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawLooseWeaponGripZones", "15. Debug Overlay",
            "Draws the loose-weapon projected grip point, palm relation, and equip radius state.",
            Group::Developer },
        { "DebugOverlay", "DebugDrawDynamicWeaponColliders", "15. Debug Overlay",
            "Draws the layer-51 dynamic weapon compound proxy and contact/correction telemetry.",
            Group::Developer },
        { "DebugOverlay", "DebugMaxWeaponBodiesDrawn", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 200.0) },
        { "DebugOverlay", "DebugMaxShapeCapturesPerFrame", "15. Debug Overlay",
            "Bounded overlay work and memory budgets. Zero disables the corresponding capture/upload/draw stream where "
            "noted.\nRanges: captures 0..32, compound slots 1..1024, depth 1..8, queued/completed 0..256,\nuploads "
            "0..16, cache entries 16..4096, cache bytes 1048576..536870912,\nbody instances 0..163, line vertices "
            "0..65536.",
            Group::Developer, slideOver(0.0, 32.0) },
        { "DebugOverlay", "DebugMaxConvexSupportVertices", "15. Debug Overlay", "",
            Group::Developer, slideOver(4.0, 32.0) },
        { "DebugOverlay", "DebugMaxCompoundChildren", "15. Debug Overlay", "",
            Group::Developer, slideOver(1.0, 1024.0) },
        { "DebugOverlay", "DebugMaxCompoundDepth", "15. Debug Overlay", "",
            Group::Developer, slideOver(1.0, 8.0) },
        { "DebugOverlay", "DebugMaxShapeQueuedJobs", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 256.0) },
        { "DebugOverlay", "DebugMaxShapeCompletedJobs", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 256.0) },
        { "DebugOverlay", "DebugMaxShapeUploadsPerFrame", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 16.0) },
        { "DebugOverlay", "DebugMaxShapeCacheEntries", "15. Debug Overlay", "",
            Group::Developer, slideOver(16.0, 4096.0) },
        { "DebugOverlay", "DebugMaxShapeCacheBytes", "15. Debug Overlay", "",
            Group::Developer, slideOver(1048576.0, 536870912.0) },
        { "DebugOverlay", "DebugMaxBodyInstances", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 163.0) },
        { "DebugOverlay", "DebugMaxLineVertices", "15. Debug Overlay", "",
            Group::Developer, slideOver(0.0, 65536.0) },
        { "DebugOverlay", "DebugUseBoundsForHeavyConvex", "15. Debug Overlay", "",
            Group::Developer },
        { "LogsAndTelemetry", "DebugVerboseLogging", "16. Logs and Telemetry",
            "Very noisy per-frame diagnostic logging. Use only while investigating a specific issue.",
            Group::Developer },
        { "LogsAndTelemetry", "DebugTestbenchTelemetry", "16. Logs and Telemetry",
            "Publishes per-frame hand, head and controller state and ROCK events to shared memory for the external "
            "test bench. Leave off for play.",
            Group::Developer },
        { "SkeletonDebug", "DebugShowRootFlattenedFingerSkeletonMarkers", "17. Skeleton Debug",
            "Draws markers and axes for live first-person skeleton bones.", Group::Developer },
        { "SkeletonDebug", "DebugShowSkeletonBoneVisualizer", "17. Skeleton Debug", "",
            Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneMode", "17. Skeleton Debug",
            "0 = minimal/core bones, 1 = core body and fingers, higher modes may draw more if supported.",
            Group::Developer, slideOver(0.0, 3.0) },
        { "SkeletonDebug", "DebugSkeletonBoneSource", "17. Skeleton Debug",
            "Selects the skeleton source used by the debug visualizer.", Group::Developer, slideOver(1.0, 2.0) },
        { "SkeletonDebug", "DebugMaxSkeletonBonesDrawn", "17. Skeleton Debug",
            "Draw limits and filters for skeleton debug drawing.", Group::Developer, slideOver(0.0, 768.0) },
        { "SkeletonDebug", "DebugDrawSkeletonBoneAxes", "17. Skeleton Debug", "",
            Group::Developer },
        { "SkeletonDebug", "DebugMaxSkeletonBoneAxesDrawn", "17. Skeleton Debug", "",
            Group::Developer, slideOver(0.0, 768.0) },
        { "SkeletonDebug", "DebugSkeletonAxisBoneFilter", "17. Skeleton Debug", "",
            Group::Developer },
        { "SkeletonDebug", "DebugLogSkeletonBones", "17. Skeleton Debug",
            "Logs bone names and truncation information to ROCK.log.", Group::Developer },
        { "SkeletonDebug", "DebugLogSkeletonBoneTruncation", "17. Skeleton Debug", "",
            Group::Developer },
        { "SkeletonDebug", "DebugSkeletonBoneLogIntervalFrames", "17. Skeleton Debug", "",
            Group::Developer, slideOver(1.0, 1000.0) },
        { "SkeletonDebug", "DebugSkeletonBoneLogFilter", "17. Skeleton Debug", "",
            Group::Developer },
        { "SkeletonDebug", "DebugRootFlattenedFingerSkeletonMarkerSize", "17. Skeleton Debug",
            "Marker and axis sizes for skeleton debug visuals.", Group::Developer, slideOver(0.1, 10.0) },
        { "SkeletonDebug", "DebugSkeletonBonePointSize", "17. Skeleton Debug", "",
            Group::Developer, slideOver(0.1, 10.0) },
        { "SkeletonDebug", "DebugSkeletonBoneAxisLength", "17. Skeleton Debug", "",
            Group::Developer, slideOver(0.1, 30.0) },
        { "PhysicsTiming", "HavokTimingFixEnabled", "18. Physics Timing",
            "Stabilizes FO4VR physics timing when rendering falls below the reference rate.",
            Group::Developer },
        { "PhysicsTiming", "VatsPhysicsFixes", "18. Physics Timing",
            "VATS Physics Fixes\nPreserves equipped-weapon handling speed during VATS and other game slow "
            "motion.\nCompensates the weapon drive and constraint while keeping dynamic collision.\nChanges apply at "
            "the next physics step; false restores uncompensated handling.",
            Group::Developer },
        { "PhysicsTiming", "HavokTimingFixMinPhysicsFrameRate", "18. Physics Timing", "",
            Group::Developer, slideOver(30.0, 240.0) },
        { "PhysicsTiming", "HavokTimingFixMaxSubsteps", "18. Physics Timing", "",
            Group::Developer, slideOver(1.0, 6.0) },
        { "ScopePlacement", "NativeScopeFiringGripFallbackOffset", "19. Scope Placement",
            "Scope Anchor Without Optic Geometry\nUsed only when a scoped weapon has no separate Scope or Sight part "
            "ROCK can measure, such as an optic modelled into the receiver. The scope view then starts at the firing "
            "hand's grip, moved by these values. Weapons with a measurable optic anchor on the optic and ignore these "
            "keys.\nOffsets are game units (70 = 1 metre) in the weapon's axes: "
            "X sideways, +Y toward the muzzle, Z up; all zero puts the anchor on the firing grip.\nRotations are "
            "degrees added after the game's camera calibration: pitch about X, yaw about Z, roll about the muzzle "
            "axis.\nPosition moves the activation point, camera and overlay together; rotation corrects the camera "
            "and overlay only.\nThis key: the position. Each value: -100 to 100.",
            Group::Developer, clampTo(-100.0, 100.0) },
        { "ScopePlacement", "NativeScopeFiringGripFallbackRotation", "19. Scope Placement",
            "Rotation of the scope anchor used without optic geometry (see the offset above), in degrees: rot is "
            "[pitch, roll, yaw]. Each value: -180 to 180.",
            Group::Developer, clampTo(-180.0, 180.0) },
        { "ScopePlacement", "NativeScopeOverlayOffset", "19. Scope Placement",
            "Scope Overlay\nFine-tunes only the game's own world_scope.nif overlay. The activation camera stays on "
            "the resolved sight or firing-grip anchor, so these values do not change when the scope activates.\nAll "
            "zero keeps automatic placement with the game's own model orientation and scale.\nOffsets are game units "
            "in the calibrated model frame; rotations are degrees: pitch about X, yaw about Z, roll about the optical "
            "Y axis.\nTune one axis at a time; replacement scope meshes can use a reversed sign.\nThis key: the "
            "position. Each value: -100 to 100.",
            Group::Developer, clampTo(-100.0, 100.0) },
        { "ScopePlacement", "NativeScopeOverlayRotation", "19. Scope Placement",
            "Scope overlay rotation, in degrees: rot is [pitch, roll, yaw]. Each value: -180 to 180.",
            Group::Developer, clampTo(-180.0, 180.0) },
    };
}
