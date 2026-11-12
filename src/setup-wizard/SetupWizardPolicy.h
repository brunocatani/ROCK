#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "setup-wizard/SteamVrBindingPolicy.h"

/*
 * The ROCK setup wizard's content and decisions, free of the game and of drawing:
 * its topic pages, the options each page offers in the player's words, and what
 * finishing the wizard does with the player's controllers.
 *
 * The wizard asks only what a player should decide on the first start. Every
 * other setting, and every button binding, stays in ROCK.toml and PALM's Config.
 * Each option names a ROCK.toml setting; its choices are values that setting takes,
 * one of them its default (tests/SetupWizardTests.cpp checks both against the
 * catalog and the shipped example file).
 */
namespace rock::setup_wizard_policy
{
    namespace sv = steamvr_binding_policy;

    struct Page
    {
        std::string_view title;
        std::string_view intro;
    };

    inline constexpr std::array kPages{
        Page{ "Grabbing", "How you pick things up." },
        Page{ "Holding weapons", "How your hands take, hold and let go of weapons." },
        Page{ "Aiming and shooting", "How weapons feel in your hands when you aim and fire." },
        Page{ "Grenades and healing", "How you throw grenades and use stimpaks." },
        Page{ "Bipod and VATS", "What the right stick click and the VATS button do." },
    };

    struct Choice
    {
        std::string_view value;
        std::string_view label;
        std::string_view detail;
    };

    inline constexpr std::size_t kMaxChoices = 5;

    // While another option (`key`) is at `value`, this option has only `fixed`, for `reason`.
    struct FixedWhen
    {
        std::string_view key;
        std::string_view value;
        std::string_view fixed;
        std::string_view reason;
    };

    // The option shows only while another option (`key`) is at `value`.
    struct ShownWhen
    {
        std::string_view key;
        std::string_view value;
    };

    struct Option
    {
        std::size_t page;
        std::string_view section;
        std::string_view key;
        std::string_view title;
        std::array<Choice, kMaxChoices> choices;
        FixedWhen fixedWhen{};
        ShownWhen shownWhen{};

        [[nodiscard]] constexpr std::size_t choiceCount() const noexcept
        {
            std::size_t count = 0;
            while (count < choices.size() && !choices[count].value.empty()) ++count;
            return count;
        }

        [[nodiscard]] constexpr const Choice* find(std::string_view value) const noexcept
        {
            for (std::size_t i = 0; i < choiceCount(); ++i)
                if (choices[i].value == value) return &choices[i];
            return nullptr;
        }
    };

    inline constexpr std::array kOptions{
        // ---- Grabbing ----
        Option{ 0, "Grabbing", "GrabDirection", "Which way your hand grabs",
            { {
                { "1", "Palm down (HIGGS style)",
                    "Your hand reaches out of the open palm: the object your palm faces is the one you grab, near "
                    "or far." },
                { "2", "Fingers (Alyx style)",
                    "Your hand reaches along your fingers: the object your fingers point at is the one you grab, "
                    "near or far." },
            } } },
        Option{ 0, "Grabbing", "FarGrabStyle", "Grabbing things far away",
            { {
                { "1", "Immediate", "Point at an object and press grab: it flies straight into your hand." },
                { "2", "Pull (flick)",
                    "Hold grab to lock on and flick your hand toward you. Keep holding to catch it, or let go and "
                    "squeeze again as it reaches you." },
            } } },
        Option{ 0, "SelectionFeedback", "HighlightEnabled", "Highlight what you can grab",
            { {
                { "true", "On", "The object your hand would grab gets an outline." },
                { "false", "Off", "No outline." },
            } } },
        Option{ 0, "SelectionFeedback", "HighlightColor", "Highlight color",
            { {
                { "blue", "Blue", "A blue outline." },
                { "red", "Red", "A red outline." },
                { "orange", "Orange", "An orange outline." },
                { "white", "White", "A white outline." },
            } },
            {}, { "HighlightEnabled", "true" } },

        // ---- Holding weapons ----
        Option{ 1, "FiringHand", "DefaultHand", "Your shooting hand",
            { {
                { "right", "Right hand",
                    "Weapons equipped from favorites, PALM or other mods go to your right hand. A weapon you grab "
                    "goes to the hand that grabs it." },
                { "left", "Left hand",
                    "Weapons equipped from favorites, PALM or other mods go to your left hand. A weapon you grab "
                    "goes to the hand that grabs it." },
            } } },
        Option{ 1, "WeaponHandling", "AutoEquip", "Picking up a weapon",
            { {
                { "true", "Equip it when your hand takes the grip",
                    "A weapon you pick up is equipped as soon as your hand settles on its grip, no trigger needed." },
                { "false", "Pull the trigger to equip it",
                    "Hold a weapon you picked up and pull the trigger to equip it." },
            } } },
        Option{ 1, "WeaponHandling", "WeaponGrabMode", "Holding a weapon",
            { {
                { "1", "Toggle", "Press grab to take a grip and press it again to let go, with either hand." },
                { "2", "Toggle the shooting grip",
                    "Your shooting hand toggles its grip; your support hand holds on while you hold grab." },
                { "3", "Hold", "Each hand holds the weapon while you hold grab." },
            } } },
        Option{ 1, "WeaponHandling", "DetachEitherHand", "Which hand can let go",
            { {
                { "true", "Either hand", "Either hand can let go of the weapon; the other hand keeps holding it." },
                { "false", "Only the support hand",
                    "Only your support hand lets go; your shooting hand stays on the weapon." },
            } } },
        Option{ 1, "WeaponHandling", "WeaponReleaseMode", "When you let go",
            { {
                { "1", "It stays equipped", "The weapon stays in your hand, as in the game." },
                { "2", "It stays loose in your hand",
                    "Press grab again to drop or stash it, or pull the trigger to equip it again." },
                { "3", "It drops", "The weapon drops where it is, leaving with your hand's motion." },
            } },
            { "AutoEquip", "true", "3",
                "Equipping when your hand takes the grip always drops: a weapon kept in your hand would equip again "
                "at once." } },

        // ---- Aiming and shooting ----
        Option{ 2, "WeaponHandling", "WeaponFilteringPreset", "Steadying your aim",
            { {
                { "0", "Off", "Weapons and held objects follow every movement of your controllers, shake included." },
                { "1", "Responsive", "A little smoothing; follows your hands almost at once." },
                { "2", "Balanced", "Takes the shake out and still follows your hands quickly." },
                { "3", "Steady", "Steadier aim; follows your hands a little later." },
                { "4", "Heavy", "The steadiest aim; follows your hands the slowest." },
            } } },
        Option{ 2, "WeaponHandling", "GunstockMode", "Gunstock mode",
            { {
                { "true", "VR gunstock",
                    "For controllers in a VR gunstock: two-handed guns aim along the line from your shooting hand to "
                    "your support hand." },
                { "false", "Regular controllers", "For controllers in your hands: ROCK's normal two-handed aiming." },
            } } },
        Option{ 2, "WeaponRecoil", "ImmersiveRecoil", "Recoil",
            { {
                { "true", "Immersive", "Each kind of weapon kicks with its own strength, held in one hand or two, "
                                       "on either hand and in power armor." },
                { "false", "Game", "The game's own recoil, on the hand that fires." },
            } } },
        Option{ 2, "Scopes", "EnableImmersiveScopes", "Scopes",
            { {
                { "true", "ROCK's scopes",
                    "The scope view sits on the weapon; hold the scope button to look through it." },
                { "false", "Game's scopes", "The game's own scopes. Choose this if you use another scope mod." },
            } } },
        Option{ 2, "WeaponHandling", "AmmoCounterStyle", "Ammo counter",
            { {
                { "1", "Minimal", "Small white numbers beside the grip, shown while you look at them." },
                { "2", "Immersive", "The same place, in the game's HUD color and frame." },
                { "3", "Vanilla", "The game's counter on the back of your hand, always shown." },
            } } },

        // ---- Grenades and healing ----
        Option{ 3, "Grenades", "ImmersiveGrenades", "Grenades",
            { {
                { "true", "Throw by hand",
                    "Hold the grenade button to draw a grenade into your hand, then throw it yourself." },
                { "false", "Game throw", "The game's own hold and release throw." },
            } } },
        Option{ 3, "ImmersiveAid", "Enabled", "Stimpaks and Med-X",
            { {
                { "true", "Inject", "They sit in a syringe grip and inject when you hold them against your body." },
                { "false", "Like any item", "You grab them like any object and use them by bringing them to your mouth." },
            } } },

        // ---- Stick click and menus ----
        Option{ 4, "WeaponHandling", "BipodMode", "Bipod",
            { {
                { "true", "On", "Click the right stick while your weapon touches a surface to rest it there. "
                                "Otherwise the click opens RobCo PALM, or the game's Favorites menu without PALM." },
                { "false", "Off", "The right stick click is left to other mods." },
            } } },
        Option{ 4, "ControllerInput", "SuppressNativeVats", "VATS",
            { {
                { "false", "Allow VATS", "The VATS button opens VATS." },
                { "true", "Block VATS", "The VATS button no longer opens VATS; ROCK and other mods still use it." },
            } } },
    };

    // ---- finishing -----------------------------------------------------------------------------

    enum class Finish : std::uint8_t
    {
        // Saves the player's choices.
        Save,
        // Every option at ROCK's recommendation (its default), saved at once.
        Recommended,
        // Closes without saving the options.
        Close,
    };

    enum class BindingAction : std::uint8_t
    {
        None,
        // Install ROCK's SteamVR binding for `controller`.
        Install,
        // The player chose controllers that need no binding over detected ones ROCK offers one
        // for: stop offering it ([ControllerInput] OfferSteamVRBinding = false).
        StopOffering,
    };

    struct ControllerPlan
    {
        BindingAction action{ BindingAction::None };
        std::optional<sv::Controller> controller{};
    };

    /*
     * What finishing does with the controllers, the same for every Finish: the player's
     * choice decides; without one, the detected controllers do while ROCK still offers its
     * binding (OfferSteamVRBinding). ROCK installs only over SteamVR's default binding.
     */
    [[nodiscard]] constexpr ControllerPlan planControllers(std::optional<sv::Headset> chosen,
        std::optional<sv::Headset> detected, sv::Status index, sv::Status vive, bool offering) noexcept
    {
        const auto statusOf = [&](sv::Controller controller) {
            return controller == sv::Controller::Index ? index : vive;
        };
        const auto effective = chosen ? chosen : offering ? detected : std::nullopt;
        if (const auto controller = effective ? sv::controllerOf(*effective) : std::nullopt) {
            if (statusOf(*controller) == sv::Status::Offer) return { BindingAction::Install, controller };
            return {};
        }
        const auto detectedController = detected ? sv::controllerOf(*detected) : std::nullopt;
        if (chosen && offering && detectedController && statusOf(*detectedController) == sv::Status::Offer)
            return { BindingAction::StopOffering, std::nullopt };
        return {};
    }
}
