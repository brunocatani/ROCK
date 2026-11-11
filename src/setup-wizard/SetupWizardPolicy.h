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
        Page{ "Stick click and menus", "What the right stick click and the VATS button do." },
    };

    struct Choice
    {
        std::string_view value;
        std::string_view label;
        std::string_view detail;
    };

    inline constexpr std::size_t kMaxChoices = 5;

    struct Option
    {
        std::size_t page;
        std::string_view section;
        std::string_view key;
        std::string_view title;
        std::array<Choice, kMaxChoices> choices;

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
        Option{ 0, "Grabbing", "FarGrabMode", "Grabbing things far away",
            { {
                { "1", "Instant pull", "Point at an object and press grab: it flies straight into your hand." },
                { "2", "Flick to pull (HIGGS style)",
                    "Hold grab to lock on, flick your hand toward you, and keep holding while it flies in. Let go "
                    "before it arrives to cancel." },
            } } },
        Option{ 0, "SelectionFeedback", "HighlightEnabled", "Highlight what you can grab",
            { {
                { "true", "On", "The object your hand would grab gets an outline." },
                { "false", "Off", "No outline." },
            } } },

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
        Option{ 1, "WeaponHandling", "WeaponGrabMode", "Holding a weapon",
            { {
                { "1", "Toggle", "Press grab to take a grip and press it again to let go, with either hand." },
                { "2", "Toggle the shooting grip",
                    "Your shooting hand toggles its grip; your support hand holds on while you hold grab." },
                { "3", "Hold", "Each hand holds the weapon while you hold grab." },
            } } },
        Option{ 1, "WeaponHandling", "WeaponEquipMode", "Letting go of a weapon",
            { {
                { "1", "Keep it", "The weapon stays in your hand when you let go, as in the game. Pull the trigger on "
                               "a weapon you picked up to equip it." },
                { "2", "Drop on a second press",
                    "Letting go leaves the weapon loose in your hand: press grab again to drop or stash it, or pull "
                    "the trigger to equip it again." },
                { "3", "Drop it", "Letting go drops the weapon where it is. Pull the trigger on a weapon you picked "
                                  "up to equip it." },
                { "4", "Grab to equip", "A weapon you pick up is equipped as soon as your hand settles on its grip, "
                                        "no trigger needed. Letting go drops it." },
            } } },
        Option{ 1, "WeaponHandling", "DetachEitherHand", "Letting go with your shooting hand",
            { {
                { "true", "Either hand", "Either hand can let go of the weapon; the other hand keeps holding it." },
                { "false", "Support hand only", "Only your support hand lets go; your shooting hand stays on the weapon." },
            } } },

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
                { "true", "On", "Click the right stick while your weapon touches a surface to rest it there." },
                { "false", "Off", "The right stick click is left to other mods." },
            } } },
        Option{ 4, "ControllerInput", "VanillaFavoritesMenu", "Favorites menu",
            { {
                { "true", "Game's Favorites",
                    "The right stick click opens the game's Favorites menu. With RobCo PALM installed it opens PALM "
                    "instead." },
                { "false", "No Favorites", "ROCK blocks the Favorites menu; the stick click is only for the bipod." },
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
