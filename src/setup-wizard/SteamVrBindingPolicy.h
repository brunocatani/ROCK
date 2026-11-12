#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

/*
 * Whether ROCK offers its SteamVR binding for Fallout 4 VR, decided from what SteamVR
 * reports, with no game memory read.
 *
 * SteamVR's default legacy binding for Index controllers and Vive wands leaves the game in
 * its wand mode: Index A and X arrive as grip and the wands have no A/X at all (bench
 * button matrix, 13 of 17 probes match the Touch reference). ROCK's templates in
 * steamvr_bindings/ present those controllers as Touch controllers (17 of 17). Steam Frame,
 * Pico, Focus 3 and Touch match without a binding, and OpenComposite does not use SteamVR
 * bindings at all.
 *
 * SteamVR keeps each app's chosen legacy binding in steamvr.vrsettings, section = the app
 * key, key = "<controller type>_250820_CurrentURL_legacy" (250820 is SteamVR's own Steam app
 * ID; vrserver builds the key at runtime). An empty value is the default binding. Anything
 * else is a choice the player made in SteamVR and is never replaced.
 */
namespace rock::steamvr_binding_policy
{
    enum class Controller : std::uint8_t
    {
        Index,
        ViveWand,
    };

    enum class Status : std::uint8_t
    {
        // Not SteamVR, SteamVR unreachable, or a controller that needs no binding.
        NotApplicable,
        // SteamVR's default binding on a controller ROCK has a binding for.
        Offer,
        // The setting names ROCK's binding file.
        RockActive,
        // The player chose another binding in SteamVR.
        PlayersOwn,
    };

    struct ControllerInfo
    {
        Controller controller;
        // SteamVR's Prop_ControllerType_String.
        std::string_view controllerType;
        // The template's name in steamvr_bindings/, kept for the installed copy.
        std::string_view fileName;
        std::string_view displayName;
    };

    [[nodiscard]] constexpr std::optional<ControllerInfo> controllerFor(std::string_view controllerType) noexcept
    {
        if (controllerType == "knuckles")
            return ControllerInfo{ Controller::Index, "knuckles", "legacy_bindings_knuckles_rock.json", "Index controllers" };
        if (controllerType == "vive_controller")
            return ControllerInfo{ Controller::ViveWand, "vive_controller", "legacy_bindings_vive_controller_rock.json",
                "Vive wands" };
        return std::nullopt;
    }

    [[nodiscard]] inline std::string settingKey(std::string_view controllerType)
    {
        std::string key{ controllerType };
        key += "_250820_CurrentURL_legacy";
        return key;
    }

    [[nodiscard]] constexpr char lower(char c) noexcept { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; }

    // True when url names fileName as its last path part, whatever folder it is in.
    [[nodiscard]] constexpr bool namesFile(std::string_view url, std::string_view fileName) noexcept
    {
        if (fileName.empty() || url.size() <= fileName.size()) return false;
        const auto separator = url[url.size() - fileName.size() - 1];
        if (separator != '/' && separator != '\\') return false;
        const auto tail = url.substr(url.size() - fileName.size());
        for (std::size_t i = 0; i < tail.size(); ++i)
            if (lower(tail[i]) != lower(fileName[i])) return false;
        return true;
    }

    [[nodiscard]] constexpr Status classify(bool steamVr, const std::optional<ControllerInfo>& controller,
        std::string_view currentUrl) noexcept
    {
        if (!steamVr || !controller) return Status::NotApplicable;
        if (currentUrl.empty()) return Status::Offer;
        if (namesFile(currentUrl, controller->fileName)) return Status::RockActive;
        return Status::PlayersOwn;
    }

    // The controllers a player can name in the setup wizard.
    enum class Headset : std::uint8_t
    {
        Index,
        ViveWands,
        // Touch, Pico, Focus 3, Steam Frame: every button reaches the game without a binding.
        NoBindingNeeded,
        // Controllers ROCK has no binding for.
        Other,
    };

    [[nodiscard]] constexpr std::optional<Controller> controllerOf(Headset headset) noexcept
    {
        if (headset == Headset::Index) return Controller::Index;
        if (headset == Headset::ViveWands) return Controller::ViveWand;
        return std::nullopt;
    }

    /*
     * The controllers SteamVR reports, from Prop_ControllerType_String and Fallout 4 VR's
     * setting for each controller ROCK has a binding for. ROCK's binding presents Index
     * controllers and wands as Touch, so SteamVR then reports oculus_touch: the controller
     * whose setting names ROCK's file is the one in the player's hands.
     */
    [[nodiscard]] constexpr std::optional<Headset> detectHeadset(bool steamVr, std::string_view reportedType, Status index,
        Status vive) noexcept
    {
        if (!steamVr || reportedType.empty()) return std::nullopt;
        if (reportedType == "knuckles") return Headset::Index;
        if (reportedType == "vive_controller") return Headset::ViveWands;
        if (reportedType == "oculus_touch") {
            if (index == Status::RockActive) return Headset::Index;
            if (vive == Status::RockActive) return Headset::ViveWands;
            return Headset::NoBindingNeeded;
        }
        if (reportedType == "frame_controller") return Headset::NoBindingNeeded;
        return Headset::Other;
    }

    // The file:/// URL SteamVR accepts for a Windows path: forward slashes only.
    [[nodiscard]] inline std::string fileUrl(std::string_view windowsPath)
    {
        std::string url{ "file:///" };
        for (const char c : windowsPath) url += c == '\\' ? '/' : c;
        return url;
    }
}
