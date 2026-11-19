// The Mods Config FO4 VR page (data/mod/ModsConfigFO4/Config/ROCK/spec.json)
// against ROCK's settings catalog: every setting it shows is a ROCK.toml option
// with ROCK's own type and compiled default, and every ROCK.toml option is
// either on the page or deliberately left out. Ranges and help are not checked.
#include "RockConfig.h"
#include "config/RockSettings.h"
#include "config/SettingMetadata.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using namespace rock::config;
    using nlohmann::json;

    // ROCK.toml options the page does not show, as "Table.Key".
    constexpr std::string_view kLeftOut[] = {
        // Written by the setup wizard to remember which edition it showed.
        "SetupWizard.ShownEdition",
        // Shows the ROCK Dev page in RPS UI Framework's settings menu only.
        "Developer.EnableDeveloperSettings",
        // Transforms, not on the page yet.
        "FiringHand.Left.AimOffset",
        "FiringHand.Left.GripOffset",
        "FiringHand.Right.SupportGripOffset",
    };

    void require(bool condition, const std::string& message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    /**
     * Compares two numbers with a tolerance relative to their size, as the spec's
     * JSON numbers and ROCK's float defaults round differently.
     */
    bool nearlyEqual(double left, double right)
    {
        return std::abs(left - right) <= 1e-6 * (std::max)({ 1.0, std::abs(left), std::abs(right) });
    }

    /**
     * Collects every row with a storage, from the front page, its sections and
     * every page and sub-page.
     */
    void collectSettings(const json& node, std::vector<const json*>& settings)
    {
        if (node.is_array()) {
            for (const auto& item : node)
                collectSettings(item, settings);
            return;
        }
        if (!node.is_object())
            return;
        if (node.contains("storage"))
            settings.push_back(&node);
        for (const char* child : { "content", "pages" }) {
            if (node.contains(child))
                collectSettings(node.at(child), settings);
        }
    }

    /**
     * Maps a spec storage path to ROCK's "Table.Key" name: the TOML tables joined
     * with dots, as the catalog names nested tables, then the key.
     */
    std::string settingName(const json& storage)
    {
        require(!storage.contains("file"), "a setting names a file; the page edits ROCK.toml only");
        require(storage.contains("path") && storage.at("path").is_array() && storage.at("path").size() >= 2,
            std::format("a setting's storage is not a TOML table and key: {}", storage.dump()));
        std::string name;
        for (const auto& part : storage.at("path")) {
            if (!name.empty())
                name += '.';
            name += part.get<std::string>();
        }
        return name;
    }

    /**
     * The spec's valueType for one of ROCK's value types; Float3 has none.
     */
    std::string_view specType(ValueType type)
    {
        switch (type) {
        case ValueType::Boolean: return "bool";
        case ValueType::Integer: return "int";
        case ValueType::Float: return "float";
        case ValueType::String: return "string";
        default: return "";
        }
    }

    /**
     * Checks one row's valueType and default against ROCK's compiled default, the
     * value Reset must set.
     */
    void checkDefault(const std::string& name, const json& setting, const SettingDefault& compiled)
    {
        const auto type = specType(compiled.type);
        require(!type.empty(), std::format("{} has a type the page can't show", name));
        require(setting.value("valueType", "") == type,
            std::format("{} is valueType {} in the spec, {} in ROCK", name, setting.value("valueType", ""), type));
        require(setting.contains("default"), std::format("{} has no default", name));
        const auto& value = setting.at("default");
        bool same = false;
        switch (compiled.type) {
        case ValueType::Boolean:
            same = value.is_boolean() && value.get<bool>() == (compiled.value == "true");
            break;
        case ValueType::Integer:
            same = value.is_number_integer() && value.get<long long>() == std::stoll(compiled.value);
            break;
        case ValueType::Float:
            same = value.is_number() && nearlyEqual(value.get<double>(), std::stod(compiled.value));
            break;
        case ValueType::String:
            same = value.is_string() && value.get<std::string>() == compiled.value;
            break;
        default:
            break;
        }
        require(same, std::format("{} defaults to {} in the spec, {} in ROCK", name, value.dump(), compiled.value));
    }
}

int main()
{
    try {
        std::ifstream input(ROCK_MODS_CONFIG_SPEC);
        require(input.good(), "cannot read " ROCK_MODS_CONFIG_SPEC);
        const json spec = json::parse(input);
        require(spec.value("modName", "") == "ROCK", "the spec's modName is not ROCK");

        std::vector<const json*> settings;
        collectSettings(spec.at("content"), settings);
        // References into spec: value() would return a copy and leave the pointers dangling.
        if (spec.contains("pages"))
            collectSettings(spec.at("pages"), settings);

        const auto defaults = rock::RockConfig::compiledDefaults();
        std::set<std::string> shown;
        for (const auto* setting : settings) {
            const auto name = settingName(setting->at("storage"));
            require(shown.insert(name).second, std::format("{} is on the page twice", name));
            const auto metadata = std::find_if(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                [&](const SettingMetadata& entry) { return settingPath(entry.table, entry.key) == name; });
            require(metadata != std::end(kSettingMetadata), std::format("{} is not a ROCK setting", name));
            require(metadata->group == rock::api::configuration::Group::Consumer,
                std::format("{} is a developer setting, kept in ROCK_Developer.toml", name));
            const auto compiled = std::find_if(defaults.begin(), defaults.end(),
                [&](const SettingDefault& entry) { return settingPath(entry.table, entry.key) == name; });
            require(compiled != defaults.end(), std::format("{} has no compiled default", name));
            checkDefault(name, *setting, *compiled);
        }

        for (const auto leftOut : kLeftOut) {
            require(!shown.contains(std::string(leftOut)), std::format("{} is left out but on the page", leftOut));
            require(std::any_of(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                        [&](const SettingMetadata& entry) {
                            return entry.group == rock::api::configuration::Group::Consumer &&
                                   settingPath(entry.table, entry.key) == leftOut;
                        }),
                std::format("{} is left out but is not a ROCK.toml setting", leftOut));
        }
        for (const auto& entry : kSettingMetadata) {
            if (entry.group != rock::api::configuration::Group::Consumer)
                continue;
            const auto name = settingPath(entry.table, entry.key);
            require(shown.contains(name) || std::ranges::find(kLeftOut, std::string_view(name)) != std::end(kLeftOut),
                std::format("{} is a new ROCK.toml setting: add it to the Mods Config page or to kLeftOut", name));
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
