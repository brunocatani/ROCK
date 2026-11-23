// The Mods Config FO4 VR page (data/mod/ModsConfigFO4/Config/ROCK/spec.json)
// against ROCK's settings catalog: every setting it shows is an option of the
// file it edits, with ROCK's own type and compiled default; every ROCK.toml
// option is either on the page or deliberately left out, and every
// ROCK_Developer.toml option is on the Developer page. That page is generated
// from the catalog (--write-developer-page <spec.json>); the test fails while
// the spec holds another. No slider goes past what the loader accepts, and the
// loader enforces every clamp in the catalog. Hand-written help is not checked.
#include "RockConfig.h"
#include "config/RockSettings.h"
#include "config/SettingMetadata.h"
#include "physics-interaction/input/FunctionBindingText.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
    using namespace rock::config;
    using nlohmann::json;
    using nlohmann::ordered_json;

    // ROCK.toml options the page does not show, as "Table.Key".
    constexpr std::string_view kLeftOut[] = {
        // Written by the setup wizard to remember which edition it showed.
        "SetupWizard.ShownEdition",
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
     * A row with a storage and the spec file it edits.
     */
    struct ShownSetting
    {
        const json* row;
        std::string file;
    };

    /**
     * Collects every row with a storage, from the front page, its sections and every page and sub-page, each with
     * its file: the nearest defaultFile above it, as Mods Config resolves it.
     */
    void collectSettings(const json& node, const std::string& file, std::vector<ShownSetting>& settings)
    {
        if (node.is_array()) {
            for (const auto& item : node)
                collectSettings(item, file, settings);
            return;
        }
        if (!node.is_object())
            return;
        const auto own = node.contains("defaultFile") ? node.at("defaultFile").get<std::string>() : file;
        if (node.contains("storage"))
            settings.push_back({ &node, own });
        for (const char* child : { "content", "pages" }) {
            if (node.contains(child))
                collectSettings(node.at(child), own, settings);
        }
    }

    // ---- The generated Developer page ----

    constexpr std::string_view kDeveloperPageId = "developer";
    constexpr std::size_t kLineWidth = 120;

    /**
     * A row label from an option key: words split at each capital that starts one ("GrabLinearTau" is
     * "Grab Linear Tau", "HmdBack" is "Hmd Back"), keeping a run of capitals together ("VATS" stays a word).
     */
    std::string labelFromKey(std::string_view key)
    {
        std::string label;
        for (std::size_t index = 0; index < key.size(); ++index) {
            const char current = key[index];
            if (index > 0 && std::isupper(static_cast<unsigned char>(current))) {
                const char previous = key[index - 1];
                const bool nextLower = index + 1 < key.size() && std::islower(static_cast<unsigned char>(key[index + 1]));
                if (!std::isupper(static_cast<unsigned char>(previous)) || nextLower)
                    label += ' ';
            }
            label += current;
        }
        return label;
    }

    /**
     * A catalog description as one help line: its lines joined with spaces.
     */
    std::string helpLine(std::string_view description)
    {
        std::string help(description);
        std::ranges::replace(help, '\n', ' ');
        return help;
    }

    /**
     * A category heading without its "NN. " order prefix.
     */
    std::string_view categoryTitle(std::string_view category)
    {
        const auto dot = category.find(". ");
        return dot == std::string_view::npos ? category : category.substr(dot + 2);
    }

    /**
     * A float as the double its shortest text reads as (-6.85, not -6.849999904632568).
     */
    double shortest(float value)
    {
        std::array<char, 32> text{};
        const auto [end, error] = std::to_chars(text.data(), text.data() + text.size(), value);
        require(error == std::errc{}, "cannot format a compiled float");
        return std::stod(std::string(text.data(), end));
    }

    ordered_json jsonNumbers(const std::array<float, 3>& values)
    {
        ordered_json numbers = ordered_json::array();
        for (const float value : values)
            numbers.push_back(shortest(value));
        return numbers;
    }

    /**
     * A slider's range and step.
     */
    struct Slider
    {
        SettingRange range;
        double step = 0.0;
    };

    /**
     * A step of about a hundredth of the range, or a tenth of the default when that is finer, and at most the
     * given step; rounded down to 1, 2 or 5 times a power of ten, and at least 1 for a whole number.
     */
    double sliderStep(const SettingRange& range, double defaultValue, double maximumStep, bool whole)
    {
        double raw = (std::min)((range.maximum - range.minimum) / 100.0, maximumStep);
        if (std::abs(defaultValue) > 0.0)
            raw = (std::min)(raw, std::abs(defaultValue) / 10.0);
        const double magnitude = std::pow(10.0, std::floor(std::log10(raw) + 1e-9));
        double step = magnitude;
        for (const double factor : { 2.0, 5.0 }) {
            if (factor * magnitude <= raw * (1.0 + 1e-9))
                step = factor * magnitude;
        }
        return whole ? (std::max)(1.0, std::round(step)) : std::stod(std::format("{:.0e}", step));
    }

    /**
     * A developer number's slider: the catalog's slider range, or its clamp. A slider narrowing a clamp lies in it.
     */
    Slider sliderOf(const SettingMetadata& metadata, const SettingDefault& compiled)
    {
        const auto name = settingPath(metadata.table, metadata.key);
        const auto& [clamp, slider] = metadata.ranges;
        require(clamp || slider, std::format("{} needs clampTo or slideOver in the catalog (SettingMetadata.h)", name));
        const auto range = slider ? *slider : *clamp;
        require(!clamp || (range.minimum >= clamp->minimum && range.maximum <= clamp->maximum),
            std::format("{} has a slider outside its clamp", name));
        require(std::isfinite(range.minimum) && std::isfinite(range.maximum) && range.minimum < range.maximum,
            std::format("{} has no usable slider range", name));
        if (compiled.type == ValueType::Transform) {
            const auto transform = parseTransform(compiled.value);
            require(transform.has_value(), std::format("{} has a compiled transform ROCK can't read", name));
            return { range, sliderStep(range, 0.0, transform->rotation ? 1.0 : 0.1, false) };
        }
        const double defaultValue = std::stod(compiled.value);
        require(defaultValue >= range.minimum && defaultValue <= range.maximum,
            std::format("{} defaults to {}, outside its slider", name, compiled.value));
        return { range, sliderStep(range, defaultValue, std::numeric_limits<double>::max(),
                            compiled.type == ValueType::Integer) };
    }

    /**
     * A slider's min, max and step; whole-number ones as JSON integers.
     */
    ordered_json sliderOptions(const SettingMetadata& metadata, const SettingDefault& compiled)
    {
        const auto [range, step] = sliderOf(metadata, compiled);
        if (compiled.type == ValueType::Integer)
            return { { "min", std::llround(range.minimum) }, { "max", std::llround(range.maximum) },
                { "step", std::llround(step) } };
        return { { "min", range.minimum }, { "max", range.maximum }, { "step", step } };
    }

    /**
     * The values ROCK runs with when a file sets one number to a value, or every part of a transform to it, with
     * the options that limit others set out of their way.
     */
    rock::RockConfigValues valuesWith(const SettingDefault& setting, double value)
    {
        // Completed jobs are limited by the queued ones, the visual start distance by the touch and near-converge
        // distances, and each maximum by its minimum.
        LoadedValues values{ { "DebugOverlay.DebugMaxShapeQueuedJobs", "256" },
            { "GrabAcquisition.GrabTouchAcquireDistanceGameUnits", "0.5" },
            { "GrabAcquisition.GrabNearConvergeDistanceGameUnits", "100" } };
        for (const char* minimum : { "GrabContactEvidence.GrabPinchMinFingerGapGameUnits",
                 "GrabContactEvidence.GrabHandLerpTimeMin", "GrabContactEvidence.GrabHandLerpMinDistance",
                 "GrabContactEvidence.GrabHandReturnTimeMin", "GrabContactEvidence.GrabHandReturnMinDistance",
                 "GrabContactEvidence.GrabHandReturnMinAngleDegrees",
                 "WeaponCollisionAndSupport.WeaponSupportGripHandLerpTimeMin",
                 "WeaponCollisionAndSupport.WeaponSupportGripHandLerpMinDistance", "Haptics.GrabHapticBaseIntensity",
                 "Haptics.HeldImpactHapticBaseIntensity", "Haptics.ShoulderStashCandidateHapticBaseIntensity",
                 "Haptics.MouthConsumeCandidateHapticBaseIntensity" })
            values[minimum] = "0";
        const auto part = static_cast<float>(value);
        auto& text = values[settingPath(setting.table, setting.key)];
        if (setting.type == ValueType::Integer)
            text = std::to_string(std::llround(value));
        else if (setting.type != ValueType::Transform)
            text = std::format("{}", part);
        else if (parseTransform(setting.value)->rotation)
            text = formatTransform(Transform{ .rotation = std::array{ part, part, part } });
        else
            text = formatTransform(Transform{ .position = std::array{ part, part, part } });
        return rock::RockConfig::parseValues(values);
    }

    /**
     * Whether ROCK takes every value a slider offers: a range wider than the loader accepts has its last step
     * clamped or reset away, so both ends must run differently from one step inside them.
     */
    bool sliderAccepted(const SettingDefault& setting, const SettingRange& range, double step)
    {
        return !(valuesWith(setting, range.minimum) == valuesWith(setting, range.minimum + step)) &&
               !(valuesWith(setting, range.maximum) == valuesWith(setting, range.maximum - step));
    }

    /**
     * Checks that the loader enforces an option's catalog clamp: past either end runs as the end.
     */
    void checkClampEnforced(const SettingDefault& setting, const SettingRange& clamp)
    {
        const auto past = clamp.maximum - clamp.minimum;
        require(past > 0.0 && valuesWith(setting, clamp.minimum - past) == valuesWith(setting, clamp.minimum) &&
                    valuesWith(setting, clamp.maximum + past) == valuesWith(setting, clamp.maximum),
            std::format("{} has a catalog clamp the loader does not enforce", settingPath(setting.table, setting.key)));
    }

    /**
     * A hand-written row's slider, or nothing for another control.
     */
    std::optional<Slider> specSlider(const json& row)
    {
        if (row.value("type", "") != "slider")
            return std::nullopt;
        const auto& options = row.at("valueOptions");
        return Slider{ { options.at("min").get<double>(), options.at("max").get<double>() },
            options.at("step").get<double>() };
    }

    /**
     * One generated row: its control from ROCK's type, its label from the key and its help from the catalog. A
     * number is a slider over its range; a transform slides only the part its default uses.
     */
    ordered_json developerRow(const SettingMetadata& metadata, const SettingDefault& compiled)
    {
        const auto name = settingPath(metadata.table, metadata.key);
        const bool numeric = compiled.type == ValueType::Integer || compiled.type == ValueType::Float ||
                             compiled.type == ValueType::Transform;
        require(numeric || (!metadata.ranges.clamp && !metadata.ranges.slider),
            std::format("{} has a range but is not a number", name));
        ordered_json row;
        switch (compiled.type) {
        case ValueType::Boolean:
            row = { { "type", "switcher" }, { "valueType", "bool" } };
            break;
        case ValueType::Integer:
            row = { { "type", "slider" }, { "valueType", "int" } };
            break;
        case ValueType::Float:
            row = { { "type", "slider" }, { "valueType", "float" } };
            break;
        case ValueType::String:
            row = { { "type", "textinput" }, { "valueType", "string" } };
            break;
        case ValueType::Transform:
            row = { { "type", "cf.transform" }, { "valueType", "string" } };
            break;
        default:
            throw std::runtime_error(std::format("{} has a type the Developer page can't show", name));
        }
        row["text"] = labelFromKey(metadata.key);
        if (!metadata.description.empty())
            row["help"] = helpLine(metadata.description);
        switch (compiled.type) {
        case ValueType::Boolean:
            row["default"] = compiled.value == "true";
            break;
        case ValueType::Integer:
            row["default"] = std::stoll(compiled.value);
            row["valueOptions"] = sliderOptions(metadata, compiled);
            break;
        case ValueType::Float:
            row["default"] = std::stod(compiled.value);
            row["valueOptions"] = sliderOptions(metadata, compiled);
            break;
        case ValueType::String:
            row["default"] = compiled.value;
            break;
        default: {
            const auto transform = parseTransform(compiled.value);
            require(transform.has_value(), std::format("{} has a compiled transform ROCK can't read", name));
            ordered_json value = ordered_json::object();
            if (transform->position)
                value["pos"] = jsonNumbers(*transform->position);
            if (transform->rotation)
                value["rot"] = jsonNumbers(*transform->rotation);
            row["default"] = value;
            const auto slider = sliderOptions(metadata, compiled);
            row["valueOptions"] = { { "position", transform->rotation ? ordered_json(false) : slider },
                { "rotation", transform->rotation ? slider : ordered_json(false) }, { "scale", false } };
            break;
        }
        }
        row["storage"] = { { "path", ordered_json::array({ std::string(metadata.table), std::string(metadata.key) }) } };
        return row;
    }

    /**
     * The Developer page: shown while Show Developer Pages is on, editing ROCK_Developer.toml, with a sub-page per
     * category holding every developer option in catalog order.
     */
    ordered_json developerPage(std::span<const SettingDefault> defaults)
    {
        ordered_json pages = ordered_json::array();
        std::string_view category;
        for (const auto& metadata : kSettingMetadata) {
            if (metadata.group != Group::Developer)
                continue;
            if (metadata.category != category) {
                category = metadata.category;
                ordered_json categoryPage = ordered_json::object();
                categoryPage["pageDisplayName"] = std::string(categoryTitle(category));
                categoryPage["content"] = ordered_json::array();
                pages.push_back(std::move(categoryPage));
            }
            const auto compiled = std::ranges::find_if(defaults,
                [&](const SettingDefault& entry) { return entry.table == metadata.table && entry.key == metadata.key; });
            require(compiled != defaults.end(),
                std::format("{} has no compiled default", settingPath(metadata.table, metadata.key)));
            pages.back()["content"].push_back(developerRow(metadata, *compiled));
        }
        ordered_json intro = ordered_json::object();
        intro["type"] = "text";
        intro["text"] = "Developer options tune ROCK and turn on its debug drawing and logs. ROCK_Developer.toml keeps "
                        "only the options you change: Reset removes one, and the file goes when none remain. Changes "
                        "show in the game as you make them.";
        ordered_json page = ordered_json::object();
        page["pageDisplayName"] = "Developer";
        page["id"] = std::string(kDeveloperPageId);
        page["help"] = "ROCK's developer options: tuning, debug drawing and logs.";
        page["defaultFile"] = "developer";
        page["visibleWhen"] = { { "setting", "showDeveloperPages" }, { "equals", true } };
        page["content"] = ordered_json::array({ intro });
        page["pages"] = std::move(pages);
        return page;
    }

    /**
     * A JSON value on one line, in the spec's hand-written style: "{ "key": value }" and "[a, b]".
     */
    std::string inlineJson(const ordered_json& value)
    {
        std::string text;
        if (value.is_object()) {
            if (value.empty())
                return "{}";
            for (const auto& [key, part] : value.items())
                text += std::format("{}{}: {}", text.empty() ? "{ " : ", ", json(key).dump(), inlineJson(part));
            return text + " }";
        }
        if (value.is_array()) {
            for (const auto& part : value)
                text += std::format("{}{}", text.empty() ? "[" : ", ", inlineJson(part));
            return text.empty() ? "[]" : text + "]";
        }
        return value.dump();
    }

    /**
     * A JSON value in the spec's layout: on one line when it fits, otherwise a member per line, indented by two.
     */
    std::string layoutJson(const ordered_json& value, std::size_t indent, std::size_t prefixWidth = 0)
    {
        auto line = inlineJson(value);
        if (!value.is_structured() || indent + prefixWidth + line.size() <= kLineWidth)
            return line;
        const std::string inner(indent + 2, ' ');
        std::string text = value.is_object() ? "{\n" : "[\n";
        bool first = true;
        if (value.is_object()) {
            for (const auto& [key, part] : value.items()) {
                const auto name = json(key).dump() + ": ";
                text += std::format("{}{}{}{}", first ? "" : ",\n", inner, name, layoutJson(part, indent + 2, name.size()));
                first = false;
            }
        } else {
            for (const auto& part : value) {
                text += std::format("{}{}{}", first ? "" : ",\n", inner, layoutJson(part, indent + 2));
                first = false;
            }
        }
        return text + "\n" + std::string(indent, ' ') + (value.is_object() ? "}" : "]");
    }

    /**
     * The spec's text with the Developer page as its last page: the page already there, and anything after it,
     * replaced, or the page appended.
     */
    std::string withDeveloperPage(const std::string& spec, const ordered_json& page)
    {
        const auto marker = spec.find(std::format("\"id\": \"{}\"", kDeveloperPageId));
        std::string prefix;
        if (marker != std::string::npos) {
            const auto start = spec.rfind("\n    {", marker);
            require(start != std::string::npos, "the Developer page is not a page of the spec");
            prefix = spec.substr(0, start);
        } else {
            const auto end = spec.rfind("\n  ]\n}");
            require(end != std::string::npos, "the spec does not end with its pages");
            prefix = spec.substr(0, end) + ",";
        }
        return prefix + "\n    " + layoutJson(page, 4) + "\n  ]\n}\n";
    }

    std::string withoutCarriageReturns(std::string text)
    {
        std::erase(text, '\r');
        return text;
    }

    std::string readText(const std::string& path)
    {
        std::ifstream input(path, std::ios::binary);
        require(input.good(), std::format("cannot read {}", path));
        return withoutCarriageReturns({ std::istreambuf_iterator<char>(input), {} });
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
     * The spec's valueType for one of ROCK's value types; Float3 has none. A transform
     * (cf.transform) or binding (cf.inputBinding) row stores the table's text.
     */
    std::string_view specType(ValueType type)
    {
        switch (type) {
        case ValueType::Boolean: return "bool";
        case ValueType::Integer: return "int";
        case ValueType::Float: return "float";
        case ValueType::String: return "string";
        case ValueType::Transform: return "string";
        case ValueType::Table: return "string";
        default: return "";
        }
    }

    /**
     * Whether a cf.transform default (a JSON table) means ROCK's compiled transform. A part either side leaves
     * out reads as no change: no move, no rotation, scale 1.
     */
    bool sameTransform(const json& value, const Transform& compiled)
    {
        if (!value.is_object())
            return false;
        const auto sameThree = [&](const char* key, const std::optional<std::array<float, 3>>& part) {
            const auto expected = part.value_or(std::array<float, 3>{});
            if (!value.contains(key))
                return std::ranges::all_of(expected, [](float component) { return nearlyEqual(component, 0.0); });
            const auto& array = value.at(key);
            if (!array.is_array() || array.size() != 3)
                return false;
            for (std::size_t index = 0; index < 3; ++index) {
                if (!array[index].is_number() || !nearlyEqual(array[index].get<double>(), expected[index]))
                    return false;
            }
            return true;
        };
        const auto scale = value.contains("scale") ? value.at("scale") : json(1.0);
        return sameThree("pos", compiled.position) && sameThree("rot", compiled.rotation) && scale.is_number() &&
               nearlyEqual(scale.get<double>(), compiled.scale.value_or(1.0f));
    }

    /**
     * A JSON object of strings, numbers and nested objects as a TOML inline table, as Mods Config writes a
     * cf.inputBinding default to the file.
     */
    std::string tomlTable(const json& value)
    {
        std::string text = "{ ";
        bool first = true;
        for (const auto& [key, part] : value.items()) {
            text += std::format("{}{} = ", first ? "" : ", ", key);
            first = false;
            if (part.is_object())
                text += tomlTable(part);
            else
                text += part.dump();
        }
        return text + " }";
    }

    /**
     * Whether a cf.inputBinding default (a JSON table) is the binding ROCK compiles, both read by ROCK's parser:
     * the same hands, type, buttons and time.
     */
    bool sameBinding(const std::string& name, const json& value, const std::string& compiled)
    {
        namespace fb = rock::function_binding_policy;
        namespace binding_text = rock::function_binding_text;
        require(value.is_object(), std::format("{} has a default that is not a table", name));
        std::optional<fb::Function> function;
        for (std::size_t index = 0; index < fb::kFunctionCount; ++index) {
            if (name == std::format("Bindings.{}", fb::functionKey(static_cast<fb::Function>(index))))
                function = static_cast<fb::Function>(index);
        }
        require(function.has_value(), std::format("{} is a binding table but no function's key", name));
        const auto spec = binding_text::parseBindingTable(*function, tomlTable(value));
        require(spec.has_value(), std::format("{} has a default ROCK can't read: {}", name, spec ? "" : spec.error()));
        require(spec->warnings.empty(), std::format("{} has a default ROCK brings into range", name));
        const auto rock = binding_text::parseBindingTable(*function, compiled);
        require(rock.has_value(), std::format("{} has a compiled default ROCK can't read", name));
        return spec->value.binding == rock->value.binding && nearlyEqual(spec->value.seconds, rock->value.seconds);
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
        case ValueType::Table:
            require(setting.value("type", "") == "cf.inputBinding",
                std::format("{} is a binding table but not a cf.inputBinding row", name));
            same = sameBinding(name, value, compiled.value);
            break;
        case ValueType::Transform: {
            require(setting.value("type", "") == "cf.transform", std::format("{} is a transform but not a cf.transform row", name));
            const auto transform = parseTransform(compiled.value);
            same = transform && sameTransform(value, *transform);
            break;
        }
        default:
            break;
        }
        require(same, std::format("{} defaults to {} in the spec, {} in ROCK", name, value.dump(), compiled.value));
    }
}

int main(int argc, char** argv)
{
    try {
        const auto defaults = rock::RockConfig::compiledDefaults();
        const auto page = developerPage(defaults);
        if (argc == 3 && std::string_view(argv[1]) == "--write-developer-page") {
            const auto text = withDeveloperPage(readText(argv[2]), page);
            std::ofstream output(argv[2], std::ios::binary | std::ios::trunc);
            output << text;
            require(output.good(), std::format("cannot write {}", argv[2]));
            return 0;
        }

        const auto text = readText(ROCK_MODS_CONFIG_SPEC);
        require(text == withDeveloperPage(text, page),
            "the spec's Developer page differs from the catalog; regenerate it with --write-developer-page <spec.json>");
        const json spec = json::parse(text);
        require(spec.value("modName", "") == "ROCK", "the spec's modName is not ROCK");
        const auto& files = spec.at("files");
        require(files.at("config").value("path", "") == "Mods_Config/ROCK/ROCK.toml" &&
                    files.at("developer").value("path", "") == "Mods_Config/ROCK/ROCK_Developer.toml" &&
                    spec.value("defaultFile", "") == "config",
            "the spec's files are not ROCK.toml (the default) and ROCK_Developer.toml");

        std::vector<ShownSetting> settings;
        collectSettings(spec.at("content"), "config", settings);
        // References into spec: value() would return a copy and leave the pointers dangling.
        if (spec.contains("pages"))
            collectSettings(spec.at("pages"), "config", settings);

        std::set<std::string> shown;
        std::string tooWide;
        for (const auto& [setting, file] : settings) {
            const auto name = settingName(setting->at("storage"));
            require(shown.insert(name).second, std::format("{} is on the page twice", name));
            const auto metadata = std::find_if(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                [&](const SettingMetadata& entry) { return settingPath(entry.table, entry.key) == name; });
            require(metadata != std::end(kSettingMetadata), std::format("{} is not a ROCK setting", name));
            const auto group = file == "developer" ? Group::Developer : Group::Player;
            require(metadata->group == group,
                std::format("{} is shown for {}, but ROCK keeps it in the other file", name,
                    group == Group::Developer ? "ROCK_Developer.toml" : "ROCK.toml"));
            const auto compiled = std::find_if(defaults.begin(), defaults.end(),
                [&](const SettingDefault& entry) { return settingPath(entry.table, entry.key) == name; });
            require(compiled != defaults.end(), std::format("{} has no compiled default", name));
            checkDefault(name, *setting, *compiled);
            if (metadata->ranges.clamp)
                checkClampEnforced(*compiled, *metadata->ranges.clamp);
            if (const auto slider = specSlider(*setting); slider && !sliderAccepted(*compiled, slider->range, slider->step))
                tooWide += " " + name;
        }
        require(tooWide.empty(), std::format("sliders wider than ROCK accepts:{}", tooWide));

        for (const auto leftOut : kLeftOut) {
            require(!shown.contains(std::string(leftOut)), std::format("{} is left out but on the page", leftOut));
            require(std::any_of(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                        [&](const SettingMetadata& entry) {
                            return entry.group == Group::Player &&
                                   settingPath(entry.table, entry.key) == leftOut;
                        }),
                std::format("{} is left out but is not a ROCK.toml setting", leftOut));
        }
        for (const auto& entry : kSettingMetadata) {
            const auto name = settingPath(entry.table, entry.key);
            if (entry.group == Group::Developer) {
                require(shown.contains(name), std::format("{} is a developer setting missing from the Developer page", name));
                continue;
            }
            require(shown.contains(name) || std::ranges::find(kLeftOut, std::string_view(name)) != std::end(kLeftOut),
                std::format("{} is a new ROCK.toml setting: add it to the Mods Config page or to kLeftOut", name));
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
