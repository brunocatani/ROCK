#include "config/RockSettings.h"
#include "config/SettingMetadata.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/FunctionBindingText.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <format>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rock::config
{
    namespace
    {
        namespace fb = function_binding_policy;
        namespace binding_text = function_binding_text;
        using RPS::Settings::ConversionSource;
        using RPS::Settings::LegacyKey;
        using RPS::Settings::LegacyKind;

        constexpr LegacyKey kLegacyKeys[] = {
#include "config/LegacyIniMigrationTable.inl"
        };

        // Keys ROCK.toml held before a later version renamed them. Frozen like
        // the INI table: a row leaves only with its option. The firing-hand
        // offsets became position-only transforms, for Mods Config's transform editor.
        constexpr RPS::Settings::RenamedKey kRenamedKeys[] = {
            { "FiringHand.Left", "AimOffsetGameUnits", "FiringHand.Left", "AimOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "FiringHand.Left", "GripOffsetGameUnits", "FiringHand.Left", "GripOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "FiringHand.Right", "SupportGripOffsetGameUnits", "FiringHand.Right", "SupportGripOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
        };

        // The hold times ROCK.toml held before each binding carried its own duration.
        struct RetiredTime
        {
            std::string_view table;
            std::string_view key;
            float fallback;
            float minimum;
            float maximum;
        };
        constexpr RetiredTime kRetiredHold{ "Bindings", "HoldSeconds", 0.35f, 0.1f, 2.0f };
        constexpr RetiredTime kRetiredDoubleTap{ "Bindings", "DoubleTapSeconds", 0.30f, 0.15f, 0.6f };
        constexpr RetiredTime kRetiredScopeHold{ "Scopes", "ManualScopeHoldSeconds", 0.30f, 0.05f, 2.0f };
        constexpr RetiredTime kRetiredPauseHold{ "ControllerInput", "PipboyPauseHoldSeconds", 0.35f, 0.15f, 2.0f };
        constexpr std::array kRetiredTimes{ kRetiredHold, kRetiredDoubleTap, kRetiredScopeHold, kRetiredPauseHold };

        /**
         * A retired hold time in its old range, or its old default when the source has none.
         */
        float retiredSeconds(const ConversionSource& source, const RetiredTime& time)
        {
            const auto value = source.value(time.table, time.key, ValueType::Float);
            float parsed = time.fallback;
            if (value)
                std::from_chars(value->data(), value->data() + value->size(), parsed);
            return std::clamp(parsed, time.minimum, time.maximum);
        }

        /**
         * One binding's conversion to its CommonFramework table: from the old string form, and with the retired hold
         * times as its duration. A file holding neither, or already holding the table without an old string, is left
         * as it is. What the table cannot say is logged and becomes the default.
         */
        template <fb::Function F>
        std::optional<std::string> convertBinding(const ConversionSource& source)
        {
            const auto key = fb::functionKey(F);
            const auto old = source.value("Bindings", key, ValueType::String);
            const bool retired = std::any_of(kRetiredTimes.begin(), kRetiredTimes.end(),
                [&](const RetiredTime& time) { return source.value(time.table, time.key, ValueType::Float).has_value(); });
            if (!old && (!retired || source.value("Bindings", key, ValueType::Table)))
                return std::nullopt;
            const binding_text::LegacyTimes times{
                .hold = retiredSeconds(source, kRetiredHold),
                .scopeHold = retiredSeconds(source, kRetiredScopeHold),
                .pauseHold = retiredSeconds(source, kRetiredPauseHold),
                .doubleTap = retiredSeconds(source, kRetiredDoubleTap),
            };
            const auto converted =
                binding_text::fromLegacyBinding(F, old ? std::optional<std::string_view>(*old) : std::nullopt, times);
            for ([[maybe_unused]] const auto& warning : converted.warnings)
                ROCK_LOG_WARN(Config, "Bindings.{}: {}", key, warning);
            return binding_text::formatBindingTable(F, converted.value);
        }

        template <std::size_t... Index>
        constexpr auto bindingConversions(std::index_sequence<Index...>)
        {
            return std::array<RPS::Settings::ValueConversion, sizeof...(Index)>{ RPS::Settings::ValueConversion{
                "Bindings", fb::functionKey(static_cast<fb::Function>(Index)),
                &convertBinding<static_cast<fb::Function>(Index)> }... };
        }

        // Each binding's conversion from the old string form and the retired hold times. Frozen like the INI table.
        constexpr auto kConversions = bindingConversions(std::make_index_sequence<fb::kFunctionCount>{});

        // The function a [Bindings] key binds, or nullopt for another option.
        std::optional<fb::Function> bindingFunction(const Setting& setting)
        {
            if (setting.table != "Bindings")
                return std::nullopt;
            for (std::size_t f = 0; f < fb::kFunctionCount; ++f) {
                if (fb::functionKey(static_cast<fb::Function>(f)) == setting.key)
                    return static_cast<fb::Function>(f);
            }
            return std::nullopt;
        }

        // A binding table as the one-line form; a table ROCK cannot read is published as written.
        std::string bindingLine(fb::Function function, const std::string& table)
        {
            const auto parsed = binding_text::parseBindingTable(function, table);
            return parsed ? binding_text::formatBindingLine(function, parsed->value) : table;
        }

        // A transform's position as canonical Float3 text; no pos is no move.
        std::string positionText(const std::string& transform)
        {
            const auto position =
                parseTransform(transform).value_or(Transform{}).position.value_or(std::array<float, 3>{});
            return normalizeValue(ValueType::Float3, std::format("{}, {}, {}", position[0], position[1], position[2]))
                .value_or("[0.0, 0.0, 0.0]");
        }

        constexpr std::string_view kPlayerHeader =
            "## ROCK.toml - player options\n"
            "## Change these in game in the RobCo PALM wheel menu, or edit this file. Changes are picked up while the "
            "game runs.\n"
            "## Each option has its help above it. Options missing from this file use their defaults.\n"
            "## Values are typed: true or false, whole numbers, decimal numbers, \"text in quotes\", lists such as "
            "[0.0, 1.0, 2.0]\n"
            "## and tables such as { pos = [0.0, 1.0, 2.0] }.\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it adds new options with their defaults, rewrites its help lines (the lines starting "
            "with ##),\n"
            "## and moves options it no longer reads into the block at the end, as comments.\n"
            "## Your own notes are kept when they start with a single # on the lines above an option.\n"
            "## Developer options live in ROCK_Developer.toml, which exists only while a developer option is "
            "changed.\n";
        constexpr std::string_view kDeveloperHeader =
            "## ROCK_Developer.toml - developer options\n"
            "## PALM saves only developer options that differ from their defaults. Resetting an option removes it,\n"
            "## and this file is removed when no options remain. Changes are picked up while the game runs.\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it rewrites its help lines (the lines starting with ##) and moves options it no "
            "longer\n"
            "## reads into the block at the end, as comments. Your own notes are kept when they start with a single "
            "#\n"
            "## on the lines above an option.\n";
    }

    std::unique_ptr<Store> makeStore(const std::filesystem::path& directory, std::span<const SettingDefault> defaults)
    {
        std::vector<RPS::Settings::Definition> definitions;
        definitions.reserve(std::size(kSettingMetadata));
        for (const auto& metadata : kSettingMetadata) {
            const auto found = std::find_if(defaults.begin(), defaults.end(),
                [&](const SettingDefault& entry) { return entry.table == metadata.table && entry.key == metadata.key; });
            if (found == defaults.end())
                throw std::runtime_error(
                    std::format("Catalog option {} has no compiled default", settingPath(metadata.table, metadata.key)));
            definitions.push_back({
                .table = std::string(metadata.table),
                .key = std::string(metadata.key),
                .category = std::string(metadata.category),
                .description = std::string(metadata.description),
                .group = storeGroup(metadata.group),
                .type = found->type,
                .defaultValue = found->value,
            });
        }
        for (const auto& entry : defaults) {
            if (std::none_of(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                    [&](const SettingMetadata& metadata) { return metadata.table == entry.table && metadata.key == entry.key; }))
                throw std::runtime_error(
                    std::format("Compiled option {} has no catalog entry", settingPath(entry.table, entry.key)));
        }
        return std::make_unique<Store>(
            RPS::Settings::Files{
                .directory = directory,
                .modName = "ROCK",
                .playerFileName = std::string(kPlayerFileName),
                .developerFileName = std::string(kDeveloperFileName),
                .playerHeader = std::string(kPlayerHeader),
                .developerHeader = std::string(kDeveloperHeader),
            },
            std::move(definitions),
            RPS::Settings::LegacyIni{ { directory / "ROCK.ini", directory / "ROCK_Developer.ini" }, kLegacyKeys },
            kRenamedKeys, kConversions);
    }

    /**
     * A setting as the Configuration API publishes it. The API has no transform or table type, and a consumer that
     * meets an unknown type drops every setting: a Transform is published as its position, a Float3 (ROCK's
     * transforms hold only a position), and a binding table as a String in the one-line form.
     */
    ApiSetting apiSetting(const Setting& setting)
    {
        if (setting.type == ValueType::Transform)
            return { api::configuration::ValueType::Float3, positionText(setting.value), positionText(setting.defaultValue) };
        if (setting.type == ValueType::Table) {
            const auto function = bindingFunction(setting);
            if (!function)
                return { api::configuration::ValueType::String, setting.value, setting.defaultValue };
            return { api::configuration::ValueType::String, bindingLine(*function, setting.value),
                bindingLine(*function, setting.defaultValue) };
        }
        return { static_cast<api::configuration::ValueType>(setting.type), setting.value, setting.defaultValue };
    }

    /**
     * The store value of a Configuration API write. A Float3 sent to a Transform replaces its position and keeps the
     * parts the file has besides. A binding takes the one-line form (or its table) and is refused with the reason
     * when ROCK cannot read it. Any other value is passed on for the store to check.
     */
    std::expected<std::string, std::string> storeValueFromApi(const Setting& setting, std::string_view value)
    {
        if (setting.type == ValueType::Table) {
            const auto function = bindingFunction(setting);
            if (!function)
                return std::string(value);
            auto parsed = binding_text::parseBindingLine(*function, value);
            if (!parsed) {
                if (const auto table = binding_text::parseBindingTable(*function, value))
                    parsed = table;
            }
            if (!parsed)
                return std::unexpected(std::format("Bindings.{} cannot be '{}': {}", setting.key, value, parsed.error()));
            return binding_text::formatBindingTable(*function, parsed->value);
        }
        if (setting.type != ValueType::Transform)
            return std::string(value);
        const auto position = parseFloat3(value);
        if (!position)
            return std::string(value);
        auto transform = parseTransform(setting.value).value_or(Transform{});
        transform.position = *position;
        return formatTransform(transform);
    }
}
