#include "config/RockSettings.h"
#include "config/SettingMetadata.h"

#include <algorithm>
#include <array>
#include <format>
#include <stdexcept>
#include <vector>

namespace rock::config
{
    namespace
    {
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
            kRenamedKeys);
    }

    /**
     * A setting as the Configuration API publishes it. A Transform is published as its position, a Float3: the API
     * has no transform type, a consumer that meets an unknown type drops every setting, and ROCK's transforms hold
     * only a position.
     */
    ApiSetting apiSetting(const Setting& setting)
    {
        if (setting.type != ValueType::Transform)
            return { static_cast<api::configuration::ValueType>(setting.type), setting.value, setting.defaultValue };
        return { api::configuration::ValueType::Float3, positionText(setting.value), positionText(setting.defaultValue) };
    }

    /**
     * The store value of a Configuration API write. A Float3 sent to a Transform replaces its position and keeps
     * the parts the file has besides; any other value is passed on for the store to check.
     */
    std::string storeValueFromApi(const Setting& setting, std::string_view value)
    {
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
