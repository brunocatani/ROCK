#include "config/RockSettings.h"
#include "config/SettingMetadata.h"

#include <algorithm>
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

        constexpr std::string_view kPlayerHeader =
            "## ROCK.toml - player options\n"
            "## Change these in game in the RobCo PALM wheel menu, or edit this file. Changes are picked up while the "
            "game runs.\n"
            "## Each option has its help above it. Options missing from this file use their defaults.\n"
            "## Values are typed: true or false, whole numbers, decimal numbers, \"text in quotes\" and lists such as "
            "[0.0, 1.0, 2.0].\n"
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
            RPS::Settings::LegacyIni{ { directory / "ROCK.ini", directory / "ROCK_Developer.ini" }, kLegacyKeys });
    }
}
