// ROCK's settings files on RPS Framework's settings store (RPS::Settings):
// ROCK.toml and ROCK_Developer.toml, declared by the catalog
// (SettingMetadata.h) joined with the compiled defaults the loader declares
// (RockConfigLoad.cpp), the one-time migration of the INI files of earlier
// versions (LegacyIniMigrationTable.inl), the keys a later version renamed
// (kRenamedKeys in RockSettings.cpp) and the conversion of [Bindings] from
// ROCK's old string form (kConversions).
#pragma once

#include "config/SettingMetadata.h"

#include <RPS/Settings/Store.h>

#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace rock::config
{
    using RPS::Settings::Change;
    using RPS::Settings::formatTransform;
    using RPS::Settings::LoadedValues;
    using RPS::Settings::normalizeValue;
    using RPS::Settings::parseFloat3;
    using RPS::Settings::parseTransform;
    using RPS::Settings::Setting;
    using RPS::Settings::settingPath;
    using RPS::Settings::Store;
    using RPS::Settings::Transform;
    using RPS::Settings::ValueType;

    // A compiled default as the loader declares it: TOML table path, key,
    // canonical value text and type.
    struct SettingDefault
    {
        std::string table;
        std::string key;
        std::string value;
        ValueType type;
    };

    inline constexpr std::string_view kPlayerFileName = "ROCK.toml";
    inline constexpr std::string_view kDeveloperFileName = "ROCK_Developer.toml";

    [[nodiscard]] constexpr RPS::Settings::Group storeGroup(Group group) noexcept
    {
        return group == Group::Developer ? RPS::Settings::Group::Developer : RPS::Settings::Group::Player;
    }

    // Throws when the catalog and the loader disagree in either direction.
    [[nodiscard]] std::unique_ptr<Store> makeStore(const std::filesystem::path& directory,
        std::span<const SettingDefault> defaults);
}
