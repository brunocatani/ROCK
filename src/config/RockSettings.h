// ROCK's settings files on RPS Framework's settings store (RPS::Settings):
// ROCK.toml and ROCK_Developer.toml, declared by the catalog
// (SettingMetadata.h) joined with the compiled defaults the loader declares
// (RockConfigLoad.cpp), and the one-time migration of the INI files of
// earlier versions (LegacyIniMigrationTable.inl).
#pragma once

#include <ROCK/Configuration.h>
#include <RPS/Settings/Store.h>

#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace rock::config
{
    using RPS::Settings::Change;
    using RPS::Settings::LoadedValues;
    using RPS::Settings::normalizeValue;
    using RPS::Settings::parseFloat3;
    using RPS::Settings::Setting;
    using RPS::Settings::settingPath;
    using RPS::Settings::Store;
    using RPS::Settings::ValueType;

    // ROCK's Configuration API publishes the store's enums unchanged.
    static_assert(static_cast<std::uint32_t>(ValueType::Boolean) ==
            static_cast<std::uint32_t>(api::configuration::ValueType::Boolean) &&
        static_cast<std::uint32_t>(ValueType::Integer) == static_cast<std::uint32_t>(api::configuration::ValueType::Integer) &&
        static_cast<std::uint32_t>(ValueType::Float) == static_cast<std::uint32_t>(api::configuration::ValueType::Float) &&
        static_cast<std::uint32_t>(ValueType::String) == static_cast<std::uint32_t>(api::configuration::ValueType::String) &&
        static_cast<std::uint32_t>(ValueType::Float3) == static_cast<std::uint32_t>(api::configuration::ValueType::Float3));

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

    [[nodiscard]] constexpr RPS::Settings::Group storeGroup(api::configuration::Group group) noexcept
    {
        return group == api::configuration::Group::Developer ? RPS::Settings::Group::Developer
                                                             : RPS::Settings::Group::Player;
    }

    [[nodiscard]] constexpr api::configuration::Group apiGroup(RPS::Settings::Group group) noexcept
    {
        return group == RPS::Settings::Group::Developer ? api::configuration::Group::Developer
                                                        : api::configuration::Group::Consumer;
    }

    // Throws when the catalog and the loader disagree in either direction.
    [[nodiscard]] std::unique_ptr<Store> makeStore(const std::filesystem::path& directory,
        std::span<const SettingDefault> defaults);
}
