#pragma once

#include <ROCK/Configuration.h>
#include <SimpleIni.h>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rock::config
{
    using api::configuration::Group;
    using api::configuration::ValueType;

    [[nodiscard]] Group settingGroup(std::string_view section, std::string_view key) noexcept;

    struct Setting
    {
        std::string section;
        std::string key;
        std::string defaultValue;
        std::string value;
        std::string category;
        std::string description;
        Group group;
        ValueType type;
        bool specified = false;
        std::size_t displayOrder = 0;
    };

    struct Change
    {
        std::string_view section, key, value;
    };

    // Owned and serialized by RockConfig. Configuration tasks save/visit through
    // that owner; only its runtime reload applies the catalog to live settings.
    // INI sections are labels. Case-insensitive keys identify settings within
    // their owning file; the last physical occurrence wins. Startup and explicit
    // writes organize each existing file the same way: effective values under the
    // catalog's labels and help, missing player options added, entries ROCK does
    // not read kept only as comments. Startup writes only when that text differs;
    // hot reload only reads.
    class ConfigurationStore
    {
    public:
        ConfigurationStore(std::filesystem::path directory, const CSimpleIniA& defaults);
        [[nodiscard]] bool load(bool createConsumer);
        [[nodiscard]] bool setValue(Group group, std::string_view section, std::string_view key,
            std::string_view value);
        [[nodiscard]] bool setValues(Group group, std::span<const Change> changes);
        void appendLoadedValues(CSimpleIniA& target) const;

        [[nodiscard]] const std::vector<Setting>& settings() const noexcept { return _settings; }

        [[nodiscard]] const std::string& error() const noexcept { return _error; }

        [[nodiscard]] const std::filesystem::path& directory() const noexcept { return _directory; }

        [[nodiscard]] std::filesystem::path path(Group group) const;

        [[nodiscard]] std::uint64_t revision() const noexcept { return _revision; }

        [[nodiscard]] static bool isDefault(const Setting& setting, std::string_view value);

    private:
        bool readFile(Group group, CSimpleIniA& ini);
        bool refreshFile(Group group);
        bool organizeFile(Group group, const CSimpleIniA& source, CSimpleIniA& output);
        bool writeFile(Group group, const CSimpleIniA& ini, bool replace);
        bool removeDeveloperFile();
        bool publish(Group group, std::string_view contents, bool replace);
        std::filesystem::path _directory;
        std::vector<Setting> _settings;
        std::string _error;
        std::uint64_t _revision = 0;
    };
}
