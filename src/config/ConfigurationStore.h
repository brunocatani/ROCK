#pragma once

#include <ROCK/Configuration.h>
#include <array>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rock::config
{
    using api::configuration::Group;
    using api::configuration::ValueType;

    // A compiled default as the loader declares it: TOML table path, key,
    // canonical value text and type.
    struct SettingDefault
    {
        std::string table;
        std::string key;
        std::string value;
        ValueType type;
    };

    struct Setting
    {
        std::string table; // TOML table path, for example "ControllerInput.Left".
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
        std::string_view table, key, value;
    };

    // Canonical values of the settings the files specify, keyed by settingPath().
    using LoadedValues = std::map<std::string, std::string, std::less<>>;

    [[nodiscard]] std::string settingPath(std::string_view table, std::string_view key);

    // Canonical text of a value: true/false, a decimal integer, a float that
    // always carries a '.' or exponent, raw text, or "[x, y, z]". Accepts the
    // looser forms an API caller may send (1/0, yes/no, "x, y, z"). Returns
    // nullopt for a value the type cannot hold.
    [[nodiscard]] std::optional<std::string> normalizeValue(ValueType type, std::string_view value);
    [[nodiscard]] std::optional<std::array<float, 3>> parseFloat3(std::string_view value);

    // Owned and serialized by RockConfig. Configuration tasks save/visit through
    // that owner; only its runtime reload applies the catalog to live settings.
    // A setting is identified by its TOML table path and key inside its owning
    // file. Startup and explicit writes organize each file the same way:
    // effective values under the catalog's tables and help, missing player
    // options added, entries ROCK does not read kept only as comments. A file
    // that does not parse is never rewritten and its settings keep the last
    // values read from it. Startup writes only when the text differs; hot
    // reload only reads.
    class ConfigurationStore
    {
    public:
        ConfigurationStore(std::filesystem::path directory, std::span<const SettingDefault> defaults);

        // Per-file problems are reported through messages(); the other file
        // still applies and the failed file's settings keep their last values.
        void load(bool organizeFiles);
        [[nodiscard]] bool setValue(Group group, std::string_view table, std::string_view key, std::string_view value);
        [[nodiscard]] bool setValues(Group group, std::span<const Change> changes);
        [[nodiscard]] LoadedValues loadedValues() const;

        [[nodiscard]] const std::vector<Setting>& settings() const noexcept { return _settings; }

        // Why the last setValue/setValues failed.
        [[nodiscard]] const std::string& error() const noexcept { return _error; }

        // Problems found by the last load: syntax errors, values of the wrong
        // type, files that could not be updated. Each names the file and line.
        [[nodiscard]] const std::vector<std::string>& messages() const noexcept { return _messages; }

        // What the last startup load did besides reading: the migration of
        // the old INI files and their deletion.
        [[nodiscard]] const std::vector<std::string>& notices() const noexcept { return _notices; }

        [[nodiscard]] const std::filesystem::path& directory() const noexcept { return _directory; }

        [[nodiscard]] std::filesystem::path path(Group group) const;

        [[nodiscard]] std::uint64_t revision() const noexcept { return _revision; }

        [[nodiscard]] static bool isDefault(const Setting& setting, std::string_view value);

        static constexpr std::string_view kConsumerFileName = "ROCK.toml";
        static constexpr std::string_view kDeveloperFileName = "ROCK_Developer.toml";

    private:
        // Startup only: carries the old ROCK.ini and ROCK_Developer.ini into
        // the TOML files when ROCK.toml does not exist yet, then deletes them.
        void migrateLegacyFiles();
        bool removeDeveloperFile();
        bool publish(Group group, std::string_view contents, bool replace);
        std::filesystem::path _directory;
        std::vector<Setting> _settings;
        std::string _error;
        std::vector<std::string> _messages;
        std::vector<std::string> _notices;
        std::uint64_t _revision = 0;
    };
}
