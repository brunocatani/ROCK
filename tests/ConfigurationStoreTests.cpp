#include "RockConfig.h"
#include "config/ConfigurationStore.h"

#include <array>
#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace rock::config;
    namespace fs = std::filesystem;

    void require(bool condition, const char* message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    std::string bytes(const fs::path& path)
    {
        std::ifstream input(path, std::ios::binary);
        require(input.good(), "cannot read a test file");
        return { std::istreambuf_iterator<char>(input), {} };
    }

    void write(const fs::path& path, std::string_view text)
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << text;
        require(output.good(), "cannot write a test file");
    }

    const Setting& find(const ConfigurationStore& store, std::string_view table, std::string_view key)
    {
        for (const auto& setting : store.settings()) {
            if (setting.table == table && setting.key == key)
                return setting;
        }
        throw std::runtime_error(std::format("missing setting {}.{}", table, key));
    }

    bool hasMessage(const ConfigurationStore& store, std::string_view text)
    {
        for (const auto& message : store.messages()) {
            if (message.find(text) != std::string::npos)
                return true;
        }
        return false;
    }

    rock::RockConfigValues runtime(const ConfigurationStore& store)
    {
        return rock::RockConfig::parseValues(store.loadedValues());
    }

    void valueChecks()
    {
        require(normalizeValue(ValueType::Float3, "[1, 2, 3]") == "[1.0, 2.0, 3.0]", "float3 brackets");
        require(normalizeValue(ValueType::Float3, " 0.5,-1 , 0 ") == "[0.5, -1.0, 0.0]", "float3 without brackets");
        require(!normalizeValue(ValueType::Float3, "[1, 2]") && !normalizeValue(ValueType::Float3, "1, 2, 3, 4") &&
                !normalizeValue(ValueType::Float3, "[1, nan, 3]"),
            "malformed float3 accepted");
        require(normalizeValue(ValueType::Float, "4") == "4.0" && normalizeValue(ValueType::Float, "-0") == "0.0" &&
                normalizeValue(ValueType::Float, "1e-05") == "1e-05",
            "float canonical form");
        require(!normalizeValue(ValueType::Float, "inf") && !normalizeValue(ValueType::Integer, "2.5"),
            "non-finite float or fractional integer accepted");
        require(normalizeValue(ValueType::Boolean, "On") == "true" && !normalizeValue(ValueType::Boolean, "maybe"),
            "boolean API forms");
        require(!normalizeValue(ValueType::String, "two\nlines"), "multi-line string accepted");
    }

    // Startup organizes a hand-edited file without changing what it means:
    // player notes stay, stale help is regenerated, unknown entries become
    // comments that never apply, and a second start leaves the file alone.
    void organizeChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        ConfigurationStore store(directory, defaults);
        write(store.path(Group::Consumer),
            "## stale help from an older ROCK\n"
            "[Logging]\n"
            "# my note about logging\n"
            "LogLevel = 3\n"
            "RetiredOption = 7\n"
            "[WeaponHandling]\n"
            "WeaponEquipMode = 3 # inline comments are not kept\n"
            "[RetiredTable]\n"
            "Value = \"kept\"\n"
            "[DeveloperControls]\n"
            "DeveloperModeEnabled = true\n"
            "# a note no key follows\n");
        store.load(true);
        require(store.messages().empty(), "a valid file reported problems");
        const auto organized = bytes(store.path(Group::Consumer));
        require(organized.find("# my note about logging\nLogLevel = 3\n") != std::string::npos,
            "player note did not stay above its option");
        require(organized.find("stale help") == std::string::npos, "ROCK help was not regenerated");
        require(organized.find("# Logging.RetiredOption = 7") != std::string::npos &&
                organized.find("# RetiredTable.Value = \"kept\"") != std::string::npos &&
                organized.find("# DeveloperControls.DeveloperModeEnabled = true") != std::string::npos &&
                organized.find("# a note no key follows") != std::string::npos,
            "unread entries were not kept as comments");
        require(organized.find("[ImmersiveAid]\n") != std::string::npos, "missing player options were not added");
        const auto values = runtime(store);
        require(values.rockLogLevel == 3 && values.rockWeaponEquipMode == 3, "organized values did not apply");
        require(!values.rockDeveloperModeEnabled, "a developer option applied from the player file");
        require(!fs::exists(store.path(Group::Developer)), "loading created the developer file");
        const auto revision = store.revision();
        store.load(true);
        require(bytes(store.path(Group::Consumer)) == organized, "a second start rewrote the organized file");
        require(store.revision() == revision, "an unchanged reload advanced the revision");
        require(store.setValue(Group::Consumer, "Logging", "LogLevel", "2"), "API write failed");
        const auto edited = bytes(store.path(Group::Consumer));
        require(edited.find("# my note about logging\nLogLevel = 2\n") != std::string::npos,
            "API write lost the player note");
        require(edited.find("# Logging.RetiredOption = 7") == edited.rfind("# Logging.RetiredOption = 7"),
            "unused entries were duplicated");
    }

    // A file that does not parse is never rewritten and its options keep the
    // last values read; a value of the wrong type keeps its last value alone.
    void faultChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        ConfigurationStore store(directory, defaults);
        write(store.path(Group::Consumer), "[Logging]\nLogLevel = 4\n[Grenades]\nRealisticGrenadeFuseSeconds = 5\n");
        store.load(true);
        require(runtime(store).rockLogLevel == 4, "valid value did not load");
        require(find(store, "Grenades", "RealisticGrenadeFuseSeconds").value == "5.0",
            "a whole number was not accepted for a decimal option");

        const std::string broken = "[Logging]\nLogLevel = 1\n[Grenades\n";
        write(store.path(Group::Consumer), broken);
        const auto revision = store.revision();
        store.load(true);
        require(bytes(store.path(Group::Consumer)) == broken, "a file with a syntax error was rewritten");
        require(store.revision() == revision && runtime(store).rockLogLevel == 4,
            "a syntax error did not keep the last values");
        require(hasMessage(store, "ROCK.toml line 3"), "the syntax error was not reported with its line");
        require(!store.setValue(Group::Consumer, "Logging", "LogLevel", "2") &&
                bytes(store.path(Group::Consumer)) == broken,
            "an API write replaced a file with a syntax error");

        write(store.path(Group::Consumer), "[Logging]\nLogLevel = \"high\"\n[Grenades]\nImmersiveGrenades = false\n");
        store.load(true);
        require(runtime(store).rockLogLevel == 4 && !runtime(store).rockImmersiveGrenades,
            "a wrong-type value did not keep its last value while the rest applied");
        require(hasMessage(store, "ROCK.toml line 2: Logging.LogLevel must be a whole number"),
            "the wrong-type value was not reported");
        require(bytes(store.path(Group::Consumer)).find("LogLevel = \"high\"") != std::string::npos,
            "organizing replaced the player's wrong-type value");
        write(store.path(Group::Consumer), "[Logging]\nLogLevel = 2.5\n");
        store.load(false);
        require(runtime(store).rockLogLevel == 4, "a fractional value was accepted for a whole-number option");
    }

    void developerChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        ConfigurationStore store(directory, defaults);
        store.load(true);
        require(store.setValue(Group::Developer, "PalmAndGrabOffsets", "RightGrabAuthorityProxyOffsetGameUnits",
                    "[1, 2.5, -3]"),
            "float3 write failed");
        require(bytes(store.path(Group::Developer)).find("RightGrabAuthorityProxyOffsetGameUnits = [1.0, 2.5, -3.0]") !=
                std::string::npos,
            "float3 was not written as a TOML array");
        const std::string pattern = R"(a "quoted" \ pattern %v)";
        require(store.setValue(Group::Developer, "LoggingAndProfiling", "LogPattern", pattern), "string write failed");
        store.load(false);
        const auto values = runtime(store);
        require(values.rockRightGrabAuthorityProxyOffsetGameUnits.x == 1.0f &&
                values.rockRightGrabAuthorityProxyOffsetGameUnits.y == 2.5f &&
                values.rockRightGrabAuthorityProxyOffsetGameUnits.z == -3.0f,
            "float3 did not reach runtime");
        require(values.rockLogPattern == pattern, "string escaping did not round-trip");
        require(!store.setValue(Group::Developer, "Logging", "LogLevel", "2"), "a write to the wrong file was accepted");
        require(!store.setValue(Group::Developer, "LoggingAndProfiling", "LogFlushImmediate", "maybe"),
            "an invalid value was accepted");
        const auto& offset = find(store, "PalmAndGrabOffsets", "RightGrabAuthorityProxyOffsetGameUnits");
        require(store.setValue(Group::Developer, offset.table, offset.key, offset.defaultValue) &&
                    store.setValue(Group::Developer, "LoggingAndProfiling", "LogPattern",
                        find(store, "LoggingAndProfiling", "LogPattern").defaultValue),
            "developer reset failed");
        require(!fs::exists(store.path(Group::Developer)), "resetting every developer option kept the file");
    }

    // The examples are exactly what ROCK writes for files holding every option
    // at its default, under a reference-only header, so their help is the catalog's.
    constexpr std::array<const char*, 2> kExampleNames{ "ROCK_example.toml", "ROCK_Developer_example.toml" };
    constexpr std::array<std::string_view, 2> kExampleHeaders{
        "## ROCK_example.toml - every player option with its default and help\n"
        "## Reference only: ROCK never reads this file. The first time ROCK starts it writes this same text as\n"
        "## ROCK.toml in Documents\\My Games\\Fallout4VR\\Mods_Config\\ROCK.\n",
        "## ROCK_Developer_example.toml - every developer option with its default and help\n"
        "## Reference only: ROCK never reads this file. ROCK_Developer.toml in Documents\\My Games\\Fallout4VR\\\n"
        "## Mods_Config\\ROCK holds only the developer options changed from these defaults.\n",
    };

    std::string withoutCarriageReturns(std::string text)
    {
        std::erase(text, '\r');
        return text;
    }

    // The body starts at the first category heading.
    std::string bodyAfterHeader(const std::string& text)
    {
        const auto start = text.find("\n## === ");
        require(start != std::string::npos, "settings text has no category");
        return text.substr(start);
    }

    std::array<std::string, 2> organizedDefaultBodies(const fs::path& directory,
        std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        ConfigurationStore store(directory, defaults);
        std::string developer;
        std::string_view table;
        for (const auto& setting : store.settings()) {
            if (setting.group != Group::Developer)
                continue;
            if (setting.table != table)
                developer += std::format("[{}]\n", setting.table);
            table = setting.table;
            developer += std::format("{} = {}\n", setting.key,
                setting.type == ValueType::String ? std::format("'{}'", setting.defaultValue) : setting.defaultValue);
        }
        write(store.path(Group::Developer), developer);
        store.load(true);
        require(store.messages().empty(), "cannot organize the default files");
        return { bodyAfterHeader(withoutCarriageReturns(bytes(store.path(Group::Consumer)))),
            bodyAfterHeader(withoutCarriageReturns(bytes(store.path(Group::Developer)))) };
    }

    // Both organized default files hold every option once at its default, and
    // reading them gives the same runtime values as reading nothing.
    void defaultChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        ConfigurationStore store(directory, defaults);
        store.load(false);
        for (const auto& setting : store.settings()) {
            require(setting.specified, "an organized default file is missing an option");
            require(setting.value == setting.defaultValue, "an organized default differs from the compiled default");
        }
        require(runtime(store) == rock::RockConfig::parseValues({}),
            "omitting options does not give the same runtime values as all documented defaults");
    }
}

int main(int argc, char** argv)
{
    try {
        const auto defaults = rock::RockConfig::compiledDefaults();
        const auto directory = fs::temp_directory_path() /
            ("ROCK-configuration-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));

        struct Cleanup
        {
            fs::path path;

            ~Cleanup()
            {
                std::error_code ec;
                fs::remove_all(path, ec);
            }
        } cleanup{ directory };

        // Construction checks the catalog against the loader in both directions.
        const auto exampleBodies = organizedDefaultBodies(directory / "examples", defaults);
        if (argc == 3 && std::string_view(argv[1]) == "--write-examples") {
            for (std::size_t index = 0; index < kExampleNames.size(); ++index)
                write(fs::path(argv[2]) / kExampleNames[index],
                    std::string(kExampleHeaders[index]) + exampleBodies[index]);
            return 0;
        }
        for (std::size_t index = 0; index < kExampleNames.size(); ++index) {
            const auto example =
                withoutCarriageReturns(bytes(fs::path(ROCK_CONFIG_REFERENCE_DIR) / kExampleNames[index]));
            if (bodyAfterHeader(example) != exampleBodies[index])
                throw std::runtime_error(
                    std::format("{} differs from the catalog; regenerate it with --write-examples <data/config>",
                        kExampleNames[index]));
        }
        defaultChecks(directory / "examples", defaults);
        valueChecks();
        organizeChecks(directory / "organize", defaults);
        faultChecks(directory / "faults", defaults);
        developerChecks(directory / "developer", defaults);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
