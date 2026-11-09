// ROCK's settings contracts on RPS::Settings (whose own tests cover
// organizing, faults and the developer file): the catalog and the loader
// agree, the example files are what ROCK writes, documented defaults are the
// runtime defaults, and ROCK's old INI keys migrate to the right options.
#include "RockConfig.h"
#include "config/RockSettings.h"

#include <array>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace rock::config;
    using RPS::Settings::Group;
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

    const Setting& find(const Store& store, std::string_view table, std::string_view key)
    {
        if (const auto* setting = store.find(table, key))
            return *setting;
        throw std::runtime_error(std::format("missing setting {}.{}", table, key));
    }

    bool hasLine(const std::vector<std::string>& lines, std::string_view text)
    {
        for (const auto& line : lines) {
            if (line.find(text) != std::string::npos)
                return true;
        }
        return false;
    }

    rock::RockConfigValues runtime(const Store& store) { return rock::RockConfig::parseValues(store.loadedValues()); }

    // A player upgrading from ROCK 0.92 or the last INI version keeps the
    // settings they changed: renamed keys, X/Y/Z triplets, bLeftHandedMode and
    // options that changed files land in the right TOML file; old defaults,
    // retired keys and values that do not fit stay behind.
    void migrationChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        const auto playerIni = directory / "ROCK.ini";
        const auto developerIni = directory / "ROCK_Developer.ini";
        write(playerIni,
            "; ROCK.ini - player options\r\n"
            "[ImmersiveWeapons]\r\n"
            "bGunstockMode = true\r\n"
            "iLogLevel = 2\r\n"
            "iWeaponFilteringPreset = 0\r\n"
            "[AmbidextrousFiring]\r\n"
            "bLeftHandedMode = true\r\n"
            "fLeftFiringAimOffsetXGameUnits = 0.0\r\n"
            "fLeftFiringAimOffsetYGameUnits = 2.5\r\n"
            "[Controls]\r\n"
            "sRightAButton = b\r\n"
            "bRockyModeEnabled = true\r\n"
            "fRealisticGrenadeFuseSeconds = soon\r\n"
            "[NativeScopes]\r\n"
            "fNativeScopeOverlayOffsetYGameUnits = 12\r\n");
        write(developerIni, "[PhysicsInteraction]\r\niDebugMaxBodyInstances = 100\r\n");

        const auto store = makeStore(directory, defaults);
        store->load(true);
        require(!fs::exists(playerIni) && !fs::exists(developerIni), "migrated INI files were not deleted");
        require(find(*store, "WeaponHandling", "GunstockMode").value == "true", "a changed option was not carried");
        require(find(*store, "Logging", "LogLevel").value == find(*store, "Logging", "LogLevel").defaultValue,
            "an old default was carried");
        require(find(*store, "WeaponHandling", "WeaponFilteringPreset").value ==
                find(*store, "WeaponHandling", "WeaponFilteringPreset").defaultValue,
            "an old default replaced the current default");
        require(find(*store, "FiringHand", "DefaultHand").value == "left", "bLeftHandedMode was not carried");
        require(find(*store, "FiringHand.Left", "AimOffsetGameUnits").value == "[0.0, 2.5, 0.0]",
            "an X/Y/Z triplet was not carried as one list");
        require(find(*store, "ControllerInput.Right", "A").value == "b", "a button layout key was not carried");
        require(find(*store, "ScopePlacement", "NativeScopeOverlayOffsetGameUnits").value == "[0.0, 12.0, 0.0]" &&
                    find(*store, "DebugOverlay", "DebugMaxBodyInstances").value == "100",
            "developer options were not carried");
        require(bytes(store->path(Group::Developer)).find("NativeScopeOverlayOffsetGameUnits = [0.0, 12.0, 0.0]") !=
                std::string::npos,
            "an option that moved to the developer file was not written there");
        require(hasLine(store->messages(), "does not fit: fRealisticGrenadeFuseSeconds = soon"),
            "a value that does not fit was not reported");
        require(hasLine(store->notices(), "bRockyModeEnabled"), "a retired key was not named");
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
        const auto store = makeStore(directory, defaults);
        std::string developer;
        std::string_view table;
        for (const auto& setting : store->settings()) {
            if (setting.group != Group::Developer)
                continue;
            if (setting.table != table)
                developer += std::format("[{}]\n", setting.table);
            table = setting.table;
            developer += std::format("{} = {}\n", setting.key,
                setting.type == ValueType::String ? std::format("'{}'", setting.defaultValue) : setting.defaultValue);
        }
        write(store->path(Group::Developer), developer);
        store->load(true);
        require(store->messages().empty(), "cannot organize the default files");
        return { bodyAfterHeader(withoutCarriageReturns(bytes(store->path(Group::Player)))),
            bodyAfterHeader(withoutCarriageReturns(bytes(store->path(Group::Developer)))) };
    }

    // Both organized default files hold every option once at its default, and
    // reading them gives the same runtime values as reading nothing.
    void defaultChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        const auto store = makeStore(directory, defaults);
        store->load(false);
        for (const auto& setting : store->settings()) {
            require(setting.specified, "an organized default file is missing an option");
            require(setting.value == setting.defaultValue, "an organized default differs from the compiled default");
        }
        require(runtime(*store) == rock::RockConfig::parseValues({}),
            "omitting options does not give the same runtime values as all documented defaults");
    }
}

int main(int argc, char** argv)
{
    const fs::path directory = fs::path(ROCK_CONFIG_TEST_DIR);
    try {
        std::error_code ec;
        fs::remove_all(directory, ec);
        const auto defaults = rock::RockConfig::compiledDefaults();
        // Construction checks the catalog against the loader in both directions.
        const auto exampleBodies = organizedDefaultBodies(directory / "examples", defaults);
        if (argc == 3 && std::string_view(argv[1]) == "--write-examples") {
            for (std::size_t index = 0; index < kExampleNames.size(); ++index)
                write(fs::path(argv[2]) / kExampleNames[index],
                    std::string(kExampleHeaders[index]) + exampleBodies[index]);
            fs::remove_all(directory, ec);
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
        migrationChecks(directory / "migration", defaults);
        fs::remove_all(directory, ec);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
