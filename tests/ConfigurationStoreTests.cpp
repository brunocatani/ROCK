// ROCK's settings contracts on RPS::Settings (whose own tests cover
// organizing, faults and the developer file): the catalog and the loader
// agree, the example files are what ROCK writes, documented defaults are the
// runtime defaults, ROCK's old INI keys and renamed TOML keys migrate to the
// right options, and [Bindings] holds CommonFramework tables, converted from
// ROCK's old strings.
#include "RockConfig.h"
#include "config/RockSettings.h"
#include "physics-interaction/input/FunctionBindingText.h"

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
            "fNativeScopeOverlayOffsetYGameUnits = 12\r\n"
            "fManualScopeHoldSeconds = 0.7\r\n"
            "[Controls]\r\n"
            "fPipboyPauseHoldSeconds = 0.350\r\n");
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
        require(find(*store, "FiringHand.Left", "AimOffset").value == "{ pos = [0.0, 2.5, 0.0] }",
            "an X/Y/Z triplet was not carried as one transform position");
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
        require(find(*store, "Bindings", "Scope").value ==
                    "{ hand = \"primary\", type = \"hold\", button = \"a\", duration = 0.7 }",
            "an INI scope hold time was not carried into the Scope binding");
        require(find(*store, "Bindings", "Pause").value == find(*store, "Bindings", "Pause").defaultValue,
            "an INI hold time at its old default changed the Pause binding");
    }

    bool near(float value, float expected) { return value > expected - 1.0e-4f && value < expected + 1.0e-4f; }

    // The [Bindings] tables are a configuration contract: CommonFramework's words with ROCK's limits, and every
    // default reading back as itself.
    void bindingTextChecks()
    {
        namespace fb = rock::function_binding_policy;
        namespace text = rock::function_binding_text;
        using fb::Function;
        for (std::size_t f = 0; f < fb::kFunctionCount; ++f) {
            const auto function = static_cast<Function>(f);
            const auto standard = text::defaultBinding(function);
            const auto table = text::parseBindingTable(function, text::formatBindingTable(function, standard));
            require(table && table->value == standard,
                "a default binding does not read back as itself");
        }
        const auto read = [](Function function, std::string_view table) { return text::parseBindingTable(function, table); };
        const auto fire = read(Function::Fire, "{ hand = 'primary', type = 'hold', button = 'trigger' }");
        require(fire && fire->value.binding.control.hand == fb::BindingHand::Firing &&
                    fire->value.binding.gesture == fb::Gesture::Press,
            "primary on a weapon function is not the firing hand, or a hold without duration is not a press");
        require(!read(Function::Fire, "{ hand = 'left', type = 'hold', button = 'trigger' }") &&
                    read(Function::Use, "{ hand = 'left', type = 'hold', button = 'a' }"),
            "a fixed hand was refused elsewhere or taken on a weapon function");
        const auto timed = read(Function::Grenade, "{ hand = 'primary', type = 'hold', button = 'menu', duration = 5 }");
        require(timed && timed->value.binding.gesture == fb::Gesture::Hold && near(timed->value.seconds, 2.0f) &&
                    timed->value.binding.control.source == fb::Source::UpperFace && timed->warnings.size() == 1,
            "a timed hold was not clamped with a warning");
        const auto doubled = read(Function::Vats, "{ hand = 'primary', type = 'double', button = 'grip' }");
        require(doubled && near(doubled->value.seconds, 0.4f), "a double without duration does not take 0.4 seconds");
        require(!read(Function::Vats, "{ hand = 'primary', type = 'press', button = 'b' }") &&
                    !read(Function::Vats, "{ hand = 'primary', type = 'longpress', button = 'b' }") &&
                    !read(Function::Vats, "{ hand = 'primary', type = 'tap', button = 'b', duration = 0.2 }") &&
                    !read(Function::Vats, "{ hand = 'primary', type = 'tap', button = 'b', suppress = true }") &&
                    !read(Function::Vats, "{ hand = 'primary', type = 'tap', button = 'system' }") &&
                    !read(Function::Vats, "\"primary tap b\""),
            "a word, key or value ROCK does not read was taken");
        const auto chord = read(Function::Vats, "{ hand = 'offhand', type = 'tap', button = 'a', modifier = { button = 'grip' } }");
        require(chord && chord->value.binding.modifier.hand == fb::BindingHand::Offhand &&
                    text::formatBindingTable(Function::Vats, chord->value) ==
                        "{ hand = \"offhand\", type = \"tap\", button = \"a\", modifier = { button = \"grip\" } }",
            "a same-hand modifier did not read and write back");
        const auto none = read(Function::Palm, "{ type = 'none' }");
        require(none && !none->value.binding.bound() &&
                    text::formatBindingTable(Function::Palm, none->value) == "{ type = \"none\" }",
            "an unbound function did not read and write back");
    }

    // A ROCK.toml from before [Bindings] held tables: each old string becomes its table with the retired hold times
    // as durations, what a table cannot say becomes the default, and the retired keys stay only as comments.
    void bindingMigrationChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        const auto store = makeStore(directory, defaults);
        write(store->path(Group::Player),
            "[Bindings]\n"
            "Reload = \"firing double grip\"\n"
            "Scope = \"firing hold a\"\n"
            "Pause = \"offhand hold y\"\n"
            "Pipboy = \"offhand hold thumbstick +offhand grip\"\n"
            "Use = \"primary press a\"\n"
            "Vats = \"right tap grip_click\"\n"
            "Fire = \"left press trigger\"\n"
            "HoldSeconds = 0.5\n"
            "DoubleTapSeconds = 0.25\n"
            "[Scopes]\n"
            "ManualScopeHoldSeconds = 0.6\n"
            "[ControllerInput]\n"
            "PipboyPauseHoldSeconds = 0.8\n");
        store->load(true);
        const auto value = [&](std::string_view key) { return find(*store, "Bindings", key).value; };
        require(value("Reload") == "{ hand = \"primary\", type = \"double\", button = \"grip\", duration = 0.25 }" &&
                    value("Scope") == "{ hand = \"primary\", type = \"hold\", button = \"a\", duration = 0.6 }" &&
                    value("Pause") == "{ hand = \"offhand\", type = \"hold\", button = \"b\", duration = 0.8 }" &&
                    value("Pipboy") == "{ hand = \"offhand\", type = \"hold\", button = \"thumbstick\", duration = 0.5, "
                                       "modifier = { button = \"grip\" } }" &&
                    value("Use") == "{ hand = \"primary\", type = \"hold\", button = \"a\" }",
            "an old binding string was not converted with its retired hold time");
        require(value("Vats") == find(*store, "Bindings", "Vats").defaultValue &&
                    value("Fire") == find(*store, "Bindings", "Fire").defaultValue,
            "a physical control or a fixed weapon hand did not become the default");
        require(value("Grenade") == "{ hand = \"primary\", type = \"hold\", button = \"b\", duration = 0.25 }",
            "a binding the file did not hold was not written at its default");
        const auto organized = bytes(store->path(Group::Player));
        require(organized.find("# Bindings.HoldSeconds = 0.5") != std::string::npos &&
                    organized.find("# Scopes.ManualScopeHoldSeconds = 0.6") != std::string::npos,
            "a retired hold time was not kept as a comment");
        const auto values = runtime(*store);
        const auto seconds = [&](rock::function_binding_policy::Function function) {
            return values.rockBindingSeconds[static_cast<std::size_t>(function)];
        };
        require(near(seconds(rock::function_binding_policy::Function::Scope), 0.6f) &&
                    near(seconds(rock::function_binding_policy::Function::Reload), 0.25f),
            "a converted duration did not apply");
        store->load(true);
        require(bytes(store->path(Group::Player)) == organized && store->notices().empty(),
            "a binding was converted twice");
    }

    // A ROCK.toml from before the firing-hand offsets became transforms keeps them.
    void transformChecks(const fs::path& directory, std::span<const SettingDefault> defaults)
    {
        fs::create_directories(directory);
        const auto store = makeStore(directory, defaults);
        write(store->path(Group::Player),
            "[FiringHand.Left]\nAimOffsetGameUnits = [1, 2, 3]\n[FiringHand.Right]\nSupportGripOffsetGameUnits = [0, 0, -4]\n");
        store->load(true);
        require(find(*store, "FiringHand.Left", "AimOffset").value == "{ pos = [1.0, 2.0, 3.0] }" &&
                    find(*store, "FiringHand.Right", "SupportGripOffset").value == "{ pos = [0.0, 0.0, -4.0] }",
            "a renamed offset was not carried into its transform");
        const auto values = runtime(*store);
        require(values.rockLeftFiringAimOffsetYGameUnits > 1.99f && values.rockLeftFiringAimOffsetYGameUnits < 2.01f &&
                    values.rockRightSupportGripOffsetGameUnits.z < -3.99f &&
                    values.rockRightSupportGripOffsetGameUnits.z > -4.01f,
            "a carried transform position did not apply");
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
        transformChecks(directory / "transforms", defaults);
        bindingTextChecks();
        bindingMigrationChecks(directory / "bindings", defaults);
        fs::remove_all(directory, ec);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
