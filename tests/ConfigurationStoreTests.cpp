#include "RockConfig.h"
#include "config/SettingMetadata.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
    void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
    std::string bytes(const std::filesystem::path& path)
    {
        std::ifstream file(path, std::ios::binary);
        return { std::istreambuf_iterator<char>(file), {} };
    }
    const rock::config::Setting& find(const rock::config::ConfigurationStore& store, std::string_view key)
    {
        for (const auto& setting : store.settings()) if (setting.key == key) return setting;
        throw std::runtime_error("Missing compiled setting");
    }

    void verifyReference(const CSimpleIniA& reference,
        const rock::config::ConfigurationStore& store, rock::config::Group owner)
    {
        CSimpleIniA::TNamesDepend sections;
        reference.GetAllSections(sections);
        for (const auto& section : sections) {
            CSimpleIniA::TNamesDepend keys;
            reference.GetAllKeys(section.pItem, keys);
            for (const auto& key : keys) {
                const auto& setting = find(store, key.pItem);
                require(setting.section == section.pItem && setting.group == owner,
                    "reference key has an unsupported section or owner");
                CSimpleIniA::TNamesDepend values;
                reference.GetAllValues(section.pItem, key.pItem, values);
                require(values.size() == 1, "reference contains a duplicate key");
            }
        }
    }
}

int main(int argc, char** argv)
{
    using namespace rock::config;
    namespace fs = std::filesystem;
    try {
        CSimpleIniA compiled;
        rock::RockConfig::buildCompiledDefaults(compiled);
        require(compiled.GetLongValue("ImmersiveWeapons", "iWeaponGrabMode", 0) == 1, "weapon grab mode default must preserve toggle both");
        require(compiled.GetValue("ImmersiveWeapons", "bToggleGrab", nullptr) == nullptr, "retired toggle boolean remains in the supported catalog");
        if (argc == 3 && std::string_view(argv[1]) == "--dump-defaults") {
            require(compiled.SaveFile(argv[2], false) >= 0, "compiled defaults dump failed");
            return 0;
        }
        const auto directory = fs::temp_directory_path() /
            ("ROCK-configuration-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        struct Cleanup { fs::path path; ~Cleanup() { std::error_code ec; fs::remove_all(path, ec); } } cleanup{ directory };
        ConfigurationStore store(directory, compiled);
        CSimpleIniA consumerExample;
        CSimpleIniA developerExample;
        consumerExample.SetMultiKey(true);
        developerExample.SetMultiKey(true);
        require(consumerExample.LoadFile((fs::path(ROCK_CONFIG_REFERENCE_DIR) / "ROCK_example.ini").c_str()) >= 0, "consumer reference unavailable");
        require(developerExample.LoadFile((fs::path(ROCK_CONFIG_REFERENCE_DIR) / "ROCK_Developer_example.ini").c_str()) >= 0, "developer reference unavailable");
        verifyReference(consumerExample, store, Group::Consumer);
        verifyReference(developerExample, store, Group::Developer);
        CSimpleIniA allReferences;
        require(allReferences.LoadFile((fs::path(ROCK_CONFIG_REFERENCE_DIR) / "ROCK_example.ini").c_str()) >= 0, "consumer reference merge failed");
        require(allReferences.LoadFile((fs::path(ROCK_CONFIG_REFERENCE_DIR) / "ROCK_Developer_example.ini").c_str()) >= 0, "developer reference merge failed");
        CSimpleIniA missingOptions;
        require(rock::RockConfig::parseValues(missingOptions) == rock::RockConfig::parseValues(allReferences),
            "omitting options does not produce the same runtime values as all documented defaults");
        require(settingGroup("PhysicsInteraction", "fRightGrabLegacyPalmPivotAHandspaceX") == Group::Developer,
            "the first option below the authored boundary is not developer-owned");
        require(settingGroup("PhysicsInteraction", "fMouthConsumeCommitHapticIntensity") == Group::Developer,
            "the final option below the authored boundary is not developer-owned");
        require(settingGroup("PhysicsInteraction", "sHighlightColor") == Group::Consumer,
            "the final option above the authored boundary is not consumer-owned");
        for (const auto& setting : store.settings()) {
            const auto& example = setting.group == Group::Consumer ? consumerExample : developerExample;
            const auto& other = setting.group == Group::Consumer ? developerExample : consumerExample;
            const auto* documented = example.GetValue(setting.section.c_str(), setting.key.c_str(), nullptr);
            if (!documented || !ConfigurationStore::isDefault(setting, documented)) {
                throw std::runtime_error(std::format("Example default differs from compiled [{}] {}: example={} compiled={}",
                    setting.section, setting.key, documented ? documented : "missing", setting.defaultValue));
            }
            require(!other.GetValue(setting.section.c_str(), setting.key.c_str(), nullptr), "example option belongs to both files");
        }
        require(store.load(true), "first-run load failed");
        require(find(store, "bEnableImmersiveScopes").value == "true", "immersive scopes must default on");
        for (const bool enabled : { false, true }) {
            require(store.setValue(Group::Consumer, "NativeScopes", "bEnableImmersiveScopes", enabled ? "true" : "false"),
                "scope mode write failed");
            require(store.load(false), "scope mode reload failed");
            CSimpleIniA scopeValues;
            store.appendLoadedValues(scopeValues);
            require(rock::RockConfig::parseValues(scopeValues).rockEnableImmersiveScopes == enabled,
                "scope mode did not reach runtime configuration");
        }
        require(find(store, "bImmersiveGrenades").value == "true", "immersive grenades must remain the default");
        require(store.setValue(Group::Consumer, "RealisticWeapons", "bImmersiveGrenades", "false"), "vanilla fallback selection failed");
        require(store.load(false), "vanilla fallback reload failed");
        require(find(store, "bImmersiveGrenades").value == "false", "vanilla fallback selection was lost on reload");
        require(store.setValue(Group::Consumer, "RealisticWeapons", "bImmersiveGrenades", "true"), "immersive mode restore failed");
        require(store.load(false), "immersive restore reload failed");
        require(fs::exists(store.path(Group::Consumer)), "consumer defaults were not created");
        require(!fs::exists(store.path(Group::Developer)), "developer INI must not be created on load");
        CSimpleIniA consumer;
        require(consumer.LoadFile(store.path(Group::Consumer).c_str()) >= 0, "consumer defaults unreadable");
        std::size_t developerCount = 0;
        for (const auto& setting : store.settings()) {
            require(!setting.defaultValue.empty() || setting.type == ValueType::String, "invalid compiled default");
            if (setting.group == Group::Developer) {
                ++developerCount;
                require(consumer.GetValue(setting.section.c_str(), setting.key.c_str(), nullptr) == nullptr,
                    "developer setting leaked into generated consumer INI");
            } else {
                require(consumer.GetValue(setting.section.c_str(), setting.key.c_str(), nullptr) != nullptr,
                    "a supported consumer default is missing from first-run INI");
            }
        }
        require(store.settings().size() == std::size(kSettingMetadata), "presentation catalog differs from supported loader options");
        for (std::size_t index = 0; index < store.settings().size(); ++index) {
            const auto& setting = store.settings()[index];
            const auto& metadata = kSettingMetadata[index];
            require(setting.section == metadata.section && setting.key == metadata.key &&
                setting.group == metadata.group && setting.category == metadata.category,
                "configuration API lost authored section or option order");
        }
        const auto originalConsumer = bytes(store.path(Group::Consumer));
        std::string_view previousCategory;
        std::size_t previousHeading = 0;
        for (const auto& setting : store.settings()) {
            if (setting.group != Group::Consumer || setting.category == previousCategory) continue;
            const auto heading = originalConsumer.find("; === " + setting.category + " ===");
            require(heading != std::string::npos && heading >= previousHeading, "generated consumer sections differ from menu order");
            previousCategory = setting.category;
            previousHeading = heading;
        }
        const auto originalRevision = store.revision();
        require(store.load(true), "repeat load failed");
        require(store.revision() == originalRevision, "unchanged reload was republished");
        require(bytes(store.path(Group::Consumer)) == originalConsumer, "loading rewrote the consumer INI");

        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowColliders", "false"), "default write failed");
        require(!fs::exists(store.path(Group::Developer)), "writing a default created the developer INI");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowColliders", "true"), "override write failed");
        require(fs::exists(store.path(Group::Developer)), "first override did not create the developer INI");
        require(store.load(false), "override reload failed");
        require(find(store, "bDebugShowColliders").value == "true", "override did not load");
        require(store.revision() > originalRevision, "changed file did not advance revision");
        CSimpleIniA developer;
        require(developer.LoadFile(store.path(Group::Developer).c_str()) >= 0, "developer file unreadable");
        require(developer.GetSectionSize("PhysicsInteraction") == 1, "first override wrote unrelated defaults");
        require(!store.setValue(Group::Consumer, "PhysicsInteraction", "bDebugShowColliders", "true"), "wrong owning file accepted");
        require(!store.setValue(Group::Developer, "Debug", "bUnknown", "true"), "unknown setting accepted");
        require(!store.setValue(Group::Developer, "Debug", "bDeveloperModeEnabled", "invalid"), "invalid bool accepted");
        require(!store.setValue(Group::Developer, "PhysicsInteraction", "fDebugVideoSyncMarkerSize", "nan"), "nonfinite override accepted");

        // An editor changes one key after the menu snapshot. A menu change to
        // another key must preserve the newer edit, then both must hot reload.
        require(developer.SetBoolValue("Debug", "bDeveloperModeEnabled", true) >= 0, "external edit setup failed");
        require(developer.SaveFile(store.path(Group::Developer).c_str(), false) >= 0, "external edit failed");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowHandAxes", "true"), "second override failed");
        require(store.load(false), "external+menu reload failed");
        require(find(store, "bDeveloperModeEnabled").value == "true", "menu overwrote an external edit");
        const auto organizedDeveloper = bytes(store.path(Group::Developer));
        const auto controlsHeading = "; === " + find(store, "bDeveloperModeEnabled").category + " ===";
        const auto overlayHeading = "; === " + find(store, "bDebugShowColliders").category + " ===";
        require(organizedDeveloper.find(overlayHeading) != std::string::npos &&
            organizedDeveloper.find(controlsHeading) < organizedDeveloper.find(overlayHeading),
            "sparse developer sections follow edit order instead of menu order");
        require(organizedDeveloper.find(overlayHeading) == organizedDeveloper.rfind(overlayHeading),
            "editing multiple options duplicated a section heading");
        require(find(store, "bDebugShowHandAxes").value == "true", "menu override missing");
        require(bytes(store.path(Group::Consumer)) == originalConsumer, "developer edits rewrote consumer options");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowColliders", "0"), "default restoration failed");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowHandAxes", "false"), "second default restoration failed");
        require(store.setValue(Group::Developer, "Debug", "bDeveloperModeEnabled", "false"), "final default restoration failed");
        require(!fs::exists(store.path(Group::Developer)), "empty developer INI was retained");
        require(store.load(false), "reload after deletion failed");
        require(find(store, "bDebugShowColliders").value == "false", "removed override did not restore default");
        require(find(store, "bDeveloperModeEnabled").value == "false", "deleted developer file retained stale values");

        require(store.setValue(Group::Consumer, "ImmersiveWeapons", "bBipodMode", "false"), "consumer edit failed");
        require(store.load(false), "consumer edit reload failed");
        require(find(store, "bBipodMode").value == "false", "consumer edit did not synchronize");
        require(!fs::exists(store.path(Group::Developer)), "consumer edit created developer INI");
        fs::remove(store.path(Group::Consumer));
        require(store.setValue(Group::Consumer, "ImmersiveWeapons", "bBipodMode", "false"), "edit after consumer deletion failed");
        CSimpleIniA recreated;
        require(recreated.LoadFile(store.path(Group::Consumer).c_str()) >= 0, "consumer INI was not recreated");
        for (const auto& setting : store.settings()) {
            if (setting.group == Group::Consumer) require(recreated.GetValue(setting.section.c_str(), setting.key.c_str(), nullptr), "recreated consumer INI lost defaults");
        }

        const auto beforeFailure = bytes(store.path(Group::Consumer));
        const std::array invalidBatch{ Change{"ImmersiveWeapons", "bBipodMode", "true"}, Change{"Debug", "bDeveloperModeEnabled", "true"} };
        require(!store.setValues(Group::Consumer, invalidBatch), "mixed-owner batch accepted");
        require(bytes(store.path(Group::Consumer)) == beforeFailure, "failed batch partially persisted");

        // Existing developer configurations may deliberately list defaults.
        // Loading them, or editing another key, must preserve those entries.
        CSimpleIniA supplied;
        require(supplied.SetBoolValue("Debug", "bDeveloperModeEnabled", false) >= 0, "supplied developer setup failed");
        require(supplied.SetLongValue("Debug", "iLogSampleMilliseconds", 2000) >= 0, "supplied default setup failed");
        require(supplied.SetValue("Debug", "sLogPattern", "%v", "; My log format") >= 0, "authored comment setup failed");
        require(supplied.SetValue("Local", "sNote", "keep me", "; My local note") >= 0, "external entry setup failed");
        require(supplied.SaveFile(store.path(Group::Developer).c_str(), false) >= 0, "supplied developer save failed");
        const auto suppliedBytes = bytes(store.path(Group::Developer));
        require(store.load(false), "supplied developer file did not load");
        require(bytes(store.path(Group::Developer)) == suppliedBytes, "loading pruned supplied developer defaults");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowHandAxes", "true"), "supplied developer edit failed");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "bDebugShowHandAxes", "false"), "supplied developer reset failed");
        CSimpleIniA retained;
        require(retained.LoadFile(store.path(Group::Developer).c_str()) >= 0, "reset deleted the supplied developer file");
        require(retained.GetValue("Debug", "bDeveloperModeEnabled", nullptr), "reset removed an unrelated supplied default");
        require(retained.GetValue("Debug", "iLogSampleMilliseconds", nullptr), "edit pruned an unrelated supplied default");
        require(std::string_view(retained.GetValue("Local", "sNote", "")) == "keep me", "organization discarded an external entry");
        const auto retainedBytes = bytes(store.path(Group::Developer));
        require(retainedBytes.find("; My log format") != std::string::npos && retainedBytes.find("; My local note") != std::string::npos,
            "organization discarded authored option comments");
        fs::remove(store.path(Group::Developer));
        require(fs::create_directory(store.path(Group::Developer)), "failure directory setup failed");
        require(!store.load(false), "unreadable developer path was accepted");
        require(!store.setValue(Group::Developer, "Debug", "bDeveloperModeEnabled", "true"), "write over directory was accepted");
        require(find(store, "bBipodMode").value == "false", "failed reload lost valid settings");
        std::cout << "Configuration persistence checks passed: " << store.settings().size()
                  << " compiled settings, " << developerCount << " developer settings.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
