#include "RockConfig.h"
#include "config/SettingMetadata.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <map>
#include <set>

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
        throw std::runtime_error("Missing compiled setting: " + std::string(key));
    }

    void decorativeSectionChecks(const std::filesystem::path& directory, const CSimpleIniA& compiled)
    {
        using namespace rock::config;
        namespace fs = std::filesystem;
        fs::create_directories(directory);
        ConfigurationStore store(directory, compiled);
        const auto write = [&](Group group, const std::string& text) {
            std::ofstream file(store.path(group), std::ios::binary | std::ios::trunc);
            file << text;
            require(file.good(), "cannot write decorative-section fixture");
        };
        const auto runtime = [&] {
            CSimpleIniA combined;
            store.appendLoadedValues(combined);
            return rock::RockConfig::parseValues(combined);
        };
        std::set<CSimpleIniA::Entry, CSimpleIniA::Entry::KeyOrder> uniqueKeys;
        std::map<std::string, std::string> expected;
        CSimpleIniA canonical;
        std::array<std::string, 2> flat, labelled;
        std::array<std::size_t, 2> counts{};
        for (const auto& setting : store.settings()) {
            require(uniqueKeys.emplace(setting.key.c_str()).second,
                "decorative sections require globally unique case-insensitive setting names");
            const auto group = setting.group == Group::Consumer ? 0u : 1u;
            const std::string value = setting.type == ValueType::Boolean ? (setting.defaultValue == "true" ? "false" : "true") :
                setting.type == ValueType::Integer ? (setting.defaultValue == "7" ? "8" : "7") :
                setting.type == ValueType::Float ? "0.625" : "custom-value";
            expected.emplace(setting.key, value);
            canonical.SetValue(setting.section.c_str(), setting.key.c_str(), value.c_str());
            std::string key = setting.key;
            if (counts[group] % 2 == 0)
                for (auto& c : key) if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
            const auto line = key + " = " + value + "\n";
            flat[group] += line;
            // Initial keys have no header; later groups repeat in nonalphabetic order.
            if (counts[group] % 7 == 3)
                labelled[group] += counts[group] % 2 == 0 ? "[Alpha notes]\n" : "[Zulu notes]\n";
            labelled[group] += line;
            ++counts[group];
        }
        for (const auto& files : { flat, labelled }) {
            write(Group::Consumer, files[0]);
            write(Group::Developer, files[1]);
            require(store.load(false), "section-independent load failed");
            for (const auto& setting : store.settings())
                require(setting.specified && setting.value == expected.at(setting.key),
                    "a known key was lost or changed outside its catalog section");
            require(runtime() == rock::RockConfig::parseValues(canonical),
                "decorative labels changed parsed runtime values");
            require(bytes(store.path(Group::Consumer)) == files[0] && bytes(store.path(Group::Developer)) == files[1],
                "loading decorative sections rewrote a user file");
        }
        const auto revision = store.revision();
        write(Group::Consumer, flat[0]);
        write(Group::Developer, flat[1]);
        require(store.load(false) && store.revision() == revision,
            "moving unchanged keys between labels caused a semantic reload");

        // Section names cannot redirect an option into the other owning file.
        write(Group::Consumer, "[Debug]\nbPerformanceProfilerEnabled=true\n");
        write(Group::Developer, "[Logging]\niLogLevel=0\n");
        require(store.load(false), "wrong-file fixture failed to load");
        require(!find(store, "bPerformanceProfilerEnabled").specified && runtime().rockLogLevel == 2,
            "decorative sections broke consumer/developer ownership");
        require(settingGroup("Any label", "BPERFORMANCEPROFILERENABLED") == Group::Developer,
            "key-only ownership lookup still depends on a section or spelling case");

        write(Group::Developer,
            "bPerformanceProfilerEnabled=false\n"
            "[Zulu]\nbPerformanceProfilerEnabled=true\n"
            "[Alpha]\nbPerformanceProfilerEnabled=false\n"
            "[Zulu]\n; My selected profiler value\nBPERFORMANCEPROFILERENABLED=true\n"
            "iPerformanceProfilerLogIntervalFrames=450\n"
            "[Alpha]\niPerformanceProfilerLogIntervalFrames=600\niPerformanceProfilerLogIntervalFrames=750\n"
            "bDebugShowColliders=false\n; My local note\nsUserNote=keep me\n");
        require(store.load(false), "duplicate fixture failed to load");
        require(runtime().rockPerformanceProfilerEnabled && runtime().rockPerformanceProfilerLogIntervalFrames == 750,
            "last physical occurrence did not win across repeated headers/keys");
        require(store.setValue(Group::Developer, "Unrelated menu label", "BDEBUGSHOWHANDAXES", "true"),
            "case-insensitive key-only menu edit failed");
        require(store.load(false) && runtime().rockPerformanceProfilerEnabled &&
                runtime().rockPerformanceProfilerLogIntervalFrames == 750,
            "organizing an unrelated edit resurrected a shadowed value");
        require(find(store, "bDebugShowColliders").specified && !runtime().rockDebugShowColliders,
            "an unrelated explicit developer default was pruned");
        const auto organized = bytes(store.path(Group::Developer));
        require(organized.find("; My selected profiler value") != std::string::npos &&
                organized.find("; My local note") != std::string::npos && organized.find("keep me") != std::string::npos,
            "writing discarded the effective option comment or an unknown entry");
        require(store.setValue(Group::Developer, "Another label", "iperformanceprofilerlogintervalframes", "900"),
            "editing a previously duplicated key failed");
        require(store.load(false) && runtime().rockPerformanceProfilerLogIntervalFrames == 900,
            "an older duplicate overrode the menu edit");
        require(store.setValue(Group::Developer, "Anything", "bPerformanceProfilerEnabled", "false"),
            "resetting a misplaced key failed");
        require(store.load(false) && !runtime().rockPerformanceProfilerEnabled &&
                !find(store, "bPerformanceProfilerEnabled").specified,
            "reset left a stale profiler override in another section");

        write(Group::Developer, "bPerformanceProfilerEnabled=true\n[One]\nbPerformanceProfilerEnabled=true\n[Two]\nBPERFORMANCEPROFILERENABLED=true\n");
        require(store.setValue(Group::Developer, "", "bPerformanceProfilerEnabled", "false"),
            "global reset of repeated keys failed");
        require(!fs::exists(store.path(Group::Developer)), "reset did not delete an empty developer file");
        // This is the production profiler's formerly ignored placement.
        write(Group::Developer, "[PhysicsInteraction]\nbPerformanceProfilerEnabled=true\niPerformanceProfilerLogIntervalFrames=300\niPerformanceProfilerWarmupFrames=120\nbPerformanceProfilerOverlayText=false\n");
        require(store.load(false), "production-layout profiler reload failed");
        const auto profiler = runtime();
        require(profiler.rockPerformanceProfilerEnabled && profiler.rockPerformanceProfilerLogIntervalFrames == 300 &&
                profiler.rockPerformanceProfilerWarmupFrames == 120 && !profiler.rockPerformanceProfilerOverlayText,
            "the real misplaced profiler configuration still does not activate");
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
        if (argc == 3 && std::string_view(argv[1]) == "--dump-defaults") {
            require(compiled.SaveFile(argv[2], false) >= 0, "compiled defaults dump failed");
            return 0;
        }
        const auto directory = fs::temp_directory_path() /
            ("ROCK-configuration-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        struct Cleanup { fs::path path; ~Cleanup() { std::error_code ec; fs::remove_all(path, ec); } } cleanup{ directory };
        decorativeSectionChecks(directory / "decorative", compiled);
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
        require(find(store, "npcDynamicCollisions").type == ValueType::Boolean &&
            find(store, "npcDynamicCollisions").group == Group::Developer &&
            !rock::RockConfig::parseValues(missingOptions).npcDynamicCollisions,
            "NPC dynamic collisions must be a developer boolean defaulting off");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "npcDynamicCollisions", "true"),
            "experimental NPC collision enable failed");
        require(store.load(false), "experimental NPC collision reload failed");
        CSimpleIniA npcValues;
        store.appendLoadedValues(npcValues);
        require(rock::RockConfig::parseValues(npcValues).npcDynamicCollisions,
            "experimental NPC collision override did not reach runtime");
        require(store.setValue(Group::Developer, "PhysicsInteraction", "npcDynamicCollisions", "false"),
            "experimental NPC collision reset failed");
        require(store.load(false), "experimental NPC collision reset reload failed");
        CSimpleIniA resetNpcValues;
        store.appendLoadedValues(resetNpcValues);
        require(!rock::RockConfig::parseValues(resetNpcValues).npcDynamicCollisions &&
            !fs::exists(store.path(Group::Developer)),
            "reset must restore NPC collisions off and remove the sole developer override");
        require(find(store, "fLaserRecoilPercent").type == ValueType::Float &&
            rock::RockConfig::parseValues(missingOptions).rockLaserRecoilPercent == 100.0f,
            "laser recoil must default to the bipod profile strength");
        for (const auto* percent : { "0", "50", "100", "300" }) {
            require(store.setValue(Group::Consumer, "ImmersiveWeapons", "fLaserRecoilPercent", percent),
                "laser recoil strength write failed");
            require(store.load(false), "laser recoil reload failed");
            CSimpleIniA recoilValues;
            store.appendLoadedValues(recoilValues);
            const auto recoilConfig = rock::RockConfig::parseValues(recoilValues);
            require(recoilConfig.rockLaserRecoilPercent == std::stof(percent),
                "laser recoil setting did not reach runtime configuration");
            require(recoilConfig.rockRifleOneHandRecoilPercent == 300.0f &&
                recoilConfig.rockRifleTwoHandRecoilPercent == 80.0f,
                "laser tuning changed ordinary rifle recoil");
        }
        for (const auto& [value, expected] : { std::pair{ "-1", 0.0f }, { "301", 300.0f }, { "nan", 100.0f } }) {
            CSimpleIniA recoilValues;
            recoilValues.SetValue("ImmersiveWeapons", "fLaserRecoilPercent", value);
            require(rock::RockConfig::parseValues(recoilValues).rockLaserRecoilPercent == expected,
                "invalid laser recoil strength was not bounded safely");
        }
        require(store.setValue(Group::Consumer, "ImmersiveWeapons", "fLaserRecoilPercent", "100"),
            "laser recoil strength reset failed");
        require(store.load(false), "laser recoil reset reload failed");
        require(find(store, "iWeaponDropMode").type == ValueType::Integer &&
            find(store, "iWeaponDropMode").value == "1", "weapon drop must default to off");
        require(find(store, "bKeepPreviousWeaponInHandOnEquip").value == "false",
            "previous weapon retention must be opt-in");
        for (const auto* enabled : { "true", "false" }) {
            require(store.setValue(Group::Consumer, "ImmersiveWeapons", "bKeepPreviousWeaponInHandOnEquip", enabled),
                "previous weapon retention write failed");
            require(store.load(false), "previous weapon retention reload failed");
            CSimpleIniA swapValues;
            store.appendLoadedValues(swapValues);
            const auto swapConfig = rock::RockConfig::parseValues(swapValues);
            require(swapConfig.rockKeepPreviousWeaponInHandOnEquip == (enabled[0] == 't') &&
                swapConfig.rockWeaponDropMode == 1 && swapConfig.rockWeaponGrabMode == 1,
                "retention reload must remain independent of weapon drop and grab modes");
        }
        require(!store.setValue(Group::Consumer, "ImmersiveWeapons", "bAutoDrop", "true"),
            "removed auto-drop boolean must not remain writable");
        for (const auto* mode : { "1", "2", "3" }) {
            require(store.setValue(Group::Consumer, "ImmersiveWeapons", "iWeaponDropMode", mode),
                "weapon drop mode write failed");
            require(store.load(false), "weapon drop mode reload failed");
            CSimpleIniA dropValues;
            store.appendLoadedValues(dropValues);
            const auto dropConfig = rock::RockConfig::parseValues(dropValues);
            require(dropConfig.rockWeaponDropMode == mode[0] - '0',
                "weapon drop mode did not reach runtime configuration");
            require(dropConfig.rockWeaponGrabMode == 1,
                "weapon drop mode must not change the grip release gesture");
        }
        for (const auto* invalid : { "-1", "0", "4", "invalid" }) {
            CSimpleIniA dropValues;
            dropValues.SetValue("ImmersiveWeapons", "iWeaponDropMode", invalid);
            require(rock::RockConfig::parseValues(dropValues).rockWeaponDropMode == 1,
                "invalid weapon drop mode must fall back to off");
        }
        require(store.setValue(Group::Consumer, "ImmersiveWeapons", "iWeaponDropMode", "1"),
            "weapon drop mode reset failed");
        require(store.load(false), "weapon drop mode reset reload failed");
        require(store.setValue(Group::Consumer, "AmbidextrousFiring", "fLeftFiringGripOffsetYGameUnits", "0.25"),
            "left firing relative placement write failed");
        require(store.setValue(Group::Consumer, "AmbidextrousFiring", "fRightSupportGripOffsetZGameUnits", "-0.5"),
            "right support relative placement write failed");
        require(store.load(false), "relative grip placement reload failed");
        CSimpleIniA gripPlacementValues;
        store.appendLoadedValues(gripPlacementValues);
        const auto gripPlacement = rock::RockConfig::parseValues(gripPlacementValues);
        require(gripPlacement.rockLeftFiringGripOffsetGameUnits == RE::NiPoint3(0.0f, 0.25f, 0.0f) &&
            gripPlacement.rockRightSupportGripOffsetGameUnits == RE::NiPoint3(0.0f, 0.0f, -0.5f),
            "relative firing and support placement must remain independent across reload");
        require(gripPlacement.rockLeftFiringAimOffsetYGameUnits == 0.0f && gripPlacement.rockLeftFiringAimYawDegrees == 0.0f,
            "relative grip placement must not change whole-carry position or aim");
        require(store.setValue(Group::Consumer, "AmbidextrousFiring", "fLeftFiringGripOffsetYGameUnits", "0"), "left placement reset failed");
        require(store.setValue(Group::Consumer, "AmbidextrousFiring", "fRightSupportGripOffsetZGameUnits", "0"), "right placement reset failed");
        require(store.load(false), "relative grip placement reset reload failed");
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
