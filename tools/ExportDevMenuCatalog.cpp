#include "RockConfig.h"

#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
    try {
        if (argc != 2) throw std::runtime_error("Expected catalog output path");
        CSimpleIniA defaults;
        rock::RockConfig::compiledDefaults(defaults);
        nlohmann::json entries = nlohmann::json::array();
        CSimpleIniA::TNamesDepend sections;
        defaults.GetAllSections(sections);
        for (const auto& section : sections) {
            CSimpleIniA::TNamesDepend keys;
            defaults.GetAllKeys(section.pItem, keys);
            for (const auto& entry : keys) {
                const std::string key(entry.pItem);
                const std::string raw(defaults.GetValue(section.pItem, entry.pItem, ""));
                nlohmann::json value = raw;
                if (key.starts_with('b')) value = defaults.GetBoolValue(section.pItem, entry.pItem);
                else if (key.starts_with('i')) value = defaults.GetLongValue(section.pItem, entry.pItem);
                else if (key.starts_with('f')) value = defaults.GetDoubleValue(section.pItem, entry.pItem);
                std::string group = "General";
                // ROCK owns its menu organization; DevMenu only renders these groups.
                if (key.find("Debug") != std::string::npos) group = "Debug visualizations and diagnostics";
                else if (key.find("Profiler") != std::string::npos || key.find("Log") != std::string::npos || key.find("Developer") != std::string::npos) group = "Developer mode, logging and profiling";
                else if (key.find("Haptic") != std::string::npos) group = "Haptics";
                else if (key.find("Mouth") != std::string::npos) group = "Mouth consumption";
                else if (key.find("Shoulder") != std::string::npos) group = "Shoulder stash";
                else if (key.find("Finger") != std::string::npos || key.find("Pinch") != std::string::npos) group = "Fingers and pinch grabs";
                else if (key.find("Pull") != std::string::npos || key.find("ForceGrab") != std::string::npos) group = "Pull and force grab";
                else if (key.find("Selection") != std::string::npos || key.find("Highlight") != std::string::npos) group = "Selection and highlighting";
                else if (key.find("Collision") != std::string::npos || key.find("Collider") != std::string::npos) group = "Collision";
                else if (key.find("Grab") != std::string::npos || key.find("Grip") != std::string::npos) group = "Grabbing and grips";
                const std::string page = std::string(section.pItem) == "PhysicsInteraction" ||
                    group.starts_with("Debug") || group.starts_with("Developer") ? group : section.pItem;
                entries.push_back({{"section", section.pItem}, {"key", key}, {"default", value}, {"group", group}, {"page", page}});
            }
        }
        if (entries.empty()) throw std::runtime_error("Compiled loader catalog is empty");
        const std::filesystem::path path(argv[1]);
        std::filesystem::create_directories(path.parent_path());
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << nlohmann::json{{"version", 1}, {"entries", entries}}.dump(2) << '\n';
        output.close();
        if (!output) throw std::runtime_error("Cannot write compiled settings catalog");
        std::cout << "Exported " << entries.size() << " compiled ROCK settings\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
