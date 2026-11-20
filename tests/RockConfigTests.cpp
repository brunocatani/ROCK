// RockConfig's settings files on a temporary Documents root: visiting and
// persisting settings (the setup wizard's path), reload, concurrent writes to
// one file, a file that stops parsing, and reentrant change subscribers. The
// catalog, loader, store and file writes are production code; no test
// operation can reach production settings files.
#include <barrier>
#include <cassert>
#include <fstream>
#include <future>
#include <latch>
#include <map>
#include <thread>
#include "RockConfig.h"

namespace
{
    using rock::config::Group;
    std::filesystem::path directory;
    // Keyed "table.key".
    using Catalog = std::map<std::string, std::string>;

    void collect(const rock::config::Setting& setting, void* context) noexcept
    {
        static_cast<Catalog*>(context)->emplace(setting.table + "." + setting.key, setting.value);
    }

    Catalog catalog(Group group)
    {
        Catalog result;
        assert(rock::g_rockConfig.visitSettings(group, collect, &result));
        assert(!result.empty());
        return result;
    }

    bool set(Group group, const char* section, const char* key, const char* value)
    {
        std::string error;
        return rock::g_rockConfig.persistSetting(group, section, key, value, error);
    }
}

// Replace only the Documents root.
namespace rock::resources
{
    std::string getPathInDocuments(const std::string&)
    {
        assert(!directory.empty());
        return directory.string();
    }
}

int main()
{
    namespace fs = std::filesystem;
    using rock::g_rockConfig;
    directory = fs::temp_directory_path() / std::format("ROCK-config-{}-{}", GetCurrentProcessId(), GetTickCount64());
    fs::create_directory(directory);

    struct Cleanup
    {
        ~Cleanup()
        {
            g_rockConfig.stopFileWatch();
            assert(fs::equivalent(directory.parent_path(), fs::temp_directory_path()) &&
                directory.filename().string().starts_with("ROCK-config-"));
            std::error_code error;
            fs::remove_all(directory, error);
        }
    } cleanup;

    const auto playerFile = directory / "ROCK.toml";
    const auto developerFile = directory / "ROCK_Developer.toml";
    Catalog missing;
    assert(!g_rockConfig.visitSettings(Group::Player, collect, &missing));
    g_rockConfig.load();
    g_rockConfig.stopFileWatch(); // Keep reload timing deterministic; writes still request it.
    assert(fs::exists(playerFile));
    assert(g_rockConfig.rockFarGrabStyle == 1);
    assert(catalog(Group::Player).at("Grabbing.FarGrabStyle") == "1");
    assert(!catalog(Group::Developer).contains("Grabbing.FarGrabStyle"));
    assert(set(Group::Player, "Grabbing", "FarGrabStyle", "2"));
    g_rockConfig.reload();
    assert(g_rockConfig.rockFarGrabStyle == 2);
    assert(set(Group::Player, "Grabbing", "FarGrabStyle", "99"));
    g_rockConfig.reload();
    assert(g_rockConfig.rockFarGrabStyle == 1);
    assert(set(Group::Player, "Grabbing", "FarGrabStyle", "1"));
    g_rockConfig.reload();
    // Identity is the exact table and key; the old INI spellings are not aliases.
    assert(!set(Group::Player, "PhysicsInteraction", "iFarGrabMode", "2"));
    assert(!set(Group::Player, "grabbing", "FarGrabStyle", "2"));
    // NPC options belong to SCISSORS.
    for (const auto* key : { "NpcDynamicCollisions", "BladePenetrationEnabled", "DebugDrawNpcNativeColliders",
             "DebugDrawNpcSupplementColliders", "DebugDrawNpcColliderLabels" }) {
        for (const auto group : { Group::Player, Group::Developer }) {
            for (const auto& [path, value] : catalog(group))
                assert(!path.ends_with(std::string(".") + key));
            assert(!set(group, "DebugOverlay", key, "true"));
        }
    }
    const auto initialRevision = g_rockConfig.configRevision();
    assert(initialRevision && !fs::exists(developerFile));
    const auto originalLogLevel = g_rockConfig.rockLogLevel;

    // A write from another thread saves at once and applies at the next reload.
    std::async(std::launch::async, [&] {
        assert(catalog(Group::Player).contains("Logging.LogLevel"));
        assert(catalog(Group::Developer).at("DeveloperControls.DeveloperModeEnabled") == "false");
        assert(set(Group::Player, "Logging", "LogLevel", "3"));
        assert(set(Group::Developer, "DeveloperControls", "DeveloperModeEnabled", "true"));
        assert(g_rockConfig.configRevision() == initialRevision);
    }).get();
    assert(g_rockConfig.rockLogLevel == originalLogLevel && !g_rockConfig.rockDeveloperModeEnabled);
    assert(fs::exists(developerFile));
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel == 3 && g_rockConfig.rockDeveloperModeEnabled);
    assert(g_rockConfig.configRevision() > initialRevision);

    // Two simultaneous edits in one file must preserve each other.
    std::barrier start(3);
    auto first = std::async(std::launch::async, [&] {
        start.arrive_and_wait();
        return set(Group::Developer, "DeveloperControls", "DeveloperModeEnabled", "false");
    });
    auto second = std::async(std::launch::async, [&] {
        start.arrive_and_wait();
        return set(Group::Developer, "SkeletonDebug", "DebugSkeletonBoneAxisLength", "8");
    });
    start.arrive_and_wait();
    assert(first.get() && second.get());
    g_rockConfig.reload();
    const auto developer = catalog(Group::Developer);
    assert(developer.at("DeveloperControls.DeveloperModeEnabled") == "false" &&
        developer.at("SkeletonDebug.DebugSkeletonBoneAxisLength") == "8.0");

    // Hold a borrowed setting while a reload becomes pending. Reload must
    // return without waiting and apply after the visitor finishes.
    struct Borrow
    {
        std::latch entered{ 1 }, finish{ 1 };
        bool first = true;
    } borrow;

    auto reader = std::async(std::launch::async, [&] {
        return g_rockConfig.visitSettings(
            Group::Player,
            +[](const rock::config::Setting& setting, void* opaque) noexcept {
                auto& b = *static_cast<Borrow*>(opaque);
                if (!b.first)
                    return;
                b.first = false;
                const std::string retained = setting.value;
                b.entered.count_down();
                b.finish.wait();
                assert(retained == setting.value);
            },
            &borrow);
    });
    borrow.entered.wait();
    {
        std::ofstream file(playerFile);
        file << "[Logging]\nLogLevel = 4\n";
    }
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel == 3);
    borrow.finish.count_down();
    assert(reader.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(210)); // Existing reload debounce.
    g_rockConfig.processPendingConfigReload();
    assert(g_rockConfig.rockLogLevel == 4);

    // A file that stops parsing keeps the last applied values, refuses writes
    // that would overwrite it, and applies again once it is fixed.
    {
        std::ofstream file(playerFile);
        file << "[Logging]\nLogLevel = 2\n[Grabbing\n";
    }
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel == 4);
    assert(!set(Group::Player, "Logging", "LogLevel", "1"));
    {
        std::ofstream file(playerFile);
        file << "[Logging]\nLogLevel = 2\n";
    }
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel == 2);

    // Reentrant subscribers run after releasing the store. The developer file
    // goes once its last option is back at its default.
    bool notified = false;
    g_rockConfig.subscribeForConfigChanged("test", [&](const std::string&) {
        notified = true;
        assert(catalog(Group::Player).at("Logging.LogLevel") == "2");
    });
    assert(set(Group::Developer, "SkeletonDebug", "DebugSkeletonBoneAxisLength", "4"));
    assert(!fs::exists(developerFile));
    g_rockConfig.reload();
    assert(notified);
    g_rockConfig.unsubscribeFromConfigChanged("test");
}
