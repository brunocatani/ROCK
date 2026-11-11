#include "PreviewBackend.h"

#include <spdlog/sinks/ringbuffer_sink.h>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/ResourceUtils.h"

namespace
{
    std::filesystem::path g_directory;
    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> g_log;

    void startLog()
    {
        if (g_log) return;
        g_log = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(400);
        g_log->set_pattern("%H:%M:%S.%e [%l] %v");
        auto logger = std::make_shared<spdlog::logger>("ROCK", g_log);
        logger->set_level(spdlog::level::info);
        rock::logger::internal::loggerInstance = std::move(logger);
    }
}

// The settings store resolves its folder through Documents; the preview gives it the
// sandbox instead, so nothing it writes can reach the game's settings.
namespace rock::resources
{
    std::string getPathInDocuments(const std::string&) { return g_directory.string(); }
}

namespace rock::setup_wizard::preview
{
    void startSettings(const std::filesystem::path& directory)
    {
        startLog();
        g_directory = directory;
        std::filesystem::create_directories(g_directory);
        g_rockConfig.load();
    }

    std::filesystem::path settingsDirectory() { return g_rockConfig.getConfigDirectory(); }

    void pumpSettings() { g_rockConfig.processPendingConfigReload(); }

    void reloadSettings() { g_rockConfig.reload(); }

    bool resetSettings(std::string& error)
    {
        std::error_code failure;
        for (const auto* name : { "ROCK.toml", "ROCK_Developer.toml" }) {
            std::filesystem::remove(g_directory / name, failure);
            if (failure) {
                error = std::string("Cannot remove ") + name + ": " + failure.message();
                return false;
            }
        }
        g_rockConfig.load();
        ROCK_LOG_INFO(Config, "Preview: sandbox settings reset to ROCK's defaults");
        return true;
    }

    bool copySettings(const std::filesystem::path& file, std::string& error)
    {
        std::error_code failure;
        std::filesystem::copy_file(file, g_directory / "ROCK.toml", std::filesystem::copy_options::overwrite_existing,
            failure);
        if (failure) {
            error = "Cannot copy " + file.string() + ": " + failure.message();
            return false;
        }
        g_rockConfig.reload();
        ROCK_LOG_INFO(Config, "Preview: sandbox settings copied from {}", file.string());
        return true;
    }

    std::vector<std::string> logLines() { return g_log ? g_log->last_formatted() : std::vector<std::string>{}; }

    void clearLog()
    {
        // The ring sink has no clear: a fresh sink replaces it.
        g_log.reset();
        rock::logger::internal::loggerInstance.reset();
        startLog();
    }
}
