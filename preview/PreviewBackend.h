#pragma once

#include <filesystem>
#include <string>
#include <vector>

/*
 * The preview's side of ROCK's own code: the real settings store
 * (g_rockConfig) over a sandbox folder, and ROCK's logger into memory.
 * Standard library only, so the preview's window code never includes
 * CommonLib; PreviewBackend.cpp is built with ROCK's settings sources.
 * Preview thread only.
 */
namespace rock::setup_wizard::preview
{
    // Points ROCK's settings store at `directory`, creating it, and loads it: ROCK.toml
    // is written with ROCK's defaults when it is missing, as in the game.
    void startSettings(const std::filesystem::path& directory);

    [[nodiscard]] std::filesystem::path settingsDirectory();

    // Every frame: applies saved and edited files after the store's settle time, as the
    // game's frame does.
    void pumpSettings();

    // Reads the files now, so a session opened right after a save sees it.
    void reloadSettings();

    // Removes the sandbox's ROCK.toml and ROCK_Developer.toml and loads ROCK's defaults.
    [[nodiscard]] bool resetSettings(std::string& error);

    // Copies `file` over the sandbox's ROCK.toml and loads it. `file` is only read.
    [[nodiscard]] bool copySettings(const std::filesystem::path& file, std::string& error);

    // ROCK's log since the last clear, oldest first.
    [[nodiscard]] std::vector<std::string> logLines();
    void clearLog();
}
