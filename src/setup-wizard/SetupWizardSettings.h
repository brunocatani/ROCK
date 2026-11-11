#pragma once

#include <vector>

#include "setup-wizard/SetupWizardPanel.h"

/*
 * The setup wizard's reads and writes of ROCK's settings, through the settings
 * store (g_rockConfig) like every other change. Shared by the game's host and
 * the desktop preview, which runs the store over a sandbox folder.
 *
 * Host thread. None of these is called while the panel's lock is held.
 */
namespace rock::setup_wizard::settings
{
    // The player's bindings, the wizard's options and the binding offer as the store holds
    // them now. The SteamVR state is the host's to add.
    [[nodiscard]] SessionStart read();

    // Writes each value into `result`; the store reloads and ROCK applies them.
    void write(const std::vector<SettingWrite>& writes, FinishResult& result);
}
