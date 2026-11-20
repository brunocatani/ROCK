#include "mods-config/ModsConfigClient.h"

#include "mods-config/ModsConfigFO4Api.h"
#include "physics-interaction/PhysicsLog.h"
#include "setup-wizard/SetupWizard.h"

namespace rock::mods_config_client
{
    namespace
    {
        using ::mods_config::api::FunctionResult;
        using ::mods_config::api::ModsConfigFO4Client;
        using ::mods_config::api::Params;

        // The modName of ROCK's config spec.
        constexpr const char* kModName = "ROCK";
        // The function the page's Setup Wizard button runs.
        constexpr const char* kOpenSetupWizard = "openSetupWizard";

        // Game thread: the menu is minimized for the wizard and comes back when the wizard is gone.
        bool g_restorePending = false;

        /**
         * Opens the setup wizard (or takes the one already up) and minimizes the menu for it. Fails only without
         * RPS UI Framework, which draws the wizard.
         */
        FunctionResult openSetupWizard(const Params&)
        {
            if (!setup_wizard::isActive() && !setup_wizard::requestOpen())
                return { .done = false, .message = "The setup wizard needs RPS UI Framework." };
            ModsConfigFO4Client::get().minimizeMenu();
            g_restorePending = true;
            ROCK_LOG_INFO(Config, "Setup wizard requested from Mods Config");
            return { .done = true };
        }
    }

    void connect() noexcept
    {
        try {
            auto& client = ModsConfigFO4Client::get();
            client.addFunction(kOpenSetupWizard, &openSetupWizard);
            if (const int error = client.connect(kModName); error != 0)
                ROCK_LOG_INFO(Config, "Mods Config FO4 VR not connected ({}): ROCK runs without its native API", error);
            else
                ROCK_LOG_INFO(Config, "Connected to Mods Config FO4 VR (API {})", client.apiVersion());
        } catch (...) {
            ROCK_LOG_WARN(Config, "Connecting to Mods Config FO4 VR failed");
        }
    }

    void update() noexcept
    {
        if (!g_restorePending || setup_wizard::isActive())
            return;
        g_restorePending = false;
        ModsConfigFO4Client::get().restoreMenu();
        ROCK_LOG_INFO(Config, "Setup wizard closed: Mods Config restored");
    }
}
