#include "setup-wizard/SetupWizardSettings.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::setup_wizard::settings
{
    SessionStart read()
    {
        SessionStart start{};
        start.bindings = g_rockConfig.rockFunctionBindings;
        start.options = optionPages();
        (void)g_rockConfig.visitSettings(api::configuration::Group::Consumer, collectOption, &start.options);
        std::erase_if(start.options, [](const OptionState& option) { return !option.found; });
        return start;
    }

    SaveResult write(const std::vector<SettingWrite>& writes)
    {
        SaveResult result{};
        for (const auto& value : writes) {
            std::string error;
            if (g_rockConfig.persistSetting(api::configuration::Group::Consumer, value.section.c_str(), value.key.c_str(),
                    value.value.c_str(), error)) {
                result.written.push_back(value);
                continue;
            }
            result.errors.push_back(value.section + "." + value.key + ": " + error);
            ROCK_LOG_WARN(Config, "Setup wizard could not save {}.{}: {}", value.section, value.key, error);
        }
        ROCK_LOG_INFO(Config, "Setup wizard saved ({} errors)", result.errors.size());
        return result;
    }

    bool keepSteamVrBinding(std::string& error)
    {
        if (!g_rockConfig.persistSetting(api::configuration::Group::Consumer, "ControllerInput", "OfferSteamVRBinding",
                "false", error)) {
            ROCK_LOG_WARN(Config, "Setup wizard could not save ControllerInput.OfferSteamVRBinding: {}", error);
            return false;
        }
        ROCK_LOG_INFO(Config, "Setup wizard: the player keeps SteamVR's binding; no automatic offer from now on");
        return true;
    }
}
