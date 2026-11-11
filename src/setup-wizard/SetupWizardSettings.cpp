#include "setup-wizard/SetupWizardSettings.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::setup_wizard::settings
{
    SessionStart read()
    {
        SessionStart start{};
        start.bindings = g_rockConfig.rockFunctionBindings;
        (void)g_rockConfig.visitSettings(api::configuration::Group::Consumer, collectSettings, &start);
        return start;
    }

    void write(const std::vector<SettingWrite>& writes, FinishResult& result)
    {
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
        if (!writes.empty()) ROCK_LOG_INFO(Config, "Setup wizard saved {} values ({} errors)", writes.size(), result.errors.size());
    }
}
