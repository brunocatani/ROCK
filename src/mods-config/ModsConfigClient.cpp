#include "mods-config/ModsConfigClient.h"

#include "RockConfig.h"
#include "mods-config/ModsConfigFO4Api.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "setup-wizard/SetupWizard.h"

#include <format>
#include <string>
#include <variant>

namespace rock::mods_config_client
{
    namespace
    {
        using ::mods_config::api::Change;
        using ::mods_config::api::FunctionResult;
        using ::mods_config::api::MenuEventType;
        using ::mods_config::api::MenuState;
        using ::mods_config::api::ModsConfigFO4Client;
        using ::mods_config::api::Params;
        using ::mods_config::api::Saved;
        using ::mods_config::api::Value;

        // The modName of ROCK's config spec.
        constexpr const char* kModName = "ROCK";
        // The function the page's Setup Wizard button runs.
        constexpr const char* kOpenSetupWizard = "openSetupWizard";

        // The spec ids of the rows ROCK shows in the world while the ray is on them.
        constexpr const char* kGripZoneMarkerSizeId = "gripZoneMarkerSize";
        constexpr const char* kFiringGripReachId = "firingGripReach";

        // Game thread: the menu is minimized for the wizard and comes back when the wizard is gone.
        bool g_restorePending = false;
        // Game thread: the row the ray is on, read at FrameBegin.
        PointedPreview g_pointedPreview = PointedPreview::None;

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

        /**
         * A value of the menu as the text ROCK's settings store reads; the store normalizes it, and a transform or a
         * binding comes as the text of its TOML table already.
         */
        std::string valueText(const Value& value)
        {
            return std::visit(
                [](const auto& held) -> std::string {
                    using Held = std::decay_t<decltype(held)>;
                    if constexpr (std::is_same_v<Held, bool>)
                        return held ? "true" : "false";
                    else if constexpr (std::is_same_v<Held, std::string>)
                        return held;
                    else
                        return std::format("{}", held);
                },
                value);
        }

        /**
         * A live setting's preview, or its Discard back to the file's value, kept by RockConfig for the next
         * FrameBegin. The setting's TOML path is its tables, then its key.
         */
        void onChange(const Change& change)
        {
            if (change.filePath.empty() || change.path.size() < 2)
                return;
            std::string table;
            for (std::size_t index = 0; index + 1 < change.path.size(); ++index) {
                if (index)
                    table += '.';
                table += change.path[index];
            }
            const auto key = change.path.back();
            if (change.discard)
                g_rockConfig.discardPreviewValue(table, key);
            else
                (void)g_rockConfig.setPreviewValue(table, key, valueText(change.value));
        }

        /**
         * A Save wrote the files: the previews end, and the files are read at the next FrameBegin instead of after
         * the watcher's debounce, so the saved values follow the previews without a frame of the old ones.
         */
        void onSaved(const Saved&)
        {
            g_rockConfig.clearPreviewValues();
            g_rockConfig.requestImmediateReload();
        }

        /**
         * Mods Config's F4SE message after a Save, sent to each plugin of the spec's `f4sePluginRequirements`. ROCK
         * learns of Saves through `onSaved`, so it ignores the message; without a listener F4SE logs a failed dispatch.
         */
        void onModsConfigMessage(F4SE::MessagingInterface::Message*) {}

        /**
         * The menu closing ends every preview, should a Discard not have come for one.
         */
        void onMenuEvent(MenuEventType event, std::string_view)
        {
            if (event == MenuEventType::MenuClosed)
                g_rockConfig.clearPreviewValues();
        }
    }

    void connect() noexcept
    {
        try {
            auto& client = ModsConfigFO4Client::get();
            client.addFunction(kOpenSetupWizard, &openSetupWizard);
            client.addState("modVersion", [] { return std::optional<Value>(std::string(Version::NAME)); });
            client.subscribeToChanges(&onChange);
            client.subscribeToSaves(&onSaved);
            client.subscribeToMenuEvents(&onMenuEvent);
            if (const auto* messaging = F4SE::GetMessagingInterface())
                (void)messaging->RegisterListener(&onModsConfigMessage, ::mods_config::api::MODS_CONFIG_FO4_F4SE_NAME);
            if (const int error = client.connect(kModName); error != 0)
                ROCK_LOG_INFO(Config, "Mods Config FO4 VR not connected ({}): ROCK runs without its native API", error);
            else
                ROCK_LOG_INFO(Config, "Connected to Mods Config FO4 VR (API {})", client.apiVersion());
        } catch (...) {
            ROCK_LOG_WARN(Config, "Connecting to Mods Config FO4 VR failed");
        }
    }

    /**
     * Every frame: tells the input remap whether the menu is open and shown (not minimized), when its pointer takes
     * the primary trigger; reads which previewed row the ray is on; and restores the menu once the setup wizard it
     * stepped aside for is gone.
     */
    void update() noexcept
    {
        auto& client = ModsConfigFO4Client::get();
        const auto* api = client.api();
        input_remap_runtime::setPointerMenuInUse(api && api->getMenuState() == MenuState::Open);
        g_pointedPreview = client.isPointedAt(kGripZoneMarkerSizeId) ? PointedPreview::GripZoneMarkerSize
            : client.isPointedAt(kFiringGripReachId)                 ? PointedPreview::FiringGripReach
                                                                     : PointedPreview::None;

        if (!g_restorePending || setup_wizard::isActive())
            return;
        g_restorePending = false;
        ModsConfigFO4Client::get().restoreMenu();
        ROCK_LOG_INFO(Config, "Setup wizard closed: Mods Config restored");
    }

    PointedPreview pointedPreview() noexcept { return g_pointedPreview; }
}
