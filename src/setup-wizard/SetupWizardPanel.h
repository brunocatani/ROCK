#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ROCK/Configuration.h>
#include <RPS_UI_Framework/Framework.h>

#include "physics-interaction/input/FunctionBindingPolicy.h"
#include "setup-wizard/SetupWizardPolicy.h"
#include "setup-wizard/SteamVrBinding.h"

/*
 * The setup wizard's panel: its pages, the player's choices and every click,
 * drawn into the Dear ImGui context RPS UI hands its callbacks. It knows
 * nothing of the game: a host opens a session with the player's settings and
 * SteamVR's state, and carries out the finish the player asks for (the
 * settings to write and the SteamVR binding to install). In the game the host
 * is SetupWizard.cpp over RPS UI Framework; on the desktop it is
 * preview/SetupWizardPreview.cpp over RPS UI's LocalPanelHost, so the preview
 * runs exactly the pages the game shows.
 *
 * Pages: the player's controllers, the topic pages of
 * setup_wizard_policy::kPages, a review, and the last page once finished.
 *
 * Threads: the host calls begin, setOpen, takeRequests and finished from its own
 * thread; the two callbacks come from the panel's render thread. One mutex guards
 * everything the callbacks touch, and no call leaves the panel while it is held.
 */
namespace rock::setup_wizard
{
    namespace fb = function_binding_policy;
    namespace wp = setup_wizard_policy;

    // The panel's layout size; RPS UI scales it to the panel's physical width.
    inline constexpr float kLayoutWidth = 1280.0f;
    inline constexpr float kLayoutHeight = 820.0f;

    // One wizard option as the settings store holds it.
    struct OptionState
    {
        const wp::Option* option{ nullptr };
        std::string value;
        std::string original;
        std::string defaultValue;
    };

    // What a session opens with: the player's settings and what SteamVR uses now.
    struct SessionStart
    {
        // The actions' buttons, to name what SteamVR's default binding breaks.
        fb::Bindings bindings{ fb::homeBindings() };
        // The wp::kOptions the store knows, in page order.
        std::vector<OptionState> options;
        // [ControllerInput] OfferSteamVRBinding.
        bool offeringBinding{ true };
        steamvr_binding::State steamVr;
    };

    // A settings visitor (RockConfig::visitSettings over the Consumer group) that fills
    // `start`, a SessionStart*: the wizard's options in page order and the binding offer.
    void collectSettings(const api::configuration::Setting& setting, void* start) noexcept;

    // One settings value the finish writes.
    struct SettingWrite
    {
        std::string section;
        std::string key;
        std::string value;
    };

    // What the player asked for since the host last looked.
    struct Requests
    {
        // The player finished the wizard: write `writes`, then carry out `binding`.
        std::optional<wp::Finish> finish;
        std::vector<SettingWrite> writes;
        wp::ControllerPlan binding;
        // The SteamVR state to install over, for BindingAction::Install.
        steamvr_binding::Inspection inspection;
        // Close the panel now (from the last page).
        bool close{ false };
    };

    // What the host did with a finish.
    struct FinishResult
    {
        std::vector<SettingWrite> written;
        // "Section.Key: reason" for each value that was not written.
        std::vector<std::string> errors;
        bool bindingInstalled{ false };
        std::string bindingError;
    };

    class WizardPanel
    {
    public:
        WizardPanel();
        ~WizardPanel();
        WizardPanel(const WizardPanel&) = delete;
        WizardPanel& operator=(const WizardPanel&) = delete;

        // The session, the fonts and the pending requests, guarded by the panel's
        // mutex; SetupWizardPanel.cpp defines it.
        struct Guarded;

        // A new session at the controllers page.
        void begin(SessionStart start);

        // Whether the panel is shown. A hidden panel draws nothing.
        void setOpen(bool open) noexcept { _open.store(open, std::memory_order_release); }
        [[nodiscard]] bool isOpen() const noexcept { return _open.load(std::memory_order_acquire); }

        // Drops requests made before the wizard opened or after it closed.
        void clearRequests();

        [[nodiscard]] Requests takeRequests();

        // The host carried out a finish. True when the panel shows its last page (something
        // was saved or installed, or failed); false when the host should close it.
        [[nodiscard]] bool finished(wp::Finish finish, const FinishResult& result);

        // The panel's RPS UI callbacks; userData is the WizardPanel.
        static void RPSUI_CALL setupCallback(const ::rpsui::api::framework::PanelSetup* setup, void* userData) noexcept;
        static void RPSUI_CALL drawCallback(const ::rpsui::api::framework::PanelDrawFrame* frame, void* userData) noexcept;

        // The page shown, for a host's status line.
        [[nodiscard]] std::string pageName();

    private:
        void setup(const ::rpsui::api::framework::PanelSetup& setup);
        void draw(const ::rpsui::api::framework::PanelDrawFrame& frame);

        std::atomic<bool> _open{ false };
        std::mutex _mutex;
        std::unique_ptr<Guarded> _guarded;
    };
}
