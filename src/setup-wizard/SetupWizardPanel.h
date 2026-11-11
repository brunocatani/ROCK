#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
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
 * SteamVR's state, feeds the controls held, and carries out what the player
 * asked for (save, close, the SteamVR choice). In the game the host is
 * SetupWizard.cpp over RPS UI Framework; on the desktop it is
 * preview/SetupWizardPreview.cpp over RPS UI's LocalPanelHost, so the preview
 * runs exactly the pages the game shows.
 *
 * Threads: the host calls begin, setOpen, takeRequests and the finish calls
 * from its own thread; feedCapture comes from the input thread; the two
 * callbacks from the panel's render thread. One mutex guards everything the
 * callbacks touch, and no call leaves the panel while it is held.
 */
namespace rock::setup_wizard
{
    namespace fb = function_binding_policy;
    namespace wp = setup_wizard_policy;

    // The panel's layout size; RPS UI scales it to the panel's physical width.
    inline constexpr float kLayoutWidth = 1280.0f;
    inline constexpr float kLayoutHeight = 820.0f;

    struct OptionState
    {
        wp::OptionPage page{};
        wp::ValueKind kind{ wp::ValueKind::Boolean };
        std::string help;
        std::string value;
        std::string original;
        std::string defaultValue;
        bool found{ false };
    };

    // One option page per wp::kOptionPages entry, waiting to be filled by collectOption.
    [[nodiscard]] std::vector<OptionState> optionPages();

    // A settings visitor (RockConfig::visitSettings) that fills the pages from `options`,
    // a std::vector<OptionState>*. Pages the visit never names stay unfound.
    void collectOption(const api::configuration::Setting& setting, void* options) noexcept;

    enum class BindingChoice : std::uint8_t
    {
        None,
        UseRock,
        KeepSteamVr,
    };

    // What a session opens with: the player's settings and what SteamVR uses now.
    struct SessionStart
    {
        fb::Bindings bindings{ fb::homeBindings() };
        // Found pages only, in page order.
        std::vector<OptionState> options;
        steamvr_binding::Inspection steamVr;
    };

    // One settings value the save writes.
    struct SettingWrite
    {
        std::string section;
        std::string key;
        std::string value;
    };

    struct SaveResult
    {
        std::vector<SettingWrite> written;
        // "Section.Key: reason" for each value that was not written.
        std::vector<std::string> errors;
    };

    // What the player asked for since the host last looked.
    struct Requests
    {
        // Pending until finishBindingChoice; the panel's choice cards stay disabled meanwhile.
        BindingChoice bindingChoice{ BindingChoice::None };
        // The SteamVR state to apply ROCK's binding over, for BindingChoice::UseRock.
        steamvr_binding::Inspection steamVr;
        bool save{ false };
        // Every changed value, in save order, when save is set.
        std::vector<SettingWrite> writes;
        bool close{ false };
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

        // A new session at its first page: the SteamVR page when ROCK has a binding to offer.
        void begin(SessionStart start);

        // Whether the panel is shown. A hidden panel draws nothing and ignores input.
        void setOpen(bool open) noexcept { _open.store(open, std::memory_order_release); }
        [[nodiscard]] bool isOpen() const noexcept { return _open.load(std::memory_order_acquire); }

        // Drops requests made before the wizard opened or after it closed.
        void clearRequests();

        // Takes the save and close requests; the binding choice stays until it is finished.
        [[nodiscard]] Requests takeRequests();

        // The host wrote a save: written values become the session's saved state and the
        // Saved page opens.
        void finishSave(const SaveResult& result);

        // The host carried out the SteamVR choice. `after` is SteamVR's state once ROCK's
        // binding is set.
        void finishBindingChoice(BindingChoice choice, bool applied, std::string error, steamvr_binding::Inspection after);

        // Input thread: the controls held per hand (left, right), a bit per
        // button_layout_policy::Source, after the controller layout.
        void feedCapture(std::array<std::uint32_t, 2> sample);

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
