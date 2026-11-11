#include "setup-wizard/SetupWizard.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

#include <RPS_UI_Framework/Framework.h>
#include <RPS_UI_Framework/ImGuiBuild.h>
#include <RPS_UI_Framework/Input.h>

#include <imgui.h>
#include <imgui_internal.h>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/visual/RpsUiQuery.h"
#include "rock_support/Fo4VrRuntime.h"
#include "setup-wizard/AutoOpenGate.h"
#include "setup-wizard/SetupWizardPanel.h"
#include "setup-wizard/SetupWizardSettings.h"
#include "setup-wizard/SteamVrBinding.h"

namespace rock::setup_wizard
{
    namespace
    {
        namespace fw = rpsui::api::framework;
        using rpsui::api::Result;

        // Game units (about 70 per metre): the panel's width, its distance in front
        // of the headset and its drop below eye level when it opens.
        constexpr float kPhysicalWidth = 96.0f;
        constexpr float kOpenDistance = 85.0f;
        constexpr float kOpenDrop = 12.0f;
        // The five buttons RPS UI can capture: B/Y, grip, A/X, stick click, trigger.
        constexpr std::uint64_t kCaptureButtons = (1ull << 1) | (1ull << 2) | (1ull << 7) | (1ull << 32) | (1ull << 33);

        enum class AutoOpen : std::uint8_t
        {
            Off,
            AfterMirror,
            WhenFree,
        };

        // Raise when the wizard gains something every player should see once (a new page, new
        // controls or options): players whose [SetupWizard] ShownEdition is lower see it again.
        constexpr int kEdition = 2;
        // How long the game has to stay free before the wizard opens by itself.
        constexpr auto kFreeTime = std::chrono::seconds(10);
        std::atomic<bool> g_newGameStarting{ false };

        /*
         * The connection fields belong to the frame thread (update, onSessionLoaded).
         * The panel guards its own state; RPS UI may hold its own locks while it runs
         * the panel and input callbacks, so no call into RPS UI is made from inside
         * the panel. The input callback reads `input` and `inputToken` through the atomics.
         */
        struct State
        {
            const fw::Api* framework{ nullptr };
            std::uint64_t owner{ 0 };
            std::uint64_t panel{ 0 };
            std::uint64_t presentationSequence{ 1 };
            rps_ui::BindGate bindGate;
            bool absentReported{ false };
            std::atomic<const rpsui::api::input::Api*> input{ nullptr };
            std::atomic<std::uint64_t> inputToken{ 0 };
            // The automatic opening: armed by each game load until the wizard has opened in
            // this game start; on a new game it first waits for the mirror to close.
            AutoOpen autoOpen{ AutoOpen::Off };
            bool mirrorSeen{ false };
            bool openedThisRun{ false };
            // The pending open is the automatic one, and the open wizard was opened that way.
            bool autoOpenRequested{ false };
            bool autoOpened{ false };
            // A game menu interrupted the automatically opened wizard: it comes back with its pages as they were.
            bool resume{ false };
            std::chrono::steady_clock::time_point quietSince{};
            auto_open_gate::Reading lastReading{};
            WizardPanel wizard;
        };

        State& state() noexcept
        {
            static State value;
            return value;
        }

        std::atomic<bool> g_openRequested{ false };

        // Game thread, every input frame: keeps the controllers captured while open, so the
        // game and ROCK's gameplay ignore the buttons that work the panel.
        void RPSUI_CALL onInput(const rpsui::api::input::Frame* frame, void*) noexcept
        {
            auto& s = state();
            const auto* input = s.input.load(std::memory_order_acquire);
            if (!frame || !input || !s.wizard.isOpen()) return;
            // Renewed every callback while open; it lapses by itself three callbacks after the wizard closes.
            rpsui::api::input::Capture capture{};
            capture.buttons[0] = kCaptureButtons;
            capture.buttons[1] = kCaptureButtons;
            (void)input->capture(s.inputToken.load(std::memory_order_acquire), &capture);
        }

        // ---- binding -----------------------------------------------------------------------

        bool bind(State& s)
        {
            if (s.framework && s.panel) return true;
            const auto query = s.bindGate.due();
            if (!query && !s.bindGate.absent()) return false;
            const fw::Api* framework{};
            const rpsui::api::input::Api* input{};
            if (rpsui::api::acquire(query, framework) != Result::Ok || rpsui::api::acquire(query, input) != Result::Ok) {
                if (query) s.bindGate.refused();
                if (!s.absentReported) {
                    s.absentReported = true;
                    ROCK_LOG_INFO(Config, "Setup wizard unavailable: RPS UI Framework 2.0 is not loaded");
                }
                return false;
            }
            if (!s.owner) {
                fw::ConsumerRegistration consumer{};
                std::snprintf(consumer.consumerId, sizeof(consumer.consumerId), "%s", "rock.setup");
                std::snprintf(consumer.displayName, sizeof(consumer.displayName), "%s", "ROCK Setup");
                fw::ConsumerHandle handle{};
                if (framework->registerConsumer(&consumer, &handle) != Result::Ok || !handle.ownerToken) {
                    s.bindGate.refused();
                    return false;
                }
                s.owner = handle.ownerToken;
            }
            fw::PanelRegistration panel{};
            std::snprintf(panel.panelId, sizeof(panel.panelId), "%s", "rock.setup.wizard");
            std::snprintf(panel.displayName, sizeof(panel.displayName), "%s", "ROCK Setup");
            panel.layoutWidth = kLayoutWidth;
            panel.layoutHeight = kLayoutHeight;
            panel.defaultPhysicalWidth = kPhysicalWidth;
            panel.minimumPhysicalWidth = 72.0f;
            panel.maximumPhysicalWidth = 180.0f;
            panel.sortOrder = 300;
            panel.flags = static_cast<std::uint32_t>(fw::PanelFlag::ConfigNavigation) |
                static_cast<std::uint32_t>(fw::PanelFlag::Movable);
            panel.imgui = RPSUI_IMGUI_BUILD();
            panel.setupCallback = &WizardPanel::setupCallback;
            panel.drawCallback = &WizardPanel::drawCallback;
            panel.userData = &s.wizard;
            const auto registered = framework->registerPanel(s.owner, &panel, &s.panel);
            if (registered != Result::Ok || !s.panel) {
                ROCK_LOG_WARN(Config, "Setup wizard panel refused by RPS UI ({})", static_cast<std::uint32_t>(registered));
                s.panel = 0;
                s.bindGate.refused();
                return false;
            }
            s.input.store(input, std::memory_order_release);
            s.inputToken.store(input->subscribe(&onInput, nullptr), std::memory_order_release);
            s.framework = framework;
            ROCK_LOG_INFO(Config, "Setup wizard registered RPS UI panel {} (input {})", s.panel,
                s.inputToken.load(std::memory_order_acquire));
            return true;
        }

        [[nodiscard]] bool present(State& s, bool open)
        {
            fw::PanelPresentation presentation{};
            presentation.sequence = s.presentationSequence++;
            presentation.open = open ? 1 : 0;
            if (open) {
                const auto* nodes = f4vr::getPlayerNodes();
                if (!nodes || !nodes->HmdNode) return false;
                const auto& world = nodes->HmdNode->world;
                auto forward = world.rotate.Transpose() * RE::NiPoint3(0.0f, 1.0f, 0.0f);
                forward.z = 0.0f;
                const float length = std::sqrt(forward.x * forward.x + forward.y * forward.y);
                if (!std::isfinite(length) || length < 1.0e-3f) return false;
                forward = forward / length;
                auto& pose = presentation.pose;
                pose.center[0] = world.translate.x + forward.x * kOpenDistance;
                pose.center[1] = world.translate.y + forward.y * kOpenDistance;
                pose.center[2] = world.translate.z - kOpenDrop;
                pose.right[0] = forward.y;
                pose.right[1] = -forward.x;
                pose.right[2] = 0.0f;
                pose.up[0] = 0.0f;
                pose.up[1] = 0.0f;
                pose.up[2] = 1.0f;
                pose.front[0] = -forward.x;
                pose.front[1] = -forward.y;
                pose.front[2] = 0.0f;
                pose.physicalWidth = kPhysicalWidth;
                pose.physicalHeight = kPhysicalWidth * kLayoutHeight / kLayoutWidth;
            }
            const auto result = s.framework->submitPanelPresentation(s.owner, s.panel, &presentation);
            if (result != Result::Ok) {
                ROCK_LOG_WARN(Config, "Setup wizard presentation refused by RPS UI ({})", static_cast<std::uint32_t>(result));
                return false;
            }
            return true;
        }

        // Frame thread, outside the panel: the player's finish. The settings store and SteamVR
        // are called here, then the result goes back to the panel. False: close the wizard.
        [[nodiscard]] bool finish(State& s, const Requests& requests)
        {
            FinishResult result{};
            settings::write(requests.writes, result);
            if (requests.binding.action == setup_wizard_policy::BindingAction::Install) {
                result.bindingInstalled = steamvr_binding::apply(requests.inspection, result.bindingError);
                if (!result.bindingInstalled)
                    ROCK_LOG_WARN(Config, "Setup wizard could not set ROCK's SteamVR binding: {}", result.bindingError);
            }
            return s.wizard.finished(*requests.finish, result);
        }

        // Frame thread, while the wizard is closed. Once the game has been free for kFreeTime
        // (on a new game: after the mirror), opens the wizard when this ROCK brings a newer
        // edition of it, ROCK has a SteamVR binding to offer, or it is coming back from a game
        // menu that interrupted it. Logs each change of the gate's reading while armed.
        void autoOpenWhenFree(State& s)
        {
            if (s.autoOpen == AutoOpen::Off) return;
            const auto reading = auto_open_gate::read();
            if (!(reading == s.lastReading)) {
                s.lastReading = reading;
                ROCK_LOG_INFO(Config, "Setup wizard waits: {}", auto_open_gate::describe(reading));
            }
            if (s.autoOpen == AutoOpen::AfterMirror) {
                s.quietSince = {};
                if (reading.mirrorOpen) {
                    s.mirrorSeen = true;
                } else if (s.mirrorSeen) {
                    s.autoOpen = AutoOpen::WhenFree;
                    ROCK_LOG_INFO(Config, "Setup wizard: character creation finished");
                }
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (!reading.ready()) {
                s.quietSince = {};
                return;
            }
            if (s.quietSince == std::chrono::steady_clock::time_point{}) {
                s.quietSince = now;
                return;
            }
            if (now - s.quietSince < kFreeTime) return;
            s.autoOpen = AutoOpen::Off;
            const char* reason = nullptr;
            if (s.resume) reason = "coming back after a game menu";
            else if (g_rockConfig.rockSetupWizardShownEdition < kEdition) reason = "a new edition to show";
            else if (g_rockConfig.rockOfferSteamVRBinding &&
                     steamvr_binding::inspect().status == steamvr_binding_policy::Status::Offer)
                reason = "a SteamVR binding to offer";
            if (!reason) return;
            if (!rps_ui::query()) {
                ROCK_LOG_INFO(Config, "Setup wizard does not open by itself: RPS UI Framework is not loaded");
                return;
            }
            ROCK_LOG_INFO(Config, "Setup wizard opens by itself: {}", reason);
            s.autoOpenRequested = true;
            g_openRequested.store(true, std::memory_order_release);
        }

        // Frame thread: the wizard opened by itself. Records the edition shown so it opens
        // that way once per edition.
        void recordShownEdition()
        {
            if (g_rockConfig.rockSetupWizardShownEdition >= kEdition) return;
            std::string error;
            if (!g_rockConfig.persistSetting(api::configuration::Group::Consumer, "SetupWizard", "ShownEdition",
                    std::to_string(kEdition).c_str(), error))
                ROCK_LOG_WARN(Config, "Setup wizard could not save SetupWizard.ShownEdition: {}", error);
        }

        // Frame thread.
        void close(State& s, const char* reason)
        {
            if (!s.wizard.isOpen()) return;
            s.wizard.setOpen(false);
            s.autoOpened = false;
            s.wizard.clearRequests();
            if (s.framework && s.panel) (void)present(s, false);
            ROCK_LOG_INFO(Config, "Setup wizard closed ({})", reason);
        }
    }

    bool requestOpen() noexcept
    {
        if (!rps_ui::query()) return false;
        g_openRequested.store(true, std::memory_order_release);
        return true;
    }

    void update() noexcept
    {
        auto& s = state();
        try {
            if (g_openRequested.load(std::memory_order_acquire)) {
                if (!bind(s)) {
                    if (!rps_ui::query()) g_openRequested.store(false, std::memory_order_release);
                } else {
                    g_openRequested.store(false, std::memory_order_release);
                    const bool automatic = std::exchange(s.autoOpenRequested, false);
                    const bool resume = automatic && std::exchange(s.resume, false);
                    if (s.wizard.isOpen()) {
                        // Already open.
                    } else if (input_remap_runtime::isMenuInputActive()) {
                        ROCK_LOG_INFO(Config, "Setup wizard not opened: a game menu has the controllers");
                        if (automatic) {
                            s.resume = resume;
                            s.autoOpen = AutoOpen::WhenFree;
                        }
                    } else {
                        if (!resume) {
                            auto start = settings::read();
                            start.steamVr = steamvr_binding::read();
                            s.wizard.begin(std::move(start));
                        }
                        s.wizard.clearRequests();
                        if (present(s, true)) {
                            s.wizard.setOpen(true);
                            s.autoOpened = automatic;
                            s.openedThisRun = true;
                            s.autoOpen = AutoOpen::Off;
                            recordShownEdition();
                            ROCK_LOG_INFO(Config, "Setup wizard opened{}", automatic ? " by itself" : "");
                        }
                    }
                }
            }
            if (!s.wizard.isOpen()) {
                autoOpenWhenFree(s);
                return;
            }
            const auto requests = s.wizard.takeRequests();
            if (requests.close || (requests.finish && !finish(s, requests))) {
                close(s, "closed by the player");
                return;
            }
            fw::PanelState panel{};
            if (s.framework->getPanelState && s.framework->getPanelState(s.owner, s.panel, &panel) == Result::Ok && !panel.open) {
                close(s, "panel closed");
            } else if (input_remap_runtime::isMenuInputActive()) {
                const bool wasAutomatic = s.autoOpened;
                close(s, "a game menu opened");
                // A message box or scene menu took over the wizard that opened by itself: it
                // comes back where the player was once the game is free again.
                if (wasAutomatic) {
                    s.resume = true;
                    s.autoOpen = AutoOpen::WhenFree;
                    s.quietSince = {};
                }
            }
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Config, 5000, "Setup wizard update failed");
        }
    }

    void noteSessionStart(bool newGame) noexcept
    {
        g_newGameStarting.store(newGame, std::memory_order_release);
    }

    void onSessionLoaded() noexcept
    {
        auto& s = state();
        close(s, "game loaded");
        s.resume = false;
        s.autoOpenRequested = false;
        if (s.openedThisRun) {
            s.autoOpen = AutoOpen::Off;
            return;
        }
        const bool newGame = g_newGameStarting.exchange(false, std::memory_order_acq_rel);
        const bool newEdition = g_rockConfig.rockSetupWizardShownEdition < kEdition;
        if (!newEdition && !(g_rockConfig.rockOfferSteamVRBinding &&
                               steamvr_binding::inspect().status == steamvr_binding_policy::Status::Offer)) {
            s.autoOpen = AutoOpen::Off;
            return;
        }
        s.autoOpen = newGame ? AutoOpen::AfterMirror : AutoOpen::WhenFree;
        s.mirrorSeen = false;
        s.quietSince = {};
        s.lastReading = {};
        ROCK_LOG_INFO(Config, "Setup wizard armed to open by itself {}",
            newGame ? "after the mirror" : "once the game is free");
    }
}
