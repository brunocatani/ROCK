#include "setup-wizard/SetupWizard.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <RPS_UI_Framework/BindingIcons.h>
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
#include "setup-wizard/SetupWizardPolicy.h"
#include "setup-wizard/SteamVrBinding.h"

namespace rock::setup_wizard
{
    namespace
    {
        namespace fb = function_binding_policy;
        namespace wp = setup_wizard_policy;
        namespace fw = rpsui::api::framework;
        namespace bt = rpsui::binding;
        using rpsui::api::Result;

        constexpr float kLayoutWidth = 1280.0f;
        constexpr float kLayoutHeight = 820.0f;
        // Game units (about 70 per metre): the panel's width, its distance in front
        // of the headset and its drop below eye level when it opens.
        constexpr float kPhysicalWidth = 96.0f;
        constexpr float kOpenDistance = 85.0f;
        constexpr float kOpenDrop = 12.0f;
        // The five buttons RPS UI can capture: B/Y, grip, A/X, stick click, trigger.
        constexpr std::uint64_t kCaptureButtons = (1ull << 1) | (1ull << 2) | (1ull << 7) | (1ull << 32) | (1ull << 33);

        struct FunctionText
        {
            const char* name;
            const char* detail;
        };

        [[nodiscard]] constexpr FunctionText functionText(fb::Function function) noexcept
        {
            switch (function) {
            case fb::Function::Grab: return { "Grab", "Take hold of objects, surfaces and weapons." };
            case fb::Function::Fire: return { "Fire", "Shoot the weapon in your firing hand." };
            case fb::Function::SupportTrigger:
                return { "Support trigger", "The other hand's trigger: a second weapon, or what that hand holds." };
            case fb::Function::Use: return { "Use", "Take, open or talk to what you point at." };
            case fb::Function::Reload: return { "Reload", "Reload the weapon." };
            case fb::Function::Scope: return { "Scope", "Look through the scope while active." };
            case fb::Function::Vats: return { "VATS", "Open VATS." };
            case fb::Function::Grenade: return { "Grenade", "Draw a grenade into your hand." };
            case fb::Function::Pipboy: return { "Pip-Boy", "Open the Pip-Boy." };
            case fb::Function::Pause: return { "Pause menu", "Open the Pause menu." };
            case fb::Function::StickActions:
                return { "Bipod and Favorites", "Latch the weapon to a surface, hold to place, and the Favorites menu." };
            case fb::Function::Palm: return { "RobCo PALM", "Open the PALM wheel." };
            case fb::Function::Count: break;
            }
            return { "", "" };
        }

        // Grab, Fire and Use always need a button.
        [[nodiscard]] constexpr bool mayClear(fb::Function function) noexcept
        {
            return function != fb::Function::Grab && function != fb::Function::Fire && function != fb::Function::Use;
        }

        [[nodiscard]] constexpr const char* gestureLabel(fb::Gesture gesture) noexcept
        {
            switch (gesture) {
            case fb::Gesture::Press: return "Press";
            case fb::Gesture::Tap: return "Tap";
            case fb::Gesture::Hold: return "Hold";
            case fb::Gesture::DoubleTap: return "Double tap";
            }
            return "";
        }

        [[nodiscard]] constexpr const char* gestureDetail(fb::Gesture gesture) noexcept
        {
            switch (gesture) {
            case fb::Gesture::Press: return "Acts from the moment you press until you let go.";
            case fb::Gesture::Tap: return "A short press. Waits for a hold or a second tap only when another action uses them.";
            case fb::Gesture::Hold: return "Keep the button down; acts after the hold time until you let go.";
            case fb::Gesture::DoubleTap: return "Two quick taps.";
            }
            return "";
        }

        [[nodiscard]] constexpr const char* handRole(fb::BindingHand hand) noexcept
        {
            switch (hand) {
            case fb::BindingHand::Firing: return "your firing hand";
            case fb::BindingHand::Support: return "your support hand";
            case fb::BindingHand::Primary: return "your main hand";
            case fb::BindingHand::Offhand: return "your off hand";
            case fb::BindingHand::Left: return "the left hand";
            case fb::BindingHand::Right: return "the right hand";
            case fb::BindingHand::Each: return "either hand";
            }
            return "";
        }

        [[nodiscard]] constexpr bt::Control control(fb::Source source) noexcept
        {
            switch (source) {
            case fb::Source::Trigger: return bt::Control::Trigger;
            case fb::Source::Grip: return bt::Control::Grip;
            case fb::Source::LowerFace: return bt::Control::LowerFace;
            case fb::Source::UpperFace: return bt::Control::UpperFace;
            case fb::Source::Stick: return bt::Control::StickClick;
            case fb::Source::GripClick: return bt::Control::GripClick;
            case fb::Source::Axis3Click: return bt::Control::Axis3Click;
            case fb::Source::Axis4Click: return bt::Control::Axis4Click;
            case fb::Source::PadUp: return bt::Control::PadUp;
            case fb::Source::PadDown: return bt::Control::PadDown;
            case fb::Source::PadLeft: return bt::Control::PadLeft;
            case fb::Source::PadRight: return bt::Control::PadRight;
            case fb::Source::PadCenter: return bt::Control::PadCenter;
            default: return bt::Control::None;
            }
        }

        // The hand a meaning names now; "each" is shown on the right.
        [[nodiscard]] bool showsLeft(fb::BindingHand hand, const fb::Context& now) noexcept
        {
            return hand != fb::BindingHand::Each && fb::sideMatches(hand, true, now);
        }

        [[nodiscard]] bt::Binding display(const fb::Binding& binding, const fb::Context& now) noexcept
        {
            bt::Binding result{};
            if (!binding.bound()) return result;
            result.input = { showsLeft(binding.control.hand, now) ? bt::Hand::Left : bt::Hand::Right,
                control(binding.control.source) };
            result.gesture = static_cast<bt::Gesture>(binding.gesture);
            if (binding.chord())
                result.modifier = bt::Input{ showsLeft(binding.modifier.hand, now) ? bt::Hand::Left : bt::Hand::Right,
                    control(binding.modifier.source) };
            return result;
        }

        [[nodiscard]] std::string describe(const fb::Binding& binding, const fb::Context& now)
        {
            if (!binding.bound()) return "No button";
            auto text = bt::sentence(display(binding, now));
            text += " (";
            text += handRole(binding.control.hand);
            text += ')';
            return text;
        }

        // The window-local x of the content region's right edge, for right-aligned footer buttons.
        [[nodiscard]] float contentRight() noexcept
        {
            const auto* window = ImGui::GetCurrentWindow();
            return window->ContentRegionRect.Max.x - window->Pos.x;
        }

        enum class Page : std::uint8_t
        {
            // Offers ROCK's SteamVR binding; first, and only when SteamVR's default binding is in use.
            Controller,
            Welcome,
            Controls,
            Capture,
            Option,
            Review,
            Saved,
        };

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

        struct Session
        {
            Page page{ Page::Welcome };
            std::size_t option{ 0 };
            fb::Bindings bindings{ fb::homeBindings() };
            fb::Bindings original{ fb::homeBindings() };
            std::vector<OptionState> options;
            fb::Function editing{ fb::Function::Count };
            wp::Capture capture{};
            bool listening{ false };
            wp::HandChoices hands{};
            std::size_t hand{ 0 };
            fb::Gesture gesture{ fb::Gesture::Tap };
            std::vector<std::string> saveErrors;
            // What SteamVR uses for Fallout 4 VR, read when the wizard opened.
            steamvr_binding::Inspection steamVr;
            bool controllerPage{ false };
            bool bindingApplied{ false };
            std::string bindingError;
        };

        enum class BindingChoice : std::uint8_t
        {
            None,
            UseRock,
            KeepSteamVr,
        };

        enum class AutoOpen : std::uint8_t
        {
            Off,
            AfterMirror,
            WhenFree,
        };

        // Raise when the wizard gains something every player should see once (a new page, new
        // controls or options): players whose [SetupWizard] ShownEdition is lower see it again.
        constexpr int kEdition = 1;
        // How long the game has to stay free before the wizard opens by itself.
        constexpr auto kFreeTime = std::chrono::seconds(10);
        std::atomic<bool> g_newGameStarting{ false };

        /*
         * The connection fields belong to the frame thread (update, onSessionLoaded)
         * and are never touched under `mutex`: RPS UI may hold its own locks while it
         * runs the panel and input callbacks, which take `mutex`, so no call into RPS
         * UI is made while `mutex` is held. The input callback reads `input` and
         * `inputToken` through the atomics.
         */
        struct State
        {
            // Frame thread.
            const fw::Api* framework{ nullptr };
            std::uint64_t owner{ 0 };
            std::uint64_t panel{ 0 };
            std::uint64_t presentationSequence{ 1 };
            rps_ui::BindGate bindGate;
            bool absentReported{ false };
            std::atomic<const rpsui::api::input::Api*> input{ nullptr };
            std::atomic<std::uint64_t> inputToken{ 0 };
            std::atomic<bool> open{ false };
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

            // Guarded by mutex.
            std::mutex mutex;
            BindingChoice bindingChoice{ BindingChoice::None };
            bool saveRequested{ false };
            bool closeRequested{ false };
            // The pointer's hand while it is on the panel: its trigger is a click, not a capture.
            fw::PhysicalHand pointerHand{ fw::PhysicalHand::None };
            std::uint64_t contextGeneration{ 0 };
            std::array<ImFont*, fw::kFontRoleCount> fonts{};
            bool backPending{ false };
            Session session;

        };

        State& state() noexcept
        {
            static State value;
            return value;
        }

        std::atomic<bool> g_openRequested{ false };

        [[nodiscard]] fb::Context displayContext(const Session& session) noexcept
        {
            fb::Context context{};
            for (const auto& option : session.options)
                if (option.page.section == "FiringHand" && option.page.key == "DefaultHand")
                    context.firingIsLeft = option.value == "left";
            context.primaryIsLeft = false;
            return context;
        }

        // ---- settings ----------------------------------------------------------------------

        void collectOption(const api::configuration::Setting& setting, void* opaque) noexcept
        {
            auto& options = *static_cast<std::vector<OptionState>*>(opaque);
            if (!setting.section || !setting.key) return;
            for (auto& option : options) {
                if (option.page.section != setting.section || option.page.key != setting.key) continue;
                option.found = true;
                option.help = setting.description ? setting.description : "";
                option.value = setting.value ? setting.value : "";
                option.original = option.value;
                option.defaultValue = setting.defaultValue ? setting.defaultValue : "";
                option.kind = setting.type == api::configuration::ValueType::Boolean ? wp::ValueKind::Boolean
                    : setting.type == api::configuration::ValueType::Integer       ? wp::ValueKind::Integer
                                                                                   : wp::ValueKind::Text;
            }
        }

        void loadSession(Session& session)
        {
            session = {};
            session.bindings = g_rockConfig.rockFunctionBindings;
            session.original = session.bindings;
            for (const auto& page : wp::kOptionPages) session.options.push_back({ .page = page });
            (void)g_rockConfig.visitSettings(api::configuration::Group::Consumer, collectOption, &session.options);
            std::erase_if(session.options, [](const OptionState& option) { return !option.found; });
        }

        // Frame thread. Writes every changed value; the settings store applies them.
        void saveSession(Session& session)
        {
            session.saveErrors.clear();
            const auto write = [&](const char* section, const char* key, const std::string& value) {
                std::string error;
                if (!g_rockConfig.persistSetting(api::configuration::Group::Consumer, section, key, value.c_str(), error)) {
                    session.saveErrors.push_back(std::string(section) + "." + key + ": " + error);
                    ROCK_LOG_WARN(Config, "Setup wizard could not save {}.{}: {}", section, key, error);
                }
            };
            for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                if (session.bindings[i] == session.original[i]) continue;
                const std::string key{ fb::functionKey(static_cast<fb::Function>(i)) };
                write("Bindings", key.c_str(), fb::formatBinding(session.bindings[i]));
            }
            for (const auto& option : session.options) {
                if (option.value == option.original) continue;
                const std::string section{ option.page.section }, key{ option.page.key };
                write(section.c_str(), key.c_str(), option.value);
            }
            ROCK_LOG_INFO(Config, "Setup wizard saved ({} errors)", session.saveErrors.size());
        }

        // ---- drawing -----------------------------------------------------------------------

        struct Draw
        {
            const fw::Theme& theme;
            State& state;
            Session& session;
            fb::Context now;

            [[nodiscard]] ImU32 ink(const fw::ThemeColor& color, float alpha = 1.0f) const
            {
                return ImGui::GetColorU32(rpsui::imgui_host::color(color, alpha));
            }

            void font(fw::FontRole role, float size) const
            {
                ImGui::PushFont(state.fonts[static_cast<std::size_t>(role)], size);
            }

            void heading(const char* text) const
            {
                font(fw::FontRole::Strong, theme.titleFontSize);
                ImGui::TextUnformatted(text);
                ImGui::PopFont();
            }

            void prose(const std::string_view text, bool muted = true) const
            {
                font(fw::FontRole::Prose, theme.fontSize);
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(muted ? theme.textMuted : theme.text));
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
                ImGui::TextUnformatted(text.data(), text.data() + text.size());
                ImGui::PopTextWrapPos();
                ImGui::PopStyleColor();
                ImGui::PopFont();
            }

            // A footer or page button; the accent one is the page's main action.
            [[nodiscard]] bool button(const char* label, bool accent = false, bool enabled = true, float width = 0.0f) const
            {
                const float height = theme.targetHeight * 1.15f;
                if (accent) {
                    ImGui::PushStyleColor(ImGuiCol_Button, rpsui::imgui_host::color(theme.accent));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rpsui::imgui_host::color(theme.accent, 0.85f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, rpsui::imgui_host::color(theme.pressed));
                    ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(theme.onAccent));
                }
                ImGui::BeginDisabled(!enabled);
                const bool pressed = ImGui::Button(label, { width > 0.0f ? width : 0.0f, height });
                ImGui::EndDisabled();
                if (accent) ImGui::PopStyleColor(4);
                return pressed && enabled;
            }

            // A large choice: label, detail, the selection shown by the accent edge.
            [[nodiscard]] bool card(const char* id, const std::string_view label, const std::string_view detail,
                bool selected, bool enabled = true, std::string_view badge = {}) const
            {
                const float width = ImGui::GetContentRegionAvail().x;
                font(fw::FontRole::Prose, theme.smallFontSize);
                const float detailHeight = detail.empty() ? 0.0f
                    : ImGui::CalcTextSize(detail.data(), detail.data() + detail.size(), false, width - theme.padding * 3.0f).y;
                ImGui::PopFont();
                const float height = std::max(theme.targetHeight * 1.4f, theme.padding * 2.0f + theme.fontSize * 1.3f +
                    (detail.empty() ? 0.0f : detailHeight + theme.spacing * 0.5f));
                const ImVec2 lo = ImGui::GetCursorScreenPos();
                ImGui::PushID(id);
                ImGui::BeginDisabled(!enabled);
                const bool clicked = ImGui::InvisibleButton("card", { width, height });
                ImGui::EndDisabled();
                const bool hovered = ImGui::IsItemHovered() && enabled;
                ImGui::PopID();
                const ImVec2 hi{ lo.x + width, lo.y + height };
                auto* draw = ImGui::GetWindowDrawList();
                draw->AddRectFilled(lo, hi, ink(selected ? theme.selected : hovered ? theme.hover : theme.surface),
                    theme.rounding);
                draw->AddRect(lo, hi, ink(selected ? theme.accent : theme.border), theme.rounding,
                    selected ? theme.borderWidth * 3.0f : theme.borderWidth);
                const auto& textColor = enabled ? theme.text : theme.textDisabled;
                auto* strong = state.fonts[static_cast<std::size_t>(fw::FontRole::Strong)];
                draw->AddText(strong, theme.fontSize, { lo.x + theme.padding * 1.5f, lo.y + theme.padding }, ink(textColor),
                    label.data(), label.data() + label.size());
                if (!badge.empty()) {
                    const float badgeWidth = strong->CalcTextSizeA(theme.smallFontSize, 10000.0f, 0.0f, badge.data(),
                        badge.data() + badge.size()).x;
                    draw->AddText(strong, theme.smallFontSize, { hi.x - theme.padding * 1.5f - badgeWidth, lo.y + theme.padding },
                        ink(theme.textMuted), badge.data(), badge.data() + badge.size());
                }
                if (!detail.empty()) {
                    auto* prose = state.fonts[static_cast<std::size_t>(fw::FontRole::Prose)];
                    draw->AddText(prose, theme.smallFontSize,
                        { lo.x + theme.padding * 1.5f, lo.y + theme.padding + theme.fontSize * 1.3f + theme.spacing * 0.5f },
                        ink(enabled ? theme.textMuted : theme.textDisabled), detail.data(), detail.data() + detail.size(),
                        width - theme.padding * 3.0f);
                }
                ImGui::Dummy({ 0.0f, theme.spacing * 0.5f });
                return clicked && enabled;
            }

            void beginBody(float footerHeight) const
            {
                ImGui::BeginChild("body", { 0.0f, -footerHeight }, ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
            }

            void endBody() const
            {
                ImGui::EndChild();
                auto* draw = ImGui::GetWindowDrawList();
                const ImVec2 at = ImGui::GetCursorScreenPos();
                draw->AddLine({ at.x, at.y }, { at.x + ImGui::GetContentRegionAvail().x, at.y }, ink(theme.divider),
                    theme.borderWidth);
                ImGui::Dummy({ 0.0f, theme.spacing });
            }

            [[nodiscard]] float footerHeight() const { return theme.targetHeight * 1.15f + theme.spacing * 3.0f; }
        };

        void startEditing(Session& session, fb::Function function)
        {
            session.editing = function;
            session.capture.start();
            session.listening = true;
            session.page = Page::Capture;
        }

        // The binding the capture page shows: the captured control with the chosen hand and gesture.
        [[nodiscard]] std::optional<fb::Binding> candidate(const Session& session, const fb::Context& now)
        {
            const auto& captured = session.capture.result();
            if (session.listening || !captured || session.hand >= session.hands.count) return std::nullopt;
            fb::Binding binding{};
            binding.control = { session.hands.hands[session.hand], captured->control.source };
            binding.gesture = session.gesture;
            if (captured->modifier) {
                const auto modifierLeft = captured->modifier->isLeft;
                // The modifier keeps the control's meaning when it is on the same hand.
                const auto hand = modifierLeft == captured->control.isLeft ? binding.control.hand
                    : modifierLeft                                         ? fb::BindingHand::Left
                                                                           : fb::BindingHand::Right;
                binding.modifier = { hand, captured->modifier->source };
            }
            (void)now;
            return binding;
        }

        void drawHeader(const Draw& d)
        {
            d.font(fw::FontRole::Strong, d.theme.fontSize);
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted("ROCK SETUP");
            ImGui::PopStyleColor();
            ImGui::PopFont();
            const char* step = "";
            char buffer[64]{};
            switch (d.session.page) {
            case Page::Controller: step = "Controller"; break;
            case Page::Welcome: step = "Start"; break;
            case Page::Controls:
            case Page::Capture: step = "Controls"; break;
            case Page::Option:
                std::snprintf(buffer, sizeof(buffer), "Option %zu of %zu", d.session.option + 1, d.session.options.size());
                step = buffer;
                break;
            case Page::Review: step = "Review"; break;
            case Page::Saved: step = "Saved"; break;
            }
            const float closeWidth = d.theme.targetHeight * 2.4f;
            ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted(step);
            ImGui::PopStyleColor();
            ImGui::SameLine(contentRight() - closeWidth);
            if (ImGui::Button("Close##header", { closeWidth, 0.0f })) d.state.closeRequested = true;
            ImGui::Dummy({ 0.0f, d.theme.spacing });
        }

        void drawController(const Draw& d)
        {
            auto& session = d.session;
            const auto& controller = session.steamVr.controller;
            const std::string name{ controller ? controller->displayName : "controllers" };
            const bool index = controller && controller->controller == steamvr_binding_policy::Controller::Index;
            const bool pending = d.state.bindingChoice != BindingChoice::None;
            d.beginBody(d.footerHeight());
            if (session.bindingApplied) {
                d.heading("ROCK's binding is set");
                ImGui::Dummy({ 0.0f, d.theme.spacing });
                d.prose("SteamVR uses ROCK's binding for Fallout 4 VR on your " + name +
                        " from the next start of the game. Quit and start Fallout 4 VR again before you play.",
                    false);
                ImGui::Dummy({ 0.0f, d.theme.spacing });
                d.prose("To go back, choose another binding for Fallout 4 VR in SteamVR's Controller Settings.");
            } else {
                d.heading("Your controllers");
                ImGui::Dummy({ 0.0f, d.theme.spacing });
                d.prose("Your " + name + " use SteamVR's default binding for Fallout 4 VR. " +
                        (index ? "It turns A and X into grip and makes the game treat the controllers as wands, so "
                                 "reload and scope do not work."
                               : "It makes the game treat the wands as controllers without A and X, so reload and "
                                 "scope cannot be reached.") +
                        " ROCK has a binding that gives the game the button layout ROCK is made for.",
                    false);
                ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
                if (d.card("use-rock", "Use ROCK's binding",
                        "ROCK saves its binding beside its settings and sets it for Fallout 4 VR in SteamVR. Other games "
                        "keep their bindings.",
                        false, !pending)) {
                    d.state.bindingChoice = BindingChoice::UseRock;
                    session.bindingError.clear();
                }
                if (d.card("keep-steamvr", "Keep SteamVR's binding",
                        "Nothing changes in SteamVR. ROCK stops opening this page by itself; it is still here when you "
                        "open the setup wizard.",
                        false, !pending)) {
                    d.state.bindingChoice = BindingChoice::KeepSteamVr;
                }
                if (!session.bindingError.empty()) {
                    ImGui::Dummy({ 0.0f, d.theme.spacing });
                    d.prose("ROCK could not set its binding: " + session.bindingError + ".", false);
                }
            }
            d.endBody();
            if (d.button(session.bindingApplied ? "Continue" : "Not now", session.bindingApplied, !pending))
                session.page = Page::Welcome;
        }

        void drawWelcome(const Draw& d)
        {
            d.beginBody(d.footerHeight());
            d.heading("Set up ROCK");
            ImGui::Dummy({ 0.0f, d.theme.spacing });
            d.prose("Choose the button and gesture for each action, then go through ROCK's main options one page at a "
                    "time. Nothing changes until you save on the last page.",
                false);
            ImGui::Dummy({ 0.0f, d.theme.spacing * 3.0f });
            if (d.card("customize", "Choose my controls and options",
                    "Start with the controls: pick an action, press the button you want and choose tap, hold or double tap.",
                    false)) {
                d.session.page = Page::Controls;
            }
            if (d.card("defaults", "Use ROCK's defaults",
                    "Every action on its usual button and every main option at ROCK's default. You see the changes "
                    "before anything is saved.",
                    false)) {
                d.session.bindings = fb::homeBindings();
                for (auto& option : d.session.options) option.value = option.defaultValue;
                d.session.page = Page::Review;
            }
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
            d.prose("Open this again any time from the Setup wizard button on the ROCK page of PALM's Config.");
            d.endBody();
            if (d.button("Close")) d.state.closeRequested = true;
        }

        void drawControls(const Draw& d)
        {
            d.beginBody(d.footerHeight());
            d.heading("Controls");
            d.prose("Select an action to change its button. A moved action reaches the game and other mods as its usual "
                    "button, so they keep working.");
            ImGui::Dummy({ 0.0f, d.theme.spacing });
            const auto found = wp::conflicts(d.session.bindings);
            const float chipWidth = ImGui::GetContentRegionAvail().x * 0.52f;
            for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                const auto function = static_cast<fb::Function>(i);
                const auto text = functionText(function);
                bool conflicted = false;
                for (std::size_t c = 0; c < found.count; ++c)
                    conflicted |= found.items[c].a == function || found.items[c].b == function;
                const bool changed = d.session.bindings[i] != d.session.original[i];
                ImGui::PushID(static_cast<int>(i));
                const ImVec2 lo = ImGui::GetCursorScreenPos();
                const float width = ImGui::GetContentRegionAvail().x;
                const float height = d.theme.targetHeight * 1.5f;
                const bool clicked = ImGui::InvisibleButton("row", { width, height });
                const bool hovered = ImGui::IsItemHovered();
                auto* draw = ImGui::GetWindowDrawList();
                draw->AddRectFilled(lo, { lo.x + width, lo.y + height }, d.ink(hovered ? d.theme.hover : d.theme.surface),
                    d.theme.rounding);
                if (conflicted || changed)
                    draw->AddRectFilled(lo, { lo.x + d.theme.borderWidth * 4.0f, lo.y + height },
                        d.ink(conflicted ? d.theme.invalid : d.theme.pending));
                auto* strong = d.state.fonts[static_cast<std::size_t>(fw::FontRole::Strong)];
                draw->AddText(strong, d.theme.fontSize, { lo.x + d.theme.padding * 1.5f, lo.y + d.theme.padding * 0.6f },
                    d.ink(d.theme.text), text.name);
                std::string note = conflicted ? std::string("Shares its button and gesture with another action")
                                              : std::string(text.detail);
                if (!conflicted && d.session.bindings[i].bound()) {
                    note += "  On ";
                    note += handRole(d.session.bindings[i].control.hand);
                    note += '.';
                }
                draw->AddText(d.state.fonts[static_cast<std::size_t>(fw::FontRole::Prose)], d.theme.smallFontSize,
                    { lo.x + d.theme.padding * 1.5f, lo.y + d.theme.padding * 0.6f + d.theme.fontSize * 1.15f },
                    d.ink(conflicted ? d.theme.invalid : d.theme.textMuted), note.c_str());
                ImGui::SetCursorScreenPos({ lo.x + width - chipWidth - d.theme.padding, lo.y + (height - d.theme.targetHeight) * 0.5f });
                bt::chip(display(d.session.bindings[i], d.now), d.theme, d.theme.targetHeight, chipWidth);
                ImGui::SetCursorScreenPos({ lo.x, lo.y + height + d.theme.spacing * 0.5f });
                ImGui::Dummy({ 0.0f, 0.0f });
                ImGui::PopID();
                if (clicked) startEditing(d.session, function);
            }
            d.endBody();
            if (d.button("Back")) d.session.page = Page::Welcome;
            ImGui::SameLine();
            if (d.button("Default controls")) d.session.bindings = fb::homeBindings();
            ImGui::SameLine(contentRight() - d.theme.targetHeight * 3.0f);
            if (d.button("Next", true, true, d.theme.targetHeight * 3.0f)) {
                d.session.page = d.session.options.empty() ? Page::Review : Page::Option;
                d.session.option = 0;
            }
        }

        void drawCapture(const Draw& d)
        {
            auto& session = d.session;
            const auto function = session.editing;
            const auto index = static_cast<std::size_t>(function);
            const auto text = functionText(function);
            d.beginBody(d.footerHeight());
            d.heading(text.name);
            d.prose(text.detail);
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted("Now");
            ImGui::PopStyleColor();
            bt::chip(display(session.bindings[index], d.now), d.theme, d.theme.targetHeight * 1.2f);
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });

            if (session.listening) {
                const auto phase = session.capture.phase();
                d.font(fw::FontRole::Strong, d.theme.titleFontSize);
                ImGui::TextUnformatted(phase == wp::Capture::Phase::WaitingForRelease ? "Let go of every button"
                                                                                       : "Press the button you want");
                ImGui::PopFont();
                d.prose("Point away from this panel, then press and release the button. For two buttons together, hold "
                        "the first and press the second. Aimed at the panel, the trigger clicks instead.");
                ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
                if (const auto& pending = session.capture.pending()) {
                    fb::Binding shown{};
                    shown.control = { pending->control.isLeft ? fb::BindingHand::Left : fb::BindingHand::Right,
                        pending->control.source };
                    shown.gesture = fb::Gesture::Press;
                    if (pending->modifier)
                        shown.modifier = { pending->modifier->isLeft ? fb::BindingHand::Left : fb::BindingHand::Right,
                            pending->modifier->source };
                    bt::chip(display(shown, d.now), d.theme, d.theme.targetHeight * 1.5f);
                }
                d.endBody();
                if (d.button("Cancel")) session.page = Page::Controls;
                if (mayClear(function)) {
                    ImGui::SameLine();
                    if (d.button("No button")) {
                        session.bindings[index] = {};
                        session.page = Page::Controls;
                    }
                }
                return;
            }

            // Hand meaning.
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted("Which hand");
            ImGui::PopStyleColor();
            for (std::size_t i = 0; i < session.hands.count; ++i) {
                const auto hand = session.hands.hands[i];
                const bool literal = hand == fb::BindingHand::Left || hand == fb::BindingHand::Right;
                std::string label = literal ? std::string("Only ") + handRole(hand) : bt::capitalized(handRole(hand));
                const char* detail = literal ? "Stays on this hand whichever hand holds the weapon."
                                             : "Follows the hand when you switch firing hands.";
                if (d.card(("hand" + std::to_string(i)).c_str(), label, detail, session.hand == i)) session.hand = i;
            }
            ImGui::Dummy({ 0.0f, d.theme.spacing });
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted("How");
            ImGui::PopStyleColor();
            const float columnWidth = (ImGui::GetContentRegionAvail().x - d.theme.spacing * 3.0f) / 4.0f;
            for (const auto gesture : { fb::Gesture::Press, fb::Gesture::Tap, fb::Gesture::Hold, fb::Gesture::DoubleTap }) {
                if (gesture != fb::Gesture::Press) ImGui::SameLine();
                const bool allowed = wp::gestureAllowed(function, gesture);
                const bool selected = session.gesture == gesture;
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, rpsui::imgui_host::color(d.theme.accent));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rpsui::imgui_host::color(d.theme.accent, 0.85f));
                    ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.onAccent));
                }
                ImGui::BeginDisabled(!allowed);
                if (ImGui::Button(gestureLabel(gesture), { columnWidth, d.theme.targetHeight * 1.15f })) session.gesture = gesture;
                ImGui::EndDisabled();
                if (selected) ImGui::PopStyleColor(3);
            }
            d.prose(gestureDetail(session.gesture));
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });

            const auto chosen = candidate(session, d.now);
            if (chosen) {
                bt::chip(display(*chosen, d.now), d.theme, d.theme.targetHeight * 1.5f);
                const auto clashes = wp::clashesWith(session.bindings, function, *chosen);
                if (clashes) {
                    std::string names;
                    for (std::size_t i = 0; i < fb::kFunctionCount; ++i)
                        if (clashes & fb::functionBit(static_cast<fb::Function>(i))) {
                            if (!names.empty()) names += ", ";
                            names += functionText(static_cast<fb::Function>(i)).name;
                        }
                    ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.invalid));
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::Text("Also on this button and gesture: %s. Choose another gesture or button, or change those "
                                "actions too.",
                        names.c_str());
                    ImGui::PopTextWrapPos();
                    ImGui::PopStyleColor();
                }
            }
            d.endBody();
            if (d.button("Try again")) {
                session.capture.start();
                session.listening = true;
            }
            ImGui::SameLine();
            if (d.button("Cancel")) session.page = Page::Controls;
            ImGui::SameLine(contentRight() - d.theme.targetHeight * 3.0f);
            if (d.button("Keep", true, chosen.has_value(), d.theme.targetHeight * 3.0f) && chosen) {
                session.bindings[index] = *chosen;
                session.page = Page::Controls;
            }
        }

        [[nodiscard]] std::string choiceLabel(const OptionState& option, std::string_view value)
        {
            const auto choices = wp::choicesFor(option.page, option.kind, option.help);
            for (std::size_t i = 0; i < choices.count; ++i)
                if (choices.items[i].value == value) return std::string(choices.items[i].label);
            return std::string(value);
        }

        void drawOption(const Draw& d)
        {
            auto& session = d.session;
            auto& option = session.options[session.option];
            const auto help = wp::splitHelp(option.help);
            const auto choices = wp::choicesFor(option.page, option.kind, option.help);
            d.beginBody(d.footerHeight());
            d.heading(std::string(help.title).c_str());
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
            for (std::size_t i = 0; i < choices.count; ++i) {
                const auto& choice = choices.items[i];
                const bool isDefault = choice.value == option.defaultValue;
                if (d.card(std::string(choice.value).c_str(), choice.label, choice.detail, option.value == choice.value, true,
                        isDefault ? "Default" : ""))
                    option.value = std::string(choice.value);
            }
            ImGui::Dummy({ 0.0f, d.theme.spacing * 2.0f });
            // Booleans show their two halves in the cards; modes keep the whole explanation below.
            if (option.kind != wp::ValueKind::Boolean) {
                auto body = help.body;
                if (option.kind == wp::ValueKind::Integer) {
                    // The first line(s) hold the numbered labels the cards already show.
                    while (!body.empty() && body.front() >= '1' && body.front() <= '9') {
                        const auto newline = body.find('\n');
                        body = newline == std::string_view::npos ? std::string_view{} : body.substr(newline + 1);
                    }
                }
                d.prose(body);
            }
            d.endBody();
            if (d.button("Back")) {
                if (session.option == 0) session.page = Page::Controls;
                else --session.option;
            }
            ImGui::SameLine(contentRight() - d.theme.targetHeight * 3.0f);
            if (d.button("Next", true, true, d.theme.targetHeight * 3.0f)) {
                if (session.option + 1 < session.options.size()) ++session.option;
                else session.page = Page::Review;
            }
        }

        void drawReview(const Draw& d)
        {
            auto& session = d.session;
            d.beginBody(d.footerHeight());
            d.heading("Review");
            bool any = false;
            const auto found = wp::conflicts(session.bindings);
            for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                if (session.bindings[i] == session.original[i]) continue;
                if (!any) {
                    ImGui::Dummy({ 0.0f, d.theme.spacing });
                    d.font(fw::FontRole::Strong, d.theme.fontSize);
                    ImGui::TextUnformatted("Controls");
                    ImGui::PopFont();
                }
                any = true;
                const auto name = functionText(static_cast<fb::Function>(i)).name;
                d.prose(std::string(name) + ": " + describe(session.original[i], d.now) + "  >  " +
                        describe(session.bindings[i], d.now),
                    false);
            }
            bool anyOption = false;
            for (const auto& option : session.options) {
                if (option.value == option.original) continue;
                if (!anyOption) {
                    ImGui::Dummy({ 0.0f, d.theme.spacing });
                    d.font(fw::FontRole::Strong, d.theme.fontSize);
                    ImGui::TextUnformatted("Options");
                    ImGui::PopFont();
                }
                anyOption = true;
                d.prose(std::string(wp::splitHelp(option.help).title) + ": " + choiceLabel(option, option.original) +
                        "  >  " + choiceLabel(option, option.value),
                    false);
            }
            if (!any && !anyOption) {
                ImGui::Dummy({ 0.0f, d.theme.spacing });
                d.prose("Nothing changed.");
            }
            if (found.count) {
                ImGui::Dummy({ 0.0f, d.theme.spacing });
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.invalid));
                for (std::size_t c = 0; c < found.count; ++c)
                    ImGui::Text("%s and %s share a button and gesture.", functionText(found.items[c].a).name,
                        functionText(found.items[c].b).name);
                ImGui::PopStyleColor();
            }
            d.endBody();
            if (d.button("Back")) {
                if (session.options.empty()) session.page = Page::Controls;
                else {
                    session.page = Page::Option;
                    session.option = session.options.size() - 1;
                }
            }
            ImGui::SameLine(contentRight() - d.theme.targetHeight * 3.0f);
            if (d.button("Save", true, any || anyOption, d.theme.targetHeight * 3.0f)) d.state.saveRequested = true;
        }

        void drawSaved(const Draw& d)
        {
            d.beginBody(d.footerHeight());
            d.heading(d.session.saveErrors.empty() ? "Saved" : "Some settings were not saved");
            ImGui::Dummy({ 0.0f, d.theme.spacing });
            if (d.session.saveErrors.empty()) d.prose("ROCK uses the new controls and options now.", false);
            for (const auto& error : d.session.saveErrors) d.prose(error, false);
            d.endBody();
            if (d.button("Close", true)) d.state.closeRequested = true;
        }

        void goBack(Session& session)
        {
            switch (session.page) {
            case Page::Welcome:
                if (session.controllerPage) session.page = Page::Controller;
                break;
            case Page::Controls: session.page = Page::Welcome; break;
            case Page::Capture:
                if (!session.listening) session.page = Page::Controls;
                break;
            case Page::Option:
                if (session.option == 0) session.page = Page::Controls;
                else --session.option;
                break;
            case Page::Review:
                if (session.options.empty()) session.page = Page::Controls;
                else {
                    session.page = Page::Option;
                    session.option = session.options.size() - 1;
                }
                break;
            default: break;
            }
        }

        void RPSUI_CALL setupPanel(const fw::PanelSetup* setup, void*) noexcept
        {
            if (!setup || setup->size < sizeof(*setup)) return;
            auto& s = state();
            std::scoped_lock lock(s.mutex);
            RPSUI_IMGUI_BIND(setup->imgui);
            for (std::size_t i = 0; i < s.fonts.size(); ++i) s.fonts[i] = static_cast<ImFont*>(setup->fonts[i]);
            s.contextGeneration = setup->contextGeneration;
        }

        void RPSUI_CALL drawPanel(const fw::PanelDrawFrame* frame, void*) noexcept
        {
            if (!frame || frame->size < sizeof(*frame) || !frame->theme) return;
            auto& s = state();
            std::scoped_lock lock(s.mutex);
            if (frame->contextGeneration != s.contextGeneration || !s.open.load(std::memory_order_acquire)) return;
            RPSUI_IMGUI_BIND(frame->imgui);
            for (std::size_t i = 0; i < s.fonts.size(); ++i) s.fonts[i] = static_cast<ImFont*>(frame->fonts[i]);
            s.pointerHand = frame->pointerValid ? frame->pointerHand : fw::PhysicalHand::None;
            try {
                // A grip tap on the panel is Back, except while a button is being captured.
                if (frame->backPressed && !(s.session.page == Page::Capture && s.session.listening)) goBack(s.session);

                const auto& theme = *frame->theme;
                const Draw d{ theme, s, s.session, displayContext(s.session) };
                ImGui::SetNextWindowPos({ 0.0f, 0.0f });
                ImGui::SetNextWindowSize({ frame->layoutWidth, frame->layoutHeight });
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { theme.padding * 3.0f, theme.padding * 2.0f });
                ImGui::Begin("##rock-setup", nullptr,
                    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                        ImGuiWindowFlags_NoBringToFrontOnFocus);
                ImGui::PopStyleVar();
                drawHeader(d);
                switch (s.session.page) {
                case Page::Controller: drawController(d); break;
                case Page::Welcome: drawWelcome(d); break;
                case Page::Controls: drawControls(d); break;
                case Page::Capture: drawCapture(d); break;
                case Page::Option:
                    if (s.session.option < s.session.options.size()) drawOption(d);
                    else s.session.page = Page::Review;
                    break;
                case Page::Review: drawReview(d); break;
                case Page::Saved: drawSaved(d); break;
                }
                ImGui::End();
            } catch (...) {
                ROCK_LOG_SAMPLE_WARN(Config, 5000, "Setup wizard panel draw failed");
            }
        }

        // Game thread, every input frame: keeps the controllers captured while open and
        // feeds the capture page.
        void RPSUI_CALL onInput(const rpsui::api::input::Frame* frame, void*) noexcept
        {
            auto& s = state();
            const auto* input = s.input.load(std::memory_order_acquire);
            if (!frame || !input || !s.open.load(std::memory_order_acquire)) return;
            // Renewed every callback while open; it lapses by itself three callbacks after the wizard closes.
            rpsui::api::input::Capture capture{};
            capture.buttons[0] = kCaptureButtons;
            capture.buttons[1] = kCaptureButtons;
            (void)input->capture(s.inputToken.load(std::memory_order_acquire), &capture);

            auto sample = input_remap_runtime::takeBindingCaptureSample();
            std::scoped_lock lock(s.mutex);
            auto& session = s.session;
            if (session.page != Page::Capture || !session.listening) return;
            // A trigger pulled at the panel clicks it.
            const auto triggerBit = 1u << static_cast<unsigned>(fb::Source::Trigger);
            if (s.pointerHand == fw::PhysicalHand::Left) sample[0] &= ~triggerBit;
            if (s.pointerHand == fw::PhysicalHand::Right) sample[1] &= ~triggerBit;
            session.capture.update(wp::CaptureSample{ sample });
            if (session.capture.phase() != wp::Capture::Phase::Done || !session.capture.result()) return;
            session.listening = false;
            const auto& captured = *session.capture.result();
            const auto now = displayContext(session);
            session.hands = wp::handChoices(session.editing, captured.control.isLeft, now);
            session.hand = session.hands.suggested;
            // Keep the function's gesture when it may stay; a press otherwise.
            const auto current = session.bindings[static_cast<std::size_t>(session.editing)];
            const auto home = fb::homeBinding(session.editing);
            session.gesture = current.bound() ? current.gesture : home.gesture;
            if (!wp::gestureAllowed(session.editing, session.gesture)) session.gesture = fb::Gesture::Press;
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
            panel.setupCallback = &setupPanel;
            panel.drawCallback = &drawPanel;
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

        // Frame thread, without the mutex: SteamVR and the settings store are called here,
        // then the result is handed to the panel.
        void applyBindingChoice(State& s, BindingChoice choice, const steamvr_binding::Inspection& inspection)
        {
            std::string error;
            bool applied = false;
            steamvr_binding::Inspection after;
            if (choice == BindingChoice::UseRock) {
                applied = steamvr_binding::apply(inspection, error);
                if (applied) after = steamvr_binding::inspect();
                else ROCK_LOG_WARN(Config, "Setup wizard could not set ROCK's SteamVR binding: {}", error);
            } else if (!g_rockConfig.persistSetting(api::configuration::Group::Consumer, "ControllerInput", "OfferSteamVRBinding",
                           "false", error)) {
                ROCK_LOG_WARN(Config, "Setup wizard could not save ControllerInput.OfferSteamVRBinding: {}", error);
            } else {
                ROCK_LOG_INFO(Config, "Setup wizard: the player keeps SteamVR's binding; no automatic offer from now on");
            }
            std::scoped_lock lock(s.mutex);
            s.bindingChoice = BindingChoice::None;
            auto& session = s.session;
            if (choice == BindingChoice::KeepSteamVr) {
                session.page = Page::Welcome;
                return;
            }
            session.bindingApplied = applied;
            session.bindingError = applied ? std::string{} : error;
            if (applied) session.steamVr = std::move(after);
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

        // Frame thread, without the mutex.
        void close(State& s, const char* reason)
        {
            if (!s.open.exchange(false, std::memory_order_acq_rel)) return;
            s.autoOpened = false;
            {
                std::scoped_lock lock(s.mutex);
                s.saveRequested = false;
                s.closeRequested = false;
            }
            input_remap_runtime::setSetupWizardInput(false);
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
                    if (s.open.load(std::memory_order_acquire)) {
                        // Already open.
                    } else if (input_remap_runtime::isMenuInputActive()) {
                        ROCK_LOG_INFO(Config, "Setup wizard not opened: a game menu has the controllers");
                        if (automatic) {
                            s.resume = resume;
                            s.autoOpen = AutoOpen::WhenFree;
                        }
                    } else {
                        auto steamVr = resume ? steamvr_binding::Inspection{} : steamvr_binding::inspect();
                        {
                            std::scoped_lock lock(s.mutex);
                            if (!resume) {
                                loadSession(s.session);
                                s.session.controllerPage = steamVr.status == steamvr_binding_policy::Status::Offer;
                                s.session.steamVr = std::move(steamVr);
                                if (s.session.controllerPage) s.session.page = Page::Controller;
                            }
                            s.bindingChoice = BindingChoice::None;
                            s.saveRequested = false;
                            s.closeRequested = false;
                        }
                        if (present(s, true)) {
                            input_remap_runtime::setSetupWizardInput(true);
                            s.open.store(true, std::memory_order_release);
                            s.autoOpened = automatic;
                            s.openedThisRun = true;
                            s.autoOpen = AutoOpen::Off;
                            recordShownEdition();
                            ROCK_LOG_INFO(Config, "Setup wizard opened{}", automatic ? " by itself" : "");
                        }
                    }
                }
            }
            if (!s.open.load(std::memory_order_acquire)) {
                autoOpenWhenFree(s);
                return;
            }
            bool closeRequested = false;
            BindingChoice choice = BindingChoice::None;
            steamvr_binding::Inspection inspection;
            {
                std::scoped_lock lock(s.mutex);
                choice = s.bindingChoice;
                if (choice == BindingChoice::UseRock) inspection = s.session.steamVr;
                if (s.saveRequested) {
                    s.saveRequested = false;
                    saveSession(s.session);
                    s.session.original = s.session.bindings;
                    for (auto& option : s.session.options) option.original = option.value;
                    s.session.page = Page::Saved;
                }
                closeRequested = std::exchange(s.closeRequested, false);
            }
            if (choice != BindingChoice::None) applyBindingChoice(s, choice, inspection);
            if (closeRequested) {
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
