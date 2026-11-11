#include "setup-wizard/SetupWizardPanel.h"

#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>

#include <RPS_UI_Framework/BindingIcons.h>
#include <RPS_UI_Framework/ImGuiBuild.h>

#include <imgui.h>
#include <imgui_internal.h>

#include "physics-interaction/PhysicsLog.h"

namespace rock::setup_wizard
{
    namespace
    {
        namespace fw = rpsui::api::framework;
        namespace bt = rpsui::binding;

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

        [[nodiscard]] constexpr const char* pageLabel(Page page) noexcept
        {
            switch (page) {
            case Page::Controller: return "Controller";
            case Page::Welcome: return "Start";
            case Page::Controls: return "Controls";
            case Page::Capture: return "Capture";
            case Page::Option: return "Option";
            case Page::Review: return "Review";
            case Page::Saved: return "Saved";
            }
            return "";
        }

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
    }

    struct WizardPanel::Guarded
    {
        BindingChoice bindingChoice{ BindingChoice::None };
        bool saveRequested{ false };
        bool closeRequested{ false };
        // The pointer's hand while it is on the panel: its trigger is a click, not a capture.
        fw::PhysicalHand pointerHand{ fw::PhysicalHand::None };
        std::uint64_t contextGeneration{ 0 };
        std::array<ImFont*, fw::kFontRoleCount> fonts{};
        Session session;
    };

    WizardPanel::WizardPanel() : _guarded(std::make_unique<Guarded>()) {}
    WizardPanel::~WizardPanel() = default;

    namespace
    {
        using Guarded = WizardPanel::Guarded;

        [[nodiscard]] fb::Context displayContext(const Session& session) noexcept
        {
            fb::Context context{};
            for (const auto& option : session.options)
                if (option.page.section == "FiringHand" && option.page.key == "DefaultHand")
                    context.firingIsLeft = option.value == "left";
            context.primaryIsLeft = false;
            return context;
        }

        // ---- drawing -----------------------------------------------------------------------

        struct Draw
        {
            const fw::Theme& theme;
            Guarded& g;
            Session& session;
            fb::Context now;

            [[nodiscard]] ImU32 ink(const fw::ThemeColor& color, float alpha = 1.0f) const
            {
                return ImGui::GetColorU32(rpsui::imgui_host::color(color, alpha));
            }

            [[nodiscard]] ImFont* face(fw::FontRole role) const { return g.fonts[static_cast<std::size_t>(role)]; }

            void font(fw::FontRole role, float size) const { ImGui::PushFont(face(role), size); }

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
                auto* strong = face(fw::FontRole::Strong);
                draw->AddText(strong, theme.fontSize, { lo.x + theme.padding * 1.5f, lo.y + theme.padding }, ink(textColor),
                    label.data(), label.data() + label.size());
                if (!badge.empty()) {
                    const float badgeWidth = strong->CalcTextSizeA(theme.smallFontSize, 10000.0f, 0.0f, badge.data(),
                        badge.data() + badge.size()).x;
                    draw->AddText(strong, theme.smallFontSize, { hi.x - theme.padding * 1.5f - badgeWidth, lo.y + theme.padding },
                        ink(theme.textMuted), badge.data(), badge.data() + badge.size());
                }
                if (!detail.empty()) {
                    draw->AddText(face(fw::FontRole::Prose), theme.smallFontSize,
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
        [[nodiscard]] std::optional<fb::Binding> candidate(const Session& session)
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
            if (ImGui::Button("Close##header", { closeWidth, 0.0f })) d.g.closeRequested = true;
            ImGui::Dummy({ 0.0f, d.theme.spacing });
        }

        void drawController(const Draw& d)
        {
            auto& session = d.session;
            const auto& controller = session.steamVr.controller;
            const std::string name{ controller ? controller->displayName : "controllers" };
            const bool index = controller && controller->controller == steamvr_binding_policy::Controller::Index;
            const bool pending = d.g.bindingChoice != BindingChoice::None;
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
                    d.g.bindingChoice = BindingChoice::UseRock;
                    session.bindingError.clear();
                }
                if (d.card("keep-steamvr", "Keep SteamVR's binding",
                        "Nothing changes in SteamVR. ROCK stops opening this page by itself; it is still here when you "
                        "open the setup wizard.",
                        false, !pending)) {
                    d.g.bindingChoice = BindingChoice::KeepSteamVr;
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
            if (d.button("Close")) d.g.closeRequested = true;
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
                draw->AddText(d.face(fw::FontRole::Strong), d.theme.fontSize,
                    { lo.x + d.theme.padding * 1.5f, lo.y + d.theme.padding * 0.6f }, d.ink(d.theme.text), text.name);
                std::string note = conflicted ? std::string("Shares its button and gesture with another action")
                                              : std::string(text.detail);
                if (!conflicted && d.session.bindings[i].bound()) {
                    note += "  On ";
                    note += handRole(d.session.bindings[i].control.hand);
                    note += '.';
                }
                draw->AddText(d.face(fw::FontRole::Prose), d.theme.smallFontSize,
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

            const auto chosen = candidate(session);
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
            if (d.button("Save", true, any || anyOption, d.theme.targetHeight * 3.0f)) d.g.saveRequested = true;
        }

        void drawSaved(const Draw& d)
        {
            d.beginBody(d.footerHeight());
            d.heading(d.session.saveErrors.empty() ? "Saved" : "Some settings were not saved");
            ImGui::Dummy({ 0.0f, d.theme.spacing });
            if (d.session.saveErrors.empty()) d.prose("ROCK uses the new controls and options now.", false);
            for (const auto& error : d.session.saveErrors) d.prose(error, false);
            d.endBody();
            if (d.button("Close", true)) d.g.closeRequested = true;
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

        // Every changed value of the session, bindings first, in save order.
        [[nodiscard]] std::vector<SettingWrite> changes(const Session& session)
        {
            std::vector<SettingWrite> writes;
            for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                if (session.bindings[i] == session.original[i]) continue;
                writes.push_back({ "Bindings", std::string(fb::functionKey(static_cast<fb::Function>(i))),
                    fb::formatBinding(session.bindings[i]) });
            }
            for (const auto& option : session.options) {
                if (option.value == option.original) continue;
                writes.push_back({ std::string(option.page.section), std::string(option.page.key), option.value });
            }
            return writes;
        }
    }

    // ---- settings ------------------------------------------------------------------------------

    std::vector<OptionState> optionPages()
    {
        std::vector<OptionState> options;
        options.reserve(wp::kOptionPages.size());
        for (const auto& page : wp::kOptionPages) options.push_back({ .page = page });
        return options;
    }

    void collectOption(const api::configuration::Setting& setting, void* opaque) noexcept
    {
        auto& options = *static_cast<std::vector<OptionState>*>(opaque);
        if (!setting.section || !setting.key) return;
        try {
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
        } catch (...) {
            // An allocation failure leaves the page unfound; the wizard skips it.
        }
    }

    // ---- host side -----------------------------------------------------------------------------

    void WizardPanel::begin(SessionStart start)
    {
        std::scoped_lock lock(_mutex);
        auto& session = _guarded->session;
        session = {};
        session.bindings = start.bindings;
        session.original = start.bindings;
        session.options = std::move(start.options);
        session.controllerPage = start.steamVr.status == steamvr_binding_policy::Status::Offer;
        session.steamVr = std::move(start.steamVr);
        if (session.controllerPage) session.page = Page::Controller;
    }

    void WizardPanel::clearRequests()
    {
        std::scoped_lock lock(_mutex);
        _guarded->bindingChoice = BindingChoice::None;
        _guarded->saveRequested = false;
        _guarded->closeRequested = false;
    }

    Requests WizardPanel::takeRequests()
    {
        std::scoped_lock lock(_mutex);
        auto& g = *_guarded;
        Requests requests{};
        requests.bindingChoice = g.bindingChoice;
        if (g.bindingChoice == BindingChoice::UseRock) requests.steamVr = g.session.steamVr;
        requests.save = std::exchange(g.saveRequested, false);
        if (requests.save) requests.writes = changes(g.session);
        requests.close = std::exchange(g.closeRequested, false);
        return requests;
    }

    void WizardPanel::finishSave(const SaveResult& result)
    {
        std::scoped_lock lock(_mutex);
        auto& session = _guarded->session;
        session.saveErrors = result.errors;
        // Only what was written becomes the saved state: a value changed while the host
        // wrote, or one that failed, stays a change.
        for (const auto& write : result.written) {
            if (write.section == "Bindings") {
                for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                    const auto function = static_cast<fb::Function>(i);
                    if (fb::functionKey(function) != write.key) continue;
                    if (fb::formatBinding(session.bindings[i]) == write.value) session.original[i] = session.bindings[i];
                    else if (const auto parsed = fb::parseBinding(write.value)) session.original[i] = *parsed;
                }
                continue;
            }
            for (auto& option : session.options)
                if (option.page.section == write.section && option.page.key == write.key) option.original = write.value;
        }
        session.page = Page::Saved;
    }

    void WizardPanel::finishBindingChoice(BindingChoice choice, bool applied, std::string error,
        steamvr_binding::Inspection after)
    {
        std::scoped_lock lock(_mutex);
        _guarded->bindingChoice = BindingChoice::None;
        auto& session = _guarded->session;
        if (choice == BindingChoice::KeepSteamVr) {
            session.page = Page::Welcome;
            return;
        }
        session.bindingApplied = applied;
        session.bindingError = applied ? std::string{} : std::move(error);
        if (applied) session.steamVr = std::move(after);
    }

    std::string WizardPanel::pageName()
    {
        std::scoped_lock lock(_mutex);
        const auto& session = _guarded->session;
        std::string name = pageLabel(session.page);
        if (session.page == Page::Option)
            name += " " + std::to_string(session.option + 1) + " of " + std::to_string(session.options.size());
        if (session.page == Page::Capture) {
            name += ": ";
            name += functionText(session.editing).name;
            name += session.listening ? " (listening)" : " (choosing hand and gesture)";
        }
        return name;
    }

    void WizardPanel::feedCapture(std::array<std::uint32_t, 2> sample)
    {
        if (!isOpen()) return;
        std::scoped_lock lock(_mutex);
        auto& g = *_guarded;
        auto& session = g.session;
        if (session.page != Page::Capture || !session.listening) return;
        // A trigger pulled at the panel clicks it.
        const auto triggerBit = 1u << static_cast<unsigned>(fb::Source::Trigger);
        if (g.pointerHand == fw::PhysicalHand::Left) sample[0] &= ~triggerBit;
        if (g.pointerHand == fw::PhysicalHand::Right) sample[1] &= ~triggerBit;
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

    // ---- RPS UI callbacks ----------------------------------------------------------------------

    void RPSUI_CALL WizardPanel::setupCallback(const fw::PanelSetup* setup, void* userData) noexcept
    {
        if (!setup || setup->size < sizeof(*setup) || !userData) return;
        try {
            static_cast<WizardPanel*>(userData)->setup(*setup);
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Config, 5000, "Setup wizard panel setup failed");
        }
    }

    void RPSUI_CALL WizardPanel::drawCallback(const fw::PanelDrawFrame* frame, void* userData) noexcept
    {
        if (!frame || frame->size < sizeof(*frame) || !frame->theme || !userData) return;
        try {
            static_cast<WizardPanel*>(userData)->draw(*frame);
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Config, 5000, "Setup wizard panel draw failed");
        }
    }

    void WizardPanel::setup(const fw::PanelSetup& setup)
    {
        std::scoped_lock lock(_mutex);
        RPSUI_IMGUI_BIND(setup.imgui);
        for (std::size_t i = 0; i < _guarded->fonts.size(); ++i) _guarded->fonts[i] = static_cast<ImFont*>(setup.fonts[i]);
        _guarded->contextGeneration = setup.contextGeneration;
    }

    void WizardPanel::draw(const fw::PanelDrawFrame& frame)
    {
        std::scoped_lock lock(_mutex);
        auto& g = *_guarded;
        if (frame.contextGeneration != g.contextGeneration || !isOpen()) return;
        RPSUI_IMGUI_BIND(frame.imgui);
        for (std::size_t i = 0; i < g.fonts.size(); ++i) g.fonts[i] = static_cast<ImFont*>(frame.fonts[i]);
        g.pointerHand = frame.pointerValid ? frame.pointerHand : fw::PhysicalHand::None;
        // A grip tap on the panel is Back, except while a button is being captured.
        if (frame.backPressed && !(g.session.page == Page::Capture && g.session.listening)) goBack(g.session);

        const auto& theme = *frame.theme;
        const Draw d{ theme, g, g.session, displayContext(g.session) };
        ImGui::SetNextWindowPos({ 0.0f, 0.0f });
        ImGui::SetNextWindowSize({ frame.layoutWidth, frame.layoutHeight });
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { theme.padding * 3.0f, theme.padding * 2.0f });
        ImGui::Begin("##rock-setup", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::PopStyleVar();
        drawHeader(d);
        switch (g.session.page) {
        case Page::Controller: drawController(d); break;
        case Page::Welcome: drawWelcome(d); break;
        case Page::Controls: drawControls(d); break;
        case Page::Capture: drawCapture(d); break;
        case Page::Option:
            if (g.session.option < g.session.options.size()) drawOption(d);
            else g.session.page = Page::Review;
            break;
        case Page::Review: drawReview(d); break;
        case Page::Saved: drawSaved(d); break;
        }
        ImGui::End();
    }
}
