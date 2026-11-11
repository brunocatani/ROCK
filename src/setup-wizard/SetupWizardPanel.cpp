#include "setup-wizard/SetupWizardPanel.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <utility>

#include <RPS_UI_Framework/ImGuiBuild.h>
#include <RPS_UI_Framework/ThemeStyle.h>

#include <imgui.h>
#include <imgui_internal.h>

#include "physics-interaction/PhysicsLog.h"

namespace rock::setup_wizard
{
    namespace
    {
        namespace fw = rpsui::api::framework;
        namespace sv = steamvr_binding_policy;
        using sv::Headset;

        [[nodiscard]] constexpr const char* actionName(fb::Function function) noexcept
        {
            switch (function) {
            case fb::Function::Grab: return "Grab";
            case fb::Function::Fire: return "Fire";
            case fb::Function::SupportTrigger: return "Support trigger";
            case fb::Function::Use: return "Use";
            case fb::Function::Reload: return "Reload";
            case fb::Function::Scope: return "Scope";
            case fb::Function::Vats: return "VATS";
            case fb::Function::Grenade: return "Grenade";
            case fb::Function::Pipboy: return "Pip-Boy";
            case fb::Function::Pause: return "Pause menu";
            case fb::Function::StickActions: return "Bipod and Favorites";
            case fb::Function::Palm: return "RobCo PALM";
            case fb::Function::Count: break;
            }
            return "";
        }

        struct HeadsetText
        {
            Headset headset;
            const char* label;
            const char* detail;
        };

        constexpr std::array kHeadsets{
            HeadsetText{ Headset::Index, "Valve Index controllers", "ROCK installs its SteamVR binding for them." },
            HeadsetText{ Headset::ViveWands, "HTC Vive wands", "ROCK installs its SteamVR binding for them." },
            HeadsetText{ Headset::NoBindingNeeded, "Quest, Rift, Pico, Vive Focus 3 or Steam Frame",
                "Every button works as it is. ROCK installs nothing." },
            HeadsetText{ Headset::Other, "Other controllers", "ROCK has no binding for them. SteamVR's binding stays." },
        };

        [[nodiscard]] std::string controllerName(sv::Controller controller)
        {
            const auto info = sv::controllerFor(controller == sv::Controller::Index ? "knuckles" : "vive_controller");
            return info ? std::string(info->displayName) : std::string("controllers");
        }

        // The actions on A and X, which SteamVR's default binding takes from Index controllers and wands.
        [[nodiscard]] std::string actionsOnAandX(const fb::Bindings& bindings)
        {
            std::vector<const char*> names;
            for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
                const auto& binding = bindings[i];
                if (binding.control.source == fb::Source::LowerFace || binding.modifier.source == fb::Source::LowerFace)
                    names.push_back(actionName(static_cast<fb::Function>(i)));
            }
            std::string text;
            for (std::size_t i = 0; i < names.size(); ++i) {
                if (i) text += i + 1 == names.size() ? " and " : ", ";
                text += names[i];
            }
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
            Controllers,
            Topic,
            Review,
            Finished,
        };

        struct Session
        {
            Page page{ Page::Controllers };
            // Index into `topics`.
            std::size_t topic{ 0 };
            // The wp::kPages that have an option the store knows, in order.
            std::vector<std::size_t> topics;
            std::optional<Headset> chosen;
            SessionStart start;
            // The last page.
            wp::Finish finish{ wp::Finish::Save };
            wp::ControllerPlan plan;
            FinishResult result;
        };

        [[nodiscard]] wp::ControllerPlan plan(const Session& session)
        {
            const auto& steamVr = session.start.steamVr;
            return wp::planControllers(session.chosen, steamVr.detected(), steamVr.index.status, steamVr.vive.status,
                session.start.offeringBinding);
        }

        // The rule fixing an option to one choice while another option's value demands it.
        [[nodiscard]] const wp::FixedWhen* activeFixed(const std::vector<OptionState>& options, const OptionState& state)
        {
            const auto& fixed = state.option->fixedWhen;
            if (fixed.key.empty()) return nullptr;
            const auto other =
                std::ranges::find_if(options, [&](const OptionState& o) { return o.option->key == fixed.key; });
            return other != options.end() && other->value == fixed.value ? &fixed : nullptr;
        }

        // Whether an option shows now: always, or while another option has the value it needs.
        [[nodiscard]] bool shown(const std::vector<OptionState>& options, const OptionState& state)
        {
            const auto& when = state.option->shownWhen;
            if (when.key.empty()) return true;
            const auto other =
                std::ranges::find_if(options, [&](const OptionState& o) { return o.option->key == when.key; });
            return other != options.end() && other->value == when.value;
        }

        // Puts every fixed option on its one choice, so what the page shows is what is saved.
        void applyFixed(std::vector<OptionState>& options)
        {
            for (auto& state : options)
                if (const auto* fixed = activeFixed(options, state)) state.value = std::string(fixed->fixed);
        }
    }

    struct WizardPanel::Guarded
    {
        std::optional<wp::Finish> finishRequested;
        bool closeRequested{ false };
        std::uint64_t contextGeneration{ 0 };
        std::array<ImFont*, fw::kFontRoleCount> fonts{};
        Session session;
    };

    WizardPanel::WizardPanel() : _guarded(std::make_unique<Guarded>()) {}
    WizardPanel::~WizardPanel() = default;

    namespace
    {
        using Guarded = WizardPanel::Guarded;

        // ---- drawing -----------------------------------------------------------------------

        struct Draw
        {
            const fw::Theme& theme;
            Guarded& g;
            Session& session;

            [[nodiscard]] ImU32 ink(const fw::ThemeColor& color, float alpha = 1.0f) const
            {
                return ImGui::GetColorU32(rpsui::imgui_host::color(color, alpha));
            }

            [[nodiscard]] ImFont* face(fw::FontRole role) const { return g.fonts[static_cast<std::size_t>(role)]; }

            void font(fw::FontRole role, float size) const { ImGui::PushFont(face(role), size); }

            void heading(std::string_view text) const
            {
                font(fw::FontRole::Strong, theme.titleFontSize);
                ImGui::TextUnformatted(text.data(), text.data() + text.size());
                ImGui::PopFont();
            }

            void strong(std::string_view text) const
            {
                font(fw::FontRole::Strong, theme.fontSize);
                ImGui::TextUnformatted(text.data(), text.data() + text.size());
                ImGui::PopFont();
            }

            void prose(const std::string_view text, bool muted = true, float size = 0.0f) const
            {
                font(fw::FontRole::Prose, size > 0.0f ? size : theme.fontSize);
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(muted ? theme.textMuted : theme.text));
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
                ImGui::TextUnformatted(text.data(), text.data() + text.size());
                ImGui::PopTextWrapPos();
                ImGui::PopStyleColor();
                ImGui::PopFont();
            }

            void warning(const std::string_view text) const
            {
                font(fw::FontRole::Prose, theme.fontSize);
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(theme.pending));
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
                ImGui::TextUnformatted(text.data(), text.data() + text.size());
                ImGui::PopTextWrapPos();
                ImGui::PopStyleColor();
                ImGui::PopFont();
            }

            void gap(float spacings = 1.0f) const { ImGui::Dummy({ 0.0f, theme.spacing * spacings }); }

            void pushAccent() const
            {
                ImGui::PushStyleColor(ImGuiCol_Button, rpsui::imgui_host::color(theme.accent));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, rpsui::imgui_host::color(theme.accent, 0.85f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, rpsui::imgui_host::color(theme.pressed));
                ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(theme.onAccent));
            }

            // A footer button; the accent one is the page's main action.
            [[nodiscard]] bool button(const char* label, bool accent = false, bool enabled = true, float width = 0.0f) const
            {
                const float height = theme.targetHeight * 1.15f;
                if (accent) pushAccent();
                ImGui::BeginDisabled(!enabled);
                const bool pressed = ImGui::Button(label, { width > 0.0f ? width : 0.0f, height });
                ImGui::EndDisabled();
                if (accent) ImGui::PopStyleColor(4);
                return pressed && enabled;
            }

            // A large choice: label, detail, the selection shown by the accent edge.
            [[nodiscard]] bool card(const char* id, const std::string_view label, const std::string_view detail,
                bool selected, std::string_view badge = {}) const
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
                const bool clicked = ImGui::InvisibleButton("card", { width, height });
                const bool hovered = ImGui::IsItemHovered();
                ImGui::PopID();
                const ImVec2 hi{ lo.x + width, lo.y + height };
                auto* draw = ImGui::GetWindowDrawList();
                draw->AddRectFilled(lo, hi, ink(selected ? theme.selected : hovered ? theme.hover : theme.surface),
                    theme.rounding);
                draw->AddRect(lo, hi, ink(selected ? theme.accent : theme.border), theme.rounding,
                    selected ? theme.borderWidth * 3.0f : theme.borderWidth);
                auto* strongFace = face(fw::FontRole::Strong);
                draw->AddText(strongFace, theme.fontSize, { lo.x + theme.padding * 1.5f, lo.y + theme.padding }, ink(theme.text),
                    label.data(), label.data() + label.size());
                if (!badge.empty()) {
                    const float badgeWidth = strongFace->CalcTextSizeA(theme.smallFontSize, 10000.0f, 0.0f, badge.data(),
                        badge.data() + badge.size()).x;
                    draw->AddText(strongFace, theme.smallFontSize,
                        { hi.x - theme.padding * 1.5f - badgeWidth, lo.y + theme.padding }, ink(theme.accent), badge.data(),
                        badge.data() + badge.size());
                }
                if (!detail.empty()) {
                    draw->AddText(face(fw::FontRole::Prose), theme.smallFontSize,
                        { lo.x + theme.padding * 1.5f, lo.y + theme.padding + theme.fontSize * 1.3f + theme.spacing * 0.5f },
                        ink(theme.textMuted), detail.data(), detail.data() + detail.size(), width - theme.padding * 3.0f);
                }
                ImGui::Dummy({ 0.0f, theme.spacing * 0.5f });
                return clicked;
            }

            // One option: its title, a button per choice, and what the chosen one does. While
            // another option fixes it (`fixed`), only that choice can be picked and the reason shows.
            void option(OptionState& state, const wp::FixedWhen* fixed) const
            {
                const auto& option = *state.option;
                ImGui::PushID(option.key.data(), option.key.data() + option.key.size());
                ImGui::PushID(option.section.data(), option.section.data() + option.section.size());
                strong(option.title);
                const std::size_t count = option.choiceCount();
                const float width = (ImGui::GetContentRegionAvail().x - theme.spacing * static_cast<float>(count - 1)) /
                    static_cast<float>(count);
                for (std::size_t i = 0; i < count; ++i) {
                    const auto& choice = option.choices[i];
                    if (i) ImGui::SameLine(0.0f, theme.spacing);
                    const bool selected = state.value == choice.value;
                    const bool unavailable = fixed && choice.value != fixed->fixed;
                    if (selected) rpsui::imgui_host::pushChosen(theme);
                    ImGui::BeginDisabled(unavailable);
                    ImGui::PushID(static_cast<int>(i));
                    if (ImGui::Button(std::string(choice.label).c_str(), { width, theme.targetHeight }))
                        state.value = std::string(choice.value);
                    ImGui::PopID();
                    ImGui::EndDisabled();
                    if (selected) rpsui::imgui_host::popChosen();
                }
                if (const auto* chosen = option.find(state.value)) prose(chosen->detail, true, theme.smallFontSize);
                if (fixed) prose(fixed->reason, true, theme.smallFontSize);
                else if (state.value != state.defaultValue)
                    if (const auto* recommended = option.find(state.defaultValue))
                        prose("ROCK recommends: " + std::string(recommended->label) + ".", true, theme.smallFontSize);
                ImGui::PopID();
                ImGui::PopID();
                gap(1.5f);
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
                gap();
            }

            [[nodiscard]] float footerHeight() const { return theme.targetHeight * 1.15f + theme.spacing * 3.0f; }

            [[nodiscard]] float mainWidth() const { return theme.targetHeight * 4.0f; }
        };

        [[nodiscard]] std::size_t stepCount(const Session& session) { return session.topics.size() + 2; }

        void drawHeader(const Draw& d)
        {
            const auto& session = d.session;
            d.font(fw::FontRole::Strong, d.theme.fontSize);
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted("ROCK SETUP");
            ImGui::PopStyleColor();
            ImGui::PopFont();
            std::string step;
            switch (session.page) {
            case Page::Controllers: step = "Step 1 of " + std::to_string(stepCount(session)); break;
            case Page::Topic:
                step = "Step " + std::to_string(session.topic + 2) + " of " + std::to_string(stepCount(session));
                break;
            case Page::Review: step = "Step " + std::to_string(stepCount(session)) + " of " + std::to_string(stepCount(session)); break;
            case Page::Finished: step = "Done"; break;
            }
            const float closeWidth = d.theme.targetHeight * 2.4f;
            ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Text, rpsui::imgui_host::color(d.theme.textMuted));
            ImGui::TextUnformatted(step.c_str());
            ImGui::PopStyleColor();
            ImGui::SameLine(contentRight() - closeWidth);
            if (ImGui::Button("Close##header", { closeWidth, 0.0f })) {
                // Closing before the end is a finish too: ROCK still installs the binding it would.
                if (session.page == Page::Finished) d.g.closeRequested = true;
                else d.g.finishRequested = wp::Finish::Close;
            }
            d.gap();
        }

        // What the chosen controllers get, in words.
        void drawControllerOutcome(const Draw& d, Headset headset)
        {
            const auto& start = d.session.start;
            const auto controller = sv::controllerOf(headset);
            if (!controller) {
                d.prose(headset == Headset::NoBindingNeeded
                        ? "Every action works with these controllers as they are. ROCK installs nothing."
                        : "ROCK has no binding for these controllers. Fallout 4 VR keeps the binding SteamVR gives it.",
                    false);
                return;
            }
            const auto name = controllerName(*controller);
            if (!start.steamVr.steamVr) {
                d.prose("Fallout 4 VR runs through OpenComposite here, which does not use SteamVR bindings. ROCK installs "
                        "nothing.",
                    false);
                return;
            }
            const auto& inspection = start.steamVr.of(*controller);
            switch (inspection.status) {
            case sv::Status::RockActive:
                d.prose("ROCK's binding for your " + name + " is already set. Every action works.", false);
                return;
            case sv::Status::PlayersOwn:
                d.prose("You chose your own binding for Fallout 4 VR in SteamVR. ROCK leaves it as it is.", false);
                return;
            case sv::Status::Offer: {
                const auto broken = actionsOnAandX(start.bindings);
                d.prose(std::string(*controller == sv::Controller::Index
                                ? "SteamVR's own binding makes Fallout 4 VR treat Index controllers as Vive wands, and A "
                                  "and X act as grip."
                                : "SteamVR's own binding gives Fallout 4 VR wands without A and X.") +
                        (broken.empty() ? std::string{} : " These actions do not work: " + broken + ".") +
                        " ROCK's binding gives the game every button.",
                    false);
                d.gap(0.5f);
                d.warning("ROCK installs its binding for your " + name +
                          " when you finish, for Fallout 4 VR only. Restart the game afterwards.");
                return;
            }
            case sv::Status::NotApplicable:
                d.prose("SteamVR does not tell ROCK this game's binding now, so ROCK installs nothing. Open the wizard "
                        "again from PALM's Config once SteamVR runs the game.",
                    false);
                return;
            }
        }

        void drawControllers(const Draw& d)
        {
            auto& session = d.session;
            const auto detected = session.start.steamVr.detected();
            d.beginBody(d.footerHeight());
            d.heading("Which controllers do you play with?");
            d.gap(0.5f);
            d.prose("Some controllers need ROCK's SteamVR binding for every button to reach the game. Choose yours and "
                    "ROCK tells you what it does.",
                false);
            d.gap(1.5f);
            for (const auto& headset : kHeadsets) {
                const bool selected = session.chosen == headset.headset;
                if (d.card(headset.label, headset.label, headset.detail, selected,
                        detected == headset.headset ? "Detected" : ""))
                    session.chosen = headset.headset;
            }
            d.gap();
            if (session.chosen) drawControllerOutcome(d, *session.chosen);
            else d.prose("\"Detected\" marks the controllers SteamVR reports now.");
            d.gap(1.5f);
            d.prose("Use recommended settings finishes now: ROCK's binding where your controllers need it and ROCK's "
                    "recommendation for every option, saved at once.",
                true, d.theme.smallFontSize);
            d.endBody();
            if (d.button("Use recommended settings")) d.g.finishRequested = wp::Finish::Recommended;
            ImGui::SameLine(contentRight() - d.mainWidth());
            if (d.button("Next", true, session.chosen.has_value(), d.mainWidth())) {
                session.topic = 0;
                session.page = session.topics.empty() ? Page::Review : Page::Topic;
            }
        }

        void drawTopic(const Draw& d)
        {
            auto& session = d.session;
            const auto pageIndex = session.topics[session.topic];
            const auto& page = wp::kPages[pageIndex];
            d.beginBody(d.footerHeight());
            d.heading(page.title);
            d.prose(page.intro);
            d.gap(1.5f);
            // In page order, so a choice fixes the options below it in the same frame.
            for (auto& option : session.start.options) {
                if (option.option->page != pageIndex || !shown(session.start.options, option)) continue;
                const auto* fixed = activeFixed(session.start.options, option);
                if (fixed) option.value = std::string(fixed->fixed);
                d.option(option, fixed);
            }
            d.endBody();
            if (d.button("Back")) {
                if (session.topic == 0) session.page = Page::Controllers;
                else --session.topic;
            }
            ImGui::SameLine(contentRight() - d.mainWidth());
            if (d.button("Next", true, true, d.mainWidth())) {
                if (session.topic + 1 < session.topics.size()) ++session.topic;
                else session.page = Page::Review;
            }
        }

        [[nodiscard]] std::string choiceLabel(const OptionState& option, std::string_view value)
        {
            const auto* choice = option.option->find(value);
            return choice ? std::string(choice->label) : std::string(value);
        }

        void drawReview(const Draw& d)
        {
            auto& session = d.session;
            d.beginBody(d.footerHeight());
            d.heading("Review");
            d.gap();
            d.strong("Controllers");
            const auto controllers = plan(session);
            if (controllers.action == wp::BindingAction::Install)
                d.warning("ROCK installs its SteamVR binding for your " + controllerName(*controllers.controller) +
                          ". Restart the game afterwards.");
            else d.prose("Nothing to install.", false);
            d.gap();
            d.strong("Options");
            bool any = false;
            for (const auto& option : session.start.options) {
                if (option.value == option.original) continue;
                any = true;
                d.prose(std::string(option.option->title) + ": " + choiceLabel(option, option.original) + "  >  " +
                        choiceLabel(option, option.value),
                    false);
            }
            if (!any) d.prose("No option changed.");
            d.endBody();
            if (d.button("Back")) {
                if (session.topics.empty()) session.page = Page::Controllers;
                else {
                    session.page = Page::Topic;
                    session.topic = session.topics.size() - 1;
                }
            }
            ImGui::SameLine(contentRight() - d.mainWidth());
            if (d.button("Save", true, true, d.mainWidth())) d.g.finishRequested = wp::Finish::Save;
        }

        void drawFinished(const Draw& d)
        {
            const auto& session = d.session;
            const auto& result = session.result;
            d.beginBody(d.footerHeight());
            const bool failed = !result.errors.empty() || !result.bindingError.empty();
            d.heading(failed ? "Not everything was set" : session.finish == wp::Finish::Close ? "Before you play" : "ROCK is set up");
            d.gap();
            if (result.bindingInstalled) {
                d.prose("ROCK installed its SteamVR binding for your " + controllerName(*session.plan.controller) + ".", false);
                d.gap(0.5f);
                d.warning("Quit and start Fallout 4 VR again before you play: SteamVR loads the game's binding when the "
                          "game starts.");
                d.gap();
            }
            if (!result.bindingError.empty()) {
                d.prose("ROCK could not install its SteamVR binding: " + result.bindingError +
                            ". Your controllers keep SteamVR's binding.",
                    false);
                d.gap();
            }
            if (session.finish == wp::Finish::Recommended) d.prose("Every option is at ROCK's recommendation.", false);
            else if (!result.written.empty() && result.errors.empty())
                d.prose("ROCK uses your options now.", false);
            for (const auto& error : result.errors) d.prose(error, false);
            d.gap();
            d.prose("To change anything later, open this wizard again from PALM's Config, on the ROCK page.");
            d.endBody();
            ImGui::SetCursorPosX(contentRight() - d.mainWidth());
            if (d.button("Close", true, true, d.mainWidth())) d.g.closeRequested = true;
        }

        void goBack(Session& session)
        {
            switch (session.page) {
            case Page::Topic:
                if (session.topic == 0) session.page = Page::Controllers;
                else --session.topic;
                break;
            case Page::Review:
                if (session.topics.empty()) session.page = Page::Controllers;
                else {
                    session.page = Page::Topic;
                    session.topic = session.topics.size() - 1;
                }
                break;
            default: break;
            }
        }
    }

    // ---- settings ------------------------------------------------------------------------------

    void collectSettings(const api::configuration::Setting& setting, void* opaque) noexcept
    {
        auto& start = *static_cast<SessionStart*>(opaque);
        if (!setting.section || !setting.key) return;
        const std::string_view section{ setting.section }, key{ setting.key };
        try {
            if (section == "ControllerInput" && key == "OfferSteamVRBinding") {
                start.offeringBinding = setting.value && std::string_view{ setting.value } == "true";
                return;
            }
            for (const auto& option : wp::kOptions) {
                if (option.section != section || option.key != key) continue;
                OptionState state{ &option, setting.value ? setting.value : "", {},
                    setting.defaultValue ? setting.defaultValue : "" };
                state.original = state.value;
                start.options.push_back(std::move(state));
            }
        } catch (...) {
            // An allocation failure leaves the option out; the wizard skips it.
        }
    }

    // ---- host side -----------------------------------------------------------------------------

    void WizardPanel::begin(SessionStart start)
    {
        // Page order, whatever order the store visits in.
        std::ranges::stable_sort(start.options,
            [](const OptionState& a, const OptionState& b) { return a.option < b.option; });
        std::scoped_lock lock(_mutex);
        auto& session = _guarded->session;
        session = {};
        session.start = std::move(start);
        for (std::size_t page = 0; page < wp::kPages.size(); ++page)
            if (std::ranges::any_of(session.start.options, [&](const OptionState& o) { return o.option->page == page; }))
                session.topics.push_back(page);
    }

    void WizardPanel::clearRequests()
    {
        std::scoped_lock lock(_mutex);
        _guarded->finishRequested.reset();
        _guarded->closeRequested = false;
    }

    Requests WizardPanel::takeRequests()
    {
        std::scoped_lock lock(_mutex);
        auto& g = *_guarded;
        auto& session = g.session;
        Requests requests{};
        requests.close = std::exchange(g.closeRequested, false);
        requests.finish = std::exchange(g.finishRequested, std::nullopt);
        if (!requests.finish) return requests;
        if (*requests.finish == wp::Finish::Recommended)
            for (auto& option : session.start.options) option.value = option.defaultValue;
        applyFixed(session.start.options);
        if (*requests.finish != wp::Finish::Close)
            for (const auto& option : session.start.options)
                if (option.value != option.original)
                    requests.writes.push_back(
                        { std::string(option.option->section), std::string(option.option->key), option.value });
        requests.binding = plan(session);
        if (requests.binding.action == wp::BindingAction::StopOffering)
            requests.writes.push_back({ "ControllerInput", "OfferSteamVRBinding", "false" });
        if (requests.binding.controller) requests.inspection = session.start.steamVr.of(*requests.binding.controller);
        return requests;
    }

    bool WizardPanel::finished(wp::Finish finish, const FinishResult& result)
    {
        std::scoped_lock lock(_mutex);
        auto& session = _guarded->session;
        for (const auto& write : result.written)
            for (auto& option : session.start.options)
                if (option.option->section == write.section && option.option->key == write.key) option.original = write.value;
        session.finish = finish;
        session.plan = plan(session);
        session.result = result;
        const bool show = finish != wp::Finish::Close || result.bindingInstalled || !result.bindingError.empty() ||
            !result.errors.empty();
        if (show) session.page = Page::Finished;
        return show;
    }

    std::string WizardPanel::pageName()
    {
        std::scoped_lock lock(_mutex);
        const auto& session = _guarded->session;
        switch (session.page) {
        case Page::Controllers: return "Controllers";
        case Page::Topic:
            return std::string(wp::kPages[session.topics[session.topic]].title) + " (" + std::to_string(session.topic + 1) +
                " of " + std::to_string(session.topics.size()) + ")";
        case Page::Review: return "Review";
        case Page::Finished: return "Done";
        }
        return "";
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
        // A grip tap on the panel is Back.
        if (frame.backPressed) goBack(g.session);

        const auto& theme = *frame.theme;
        const Draw d{ theme, g, g.session };
        ImGui::SetNextWindowPos({ 0.0f, 0.0f });
        ImGui::SetNextWindowSize({ frame.layoutWidth, frame.layoutHeight });
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { theme.padding * 3.0f, theme.padding * 2.0f });
        ImGui::Begin("##rock-setup", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::PopStyleVar();
        drawHeader(d);
        switch (g.session.page) {
        case Page::Controllers: drawControllers(d); break;
        case Page::Topic:
            if (g.session.topic < g.session.topics.size()) drawTopic(d);
            else g.session.page = Page::Review;
            break;
        case Page::Review: drawReview(d); break;
        case Page::Finished: drawFinished(d); break;
        }
        ImGui::End();
    }
}
