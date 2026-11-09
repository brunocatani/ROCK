#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "physics-interaction/input/FunctionBindingPolicy.h"

/*
 * The ROCK setup wizard's decisions, free of the game and of drawing: which
 * bindings conflict, how a pressed control is captured, which hand meanings a
 * capture offers, and how a setting's help text becomes a page of choices.
 */
namespace rock::setup_wizard_policy
{
    namespace fb = function_binding_policy;
    using fb::BindingHand;
    using fb::Function;
    using fb::Gesture;
    using fb::Source;

    // ---- conflicts -----------------------------------------------------------------------------

    // Whether two hand meanings can name the same physical hand in some game state
    // (either firing hand, either primary hand).
    [[nodiscard]] constexpr bool handsCanMeet(BindingHand a, BindingHand b) noexcept
    {
        for (int firing = 0; firing < 2; ++firing) {
            for (int primary = 0; primary < 2; ++primary) {
                fb::Context context{};
                context.firingIsLeft = firing != 0;
                context.primaryIsLeft = primary != 0;
                for (int side = 0; side < 2; ++side) {
                    if (fb::sideMatches(a, side == 0, context) && fb::sideMatches(b, side == 0, context)) return true;
                }
            }
        }
        return false;
    }

    // Pairs ROCK splits on purpose: Use takes the press only while there is
    // something to use, and PALM and the stick actions share the stick by context.
    [[nodiscard]] constexpr bool sharedByDesign(Function a, Function b) noexcept
    {
        const auto pair = [&](Function x, Function y) { return (a == x && b == y) || (a == y && b == x); };
        return pair(Function::Use, Function::Reload) || pair(Function::Use, Function::Scope) ||
            pair(Function::StickActions, Function::Palm);
    }

    // A press acts at once, so it takes the control from every other gesture.
    [[nodiscard]] constexpr bool gesturesClash(Gesture a, Gesture b) noexcept
    {
        return a == b || a == Gesture::Press || b == Gesture::Press;
    }

    [[nodiscard]] constexpr bool clash(const fb::Binding& a, const fb::Binding& b) noexcept
    {
        if (!a.bound() || !b.bound() || a.control.source != b.control.source) return false;
        if (!handsCanMeet(a.control.hand, b.control.hand)) return false;
        // A held chord takes the press from the plain binding: different modifiers never clash.
        if (a.modifier != b.modifier) return false;
        return gesturesClash(a.gesture, b.gesture);
    }

    struct Conflict
    {
        Function a{ Function::Count };
        Function b{ Function::Count };
    };

    struct Conflicts
    {
        std::array<Conflict, fb::kFunctionCount * fb::kFunctionCount / 2> items{};
        std::size_t count{ 0 };
    };

    [[nodiscard]] constexpr Conflicts conflicts(const fb::Bindings& bindings) noexcept
    {
        Conflicts result{};
        for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
            for (std::size_t j = i + 1; j < fb::kFunctionCount; ++j) {
                const auto a = static_cast<Function>(i);
                const auto b = static_cast<Function>(j);
                if (sharedByDesign(a, b) || !clash(bindings[i], bindings[j])) continue;
                result.items[result.count++] = { a, b };
            }
        }
        return result;
    }

    // The functions whose binding clashes with `candidate` for `function`.
    [[nodiscard]] constexpr std::uint32_t clashesWith(const fb::Bindings& bindings, Function function,
        const fb::Binding& candidate) noexcept
    {
        std::uint32_t bits = 0;
        for (std::size_t i = 0; i < fb::kFunctionCount; ++i) {
            const auto other = static_cast<Function>(i);
            if (other == function || sharedByDesign(function, other)) continue;
            if (clash(candidate, bindings[i])) bits |= fb::functionBit(other);
        }
        return bits;
    }

    // ---- capture -------------------------------------------------------------------------------

    // Controls held on each hand (bit per Source), after the controller layout.
    struct CaptureSample
    {
        std::array<std::uint32_t, 2> pressed{};
    };

    // One physical press can light several sources: the Touch grip reports its
    // companion click with it, and a stick or trackpad click lights its region.
    // The capture keeps the control the player pressed: the grip, the stick click.
    [[nodiscard]] constexpr std::uint32_t pressedControls(std::uint32_t bits) noexcept
    {
        const auto bit = [](Source source) { return 1u << static_cast<unsigned>(source); };
        if (bits & bit(Source::Grip)) bits &= ~bit(Source::GripClick);
        if (bits & bit(Source::Stick))
            bits &= ~(bit(Source::PadUp) | bit(Source::PadDown) | bit(Source::PadLeft) | bit(Source::PadRight) |
                bit(Source::PadCenter));
        return bits;
    }

    struct CapturedControl
    {
        bool isLeft{ false };
        Source source{ Source::None };
        constexpr bool operator==(const CapturedControl&) const = default;
    };

    struct Captured
    {
        CapturedControl control{};
        // Held first, for a chord.
        std::optional<CapturedControl> modifier{};
    };

    // Listens for one control: everything must be released first, then the first
    // control pressed is captured when every control is released again. A second
    // control pressed while the first is held makes a chord: the first is the
    // modifier and the second the control.
    class Capture
    {
    public:
        enum class Phase : std::uint8_t
        {
            WaitingForRelease,
            Listening,
            Holding,
            Done,
        };

        void start() noexcept { *this = {}; }
        [[nodiscard]] Phase phase() const noexcept { return _phase; }
        [[nodiscard]] const std::optional<Captured>& result() const noexcept { return _result; }
        // What is held now, for the live preview.
        [[nodiscard]] const std::optional<Captured>& pending() const noexcept { return _pending; }

        void update(CaptureSample sample) noexcept
        {
            for (auto& bits : sample.pressed) bits = pressedControls(bits);
            const bool any = sample.pressed[0] != 0 || sample.pressed[1] != 0;
            switch (_phase) {
            case Phase::WaitingForRelease:
                if (!any) _phase = Phase::Listening;
                return;
            case Phase::Listening:
                if (const auto first = firstPressed(sample, std::nullopt)) {
                    _pending = Captured{ *first };
                    _phase = Phase::Holding;
                }
                return;
            case Phase::Holding:
                if (!any) {
                    _result = _pending;
                    _phase = Phase::Done;
                    return;
                }
                if (!_pending->modifier && held(sample, _pending->control)) {
                    if (const auto second = firstPressed(sample, _pending->control))
                        _pending = Captured{ *second, _pending->control };
                }
                return;
            case Phase::Done:
                return;
            }
        }

    private:
        [[nodiscard]] static bool held(const CaptureSample& sample, const CapturedControl& control) noexcept
        {
            return (sample.pressed[control.isLeft ? 0 : 1] & (1u << static_cast<unsigned>(control.source))) != 0;
        }

        // The control with the lowest source on the right hand first, then the left;
        // skips `except`. The analog stick's touch and the trackpad's centre are
        // only taken when nothing more specific is held.
        [[nodiscard]] static std::optional<CapturedControl> firstPressed(const CaptureSample& sample,
            std::optional<CapturedControl> except) noexcept
        {
            for (const bool left : { false, true }) {
                const auto bits = sample.pressed[left ? 0 : 1];
                for (std::size_t index = 0; index < fb::kSourceCount; ++index) {
                    const auto source = static_cast<Source>(index);
                    if (source == Source::None || (bits & (1u << index)) == 0) continue;
                    const CapturedControl control{ left, source };
                    if (except && *except == control) continue;
                    return control;
                }
            }
            return std::nullopt;
        }

        Phase _phase{ Phase::WaitingForRelease };
        std::optional<Captured> _pending{};
        std::optional<Captured> _result{};
    };

    // ---- hand meaning --------------------------------------------------------------------------

    struct HandChoices
    {
        std::array<BindingHand, 4> hands{};
        std::size_t count{ 0 };
        // Index of the suggested meaning.
        std::size_t suggested{ 0 };
    };

    // The meanings a control captured on one hand may keep: the function's own
    // role (firing, support, primary, offhand, each) when it names that hand now,
    // then that physical hand alone.
    [[nodiscard]] constexpr HandChoices handChoices(Function function, bool capturedLeft, const fb::Context& now) noexcept
    {
        HandChoices result{};
        const auto home = fb::homeBinding(function).control.hand;
        const auto add = [&](BindingHand hand) {
            for (std::size_t i = 0; i < result.count; ++i)
                if (result.hands[i] == hand) return;
            result.hands[result.count++] = hand;
        };
        if (fb::sideMatches(home, capturedLeft, now)) add(home);
        // A role the player may prefer to the function's own.
        for (const auto role : { BindingHand::Firing, BindingHand::Support, BindingHand::Primary, BindingHand::Offhand }) {
            if (role != home && fb::sideMatches(role, capturedLeft, now) && result.count < 2) add(role);
        }
        add(capturedLeft ? BindingHand::Left : BindingHand::Right);
        result.suggested = 0;
        return result;
    }

    // The gestures a function may take. Press is for functions that act while held.
    [[nodiscard]] constexpr bool gestureAllowed(Function function, Gesture gesture) noexcept
    {
        switch (function) {
        case Function::Grab:
        case Function::Fire:
        case Function::SupportTrigger:
        case Function::Use:
            // Held actions: they need the press for as long as it lasts.
            return gesture == Gesture::Press || gesture == Gesture::Hold;
        case Function::StickActions:
            return gesture != Gesture::Tap;
        default:
            return true;
        }
    }

    // ---- option pages --------------------------------------------------------------------------

    // One page of the wizard per major option, in page order.
    struct OptionPage
    {
        std::string_view section;
        std::string_view key;
    };

    inline constexpr std::array kOptionPages{
        OptionPage{ "FiringHand", "DefaultHand" },
        OptionPage{ "WeaponHandling", "WeaponGrabMode" },
        OptionPage{ "WeaponHandling", "WeaponEquipMode" },
        OptionPage{ "WeaponHandling", "DetachEitherHand" },
        OptionPage{ "WeaponHandling", "GunstockMode" },
        OptionPage{ "WeaponHandling", "BipodMode" },
        OptionPage{ "Scopes", "EnableImmersiveScopes" },
        OptionPage{ "Grenades", "ImmersiveGrenades" },
        OptionPage{ "WeaponRecoil", "ImmersiveRecoil" },
        OptionPage{ "Grabbing", "FarGrabMode" },
        OptionPage{ "ImmersiveAid", "Enabled" },
        OptionPage{ "WeaponHandling", "AmmoCounterStyle" },
        OptionPage{ "SelectionFeedback", "HighlightEnabled" },
    };

    struct Choice
    {
        std::string_view value;
        std::string_view label;
        // What the choice does, from the help text; may be empty.
        std::string_view detail;
    };

    struct ChoiceList
    {
        std::array<Choice, 8> items{};
        std::size_t count{ 0 };
    };

    struct HelpText
    {
        std::string_view title;
        std::string_view body;
    };

    // The catalog's help: the title line, then the body.
    [[nodiscard]] constexpr HelpText splitHelp(std::string_view help) noexcept
    {
        const auto newline = help.find('\n');
        if (newline == std::string_view::npos) return { help, {} };
        return { help.substr(0, newline), help.substr(newline + 1) };
    }

    // The line of `body` that begins with `prefix`, without the prefix.
    [[nodiscard]] constexpr std::string_view lineStartingWith(std::string_view body, std::string_view prefix) noexcept
    {
        std::size_t start = 0;
        while (start <= body.size()) {
            auto end = body.find('\n', start);
            if (end == std::string_view::npos) end = body.size();
            const auto line = body.substr(start, end - start);
            if (line.starts_with(prefix)) return line.substr(prefix.size());
            start = end + 1;
        }
        return {};
    }

    // The body without the line that begins with `prefix`, up to that line.
    [[nodiscard]] constexpr std::string_view beforeLine(std::string_view body, std::string_view prefix) noexcept
    {
        std::size_t start = 0;
        while (start < body.size()) {
            auto end = body.find('\n', start);
            if (end == std::string_view::npos) end = body.size();
            if (body.substr(start, end - start).starts_with(prefix)) return body.substr(0, start ? start - 1 : 0);
            start = end + 1;
        }
        return body;
    }

    [[nodiscard]] constexpr std::string_view trimmed(std::string_view text) noexcept
    {
        while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
        while (!text.empty() && (text.back() == ' ' || text.back() == '.' || text.back() == '\t')) text.remove_suffix(1);
        return text;
    }

    // "N = label" pairs on the body's first line (the catalog's convention for
    // integer modes: labels hold no , ; or full stop).
    [[nodiscard]] constexpr ChoiceList numberedChoices(std::string_view body) noexcept
    {
        ChoiceList result{};
        auto end = body.find('\n');
        // The pairs may run over two lines ("1 = ... 2 = ...\n3 = ...").
        if (end != std::string_view::npos && end + 1 < body.size() && body[end + 1] >= '1' && body[end + 1] <= '9' &&
            end + 3 < body.size() && body[end + 2] == ' ' && body[end + 3] == '=') {
            const auto second = body.find('\n', end + 1);
            end = second;
        }
        const auto line = body.substr(0, end == std::string_view::npos ? body.size() : end);
        std::size_t i = 0;
        while (i < line.size() && result.count < result.items.size()) {
            while (i < line.size() && (line[i] < '0' || line[i] > '9')) ++i;
            const std::size_t valueStart = i;
            while (i < line.size() && line[i] >= '0' && line[i] <= '9') ++i;
            if (valueStart == i) break;
            const auto value = line.substr(valueStart, i - valueStart);
            if (i + 2 >= line.size() || line.substr(i, 3) != " = ") break;
            i += 3;
            const std::size_t labelStart = i;
            while (i < line.size() && line[i] != '.' && line[i] != '\n') ++i;
            result.items[result.count++] = { value, trimmed(line.substr(labelStart, i - labelStart)), {} };
            if (i < line.size()) ++i;
        }
        return result;
    }

    enum class ValueKind : std::uint8_t
    {
        Boolean,
        Integer,
        Text,
    };

    // The page's choices: On/Off for a switch (Off described by the help's
    // "False:" line), the numbered modes of an integer, right/left for the firing hand.
    [[nodiscard]] constexpr ChoiceList choicesFor(const OptionPage& page, ValueKind kind, std::string_view help) noexcept
    {
        const auto text = splitHelp(help);
        ChoiceList result{};
        if (kind == ValueKind::Boolean) {
            result.items[0] = { "true", "On", trimmed(beforeLine(text.body, "False:")) };
            result.items[1] = { "false", "Off", trimmed(lineStartingWith(text.body, "False:")) };
            result.count = 2;
            return result;
        }
        if (page.section == "FiringHand" && page.key == "DefaultHand") {
            result.items[0] = { "right", "Right hand", {} };
            result.items[1] = { "left", "Left hand", {} };
            result.count = 2;
            return result;
        }
        if (kind == ValueKind::Integer) return numberedChoices(text.body);
        return result;
    }
}
