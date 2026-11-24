#include "physics-interaction/input/FunctionBindingText.h"

#include <RPS/Settings/Store.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <format>
#include <optional>

namespace rock::function_binding_text
{
    namespace
    {
        using button_layout_policy::Source;
        using button_layout_policy::tokenEquals;
        using fb::BindingHand;
        using fb::Gesture;

        constexpr float kMinimumHoldSeconds = 0.05f;
        constexpr float kMaximumHoldSeconds = 2.0f;
        constexpr float kMinimumDoubleSeconds = 0.15f;
        constexpr float kMaximumDoubleSeconds = 0.6f;

        enum class Type
        {
            Hold,
            Tap,
            Double,
            None,
        };

        // The words of one binding as either text form gives them, before they are checked.
        struct Words
        {
            std::optional<std::string> hand;
            std::optional<std::string> type;
            std::optional<std::string> button;
            std::optional<double> duration;
            bool chord{ false };
            std::optional<std::string> modifierHand;
            std::optional<std::string> modifierButton;
        };

        std::string secondsText(float seconds)
        {
            std::array<char, 32> buffer{};
            const auto [end, error] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), seconds);
            std::string text = error == std::errc{} ? std::string(buffer.data(), end) : std::string("0");
            if (text.find_first_of(".eE") == std::string::npos)
                text += ".0";
            return text;
        }

        /**
         * A hand word. For the weapon functions primary is the firing hand and offhand the support hand, and a
         * fixed or either hand is refused for the binding itself (a modifier may name any hand).
         */
        std::expected<BindingHand, std::string> parseHand(fb::Function function, std::string_view word, bool modifier)
        {
            const bool weapon = fb::followsWeapon(function);
            if (tokenEquals(word, "primary"))
                return weapon ? BindingHand::Firing : BindingHand::Primary;
            if (tokenEquals(word, "offhand"))
                return weapon ? BindingHand::Support : BindingHand::Offhand;
            if (tokenEquals(word, "firing") || tokenEquals(word, "weapon"))
                return BindingHand::Firing;
            if (tokenEquals(word, "support"))
                return BindingHand::Support;
            const bool fixed = tokenEquals(word, "left") || tokenEquals(word, "right");
            const bool either = tokenEquals(word, "any") || tokenEquals(word, "each") || tokenEquals(word, "both");
            if ((fixed || either) && weapon && !modifier)
                return std::unexpected(std::format("'{}' is not a hand for this function, which takes primary (the hand "
                                                   "holding the weapon's trigger) or offhand",
                    word));
            if (tokenEquals(word, "left"))
                return BindingHand::Left;
            if (tokenEquals(word, "right"))
                return BindingHand::Right;
            if (either)
                return BindingHand::Each;
            return std::unexpected(std::format("'{}' is not a hand, one of primary, offhand, left, right, any", word));
        }

        std::expected<Type, std::string> parseType(std::string_view word)
        {
            if (tokenEquals(word, "hold") || tokenEquals(word, "holddown"))
                return Type::Hold;
            if (tokenEquals(word, "tap"))
                return Type::Tap;
            if (tokenEquals(word, "double") || tokenEquals(word, "doubletap") || tokenEquals(word, "doublepress"))
                return Type::Double;
            if (tokenEquals(word, "none"))
                return Type::None;
            return std::unexpected(std::format("'{}' is not a type ROCK reads: hold, tap, double or none", word));
        }

        std::expected<Source, std::string> parseButton(std::string_view word)
        {
            if (tokenEquals(word, "trigger"))
                return Source::Trigger;
            if (tokenEquals(word, "grip"))
                return Source::Grip;
            if (tokenEquals(word, "a"))
                return Source::LowerFace;
            if (tokenEquals(word, "b") || tokenEquals(word, "menu"))
                return Source::UpperFace;
            if (tokenEquals(word, "thumbstick"))
                return Source::Stick;
            return std::unexpected(std::format("'{}' is not a button ROCK reads: trigger, grip, a, b, thumbstick (other "
                                               "controls reach these through the button layout)",
                word));
        }

        std::string_view handWord(fb::Function function, BindingHand hand)
        {
            const bool weapon = fb::followsWeapon(function);
            switch (hand) {
            case BindingHand::Firing: return weapon ? "primary" : "firing";
            case BindingHand::Support: return weapon ? "offhand" : "support";
            case BindingHand::Primary: return "primary";
            case BindingHand::Offhand: return "offhand";
            case BindingHand::Left: return "left";
            case BindingHand::Right: return "right";
            case BindingHand::Each: return "any";
            }
            return "primary";
        }

        std::string_view typeWord(Gesture gesture)
        {
            switch (gesture) {
            case Gesture::Tap: return "tap";
            case Gesture::DoubleTap: return "double";
            case Gesture::Press:
            case Gesture::Hold: return "hold";
            }
            return "hold";
        }

        std::string_view buttonWord(Source source)
        {
            switch (source) {
            case Source::Trigger: return "trigger";
            case Source::Grip: return "grip";
            case Source::LowerFace: return "a";
            case Source::UpperFace: return "b";
            case Source::Stick: return "thumbstick";
            default: return "none";
            }
        }

        /**
         * The binding the words describe. A hold with no or a zero duration acts while held (a press); a timed hold,
         * and a double tap, keep a duration in its range, clamped with a warning.
         */
        std::expected<ParsedBinding, std::string> build(fb::Function function, const Words& words)
        {
            if (!words.type)
                return std::unexpected("it has no type");
            const auto type = parseType(*words.type);
            if (!type)
                return std::unexpected(type.error());
            ParsedBinding result;
            if (*type == Type::None)
                return result;
            if (!words.hand)
                return std::unexpected("it has no hand");
            if (!words.button)
                return std::unexpected("it has no button");
            const auto hand = parseHand(function, *words.hand, false);
            if (!hand)
                return std::unexpected(hand.error());
            const auto button = parseButton(*words.button);
            if (!button)
                return std::unexpected(button.error());
            auto& binding = result.value.binding;
            binding.control = { *hand, *button };

            const auto clamped = [&](double duration, float minimum, float maximum) {
                const auto value = static_cast<float>(duration);
                const float inRange = std::clamp(value, minimum, maximum);
                if (std::abs(inRange - value) > 1.0e-6f)
                    result.warnings.push_back(std::format("duration {} is outside {} to {} seconds; using {}",
                        secondsText(value), secondsText(minimum), secondsText(maximum), secondsText(inRange)));
                return inRange;
            };
            if (words.duration && !std::isfinite(*words.duration))
                return std::unexpected("its duration is not a number");
            switch (*type) {
            case Type::Hold:
                if (words.duration && *words.duration < 0.0)
                    return std::unexpected("its duration is below 0");
                if (words.duration && *words.duration > 0.0) {
                    binding.gesture = Gesture::Hold;
                    result.value.seconds = clamped(*words.duration, kMinimumHoldSeconds, kMaximumHoldSeconds);
                } else {
                    binding.gesture = Gesture::Press;
                }
                break;
            case Type::Tap:
                if (words.duration)
                    return std::unexpected("a tap takes no duration");
                binding.gesture = Gesture::Tap;
                break;
            case Type::Double:
                binding.gesture = Gesture::DoubleTap;
                result.value.seconds = words.duration ?
                    clamped(*words.duration, kMinimumDoubleSeconds, kMaximumDoubleSeconds) :
                    fb::kDefaultDoubleTapSeconds;
                break;
            case Type::None:
                break;
            }

            if (words.chord) {
                if (!words.modifierButton)
                    return std::unexpected("its modifier has no button");
                const auto modifierButton = parseButton(*words.modifierButton);
                if (!modifierButton)
                    return std::unexpected("modifier: " + modifierButton.error());
                BindingHand modifierHand = *hand;
                if (words.modifierHand) {
                    const auto parsed = parseHand(function, *words.modifierHand, true);
                    if (!parsed)
                        return std::unexpected("modifier: " + parsed.error());
                    modifierHand = *parsed;
                }
                binding.modifier = { modifierHand, *modifierButton };
            }
            return result;
        }

        std::optional<std::string> textOf(const RPS::Settings::TableValue& value)
        {
            if (const auto* text = std::get_if<std::string>(&value.value))
                return *text;
            return std::nullopt;
        }

        std::optional<double> numberOf(const RPS::Settings::TableValue& value)
        {
            if (const auto* number = std::get_if<double>(&value.value))
                return *number;
            if (const auto* integer = std::get_if<std::int64_t>(&value.value))
                return static_cast<double>(*integer);
            return std::nullopt;
        }
    }

    /**
     * Each function's default: its home binding, with its hold time on a timed hold.
     */
    ConfiguredBinding defaultBinding(fb::Function function)
    {
        return { fb::homeBinding(function), fb::homeSeconds()[static_cast<std::size_t>(function)] };
    }

    /**
     * The function's hold time: its binding's when the binding is a timed hold, otherwise the function's default.
     * The home gestures that split a button between two functions (Scope, Pause, Grenade) time it with this.
     */
    float holdSeconds(fb::Function function, const ConfiguredBinding& binding)
    {
        return binding.binding.gesture == Gesture::Hold && binding.seconds > 0.0f ? binding.seconds :
                                                                                    fb::defaultHoldSeconds(function);
    }

    /**
     * A binding from its CommonFramework table. Keys other than hand, type, button, duration and modifier (the
     * framework's suppress, axis, direction, threshold, accelerate) make it invalid, as do words ROCK does not read.
     */
    std::expected<ParsedBinding, std::string> parseBindingTable(fb::Function function, std::string_view text)
    {
        const auto entries = RPS::Settings::parseTable(text);
        if (!entries)
            return std::unexpected("it is not an inline table");
        Words words;
        const auto keyError = [](std::string_view key) {
            return std::format("'{}' is not a key ROCK reads: hand, type, button, duration, modifier", key);
        };
        for (const auto& [key, value] : *entries) {
            if (key == "hand" || key == "type" || key == "button") {
                auto word = textOf(value);
                if (!word)
                    return std::unexpected(std::format("{} must be a word in quotes", key));
                (key == "hand" ? words.hand : key == "type" ? words.type : words.button) = std::move(word);
            } else if (key == "duration") {
                words.duration = numberOf(value);
                if (!words.duration)
                    return std::unexpected("duration must be a number of seconds");
            } else if (key == "modifier") {
                const auto* modifier = std::get_if<std::vector<RPS::Settings::TableEntry>>(&value.value);
                if (!modifier)
                    return std::unexpected("modifier must be a table such as { hand = \"offhand\", button = \"grip\" }");
                words.chord = true;
                for (const auto& [modifierKey, modifierValue] : *modifier) {
                    if (modifierKey != "hand" && modifierKey != "button")
                        return std::unexpected(
                            std::format("'{}' is not a modifier key, which are hand and button", modifierKey));
                    auto word = textOf(modifierValue);
                    if (!word)
                        return std::unexpected(std::format("modifier {} must be a word in quotes", modifierKey));
                    (modifierKey == "hand" ? words.modifierHand : words.modifierButton) = std::move(word);
                }
            } else {
                return std::unexpected(keyError(key));
            }
        }
        return build(function, words);
    }

    /**
     * The table ROCK writes: hand, type, button, then a timed hold's or a double tap's duration and the modifier,
     * whose hand is left out when it is the binding's own. An unbound function is { type = "none" }.
     */
    std::string formatBindingTable(fb::Function function, const ConfiguredBinding& binding)
    {
        const auto& value = binding.binding;
        if (!value.bound())
            return "{ type = \"none\" }";
        auto text = std::format("{{ hand = \"{}\", type = \"{}\", button = \"{}\"", handWord(function, value.control.hand),
            typeWord(value.gesture), buttonWord(value.control.source));
        if (value.gesture == Gesture::Hold || value.gesture == Gesture::DoubleTap)
            text += std::format(", duration = {}", secondsText(binding.seconds));
        if (value.chord()) {
            text += ", modifier = { ";
            if (value.modifier.hand != value.control.hand)
                text += std::format("hand = \"{}\", ", handWord(function, value.modifier.hand));
            text += std::format("button = \"{}\" }}", buttonWord(value.modifier.source));
        }
        return text + " }";
    }

    /**
     * A binding from ROCK's old string form, with the retired hold times as its durations: an old press acts while
     * held, an old hold takes the function's old hold time (Grenade its default; it followed the game's). What the
     * tables cannot say becomes the function's default with a warning: an unreadable string, a physical control no
     * logical button is, or a fixed or either hand on a weapon function.
     */
    ParsedBinding fromLegacyBinding(fb::Function function, std::optional<std::string_view> oldText, const LegacyTimes& times)
    {
        const auto withTimes = [&](fb::Binding binding) {
            ConfiguredBinding result{ binding, 0.0f };
            if (binding.gesture == Gesture::DoubleTap) {
                result.seconds = times.doubleTap;
            } else if (binding.gesture == Gesture::Hold) {
                switch (function) {
                case fb::Function::Scope: result.seconds = times.scopeHold; break;
                case fb::Function::Pause: result.seconds = times.pauseHold; break;
                case fb::Function::Grenade: result.seconds = fb::defaultHoldSeconds(function); break;
                default: result.seconds = times.hold; break;
                }
            }
            return result;
        };
        ParsedBinding parsed;
        if (!oldText) {
            parsed.value = withTimes(fb::homeBinding(function));
            return parsed;
        }
        const auto text = *oldText;
        const auto old = fb::parseLegacyBinding(text);
        const auto fallBack = [&](std::string reason) {
            parsed.value = withTimes(fb::homeBinding(function));
            parsed.warnings.push_back(std::move(reason));
            return parsed;
        };
        if (!old)
            return fallBack(std::format("\"{}\" could not be read; using the default", text));
        if (old->bound() && (!fb::isLogicalButton(old->control.source) ||
                                (old->chord() && !fb::isLogicalButton(old->modifier.source))))
            return fallBack(std::format("\"{}\" names a control bindings no longer take; using the default. Map that "
                                        "control to a button in the button layout ([ControllerInput.Left] or "
                                        "[ControllerInput.Right]) and bind that button",
                text));
        const auto hand = old->control.hand;
        if (old->bound() && fb::followsWeapon(function) &&
            (hand == BindingHand::Left || hand == BindingHand::Right || hand == BindingHand::Each))
            return fallBack(std::format("\"{}\" names a fixed hand; this function follows the weapon, so it takes "
                                        "primary or offhand. Using the default",
                text));
        parsed.value = withTimes(*old);
        return parsed;
    }
}
