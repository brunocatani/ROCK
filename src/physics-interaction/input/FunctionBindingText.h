#pragma once

#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "physics-interaction/input/FunctionBindingPolicy.h"

/*
 * [Bindings] text. Each function's binding is a CommonFramework binding table,
 * { hand = "primary", type = "hold", button = "a", duration = 0.3, modifier = { hand = "offhand", button = "grip" } },
 * which Mods Config's binding editor reads as is. The Configuration API, which has
 * no table type, carries the framework's one-line form, "primary hold a 0.3 +offhand:grip".
 *
 * ROCK reads the framework's words with these limits: types hold (no duration: while
 * held; a duration: from that time until release), tap and double; the five logical
 * buttons (trigger, grip, a, b, thumbstick), the physical extras reaching them through
 * the button layout; and, for Fire, Reload, Scope and SupportTrigger, only primary (the
 * hand on the weapon's firing grip) and offhand. Elsewhere primary is the right hand.
 */
namespace rock::function_binding_text
{
    namespace fb = function_binding_policy;

    // A binding and its time: a hold's hold time or a double tap's longest gap, 0 for a press or a tap.
    struct ConfiguredBinding
    {
        fb::Binding binding{};
        float seconds{ 0.0f };
        bool operator==(const ConfiguredBinding&) const = default;
    };

    // A read binding, with a warning for each duration brought into its range.
    struct ParsedBinding
    {
        ConfiguredBinding value;
        std::vector<std::string> warnings;
    };

    // The retired hold times an old file carried, each already in its old range.
    struct LegacyTimes
    {
        float hold{ fb::kDefaultHoldSeconds };
        float scopeHold{ fb::defaultHoldSeconds(fb::Function::Scope) };
        float pauseHold{ fb::defaultHoldSeconds(fb::Function::Pause) };
        float doubleTap{ 0.30f };
    };

    [[nodiscard]] ConfiguredBinding defaultBinding(fb::Function function);
    [[nodiscard]] float holdSeconds(fb::Function function, const ConfiguredBinding& binding);

    [[nodiscard]] std::expected<ParsedBinding, std::string> parseBindingTable(fb::Function function, std::string_view text);
    [[nodiscard]] std::string formatBindingTable(fb::Function function, const ConfiguredBinding& binding);
    [[nodiscard]] std::expected<ParsedBinding, std::string> parseBindingLine(fb::Function function, std::string_view text);
    [[nodiscard]] std::string formatBindingLine(fb::Function function, const ConfiguredBinding& binding);

    // text is the old string, or nullopt for a file without one (the default binding with the old times).
    [[nodiscard]] ParsedBinding fromLegacyBinding(fb::Function function, std::optional<std::string_view> text,
        const LegacyTimes& times);
}
