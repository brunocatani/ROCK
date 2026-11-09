#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "physics-interaction/input/ButtonLayoutPolicy.h"

/*
 * Function bindings: the player binds each gameplay function to a control and
 * a gesture (press, tap, hold, double tap), optionally while another control
 * is held.
 *
 * Every function has a home: the hand, logical button and gesture of its
 * default binding, where ROCK's policies, the game's native events, FRIK and
 * ROCK's API consumers already look for it. The composer runs on each
 * controller sample after the controller layout and presents, on the homes,
 * the presses a default-bound player would have made: a moved Reload bound to
 * a double tap of the grip appears as a tap of A. Downstream code therefore
 * keeps reading logical buttons. Each synthesized press is tagged with the
 * function that made it, so the policies that split one button between two
 * functions (A: Use / Reload / Scope, B: VATS / Grenade, Y: Pip-Boy / Pause,
 * stick click: bipod and placement / PALM) act on it at once and never turn it
 * into the partner function.
 *
 * A home button whose functions all keep their home bindings, and on which no
 * moved binding lands, passes through untouched and untagged, so default
 * bindings leave every read byte-for-byte unchanged and the policies keep
 * their own timing. A button that is affected is owned by the recognizer for
 * all of its functions.
 */
namespace rock::function_binding_policy
{
    using button_layout_policy::Slot;
    using button_layout_policy::Source;

    enum class Function : std::uint8_t
    {
        Grab,
        Fire,
        SupportTrigger,
        // The game's Activate on a use target; owns the whole press while the target exists.
        Use,
        Reload,
        Scope,
        Vats,
        Grenade,
        Pipboy,
        Pause,
        // Bipod latch, held placement and the game's Favorites.
        StickActions,
        // RobCo PALM's wheel.
        Palm,
        Count,
    };
    inline constexpr std::size_t kFunctionCount = static_cast<std::size_t>(Function::Count);
    inline constexpr std::size_t kSourceCount = static_cast<std::size_t>(Source::Count);

    [[nodiscard]] constexpr std::uint32_t functionBit(Function function) noexcept
    {
        return std::uint32_t{ 1 } << static_cast<unsigned>(function);
    }

    enum class BindingHand : std::uint8_t
    {
        // The hand on the equipped weapon's firing grip; follows left-hand firing.
        Firing,
        Support,
        // The game's handedness.
        Primary,
        Offhand,
        Left,
        Right,
        // Both hands, each acting for itself.
        Each,
    };

    enum class Gesture : std::uint8_t
    {
        Press,
        Tap,
        Hold,
        DoubleTap,
    };

    struct Control
    {
        BindingHand hand{ BindingHand::Primary };
        Source source{ Source::None };
        constexpr bool operator==(const Control&) const = default;
    };

    struct Binding
    {
        Control control{};
        Gesture gesture{ Gesture::Press };
        // Held when the press begins, for a chord. Source::None for none.
        Control modifier{};

        [[nodiscard]] constexpr bool bound() const noexcept { return control.source != Source::None; }
        [[nodiscard]] constexpr bool chord() const noexcept { return modifier.source != Source::None; }
        constexpr bool operator==(const Binding&) const = default;
    };

    using Bindings = std::array<Binding, kFunctionCount>;

    [[nodiscard]] constexpr Binding homeBinding(Function function) noexcept
    {
        switch (function) {
        case Function::Grab: return { { BindingHand::Each, Source::Grip }, Gesture::Press };
        case Function::Fire: return { { BindingHand::Firing, Source::Trigger }, Gesture::Press };
        case Function::SupportTrigger: return { { BindingHand::Support, Source::Trigger }, Gesture::Press };
        case Function::Use: return { { BindingHand::Primary, Source::LowerFace }, Gesture::Press };
        case Function::Reload: return { { BindingHand::Firing, Source::LowerFace }, Gesture::Tap };
        case Function::Scope: return { { BindingHand::Firing, Source::LowerFace }, Gesture::Hold };
        case Function::Vats: return { { BindingHand::Primary, Source::UpperFace }, Gesture::Tap };
        case Function::Grenade: return { { BindingHand::Primary, Source::UpperFace }, Gesture::Hold };
        case Function::Pipboy: return { { BindingHand::Offhand, Source::UpperFace }, Gesture::Tap };
        case Function::Pause: return { { BindingHand::Offhand, Source::UpperFace }, Gesture::Hold };
        case Function::StickActions: return { { BindingHand::Primary, Source::Stick }, Gesture::Press };
        case Function::Palm: return { { BindingHand::Primary, Source::Stick }, Gesture::Press };
        case Function::Count: break;
        }
        return {};
    }

    [[nodiscard]] constexpr Bindings homeBindings() noexcept
    {
        Bindings bindings{};
        for (std::size_t i = 0; i < kFunctionCount; ++i) bindings[i] = homeBinding(static_cast<Function>(i));
        return bindings;
    }

    // The settings key of each function in [Bindings].
    [[nodiscard]] constexpr std::string_view functionKey(Function function) noexcept
    {
        switch (function) {
        case Function::Grab: return "Grab";
        case Function::Fire: return "Fire";
        case Function::SupportTrigger: return "SupportTrigger";
        case Function::Use: return "Use";
        case Function::Reload: return "Reload";
        case Function::Scope: return "Scope";
        case Function::Vats: return "Vats";
        case Function::Grenade: return "Grenade";
        case Function::Pipboy: return "Pipboy";
        case Function::Pause: return "Pause";
        case Function::StickActions: return "StickActions";
        case Function::Palm: return "Palm";
        case Function::Count: break;
        }
        return "";
    }

    // The home's logical button. Every home is one of the five layout slots.
    [[nodiscard]] constexpr Slot slotOf(Source source) noexcept
    {
        switch (source) {
        case Source::Trigger: return Slot::Trigger;
        case Source::Grip: return Slot::Grab;
        case Source::LowerFace: return Slot::LowerFace;
        case Source::UpperFace: return Slot::UpperFace;
        case Source::Stick: return Slot::StickClick;
        default: return Slot::Count;
        }
    }

    // ---- text ----------------------------------------------------------------------------------

    [[nodiscard]] constexpr std::optional<BindingHand> parseHand(std::string_view text) noexcept
    {
        using button_layout_policy::tokenEquals;
        if (tokenEquals(text, "firing") || tokenEquals(text, "weapon")) return BindingHand::Firing;
        if (tokenEquals(text, "support")) return BindingHand::Support;
        if (tokenEquals(text, "primary")) return BindingHand::Primary;
        if (tokenEquals(text, "offhand")) return BindingHand::Offhand;
        if (tokenEquals(text, "left")) return BindingHand::Left;
        if (tokenEquals(text, "right")) return BindingHand::Right;
        if (tokenEquals(text, "each") || tokenEquals(text, "both")) return BindingHand::Each;
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::optional<Gesture> parseGesture(std::string_view text) noexcept
    {
        using button_layout_policy::tokenEquals;
        if (tokenEquals(text, "press")) return Gesture::Press;
        if (tokenEquals(text, "tap")) return Gesture::Tap;
        if (tokenEquals(text, "hold")) return Gesture::Hold;
        if (tokenEquals(text, "double") || tokenEquals(text, "doubletap")) return Gesture::DoubleTap;
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::string_view handName(BindingHand hand) noexcept
    {
        switch (hand) {
        case BindingHand::Firing: return "firing";
        case BindingHand::Support: return "support";
        case BindingHand::Primary: return "primary";
        case BindingHand::Offhand: return "offhand";
        case BindingHand::Left: return "left";
        case BindingHand::Right: return "right";
        case BindingHand::Each: return "each";
        }
        return "primary";
    }

    [[nodiscard]] constexpr std::string_view gestureName(Gesture gesture) noexcept
    {
        switch (gesture) {
        case Gesture::Press: return "press";
        case Gesture::Tap: return "tap";
        case Gesture::Hold: return "hold";
        case Gesture::DoubleTap: return "double";
        }
        return "press";
    }

    // Face buttons are written with the letters of the hand they usually are on.
    [[nodiscard]] constexpr bool writesLeftLabels(BindingHand hand) noexcept
    {
        return hand == BindingHand::Offhand || hand == BindingHand::Support || hand == BindingHand::Left;
    }

    // "<hand> <gesture> <control> [+<hand> <control>]" or "none". Tokens are
    // separated by spaces; the modifier's hand may be left out for the same hand.
    [[nodiscard]] inline std::optional<Binding> parseBinding(std::string_view text) noexcept
    {
        std::array<std::string_view, 6> tokens{};
        std::size_t count = 0;
        std::size_t i = 0;
        while (i < text.size()) {
            while (i < text.size() && (text[i] == ' ' || text[i] == '\t' || text[i] == ',')) ++i;
            if (i >= text.size()) break;
            std::size_t end = i;
            while (end < text.size() && text[end] != ' ' && text[end] != '\t' && text[end] != ',') ++end;
            if (count == tokens.size()) return std::nullopt;
            tokens[count++] = text.substr(i, end - i);
            i = end;
        }
        if (count == 0) return Binding{};
        if (count == 1) {
            const auto none = button_layout_policy::parseSource(tokens[0]);
            return none && *none == Source::None ? std::optional<Binding>(Binding{}) : std::nullopt;
        }
        if (count < 3) return std::nullopt;
        const auto hand = parseHand(tokens[0]);
        const auto gesture = parseGesture(tokens[1]);
        const auto source = button_layout_policy::parseSource(tokens[2]);
        if (!hand || !gesture || !source || *source == Source::None) return std::nullopt;
        Binding binding{ { *hand, *source }, *gesture };
        if (count > 3) {
            auto first = tokens[3];
            if (first.empty() || first.front() != '+') return std::nullopt;
            first.remove_prefix(1);
            std::string_view modifierHand = first.empty() && count > 4 ? tokens[4] : first;
            std::size_t next = first.empty() ? 5 : 4;
            std::optional<BindingHand> modifierBindingHand = parseHand(modifierHand);
            std::string_view modifierSource;
            if (modifierBindingHand) {
                if (next >= count) return std::nullopt;
                modifierSource = tokens[next++];
            } else {
                modifierBindingHand = *hand;
                modifierSource = modifierHand;
            }
            if (next != count) return std::nullopt;
            const auto parsed = button_layout_policy::parseSource(modifierSource);
            if (!parsed || *parsed == Source::None) return std::nullopt;
            binding.modifier = { *modifierBindingHand, *parsed };
        }
        return binding;
    }

    [[nodiscard]] inline std::string formatBinding(const Binding& binding)
    {
        if (!binding.bound()) return "none";
        std::string text(handName(binding.control.hand));
        text += ' ';
        text += gestureName(binding.gesture);
        text += ' ';
        text += button_layout_policy::sourceName(binding.control.source, writesLeftLabels(binding.control.hand));
        if (binding.chord()) {
            text += " +";
            text += handName(binding.modifier.hand);
            text += ' ';
            text += button_layout_policy::sourceName(binding.modifier.source, writesLeftLabels(binding.modifier.hand));
        }
        return text;
    }

    // ---- recognition ---------------------------------------------------------------------------

    // A tap presented on its home: long enough for every game frame to see it,
    // shorter than any partner hold.
    inline constexpr double kVirtualTapSeconds = 0.08;
    inline constexpr float kDefaultHoldSeconds = 0.35f;
    inline constexpr float kDefaultDoubleTapSeconds = 0.30f;

    [[nodiscard]] constexpr bool exclusiveWhenEligible(Function function) noexcept
    {
        return function == Function::Use;
    }

    struct Context
    {
        bool firingIsLeft{ false };
        bool primaryIsLeft{ false };
        // A blocking menu or no gameplay: every button passes through; held presses wait for release.
        bool bypass{ false };
        // Functions that may act now; an ineligible one is left out of each new press's decision.
        std::uint32_t eligible{ ~std::uint32_t{ 0 } };
        double seconds{ 0.0 };
        std::array<float, kFunctionCount> holdSeconds{};
        float doubleTapSeconds{ kDefaultDoubleTapSeconds };
    };

    // One hand's controller sample after the controller layout.
    struct HandSample
    {
        std::array<bool, kSourceCount> pressed{};
        float triggerAnalog{ 0.0f };
        float gripAnalog{ 0.0f };
    };

    // What the composer presents on one hand's five logical buttons.
    struct HandOutput
    {
        // False: the button passes through as sampled.
        std::array<bool, button_layout_policy::kSlotCount> synthesized{};
        std::array<bool, button_layout_policy::kSlotCount> pressed{};
        // The function behind a synthesized press; Function::Count for none.
        std::array<Function, button_layout_policy::kSlotCount> tag{};
        float triggerAnalog{ 0.0f };
        float gripAnalog{ 0.0f };
    };

    [[nodiscard]] constexpr bool sideMatches(BindingHand hand, bool isLeft, const Context& context) noexcept
    {
        switch (hand) {
        case BindingHand::Firing: return isLeft == context.firingIsLeft;
        case BindingHand::Support: return isLeft != context.firingIsLeft;
        case BindingHand::Primary: return isLeft == context.primaryIsLeft;
        case BindingHand::Offhand: return isLeft != context.primaryIsLeft;
        case BindingHand::Left: return isLeft;
        case BindingHand::Right: return !isLeft;
        case BindingHand::Each: return true;
        }
        return false;
    }

    class Composer
    {
    public:
        void configure(const Bindings& bindings) noexcept
        {
            _bindings = bindings;
            _identity = bindings == homeBindings();
            reset();
        }

        [[nodiscard]] bool identity() const noexcept { return _identity; }
        [[nodiscard]] const Bindings& bindings() const noexcept { return _bindings; }

        void reset() noexcept
        {
            _controls = {};
            _functions = {};
        }

        [[nodiscard]] std::array<HandOutput, 2> update(const Context& context, const std::array<HandSample, 2>& samples) noexcept
        {
            std::array<HandOutput, 2> output{};
            for (auto& hand : output) hand.tag.fill(Function::Count);
            if (_identity) return output;

            const auto affected = affectedSlots(context);
            for (std::size_t side = 0; side < 2; ++side) {
                const bool isLeft = side == 0;
                for (std::size_t source = 0; source < kSourceCount; ++source) {
                    recognize(context, samples, isLeft, static_cast<Source>(source), affected);
                }
            }
            for (std::size_t side = 0; side < 2; ++side) {
                const bool isLeft = side == 0;
                auto& hand = output[side];
                for (std::size_t slot = 0; slot < button_layout_policy::kSlotCount; ++slot) {
                    if (!affected[side][slot]) continue;
                    hand.synthesized[slot] = true;
                    for (std::size_t f = 0; f < kFunctionCount; ++f) {
                        const auto function = static_cast<Function>(f);
                        const auto home = homeBinding(function).control;
                        if (slotOf(home.source) != static_cast<Slot>(slot) || !sideMatches(home.hand, isLeft, context)) continue;
                        if (!presenting(function, isLeft, context.seconds)) continue;
                        hand.pressed[slot] = true;
                        if (hand.tag[slot] == Function::Count) hand.tag[slot] = function;
                        analog(function, isLeft, static_cast<Slot>(slot), context, samples, hand);
                    }
                }
            }
            return output;
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            // Pressed; the gesture is not decided yet.
            Pending,
            // Released after a tap that may become a double tap.
            WaitingSecond,
            // Decided; the press is spent until release.
            Spent,
            // A press that began in a bypass or lost its function waits for release.
            Blocked,
        };

        struct ControlState
        {
            Phase phase{ Phase::Idle };
            bool wasPressed{ false };
            double pressedAt{ 0.0 };
            double releasedAt{ 0.0 };
            // Functions this press may still become.
            std::uint32_t candidates{ 0 };
        };

        struct FunctionState
        {
            // Held from recognition to the control's release (press, hold, double tap).
            std::array<bool, 2> held{};
            // A tap presented until this time.
            std::array<double, 2> tapUntil{ -1.0, -1.0 };
            // The control behind the current press, for analog readback.
            std::array<Source, 2> source{ Source::None, Source::None };
            std::array<bool, 2> sourceLeft{};
        };

        using AffectedSlots = std::array<std::array<bool, button_layout_policy::kSlotCount>, 2>;

        // A slot is affected when a function homed there is bound elsewhere, or a
        // binding of a moved function lands on its button.
        [[nodiscard]] AffectedSlots affectedSlots(const Context& context) const noexcept
        {
            AffectedSlots affected{};
            const auto mark = [&](const Control& control) {
                const auto slot = slotOf(control.source);
                if (slot == Slot::Count) return;
                for (std::size_t side = 0; side < 2; ++side)
                    if (sideMatches(control.hand, side == 0, context)) affected[side][static_cast<std::size_t>(slot)] = true;
            };
            for (std::size_t f = 0; f < kFunctionCount; ++f) {
                const auto function = static_cast<Function>(f);
                const auto& binding = _bindings[f];
                if (binding == homeBinding(function)) continue;
                mark(homeBinding(function).control);
                if (binding.bound()) mark(binding.control);
                if (binding.chord()) mark(binding.modifier);
            }
            return affected;
        }

        // A function is recognized from its binding when its home slot is affected.
        [[nodiscard]] bool recognized(Function function, bool isLeft, const Context& context, const AffectedSlots& affected) const noexcept
        {
            const auto home = homeBinding(function).control;
            for (std::size_t side = 0; side < 2; ++side) {
                if (!sideMatches(home.hand, side == 0, context)) continue;
                const auto slot = slotOf(home.source);
                if (slot != Slot::Count && affected[side][static_cast<std::size_t>(slot)]) {
                    // Each-hand functions are recognized per hand; one-hand homes on any affected side.
                    if (home.hand != BindingHand::Each || (side == 0) == isLeft) return true;
                }
            }
            return false;
        }

        [[nodiscard]] static bool modifierHeld(const Binding& binding, bool isLeft, const Context& context,
            const std::array<HandSample, 2>& samples) noexcept
        {
            if (!binding.chord()) return true;
            for (std::size_t side = 0; side < 2; ++side) {
                const bool sideLeft = side == 0;
                const bool same = binding.modifier.hand == BindingHand::Each ? sideLeft == isLeft :
                    sideMatches(binding.modifier.hand, sideLeft, context);
                if (same && samples[side].pressed[static_cast<std::size_t>(binding.modifier.source)]) return true;
            }
            return false;
        }

        [[nodiscard]] bool presenting(Function function, bool isLeft, double seconds) const noexcept
        {
            const auto& state = _functions[static_cast<std::size_t>(function)];
            const std::size_t side = isLeft ? 0 : 1;
            // An each-hand function presents on its own hand; one-hand functions on whichever hand acted.
            if (homeBinding(function).control.hand == BindingHand::Each)
                return state.held[side] || seconds < state.tapUntil[side];
            return state.held[0] || state.held[1] || seconds < state.tapUntil[0] || seconds < state.tapUntil[1];
        }

        void analog(Function function, bool isLeft, Slot slot, const Context& context, const std::array<HandSample, 2>& samples,
            HandOutput& hand) const noexcept
        {
            const auto& state = _functions[static_cast<std::size_t>(function)];
            std::size_t side = isLeft ? 0 : 1;
            if (homeBinding(function).control.hand != BindingHand::Each && state.source[side] == Source::None)
                side = 1 - side;
            const auto source = state.source[side];
            const auto& sample = samples[state.sourceLeft[side] ? 0 : 1];
            const bool tapping = context.seconds < state.tapUntil[side];
            const float value = tapping ? 1.0f : source == Source::Trigger ? sample.triggerAnalog :
                source == Source::Grip ? sample.gripAnalog : 1.0f;
            if (slot == Slot::Trigger) hand.triggerAnalog = (std::max)(hand.triggerAnalog, value > 0.0f ? value : 1.0f);
            if (slot == Slot::Grab) hand.gripAnalog = (std::max)(hand.gripAnalog, value > 0.0f ? value : 1.0f);
        }

        void activate(Function function, bool isLeft, Source source, bool sourceLeft, bool held, double seconds) noexcept
        {
            auto& state = _functions[static_cast<std::size_t>(function)];
            const std::size_t side = isLeft ? 0 : 1;
            state.source[side] = source;
            state.sourceLeft[side] = sourceLeft;
            if (held) state.held[side] = true;
            else state.tapUntil[side] = seconds + kVirtualTapSeconds;
        }

        void releaseHeld(std::uint32_t functions, bool isLeft) noexcept
        {
            for (std::size_t f = 0; f < kFunctionCount; ++f)
                if (functions & functionBit(static_cast<Function>(f))) _functions[f].held[isLeft ? 0 : 1] = false;
        }

        // Side the function presents on: each-hand functions on the pressing hand,
        // others on side 1 (any) so one-hand functions share one state.
        [[nodiscard]] static bool presentSide(Function function, bool controlLeft) noexcept
        {
            return homeBinding(function).control.hand == BindingHand::Each ? controlLeft : false;
        }

        void recognize(const Context& context, const std::array<HandSample, 2>& samples, bool isLeft, Source source,
            const AffectedSlots& affected) noexcept
        {
            auto& control = _controls[isLeft ? 0 : 1][static_cast<std::size_t>(source)];
            const bool down = samples[isLeft ? 0 : 1].pressed[static_cast<std::size_t>(source)];
            const bool pressEdge = down && !control.wasPressed;
            const bool releaseEdge = !down && control.wasPressed;
            control.wasPressed = down;

            // Functions bound to this control on this hand whose homes are recognized.
            struct Entry
            {
                Function function;
                const Binding* binding;
            };
            std::array<Entry, kFunctionCount> entries{};
            std::size_t count = 0;
            for (std::size_t f = 0; f < kFunctionCount; ++f) {
                const auto function = static_cast<Function>(f);
                const auto& binding = _bindings[f];
                if (!binding.bound() || binding.control.source != source || !sideMatches(binding.control.hand, isLeft, context))
                    continue;
                if (!recognized(function, isLeft, context, affected)) continue;
                entries[count++] = { function, &binding };
            }
            const auto bitsOf = [&](auto predicate) {
                std::uint32_t bits = 0;
                for (std::size_t i = 0; i < count; ++i)
                    if (predicate(entries[i])) bits |= functionBit(entries[i].function);
                return bits;
            };
            const auto releaseEntries = [&] {
                for (std::size_t i = 0; i < count; ++i)
                    releaseHeld(functionBit(entries[i].function), presentSide(entries[i].function, isLeft));
            };

            if (context.bypass || count == 0) {
                releaseEntries();
                control.phase = down ? Phase::Blocked : Phase::Idle;
                control.candidates = 0;
                return;
            }

            if (releaseEdge) releaseEntries();

            const auto gestureBits = [&](Gesture gesture, std::uint32_t within) {
                return bitsOf([&](const Entry& entry) { return entry.binding->gesture == gesture; }) & within;
            };
            const auto fire = [&](std::uint32_t functions, bool held) {
                for (std::size_t i = 0; i < count; ++i)
                    if (functions & functionBit(entries[i].function))
                        activate(entries[i].function, presentSide(entries[i].function, isLeft), source, isLeft, held, context.seconds);
            };
            const auto holdThreshold = [&](std::uint32_t holds) {
                float threshold = 1000.0f;
                for (std::size_t i = 0; i < count; ++i)
                    if (holds & functionBit(entries[i].function)) {
                        const float value = context.holdSeconds[static_cast<std::size_t>(entries[i].function)];
                        threshold = (std::min)(threshold, value > 0.0f ? value : kDefaultHoldSeconds);
                    }
                return threshold;
            };

            switch (control.phase) {
            case Phase::Blocked:
            case Phase::Spent:
                if (!down) control.phase = Phase::Idle;
                return;
            case Phase::WaitingSecond:
                if (pressEdge && context.seconds - control.releasedAt <= context.doubleTapSeconds) {
                    const auto doubles = gestureBits(Gesture::DoubleTap, control.candidates);
                    fire(doubles, true);
                    control.phase = Phase::Spent;
                    control.candidates = 0;
                    return;
                }
                if (context.seconds - control.releasedAt > context.doubleTapSeconds || pressEdge) {
                    fire(gestureBits(Gesture::Tap, control.candidates), false);
                    control.phase = Phase::Idle;
                    control.candidates = 0;
                    if (!pressEdge) return;
                    break;
                }
                return;
            case Phase::Pending: {
                const auto holds = gestureBits(Gesture::Hold, control.candidates);
                if (!down) {
                    const auto taps = gestureBits(Gesture::Tap, control.candidates);
                    if (gestureBits(Gesture::DoubleTap, control.candidates)) {
                        control.phase = Phase::WaitingSecond;
                        control.releasedAt = context.seconds;
                    } else {
                        fire(taps, false);
                        control.phase = Phase::Idle;
                        control.candidates = 0;
                    }
                    return;
                }
                if (holds && context.seconds - control.pressedAt >= holdThreshold(holds)) {
                    const float threshold = holdThreshold(holds);
                    std::uint32_t reached = 0;
                    for (std::size_t i = 0; i < count; ++i) {
                        const auto bit = functionBit(entries[i].function);
                        const float value = context.holdSeconds[static_cast<std::size_t>(entries[i].function)];
                        if ((holds & bit) && (value > 0.0f ? value : kDefaultHoldSeconds) <= threshold) reached |= bit;
                    }
                    fire(reached, true);
                    control.phase = Phase::Spent;
                    control.candidates = 0;
                }
                return;
            }
            case Phase::Idle:
                break;
            }

            if (!pressEdge) return;

            // A new press: its candidates are the eligible functions whose chords hold;
            // a satisfied chord takes the press from the plain bindings.
            std::uint32_t chords = 0, plain = 0;
            for (std::size_t i = 0; i < count; ++i) {
                const auto bit = functionBit(entries[i].function);
                if ((context.eligible & bit) == 0) continue;
                if (entries[i].binding->chord()) {
                    if (modifierHeld(*entries[i].binding, isLeft, context, samples)) chords |= bit;
                } else {
                    plain |= bit;
                }
            }
            std::uint32_t candidates = chords ? chords : plain;
            for (std::size_t i = 0; i < count; ++i) {
                if ((candidates & functionBit(entries[i].function)) && exclusiveWhenEligible(entries[i].function)) {
                    candidates = functionBit(entries[i].function);
                    break;
                }
            }
            control.pressedAt = context.seconds;
            fire(gestureBits(Gesture::Press, candidates), true);
            const auto taps = gestureBits(Gesture::Tap, candidates);
            const auto holds = gestureBits(Gesture::Hold, candidates);
            const auto doubles = gestureBits(Gesture::DoubleTap, candidates);
            if (taps && !holds && !doubles) {
                fire(taps, false);
                control.phase = Phase::Spent;
                control.candidates = 0;
            } else if (holds || doubles) {
                control.phase = Phase::Pending;
                control.candidates = taps | holds | doubles;
            } else {
                control.phase = Phase::Spent;
                control.candidates = 0;
            }
        }

        Bindings _bindings{ homeBindings() };
        bool _identity{ true };
        std::array<std::array<ControlState, kSourceCount>, 2> _controls{};
        std::array<FunctionState, kFunctionCount> _functions{};
    };
}
