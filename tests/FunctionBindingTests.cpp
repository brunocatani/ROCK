#include "physics-interaction/input/FunctionBindingPolicy.h"

#include <cstdio>

namespace
{
    namespace fb = rock::function_binding_policy;
    using fb::Function;
    using fb::Slot;
    using fb::Source;

    int g_failures = 0;

    void expect(const char* label, bool value)
    {
        if (!value) {
            std::printf("FAILED: %s\n", label);
            ++g_failures;
        }
    }

    constexpr std::size_t kLeft = 0, kRight = 1;

    struct Rig
    {
        fb::Composer composer;
        fb::Context context;
        std::array<fb::HandSample, 2> samples{};

        explicit Rig(const fb::Bindings& bindings)
        {
            composer.configure(bindings);
            context.holdSeconds[static_cast<std::size_t>(Function::Scope)] = 0.30f;
            context.holdSeconds[static_cast<std::size_t>(Function::Pause)] = 0.35f;
            context.holdSeconds[static_cast<std::size_t>(Function::Grenade)] = 0.25f;
        }

        void set(std::size_t side, Source source, bool down) { samples[side].pressed[static_cast<std::size_t>(source)] = down; }

        std::array<fb::HandOutput, 2> step(double seconds)
        {
            context.seconds = seconds;
            return composer.update(context, samples);
        }
    };

    [[nodiscard]] bool pressedWithTag(const fb::HandOutput& hand, Slot slot, Function tag)
    {
        const auto i = static_cast<std::size_t>(slot);
        return hand.synthesized[i] && hand.pressed[i] && hand.tag[i] == tag;
    }

    [[nodiscard]] bool released(const fb::HandOutput& hand, Slot slot)
    {
        const auto i = static_cast<std::size_t>(slot);
        return hand.synthesized[i] && !hand.pressed[i];
    }

    // Default bindings never touch a read: every downstream reader keeps today's input.
    void defaultBindingsPassThrough()
    {
        Rig rig(fb::homeBindings());
        expect("home bindings are the identity", rig.composer.identity());
        rig.set(kRight, Source::LowerFace, true);
        rig.set(kLeft, Source::UpperFace, true);
        for (const auto& hand : rig.step(0.0))
            for (const bool synthesized : hand.synthesized) expect("default binding synthesized a button", !synthesized);
    }

    // The [Bindings] values are a configuration contract.
    void bindingTextRoundTrips()
    {
        for (std::size_t f = 0; f < fb::kFunctionCount; ++f) {
            const auto home = fb::homeBinding(static_cast<Function>(f));
            const auto parsed = fb::parseBinding(fb::formatBinding(home));
            expect("a home binding does not survive its own text", parsed && *parsed == home);
        }
        const auto chord = fb::parseBinding("offhand double grip +offhand trigger");
        expect("chord parsed", chord && chord->gesture == fb::Gesture::DoubleTap && chord->control.source == Source::Grip &&
            chord->modifier.source == Source::Trigger && chord->modifier.hand == fb::BindingHand::Offhand);
        const auto sameHand = fb::parseBinding("right tap b +grip");
        expect("same-hand chord parsed", sameHand && sameHand->modifier.hand == fb::BindingHand::Right);
        expect("none parsed", fb::parseBinding("none") && !fb::parseBinding("none")->bound());
        expect("garbage refused", !fb::parseBinding("firing sometimes a") && !fb::parseBinding("left tap"));
    }

    // Reload moved to a right grip double tap: A keeps Scope and Use, and a plain
    // A tap no longer reloads.
    void movedReloadPresentsTaggedTapOnA()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Reload)] = *fb::parseBinding("right double grip");
        Rig rig(bindings);
        rig.context.eligible &= ~fb::functionBit(Function::Use);

        rig.set(kRight, Source::Grip, true);
        auto out = rig.step(0.00);
        expect("grip still grabs on press", pressedWithTag(out[kRight], Slot::Grab, Function::Grab));
        rig.set(kRight, Source::Grip, false);
        out = rig.step(0.10);
        expect("grab released", released(out[kRight], Slot::Grab));
        rig.set(kRight, Source::Grip, true);
        out = rig.step(0.20);
        expect("double tap presents a reload on A", pressedWithTag(out[kRight], Slot::LowerFace, Function::Reload));
        rig.set(kRight, Source::Grip, false);
        out = rig.step(0.25);
        out = rig.step(0.40);
        expect("double-tap reload ends with the press", released(out[kRight], Slot::LowerFace));

        rig.set(kRight, Source::LowerFace, true);
        out = rig.step(1.00);
        expect("A waits for the scope hold", released(out[kRight], Slot::LowerFace));
        rig.set(kRight, Source::LowerFace, false);
        out = rig.step(1.10);
        out = rig.step(1.12);
        expect("an A tap no longer reloads", released(out[kRight], Slot::LowerFace));

        rig.set(kRight, Source::LowerFace, true);
        out = rig.step(2.00);
        out = rig.step(2.31);
        expect("an A hold is a tagged scope", pressedWithTag(out[kRight], Slot::LowerFace, Function::Scope));
        rig.set(kRight, Source::LowerFace, false);
        out = rig.step(2.50);
        expect("scope ends on release", released(out[kRight], Slot::LowerFace));
    }

    // Use owns the whole press while its target exists.
    void useTakesTheWholePress()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Reload)] = *fb::parseBinding("right double grip");
        Rig rig(bindings);
        rig.set(kRight, Source::LowerFace, true);
        auto out = rig.step(0.0);
        expect("use presses A at once", pressedWithTag(out[kRight], Slot::LowerFace, Function::Use));
        out = rig.step(0.5);
        expect("use is never a scope", pressedWithTag(out[kRight], Slot::LowerFace, Function::Use));
    }

    // A tap with no partner acts on the press; Pip-Boy moved to the left stick click.
    void soleTapActsOnPress()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Pipboy)] = *fb::parseBinding("left tap thumbstick");
        Rig rig(bindings);
        rig.set(kLeft, Source::Stick, true);
        auto out = rig.step(0.0);
        expect("pip-boy tap presented on Y at the press", pressedWithTag(out[kLeft], Slot::UpperFace, Function::Pipboy));
        out = rig.step(0.2);
        expect("the presented tap is short", released(out[kLeft], Slot::UpperFace));
        rig.set(kLeft, Source::UpperFace, true);
        rig.set(kLeft, Source::Stick, false);
        out = rig.step(1.0);
        out = rig.step(1.4);
        expect("Y still holds for Pause", pressedWithTag(out[kLeft], Slot::UpperFace, Function::Pause));
    }

    // A press that began in a menu never acts after the menu closes.
    void bypassBlocksUntilRelease()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Vats)] = *fb::parseBinding("primary tap grip");
        Rig rig(bindings);
        rig.context.bypass = true;
        rig.set(kRight, Source::Grip, true);
        (void)rig.step(0.0);
        rig.context.bypass = false;
        auto out = rig.step(0.1);
        expect("a press held from a menu does not act", !out[kRight].pressed[static_cast<std::size_t>(Slot::UpperFace)]);
        rig.set(kRight, Source::Grip, false);
        (void)rig.step(0.2);
        rig.set(kRight, Source::Grip, true);
        out = rig.step(0.3);
        expect("a fresh press acts", pressedWithTag(out[kRight], Slot::UpperFace, Function::Vats));
    }

    // PALM's wheel and the stick actions share the stick click: moving the stick
    // actions away leaves PALM on it, and both stay readable when both act.
    void sharedStickKeepsEveryFunction()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::StickActions)] = *fb::parseBinding("primary double thumbstick");
        Rig rig(bindings);
        rig.set(kRight, Source::Stick, true);
        auto out = rig.step(0.0);
        expect("the stick press is PALM's", (out[kRight].functions & fb::functionBit(Function::Palm)) != 0 &&
            (out[kRight].functions & fb::functionBit(Function::StickActions)) == 0);
        rig.set(kRight, Source::Stick, false);
        (void)rig.step(0.1);
        rig.set(kRight, Source::Stick, true);
        out = rig.step(0.2);
        expect("the second click of a double is the stick actions'",
            (out[kRight].functions & fb::functionBit(Function::Palm)) == 0 &&
                (out[kRight].functions & fb::functionBit(Function::StickActions)) != 0);
    }

    // Grab is per hand: moving the right grip's functions leaves the left grip alone.
    void eachHandStaysIndependent()
    {
        auto bindings = fb::homeBindings();
        bindings[static_cast<std::size_t>(Function::Vats)] = *fb::parseBinding("right tap grip");
        Rig rig(bindings);
        rig.set(kLeft, Source::Grip, true);
        const auto out = rig.step(0.0);
        expect("the left grip passes through", !out[kLeft].synthesized[static_cast<std::size_t>(Slot::Grab)]);
    }
}

int main()
{
    defaultBindingsPassThrough();
    bindingTextRoundTrips();
    movedReloadPresentsTaggedTapOnA();
    useTakesTheWholePress();
    soleTapActsOnPress();
    bypassBlocksUntilRelease();
    eachHandStaysIndependent();
    sharedStickKeepsEveryFunction();
    if (g_failures == 0) std::printf("ROCK function binding tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
