#include "physics-interaction/input/ButtonLayoutApply.h"

#include <cstdio>
#include <cstring>

namespace
{
    namespace layout = rock::button_layout_policy;
    using layout::bit;
    using layout::PadLatch;
    using layout::Slot;
    using layout::Source;

    int g_failures = 0;

    void expect(const char* label, bool value)
    {
        if (!value) {
            std::printf("FAILED: %s\n", label);
            ++g_failures;
        }
    }

    vr::VRControllerState_t state(std::uint64_t pressed, std::uint64_t touched = 0)
    {
        vr::VRControllerState_t s{};
        s.unPacketNum = 42;
        s.ulButtonPressed = pressed;
        s.ulButtonTouched = touched | pressed;
        return s;
    }

    layout::HandLayout with(Slot slot, Source source, layout::HandLayout base = {})
    {
        base.sources[static_cast<std::size_t>(slot)] = source;
        return base;
    }

    vr::VRControllerState_t applied(const layout::HandLayout& hand, vr::VRControllerState_t s)
    {
        layout::apply(hand, s, layout::resolvePressLatch(hand, s));
        return s;
    }

    void identityLeavesEveryFieldUnchanged()
    {
        auto s = state(bit(1) | bit(2) | bit(7) | bit(32) | bit(33) | bit(34) | bit(0), bit(35));
        for (std::size_t i = 0; i < 5; ++i)
            s.rAxis[i] = { 0.1f * static_cast<float>(i + 1), -0.2f };
        const auto before = s;
        const auto after = applied({}, s);
        expect("identity layout is byte-identical", std::memcmp(&before, &after, sizeof(before)) == 0);
    }

    void swappedFaceButtonsMoveBothBits()
    {
        const auto hand = with(Slot::UpperFace, Source::LowerFace, with(Slot::LowerFace, Source::UpperFace));
        const auto pressA = applied(hand, state(bit(7)));
        expect("physical A drives logical B", (pressA.ulButtonPressed & bit(1)) != 0);
        expect("physical A no longer drives logical A", (pressA.ulButtonPressed & bit(7)) == 0);
        const auto pressB = applied(hand, state(bit(1)));
        expect("physical B drives logical A",
            (pressB.ulButtonPressed & bit(7)) != 0 && (pressB.ulButtonPressed & bit(1)) == 0);
    }

    void digitalTriggerSourceSynthesizesAxis()
    {
        const auto hand = with(Slot::LowerFace, Source::Trigger, with(Slot::Trigger, Source::LowerFace));
        auto pulled = state(bit(33));
        pulled.rAxis[1] = { 0.9f, 0.0f };
        const auto fire = applied(hand, pulled);
        expect("physical trigger drives logical A", (fire.ulButtonPressed & bit(7)) != 0);
        expect("physical trigger no longer fires", (fire.ulButtonPressed & bit(33)) == 0 && fire.rAxis[1].x == 0.0f);
        const auto face = applied(hand, state(bit(7)));
        expect("physical A fires with a full axis", (face.ulButtonPressed & bit(33)) != 0 && face.rAxis[1].x == 1.0f);
    }

    void grabFromAnotherButtonDropsGripCompanion()
    {
        const auto hand = with(Slot::Grab, Source::UpperFace, with(Slot::UpperFace, Source::Grip));
        auto squeeze = state(bit(2) | bit(34));
        squeeze.rAxis[2] = { 0.8f, 0.0f };
        const auto out = applied(hand, squeeze);
        expect("physical grip no longer grabs",
            (out.ulButtonPressed & (bit(2) | bit(34))) == 0 && out.rAxis[2].x == 0.0f);
        expect("physical grip drives logical B", (out.ulButtonPressed & bit(1)) != 0);
        const auto press = applied(hand, state(bit(1)));
        expect("physical B grabs with a full grip axis",
            (press.ulButtonPressed & bit(2)) != 0 && press.rAxis[2].x == 1.0f);
    }

    void padRegionsSplitTheClickAndLatch()
    {
        // Vive wand: pad up is B, pad down is A, the centre stays the stick click.
        const auto hand = with(Slot::LowerFace, Source::PadDown, with(Slot::UpperFace, Source::PadUp));
        auto up = state(bit(32));
        up.rAxis[0] = { 0.1f, 0.9f };
        const auto upOut = applied(hand, up);
        expect("pad up drives logical B", (upOut.ulButtonPressed & bit(1)) != 0);
        expect("claimed pad click is not a stick click", (upOut.ulButtonPressed & bit(32)) == 0);
        expect("stick touch stays physical", (upOut.ulButtonTouched & bit(32)) != 0);

        auto centre = state(bit(32));
        centre.rAxis[0] = { 0.05f, -0.1f };
        const auto centreOut = applied(hand, centre);
        expect("unclaimed pad centre is the stick click",
            (centreOut.ulButtonPressed & bit(32)) != 0 && (centreOut.ulButtonPressed & (bit(1) | bit(7))) == 0);

        // The region is latched at press time: sliding down keeps the press on B.
        auto slid = up;
        slid.rAxis[0] = { 0.0f, -0.9f };
        layout::apply(hand, slid, PadLatch::PadUp);
        expect("latched region survives sliding",
            (slid.ulButtonPressed & bit(1)) != 0 && (slid.ulButtonPressed & bit(7)) == 0);
    }

    void namesRoundTrip()
    {
        for (auto value = 0; value < static_cast<int>(Source::Count); ++value) {
            const auto source = static_cast<Source>(value);
            for (const bool isLeft : { false, true }) {
                const auto parsed = layout::parseSource(layout::sourceName(source, isLeft));
                expect("canonical name parses back", parsed && *parsed == source);
            }
        }
        expect("separators are ignored", layout::parseSource("Pad Up") == Source::PadUp);
        expect("unknown names are rejected", !layout::parseSource("trigger2").has_value());
        const auto hand = with(Slot::StickClick, Source::PadCenter, with(Slot::Trigger, Source::None));
        expect("pack round-trips", layout::unpack(layout::pack(hand)) == hand);
    }
}

int main()
{
    identityLeavesEveryFieldUnchanged();
    swappedFaceButtonsMoveBothBits();
    digitalTriggerSourceSynthesizesAxis();
    grabFromAnotherButtonDropsGripCompanion();
    padRegionsSplitTheClickAndLatch();
    namesRoundTrip();
    if (g_failures == 0)
        std::printf("ButtonLayoutTests passed\n");
    return g_failures == 0 ? 0 : 1;
}
