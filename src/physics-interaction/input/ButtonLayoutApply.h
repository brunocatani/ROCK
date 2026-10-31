#pragma once

#include <cmath>

#include <openvr.h>

#include "physics-interaction/input/ButtonLayoutPolicy.h"

// Applies a ButtonLayoutPolicy layout to one OpenVR controller-state read.
namespace rock::button_layout_policy
{
    // Pad latch: Released while Axis0 is up, Stick when a press is not claimed
    // by a pad region the layout uses, otherwise the claiming region.
    enum class PadLatch : std::uint8_t
    {
        Released,
        Stick,
        PadUp,
        PadDown,
        PadLeft,
        PadRight,
        PadCenter,
    };

    [[nodiscard]] inline Source classifyPadRegion(float x, float y) noexcept
    {
        if (!std::isfinite(x) || !std::isfinite(y)) return Source::PadCenter;
        if (std::sqrt(x * x + y * y) < kPadCenterRadius) return Source::PadCenter;
        if (std::fabs(y) >= std::fabs(x)) return y >= 0.0f ? Source::PadUp : Source::PadDown;
        return x >= 0.0f ? Source::PadRight : Source::PadLeft;
    }

    [[nodiscard]] constexpr PadLatch latchFor(Source region) noexcept
    {
        switch (region) {
        case Source::PadUp: return PadLatch::PadUp;
        case Source::PadDown: return PadLatch::PadDown;
        case Source::PadLeft: return PadLatch::PadLeft;
        case Source::PadRight: return PadLatch::PadRight;
        case Source::PadCenter: return PadLatch::PadCenter;
        default: return PadLatch::Stick;
        }
    }

    // The latch a new Axis0 press takes; Released when Axis0 is up.
    [[nodiscard]] inline PadLatch resolvePressLatch(const HandLayout& layout, const vr::VRControllerState_t& physical) noexcept
    {
        if ((physical.ulButtonPressed & bit(kStickBit)) == 0) return PadLatch::Released;
        if (!layout.usesPadRegions()) return PadLatch::Stick;
        const auto region = classifyPadRegion(physical.rAxis[kStickAxis].x, physical.rAxis[kStickAxis].y);
        return layout.uses(region) ? latchFor(region) : PadLatch::Stick;
    }

    struct SourceSample
    {
        bool pressed{ false };
        bool touched{ false };
        float value{ 0.0f };
    };

    [[nodiscard]] inline SourceSample sampleSource(Source source, const vr::VRControllerState_t& physical, PadLatch latch) noexcept
    {
        const auto pressedBit = [&](int index) { return (physical.ulButtonPressed & bit(index)) != 0; };
        const auto touchedBit = [&](int index) { return (physical.ulButtonTouched & bit(index)) != 0; };
        const auto digital = [&](int index) {
            const bool pressed = pressedBit(index);
            return SourceSample{ pressed, pressed || touchedBit(index), pressed ? 1.0f : 0.0f };
        };
        const auto padRegion = [&](PadLatch region) {
            const bool pressed = pressedBit(kStickBit) && latch == region;
            return SourceSample{ pressed, touchedBit(kStickBit), pressed ? 1.0f : 0.0f };
        };
        switch (source) {
        case Source::None: return {};
        case Source::Trigger:
            return { pressedBit(kTriggerBit), touchedBit(kTriggerBit) || pressedBit(kTriggerBit), physical.rAxis[kTriggerAxis].x };
        case Source::Grip: {
            const bool pressed = pressedBit(kGripBit);
            // Touch-style grips report an analog pull; button grips (Vive, WMR) do not.
            const float analog = physical.rAxis[kGripAxis].x;
            return { pressed, pressed || touchedBit(kGripBit), analog > 0.0f ? analog : (pressed ? 1.0f : 0.0f) };
        }
        case Source::LowerFace: return digital(kLowerFaceBit);
        case Source::UpperFace: return digital(kUpperFaceBit);
        case Source::Stick: {
            const bool pressed = pressedBit(kStickBit) && latch == PadLatch::Stick;
            return { pressed, touchedBit(kStickBit), pressed ? 1.0f : 0.0f };
        }
        case Source::GripClick: return digital(kGripClickBit);
        case Source::Axis3Click: return digital(kAxis3ClickBit);
        case Source::Axis4Click: return digital(kAxis4ClickBit);
        case Source::PadUp: return padRegion(PadLatch::PadUp);
        case Source::PadDown: return padRegion(PadLatch::PadDown);
        case Source::PadLeft: return padRegion(PadLatch::PadLeft);
        case Source::PadRight: return padRegion(PadLatch::PadRight);
        case Source::PadCenter: return padRegion(PadLatch::PadCenter);
        case Source::Count: break;
        }
        return {};
    }

    /*
     * Rewrites the five logical buttons of one read from their configured
     * sources. Everything else (system button, stick and finger axes, unused
     * bits) passes through. A logical trigger or grab driven by a digital
     * source reports 1.0 on its analog axis while pressed, so analog readers
     * (native fire, FRIK finger curl) stay consistent with the button bit.
     */
    inline void apply(const HandLayout& layout, vr::VRControllerState_t& state, PadLatch latch) noexcept
    {
        const vr::VRControllerState_t physical = state;
        const auto set = [&](int index, bool pressed, bool touched) {
            const auto mask = bit(index);
            state.ulButtonPressed = pressed ? (state.ulButtonPressed | mask) : (state.ulButtonPressed & ~mask);
            state.ulButtonTouched = touched ? (state.ulButtonTouched | mask) : (state.ulButtonTouched & ~mask);
        };

        const auto trigger = layout.source(Slot::Trigger);
        if (trigger != Source::Trigger) {
            const auto sample = sampleSource(trigger, physical, latch);
            set(kTriggerBit, sample.pressed, sample.touched);
            state.rAxis[kTriggerAxis] = { sample.value, 0.0f };
        }

        const auto grab = layout.source(Slot::Grab);
        if (grab != Source::Grip) {
            const auto sample = sampleSource(grab, physical, latch);
            set(kGripBit, sample.pressed, sample.touched);
            // The Touch grip's companion Axis2 click belongs to the physical grip only.
            set(kGripClickBit, false, false);
            state.rAxis[kGripAxis] = { sample.value, 0.0f };
        }

        const auto lowerFace = layout.source(Slot::LowerFace);
        if (lowerFace != Source::LowerFace) {
            const auto sample = sampleSource(lowerFace, physical, latch);
            set(kLowerFaceBit, sample.pressed, sample.touched);
        }

        const auto upperFace = layout.source(Slot::UpperFace);
        if (upperFace != Source::UpperFace) {
            const auto sample = sampleSource(upperFace, physical, latch);
            set(kUpperFaceBit, sample.pressed, sample.touched);
        }

        const auto stick = layout.source(Slot::StickClick);
        if (stick != Source::Stick || layout.usesPadRegions()) {
            // The stick touch bit stays physical with the stick axes it describes.
            const auto sample = sampleSource(stick, physical, latch);
            set(kStickBit, sample.pressed, (physical.ulButtonTouched & bit(kStickBit)) != 0);
        }
    }
}
