#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::debug_controller_policy
{
    /*
     * ROCK's debug pad is deliberately modeled as a small command surface over
     * existing runtime config fields. The PS3 pad arrives through SCP Toolkit as
     * XInput, while the values it controls are ROCK hand-space and overlay
     * settings. Keeping button edges, stick deadzones, and pivot math here makes
     * the runtime wrapper a thin polling/persistence layer instead of mixing
     * hardware details into grab or overlay ownership.
     */
    namespace xinput_button
    {
        inline constexpr std::uint16_t A = 0x1000;
        inline constexpr std::uint16_t B = 0x2000;
        inline constexpr std::uint16_t X = 0x4000;
        inline constexpr std::uint16_t Y = 0x8000;
    }

    enum class Command : std::uint8_t
    {
        ToggleHandColliders,
        ToggleWeaponColliders,
        TogglePivotTuning,
        SelectPivotHand
    };

    struct ButtonEdges
    {
        bool connected{ false };
        std::uint16_t held{ 0 };
        std::uint16_t pressed{ 0 };
        std::uint16_t released{ 0 };
    };

    struct AnalogSample
    {
        float leftX{ 0.0f };
        float leftY{ 0.0f };
        float rightX{ 0.0f };
        float rightY{ 0.0f };
    };

    struct PivotVector
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct PivotTuneSettings
    {
        float deadzone{ 0.22f };
        float speedGameUnitsPerSecond{ 8.0f };
        float minComponent{ -50.0f };
        float maxComponent{ 50.0f };
    };

    [[nodiscard]] constexpr std::uint16_t buttonForCommand(Command command)
    {
        switch (command) {
        case Command::ToggleHandColliders:
            return xinput_button::A;
        case Command::ToggleWeaponColliders:
            return xinput_button::B;
        case Command::TogglePivotTuning:
            return xinput_button::X;
        case Command::SelectPivotHand:
            return xinput_button::Y;
        }

        return 0;
    }

    [[nodiscard]] constexpr ButtonEdges evaluateButtonEdges(bool hadPreviousSample, std::uint16_t previousButtons, bool connected, std::uint16_t currentButtons)
    {
        if (!connected) {
            return {};
        }

        if (!hadPreviousSample) {
            return ButtonEdges{ .connected = true, .held = currentButtons };
        }

        return ButtonEdges{
            .connected = true,
            .held = currentButtons,
            .pressed = static_cast<std::uint16_t>(currentButtons & ~previousButtons),
            .released = static_cast<std::uint16_t>(previousButtons & ~currentButtons),
        };
    }

    [[nodiscard]] constexpr bool commandPressed(const ButtonEdges& edges, Command command)
    {
        const auto button = buttonForCommand(command);
        return edges.connected && button != 0 && (edges.pressed & button) != 0;
    }

    [[nodiscard]] inline float applyAxisDeadzone(float value, float deadzone)
    {
        if (!std::isfinite(value)) {
            return 0.0f;
        }

        const float clamped = std::clamp(value, -1.0f, 1.0f);
        const float magnitude = std::fabs(clamped);
        const float sanitizedDeadzone = std::clamp(deadzone, 0.0f, 0.95f);
        if (magnitude <= sanitizedDeadzone) {
            return 0.0f;
        }

        const float normalized = (magnitude - sanitizedDeadzone) / (1.0f - sanitizedDeadzone);
        return std::copysign(normalized, clamped);
    }

    [[nodiscard]] inline PivotVector applyPivotTuning(PivotVector current, const AnalogSample& sample, float deltaSeconds, const PivotTuneSettings& settings = {})
    {
        const float dt = std::isfinite(deltaSeconds) ? std::clamp(deltaSeconds, 0.0f, 0.1f) : 0.0f;
        const float speed = std::isfinite(settings.speedGameUnitsPerSecond) ? std::max(settings.speedGameUnitsPerSecond, 0.0f) : 0.0f;
        const float step = speed * dt;

        current.x += applyAxisDeadzone(sample.leftY, settings.deadzone) * step;
        current.y += applyAxisDeadzone(sample.leftX, settings.deadzone) * step;
        current.z += applyAxisDeadzone(sample.rightY, settings.deadzone) * step;

        const float minComponent = std::isfinite(settings.minComponent) ? settings.minComponent : -50.0f;
        const float maxComponent = std::isfinite(settings.maxComponent) ? settings.maxComponent : 50.0f;
        const auto [minValue, maxValue] = std::minmax(minComponent, maxComponent);

        current.x = std::clamp(std::isfinite(current.x) ? current.x : 0.0f, minValue, maxValue);
        current.y = std::clamp(std::isfinite(current.y) ? current.y : 0.0f, minValue, maxValue);
        current.z = std::clamp(std::isfinite(current.z) ? current.z : 0.0f, minValue, maxValue);
        return current;
    }

    [[nodiscard]] inline bool pivotChanged(const PivotVector& before, const PivotVector& after, float epsilon = 0.0001f)
    {
        return std::fabs(before.x - after.x) > epsilon || std::fabs(before.y - after.y) > epsilon || std::fabs(before.z - after.z) > epsilon;
    }
}
