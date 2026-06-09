#pragma once

#include <cstdint>

namespace RE
{
    class InputEvent;
}

namespace rock::input_remap_runtime
{
    struct RawButtonState
    {
        bool available{ false };
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    enum DiagnosticInputFlags : std::uint32_t
    {
        kDiagnosticInputPrimaryTriggerHeld = 1u << 0,
        kDiagnosticInputPrimaryTriggerPressed = 1u << 1,
        kDiagnosticInputPrimaryTriggerReleased = 1u << 2,
        kDiagnosticInputRightThumbstickLeftPressed = 1u << 3,
        kDiagnosticInputRightThumbstickRightPressed = 1u << 4,
        kDiagnosticInputRightThumbstickUpPressed = 1u << 5,
        kDiagnosticInputRightThumbstickDownPressed = 1u << 6,
    };

    enum DiagnosticSuppressionFlags : std::uint32_t
    {
        kDiagnosticSuppressionPrimaryTrigger = 1u << 0,
        kDiagnosticSuppressionRightThumbstick = 1u << 1,
    };

    struct DiagnosticInputSnapshot
    {
        std::uint64_t sequence{ 0 };
        std::uint32_t flags{ 0 };
        float rightThumbstickX{ 0.0f };
        float rightThumbstickY{ 0.0f };
        float primaryTriggerAxisX{ 0.0f };
    };

    bool installInputRemapHooks();
    bool isInputRemapHookInstalled();

    void setGameplayInputAllowed(bool allowed);
    void setWeaponDrawn(bool weaponDrawn);
    bool isMenuInputActive();
    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event);
    void processPendingWeaponToggleRequests();

    RawButtonState peekRawButtonState(bool isLeft, int buttonId);
    RawButtonState consumeRawButtonState(bool isLeft, int buttonId);
    DiagnosticInputSnapshot consumeDiagnosticInputSnapshot();

    bool setExternalPrimaryTriggerSuppression(std::uint64_t ownerToken, bool suppress);
    bool setExternalDiagnosticInputSuppression(std::uint64_t ownerToken, std::uint32_t suppressionFlags);
}
