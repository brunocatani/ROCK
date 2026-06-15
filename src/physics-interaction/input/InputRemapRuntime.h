#pragma once

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

    bool installInputRemapHooks();
    bool isInputRemapHookInstalled();

    void setGameplayInputAllowed(bool allowed);
    void setWeaponDrawn(bool weaponDrawn);
    void setRightHandHeldWeapon(bool heldWeapon);
    void setEquippedWeaponPrimaryDetachInputActive(bool active);
    void setEquippedWeaponPrimaryDetached(bool detached);
    bool isMenuInputActive();
    bool shouldDeferGrabInputForVirtualHolsters(bool isLeft, int buttonId);
    bool requestVirtualHolstersHolsterPress(bool isLeft, float releaseX, float releaseY, float releaseZ);
    void advanceVirtualHolstersSyntheticInputFrame();
    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event);
    void processPendingWeaponToggleRequests();

    RawButtonState peekRawButtonState(bool isLeft, int buttonId);
    RawButtonState consumeRawButtonState(bool isLeft, int buttonId);
}
