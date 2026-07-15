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

    bool installInputRemapHooks();
    bool isInputRemapHookInstalled();

    void setGameplayInputAllowed(bool allowed);
    void setWeaponDrawn(bool weaponDrawn);
    void setHandHeldWeapon(bool isLeft, bool heldWeapon);
    void setHandInteractionEngaged(bool isLeft, bool engaged);
    void setHeldObjectFormId(bool isLeft, std::uint32_t formId);
    void setEquippedWeaponFiringGripInputActive(bool active);
    void setEquippedWeaponPrimaryDetached(bool detached);
    // Left-hand fire: while true (LEFT hand occupies the firing grip), the
    // OpenVR hooks present the physical left trigger to the game as the
    // primary wand's trigger and blank both physical trigger identities.
    void setEquippedWeaponLeftHandFiringActive(bool active);
    void setProviderOpenVrGameInputSuppressed(bool isLeft, bool suppressed);
    /*
     * Once-per-frame reload input for the firing hand on the SECONDARY wand
     * (left X in the default layout): that button never produces an engine
     * event ROCK can hook, so PhysicsInteraction drives this poll, which
     * consumes the raw accept-button press edge and dispatches the native
     * reload action while that physical hand occupies the firing grip.
     * Call every frame regardless of weapon state so stale press edges can
     * never latch across a firing-hand change. Frame thread only.
     */
    void updateFiringHandReloadInput();
    bool isMenuInputActive();
    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event);
    bool isNativePipboyInputSuppressionActive();

    // Test-and-clear: true once for the frame after an Activate/WandAccept
    // (A button) press fired on this hand while it was holding a ROCK
    // object and developer mode is enabled. See saved_grab_offset feature.
    bool consumePendingSavedGrabOffsetRequest(bool isLeft);

    RawButtonState peekRawButtonState(bool isLeft, int buttonId);
    RawButtonState consumeRawButtonState(bool isLeft, int buttonId);
    // Physical level only: bypasses menu edge rearming so equipped-weapon
    // ownership can reconcile the player's actual hand state after menu exit.
    bool isRawButtonPhysicallyHeld(bool isLeft, int buttonId);
}
