#pragma once

#include <cstdint>

namespace RE
{
    class InputEvent;
}

namespace rock::input_remap_runtime
{
    enum class RawButtonAvailabilityReason : std::uint32_t
    {
        Available = 0,
        HookNotSampled = 1,
        BlockingMenu = 2,
        ReleaseToRearm = 3,
        InvalidButton = 4,
    };

    struct RawButtonState
    {
        bool available{ false };
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
        std::uint64_t sampleSequence{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        RawButtonAvailabilityReason availabilityReason{
            RawButtonAvailabilityReason::HookNotSampled
        };
    };

    struct LogicalJumpState
    {
        bool available{ false };
        bool held{ false };
        std::uint64_t sampleSequence{ 0 };
        std::uint64_t pressSequence{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        RawButtonAvailabilityReason availabilityReason{
            RawButtonAvailabilityReason::HookNotSampled
        };
    };

    bool installInputRemapHooks();
    bool isInputRemapHookInstalled();

    // GameLoaded/config-change boundary only: the FRIK session override
    // reloads its configuration. Never call from a controller/input hook.
    void configurePipboyInput();

    void setGameplayInputAllowed(bool allowed);
    void setWeaponDrawn(bool weaponDrawn);
    void setRealMeleeWeaponEquipped(bool equipped);
    void setHandHeldWeapon(bool isLeft, bool heldWeapon);
    void setHandInteractionEngaged(bool isLeft, bool engaged);
    void setHeldObjectFormId(bool isLeft, std::uint32_t formId);
    void setEquippedWeaponFiringGripInputActive(bool active);
    void setEquippedWeaponPrimaryDetached(bool detached);
    void setEquippedWeaponShoulderSheathActive(bool active);
    // Left-hand fire: while true (LEFT hand occupies the firing grip), the
    // OpenVR hooks present the physical left trigger to the game as the
    // primary wand's trigger and blank both physical trigger identities.
    void setEquippedWeaponLeftHandFiringActive(bool active);
    void setProviderOpenVrGameInputSuppressed(bool isLeft, bool suppressed);
    bool isProviderOpenVrGameInputSuppressedForHand(bool isLeft);
    // Reads captured physical input only; never queries provider leases.
    bool isTriggerGripChordHeld(bool isLeft);
    bool areRawButtonsHeld(bool isLeft, std::uint64_t mask);
    /*
     * Once-per-frame physical firing-hand A/X arbitration. Automatic scope
     * mode preserves press-time reload and only dispatches the secondary-wand
     * raw edge here. Manual mode classifies both hands: release before the
     * configured threshold dispatches reload, while a completed hold publishes
     * native scope activation until release. Frame thread only.
     */
    void updateFiringHandReloadInput(float deltaSeconds);
    // Published manual-scope level state. The native scope decision hook owns
    // the engine transition; input runtime owns only the physical hold gesture.
    bool isManualScopeActivationRequested();
    bool isMenuInputActive();
    // Frame-thread query using the same ViewCaster classification as Activate.
    bool hasNativeActivationTarget(bool primaryHand);
    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event);
    bool isNativePipboyInputSuppressionActive();
    bool isPipboyMenuOpen();
    LogicalJumpState readLogicalJumpState();

    // Test-and-clear: true once for the frame after an Activate/WandAccept
    // (A button) press fired on this hand while it was holding a ROCK
    // object and developer mode is enabled. See saved_grab_offset feature.
    bool consumePendingSavedGrabOffsetRequest(bool isLeft);

    // Test-and-clear: true once after primary-wand B crosses Bethesda's
    // V.A.N.S. hold threshold during eligible gameplay. Frame thread only.

    bool consumeGrenadeQuickDrawHoldRequest();
    RawButtonState peekRawButtonState(bool isLeft, int buttonId);
    // Physical Axis0, sampled before game-input suppression. False returns zero axes.
    bool peekRawThumbstick(bool isLeft, float& x, float& y);
    RawButtonState consumeRawButtonState(bool isLeft, int buttonId);
    // Physical level only: bypasses menu edge rearming so equipped-weapon
    // ownership can reconcile the player's actual hand state after menu exit.
    bool isRawButtonPhysicallyHeld(bool isLeft, int buttonId);
}
