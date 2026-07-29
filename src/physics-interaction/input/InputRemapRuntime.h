#pragma once

#include <cstdint>

#include "physics-interaction/weapon/PipboyEquipPolicy.h"

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

    struct PipboyEquipTriggerResolution
    {
        pipboy_equip_policy::Hand hand{ pipboy_equip_policy::Hand::Right };
        pipboy_equip_policy::TriggerSource source{ pipboy_equip_policy::TriggerSource::FallbackRight };
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
    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event);
    bool isNativePipboyInputSuppressionActive();
    bool isPipboyMenuOpen();

    // Consumes the menu-generation-bound trigger evidence for one Pip-Boy
    // selection. Physical held state wins; ambiguous/missing input preserves
    // Bethesda's native right-hand behavior.
    PipboyEquipTriggerResolution consumePipboyEquipTriggerResolution();

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
