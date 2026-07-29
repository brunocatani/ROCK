#pragma once

#include <cstdint>

namespace rock::input_remap_policy
{
    enum class Hand : std::uint8_t
    {
        Left,
        Right,
    };

    struct Settings
    {
        bool enabled{ true };
        int grabButtonId{ 2 };
        bool suppressRightGrabGameInput{ true };
        bool suppressRightFavoritesGameInput{ true };
        bool suppressRightTriggerGameInput{ true };
        bool suppressNativeMeleeThrowGameInput{ true };
        bool suppressPipboyGameInputWhileHolding{ true };
    };

    struct Input
    {
        Hand hand{ Hand::Right };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        std::uint64_t rawPressed{ 0 };
        std::uint64_t rawTouched{ 0 };
        std::uint64_t previousRawPressed{ 0 };
    };

    struct Decision
    {
        bool grabHeld{ false };
        bool grabPressed{ false };
        bool grabReleased{ false };
    };

    struct NativeActionSuppressionInput
    {
        bool remapEnabled{ true };
        bool suppressionEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool eventHandHeldWeapon{ false };
        bool primaryHandEvent{ false };
        bool equippedWeaponFiringGripInputActive{ false };
        bool equippedWeaponPrimaryDetached{ false };
        bool pipboyHandEngaged{ false };
        bool takeEquipHandEngaged{ false };
        bool takeEquipTargetEligible{ false };
        bool eventMatched{ false };
    };

    struct LegacyPipboyTriggerOpenInput
    {
        bool remapEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool eventMatched{ false };
        bool secondaryWandEvent{ false };
    };

    struct NativeActivateReloadInput
    {
        bool remapEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool primaryHandEvent{ false };
        bool firingHandIsPrimaryHand{ false };
        bool buttonJustPressed{ false };
        bool eventMatched{ false };
    };

    struct ManualScopeActivateInput
    {
        bool manualScopeEnabled{ false };
        bool rawInputCaptureAvailable{ false };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool primaryHandEvent{ false };
        bool firingHandIsPrimaryHand{ false };
        bool eventMatched{ false };
    };

    /*
     * X-side reload input: the secondary wand's accept button never produces
     * an engine event ROCK can hook (see shouldRouteFiringHandActivateReload),
     * so the runtime polls ROCK's own raw OpenVR press edge for it once per
     * frame and dispatches the native reload action directly.
     */
    struct SecondaryHandReloadInput
    {
        bool remapEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool firingHandIsSecondaryHand{ false };
        bool acceptButtonPressedEdge{ false };
    };

    struct EquippedWeaponFiringGripInputGate
    {
        bool featureAvailable{ false };
        bool canUseFiringGripInput{ false };
        bool menuInputActive{ false };
    };

    struct HeldWeaponEquipInput
    {
        bool remapEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool heldWeaponAtFrameStart{ false };
        bool heldWeaponNow{ false };
        Hand heldWeaponHand{ Hand::Right };
        Hand triggerInputHand{ Hand::Right };
        bool triggerPressedEdge{ false };
        bool gripZoneEquipEnabled{ false };
        bool gripZoneEquipSettled{ false };
    };

    struct EdgeTransition
    {
        std::uint64_t previousPressedForEvaluation{ 0 };
        std::uint64_t pressedEdges{ 0 };
        std::uint64_t releasedEdges{ 0 };
    };

    [[nodiscard]] constexpr bool isValidButtonId(int buttonId)
    {
        return buttonId >= 0 && buttonId < 64;
    }

    /*
     * Fallout VR consumes trigger as both an OpenVR button bit and analog Axis1.x.
     * ROCK keeps those raw values readable and suppresses the native trigger action
     * gates instead, otherwise attack handling can still auto-ready the weapon even
     * after the ReadyWeapon action itself is suppressed.
     */
    inline constexpr int kOpenVrAxisButtonBase = 32;
    inline constexpr int kOpenVrAxisCount = 5;
    inline constexpr int kOpenVrSteamVrTriggerButtonId = kOpenVrAxisButtonBase + 1;

    /*
     * OpenVR k_EButton_A: the lower face button on BOTH controllers (right A,
     * left X) sets bit 7 of its own controller's ulButtonPressed mask.
     * Live-verified 2026-07-12 via the reload-gate trace: a right-A Activate
     * event reports idCode 7 while the right tracker holds bit 7, and a held
     * left X raises the same bit on the left tracker only.
     */
    inline constexpr int kOpenVrAcceptButtonId = 7;

    [[nodiscard]] constexpr bool isAllowedGrabButtonId(int buttonId)
    {
        return isValidButtonId(buttonId) && buttonId != kOpenVrSteamVrTriggerButtonId;
    }

    [[nodiscard]] constexpr std::uint64_t buttonMask(int buttonId)
    {
        return isValidButtonId(buttonId) ? (std::uint64_t{ 1 } << static_cast<unsigned>(buttonId)) : 0;
    }

    [[nodiscard]] constexpr bool isOpenVrAxisButtonId(int buttonId)
    {
        return buttonId >= kOpenVrAxisButtonBase && buttonId < kOpenVrAxisButtonBase + kOpenVrAxisCount;
    }

    [[nodiscard]] constexpr std::uint8_t axisMaskFromOpenVrButtonId(int buttonId)
    {
        return isOpenVrAxisButtonId(buttonId) ? static_cast<std::uint8_t>(std::uint8_t{ 1 } << static_cast<unsigned>(buttonId - kOpenVrAxisButtonBase)) : 0;
    }

    [[nodiscard]] constexpr bool hasButton(std::uint64_t pressedMask, int buttonId)
    {
        const auto mask = buttonMask(buttonId);
        return mask != 0 && (pressedMask & mask) != 0;
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeGripReadyAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched &&
               (!input.weaponDrawn || input.equippedWeaponFiringGripInputActive);
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeTriggerAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched &&
               (!input.weaponDrawn || input.eventHandHeldWeapon || input.equippedWeaponPrimaryDetached);
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeGripReloadAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched &&
               input.weaponDrawn && input.primaryHandEvent;
    }

    /*
     * Reload belongs to the physical hand currently occupying the firing
     * grip: right A for a right firing grip, left X for a left firing grip.
     *
     * A-side route. The hooked ActivateHandler only ever receives Activate/
     * WandAccept from the PRIMARY wand — live-verified 2026-07-12: every gate
     * trace across sessions reports a primary-wand device and left-X presses
     * never produce an event here (an earlier raw-snapshot resolution built
     * on the opposite assumption also rejected legitimate presses whenever
     * both controllers held their shared accept-button bit). The event's
     * physical hand therefore IS the primary-wand hand; reload routes only
     * while that same hand occupies the firing grip. A secondary-hand firing
     * grip is fed by shouldDispatchSecondaryHandReloadPress instead.
     */
    [[nodiscard]] constexpr bool shouldRouteFiringHandActivateReload(const NativeActivateReloadInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.weaponDrawn &&
               input.primaryHandEvent && input.firingHandIsPrimaryHand &&
               input.buttonJustPressed && input.eventMatched;
    }

    /*
     * In manual-scope mode the complete primary-wand A/X event is claimed by
     * ROCK. Raw physical state decides later whether release means reload or
     * whether the hold threshold converted the gesture into scope ownership.
     */
    [[nodiscard]] constexpr bool shouldDeferFiringHandActivateForManualScope(const ManualScopeActivateInput& input)
    {
        return input.manualScopeEnabled && input.rawInputCaptureAvailable && input.gameplayInputAllowed &&
               !input.menuInputActive && input.weaponDrawn && input.primaryHandEvent &&
               input.firingHandIsPrimaryHand && input.eventMatched;
    }

    /*
     * X-side route twin of the gate above, evaluated per frame from the raw
     * press edge of the SECONDARY wand's accept button while that physical
     * hand occupies the firing grip.
     */
    [[nodiscard]] constexpr bool shouldDispatchSecondaryHandReloadPress(const SecondaryHandReloadInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.weaponDrawn &&
               input.firingHandIsSecondaryHand && input.acceptButtonPressedEdge;
    }

    [[nodiscard]] constexpr bool shouldConsumeEquippedWeaponFiringGripInput(const EquippedWeaponFiringGripInputGate& input)
    {
        return input.featureAvailable;
    }

    [[nodiscard]] constexpr bool shouldUseEquippedWeaponFiringGripInput(const EquippedWeaponFiringGripInputGate& input)
    {
        return input.featureAvailable && input.canUseFiringGripInput && !input.menuInputActive;
    }

    /*
     * Loose-weapon equip fires on an explicit trigger edge from the SAME hand
     * that owns the held weapon or after that hand settles in the firing-grip
     * zone. Both paths apply to either physical hand.
     */
    [[nodiscard]] constexpr bool shouldRequestHeldWeaponEquip(const HeldWeaponEquipInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.heldWeaponAtFrameStart && input.heldWeaponNow &&
               ((input.triggerPressedEdge && input.triggerInputHand == input.heldWeaponHand) ||
                   (input.gripZoneEquipEnabled && input.gripZoneEquipSettled));
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeFavoritesAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.eventMatched;
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeMeleeThrowAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched;
    }

    /*
     * ROCK permanently moves gameplay Pip-Boy opening off the secondary-wand
     * trigger. Only the verified VR WandTrigger event is claimed here: direct
     * keyboard/gamepad Pipboy bindings and primary-wand attack events remain
     * native, and menu input remains native so an open Pip-Boy keeps its
     * existing controls. Flashlight suppression remains separately governed
     * by shouldSuppressNativePipboyAction below.
     */
    [[nodiscard]] constexpr bool shouldSuppressLegacyPipboyTriggerOpen(const LegacyPipboyTriggerOpenInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive &&
               input.eventMatched && input.secondaryWandEvent;
    }

    /*
     * FO4VR's separate PipboyLightHandler still owns the secondary trigger's
     * flashlight hold. While the pipboy hand is engaged in a ROCK interaction
     * that remaining native action is suppressed, while the raw OpenVR button
     * stays readable. The same policy continues to protect direct Pipboy input
     * bindings during an engaged interaction. Menu input stays native.
     */
    [[nodiscard]] constexpr bool shouldSuppressNativePipboyAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched &&
               !input.primaryHandEvent && input.pipboyHandEngaged;
    }

    /*
     * FO4's Activate button ("Activate"/"WandAccept") is one native event regardless of
     * on-screen prompt text (Take/Talk/Open/Search/Read) - verified 2026-07-05 via raw
     * disassembly of ActivateHandler::HandleEvent's call chain: there is no separate "Take"
     * input action, the Take-vs-other outcome is decided deep inside the target ref's own
     * per-FormType virtual Activate dispatch, not reachable as a flat branch from the input
     * handler. ROCK does not chase that internal dispatch; it instead classifies the same wand
     * pick-ref target the handler is about to act on by FormType (ini-configurable allowlist)
     * and only suppresses when the SAME hand whose wand fired the press is currently holding a
     * ROCK object, so Talk/Open/Search/Read and the opposite hand's Activate keep working on
     * the same button.
     */
    [[nodiscard]] constexpr bool shouldSuppressNativeTakeEquipAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive &&
               input.eventMatched && input.takeEquipHandEngaged && input.takeEquipTargetEligible;
    }

    [[nodiscard]] constexpr bool shouldInstallNativeActionSuppressionHook(bool remapEnabled, bool suppressionEnabled)
    {
        return remapEnabled && suppressionEnabled;
    }

    [[nodiscard]] constexpr bool shouldInstallPipboyPauseArbitrationHooks(const bool remapEnabled)
    {
        return remapEnabled;
    }

    [[nodiscard]] constexpr bool shouldInstallActivateEventHook(const bool remapEnabled, const bool manualScopeEnabled)
    {
        return remapEnabled || manualScopeEnabled;
    }

    [[nodiscard]] constexpr bool shouldInstallRawControllerHooks(const bool remapEnabled, const bool manualScopeEnabled)
    {
        return remapEnabled || manualScopeEnabled;
    }

    [[nodiscard]] constexpr EdgeTransition evaluateEdgeTransition(bool hadPrevious, std::uint64_t previousPressed, std::uint64_t currentPressed)
    {
        if (!hadPrevious) {
            return EdgeTransition{ .previousPressedForEvaluation = currentPressed };
        }

        return EdgeTransition{
            .previousPressedForEvaluation = previousPressed,
            .pressedEdges = currentPressed & ~previousPressed,
            .releasedEdges = previousPressed & ~currentPressed,
        };
    }

    [[nodiscard]] constexpr Decision evaluate(const Input& input, const Settings& settings)
    {
        Decision decision{};

        const auto grabMask = isAllowedGrabButtonId(settings.grabButtonId) ? buttonMask(settings.grabButtonId) : 0;

        decision.grabHeld = grabMask != 0 && (input.rawPressed & grabMask) != 0;
        decision.grabPressed = grabMask != 0 && (input.rawPressed & grabMask) != 0 && (input.previousRawPressed & grabMask) == 0;
        decision.grabReleased = grabMask != 0 && (input.rawPressed & grabMask) == 0 && (input.previousRawPressed & grabMask) != 0;
        return decision;
    }
}
