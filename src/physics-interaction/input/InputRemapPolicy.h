#pragma once

#include <cstdint>

namespace rock::input_remap_policy
{
    enum class Hand : std::uint8_t
    {
        Left,
        Right,
    };

    enum class PhysicalButtonHandResolution : std::uint8_t
    {
        Unresolved,
        Left,
        Right,
    };

    struct PhysicalButtonHandInput
    {
        bool leftAvailable{ false };
        bool leftHeld{ false };
        bool rightAvailable{ false };
        bool rightHeld{ false };
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
        bool virtualHolstersCompatibilityEnabled{ true };
        bool virtualHolstersDeferGrabInZone{ true };
        bool virtualHolstersDeferWeaponToggleInZone{ true };
        bool virtualHolstersDeferOnlyMatchingButton{ false };
        bool realisticWeaponHandlingEnabled{ false };
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
        bool equippedWeaponPrimaryDetachInputActive{ false };
        bool equippedWeaponPrimaryDetached{ false };
        bool pipboyHandEngaged{ false };
        bool takeEquipHandEngaged{ false };
        bool takeEquipTargetEligible{ false };
        bool eventMatched{ false };
    };

    struct NativeActivateReloadInput
    {
        bool remapEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool eventHandResolved{ false };
        Hand eventHand{ Hand::Right };
        Hand firingHand{ Hand::Right };
        bool buttonJustPressed{ false };
        bool virtualHolstersOwnsInput{ false };
        bool eventMatched{ false };
    };

    struct EquippedWeaponPrimaryDetachInputGate
    {
        bool featureAvailable{ false };
        bool canUsePrimaryDetachInput{ false };
        bool menuInputActive{ false };
        bool virtualHolstersOwnsInput{ false };
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
        bool legacyAutoEquipPrimaryHand{ true };
        bool autoEquipEnabled{ false };
        bool autoEquipSettled{ false };
        bool gripZoneEquipEnabled{ false };
        bool gripZoneEquipSettled{ false };
    };

    struct VirtualHolstersCompatibilityInput
    {
        bool compatibilityEnabled{ true };
        bool deferActionEnabled{ true };
        bool deferOnlyMatchingButton{ false };
        bool realisticWeaponHandlingEnabled{ false };
        bool apiAvailable{ false };
        bool initialized{ false };
        bool handInZone{ false };
        int rockButtonId{ 2 };
        int holsterButtonId{ 2 };
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

    /*
     * FO4VR's primary/secondary wand labels do not identify the physical
     * controller that produced Activate/WandAccept. Resolve A/X from ROCK's
     * pre-remap OpenVR snapshots instead. Both controllers must be sampled,
     * and exactly one may hold the native event's button; ambiguity fails
     * closed so one hand can never reload the other hand's weapon.
     */
    [[nodiscard]] constexpr PhysicalButtonHandResolution resolvePhysicalButtonHand(const PhysicalButtonHandInput& input)
    {
        if (!input.leftAvailable || !input.rightAvailable || input.leftHeld == input.rightHeld) {
            return PhysicalButtonHandResolution::Unresolved;
        }
        return input.leftHeld ? PhysicalButtonHandResolution::Left : PhysicalButtonHandResolution::Right;
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeGripReadyAction(const NativeActionSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.eventMatched &&
               (!input.weaponDrawn || input.equippedWeaponPrimaryDetachInputActive);
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
     * Activate/WandAccept naming is shared by both controllers; physical hand
     * identity, not the game's fixed primary-wand role, selects the route.
     */
    [[nodiscard]] constexpr bool shouldRouteFiringHandActivateReload(const NativeActivateReloadInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.weaponDrawn && input.eventHandResolved &&
               input.eventHand == input.firingHand &&
               input.buttonJustPressed && !input.virtualHolstersOwnsInput && input.eventMatched;
    }

    [[nodiscard]] constexpr bool shouldConsumeEquippedWeaponPrimaryDetachInput(const EquippedWeaponPrimaryDetachInputGate& input)
    {
        return input.featureAvailable;
    }

    [[nodiscard]] constexpr bool shouldUseEquippedWeaponPrimaryDetachInput(const EquippedWeaponPrimaryDetachInputGate& input)
    {
        return input.featureAvailable && input.canUsePrimaryDetachInput && !input.menuInputActive && !input.virtualHolstersOwnsInput;
    }

    /*
     * Loose-weapon equip fires on an explicit trigger edge from the SAME hand
     * that owns the held weapon. The legacy position-blind settle timer stays
     * primary-hand-only, while the firing-grip zone applies to either hand so
     * the physical hand at the grip becomes the equipped firing hand.
     */
    [[nodiscard]] constexpr bool shouldRequestHeldWeaponEquip(const HeldWeaponEquipInput& input)
    {
        return input.remapEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.heldWeaponAtFrameStart && input.heldWeaponNow &&
               ((input.triggerPressedEdge && input.triggerInputHand == input.heldWeaponHand) ||
                   (input.legacyAutoEquipPrimaryHand && input.autoEquipEnabled && input.autoEquipSettled) ||
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
     * FO4VR's PipboyHandler owns the whole pipboy-hand trigger lifecycle:
     * press starts hold tracking, holding past the game threshold toggles the
     * pipboy light, release opens the Pip-Boy. The VR wand trigger reaches it
     * as user event "WandTrigger" (observed live 2026-07-04 via the hook
     * trace); "Pipboy" covers the flat/gamepad direct bindings. Both wands'
     * trigger events flow through the handler but it only acts on the
     * secondary wand, so suppression must exclude primary-hand events -
     * marking a primary WandTrigger stopped would also block downstream
     * attack handling. While the pipboy hand is engaged in a ROCK interaction
     * (holding an object, two-handing or supporting the equipped weapon, or
     * carrying a part while the primary grip is detached) that trigger
     * belongs to interaction consumers (e.g. PAPER through the provider
     * raw-button API), so both native actions are suppressed together at the
     * verified handler while the raw OpenVR button stays readable. Menu input
     * keeps native handling so the trigger can still close an already-open
     * Pip-Boy.
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

    [[nodiscard]] constexpr bool shouldDeferVirtualHolstersInput(const VirtualHolstersCompatibilityInput& input)
    {
        if (!input.compatibilityEnabled || input.realisticWeaponHandlingEnabled || !input.deferActionEnabled ||
            !input.apiAvailable || !input.initialized || !input.handInZone) {
            return false;
        }

        return !input.deferOnlyMatchingButton || (isValidButtonId(input.rockButtonId) && input.rockButtonId == input.holsterButtonId);
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
