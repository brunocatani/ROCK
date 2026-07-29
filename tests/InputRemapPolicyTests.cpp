#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/ManualScopeInputPolicy.h"
#include "physics-interaction/input/NativeVatsInputSuppressionPolicy.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"

#include <cstdio>

namespace
{
    namespace manual = rock::manual_scope_input_policy;
    namespace nativeVats = rock::native_vats_input_suppression_policy;
    namespace pipboyGesture = rock::pipboy_pause_gesture_policy;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectManualScopeState(const char* label,
        rock::manual_scope_input_policy::State actual,
        rock::manual_scope_input_policy::State expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected state %u, got %u\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }

    bool expectPipboyGestureState(const char* label,
        rock::pipboy_pause_gesture_policy::State actual,
        rock::pipboy_pause_gesture_policy::State expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected state %u, got %u\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }
}

int main()
{
    using namespace rock::input_remap_policy;

    bool ok = true;

    Settings settings{};
    settings.enabled = true;

    ok &= expectTrue("normal grab button id is accepted", isAllowedGrabButtonId(2));
    ok &= expectFalse("SteamVR trigger button id is reserved and rejected for grab", isAllowedGrabButtonId(kOpenVrSteamVrTriggerButtonId));

    settings.grabButtonId = kOpenVrSteamVrTriggerButtonId;
    const auto triggerGrabDecision = evaluate(Input{
                                                .hand = Hand::Right,
                                                .gameplayInputAllowed = true,
                                                .menuInputActive = false,
                                                .weaponDrawn = false,
                                                .rawPressed = buttonMask(kOpenVrSteamVrTriggerButtonId),
                                                .previousRawPressed = 0,
                                            },
        settings);
    ok &= expectFalse("SteamVR trigger does not act as ROCK grab input", triggerGrabDecision.grabPressed);
    settings.grabButtonId = 2;

    NativeActionSuppressionInput base{
        .remapEnabled = true,
        .suppressionEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = false,
        .eventMatched = true,
    };

    ok &= expectTrue("holstered WandGrip suppresses native ready action", shouldSuppressNativeGripReadyAction(base));
    auto drawnGrip = base;
    drawnGrip.weaponDrawn = true;
    ok &= expectFalse("drawn weapon allows native grip ready action", shouldSuppressNativeGripReadyAction(drawnGrip));
    auto firingGripInput = drawnGrip;
    firingGripInput.equippedWeaponFiringGripInputActive = true;
    ok &= expectTrue("firing-grip input suppresses drawn native grip ready action", shouldSuppressNativeGripReadyAction(firingGripInput));
    drawnGrip.primaryHandEvent = true;
    ok &= expectTrue("drawn primary WandGrip suppresses native reload action", shouldSuppressNativeGripReloadAction(drawnGrip));
    auto drawnOffhandGrip = drawnGrip;
    drawnOffhandGrip.primaryHandEvent = false;
    ok &= expectFalse("drawn offhand WandGrip does not suppress reload action", shouldSuppressNativeGripReloadAction(drawnOffhandGrip));

    ok &= expectTrue("holstered WandTrigger suppresses native attack gate", shouldSuppressNativeTriggerAction(base));
    auto drawnTrigger = base;
    drawnTrigger.weaponDrawn = true;
    ok &= expectFalse("drawn weapon allows native trigger attack gate", shouldSuppressNativeTriggerAction(drawnTrigger));
    auto heldWeaponTrigger = drawnTrigger;
    heldWeaponTrigger.eventHandHeldWeapon = true;
    ok &= expectTrue("trigger event from a hand holding a ROCK weapon suppresses native trigger even if weapon drawn", shouldSuppressNativeTriggerAction(heldWeaponTrigger));
    auto primaryDetachedTrigger = drawnTrigger;
    primaryDetachedTrigger.equippedWeaponPrimaryDetached = true;
    ok &= expectTrue("primary-detached equipped weapon suppresses drawn native trigger", shouldSuppressNativeTriggerAction(primaryDetachedTrigger));

    auto favorites = base;
    favorites.weaponDrawn = true;
    ok &= expectTrue("WandThumbClick suppresses native favorites even with weapon drawn", shouldSuppressNativeFavoritesAction(favorites));

    auto meleeThrow = base;
    meleeThrow.weaponDrawn = true;
    ok &= expectTrue("WandGrip suppresses native melee throw even with weapon drawn", shouldSuppressNativeMeleeThrowAction(meleeThrow));

    auto menuFavorites = favorites;
    menuFavorites.menuInputActive = true;
    ok &= expectTrue("menu input still suppresses native favorites handling", shouldSuppressNativeFavoritesAction(menuFavorites));

    auto menuMeleeThrow = meleeThrow;
    menuMeleeThrow.menuInputActive = true;
    ok &= expectFalse("menu input allows native melee throw handling", shouldSuppressNativeMeleeThrowAction(menuMeleeThrow));

    auto unmatched = base;
    unmatched.eventMatched = false;
    ok &= expectFalse("unmatched native event is not suppressed", shouldSuppressNativeTriggerAction(unmatched));

    auto pipboyIdleHand = base;
    ok &= expectFalse("matched Pipboy event with a free pipboy hand keeps native pipboy handling", shouldSuppressNativePipboyAction(pipboyIdleHand));
    auto pipboyHolding = base;
    pipboyHolding.pipboyHandEngaged = true;
    ok &= expectTrue("engaged pipboy hand (hold or weapon grip) suppresses native pipboy open/light", shouldSuppressNativePipboyAction(pipboyHolding));
    auto pipboyHoldingDrawn = pipboyHolding;
    pipboyHoldingDrawn.weaponDrawn = true;
    ok &= expectTrue("weapon drawn does not gate pipboy suppression while holding", shouldSuppressNativePipboyAction(pipboyHoldingDrawn));
    auto pipboyMenu = pipboyHolding;
    pipboyMenu.menuInputActive = true;
    ok &= expectFalse("menu input keeps native pipboy handling so the trigger can close an open Pip-Boy", shouldSuppressNativePipboyAction(pipboyMenu));
    auto pipboyDisabled = pipboyHolding;
    pipboyDisabled.suppressionEnabled = false;
    ok &= expectFalse("disabled pipboy suppression setting keeps native pipboy handling", shouldSuppressNativePipboyAction(pipboyDisabled));
    auto pipboyNoGameplay = pipboyHolding;
    pipboyNoGameplay.gameplayInputAllowed = false;
    ok &= expectFalse("blocked gameplay input keeps native pipboy handling", shouldSuppressNativePipboyAction(pipboyNoGameplay));
    auto pipboyUnmatched = pipboyHolding;
    pipboyUnmatched.eventMatched = false;
    ok &= expectFalse("non-Pipboy event is never suppressed by the pipboy gate", shouldSuppressNativePipboyAction(pipboyUnmatched));
    auto pipboyPrimaryHand = pipboyHolding;
    pipboyPrimaryHand.primaryHandEvent = true;
    ok &= expectFalse("primary-wand trigger event bypasses the pipboy gate so attack handling survives", shouldSuppressNativePipboyAction(pipboyPrimaryHand));

    LegacyPipboyTriggerOpenInput legacyPipboyTrigger{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .eventMatched = true,
        .secondaryWandEvent = true,
    };
    ok &= expectTrue("secondary WandTrigger no longer opens the Pip-Boy during gameplay", shouldSuppressLegacyPipboyTriggerOpen(legacyPipboyTrigger));
    auto legacyPrimaryTrigger = legacyPipboyTrigger;
    legacyPrimaryTrigger.secondaryWandEvent = false;
    ok &= expectFalse("primary trigger remains available to native attack handling", shouldSuppressLegacyPipboyTriggerOpen(legacyPrimaryTrigger));
    auto legacyMenuTrigger = legacyPipboyTrigger;
    legacyMenuTrigger.menuInputActive = true;
    ok &= expectFalse("open-menu trigger behavior remains native", shouldSuppressLegacyPipboyTriggerOpen(legacyMenuTrigger));
    auto legacyDirectPipboy = legacyPipboyTrigger;
    legacyDirectPipboy.eventMatched = false;
    ok &= expectFalse("direct keyboard or gamepad Pipboy binding is not the moved VR trigger", shouldSuppressLegacyPipboyTriggerOpen(legacyDirectPipboy));

    auto takeEquipIdleHand = base;
    takeEquipIdleHand.takeEquipTargetEligible = true;
    ok &= expectFalse("free hand keeps native take/equip on an eligible target", shouldSuppressNativeTakeEquipAction(takeEquipIdleHand));
    auto takeEquipHolding = base;
    takeEquipHolding.takeEquipHandEngaged = true;
    ok &= expectFalse("engaged hand does not suppress a non-eligible activate target", shouldSuppressNativeTakeEquipAction(takeEquipHolding));
    auto takeEquipHoldingEligible = takeEquipHolding;
    takeEquipHoldingEligible.takeEquipTargetEligible = true;
    ok &= expectTrue("engaged hand suppresses take/equip on an eligible target",
        shouldSuppressNativeTakeEquipAction(takeEquipHoldingEligible));
    auto takeEquipDrawn = takeEquipHoldingEligible;
    takeEquipDrawn.weaponDrawn = true;
    ok &= expectTrue("weapon drawn does not gate take/equip suppression while holding", shouldSuppressNativeTakeEquipAction(takeEquipDrawn));
    auto takeEquipMenu = takeEquipHoldingEligible;
    takeEquipMenu.menuInputActive = true;
    ok &= expectFalse("menu input keeps native activate handling for take/equip", shouldSuppressNativeTakeEquipAction(takeEquipMenu));
    auto takeEquipDisabled = takeEquipHoldingEligible;
    takeEquipDisabled.suppressionEnabled = false;
    ok &= expectFalse("disabled take/equip suppression setting keeps native activate handling", shouldSuppressNativeTakeEquipAction(takeEquipDisabled));
    auto takeEquipNoGameplay = takeEquipHoldingEligible;
    takeEquipNoGameplay.gameplayInputAllowed = false;
    ok &= expectFalse("blocked gameplay input keeps native activate handling for take/equip", shouldSuppressNativeTakeEquipAction(takeEquipNoGameplay));
    auto takeEquipUnmatched = takeEquipHoldingEligible;
    takeEquipUnmatched.eventMatched = false;
    ok &= expectFalse("non-Activate event is never suppressed by the take/equip gate", shouldSuppressNativeTakeEquipAction(takeEquipUnmatched));

    NativeActivateReloadInput activateReload{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = true,
        .primaryHandEvent = true,
        .firingHandIsPrimaryHand = true,
        .buttonJustPressed = true,
        .eventMatched = true,
    };
    ok &= expectTrue("right A routes reload while the right hand owns the firing grip", shouldRouteFiringHandActivateReload(activateReload));
    auto heldActivateReload = activateReload;
    heldActivateReload.buttonJustPressed = false;
    ok &= expectFalse("held firing-hand activate does not repeat reload", shouldRouteFiringHandActivateReload(heldActivateReload));
    auto rightAWhileLeftFiring = activateReload;
    rightAWhileLeftFiring.firingHandIsPrimaryHand = false;
    ok &= expectFalse("right A cannot reload while the left hand owns the firing grip", shouldRouteFiringHandActivateReload(rightAWhileLeftFiring));
    auto secondaryWandActivateReload = activateReload;
    secondaryWandActivateReload.primaryHandEvent = false;
    ok &= expectFalse("non-primary-wand activate never routes reload through the event hook", shouldRouteFiringHandActivateReload(secondaryWandActivateReload));
    auto holsteredActivateReload = activateReload;
    holsteredActivateReload.weaponDrawn = false;
    ok &= expectFalse("holstered firing-hand activate does not route reload", shouldRouteFiringHandActivateReload(holsteredActivateReload));
    auto menuActivateReload = activateReload;
    menuActivateReload.menuInputActive = true;
    ok &= expectFalse("menu input blocks firing-hand activate reload", shouldRouteFiringHandActivateReload(menuActivateReload));
    auto unmatchedActivateReload = activateReload;
    unmatchedActivateReload.eventMatched = false;
    ok &= expectFalse("unmatched activate event does not route reload", shouldRouteFiringHandActivateReload(unmatchedActivateReload));

    ManualScopeActivateInput manualActivate{
        .manualScopeEnabled = true,
        .rawInputCaptureAvailable = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = true,
        .primaryHandEvent = true,
        .firingHandIsPrimaryHand = true,
        .eventMatched = true,
    };
    ok &= expectTrue("manual scope claims the complete primary firing-hand activate event",
        shouldDeferFiringHandActivateForManualScope(manualActivate));
    auto automaticActivate = manualActivate;
    automaticActivate.manualScopeEnabled = false;
    ok &= expectFalse("automatic scope leaves primary activate on the existing reload route",
        shouldDeferFiringHandActivateForManualScope(automaticActivate));
    auto missingRawCapture = manualActivate;
    missingRawCapture.rawInputCaptureAvailable = false;
    ok &= expectFalse("manual scope does not swallow reload when raw capture is unavailable",
        shouldDeferFiringHandActivateForManualScope(missingRawCapture));
    auto supportHandActivate = manualActivate;
    supportHandActivate.firingHandIsPrimaryHand = false;
    ok &= expectFalse("manual scope does not claim the support hand activate event",
        shouldDeferFiringHandActivateForManualScope(supportHandActivate));

    SecondaryHandReloadInput secondaryReload{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = true,
        .firingHandIsSecondaryHand = true,
        .acceptButtonPressedEdge = true,
    };
    ok &= expectTrue("left X routes reload while the left hand owns the firing grip", shouldDispatchSecondaryHandReloadPress(secondaryReload));
    auto leftXWhileRightFiring = secondaryReload;
    leftXWhileRightFiring.firingHandIsSecondaryHand = false;
    ok &= expectFalse("left X cannot reload while the right hand owns the firing grip", shouldDispatchSecondaryHandReloadPress(leftXWhileRightFiring));
    auto secondaryReloadNoEdge = secondaryReload;
    secondaryReloadNoEdge.acceptButtonPressedEdge = false;
    ok &= expectFalse("held left X does not repeat reload without a fresh press edge", shouldDispatchSecondaryHandReloadPress(secondaryReloadNoEdge));
    auto secondaryReloadHolstered = secondaryReload;
    secondaryReloadHolstered.weaponDrawn = false;
    ok &= expectFalse("holstered weapon blocks secondary-hand reload press", shouldDispatchSecondaryHandReloadPress(secondaryReloadHolstered));
    auto secondaryReloadMenu = secondaryReload;
    secondaryReloadMenu.menuInputActive = true;
    ok &= expectFalse("menu input blocks secondary-hand reload press", shouldDispatchSecondaryHandReloadPress(secondaryReloadMenu));

    EquippedWeaponFiringGripInputGate firingGripGate{
        .featureAvailable = true,
        .canUseFiringGripInput = false,
    };
    ok &= expectTrue("firing-grip feature consumes stale edges before armed", shouldConsumeEquippedWeaponFiringGripInput(firingGripGate));
    ok &= expectFalse("firing-grip input ignores consumed edges until armed", shouldUseEquippedWeaponFiringGripInput(firingGripGate));
    firingGripGate.canUseFiringGripInput = true;
    ok &= expectTrue("firing-grip input uses fresh edge after armed", shouldUseEquippedWeaponFiringGripInput(firingGripGate));
    firingGripGate.menuInputActive = true;
    ok &= expectFalse("menu input blocks firing-grip edge use", shouldUseEquippedWeaponFiringGripInput(firingGripGate));
    firingGripGate.menuInputActive = false;
    firingGripGate.featureAvailable = false;
    ok &= expectFalse("missing hFRIK blocker export disables firing-grip input consumption", shouldConsumeEquippedWeaponFiringGripInput(firingGripGate));

    HeldWeaponEquipInput equipInput{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .heldWeaponAtFrameStart = true,
        .heldWeaponNow = true,
        .heldWeaponHand = Hand::Right,
        .triggerInputHand = Hand::Right,
        .triggerPressedEdge = true,
    };
    ok &= expectTrue("same-hand trigger edge equips already held ROCK weapon", shouldRequestHeldWeaponEquip(equipInput));
    auto oppositeHandTriggerEquipInput = equipInput;
    oppositeHandTriggerEquipInput.triggerInputHand = Hand::Left;
    ok &= expectFalse("opposite-hand trigger edge cannot equip the held weapon", shouldRequestHeldWeaponEquip(oppositeHandTriggerEquipInput));
    auto leftHandTriggerEquipInput = equipInput;
    leftHandTriggerEquipInput.heldWeaponHand = Hand::Left;
    leftHandTriggerEquipInput.triggerInputHand = Hand::Left;
    ok &= expectTrue("left trigger edge equips a left-hand-held ROCK weapon", shouldRequestHeldWeaponEquip(leftHandTriggerEquipInput));
    auto newGrabEquipInput = equipInput;
    newGrabEquipInput.heldWeaponAtFrameStart = false;
    ok &= expectFalse("trigger edge during new grab does not equip weapon", shouldRequestHeldWeaponEquip(newGrabEquipInput));
    auto noHeldWeaponEquipInput = equipInput;
    noHeldWeaponEquipInput.heldWeaponNow = false;
    ok &= expectFalse("trigger edge without held weapon does not request equip", shouldRequestHeldWeaponEquip(noHeldWeaponEquipInput));
    auto menuEquipInput = equipInput;
    menuEquipInput.menuInputActive = true;
    ok &= expectFalse("menu input blocks held weapon equip request", shouldRequestHeldWeaponEquip(menuEquipInput));
    auto heldEquipInput = equipInput;
    heldEquipInput.triggerPressedEdge = false;
    ok &= expectFalse("held trigger does not repeat held weapon equip request", shouldRequestHeldWeaponEquip(heldEquipInput));
    auto gripZoneEquipInput = heldEquipInput;
    gripZoneEquipInput.gripZoneEquipEnabled = true;
    gripZoneEquipInput.gripZoneEquipSettled = true;
    ok &= expectTrue("palm settled in grip zone equips held weapon when enabled", shouldRequestHeldWeaponEquip(gripZoneEquipInput));
    auto disabledGripZoneEquipInput = gripZoneEquipInput;
    disabledGripZoneEquipInput.gripZoneEquipEnabled = false;
    ok &= expectFalse("grip zone equip disabled does not equip held weapon", shouldRequestHeldWeaponEquip(disabledGripZoneEquipInput));
    auto offhandGripZoneEquipInput = gripZoneEquipInput;
    offhandGripZoneEquipInput.heldWeaponHand = Hand::Left;
    ok &= expectTrue("left palm settled in grip zone equips its held weapon", shouldRequestHeldWeaponEquip(offhandGripZoneEquipInput));
    auto outsideGripZoneEquipInput = gripZoneEquipInput;
    outsideGripZoneEquipInput.gripZoneEquipSettled = false;
    ok &= expectFalse("palm outside grip zone does not equip held weapon", shouldRequestHeldWeaponEquip(outsideGripZoneEquipInput));

    ok &= expectTrue("enabled suppression requests native hook install", shouldInstallNativeActionSuppressionHook(true, true));
    ok &= expectFalse("disabled remap skips native hook install", shouldInstallNativeActionSuppressionHook(false, true));
    ok &= expectTrue("enabled remap installs mandatory Pip-Boy/Pause arbitration hooks", shouldInstallPipboyPauseArbitrationHooks(true));
    ok &= expectFalse("disabled remap leaves native Pip-Boy and Pause handlers untouched", shouldInstallPipboyPauseArbitrationHooks(false));
    ok &= expectTrue("manual scope installs the activate event hook independently of general remapping",
        shouldInstallActivateEventHook(false, true));
    ok &= expectTrue("manual scope installs raw controller capture independently of general remapping",
        shouldInstallRawControllerHooks(false, true));
    ok &= expectFalse("disabled remap and automatic scope need no raw controller hook",
        shouldInstallRawControllerHooks(false, false));

    nativeVats::RuntimeState nativeVatsState{};
    auto nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .buttonDown = true });
    ok &= expectTrue(
        "unsuppressed VATS-button down reaches native V.A.N.S.",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .released = true });
    ok &= expectTrue(
        "unsuppressed VATS-button release reaches ordinary VATS",
        nativeVatsDecision.forwardNative);

    nativeVats::reset(nativeVatsState);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .suppressVans = true,
        });
    ok &= expectFalse(
        "V.A.N.S.-only suppression consumes button-down samples",
        nativeVatsDecision.forwardNative);
    ok &= expectTrue(
        "V.A.N.S.-only suppression reports the held action",
        nativeVatsDecision.vansSuppressed);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .buttonDown = true });
    ok &= expectFalse(
        "expired V.A.N.S. lease stays latched through the gesture",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .released = true });
    ok &= expectTrue(
        "V.A.N.S.-only suppression preserves release-to-VATS",
        nativeVatsDecision.forwardNative);
    ok &= expectFalse(
        "V.A.N.S. release rearms its hold latch",
        nativeVatsState.suppressVansWhileDown);

    nativeVats::reset(nativeVatsState);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .suppressVats = true,
        });
    ok &= expectTrue(
        "VATS-only suppression preserves the native V.A.N.S. hold path",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .buttonDown = true });
    ok &= expectTrue(
        "expired VATS lease keeps forwarding V.A.N.S. while release remains armed",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .released = true });
    ok &= expectFalse(
        "VATS-only suppression consumes the ordinary release action",
        nativeVatsDecision.forwardNative);
    ok &= expectTrue(
        "VATS-only suppression reports the release action",
        nativeVatsDecision.vatsSuppressed);
    ok &= expectFalse(
        "suppressed VATS release rearms its release latch",
        nativeVatsState.suppressVatsOnRelease);

    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .released = true,
            .suppressVans = true,
        });
    ok &= expectTrue(
        "V.A.N.S. suppression acquired on release cannot block ordinary VATS",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .released = true,
            .suppressVats = true,
        });
    ok &= expectFalse(
        "VATS suppression acquired on release blocks that release",
        nativeVatsDecision.forwardNative);

    nativeVats::reset(nativeVatsState);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .suppressVats = true,
            .suppressVans = true,
        });
    ok &= expectFalse(
        "combined VATS and V.A.N.S. suppression consumes held samples",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .released = true });
    ok &= expectFalse(
        "combined suppression consumes the later release after both leases expire",
        nativeVatsDecision.forwardNative);

    nativeVats::reset(nativeVatsState);
    (void)nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .suppressVats = true,
            .suppressVans = true,
        });
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .justPressed = true,
        });
    ok &= expectTrue(
        "a newly observed press discards stale suppression from a lost release",
        nativeVatsDecision.forwardNative);
    ok &= expectFalse(
        "new-press rearming clears the stale VATS release latch",
        nativeVatsState.suppressVatsOnRelease);

    nativeVats::reset(nativeVatsState);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .suppressAll = true,
        });
    ok &= expectFalse(
        "broad OpenVR game-input suppression consumes V.A.N.S. hold samples",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{ .released = true });
    ok &= expectFalse(
        "broad OpenVR game-input suppression latches through VATS release",
        nativeVatsDecision.forwardNative);

    pipboyGesture::RuntimeState pipboyGestureState{};
    ok &= expectTrue("Pip-Boy/Pause hold duration clamps low values",
        pipboyGesture::sanitizedHoldSeconds(0.01f) == pipboyGesture::kMinimumHoldSeconds);
    ok &= expectTrue("Pip-Boy/Pause hold duration clamps high values",
        pipboyGesture::sanitizedHoldSeconds(8.0f) == pipboyGesture::kMaximumHoldSeconds);
    pipboyGesture::Input pipboyGestureInput{
        .enabled = true,
        .eligible = true,
        .holdSeconds = 0.35f,
    };

    pipboyGestureInput.pressed = true;
    pipboyGestureInput.held = true;
    auto pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectPipboyGestureState("Pause-button press starts tap/hold classification", pipboyGestureDecision.state, pipboyGesture::State::Pending);
    ok &= expectTrue("pending Pause-button press is consumed", pipboyGestureDecision.consume);
    ok &= expectFalse("initial Pause-button press does not open either menu", pipboyGestureDecision.dispatchPipboy || pipboyGestureDecision.dispatchPause);

    pipboyGestureInput.pressed = false;
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    pipboyGestureInput.heldSeconds = 0.12f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectTrue("short Pause-button release opens the Pip-Boy", pipboyGestureDecision.dispatchPipboy);
    ok &= expectFalse("short Pause-button release never opens Pause", pipboyGestureDecision.dispatchPause);
    ok &= expectPipboyGestureState("short release rearms the gesture", pipboyGestureDecision.state, pipboyGesture::State::Idle);

    pipboyGestureInput = pipboyGesture::Input{
        .enabled = true,
        .eligible = true,
        .pressed = true,
        .held = true,
        .holdSeconds = 0.35f,
    };
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    pipboyGestureInput.pressed = false;
    pipboyGestureInput.heldSeconds = 0.34f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectFalse("hold below threshold opens neither menu", pipboyGestureDecision.dispatchPipboy || pipboyGestureDecision.dispatchPause);
    pipboyGestureInput.heldSeconds = 0.35f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectTrue("hold threshold opens native Pause exactly once", pipboyGestureDecision.dispatchPause);
    ok &= expectPipboyGestureState("Pause hold commits the gesture", pipboyGestureDecision.state, pipboyGesture::State::PauseCommitted);
    pipboyGestureInput.heldSeconds = 0.60f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectFalse("continued Pause hold does not repeat menu dispatch", pipboyGestureDecision.dispatchPause);
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectFalse("release after Pause hold cannot open the Pip-Boy", pipboyGestureDecision.dispatchPipboy);
    ok &= expectPipboyGestureState("Pause-hold release rearms the gesture", pipboyGestureDecision.state, pipboyGesture::State::Idle);

    pipboyGestureInput = pipboyGesture::Input{
        .enabled = true,
        .eligible = true,
        .pressed = true,
        .held = true,
        .holdSeconds = 0.35f,
    };
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    pipboyGestureInput.pressed = false;
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    pipboyGestureInput.heldSeconds = 0.50f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectTrue("release beyond threshold still chooses Pause when no held sample crossed it", pipboyGestureDecision.dispatchPause);
    ok &= expectFalse("late release cannot fall back to Pip-Boy", pipboyGestureDecision.dispatchPipboy);

    pipboyGestureInput = pipboyGesture::Input{
        .enabled = true,
        .eligible = true,
        .pressed = true,
        .held = true,
        .pipboyDispatchAllowed = false,
        .holdSeconds = 0.35f,
    };
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    pipboyGestureInput.pressed = false;
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    pipboyGestureInput.heldSeconds = 0.10f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectFalse("provider game-input suppression blocks a short Pip-Boy tap", pipboyGestureDecision.dispatchPipboy);

    pipboyGestureInput = pipboyGesture::Input{
        .enabled = true,
        .eligible = true,
        .pressed = true,
        .held = true,
        .pipboyDispatchAllowed = false,
        .holdSeconds = 0.35f,
    };
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    pipboyGestureInput.pressed = false;
    pipboyGestureInput.heldSeconds = 0.35f;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectTrue("provider Pip-Boy suppression preserves the existing native Pause escape hold", pipboyGestureDecision.dispatchPause);
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);

    pipboyGestureInput = pipboyGesture::Input{
        .enabled = true,
        .eligible = true,
        .pressed = true,
        .held = true,
    };
    (void)pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    pipboyGestureInput.pressed = false;
    pipboyGestureInput.eligible = false;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectTrue("menu transition keeps ownership of an in-flight Pause gesture", pipboyGestureDecision.consume);
    ok &= expectPipboyGestureState("ineligible held gesture blocks until release", pipboyGestureDecision.state, pipboyGesture::State::BlockedUntilRelease);
    pipboyGestureInput.held = false;
    pipboyGestureInput.released = true;
    pipboyGestureDecision = pipboyGesture::update(pipboyGestureState, pipboyGestureInput);
    ok &= expectPipboyGestureState("blocked gesture rearms only on release", pipboyGestureDecision.state, pipboyGesture::State::Idle);

    manual::RuntimeState manualState{};
    manual::Input manualInput{
        .manualModeEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = true,
        .firingHandIsLeft = false,
        .leftButton = manual::ButtonState{ .available = true },
        .rightButton = manual::ButtonState{ .available = true },
        .holdSeconds = 0.30f,
    };

    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    auto manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("manual right-A press starts pending classification", manualDecision.state, manual::State::Pending);
    ok &= expectFalse("manual right-A press does not reload immediately", manualDecision.dispatchReload);
    ok &= expectFalse("manual right-A press does not scope immediately", manualDecision.scopeRequested);

    manualInput.rightButton.pressed = false;
    manualInput.deltaSeconds = 0.29f;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("manual hold remains pending below threshold", manualDecision.state, manual::State::Pending);
    ok &= expectFalse("manual hold below threshold remains unscoped", manualDecision.scopeRequested);

    manualInput.deltaSeconds = 0.02f;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("manual hold crosses into scope ownership", manualDecision.state, manual::State::ScopeHeld);
    ok &= expectTrue("manual hold requests scope after threshold", manualDecision.scopeRequested);
    ok &= expectFalse("scope hold never dispatches reload", manualDecision.dispatchReload);

    manualInput.deltaSeconds = 0.5f;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("manual scope remains requested for the entire physical hold", manualDecision.scopeRequested);

    manualInput.rightButton.held = false;
    manualInput.rightButton.released = true;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("manual scope release returns idle", manualDecision.state, manual::State::Idle);
    ok &= expectFalse("release after scope hold never reloads", manualDecision.dispatchReload);
    ok &= expectFalse("release ends manual scope request", manualDecision.scopeRequested);

    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualInput.deltaSeconds = 0.0f;
    (void)manual::update(manualState, manualInput);
    manualInput.rightButton = manual::ButtonState{ .available = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("release before hold threshold dispatches reload exactly once", manualDecision.dispatchReload);
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectFalse("held release edge cannot repeat manual reload", manualDecision.dispatchReload);

    manualInput.rightButton = manual::ButtonState{ .available = true, .pressed = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("press and release between frame polls still dispatches reload", manualDecision.dispatchReload);

    manual::reset(manualState);
    manualInput.firingHandIsLeft = true;
    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualInput.leftButton = manual::ButtonState{ .available = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("support-hand A does not start a left-firing gesture", manualDecision.state, manual::State::Idle);
    manualInput.rightButton = manual::ButtonState{ .available = true };
    manualInput.leftButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("left-firing X starts the same pending gesture", manualDecision.state, manual::State::Pending);

    manualInput.leftButton.pressed = false;
    manualInput.firingHandIsLeft = false;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("firing-hand change invalidates the bound gesture", manualDecision.state, manual::State::BlockedUntilRelease);
    ok &= expectFalse("firing-hand change cannot convert a pending hold into reload", manualDecision.dispatchReload);
    manualInput.leftButton.held = false;
    manualInput.leftButton.released = true;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("all accept buttons up rearm after a hand change", manualDecision.state, manual::State::Idle);

    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualInput.leftButton = manual::ButtonState{ .available = true };
    (void)manual::update(manualState, manualInput);
    manualInput.rightButton.pressed = false;
    manualInput.menuInputActive = true;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("blocking menu cancels and blocks a held gesture", manualDecision.state, manual::State::BlockedUntilRelease);
    manualInput.menuInputActive = false;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("held-through-menu input stays blocked", manualDecision.state, manual::State::BlockedUntilRelease);
    manualInput.rightButton.held = false;
    manualInput.rightButton.released = true;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectFalse("menu-cancelled gesture cannot replay as reload", manualDecision.dispatchReload);

    manualInput.manualModeEnabled = false;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("automatic mode clears all manual gesture state", manualDecision.state, manual::State::Idle);

    return ok ? 0 : 1;
}
