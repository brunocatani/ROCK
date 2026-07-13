#include "physics-interaction/input/InputRemapPolicy.h"

#include <cstdio>

namespace
{
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

    PhysicalButtonHandInput physicalButtonHand{
        .leftAvailable = true,
        .leftHeld = true,
        .rightAvailable = true,
        .rightHeld = false,
    };
    ok &= expectTrue("exclusive physical left button resolves to left",
        resolvePhysicalButtonHand(physicalButtonHand) == PhysicalButtonHandResolution::Left);
    physicalButtonHand.leftHeld = false;
    physicalButtonHand.rightHeld = true;
    ok &= expectTrue("exclusive physical right button resolves to right",
        resolvePhysicalButtonHand(physicalButtonHand) == PhysicalButtonHandResolution::Right);
    physicalButtonHand.leftHeld = true;
    ok &= expectTrue("simultaneous physical buttons fail closed",
        resolvePhysicalButtonHand(physicalButtonHand) == PhysicalButtonHandResolution::Unresolved);
    physicalButtonHand.leftHeld = false;
    physicalButtonHand.rightHeld = false;
    ok &= expectTrue("no held physical button fails closed",
        resolvePhysicalButtonHand(physicalButtonHand) == PhysicalButtonHandResolution::Unresolved);
    physicalButtonHand.leftAvailable = false;
    physicalButtonHand.rightHeld = true;
    ok &= expectTrue("missing controller snapshot fails closed",
        resolvePhysicalButtonHand(physicalButtonHand) == PhysicalButtonHandResolution::Unresolved);

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
    auto primaryDetachGrip = drawnGrip;
    primaryDetachGrip.equippedWeaponPrimaryDetachInputActive = true;
    ok &= expectTrue("primary detach input suppresses drawn native grip ready action", shouldSuppressNativeGripReadyAction(primaryDetachGrip));
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
        .eventHandResolved = true,
        .eventHand = Hand::Right,
        .firingHand = Hand::Right,
        .buttonJustPressed = true,
        .eventMatched = true,
    };
    ok &= expectTrue("right A routes reload while the right hand owns the firing grip", shouldRouteFiringHandActivateReload(activateReload));
    auto heldActivateReload = activateReload;
    heldActivateReload.buttonJustPressed = false;
    ok &= expectFalse("held firing-hand activate does not repeat reload", shouldRouteFiringHandActivateReload(heldActivateReload));
    auto leftXWhileRightFiring = activateReload;
    leftXWhileRightFiring.eventHand = Hand::Left;
    ok &= expectFalse("left X cannot reload while the right hand owns the firing grip", shouldRouteFiringHandActivateReload(leftXWhileRightFiring));
    auto leftFiringActivateReload = activateReload;
    leftFiringActivateReload.eventHand = Hand::Left;
    leftFiringActivateReload.firingHand = Hand::Left;
    ok &= expectTrue("left X routes reload while the left hand owns the firing grip", shouldRouteFiringHandActivateReload(leftFiringActivateReload));
    auto unresolvedLeftFiringActivateReload = leftFiringActivateReload;
    unresolvedLeftFiringActivateReload.eventHandResolved = false;
    ok &= expectFalse("unresolved physical controller cannot reload left firing grip", shouldRouteFiringHandActivateReload(unresolvedLeftFiringActivateReload));
    auto rightAWhileLeftFiring = leftFiringActivateReload;
    rightAWhileLeftFiring.eventHand = Hand::Right;
    ok &= expectFalse("right A cannot reload while the left hand owns the firing grip", shouldRouteFiringHandActivateReload(rightAWhileLeftFiring));
    auto holsteredActivateReload = activateReload;
    holsteredActivateReload.weaponDrawn = false;
    ok &= expectFalse("holstered firing-hand activate does not route reload", shouldRouteFiringHandActivateReload(holsteredActivateReload));
    auto menuActivateReload = activateReload;
    menuActivateReload.menuInputActive = true;
    ok &= expectFalse("menu input blocks firing-hand activate reload", shouldRouteFiringHandActivateReload(menuActivateReload));
    auto unmatchedActivateReload = activateReload;
    unmatchedActivateReload.eventMatched = false;
    ok &= expectFalse("unmatched activate event does not route reload", shouldRouteFiringHandActivateReload(unmatchedActivateReload));
    auto virtualHolstersActivateReload = activateReload;
    virtualHolstersActivateReload.virtualHolstersOwnsInput = true;
    ok &= expectFalse("VirtualHolsters zone ownership blocks firing-hand activate reload", shouldRouteFiringHandActivateReload(virtualHolstersActivateReload));

    EquippedWeaponPrimaryDetachInputGate primaryDetachGate{
        .featureAvailable = true,
        .canUsePrimaryDetachInput = false,
        .virtualHolstersOwnsInput = false,
    };
    ok &= expectTrue("primary detach feature consumes stale edges before armed", shouldConsumeEquippedWeaponPrimaryDetachInput(primaryDetachGate));
    ok &= expectFalse("primary detach ignores consumed edges until armed", shouldUseEquippedWeaponPrimaryDetachInput(primaryDetachGate));
    primaryDetachGate.canUsePrimaryDetachInput = true;
    ok &= expectTrue("primary detach uses fresh edge after armed", shouldUseEquippedWeaponPrimaryDetachInput(primaryDetachGate));
    primaryDetachGate.menuInputActive = true;
    ok &= expectFalse("menu input blocks primary detach edge use", shouldUseEquippedWeaponPrimaryDetachInput(primaryDetachGate));
    primaryDetachGate.menuInputActive = false;
    primaryDetachGate.virtualHolstersOwnsInput = true;
    ok &= expectFalse("VirtualHolsters ownership blocks primary detach edge use", shouldUseEquippedWeaponPrimaryDetachInput(primaryDetachGate));
    primaryDetachGate.featureAvailable = false;
    primaryDetachGate.virtualHolstersOwnsInput = false;
    ok &= expectFalse("missing hFRIK blocker export disables primary detach consumption", shouldConsumeEquippedWeaponPrimaryDetachInput(primaryDetachGate));

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
    auto autoEquipInput = heldEquipInput;
    autoEquipInput.autoEquipEnabled = true;
    autoEquipInput.autoEquipSettled = true;
    ok &= expectTrue("settled held weapon can auto-equip when enabled", shouldRequestHeldWeaponEquip(autoEquipInput));
    auto disabledAutoEquipInput = autoEquipInput;
    disabledAutoEquipInput.autoEquipEnabled = false;
    ok &= expectFalse("settled held weapon does not auto-equip when disabled", shouldRequestHeldWeaponEquip(disabledAutoEquipInput));
    auto offhandAutoEquipInput = autoEquipInput;
    offhandAutoEquipInput.legacyAutoEquipPrimaryHand = false;
    ok &= expectFalse("settled offhand held weapon does not auto-equip", shouldRequestHeldWeaponEquip(offhandAutoEquipInput));
    auto unsettledAutoEquipInput = autoEquipInput;
    unsettledAutoEquipInput.autoEquipSettled = false;
    ok &= expectFalse("unsettled held weapon does not auto-equip", shouldRequestHeldWeaponEquip(unsettledAutoEquipInput));
    auto gripZoneEquipInput = heldEquipInput;
    gripZoneEquipInput.gripZoneEquipEnabled = true;
    gripZoneEquipInput.gripZoneEquipSettled = true;
    ok &= expectTrue("palm settled in grip zone equips held weapon when enabled", shouldRequestHeldWeaponEquip(gripZoneEquipInput));
    auto disabledGripZoneEquipInput = gripZoneEquipInput;
    disabledGripZoneEquipInput.gripZoneEquipEnabled = false;
    ok &= expectFalse("grip zone equip disabled does not equip held weapon", shouldRequestHeldWeaponEquip(disabledGripZoneEquipInput));
    auto offhandGripZoneEquipInput = gripZoneEquipInput;
    offhandGripZoneEquipInput.legacyAutoEquipPrimaryHand = false;
    offhandGripZoneEquipInput.heldWeaponHand = Hand::Left;
    ok &= expectTrue("left palm settled in grip zone equips its held weapon", shouldRequestHeldWeaponEquip(offhandGripZoneEquipInput));
    auto outsideGripZoneEquipInput = gripZoneEquipInput;
    outsideGripZoneEquipInput.gripZoneEquipSettled = false;
    ok &= expectFalse("palm outside grip zone does not equip held weapon", shouldRequestHeldWeaponEquip(outsideGripZoneEquipInput));

    ok &= expectTrue("enabled suppression requests native hook install", shouldInstallNativeActionSuppressionHook(true, true));
    ok &= expectFalse("disabled remap skips native hook install", shouldInstallNativeActionSuppressionHook(false, true));

    VirtualHolstersCompatibilityInput virtualHolsters{
        .compatibilityEnabled = true,
        .deferActionEnabled = true,
        .deferOnlyMatchingButton = true,
        .apiAvailable = true,
        .initialized = true,
        .handInZone = true,
        .rockButtonId = 2,
        .holsterButtonId = 2,
    };
    ok &= expectTrue("VirtualHolsters zone defers matching ROCK button", shouldDeferVirtualHolstersInput(virtualHolsters));

    auto realisticWeaponHandlingVirtualHolsters = virtualHolsters;
    realisticWeaponHandlingVirtualHolsters.realisticWeaponHandlingEnabled = true;
    ok &= expectFalse("realistic weapon handling disables VirtualHolsters input deferral", shouldDeferVirtualHolstersInput(realisticWeaponHandlingVirtualHolsters));

    auto unmatchedVirtualHolsters = virtualHolsters;
    unmatchedVirtualHolsters.holsterButtonId = 7;
    ok &= expectFalse("VirtualHolsters match-only mode allows unrelated ROCK button", shouldDeferVirtualHolstersInput(unmatchedVirtualHolsters));

    auto broadVirtualHolsters = unmatchedVirtualHolsters;
    broadVirtualHolsters.deferOnlyMatchingButton = false;
    ok &= expectTrue("VirtualHolsters broad mode defers unrelated ROCK button", shouldDeferVirtualHolstersInput(broadVirtualHolsters));

    auto unknownBroadVirtualHolsters = broadVirtualHolsters;
    unknownBroadVirtualHolsters.rockButtonId = -1;
    ok &= expectTrue("VirtualHolsters broad mode defers unknown native button", shouldDeferVirtualHolstersInput(unknownBroadVirtualHolsters));

    auto unknownMatchingVirtualHolsters = unknownBroadVirtualHolsters;
    unknownMatchingVirtualHolsters.deferOnlyMatchingButton = true;
    ok &= expectFalse("VirtualHolsters match-only mode requires known matching button", shouldDeferVirtualHolstersInput(unknownMatchingVirtualHolsters));

    auto inactiveVirtualHolsters = virtualHolsters;
    inactiveVirtualHolsters.handInZone = false;
    ok &= expectFalse("VirtualHolsters outside zone does not defer ROCK input", shouldDeferVirtualHolstersInput(inactiveVirtualHolsters));

    return ok ? 0 : 1;
}
