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
    settings.weaponToggleButtonId = kOpenVrAxisButtonBase;

    ok &= expectTrue("normal grab button id is accepted", isAllowedGrabButtonId(2));
    ok &= expectFalse("SteamVR trigger button id is reserved and rejected for grab", isAllowedGrabButtonId(kOpenVrSteamVrTriggerButtonId));

    WeaponToggleClickState toggleState{};
    auto togglePress = updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = true,
            .pressed = true,
            .released = false,
            .currentTimeSeconds = 10.0,
        });
    ok &= expectFalse("right thumbstick press waits for click release", togglePress.weaponToggleRequested);
    auto toggleRelease = updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = false,
            .pressed = false,
            .released = true,
            .currentTimeSeconds = 10.10,
        });
    ok &= expectTrue("short right thumbstick click release requests weapon toggle", toggleRelease.weaponToggleRequested);

    toggleState = {};
    (void)updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = true,
            .pressed = true,
            .released = false,
            .currentTimeSeconds = 20.0,
        });
    auto heldToggleRelease = updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = false,
            .pressed = false,
            .released = true,
            .currentTimeSeconds = 20.50,
        });
    ok &= expectFalse("held right thumbstick does not become weapon toggle", heldToggleRelease.weaponToggleRequested);

    toggleState = {};
    (void)updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = true,
            .pressed = true,
            .released = false,
            .currentTimeSeconds = 30.0,
        });
    (void)updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = true,
            .rightHand = true,
            .held = true,
            .pressed = false,
            .released = false,
            .currentTimeSeconds = 30.10,
        });
    auto menuToggleRelease = updateWeaponToggleClick(toggleState,
        WeaponToggleClickInput{
            .enabled = settings.enabled,
            .gameplayInputAllowed = true,
            .menuInputActive = false,
            .rightHand = true,
            .held = false,
            .pressed = false,
            .released = true,
            .currentTimeSeconds = 30.15,
        });
    ok &= expectFalse("menu opened during thumbstick hold blocks weapon toggle", menuToggleRelease.weaponToggleRequested);

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
    heldWeaponTrigger.rightHandHeldWeapon = true;
    ok &= expectTrue("right held ROCK weapon suppresses native trigger even if weapon drawn", shouldSuppressNativeTriggerAction(heldWeaponTrigger));
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

    NativeActivateReloadInput activateReload{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .weaponDrawn = true,
        .primaryHandEvent = true,
        .buttonJustPressed = true,
        .eventMatched = true,
    };
    ok &= expectTrue("primary activate edge routes to reload while weapon drawn", shouldRoutePrimaryActivateReload(activateReload));
    auto heldActivateReload = activateReload;
    heldActivateReload.buttonJustPressed = false;
    ok &= expectFalse("held primary activate does not repeat reload", shouldRoutePrimaryActivateReload(heldActivateReload));
    auto offhandActivateReload = activateReload;
    offhandActivateReload.primaryHandEvent = false;
    ok &= expectFalse("offhand activate is left for normal use", shouldRoutePrimaryActivateReload(offhandActivateReload));
    auto holsteredActivateReload = activateReload;
    holsteredActivateReload.weaponDrawn = false;
    ok &= expectFalse("holstered primary activate does not route reload", shouldRoutePrimaryActivateReload(holsteredActivateReload));
    auto menuActivateReload = activateReload;
    menuActivateReload.menuInputActive = true;
    ok &= expectFalse("menu input blocks primary activate reload", shouldRoutePrimaryActivateReload(menuActivateReload));
    auto unmatchedActivateReload = activateReload;
    unmatchedActivateReload.eventMatched = false;
    ok &= expectFalse("unmatched activate event does not route reload", shouldRoutePrimaryActivateReload(unmatchedActivateReload));
    auto virtualHolstersActivateReload = activateReload;
    virtualHolstersActivateReload.virtualHolstersOwnsInput = true;
    ok &= expectFalse("VirtualHolsters zone ownership blocks primary activate reload", shouldRoutePrimaryActivateReload(virtualHolstersActivateReload));

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
        .sameHandTriggerPressedEdge = true,
    };
    ok &= expectTrue("same-hand trigger edge equips already held ROCK weapon", shouldRequestHeldWeaponEquip(equipInput));
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
    heldEquipInput.sameHandTriggerPressedEdge = false;
    ok &= expectFalse("held trigger does not repeat held weapon equip request", shouldRequestHeldWeaponEquip(heldEquipInput));
    auto autoEquipInput = heldEquipInput;
    autoEquipInput.autoEquipEnabled = true;
    autoEquipInput.autoEquipSettled = true;
    ok &= expectTrue("settled held weapon can auto-equip when enabled", shouldRequestHeldWeaponEquip(autoEquipInput));
    auto disabledAutoEquipInput = autoEquipInput;
    disabledAutoEquipInput.autoEquipEnabled = false;
    ok &= expectFalse("settled held weapon does not auto-equip when disabled", shouldRequestHeldWeaponEquip(disabledAutoEquipInput));
    auto offhandAutoEquipInput = autoEquipInput;
    offhandAutoEquipInput.primaryHand = false;
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
    offhandGripZoneEquipInput.primaryHand = false;
    ok &= expectFalse("offhand palm in grip zone does not equip held weapon", shouldRequestHeldWeaponEquip(offhandGripZoneEquipInput));
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
