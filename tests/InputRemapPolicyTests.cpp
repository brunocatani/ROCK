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

    const auto weaponToggleMask = buttonMask(settings.weaponToggleButtonId);
    const auto toggleDecision = evaluate(Input{
                                             .hand = Hand::Right,
                                             .gameplayInputAllowed = true,
                                             .menuInputActive = false,
                                             .weaponDrawn = false,
                                             .rawPressed = weaponToggleMask,
                                             .previousRawPressed = 0,
                                         },
        settings);
    ok &= expectTrue("right thumbstick edge requests weapon toggle", toggleDecision.weaponToggleRequested);

    const auto heldToggleDecision = evaluate(Input{
                                                 .hand = Hand::Right,
                                                 .gameplayInputAllowed = true,
                                                 .menuInputActive = false,
                                                 .weaponDrawn = false,
                                                 .rawPressed = weaponToggleMask,
                                                 .previousRawPressed = weaponToggleMask,
                                             },
        settings);
    ok &= expectFalse("held right thumbstick does not repeat weapon toggle", heldToggleDecision.weaponToggleRequested);

    const auto menuToggleDecision = evaluate(Input{
                                                 .hand = Hand::Right,
                                                 .gameplayInputAllowed = true,
                                                 .menuInputActive = true,
                                                 .weaponDrawn = false,
                                                 .rawPressed = weaponToggleMask,
                                                 .previousRawPressed = 0,
                                             },
        settings);
    ok &= expectFalse("menu input blocks weapon toggle request", menuToggleDecision.weaponToggleRequested);

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

    ok &= expectTrue("holstered WandTrigger suppresses native attack gate", shouldSuppressNativeTriggerAction(base));
    auto drawnTrigger = base;
    drawnTrigger.weaponDrawn = true;
    ok &= expectFalse("drawn weapon allows native trigger attack gate", shouldSuppressNativeTriggerAction(drawnTrigger));
    auto heldWeaponTrigger = drawnTrigger;
    heldWeaponTrigger.rightHandHeldWeapon = true;
    ok &= expectTrue("right held ROCK weapon suppresses native trigger even if weapon drawn", shouldSuppressNativeTriggerAction(heldWeaponTrigger));

    auto favorites = base;
    favorites.weaponDrawn = true;
    ok &= expectTrue("WandThumbClick suppresses native favorites even with weapon drawn", shouldSuppressNativeFavoritesAction(favorites));

    auto meleeThrow = base;
    meleeThrow.weaponDrawn = true;
    ok &= expectTrue("WandGrip suppresses native melee throw even with weapon drawn", shouldSuppressNativeMeleeThrowAction(meleeThrow));

    auto menuFavorites = favorites;
    menuFavorites.menuInputActive = true;
    ok &= expectFalse("menu input allows native favorites handling", shouldSuppressNativeFavoritesAction(menuFavorites));

    auto menuMeleeThrow = meleeThrow;
    menuMeleeThrow.menuInputActive = true;
    ok &= expectFalse("menu input allows native melee throw handling", shouldSuppressNativeMeleeThrowAction(menuMeleeThrow));

    auto unmatched = base;
    unmatched.eventMatched = false;
    ok &= expectFalse("unmatched native event is not suppressed", shouldSuppressNativeTriggerAction(unmatched));

    HeldWeaponEquipInput equipInput{
        .remapEnabled = true,
        .gameplayInputAllowed = true,
        .menuInputActive = false,
        .hand = Hand::Right,
        .rightHandHeldWeapon = true,
        .pressedEdge = true,
    };
    ok &= expectTrue("right trigger edge equips right held ROCK weapon", shouldRequestRightHeldWeaponEquip(equipInput));
    auto leftEquipInput = equipInput;
    leftEquipInput.hand = Hand::Left;
    ok &= expectFalse("left hand cannot request right held weapon equip", shouldRequestRightHeldWeaponEquip(leftEquipInput));
    auto noHeldWeaponEquipInput = equipInput;
    noHeldWeaponEquipInput.rightHandHeldWeapon = false;
    ok &= expectFalse("trigger edge without right held weapon does not request equip", shouldRequestRightHeldWeaponEquip(noHeldWeaponEquipInput));
    auto menuEquipInput = equipInput;
    menuEquipInput.menuInputActive = true;
    ok &= expectFalse("menu input blocks right held weapon equip request", shouldRequestRightHeldWeaponEquip(menuEquipInput));
    auto heldEquipInput = equipInput;
    heldEquipInput.pressedEdge = false;
    ok &= expectFalse("held trigger does not repeat right held weapon equip request", shouldRequestRightHeldWeaponEquip(heldEquipInput));

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

    auto unmatchedVirtualHolsters = virtualHolsters;
    unmatchedVirtualHolsters.holsterButtonId = 7;
    ok &= expectFalse("VirtualHolsters match-only mode allows unrelated ROCK button", shouldDeferVirtualHolstersInput(unmatchedVirtualHolsters));

    auto broadVirtualHolsters = unmatchedVirtualHolsters;
    broadVirtualHolsters.deferOnlyMatchingButton = false;
    ok &= expectTrue("VirtualHolsters broad mode defers unrelated ROCK button", shouldDeferVirtualHolstersInput(broadVirtualHolsters));

    auto inactiveVirtualHolsters = virtualHolsters;
    inactiveVirtualHolsters.handInZone = false;
    ok &= expectFalse("VirtualHolsters outside zone does not defer ROCK input", shouldDeferVirtualHolstersInput(inactiveVirtualHolsters));

    return ok ? 0 : 1;
}
