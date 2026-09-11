#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/ManualScopeInputPolicy.h"
#include "physics-interaction/input/NativeVatsInputSuppressionPolicy.h"
#include "physics-interaction/input/VatsGrenadeGesturePolicy.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"

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

static bool testInputRouting()
{
    using namespace rock::input_remap_policy;
    bool ok = true;
    ok &= expectTrue("consumer owns gameplay input", providerSuppressionApplies(false, true));
    ok &= expectFalse("native menu bypasses stale consumer lease", providerSuppressionApplies(true, true));
    ok &= expectFalse("no lease cannot suppress gameplay", providerSuppressionApplies(false, false));
    ok &= expectFalse("native menu without lease remains native", providerSuppressionApplies(true, false));

    namespace grenade = rock::vats_grenade_gesture_policy;
    grenade::RuntimeState grenadeState{};
    grenade::Input grenadeInput{ .pressed = true, .held = true };
    ok &= expectFalse("grenade mode leaves B press pending", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .released = true, .heldSeconds = 0.1f };
    ok &= expectFalse("grenade mode leaves short B tap native", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .pressed = true, .held = true };
    (void)grenade::update(grenadeState, grenadeInput);
    grenadeInput = { .held = true, .heldSeconds = 0.25f };
    ok &= expectTrue("grenade mode draws at hold threshold", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput.heldSeconds = 2.0f;
    ok &= expectFalse("continued B hold cannot draw a second grenade", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .released = true, .heldSeconds = 2.0f };
    ok &= expectFalse("release after draw cannot repeat it", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .pressed = true, .held = true };
    (void)grenade::update(grenadeState, grenadeInput);
    grenadeInput = { .released = true, .heldSeconds = 0.3f };
    ok &= expectTrue("late release observes a missed threshold exactly once", grenade::update(grenadeState, grenadeInput).requestGrenade);

    // A wheel claim, native menu or provider loss cancels a pending gesture.
    grenadeInput = { .pressed = true, .held = true };
    (void)grenade::update(grenadeState, grenadeInput);
    grenadeInput = { .eligible = false, .held = true, .heldSeconds = 0.1f };
    ok &= expectFalse("wheel readiness cancels pending grenade mode", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .held = true, .heldSeconds = 0.5f };
    ok &= expectFalse("claim expiry cannot reuse held B", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .released = true, .heldSeconds = 0.6f };
    ok &= expectFalse("claim expiry cannot draw on old release", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .pressed = true, .held = true };
    (void)grenade::update(grenadeState, grenadeInput);
    grenadeInput = { .held = true, .heldSeconds = 0.25f };
    ok &= expectTrue("fresh B hold draws after claim release", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenade::reset(grenadeState);
    grenadeInput = { .held = true, .heldSeconds = 1.0f };
    ok &= expectFalse("B already held on load cannot draw", grenade::update(grenadeState, grenadeInput).requestGrenade);
    grenadeInput = { .eligible = false, .pressed = true, .held = true, .heldSeconds = 0.4f };
    ok &= expectFalse("disabled grenade mode cannot draw even on a late press", grenade::update(grenadeState, grenadeInput).requestGrenade);

    Settings settings{};

    ok &= expectTrue("fixed grab button is OpenVR grip", kGrabButtonId == 2);
    ok &= expectTrue("fixed grab button id is accepted", isAllowedGrabButtonId(kGrabButtonId));
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
    settings.grabButtonId = kGrabButtonId;

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
    auto mandatoryFavorites = favorites;
    mandatoryFavorites.remapEnabled = false;
    mandatoryFavorites.suppressionEnabled = false;
    mandatoryFavorites.gameplayInputAllowed = false;
    mandatoryFavorites.menuInputActive = true;
    ok &= expectTrue("native favorites suppression cannot be disabled by optional input gates", shouldSuppressNativeFavoritesAction(mandatoryFavorites));
    mandatoryFavorites.eventMatched = false;
    ok &= expectFalse("mandatory favorites suppression leaves unrelated events untouched", shouldSuppressNativeFavoritesAction(mandatoryFavorites));

    auto meleeThrow = base;
    meleeThrow.weaponDrawn = true;
    ok &= expectTrue("non-melee WandGrip suppresses native melee throw even with weapon drawn", shouldSuppressNativeMeleeThrowAction(meleeThrow));

    auto nativeHolsteredMelee = base;
    nativeHolsteredMelee.realMeleeWeaponEquipped = true;
    ok &= expectTrue("native-mode real melee still blocks holstered grip ready handling", shouldSuppressNativeGripReadyAction(nativeHolsteredMelee));
    ok &= expectTrue("native-mode real melee still blocks holstered trigger handling", shouldSuppressNativeTriggerAction(nativeHolsteredMelee));
    ok &= expectTrue("native-mode real melee still blocks native grip grenade handling", shouldSuppressNativeMeleeThrowAction(nativeHolsteredMelee));

    auto nativeDrawnMelee = drawnGrip;
    nativeDrawnMelee.realMeleeWeaponEquipped = true;
    ok &= expectFalse("native-mode real melee keeps drawn grip reload handling", shouldSuppressNativeGripReloadAction(nativeDrawnMelee));
    ok &= expectFalse("native-mode real melee keeps drawn trigger handling", shouldSuppressNativeTriggerAction(nativeDrawnMelee));
    ok &= expectTrue("ROCK grab always blocks native grip grenade handling", shouldSuppressNativeMeleeThrowAction(nativeDrawnMelee));

    auto shoulderStashedMeleeTransition = nativeDrawnMelee;
    shoulderStashedMeleeTransition.equippedWeaponShoulderSheathActive = true;
    ok &= expectTrue("shoulder-stashed melee transition blocks native grip ready handling", shouldSuppressNativeGripReadyAction(shoulderStashedMeleeTransition));
    ok &= expectTrue("shoulder-stashed melee transition blocks native trigger handling", shouldSuppressNativeTriggerAction(shoulderStashedMeleeTransition));
    ok &= expectTrue("shoulder-stashed melee transition blocks native grip reload handling", shouldSuppressNativeGripReloadAction(shoulderStashedMeleeTransition));

    auto suppressedHolsteredMelee = nativeHolsteredMelee;
    suppressedHolsteredMelee.nativeMeleeSuppressionActive = true;
    ok &= expectTrue("suppressed real melee claims holstered grip ready handling", shouldSuppressNativeGripReadyAction(suppressedHolsteredMelee));

    auto suppressedDrawnMelee = nativeDrawnMelee;
    suppressedDrawnMelee.nativeMeleeSuppressionActive = true;
    ok &= expectTrue("suppressed real melee claims drawn grip reload handling", shouldSuppressNativeGripReloadAction(suppressedDrawnMelee));
    ok &= expectTrue("suppressed real melee claims melee throw handling", shouldSuppressNativeMeleeThrowAction(suppressedDrawnMelee));

    auto menuFavorites = favorites;
    menuFavorites.menuInputActive = true;
    ok &= expectTrue("menu input still suppresses native favorites handling", shouldSuppressNativeFavoritesAction(menuFavorites));

    auto menuMeleeThrow = meleeThrow;
    menuMeleeThrow.menuInputActive = true;
    ok &= expectFalse("menu input allows native melee throw handling", shouldSuppressNativeMeleeThrowAction(menuMeleeThrow));

    auto unmatched = base;
    unmatched.eventMatched = false;
    ok &= expectFalse("unmatched native event is not suppressed", shouldSuppressNativeTriggerAction(unmatched));

    LegacyPipboyTriggerOpenInput legacyPipboyTrigger{
        .remapEnabled = true,
        .pipboyMenuOpen = false,
        .eventMatched = true,
        .secondaryWandEvent = true,
    };
    ok &= expectTrue("secondary WandTrigger no longer opens the Pip-Boy during gameplay", shouldSuppressLegacyPipboyTriggerOpen(legacyPipboyTrigger));
    auto legacyPrimaryTrigger = legacyPipboyTrigger;
    legacyPrimaryTrigger.secondaryWandEvent = false;
    ok &= expectFalse("primary trigger remains available to native attack handling", shouldSuppressLegacyPipboyTriggerOpen(legacyPrimaryTrigger));
    auto legacyMenuTrigger = legacyPipboyTrigger;
    legacyMenuTrigger.pipboyMenuOpen = true;
    ok &= expectFalse("an already-open Pip-Boy retains native trigger controls", shouldSuppressLegacyPipboyTriggerOpen(legacyMenuTrigger));
    legacyMenuTrigger.pipboyMenuOpen = false;
    ok &= expectTrue("closing the Pip-Boy immediately removes trigger opening again", shouldSuppressLegacyPipboyTriggerOpen(legacyMenuTrigger));
    auto legacyDirectPipboy = legacyPipboyTrigger;
    legacyDirectPipboy.eventMatched = false;
    ok &= expectFalse("direct keyboard or gamepad Pipboy binding is not the moved VR trigger", shouldSuppressLegacyPipboyTriggerOpen(legacyDirectPipboy));

    for (const char* formType : { "WEAP", "ARMO", "AMMO", "MISC", "INGR", "ALCH", "BOOK", "KEYM", "SLGM" }) {
        ok &= expectTrue(formType, rock::far_selection_blacklist_policy::listContainsText(kNativeTakeEquipFormTypes, formType));
    }
    for (const char* formType : { "DOOR", "NPC_", "CONT", "TERM", "", "WEA" }) {
        ok &= expectFalse(formType, rock::far_selection_blacklist_policy::listContainsText(kNativeTakeEquipFormTypes, formType));
    }

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
    auto takeEquipMandatory = takeEquipHoldingEligible;
    takeEquipMandatory.suppressionEnabled = false;
    ok &= expectTrue("take/equip protection cannot be disabled", shouldSuppressNativeTakeEquipAction(takeEquipMandatory));
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
    auto missingRawCapture = manualActivate;
    missingRawCapture.rawInputCaptureAvailable = false;
    ok &= expectFalse("manual scope does not swallow reload when raw capture is unavailable",
        shouldDeferFiringHandActivateForManualScope(missingRawCapture));
    auto supportHandActivate = manualActivate;
    supportHandActivate.firingHandIsPrimaryHand = false;
    ok &= expectFalse("manual scope does not claim the support hand activate event",
        shouldDeferFiringHandActivateForManualScope(supportHandActivate));

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

    ok &= expectTrue("enabled remap installs mandatory Pip-Boy/Pause arbitration hooks", shouldInstallPipboyPauseArbitrationHooks(true));
    ok &= expectFalse("disabled remap leaves native Pip-Boy and Pause handlers untouched", shouldInstallPipboyPauseArbitrationHooks(false));
    return ok;
}

static bool testNativeVats()
{
    using namespace rock::input_remap_policy;
    bool ok = true;
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
            .justPressed = true,
            .heldSeconds = 0.0f,
            .suppressVans = true,
            .reserveHoldGesture = true,
        });
    ok &= expectFalse(
        "ROCK grenade gesture consumes native V.A.N.S. down phase",
        nativeVatsDecision.forwardNative);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .heldSeconds = nativeVats::kDefaultHoldSeconds,
        });
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .released = true,
            .heldSeconds = 0.40f,
        });
    ok &= expectFalse(
        "ROCK grenade hold consumes eventual VATS release",
        nativeVatsDecision.forwardNative);
    ok &= expectTrue(
        "ROCK grenade hold reports duration-owned release suppression",
        nativeVatsDecision.holdReleaseSuppressed);

    nativeVats::reset(nativeVatsState);
    (void)nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .buttonDown = true,
            .justPressed = true,
            .heldSeconds = 0.0f,
            .suppressVans = true,
            .reserveHoldGesture = true,
        });
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .released = true,
            .heldSeconds = 0.10f,
        });
    ok &= expectTrue(
        "ROCK short tap preserves release-to-VATS",
        nativeVatsDecision.forwardNative);
    ok &= expectFalse(
        "ROCK short tap is not classified as held release",
        nativeVatsDecision.holdReleaseSuppressed);

    nativeVats::reset(nativeVatsState);
    nativeVatsDecision = nativeVats::update(
        nativeVatsState,
        nativeVats::Input{
            .released = true,
            .heldSeconds = 0.40f,
            .reserveHoldGesture = true,
        });
    ok &= expectFalse(
        "long release suppresses VATS even when down sample was hidden",
        nativeVatsDecision.forwardNative);

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

    return ok;
}

static bool testPipboyGestures()
{
    using namespace rock::input_remap_policy;
    bool ok = true;
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

    using PipboyRoute = pipboyGesture::PipboyRoute;
    ok &= expectTrue("wrist presentation routes through FRIK", pipboyGesture::selectPipboyRoute(true, false, false, false, true) == PipboyRoute::FrikWrist);
    ok &= expectTrue("projected presentation routes through native handler", pipboyGesture::selectPipboyRoute(true, true, false, false, true) == PipboyRoute::Native);
    ok &= expectTrue("HMD presentation routes through native handler", pipboyGesture::selectPipboyRoute(true, false, true, false, true) == PipboyRoute::Native);
    ok &= expectTrue("power armor uses native handler even with wrist preference", pipboyGesture::selectPipboyRoute(true, false, false, true, true) == PipboyRoute::Native);
    ok &= expectTrue("missing presentation settings cannot fall back to native trigger", pipboyGesture::selectPipboyRoute(false, false, false, false, true) == PipboyRoute::Unavailable);
    ok &= expectTrue("missing FRIK binding cannot bypass wrist screen owner", pipboyGesture::selectPipboyRoute(true, false, false, false, false) == PipboyRoute::Unavailable);
    ok &= expectTrue("native presentation does not require wrist binding", pipboyGesture::selectPipboyRoute(true, true, false, false, false) == PipboyRoute::Native);

    // Simulate the native menu consuming the previous gesture's release.
    // The next real down edge must work on its first tap, without a trigger.
    for (const auto staleState : { pipboyGesture::State::Pending, pipboyGesture::State::PauseCommitted, pipboyGesture::State::BlockedUntilRelease }) {
        pipboyGestureState.state = staleState;
        auto recovered = pipboyGesture::update(pipboyGestureState,
            pipboyGesture::Input{ .pressed = true, .held = true });
        ok &= expectPipboyGestureState("fresh Y press replaces gesture whose release was consumed", recovered.state, pipboyGesture::State::Pending);
        ok &= expectFalse("recovery cannot dispatch either menu on press", recovered.dispatchPipboy || recovered.dispatchPause);
        recovered = pipboyGesture::update(pipboyGestureState,
            pipboyGesture::Input{ .released = true, .heldSeconds = 0.1f });
        ok &= expectTrue("first fresh Y tap opens after a lost release", recovered.dispatchPipboy);
        ok &= expectFalse("first fresh Y tap after lost release does not open Pause", recovered.dispatchPause);
    }
    pipboyGestureState.state = pipboyGesture::State::BlockedUntilRelease;
    auto stillBlocked = pipboyGesture::update(pipboyGestureState,
        pipboyGesture::Input{ .held = true, .heldSeconds = 0.1f });
    ok &= expectPipboyGestureState("holding Y across a menu does not count as a fresh press", stillBlocked.state, pipboyGesture::State::BlockedUntilRelease);
    stillBlocked = pipboyGesture::update(pipboyGestureState,
        pipboyGesture::Input{ .released = true, .heldSeconds = 0.2f });
    ok &= expectFalse("held-through-menu release cannot open either menu", stillBlocked.dispatchPipboy || stillBlocked.dispatchPause);
    pipboyGestureState.state = pipboyGesture::State::PauseCommitted;
    stillBlocked = pipboyGesture::update(pipboyGestureState,
        pipboyGesture::Input{ .eligible = false, .pressed = true, .held = true });
    ok &= expectPipboyGestureState("fresh press cannot bypass a currently blocking menu", stillBlocked.state, pipboyGesture::State::BlockedUntilRelease);

    return ok;
}

static bool testManualScope()
{
    using namespace rock::input_remap_policy;
    bool ok = true;
    manual::RuntimeState manualState{};
    manual::Input manualInput{
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

    manual::reset(manualState);
    manualInput.nativeActivationTarget = true;
    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("raw press on a use target belongs to native activation", manualDecision.state, manual::State::NativeActivation);
    ok &= expectFalse("use target press cannot reload or scope", manualDecision.dispatchReload || manualDecision.scopeRequested);
    manualInput.nativeActivationTarget = false;
    manualInput.rightButton.pressed = false;
    manualInput.deltaSeconds = 1.0f;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("looking away retains native activation ownership through a long hold", manualDecision.state, manual::State::NativeActivation);
    ok &= expectFalse("native activation hold cannot become scope", manualDecision.scopeRequested);
    manualInput.rightButton = manual::ButtonState{ .available = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("native activation release rearms the gesture", manualDecision.state, manual::State::Idle);
    ok &= expectFalse("native activation release cannot reload", manualDecision.dispatchReload);

    manual::reset(manualState);
    manual::beginPrimaryActivateGesture(manualState, true);
    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("native event before raw polling retains its use decision", manualDecision.state, manual::State::NativeActivation);
    ok &= expectFalse("native event followed by raw press cannot reload or scope", manualDecision.dispatchReload || manualDecision.scopeRequested);
    manualInput.menuInputActive = true;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("a menu opened by native activation cancels the held gesture", manualDecision.state, manual::State::BlockedUntilRelease);
    manualInput.menuInputActive = false;
    manualInput.rightButton = manual::ButtonState{ .available = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectFalse("closing a use menu cannot reload on release", manualDecision.dispatchReload);

    manualInput.nativeActivationTarget = true;
    manualInput.rightButton = manual::ButtonState{ .available = true, .pressed = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectFalse("a complete native use tap between polls cannot reload or scope", manualDecision.dispatchReload || manualDecision.scopeRequested);
    manualInput.nativeActivationTarget = false;
    manualInput.rightButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("the next press without a use target starts weapon input", manualDecision.state, manual::State::Pending);
    manual::beginPrimaryActivateGesture(manualState, true);
    manualInput.nativeActivationTarget = true;
    manualInput.rightButton.pressed = false;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("aiming at a use target cannot steal an existing scope gesture", manualDecision.scopeRequested);
    manualInput.rightButton = manual::ButtonState{ .available = true, .released = true };
    (void)manual::update(manualState, manualInput);

    manual::beginPrimaryActivateGesture(manualState, false);
    manualInput.rightButton = manual::ButtonState{ .available = true, .pressed = true, .released = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("native event weapon decision survives a use target appearing before raw polling", manualDecision.dispatchReload);

    manual::reset(manualState);
    manualInput.nativeActivationTarget = true;
    manualDecision = manual::update(manualState, manualInput);
    manual::beginPrimaryActivateGesture(manualState, false);
    ok &= expectTrue("raw use tap retains native routing when the native press arrives after release", manualState.primaryPressUsesNative);
    ok &= expectManualScopeState("late native press cannot restart the completed use tap", manualState.state, manual::State::Idle);
    ok &= expectFalse("raw use tap dispatches no weapon action", manualDecision.dispatchReload || manualDecision.scopeRequested);

    manual::reset(manualState);
    manualInput.nativeActivationTarget = false;
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectTrue("raw weapon tap reloads when no use target is present", manualDecision.dispatchReload);
    manual::beginPrimaryActivateGesture(manualState, true);
    ok &= expectFalse("a late native press cannot also activate a newly acquired use target", manualState.primaryPressUsesNative);
    ok &= expectManualScopeState("late native press cannot restart a completed reload tap", manualState.state, manual::State::Idle);
    manualInput.rightButton = manual::ButtonState{ .available = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectFalse("the frame after a late native press cannot repeat reload", manualDecision.dispatchReload);

    manualInput.nativeActivationTarget = true;
    manualInput.firingHandIsLeft = true;
    manualInput.leftButton = manual::ButtonState{ .available = true, .held = true, .pressed = true };
    manualInput.rightButton = manual::ButtonState{ .available = true };
    manualDecision = manual::update(manualState, manualInput);
    ok &= expectManualScopeState("right-wand use target leaves left firing-hand input available", manualDecision.state, manual::State::Pending);

    return ok;
}

int main()
{
    bool ok = true;
    ok &= testInputRouting();
    ok &= testNativeVats();
    ok &= testPipboyGestures();
    ok &= testManualScope();
    return ok ? 0 : 1;
}
