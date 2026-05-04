#include "physics-interaction/InputRemapPolicy.h"

#include <cstdio>
#include <cstdint>

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

    bool expectMask(const char* label, std::uint64_t actual, std::uint64_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected 0x%016llX got 0x%016llX\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }

    bool expectByteMask(const char* label, std::uint8_t actual, std::uint8_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected 0x%02X got 0x%02X\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }
}

int main()
{
    using namespace frik::rock::input_remap_policy;

    bool ok = true;

    const auto gripMask = buttonMask(2);
    const auto stickMask = buttonMask(32);
    const auto triggerMask = buttonMask(33);
    const auto leftOnlyMask = buttonMask(7);

    Settings settings{};
    settings.enabled = true;
    settings.grabButtonId = 2;
    settings.weaponToggleButtonId = 32;
    settings.suppressRightGrabGameInput = true;
    settings.suppressRightFavoritesGameInput = true;

    Input rightPress{};
    rightPress.hand = Hand::Right;
    rightPress.gameplayInputAllowed = true;
    rightPress.rawPressed = gripMask | stickMask | leftOnlyMask;
    rightPress.rawTouched = gripMask | stickMask;
    rightPress.previousRawPressed = gripMask;

    auto decision = evaluate(rightPress, settings);
    ok &= expectMask("right hand filters grip and favorites pressed bits", decision.filteredPressed, leftOnlyMask);
    ok &= expectMask("right hand filters grip and favorites touched bits", decision.filteredTouched, 0);
    ok &= expectTrue("right stick rising edge requests weapon toggle", decision.weaponToggleRequested);
    ok &= expectTrue("raw grip remains visible to ROCK", decision.grabHeld);
    ok &= expectFalse("held grip is not a fresh grab edge", decision.grabPressed);
    ok &= expectFalse("held grip is not a release edge", decision.grabReleased);

    Input heldStick = rightPress;
    heldStick.previousRawPressed = rightPress.rawPressed;
    decision = evaluate(heldStick, settings);
    ok &= expectFalse("held right stick click does not repeat weapon toggle", decision.weaponToggleRequested);

    Input drawnGripReload = rightPress;
    drawnGripReload.weaponDrawn = true;
    drawnGripReload.rawPressed = gripMask | leftOnlyMask;
    drawnGripReload.rawTouched = gripMask;
    drawnGripReload.previousRawPressed = 0;
    decision = evaluate(drawnGripReload, settings);
    ok &= expectMask("right grip passes through for vanilla reload while weapon is drawn", decision.filteredPressed, gripMask | leftOnlyMask);
    ok &= expectMask("right grip touch passes through while weapon is drawn", decision.filteredTouched, gripMask);
    ok &= expectTrue("drawn right grip remains visible to ROCK as raw grab", decision.grabPressed);

    Input holsteredTrigger = rightPress;
    holsteredTrigger.rawPressed = triggerMask | leftOnlyMask;
    holsteredTrigger.rawTouched = triggerMask;
    holsteredTrigger.previousRawPressed = 0;
    decision = evaluate(holsteredTrigger, settings);
    ok &= expectMask("right trigger is removed from game-facing input while holstered", decision.filteredPressed, leftOnlyMask);
    ok &= expectMask("right trigger touch is removed from game-facing input while holstered", decision.filteredTouched, 0);
    ok &= expectByteMask("right trigger analog axis is removed from game-facing input while holstered",
        decision.filteredAxisMask,
        axisMaskFromOpenVrButtonId(kOpenVrSteamVrTriggerButtonId));

    Input drawnTrigger = holsteredTrigger;
    drawnTrigger.weaponDrawn = true;
    decision = evaluate(drawnTrigger, settings);
    ok &= expectMask("right trigger passes through while weapon drawn", decision.filteredPressed, triggerMask | leftOnlyMask);
    ok &= expectMask("right trigger touch passes through while weapon drawn", decision.filteredTouched, triggerMask);
    ok &= expectByteMask("right trigger analog axis passes through while weapon drawn", decision.filteredAxisMask, 0);

    ok &= expectTrue("native ready action is suppressed only while holstered",
        shouldSuppressNativeReadyWeaponAutoReady(NativeReadyWeaponSuppressionInput{
            .remapEnabled = true,
            .suppressionEnabled = true,
            .gameplayInputAllowed = true,
            .weaponDrawn = false,
            .originalReadyActionMatched = true,
        }));
    ok &= expectFalse("native ready action passes while weapon drawn",
        shouldSuppressNativeReadyWeaponAutoReady(NativeReadyWeaponSuppressionInput{
            .remapEnabled = true,
            .suppressionEnabled = true,
            .gameplayInputAllowed = true,
            .weaponDrawn = true,
            .originalReadyActionMatched = true,
        }));
    ok &= expectFalse("native ready action passes in menus",
        shouldSuppressNativeReadyWeaponAutoReady(NativeReadyWeaponSuppressionInput{
            .remapEnabled = true,
            .suppressionEnabled = true,
            .gameplayInputAllowed = false,
            .weaponDrawn = false,
            .originalReadyActionMatched = true,
        }));
    ok &= expectFalse("native ready action passes when a menu is active despite stale gameplay input",
        shouldSuppressNativeReadyWeaponAutoReady(NativeReadyWeaponSuppressionInput{
            .remapEnabled = true,
            .suppressionEnabled = true,
            .gameplayInputAllowed = true,
            .menuInputActive = true,
            .weaponDrawn = false,
            .originalReadyActionMatched = true,
        }));
    ok &= expectFalse("native ready action ignores non-ready events",
        shouldSuppressNativeReadyWeaponAutoReady(NativeReadyWeaponSuppressionInput{
            .remapEnabled = true,
            .suppressionEnabled = true,
            .gameplayInputAllowed = true,
            .weaponDrawn = false,
            .originalReadyActionMatched = false,
        }));

    Input releasedGrip = rightPress;
    releasedGrip.rawPressed = stickMask;
    releasedGrip.previousRawPressed = gripMask | stickMask;
    decision = evaluate(releasedGrip, settings);
    ok &= expectFalse("released grip no longer held", decision.grabHeld);
    ok &= expectTrue("released grip reports raw ROCK release edge", decision.grabReleased);

    Input leftHand = rightPress;
    leftHand.hand = Hand::Left;
    decision = evaluate(leftHand, settings);
    ok &= expectMask("left hand is not game-filtered by right-hand remap", decision.filteredPressed, rightPress.rawPressed);
    ok &= expectFalse("left hand never requests right weapon toggle", decision.weaponToggleRequested);

    Input menuInput = rightPress;
    menuInput.gameplayInputAllowed = false;
    decision = evaluate(menuInput, settings);
    ok &= expectMask("menu input bypasses game filtering", decision.filteredPressed, rightPress.rawPressed);
    ok &= expectFalse("menu input bypass blocks weapon toggle", decision.weaponToggleRequested);

    Input staleGameplayGateMenuInput = rightPress;
    staleGameplayGateMenuInput.gameplayInputAllowed = true;
    staleGameplayGateMenuInput.menuInputActive = true;
    decision = evaluate(staleGameplayGateMenuInput, settings);
    ok &= expectMask("active menu input bypasses stale gameplay filtering", decision.filteredPressed, rightPress.rawPressed);
    ok &= expectMask("active menu input bypass preserves touched bits", decision.filteredTouched, rightPress.rawTouched);
    ok &= expectFalse("active menu input bypass blocks stale weapon toggle", decision.weaponToggleRequested);

    Settings disabled = settings;
    disabled.enabled = false;
    decision = evaluate(rightPress, disabled);
    ok &= expectMask("disabled remap leaves game state unchanged", decision.filteredPressed, rightPress.rawPressed);
    ok &= expectFalse("disabled remap does not request weapon toggle", decision.weaponToggleRequested);

    Settings invalidButtons = settings;
    invalidButtons.grabButtonId = 64;
    invalidButtons.weaponToggleButtonId = -1;
    decision = evaluate(rightPress, invalidButtons);
    ok &= expectMask("invalid buttons do not clear unrelated state", decision.filteredPressed, rightPress.rawPressed);
    ok &= expectFalse("invalid weapon button cannot request toggle", decision.weaponToggleRequested);
    ok &= expectFalse("invalid grab button cannot report held", decision.grabHeld);

    const auto initialRawEdge = evaluateEdgeTransition(false, 0, 0);
    ok &= expectMask("first raw sample uses current state as previous", initialRawEdge.previousPressedForEvaluation, 0);
    ok &= expectMask("first raw sample produces no pressed edges", initialRawEdge.pressedEdges, 0);
    ok &= expectMask("first raw sample produces no released edges", initialRawEdge.releasedEdges, 0);

    const auto externalPollRawEdge = evaluateEdgeTransition(true, 0, stickMask);
    ok &= expectMask("external poll records raw right stick pressed edge for ROCK", externalPollRawEdge.pressedEdges, stickMask);

    const auto gamePollAfterExternalRawPoll = evaluateEdgeTransition(true, 0, stickMask);
    Input gameFacingStickPress = rightPress;
    gameFacingStickPress.rawPressed = stickMask;
    gameFacingStickPress.rawTouched = stickMask;
    gameFacingStickPress.previousRawPressed = gamePollAfterExternalRawPoll.previousPressedForEvaluation;
    decision = evaluate(gameFacingStickPress, settings);
    ok &= expectTrue("game-facing right stick edge survives an earlier external raw poll", decision.weaponToggleRequested);

    const auto repeatedGamePoll = evaluateEdgeTransition(true, stickMask, stickMask);
    Input repeatedGameFacingStickPress = gameFacingStickPress;
    repeatedGameFacingStickPress.previousRawPressed = repeatedGamePoll.previousPressedForEvaluation;
    decision = evaluate(repeatedGameFacingStickPress, settings);
    ok &= expectFalse("game-facing right stick hold does not repeat after game state advances", decision.weaponToggleRequested);

    ok &= expectTrue("native ReadyWeapon hook installs only when remap and suppression are enabled",
        shouldInstallNativeReadyWeaponSuppressionHook(true, true));
    ok &= expectFalse("native ReadyWeapon hook does not install when remap is disabled", shouldInstallNativeReadyWeaponSuppressionHook(false, true));
    ok &= expectFalse("native ReadyWeapon hook does not install when suppression is disabled", shouldInstallNativeReadyWeaponSuppressionHook(true, false));

    ok &= expectTrue("game text caller is filtered", shouldFilterGameFacingInput(0x1010, 0x1000, 0x100));
    ok &= expectFalse("external mod caller is not filtered", shouldFilterGameFacingInput(0x2100, 0x1000, 0x100));
    ok &= expectFalse("game text end is exclusive", shouldFilterGameFacingInput(0x1100, 0x1000, 0x100));
    ok &= expectFalse("missing game text range is not filtered", shouldFilterGameFacingInput(0x1010, 0x1000, 0));

    return ok ? 0 : 1;
}
