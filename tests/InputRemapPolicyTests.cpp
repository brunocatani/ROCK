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

    ok &= expectTrue("enabled suppression requests native hook install", shouldInstallNativeActionSuppressionHook(true, true));
    ok &= expectFalse("disabled remap skips native hook install", shouldInstallNativeActionSuppressionHook(false, true));

    return ok ? 0 : 1;
}
