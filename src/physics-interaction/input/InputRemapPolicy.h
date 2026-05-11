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
        int weaponToggleButtonId{ 32 };
        bool suppressRightGrabGameInput{ true };
        bool suppressRightFavoritesGameInput{ true };
        bool suppressRightTriggerGameInput{ true };
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
        std::uint64_t filteredPressed{ 0 };
        std::uint64_t filteredTouched{ 0 };
        std::uint8_t filteredAxisMask{ 0 };
        bool weaponToggleRequested{ false };
        bool grabHeld{ false };
        bool grabPressed{ false };
        bool grabReleased{ false };
    };

    struct NativeReadyWeaponSuppressionInput
    {
        bool remapEnabled{ true };
        bool suppressionEnabled{ true };
        bool gameplayInputAllowed{ true };
        bool menuInputActive{ false };
        bool weaponDrawn{ false };
        bool originalReadyActionMatched{ false };
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
     * ROCK's draw/holster remap must therefore suppress both representations while
     * the weapon is holstered; otherwise the attack handler can still auto-ready the
     * weapon even after the ReadyWeapon action itself is suppressed.
     */
    inline constexpr int kOpenVrAxisButtonBase = 32;
    inline constexpr int kOpenVrAxisCount = 5;
    inline constexpr int kOpenVrSteamVrTriggerButtonId = kOpenVrAxisButtonBase + 1;

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

    [[nodiscard]] constexpr bool shouldFilterGameFacingInput(std::uintptr_t callerAddress, std::uintptr_t gameTextStart, std::uintptr_t gameTextSize)
    {
        return gameTextSize != 0 && callerAddress >= gameTextStart && (callerAddress - gameTextStart) < gameTextSize;
    }

    [[nodiscard]] constexpr bool shouldSuppressNativeReadyWeaponAutoReady(const NativeReadyWeaponSuppressionInput& input)
    {
        return input.remapEnabled && input.suppressionEnabled && input.gameplayInputAllowed && !input.menuInputActive && input.originalReadyActionMatched && !input.weaponDrawn;
    }

    [[nodiscard]] constexpr bool shouldInstallNativeReadyWeaponSuppressionHook(bool remapEnabled, bool suppressionEnabled)
    {
        return remapEnabled && suppressionEnabled;
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
        decision.filteredPressed = input.rawPressed;
        decision.filteredTouched = input.rawTouched;

        const auto grabMask = buttonMask(settings.grabButtonId);
        const auto weaponToggleMask = buttonMask(settings.weaponToggleButtonId);
        const auto triggerMask = buttonMask(kOpenVrSteamVrTriggerButtonId);

        decision.grabHeld = grabMask != 0 && (input.rawPressed & grabMask) != 0;
        decision.grabPressed = grabMask != 0 && (input.rawPressed & grabMask) != 0 && (input.previousRawPressed & grabMask) == 0;
        decision.grabReleased = grabMask != 0 && (input.rawPressed & grabMask) == 0 && (input.previousRawPressed & grabMask) != 0;

        if (!settings.enabled || !input.gameplayInputAllowed || input.menuInputActive || input.hand != Hand::Right) {
            return decision;
        }

        if (settings.suppressRightGrabGameInput && !input.weaponDrawn && grabMask != 0) {
            decision.filteredPressed &= ~grabMask;
            decision.filteredTouched &= ~grabMask;
        }

        if (settings.suppressRightTriggerGameInput && !input.weaponDrawn && triggerMask != 0) {
            decision.filteredPressed &= ~triggerMask;
            decision.filteredTouched &= ~triggerMask;
            decision.filteredAxisMask |= axisMaskFromOpenVrButtonId(kOpenVrSteamVrTriggerButtonId);
        }

        if (settings.suppressRightFavoritesGameInput && weaponToggleMask != 0) {
            decision.filteredPressed &= ~weaponToggleMask;
            decision.filteredTouched &= ~weaponToggleMask;
        }

        decision.weaponToggleRequested =
            weaponToggleMask != 0 && (input.rawPressed & weaponToggleMask) != 0 && (input.previousRawPressed & weaponToggleMask) == 0;
        return decision;
    }
}
