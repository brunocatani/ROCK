#pragma once

#include <cstdint>
#include <string_view>

namespace rock::pipboy_equip_policy
{
    enum class Hand : std::uint8_t
    {
        Right,
        Left,
    };

    enum class TriggerSource : std::uint8_t
    {
        FallbackRight,
        PhysicalLevel,
        FreshTransition,
        ConfiguredPreference,
    };

    enum class EquipMode : std::uint8_t
    {
        NativeRight,
        PreferredLeft,
        TriggerHand,
    };

    struct TransitionToken
    {
        std::uint32_t menuGeneration{ 0 };
        std::uint32_t tickMilliseconds{ 0 };
        bool present{ false };
    };

    struct TriggerResolution
    {
        Hand hand{ Hand::Right };
        TriggerSource source{ TriggerSource::FallbackRight };
    };

    [[nodiscard]] inline constexpr EquipMode resolveEquipMode(
        const bool triggerHandEnabled,
        const bool preferredHandLeft) noexcept
    {
        if (triggerHandEnabled) {
            return EquipMode::TriggerHand;
        }
        return preferredHandLeft ? EquipMode::PreferredLeft : EquipMode::NativeRight;
    }

    [[nodiscard]] inline constexpr bool managesHandAssignment(const EquipMode mode) noexcept
    {
        return mode != EquipMode::NativeRight;
    }

    [[nodiscard]] inline constexpr TriggerResolution resolveRequestedHand(
        const EquipMode mode,
        const TriggerResolution triggerResolution) noexcept
    {
        if (mode == EquipMode::TriggerHand) {
            return triggerResolution;
        }
        return TriggerResolution{
            .hand = mode == EquipMode::PreferredLeft ? Hand::Left : Hand::Right,
            .source = TriggerSource::ConfiguredPreference,
        };
    }

    [[nodiscard]] inline constexpr bool isFreshTransition(
        const TransitionToken& token,
        const std::uint32_t currentMenuGeneration,
        const std::uint32_t nowMilliseconds,
        const std::uint32_t maximumAgeMilliseconds) noexcept
    {
        return token.present &&
               token.menuGeneration != 0 &&
               token.menuGeneration == currentMenuGeneration &&
               static_cast<std::uint32_t>(nowMilliseconds - token.tickMilliseconds) <= maximumAgeMilliseconds;
    }

    /*
     * Physical level is authoritative because the Pip-Boy may invoke the
     * selection callback before OpenVR publishes the corresponding release.
     * A transition token is only a short, menu-generation-bound bridge for
     * controllers whose UI action fires on release. Both/no evidence remains
     * native right-hand behavior; ROCK never guesses between two hands.
     */
    [[nodiscard]] inline constexpr TriggerResolution resolveTriggerHand(
        const bool leftHeld,
        const bool rightHeld,
        const TransitionToken& leftTransition,
        const TransitionToken& rightTransition,
        const std::uint32_t currentMenuGeneration,
        const std::uint32_t nowMilliseconds,
        const std::uint32_t maximumAgeMilliseconds) noexcept
    {
        if (leftHeld != rightHeld) {
            return TriggerResolution{
                .hand = leftHeld ? Hand::Left : Hand::Right,
                .source = TriggerSource::PhysicalLevel,
            };
        }
        if (leftHeld && rightHeld) {
            return {};
        }

        const bool freshLeft = isFreshTransition(leftTransition, currentMenuGeneration, nowMilliseconds, maximumAgeMilliseconds);
        const bool freshRight = isFreshTransition(rightTransition, currentMenuGeneration, nowMilliseconds, maximumAgeMilliseconds);
        if (freshLeft != freshRight) {
            return TriggerResolution{
                .hand = freshLeft ? Hand::Left : Hand::Right,
                .source = TriggerSource::FreshTransition,
            };
        }
        return {};
    }

    [[nodiscard]] inline constexpr std::string_view handTag(const Hand hand) noexcept
    {
        return hand == Hand::Left ? " [Left]" : " [Right]";
    }

    /*
     * Direct left carry is serviced before TwoHandedGrip's per-frame update.
     * Requiring two consecutive observations of hFRIK's stable native-right
     * offset reserves the intervening update for canonical-frame capture.
     */
    [[nodiscard]] inline constexpr bool advanceNativeOffsetReadiness(
        const bool offsetSampleValid,
        const bool liveOffsetMatches,
        std::uint8_t& consecutiveMatchingFrames) noexcept
    {
        if (!offsetSampleValid || !liveOffsetMatches) {
            consecutiveMatchingFrames = 0;
            return false;
        }

        if (consecutiveMatchingFrames < 2) {
            ++consecutiveMatchingFrames;
        }
        return consecutiveMatchingFrames >= 2;
    }

    [[nodiscard]] inline constexpr bool shouldReacquirePersistentLeftCarry(
        const bool assignmentActive,
        const bool assignedLeft,
        const bool effectiveLeft,
        const bool persistentCarryActive,
        const bool manualOwnershipActive) noexcept
    {
        return assignmentActive &&
               assignedLeft &&
               effectiveLeft &&
               !persistentCarryActive &&
               !manualOwnershipActive;
    }
}
