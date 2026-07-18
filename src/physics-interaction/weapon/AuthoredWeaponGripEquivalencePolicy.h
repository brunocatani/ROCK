#pragma once

#include <cstdint>

namespace rock::authored_weapon_grip_equivalence_policy
{
    // The equipped graph can still be inside its draw transition when the
    // first live sample arrives. Equivalence telemetry therefore waits for a
    // short run of nearly identical Weapon-relative samples before labelling a
    // comparison stable. These constants affect diagnostics only.
    constexpr float kStableTranslationToleranceGameUnits = 0.025f;
    constexpr float kStableRotationToleranceDegrees = 0.1f;
    constexpr float kStableScaleTolerance = 0.0005f;
    constexpr std::uint8_t kStableSamplesRequired = 6;

    [[nodiscard]] constexpr std::uint8_t advanceStableSampleCount(
        const bool previousSampleAvailable,
        const std::uint8_t currentStableSamples,
        const float translationDeltaGameUnits,
        const float rotationDeltaDegrees,
        const float scaleDelta)
    {
        const bool stable = previousSampleAvailable &&
            translationDeltaGameUnits <= kStableTranslationToleranceGameUnits &&
            rotationDeltaDegrees <= kStableRotationToleranceDegrees &&
            scaleDelta <= kStableScaleTolerance;
        if (!stable) {
            return 1;
        }
        return currentStableSamples < kStableSamplesRequired ?
            static_cast<std::uint8_t>(currentStableSamples + 1) :
            kStableSamplesRequired;
    }

    [[nodiscard]] constexpr bool readyForStableReport(const std::uint8_t stableSamples)
    {
        return stableSamples >= kStableSamplesRequired;
    }
}
