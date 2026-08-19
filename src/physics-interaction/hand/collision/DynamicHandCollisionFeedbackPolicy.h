#pragma once

/*
 * Dynamic hand collision feedback is driven by contact-entry generations
 * published by the physics thread. The policy is engine-independent so the
 * edge/cooldown and intensity contract can be tested without a running world.
 * Runtime code remains responsible for delivering the resulting pulse through
 * ROCK's main-thread FeedbackHaptics queue.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::dynamic_hand_collision_feedback
{
    struct ContactPulseConfig
    {
        bool enabled = true;
        float baseIntensity = 0.18f;
        float maxIntensity = 0.55f;
        float speedScale = 0.006f;
        float minApproachSpeedGameUnitsPerSecond = 3.0f;
        float cooldownSeconds = 0.12f;
    };

    struct ContactPulseState
    {
        std::uint64_t observedEntrySequence = 0;
        float cooldownRemainingSeconds = 0.0f;
    };

    struct ContactPulseDecision
    {
        bool fire = false;
        float intensity = 0.0f;
        float approachSpeedGameUnitsPerSecond = 0.0f;
        std::uint64_t entrySequence = 0;
    };

    template <class Vector>
    [[nodiscard]] inline float projectedApproachSpeedGameUnitsPerSecond(
        const Vector& targetVelocityGameUnitsPerSecond,
        const Vector& contactDeviationGame)
    {
        const float deviationLengthSquared =
            contactDeviationGame.x * contactDeviationGame.x +
            contactDeviationGame.y * contactDeviationGame.y +
            contactDeviationGame.z * contactDeviationGame.z;
        if (!std::isfinite(deviationLengthSquared) || deviationLengthSquared <= 1.0e-8f ||
            !std::isfinite(targetVelocityGameUnitsPerSecond.x) ||
            !std::isfinite(targetVelocityGameUnitsPerSecond.y) ||
            !std::isfinite(targetVelocityGameUnitsPerSecond.z)) {
            return 0.0f;
        }

        const float inverseDeviationLength = 1.0f / std::sqrt(deviationLengthSquared);
        const float velocityAlongOutwardDeviation =
            targetVelocityGameUnitsPerSecond.x * contactDeviationGame.x * inverseDeviationLength +
            targetVelocityGameUnitsPerSecond.y * contactDeviationGame.y * inverseDeviationLength +
            targetVelocityGameUnitsPerSecond.z * contactDeviationGame.z * inverseDeviationLength;
        return std::isfinite(velocityAlongOutwardDeviation) ? std::max(0.0f, -velocityAlongOutwardDeviation) : 0.0f;
    }

    [[nodiscard]] inline ContactPulseDecision updateContactPulse(
        ContactPulseState& state,
        std::uint64_t entrySequence,
        float approachSpeedGameUnitsPerSecond,
        float deltaSeconds,
        bool authorityAllowsFeedback,
        const ContactPulseConfig& config)
    {
        const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
        state.cooldownRemainingSeconds = std::max(0.0f, state.cooldownRemainingSeconds - dt);

        ContactPulseDecision decision{};
        decision.entrySequence = entrySequence;
        decision.approachSpeedGameUnitsPerSecond =
            std::isfinite(approachSpeedGameUnitsPerSecond) ? std::max(0.0f, approachSpeedGameUnitsPerSecond) : 0.0f;

        const bool newEntry = entrySequence != 0 && entrySequence != state.observedEntrySequence;
        if (newEntry) {
            // Consume suppressed/disabled entries so authority handoffs never
            // replay a stale collision as a delayed haptic pulse.
            state.observedEntrySequence = entrySequence;
        }

        const float minApproachSpeed =
            std::isfinite(config.minApproachSpeedGameUnitsPerSecond) ? std::max(0.0f, config.minApproachSpeedGameUnitsPerSecond) : 0.0f;
        if (!config.enabled || !authorityAllowsFeedback || !newEntry ||
            state.cooldownRemainingSeconds > 0.0f ||
            decision.approachSpeedGameUnitsPerSecond < minApproachSpeed) {
            return decision;
        }

        const float baseIntensity =
            std::clamp(std::isfinite(config.baseIntensity) ? config.baseIntensity : 0.18f, 0.0f, 1.0f);
        const float maxIntensity =
            std::clamp(std::isfinite(config.maxIntensity) ? config.maxIntensity : 0.55f, baseIntensity, 1.0f);
        const float speedScale = std::isfinite(config.speedScale) ? std::max(0.0f, config.speedScale) : 0.0f;

        decision.fire = true;
        decision.intensity = std::clamp(
            baseIntensity + decision.approachSpeedGameUnitsPerSecond * speedScale,
            baseIntensity,
            maxIntensity);
        state.cooldownRemainingSeconds =
            std::isfinite(config.cooldownSeconds) ? std::max(0.0f, config.cooldownSeconds) : 0.12f;
        return decision;
    }
}
