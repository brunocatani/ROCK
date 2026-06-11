#pragma once

/*
 * Small haptic feel policies live together so grab and shoulder-stash feedback
 * share intensity sanitization without scattering near-identical math across
 * interaction subsystems. The runtime owners still decide when pulses happen.
 */

#include <algorithm>
#include <cmath>

namespace rock::haptic_policy_detail
{
    [[nodiscard]] inline float finiteOr(float value, float fallback) noexcept
    {
        return std::isfinite(value) ? value : fallback;
    }

    [[nodiscard]] inline float sanitizeIntensity(float value, float fallback) noexcept
    {
        return std::clamp(finiteOr(value, fallback), 0.0f, 1.0f);
    }
}

namespace rock::grab_haptic_policy
{
    struct MassPulseConfig
    {
        bool enabled = true;
        float baseIntensity = 0.12f;
        float maxIntensity = 0.80f;
        float massScale = 0.06f;
        float massExponent = 0.60f;
    };

    struct ImpactPulseConfig
    {
        bool enabled = true;
        float baseIntensity = 0.12f;
        float maxIntensity = 0.85f;
        float speedScale = 0.006f;
        float massScale = 0.035f;
        float massExponent = 0.55f;
        float minSpeedGameUnitsPerSecond = 8.0f;
        float dampedMultiplier = 0.55f;
    };

    [[nodiscard]] inline float finiteOr(float value, float fallback) noexcept
    {
        return haptic_policy_detail::finiteOr(value, fallback);
    }

    [[nodiscard]] inline float sanitizeIntensity(float value, float fallback) noexcept
    {
        return haptic_policy_detail::sanitizeIntensity(value, fallback);
    }

    [[nodiscard]] inline float computeMassPulseIntensity(float mass, const MassPulseConfig& config)
    {
        if (!config.enabled) {
            return 0.0f;
        }

        const float base = sanitizeIntensity(config.baseIntensity, 0.12f);
        const float maxIntensity = std::clamp(sanitizeIntensity(config.maxIntensity, 0.80f), base, 1.0f);
        const float finiteMass = std::isfinite(mass) && mass > 0.0f ? mass : 0.0f;
        const float massExponent = std::clamp(finiteOr(config.massExponent, 0.60f), 0.0f, 2.0f);
        const float massScale = (std::max)(0.0f, finiteOr(config.massScale, 0.06f));
        return std::clamp(base + massScale * std::pow(finiteMass, massExponent), base, maxIntensity);
    }

    [[nodiscard]] inline float computeImpactPulseIntensity(float mass, float speedGameUnitsPerSecond, bool damped, const ImpactPulseConfig& config)
    {
        if (!config.enabled) {
            return 0.0f;
        }

        const float speed = std::isfinite(speedGameUnitsPerSecond) && speedGameUnitsPerSecond > 0.0f ? speedGameUnitsPerSecond : 0.0f;
        const float minSpeed = (std::max)(0.0f, finiteOr(config.minSpeedGameUnitsPerSecond, 8.0f));
        if (speed < minSpeed) {
            return 0.0f;
        }

        const float base = sanitizeIntensity(config.baseIntensity, 0.12f);
        const float maxIntensity = std::clamp(sanitizeIntensity(config.maxIntensity, 0.85f), base, 1.0f);
        const float finiteMass = std::isfinite(mass) && mass > 0.0f ? mass : 0.0f;
        const float massExponent = std::clamp(finiteOr(config.massExponent, 0.55f), 0.0f, 2.0f);
        const float massScale = (std::max)(0.0f, finiteOr(config.massScale, 0.035f));
        const float speedScale = (std::max)(0.0f, finiteOr(config.speedScale, 0.006f));

        float intensity = base + speed * speedScale + massScale * std::pow(finiteMass, massExponent);
        intensity = std::clamp(intensity, base, maxIntensity);
        if (damped) {
            const float multiplier = std::clamp(finiteOr(config.dampedMultiplier, 0.55f), 0.0f, 1.0f);
            intensity = std::clamp(intensity * multiplier, base, maxIntensity);
        }
        return intensity;
    }
}

namespace rock::candidate_haptic_policy
{
    struct CandidatePulseConfig
    {
        bool enabled = true;
        float baseIntensity = 0.20f;
        float maxIntensity = 0.42f;
    };

    [[nodiscard]] inline float finiteOr(float value, float fallback) noexcept
    {
        return haptic_policy_detail::finiteOr(value, fallback);
    }

    [[nodiscard]] inline float sanitizeIntensity(float value, float fallback) noexcept
    {
        return haptic_policy_detail::sanitizeIntensity(value, fallback);
    }

    [[nodiscard]] inline float computeCandidatePulseIntensity(float confidence, const CandidatePulseConfig& config) noexcept
    {
        if (!config.enabled) {
            return 0.0f;
        }

        const float base = sanitizeIntensity(config.baseIntensity, 0.20f);
        const float maxIntensity = std::clamp(sanitizeIntensity(config.maxIntensity, 0.42f), base, 1.0f);
        const float normalizedConfidence = std::clamp(finiteOr(confidence, 0.0f), 0.0f, 1.0f);
        return std::clamp(base + (maxIntensity - base) * normalizedConfidence, base, maxIntensity);
    }
}

namespace rock::shoulder_stash_haptic_policy
{
    using CandidatePulseConfig = candidate_haptic_policy::CandidatePulseConfig;

    [[nodiscard]] inline float computeCandidatePulseIntensity(float confidence, const CandidatePulseConfig& config) noexcept
    {
        return candidate_haptic_policy::computeCandidatePulseIntensity(confidence, config);
    }
}
