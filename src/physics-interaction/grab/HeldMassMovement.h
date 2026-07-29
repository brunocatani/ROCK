#pragma once

#include <algorithm>
#include <cmath>

namespace rock::held_mass_movement
{
    struct Config
    {
        bool enabled = true;
        float massProportion = 0.675f;
        float massExponent = 1.0f;
        float maxReduction = 75.0f;
        float fadeOutSeconds = 5.0f;
    };

    inline float sanitizeReduction(float value)
    {
        return std::clamp(std::isfinite(value) ? value : 0.0f, 0.0f, 99.0f);
    }

    inline float computeHeldMassReduction(float heldMass, const Config& config)
    {
        if (!config.enabled || !std::isfinite(heldMass) || heldMass <= 0.0f) {
            return 0.0f;
        }

        const float massProportion = (std::max)(0.0f, std::isfinite(config.massProportion) ? config.massProportion : 0.675f);
        const float massExponent = std::clamp(std::isfinite(config.massExponent) ? config.massExponent : 1.0f, 0.0f, 4.0f);
        const float maxReduction = sanitizeReduction(config.maxReduction);
        return std::clamp(std::pow(heldMass, massExponent) * massProportion, 0.0f, maxReduction);
    }

    inline float computeFadeOutReduction(float startReduction, float elapsedSeconds, float fadeOutSeconds)
    {
        const float start = sanitizeReduction(startReduction);
        if (start <= 0.0f) {
            return 0.0f;
        }

        const float fade = std::isfinite(fadeOutSeconds) ? fadeOutSeconds : 0.0f;
        if (fade <= 0.001f) {
            return 0.0f;
        }

        const float elapsed = std::isfinite(elapsedSeconds) ? (std::max)(0.0f, elapsedSeconds) : 0.0f;
        return start * std::clamp(1.0f - elapsed / fade, 0.0f, 1.0f);
    }
}
