#pragma once

/*
 * Shoulder stash haptics are separated from the frame loop because the HMD
 * back gesture needs steady proximity feedback while the core interaction code
 * only decides when a stash candidate exists. Keeping the confidence-to-pulse
 * mapping pure makes the feel tunable without coupling it to inventory transfer
 * or body-zone fallback rules.
 */

#include <algorithm>
#include <cmath>

namespace rock::shoulder_stash_haptic_policy
{
    struct CandidatePulseConfig
    {
        bool enabled = true;
        float baseIntensity = 0.20f;
        float maxIntensity = 0.42f;
    };

    [[nodiscard]] inline float finiteOr(float value, float fallback) noexcept
    {
        return std::isfinite(value) ? value : fallback;
    }

    [[nodiscard]] inline float sanitizeIntensity(float value, float fallback) noexcept
    {
        return std::clamp(finiteOr(value, fallback), 0.0f, 1.0f);
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
