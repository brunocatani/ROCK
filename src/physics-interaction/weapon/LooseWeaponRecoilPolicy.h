#pragma once
#include <algorithm>
#include <cmath>

namespace rock::loose_weapon_recoil
{
    // Per-weapon equivalent of FO4VR EF96B0 + EFD1C0. The actor-global
    // implementation only finds equipped index zero; a loose gun owns this
    // same native kickback envelope without mutating the player's arm graph.
    struct Kick
    {
        float offset{}, duration{}, elapsed{};
        void fire(float targetMagnitude, float currentMagnitude, float minOffset,
            float maxOffset, float minDuration, float maxDuration) noexcept
        {
            *this = {};
            for (const float value : {targetMagnitude, currentMagnitude, minOffset, maxOffset, minDuration, maxDuration})
                if (!std::isfinite(value)) return;
            if (minOffset < 0 || maxOffset < minOffset || minDuration <= 0 || maxDuration < minDuration) return;
            // EF96B0 multiplies by the verified 4/pi constant at RVA 2D85C2C.
            const float strength = std::abs(targetMagnitude - currentMagnitude) * 1.2732395447351627f;
            offset = std::clamp(minOffset + (maxOffset - minOffset) * strength, minOffset, maxOffset);
            duration = std::clamp(minDuration + (maxDuration - minDuration) * strength, minDuration, maxDuration);
        }
        float advance(float seconds) noexcept
        {
            if (!std::isfinite(seconds) || seconds < 0 || duration <= 0) return 0;
            elapsed = (std::min)(duration, elapsed + seconds);
            const float remaining = 1.0f - elapsed / duration;
            return offset * remaining * remaining;
        }
    };
}
