#pragma once

/*
 * All semantic children belong to one hand compound. Their solver deviations
 * already measure its displacement from controller intent, including forearm
 * contacts. Applying an additional IK leverage gain overcorrects the hand and
 * makes it move away from a surface as the controller presses farther into it.
 */

#include <array>
#include <cmath>
#include <cstddef>

namespace rock::dynamic_hand_collision_kinematics
{
    // Preserve the existing bounded half-space projection: contacts sharing a
    // surface collapse to one correction; contacts around a corner compose.
    template <class Vector, std::size_t SlotCount>
    [[nodiscard]] inline Vector combineTwinDeviations(
        const std::array<Vector, SlotCount>& deviations,
        const std::array<bool, SlotCount>& deviationValid) noexcept
    {
        constexpr float kTinyDeviation = 1.0e-4f;
        Vector combined{};
        for (int pass = 0; pass < 3; ++pass) {
            bool changed = false;
            for (std::size_t i = 0; i < deviations.size(); ++i) {
                const auto& deviation = deviations[i];
                if (!deviationValid[i] || !std::isfinite(deviation.x) ||
                    !std::isfinite(deviation.y) || !std::isfinite(deviation.z)) {
                    continue;
                }
                const float length = std::sqrt(
                    deviation.x * deviation.x +
                    deviation.y * deviation.y +
                    deviation.z * deviation.z);
                if (!std::isfinite(length) || length <= kTinyDeviation) {
                    continue;
                }
                const Vector direction{
                    deviation.x / length,
                    deviation.y / length,
                    deviation.z / length,
                };
                const float needed =
                    length - (combined.x * direction.x + combined.y * direction.y + combined.z * direction.z);
                if (needed <= kTinyDeviation) {
                    continue;
                }
                combined.x += direction.x * needed;
                combined.y += direction.y * needed;
                combined.z += direction.z * needed;
                changed = true;
            }
            if (!changed) {
                break;
            }
        }
        return combined;
    }
}
