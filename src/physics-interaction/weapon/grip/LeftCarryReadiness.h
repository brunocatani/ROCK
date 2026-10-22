#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstdint>

// Shared observations for handing the equipped weapon carrier to the left
// firing hand (held trigger/grip-zone equip and shoulder retrieval).

namespace rock::left_carry_readiness
{
    [[nodiscard]] inline bool finiteTransform(
        const RE::NiTransform& transform) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > 0.0001f;
    }
}
