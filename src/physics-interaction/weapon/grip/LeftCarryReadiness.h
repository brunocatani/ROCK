#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include "physics-interaction/weapon/AuthoredSupportGrabPolicy.h"

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

    struct TakeoverWitness
    {
        authored_support_grab_policy::LeftFiringTakeoverReadiness last{
            authored_support_grab_policy::LeftFiringTakeoverReadiness::
                NotRequired
        };

        // Records the observation; returns true when it changed so the
        // caller can emit its one log-on-change diagnostic.
        [[nodiscard]] bool observe(
            const authored_support_grab_policy::LeftFiringTakeoverReadiness
                current) noexcept
        {
            if (last == current) {
                return false;
            }
            last = current;
            return true;
        }
    };
}
