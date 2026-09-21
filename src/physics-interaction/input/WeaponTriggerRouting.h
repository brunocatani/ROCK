#pragma once

#include <cstdint>

namespace rock::weapon_trigger_routing
{
    // Written by the interaction frame; copied as one atomic word to input
    // hooks. Physical hand facts must not straddle a firing-owner change.
    struct Snapshot
    {
        bool leftFiring{ false };
        bool detached{ false };
        bool rightLoose{ false };
        bool leftLoose{ false };
        bool transferPending{ false };

        [[nodiscard]] constexpr std::uint32_t pack() const noexcept
        {
            return (leftFiring ? 1u : 0u) | (detached ? 2u : 0u) |
                (rightLoose ? 4u : 0u) | (leftLoose ? 8u : 0u) | (transferPending ? 16u : 0u);
        }
        [[nodiscard]] static constexpr Snapshot unpack(std::uint32_t bits) noexcept
        {
            return { (bits & 1u) != 0, (bits & 2u) != 0, (bits & 4u) != 0,
                (bits & 8u) != 0, (bits & 16u) != 0 };
        }
        [[nodiscard]] constexpr bool physicalSourceIsLeft(bool nativePrimary, bool triggerEvent) const noexcept
        {
            return triggerEvent && nativePrimary && leftFiring ? true : !nativePrimary;
        }
        [[nodiscard]] constexpr bool holdsLoose(bool left) const noexcept { return left ? leftLoose : rightLoose; }
    };
}
