#pragma once

#include <cstdint>

namespace rock::frik_skeleton_profile
{
    struct Snapshot
    {
        std::uint32_t generation = 0;
        bool inPowerArmor = false;
        bool valid = false;
    };

    /*
     * hFRIK owns the player skeleton generation and debounces the game's
     * transient power-armor detector before it rebuilds that skeleton. Publish
     * the detector value only when hFRIK announces a ready generation, then
     * retain it until the matching skeleton is destroyed. Every ROCK choice
     * tied to skeleton geometry must use effectiveInPowerArmor() instead of
     * sampling the raw detector independently.
     */
    void publishReady(std::uint32_t generation, bool inPowerArmor) noexcept;
    void clear() noexcept;

    [[nodiscard]] Snapshot current() noexcept;
    [[nodiscard]] bool effectiveInPowerArmor() noexcept;
}
