#pragma once

#include "physics-interaction/visual/AmmoCounterPose.h"

#include <array>
#include <cstdint>

/*
 * ROCK's ammo counters: one per firing weapon, on the back of the hand that
 * holds its firing grip, drawn as RPS UI number overlays (loaded rounds and
 * reserve). RPS UI is optional; without it nothing is drawn here and the
 * vanilla counter stays.
 */
namespace rock::ammo_counters
{
    struct HandCounter
    {
        bool shown{ false };
        std::uint32_t loaded{ 0 };
        std::uint32_t reserve{ 0 };
        ammo_counter_pose::Pose pose{};
    };

    // Game thread, once per presented frame. Indexed by hand: [0] right,
    // [1] left. True while RPS UI accepted every overlay update, so the
    // counters can replace the vanilla one.
    bool update(const std::array<HandCounter, 2>& hands) noexcept;

    // Game thread. Closes every open counter.
    void hide() noexcept;
}
