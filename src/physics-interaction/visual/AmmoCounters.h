#pragma once

#include "physics-interaction/visual/AmmoCounterPose.h"

#include <array>
#include <cstdint>

/*
 * ROCK's ammo counters: one per firing weapon, on the hand that holds its
 * firing grip, drawn as RPS UI number overlays. RPS UI is optional; without
 * it nothing is drawn here and the vanilla counter stays.
 */
namespace rock::ammo_counters
{
    // iAmmoCounterStyle in ROCK.ini.
    enum class Style : int
    {
        // White loaded and reserve rounds on the weapon's palm side, shown
        // while looked at: two plain overlays per hand.
        Minimal = 1,
        // Minimal's place and fade, in the game's HUD style: one bracketed
        // HUD overlay per hand in the HUD color.
        Immersive = 2,
        // The game's counter on the back of the holding hand: one HUD overlay
        // per hand in the HUD color and opacity.
        Vanilla = 3,
    };

    struct HandCounter
    {
        bool shown{ false };
        std::uint32_t loaded{ 0 };
        std::uint32_t reserve{ 0 };
        ammo_counter_pose::Pose pose{};
    };

    // Game thread, once per presented frame. Indexed by hand: [0] right,
    // [1] left. hudColor: the game's HUD color, 0..1 per channel, used by the
    // HUD styles. True while RPS UI accepted every overlay update, so the
    // counters can replace the vanilla one.
    bool update(const std::array<HandCounter, 2>& hands, Style style, const std::array<float, 3>& hudColor) noexcept;

    // Game thread. Closes every open counter.
    void hide() noexcept;
}
