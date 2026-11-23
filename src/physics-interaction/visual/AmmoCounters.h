#pragma once

#include "physics-interaction/visual/AmmoCounterPose.h"

#include <ROCK/Weapon.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

/*
 * ROCK's ammo counters: one per firing weapon, on the hand that holds its
 * firing grip, drawn as RPS UI number overlays. RPS UI is optional; without
 * it nothing is drawn here and the vanilla counter stays.
 *
 * Every style names the weapon's ammunition (its ammo form's name) as the
 * overlays' own label (RPS UI NumberOverlay 2.1), in the digits' face, color
 * and fade: the HUD styles beside the reserve row in the chamber column,
 * Minimal after the reserve rounds. With RPS UI 2.0 the counters show no
 * caliber.
 */
namespace rock::ammo_counters
{
    // WeaponHandling.AmmoCounterStyle in ROCK.toml.
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

    // The caliber label: at most kCaliberCapacity - 1 characters, terminated.
    inline constexpr std::size_t kCaliberCapacity = 24;
    using Caliber = std::array<char, kCaliberCapacity>;

    // The label for an ammo form's display name: surrounding spaces trimmed
    // and a trailing "Round", "Rounds" or "Ammo" word dropped when another
    // word precedes it (".38 Round" shows ".38"). A longer name ends at its
    // last whole word that fits. Empty for an empty name. It builds a string,
    // so callers keep the result instead of calling it every frame.
    [[nodiscard]] Caliber caliberLabel(std::string_view name) noexcept;

    struct HandCounter
    {
        bool shown{ false };
        std::uint32_t loaded{ 0 };
        std::uint32_t reserve{ 0 };
        api::weapon::ChamberMarker chamber{ api::weapon::ChamberMarker::None };
        // A counter presentation's colour, replacing the style's.
        bool colorOverride{ false };
        std::array<float, 3> color{ 1.0f, 1.0f, 1.0f };
        ammo_counter_pose::Pose pose{};
        // The weapon's caliber label, owned by the caller for the frame; null
        // or empty for none. Minimal's pose must be resolved labelled when
        // labels() and this is not empty.
        const char* caliber{ nullptr };
    };

    // Game thread. RPS UI draws overlay labels (NumberOverlay 2.1 or later).
    [[nodiscard]] bool labels() noexcept;

    // Game thread, once per presented frame. Indexed by hand: [0] right,
    // [1] left. hudColor: the game's HUD color, 0..1 per channel, used by the
    // HUD styles. True while RPS UI accepted every overlay update, so the
    // counters can replace the vanilla one.
    bool update(const std::array<HandCounter, 2>& hands, Style style, const std::array<float, 3>& hudColor) noexcept;

    // Game thread. Closes every open counter.
    void hide() noexcept;
}
