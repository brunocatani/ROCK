#pragma once

#include <cstdint>

namespace rock::equipped_weapon_drop_policy
{
    enum class SourceHand : std::uint8_t
    {
        None = 0,
        Right,
        Left,
    };

    [[nodiscard]] inline constexpr bool isLeft(SourceHand hand) noexcept
    {
        return hand == SourceHand::Left;
    }

    [[nodiscard]] inline constexpr const char* sourceHandName(SourceHand hand) noexcept
    {
        switch (hand) {
        case SourceHand::Right:
            return "right";
        case SourceHand::Left:
            return "left";
        case SourceHand::None:
            break;
        }
        return "none";
    }

    [[nodiscard]] inline constexpr SourceHand sourceForSupportRelease(bool primaryReleasedThisFrame) noexcept
    {
        return primaryReleasedThisFrame ? SourceHand::Right : SourceHand::Left;
    }

    [[nodiscard]] inline constexpr bool shouldSurrenderReleaseToVirtualHolsters(SourceHand sourceHand, bool virtualHolstersOwnsSourceHand) noexcept
    {
        return sourceHand != SourceHand::None && virtualHolstersOwnsSourceHand;
    }

    /*
     * Equipped-weapon shoulder stash only tracks a single carrying hand: the
     * gesture is "carry the weapon over the shoulder and let go", which
     * requires that releasing this hand is what would drop the weapon. With
     * two active holds no single release drops, so no hand is a stash carry
     * candidate until one grip lets go.
     */
    [[nodiscard]] inline constexpr SourceHand resolveEquippedWeaponStashCarryHand(
        bool primaryOnlyActive,
        bool partCarryActive,
        bool leftPartGripActive,
        bool rightPartGripActive,
        bool firingHandIsLeft) noexcept
    {
        if (primaryOnlyActive) {
            return firingHandIsLeft ? SourceHand::Left : SourceHand::Right;
        }
        if (partCarryActive && leftPartGripActive != rightPartGripActive) {
            return leftPartGripActive ? SourceHand::Left : SourceHand::Right;
        }
        return SourceHand::None;
    }

}
