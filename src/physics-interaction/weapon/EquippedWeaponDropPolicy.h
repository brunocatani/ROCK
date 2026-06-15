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

}
