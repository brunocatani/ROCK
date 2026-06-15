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

    inline constexpr int kPostDropHandCollisionSuppressionFrames = 5;

    struct SuppressionFrameState
    {
        int framesRemaining{ 0 };

        [[nodiscard]] constexpr bool active() const noexcept { return framesRemaining > 0; }
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

    inline constexpr void beginPostDropSuppression(SuppressionFrameState& state) noexcept
    {
        state.framesRemaining = kPostDropHandCollisionSuppressionFrames;
    }

    inline constexpr void clearPostDropSuppression(SuppressionFrameState& state) noexcept
    {
        state.framesRemaining = 0;
    }

    [[nodiscard]] inline constexpr bool advancePostDropSuppression(SuppressionFrameState& state) noexcept
    {
        if (state.framesRemaining <= 0) {
            state.framesRemaining = 0;
            return false;
        }

        --state.framesRemaining;
        return state.framesRemaining == 0;
    }
}
