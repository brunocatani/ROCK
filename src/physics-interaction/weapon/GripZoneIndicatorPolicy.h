#pragma once

#include <cstdint>

namespace rock::grip_zone_indicator_policy
{
    inline constexpr float kDefaultDiameterGameUnits = 0.70f;
    inline constexpr float kMinimumDiameterGameUnits = 0.10f;
    inline constexpr float kMaximumDiameterGameUnits = 5.00f;

    [[nodiscard]] constexpr bool isCurrentRenderFrame(
        const std::uint64_t markerFrameIndex,
        const std::uint64_t latestPublishedFrameIndex) noexcept
    {
        return markerFrameIndex != 0 &&
               markerFrameIndex == latestPublishedFrameIndex;
    }
}
