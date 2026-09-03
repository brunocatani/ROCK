#pragma once

#include <cstdint>

namespace rock::grip_zone_indicator_policy
{
    // Mode 2 is the explicit material-system fallback for shader-replacement
    // setups. Keep it opt-in until the normal NIF path is proven independent
    // of those replacements and this INI contract is deliberately retired.
    enum class Mode : int
    {
        Off = 0,
        Nif = 1,
        DebugOverlay = 2,
    };

    inline constexpr float kDefaultDebugDiameterGameUnits = 0.70f;
    inline constexpr float kMinimumDebugDiameterGameUnits = 0.10f;
    inline constexpr float kMaximumDebugDiameterGameUnits = 5.00f;

    [[nodiscard]] constexpr bool isSupportedMode(const long value) noexcept
    {
        return value >= static_cast<long>(Mode::Off) &&
               value <= static_cast<long>(Mode::DebugOverlay);
    }

    [[nodiscard]] constexpr Mode modeFromIni(const long value) noexcept
    {
        return isSupportedMode(value) ?
                   static_cast<Mode>(value) :
                   Mode::Nif;
    }

    [[nodiscard]] constexpr bool usesNif(const Mode mode) noexcept
    {
        return mode == Mode::Nif;
    }

    [[nodiscard]] constexpr bool usesDebugOverlay(const Mode mode) noexcept
    {
        return mode == Mode::DebugOverlay;
    }

    [[nodiscard]] constexpr bool isCurrentRenderFrame(
        const std::uint64_t markerFrameIndex,
        const std::uint64_t latestPublishedFrameIndex) noexcept
    {
        return markerFrameIndex != 0 &&
               markerFrameIndex == latestPublishedFrameIndex;
    }
}
