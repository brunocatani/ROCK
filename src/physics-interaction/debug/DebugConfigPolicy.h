#pragma once

namespace rock::debug_config_policy
{
    [[nodiscard]] constexpr bool subsystemEnabled(const bool globalEnabled, const bool requestedEnabled) noexcept
    {
        return globalEnabled && requestedEnabled;
    }

    [[nodiscard]] constexpr bool childEnabled(const bool parentEnabled, const bool requestedEnabled) noexcept
    {
        return parentEnabled && requestedEnabled;
    }
}
