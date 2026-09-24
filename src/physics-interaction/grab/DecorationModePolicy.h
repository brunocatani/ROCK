#pragma once

#include <cstdint>

namespace rock::decoration_mode
{
    inline constexpr int kButtonId = 32;

    // A two-handed hold is one reference. Two different objects are ambiguous.
    [[nodiscard]] constexpr std::uint32_t singleObject(std::uint32_t right, std::uint32_t left) noexcept
    {
        return right && left && right != left ? 0 : (right ? right : left);
    }

    struct ClickState
    {
        bool armed{};
        bool draining{};
        bool reserved{};
        std::uint32_t request{};

        void update(bool enabled, std::uint32_t candidate, bool available,
            bool held, bool pressed, std::uint32_t ageMilliseconds) noexcept
        {
            request = 0;
            reserved = false;
            if (!available || ageMilliseconds > 100 || (!enabled && !draining)) {
                armed = false;
                draining = false;
                return;
            }
            // Retain ownership through the release of an accepted click even
            // after the object is released or the setting is disabled.
            reserved = draining || (enabled && candidate != 0);
            if (!held) {
                if (armed && pressed && enabled && candidate && !draining) request = candidate;
                armed = true;
                draining = false;
                return;
            }
            if (armed && pressed && enabled && candidate && !draining) {
                request = candidate;
                draining = true;
            }
            armed = false;
        }
    };
}
